// raibert.hpp —— Raibert 落脚点规划
// =============================================================================
// 所属模块: parallel::raibert (planning 流水线)
// 职责: 基于 Raibert 原理计算足端 touchdown 偏移（body 系），注入到 gait::trot_body_target
//       的 swing 终点 + stance 起点，同步 shift 保持 stroke 长度不变。
//       当前仅保留 capture-point(roll) 项，历史的 neutral-point/速度反馈/pitch 项已删。
// =============================================================================

#pragma once

#include <Eigen/Core>
#include <array>

#include "parallel/common/pod_types.hpp"   // N_LEGS

namespace parallel {
namespace raibert {

struct RaibertConfig {
    // capture-point 落脚: 机身侧晃→摆脚往晃侧多落"接住"。其余 neutral-point/速度反馈/pitch 项已删。
    bool   enabled = true;   // enabled=false 时下面 capture_k_roll 不生效
    double capture_k_roll = -0.03;   // 负号正确, -0.03 将 roll 从 1.30° 降至 0.99°。仅 enabled=true 时生效
};

struct RaibertOffset {
    double x = 0.0;
    double y = 0.0;
};

// 单腿计算 touchdown 偏移(body 系): capture-point(roll) = 机身侧晃 → 摆动脚往晃侧偏移去"接住"。
inline RaibertOffset compute_touchdown(const RaibertConfig& cfg, double roll_rate) {
    RaibertOffset out;
    if (!cfg.enabled) return out;
    out.y = -cfg.capture_k_roll * roll_rate;   // 脚往晃的方向落
    return out;
}
// ============================================================================
// TouchdownLatch: 每腿独立的落点锁存器。
// 检测 stance→swing 抬脚上升沿那一刻锁存当前实时偏移，整个 swing+stance 用同一偏移，
// 避免实时 capture 项(∝ roll_rate)每帧抖动导致足端追抖目标。每步只在抬脚瞬间采样一次。
// ============================================================================
class TouchdownLatch {
public:
    // leg      : 腿索引
    // in_swing : 该腿本帧是否处于摆动（gait nominal swing）
    // fresh    : 本帧实时算出的落点偏移（raibert + capture）
    // 返回该腿应使用的【锁存】偏移。
    RaibertOffset update(int leg, bool in_swing, const RaibertOffset& fresh) {
        if (in_swing && !prev_swing_[leg]) latched_[leg] = fresh;  // 抬脚上升沿 → 锁存一次
        prev_swing_[leg] = in_swing;
        return latched_[leg];
    }
    void reset() { latched_.fill(RaibertOffset{}); prev_swing_.fill(false); }

private:
    std::array<RaibertOffset, N_LEGS> latched_{};
    std::array<bool, N_LEGS>          prev_swing_{};
};

}  // namespace raibert
}  // namespace parallel
