// raibert.hpp —— Raibert 落脚点规划（参考 chapt2_quadruped/Vmc.py:raibert_target_body）
//
// 原理（Marc Raibert 1986）：
//   trot 时，足端落地位置 (body 系) 应该满足：
//     touchdown_x = 0.5 · vbx · t_stance + Kp · (vbx - vcmd_x)
//                   └─ neutral point ─┘   └─ feedback term ─┘
//
//   neutral point：让脚落在"机身正下方"（stance 中点处机身刚好在脚上方）
//   feedback：vel 超过 cmd 时把脚再向前伸 → 接触瞬间足底反作用力把机身刹回来
//             vel 慢于 cmd 时把脚向后收 → 反扫给机身向前推力
//
// 实际落地位置：原 trot stroke (+step_x/2, +step_y/2) + raibert 偏移
//   touchdown_body_x = step_x/2  + raibert.x
//   touchdown_body_y = leg_y_offset + step_y/2 + raibert.y
//
// 接口设计：单独输出 (raibert.x, raibert.y)，让 gait::trot_body_target 把偏移
// 注入到 swing 终点 + stance 起点（同步 shift，保持 stroke 长度不变）。

#pragma once

#include <Eigen/Core>
#include <array>

#include "motor_test/common/pod_types.hpp"   // N_LEGS

namespace motor_test_cpp {
namespace raibert {

struct RaibertConfig {
    // capture-point落脚: 机身侧晃→摆脚往晃侧多落"接住". 其余neutral-point/速度反馈/capture_pitch已删(吃幻象est_vx).
    bool   enabled = true;   // ★capture 当前关(走路轮判 cycle_s=0.40 已顶住 roll); enabled=false → 下面 capture_k_roll 不生效
    double capture_k_roll = -0.03;   // 负号实测正确, -0.03=V形谷底(memory: roll 1.30→0.99°)。★仅 enabled=true 时生效, 现为待用值(崎岖/扰动场景重开)
};

struct RaibertOffset {
    double x = 0.0;
    double y = 0.0;
};

// 单腿计算 touchdown 偏移(body 系)。当前只剩 capture-point(roll) 一项:
//   roll_rate = 机身角速度 ω_x (body 系, rad/s)。机身侧晃 → 摆动脚往晃侧偏移去"接住"。
//   (历史的 neutral-point / 速度反馈 / capture_pitch 已删, 见 RaibertConfig 注释。)
inline RaibertOffset compute_touchdown(const RaibertConfig& cfg, double roll_rate) {
    RaibertOffset out;
    if (!cfg.enabled) return out;
    out.y = -cfg.capture_k_roll * roll_rate;   // 脚往晃的方向落(符号对齐 chapt2; A/B 见 cfg 注释)
    return out;
}
// ============================================================================
// 【为什么要它】
//   本项目 gait 每帧重算 body_tgt。若把 compute_touchdown 的实时偏移(尤其 capture
//   项 ∝ roll_rate，trot 时 ±1.3rad/s 剧烈摆)每帧灌进落点 → 落点每帧跳 ±cm → 摆动脚
//   追抖动目标 → 足端抖乱。chapt2 在 liftoff 把落点算定一次、swing 插值到固定点，故不抖。
//
// 【做法】
//   每条腿独立。检测 stance→swing 抬脚上升沿，那一刻锁存当前实时偏移；其余帧返回锁存值。
//   → capture·roll_rate 每步只采样一次（抬脚瞬间），整个 swing+stance 用同一偏移，平滑。
//   也是后续"sticky world anchor / 推进"的地基。
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
}  // namespace motor_test_cpp
