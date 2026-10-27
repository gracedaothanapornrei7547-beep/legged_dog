// contact_estimator.hpp —— 多腿接触迟滞滤波 (估计流水线第 2 步)
// =============================================================================
//
// 【模块定位】
//   contact_detector → ★contact_estimator → estimator → body_posture / WBC
//
//   contact_detector 给的 confidence ∈ [0,1] 是**瞬时的、抖**的：
//     - 接触瞬间 confidence 可能在 0.5~1.0 之间快速跳
//     - 离地瞬间又跳回 0~0.5
//     - 直接二值化会引起 swing/stance 误切换
//
//   contact_estimator 的工作：**双阈迟滞 + 时间窗** → 输出干净的 bool[4]
//
// 【迟滞算法（per leg）】
//
//   confidence ∈ [0, 1]                         (来自 contact_detector)
//        ↓
//   if confidence ≥ prob_on:      on_time  += dt;  off_time = 0
//   if confidence ≤ prob_off:     off_time += dt;  on_time  = 0
//   else (中间灰区):              on_time = off_time = 0   (清零)
//        ↓
//   state machine:
//     if (!contact && on_time ≥ on_delay)  → contact = true   (触地)
//     if ( contact && off_time ≥ off_delay) → contact = false  (离地)
//
//   双阈值（prob_on > prob_off）+ 时间累计 = 不会因瞬时抖动反复翻转
//
// 【典型参数】
//   prob_on  = 0.70  ← 高阈，进入接触前要求 confidence ≥ 0.7
//   prob_off = 0.30  ← 低阈，离开接触前要求 confidence ≤ 0.3
//   on_delay  = 10ms ← 触地切换前 confidence 稳定 >= prob_on 至少 10ms
//   off_delay = 35ms ← 离地切换前 confidence 稳定 <= prob_off 至少 35ms
//                     ↑ 离地时间窗比触地长：典型 trot 的 stance 末段可能短暂
//                       confidence 下降，但不应该误判为已离地
//
// 【输出去向】
//   is_contact[4] 喂给：
//     1. estimator: 哪些腿在地 → 用作 anchor 锚点修正 pos/vel
//     2. force_alloc/QP: 给真接触的腿分配 wrench（不浪费力在空中腿）
//     3. swing/stance kp 切换：override gait phase（处理早触地/晚抬起）
//
// 【与 chapt2_quadruped/Contact.py 对齐】
//   简化的第一版（chapt2 还有 Stage 2/3/4 的 torque/Jacobian/EKF 融合）
//   后续可扩展 update() 多传 score（torque-based, kinematic, etc.），状态机不变
// =============================================================================

#pragma once

#include <array>

#include "motor_test/common/pod_types.hpp"          // N_LEGS
#include "motor_test/estimation/contact_detector.hpp"  // ContactEstimate

namespace motor_test_cpp {
namespace contact {

// ============================================================================
// 接触估计参数
// ============================================================================

struct ContactEstimatorConfig {
    // ---- 双阈值（必须 prob_on > prob_off 形成迟滞带）----
    // confidence ≥ prob_on   → 强信号"在接触"
    // confidence ≤ prob_off  → 强信号"已离地"
    // prob_off < confidence < prob_on → 灰区，时间累计清零（不切换）
    double prob_on   = 0.70;
    double prob_off  = 0.30;

    // ---- 切换延迟 ----
    // on_delay 短 → 早切换 stance（响应快但易误触发）
    // off_delay 长 → 离地确认慢（防止 stance 末段短暂 confidence 下降被误判离开）
    double on_delay  = 0.005;    // 触地切换前 confidence 稳定 ≥ prob_on 至少 5ms
    double off_delay = 0.020;    // 离地切换前 confidence 稳定 ≤ prob_off 至少 20ms
};

// ============================================================================
// 单腿状态机（per-leg state）
// ============================================================================
//
// 每个 update(confidence, dt) 调用：
//   1. 根据 confidence 累计 on_time / off_time（在阈值中间则双归零）
//   2. 触地判断：state==false && on_time>=on_delay → state=true
//   3. 离地判断：state==true  && off_time>=off_delay → state=false
//   4. 返回当前 state

class LegContactState {
public:
    explicit LegContactState(const ContactEstimatorConfig& cfg = {}) : cfg_(cfg) {}

    // 重置（一般在腿状态初始化时调，true=初始假设在地）
    void reset(bool initial_contact = false) {
        contact_state_ = initial_contact;
        on_time_  = 0.0;
        off_time_ = 0.0;
    }

    bool update(double confidence, double dt) {
        if (confidence >= cfg_.prob_on) {
            on_time_  += dt; off_time_ = 0.0;
        } else if (confidence <= cfg_.prob_off) {
            off_time_ += dt; on_time_  = 0.0;
        } else {
            on_time_ = off_time_ = 0.0;
        }
        if (!contact_state_ && on_time_  >= cfg_.on_delay)  contact_state_ = true;
        if ( contact_state_ && off_time_ >= cfg_.off_delay) contact_state_ = false;
        return contact_state_;
    }

    bool state()         const { return contact_state_; }

private:
    ContactEstimatorConfig cfg_;
    bool   contact_state_ = false;
    double on_time_  = 0.0;
    double off_time_ = 0.0;
};

// ============================================================================
// 多腿接触估计器（per-quadruped）
// ============================================================================

class ContactEstimator {
public:
    explicit ContactEstimator(const ContactEstimatorConfig& cfg = {}) {
        for (auto& s : states_) s = LegContactState(cfg);
    }

    // 重置所有腿（cosine_lerp / 缓启动结束时调）
    void reset_all(bool initial_contact = false) {
        for (auto& s : states_) s.reset(initial_contact);
    }

    // 一拍批量更新：confidences[N_LEGS] 来自 ContactDetector::update()
    // 返回每腿是否接触（带迟滞防抖）
    std::array<bool, N_LEGS> update(
        const std::array<double, N_LEGS>& confidences, double dt) {
        std::array<bool, N_LEGS> out{};
        for (int i = 0; i < N_LEGS; ++i) {
            out[i] = states_[i].update(confidences[i], dt);
        }
        return out;
    }

private:
    std::array<LegContactState, N_LEGS> states_;
};

}  // namespace contact
}  // namespace motor_test_cpp
