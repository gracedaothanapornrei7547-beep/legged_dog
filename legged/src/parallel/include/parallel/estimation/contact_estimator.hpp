// contact_estimator.hpp —— 多腿接触迟滞滤波
// =============================================================================
// 所属模块: parallel::contact (estimation 流水线第 2 步)
// 职责: 对 contact_detector 输出的瞬时 confidence [0,1] 做双阈迟滞 + 时间窗滤波，
//       输出干净的 bool[4] 接触状态，防瞬时抖动引起 swing/stance 误切换。
//       下游喂给 estimator(锚点修正)、force_alloc(分配 wrench)、kp 切换。
// =============================================================================

#pragma once

#include <array>

#include "parallel/common/pod_types.hpp"          // N_LEGS
#include "parallel/estimation/contact_detector.hpp"  // ContactEstimate

namespace parallel {
namespace contact {

// ============================================================================
// 接触估计参数
// ============================================================================

struct ContactEstimatorConfig {
    // 双阈值（prob_on > prob_off 形成迟滞带）
    double prob_on   = 0.70;   // confidence ≥ prob_on → 强信号"在接触"
    double prob_off  = 0.30;   // confidence ≤ prob_off → 强信号"已离地"；灰区 = 时间清零不切换

    // 切换延迟
    double on_delay  = 0.005;  // 触地确认时间(s): confidence ≥ prob_on 需持续此时长才切换
    double off_delay = 0.020;  // 离地确认时间(s): 比 on_delay 长，防 stance 末段短暂 confidence 下降误判
};

// ============================================================================
// 单腿状态机
// ============================================================================
//
// 每拍 update(confidence, dt): 根据 confidence 累计 on_time/off_time（灰区双归零），
// 触地判断: state==false && on_time>=on_delay → state=true
// 离地判断: state==true  && off_time>=off_delay → state=false

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
}  // namespace parallel
