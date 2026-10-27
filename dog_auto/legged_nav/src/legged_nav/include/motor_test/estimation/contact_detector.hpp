// contact_detector.hpp —— shadow 触地估计器（与 motor_test/control/contact.py 1:1 对齐）。
//
// 输入 ContactInput (per-tick) → 输出 ContactEstimate。
// 内部维护跨周期状态：空载力矩基线、arm 标志、连续 z 超标计数。
//
// Phase 1 策略（最小可行）：力矩残差 + 高度误差，只做 shadow 诊断，不参与控制。

#pragma once

namespace motor_test_cpp {
namespace contact {

// ============================================================================
// 配置（与 Python ContactConfig 同值）
// ============================================================================

struct ContactConfig {
    // 力矩检测
    double tau_threshold_nm = 1.0;       // 这个阈值对拍时的残差分布（见 contact_test）以及实际触地时的残差分布（见 real_robot_test）来选：
    double unloaded_tau_nm  = 0.4;       // 触地时的残差分布（real_robot_test）显示有个 ~0.5Nm 的偏移，可能是摩擦或装配误差引起的。这个参数用来在检测时扣除这个偏移，让残差分布更对称，提升检测性能。
    // 高度辅助
    double z_error_threshold_m = 0.005;  // 这个阈值对拍时的 z 误差分布（见 contact_test）以及实际触地时的 z 误差分布（见 real_robot_test）来选：过小会导致误判，过大会失去辅助作用。
    double lift_arm_threshold_m = 0.003; // 这个阈值对拍时的 z 误差分布（见 contact_test）以及实际触地时的 z 误差分布（见 real_robot_test）来选：过小会导致过早进入 arm 状态，过大可能会错过触地检测的最佳时机。
    // 时序 
    int    arm_hold_frames  = 10;        // 进入 arm 状态后保持多少帧（即使 z 回落）才允许重新检测触地。这个参数用来防止在 leg 刚触地但还没完全承重时因为 z 误差回落而过早退出 arm 状态，导致错过后续的力矩残差检测。
    int    z_contact_frames = 3;         // 需要连续多少帧 z 误差超过 z_error_threshold_m 才认为是触地。这个参数用来防止因为单帧的 z 误差波动而误判触地。
    // 平滑
    double tau_baseline_alpha = 0.1;     // 用于力矩基线的指数移动平均滤波，范围 [0,1]：越大响应越快但越受噪声影响，越小越平滑但响应更慢。这个参数用来在 leg 刚触地时快速适应新的力矩基线，同时在 leg 已经承重时保持稳定的基线估计。
    // 下垂感知(修"stance 地板=步态回声"): 纯 VMC stance 腿被分配下蹬力 F, 脚【悬空】
    //   → F 把脚压到目标下方 F/kp_stance_z(满 share 25N/1000≈2.5cm) 弹簧平衡; 脚【在地上】→ 被
    //   地面顶住停在目标(z_error≈0/偏上)。z_error < -sag_air_m = 脚在沉 = 物理悬空电平信号。
    //   双支撑半 share(~12N)沉 1.2cm 刚好压线 → 取 0.012。一触地立刻停沉, 同帧恢复。
    double sag_air_m = 0.012;
};

// ============================================================================
// 输入/输出
// ============================================================================

struct ContactInput {
    bool   nominal_swing = false;
    double act_z_body    = 0.0;
    double des_z_body    = 0.0;
    double des_vz_body   = 0.0;
    // 实测 motor τ（含 body_posture/force_control 注入的 ff + 地面反作用 + friction）
    double tau1_meas     = 0.0;
    double tau2_meas     = 0.0;
    // 我们上一拍 commanded 的 ff τ。
    // 残差判触：τ_ext = τ_meas - τ_ff，扣除自己注入的力，剩下的才是"外部扰动"（接触 / friction）。
    // 不传则默认 0 → 退化到旧行为（baseline 包含 ff，body_posture ON 时校不准）。
    double tau1_ff       = 0.0;
    double tau2_ff       = 0.0;
};

struct ContactEstimate {
    double confidence   = 0.0;
    bool   touchdown    = false;
    bool   armed        = false;
    double tau_residual = 0.0;
    double z_error      = 0.0;
    bool   sag_air      = false;   // ★观测用: nominal-stance 腿被判悬空(z_error<-sag_air_m→conf封顶0.45)。仅 sag 地板诊断, 不进控制。
};

// ============================================================================
// 检测器（有状态）
// ============================================================================

class ContactDetector {
public:
    explicit ContactDetector(const ContactConfig& cfg = {}) : cfg_(cfg) {}

    void reset_for_stance(double act_z_body) {
        z_contact_count_  = 0;
        armed_            = false;
        swing_hold_count_ = 0;
        swing_entry_z_    = act_z_body;
        was_nominal_swing_ = false;
    }

    ContactEstimate update(const ContactInput& inp);

    // 调试/测试：读内部状态
    const ContactConfig& cfg() const { return cfg_; }
    double tau_baseline_1() const { return tau_baseline_1_; }
    double tau_baseline_2() const { return tau_baseline_2_; }
    bool   armed() const { return armed_; }
    int    z_contact_count() const { return z_contact_count_; }

private:
    void enter_swing(double act_z_body);
    void update_arm(const ContactInput& inp, double tau_residual);

    ContactConfig cfg_;

    int    z_contact_count_  = 0;
    double tau_baseline_1_   = 0.0;
    double tau_baseline_2_   = 0.0;
    bool   armed_            = false;
    int    swing_hold_count_ = 0;
    double swing_entry_z_    = 0.0;
    bool   was_nominal_swing_ = false;
};

}  // namespace contact
}  // namespace motor_test_cpp
