// contact_detector.hpp —— 单腿触地检测器（力矩残差 + 高度误差）
// =============================================================================
// 所属模块: parallel::contact (estimation 流水线第 1 步)
// 职责: 每拍根据 ContactInput 输出 ContactEstimate（confidence / touchdown / armed），
//       内部维护跨周期状态（空载力矩基线、arm 标志、连续 z 超标计数）。
//       与 parallel/control/contact.py 1:1 对齐，只做 shadow 诊断，不参与控制。
// =============================================================================

#pragma once

namespace parallel {
namespace contact {

// ============================================================================
// 配置（与 Python ContactConfig 同值）
// ============================================================================

struct ContactConfig {
    // 力矩检测
    double tau_threshold_nm = 1.0;       // 力矩残差触地阈值(Nm)，基于拍打测试和真机触地残差分布选定
    double unloaded_tau_nm  = 0.4;       // 空载力矩基线偏移(Nm)，扣除摩擦/装配误差(~0.5Nm)使残差分布对称
    // 高度辅助
    double z_error_threshold_m = 0.005;  // z 误差触地阈值(m)，过小误判、过大失效
    double lift_arm_threshold_m = 0.003; // 抬腿 arm 阈值(m)，过小过早 arm、过大错过触地
    // 时序
    int    arm_hold_frames  = 10;        // arm 状态保持帧数，防刚触地未完全承重时 z 回落导致过早退出
    int    z_contact_frames = 3;         // 连续 z 超标帧数阈值，防单帧波动误判
    // 平滑
    double tau_baseline_alpha = 0.1;     // 力矩基线 EMA 系数 [0,1]，平衡响应速度与噪声抑制
    // 下垂感知: stance 腿被分配下蹬力时若悬空会下沉，z_error < -sag_air_m 判为物理悬空。
    // 双支撑半 share(~12N)沉 ~1.2cm 压线，取 0.012。一触地立刻停沉，同帧恢复。
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
    // 上一拍命令的前馈力矩。残差 τ_ext = τ_meas - τ_ff，扣除自身注入力后判外部接触。
    // 不传默认 0 → 退化到旧行为（body_posture ON 时校不准）。
    double tau1_ff       = 0.0;
    double tau2_ff       = 0.0;
};

struct ContactEstimate {
    double confidence   = 0.0;
    bool   touchdown    = false;
    bool   armed        = false;
    double tau_residual = 0.0;
    double z_error      = 0.0;
    bool   sag_air      = false;   // nominal-stance 腿被判悬空标志(z_error < -sag_air_m)，仅诊断不进控制
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
}  // namespace parallel
