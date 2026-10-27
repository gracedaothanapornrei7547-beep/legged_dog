// =============================================================================
// 文件: force_control.hpp
// 模块: parallel/dynamics
// 职责: 单腿足端力→关节扭矩投影——虚功原理+笛卡尔阻抗（WBC第3步，力的最终出口）
//
// 控制链: force_alloc → F_body[i] → force_control::compute_leg_tau → τ → 电机
// 每帧每腿独立调用，组合机身目标力与笛卡尔跟踪阻抗力后投影到关节扭矩。
//
// 数学 (per leg, per tick):
//   F_imp = Kp·(p_des-p_act) + Kd·(v_des-v_act)    // Kp/Kd按support_weight线性插值
//   F_local = F_imp_local + F_body_local
//   τ_math = Jᵀ(math_act)·F_local                   // 用实测关节角，非期望值
//   τ_motor = cal.math_torque_to_motor(τ_math), clamped to ±tau_max
//
// 关键调参:
//   - kp_swing≈600: swing期跟踪硬度, 过低落地点偏, 过高落地冲击大
//   - kp_stance_z≈800: 硬腿软化, 从900退刚化至目标heave σ<12mm; kd_stance_z若震可退至30
//   - WBC-lite blend=0.5甜点: 矢状面Λ整形抵消5-bar各向异性(0=关→摔, 1=全整形→roll炸)
//   - tau_max≈1000: 安全网, 正常force_alloc限幅下不触发
//   - imp_enabled=false: F_imp=0, 纯Jᵀ·F_body重力前馈模式
// =============================================================================

#pragma once

#include <Eigen/Core>

#include "parallel/kinematics/leg_calibration.hpp"
#include "parallel/kinematics/leg_kinematics.hpp"

namespace parallel {
namespace force_control {

// ============================================================================
// 力控参数 —— 笛卡尔阻抗 + WBC-lite 操作空间整形
// ============================================================================
// F_imp = kp·pos_err + kd·vel_err, τ_ff = Jᵀ(q_act)·(F_imp+F_body)
//
// 调参约束:
//   - 电机13A/13Nm CAN墙: τ_ff峰值受它封顶, 增益不随意提
//   - 编码器量化噪声~0.1m/s → kd不能太大, 否则高频力抖
//   - 纯VMC(电机kp=0): 所有跟踪靠Cartesian阻抗, kp_swing是唯一旋钮
//   - 摆动欠跟踪根因: 阻抗带宽+5-bar各向异性, 非kp绝对值不足
// ============================================================================

struct ForceCtrlConfig {
    // 笛卡尔阻抗总开关；off → F_imp=0，只剩 J^T·F_body（"纯重力前馈"）
    bool   imp_enabled  = true;

    /* ---- swing 期增益（三轴同值）---- */
    //   kp_swing: 跟踪 swing 抛物线轨迹的硬度。过低→落地点偏/失稳; 过高→落地冲击大/CAN 墙。
    double kp_swing     = 600.0;  // (N/m) swing跟踪硬度; 开源2000但腿更长, 1200震动已回稳
    double kd_swing     = 40.0;   // (N·s/m) 阻尼抑制高频振

    /* ---- stance 期增益 ---- */
    //   kp_stance: stance 期允许脚轻微滑动的硬度。太高→姿态环自激; 太低→力分配被抵消。
    double kp_stance_x  = 700.0;  // (N/m)
    double kd_stance_x  = 35.0;   // (N·s/m)
    double kp_stance_y  = 600.0;  // (N/m)
    double kd_stance_y  = 35.0;
    //   kp_stance_z: z方向承重弹簧。太硬→自激; 太软→下沉。
    //   硬腿软化: 从900退至800, 目标heave σ<12mm; kd_stance_z若震可退至30
    double kp_stance_z  = 800.0;  // (N/m) 硬腿软化, 目标heave σ<12mm
    double kd_stance_z  = 40.0;   // (N·s/m)

    // 单电机 τ 钳制 (Nm): 安全网, 正常 force_alloc 已限幅不触发。
    //   触发 = force_alloc 异常 / Jacobian 病态 / 力分配 bug。
    double tau_max      = 1000.0;

    // 惯性前馈已证伪删除: 摆动顶点a_des向下→F_inertia拽脚下→抬更低

    // ---- Cartesian 积分 Ki (对标开源 VMC_Quadruped) ----
    //   消除稳态跟踪误差, 让脚最终落到目标位置。swing期=0(摆动误差是设计内).
    double ki_swing     = 0.0;     // swing 积分0(不需要)
    double ki_stance_x  = 0.0;    // 关
    double ki_stance_y  = 0.0;    // 关
    double ki_stance_z  = 0.0;    // 关
    double ki_windup_n  = 15.0;    // 积分力饱和上限(N), 防 windup (30→15 减左右积分不对称)

    // ============================================================================
    // WBC-lite: 操作空间摆动整形 —— 抵消5-bar各向异性(领腿抬不起的根因)
    // ============================================================================
    //   ẍ=Λ⁻¹·F, Λ各向异性→标量kp下难动方向欠跟踪。解: F_imp=Λ_shaped·(kp·e+kd·ė),
    //   Λ用det^(1/3)归一化(pure shape)→ẍ≈kp·e各向同性。替代lead_lift_scale补丁的根治方案。
    //
    //   仅swing期整形(support_weight<wbc_swing_gate), stance保持标量阻抗。
    //   仅矢状面(x,z↔th1,th2)整形: 全3×3会放大th3致roll失稳。
    //   混合系数α∈[0,1]: Λ_eff=(1-α)·I+α·Λ_sag。甜点blend=0.5(0=关→摔, 1=全整形→roll炸, 0.5稳定)
    bool   wbc_lite       = true;   // ON: 矢状面Λ整形抑制roll恶化
    double wbc_swing_gate = 0.5;    // support_weight<此→swing整形; ≥→原阻抗
    double wbc_damp       = 1e-3;   // J_sag奇异阻尼ε(×trace), 防工作空间边缘求逆爆
    double wbc_blend      = 0.5;    // Λ整形混合α (0.5=甜点; 1=全整形; 0=关=纯标量阻抗)
};

// ============================================================================
// 单腿 τ 计算（per-leg, per-tick）
//
// 输入（全部 local 系，math_act 是 raw_to_math 后的实测关节角）：
//   math_act      : (th3, th1, th2) 实测
//   p_des / v_des : 足端期望位置/速度
//   p_act / v_act : 足端实测位置/速度（通常由 FK / J·dq 算出）
//   F_body        : body 系机身受力（来自 force_alloc::allocate_*）
//   is_stance     : 笛卡尔阻抗 swing/stance 系数切换
//
// 输出：(τ3, τ1, τ2)_motor，已 ±tau_max 钳制。
// ============================================================================
// support_weight ∈ [0,1]：在 Kp_swing (=0) 和 Kp_stance (=1) 之间线性插值
//   Kp_eff = (1 - w) · Kp_swing + w · Kp_stance
// 比 bool is_stance 更平滑，避免相位切换瞬间 Kp 阶跃。
Eigen::Vector3d compute_leg_tau(const ForceCtrlConfig& cfg,
                                 const robot::LegCalibration& cal,
                                 const kinematics::LegKinematics& kin,
                                 const Eigen::Vector3d& math_act,
                                 const Eigen::Vector3d& p_des_local,
                                 const Eigen::Vector3d& v_des_local,
                                 const Eigen::Vector3d& p_act_local,
                                 const Eigen::Vector3d& v_act_local,
                                 const Eigen::Vector3d& F_body,
                                 double support_weight,
                                 Eigen::Vector3d& F_int_local);   // [in/out] Cartesian积分(anti-windup)

}  // namespace force_control
}  // namespace parallel
