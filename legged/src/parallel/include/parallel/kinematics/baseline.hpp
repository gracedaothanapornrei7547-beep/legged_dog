// =============================================================================
// 文件: baseline.hpp
// 所属模块: parallel/kinematics
// 职责: 步态足端目标(body系)到电机命令(raw关节角)的运动学控制链——纯几何变换+IK+安全检查，不含力闭环。
//       同时定义电机固件PD参数(BaselineConfig)与控制链诊断结构(ControlChainDebug)。
// =============================================================================
//
// 控制链 8 步（per leg, per tick），任一步失败则沿用 last_raw：
//   1. body_to_local       body 系 → 单腿 local 系（sign 翻转）
//   2. clamp_workspace     限制到工作空间（r ∈ [0.10, 0.33]m, |y| ≤ 0.20）
//   3. inverse_kinematics  local (x,y,z) → 关节 (th3, th1, th2)
//   4. joints_in_range     关节软限位检查
//   5. fk_err 回算          FK(关节) - target，残差 ≥ 1cm 视为 IK 错误
//   6. clamp_joints        卡到硬软限位
//   7. math_to_raw         math 关节 → 电机 raw（dir 翻号 + deadzone）
//   8. jump_check          帧间 raw 跳变 > 0.3 rad 则拒绝
//
// 电机固件控制律: τ = kp·pos_err + kd·vel_err + tau_ff
//   - kp=0 时纯力控（对齐 qiayuanl），位置跟踪交 Cartesian 阻抗
//   - kp_stance 实测下限 60：kp≤20 撑不住机身（缺完整 M·ü+h 补偿）

#pragma once

#include <Eigen/Core>

#include "parallel/common/safety.hpp"
#include "parallel/kinematics/leg_calibration.hpp"
#include "parallel/kinematics/leg_kinematics.hpp"

namespace parallel {
namespace baseline {

// ============================================================================
// 电机层 PD 参数 + main 共用开关
// ============================================================================
// kp/kd 是电机固件关节 PD（非笛卡尔阻抗）。纯 VMC 力控架构：kp=0 使 τ 完全由 J^T·F 主导。
// kd 不对称：swing 低（不承重），stance 留高阻尼抗快扰动（固件内零延迟）。
struct BaselineConfig {
    // ── 电机固件 PD (swing/stance 两套，跳跃期单独覆盖) ──
    // kp=0: 纯力控路线，位置跟踪交 Cartesian 阻抗；优点是无 IK 翻肘电流尖峰，代价是无关节位置兜底。
    double kp_swing  = 0.0;     // (Nm/rad)
    double kd_swing  = 0.4;     // (Nm·s/rad)
    double kp_stance = 5.0;     // (Nm/rad)
    double kd_stance = 0.75;    // (Nm·s/rad)
    // 跳跃期（THRUST/FLIGHT）: kp_jump=0 纯力控, kd_jump=0 少刹蹬伸。LAND/SETTLE 回落 stance。
    double kp_jump   = 0.0;     // (Nm/rad)
    double kd_jump   = 0.0;     // (Nm·s/rad)

    // 重力前馈总开关。机身质量唯一真源在 body_posture::mass_kg（默认 13kg）。
    bool   ff_grav   = true;
    // F_body 一阶 LPF α: 平滑 swing↔stance 切换力跳变。0.20 (τ≈6ms@500Hz)。
    double f_lpf_alpha = 0.20;
    // tau_max 已迁到 force_control::ForceCtrlConfig
};

// ============================================================================
// 控制链诊断（可选）：fail_step 0=成功, 1=IK无解, 2=关节越限, 3=FK残差>1cm, 4=跳变>0.3rad
// ============================================================================
struct ControlChainDebug {
    int    fail_step = 0;
    double th3 = 0, th1 = 0, th2 = 0;
    double fk_err = 0, jump = 0;
    Eigen::Vector3d local_tgt{0, 0, 0};
};

// ============================================================================
// body_tgt → raw_cmd + math_des。成功返回 true（更新 last_raw），失败沿用 last_raw。
// ============================================================================
bool control_chain(const robot::LegCalibration& cal,
                   const kinematics::LegKinematics& kin,
                   const robot::JointLimits& limits,
                   const Eigen::Vector3d& body_tgt,
                   Eigen::Vector3d& last_raw,    // [in/out] IK fallback
                   Eigen::Vector3d& raw_out,     // [out] 发给电机的 raw
                   Eigen::Vector3d& math_out,    // [out] clamp 后的 math
                   ControlChainDebug* dbg = nullptr);  // [out,可选] 诊断

}  // namespace baseline
}  // namespace parallel
