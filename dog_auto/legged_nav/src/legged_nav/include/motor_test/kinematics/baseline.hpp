/**
 * @file baseline.hpp
 * @brief 运动学控制链 (Kinematics Control Chain) 与执行器固件参数配置
 * 
 * 模块定位: Layer 2 运动学映射层 (motor_test_cpp::baseline)
 * 职责描述:
 *   作为步态规划层至执行器物理驱动层之间的运动学映射管道，
 *   完成足端坐标系变换、可达工作空间凸包裁剪、逆运动学求解、
 *   解析正解反算复核 (FK Residual Check)、关节软硬限位判定与角速度阶跃滤波 (Jump Check)。
 */

#pragma once

#include <Eigen/Core>

#include "motor_test/common/safety.hpp"
#include "motor_test/kinematics/leg_calibration.hpp"
#include "motor_test/kinematics/leg_kinematics.hpp"

namespace motor_test_cpp {
namespace baseline {

// ============================================================================
// 电机层 PD 参数 + 一些 main 共用的开关
// ============================================================================
//
// 注意：kp_*/kd_* 这里是 **电机 firmware 关节 PD**，不是笛卡尔阻抗。
//   - 与 qiayuanl/legged_control 对齐时建议：kp_stance ≈ 0, kd_stance ≈ 3
//     让 τ 真正由 force_control 的 J^T·F 主导（电机只剩小阻尼）。
//
struct BaselineConfig {
    // ── 电机 firmware PD (按 swing/stance 切换) ──
    //   τ_actual = kp·pos_err + kd·vel_err + τ_ff
    //   纯 VMC 力控架构(对齐 qiayuanl/kp=0): kp=0→位置项零, 电机只跟 J^T·F + 阻尼。
    //   位置跟踪交 force_control 笛卡尔阻抗(Cartesian N/m, 非关节 Nm/rad)。
    //
    //   好处: ①无 IK 翻肘→电流尖峰→CAN 降级链路 ②IK fail 时不会乱发位置 ③swing/stance 平滑。
    //   代价: ①无关节位置兜底(VMC 失效→腿没扶) ②启动期依赖 Cartesian 阻抗拉到位。
    //   安全: safe-hold 模式走 kp=5/kd=1.5 紧急位置锁, bus 异常时腿不乱飞。
    //
    //   kd 不对称设置: swing 低(不承重无快扰动, 低 kd 换跟踪率), stance 高(firmware 内 0 延迟,
    //   Cartesian Kd 走 USB-CAN loop 有延迟→快扰动来不及→stance 留高 motor kd 保稳)。
    double kp_swing  = 0.0;     // (Nm/rad)  纯力控—位置交 Cartesian 阻抗
    double kd_swing  = 0.4;     // (Nm·s/rad) swing 低阻尼换跟踪率
    double kp_stance = 5.0;    // (Nm/rad)  ★当前 0(候选未采用: 0→10 微位置兜底=0.1Nm@1°误差, 不打架 Cart imp)
    double kd_stance = 0.75;     // (Nm·s/rad) ★当前 1(候选未采用: 1→3 firmware 内零延迟阻尼杀高频振)

    // ★机身质量(m·g 重力前馈)唯一真源 = body_posture.hpp 的 mass_kg(默认13kg)。
    //   此处不再留副本——banner 打印与 --no-body-posture 兜底都读 a.body_posture.mass_kg。删字段防两值漂移。
    // 重力前馈总开关（影响 body_posture 的 ramp + fz 输出）
    // 默认 ON：关掉重力前馈机身会塌，body_posture/force_alloc 力前馈依赖它。要关用 --no-ff-grav。

    bool   ff_grav   = true;
    // F_body 一阶 LPF α(0..1): 平滑 swing↔stance 切换力跳变。
    //   α 小→滞后大→腿承重爬得慢→机身颠; α 大→平滑差→跨相位力跳变。
    //   0.20 (τ≈6ms@500Hz): swing 开始及时归零 fz 同时略平滑。
    double f_lpf_alpha = 0.20;
    // 注意：tau_max 已迁到 force_control::ForceCtrlConfig
};

// ============================================================================
// 控制链诊断（可选）：填到哪步失败 + 关键数值，供 main 聚合统计 ik_fail 拆解。
//   fail_step: 0=成功 1=IK无解(步3) 2=关节越限(步4) 3=FK回算>1cm(步5) 4=jump>0.3(步8)
//   th3/th1/th2: 本帧 IK 解出的关节角(rad)——看哪个关节越限
//   fk_err / jump: 步5 残差 / 步8 最大跳变
//   local_tgt: clamp 后的 local 目标(诊断 body_to_local 后落点)
// ============================================================================
struct ControlChainDebug {
    int    fail_step = 0;
    double th3 = 0, th1 = 0, th2 = 0;
    double fk_err = 0, jump = 0;
    Eigen::Vector3d local_tgt{0, 0, 0};
};

// ============================================================================
// 控制链：body_tgt → raw_cmd（同时输出 math_des 给 force_control 的 FK/J 用）
//
// 返回 true 表示 IK 成功（last_raw 已更新）；
// false 表示沿用 last_raw（IK 无解 / 超限 / fk_err 过大 / 跳变）。
// dbg 非空 → 填充诊断（哪步失败 + 关节角/误差），不影响控制逻辑。
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
}  // namespace motor_test_cpp
