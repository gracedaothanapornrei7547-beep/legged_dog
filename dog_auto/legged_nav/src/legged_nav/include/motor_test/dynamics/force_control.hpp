/**
 * @file force_control.hpp
 * @brief 笛卡尔空间虚拟模型力控制 (VMC) 与雅可比转置映射接口
 * 
 * 模块定位: Layer 3 动力学控制层 (motor_test_cpp::force_control)
 * 职责描述:
 *   基于虚拟模型控制 (Virtual Model Control) 与虚功原理，
 *   结合笛卡尔空间阻抗控制律与矢状面操作空间惯量解耦 (WBC-lite OSIC)，
 *   将机身分配的期望接触力与足端轨迹跟踪力合成并映射至驱动关节力矩。
 */

#pragma once

#include <Eigen/Core>

#include "motor_test/kinematics/leg_calibration.hpp"
#include "motor_test/kinematics/leg_kinematics.hpp"

namespace motor_test_cpp {
namespace force_control {

/**
 * @brief 笛卡尔空间阻抗与力控制参数配置
 */
struct ForceCtrlConfig {
    bool   imp_enabled  = true;     ///< 笛卡尔阻抗控制总使能

    // ---- 摆动相各向同性空间阻抗增益 ----
    double kp_swing     = 600.0;    ///< 摆动相位置跟踪刚度 (N/m)
    double kd_swing     = 40.0;     ///< 摆动相速度阻尼 (N*s/m)

    // ---- 支撑相各向异性空间阻抗增益 ----
    double kp_stance_x  = 700.0;    ///< 支撑相 X 轴刚度 (N/m)
    double kd_stance_x  = 35.0;     ///< 支撑相 X 轴阻尼 (N*s/m)
    double kp_stance_y  = 600.0;    ///< 支撑相 Y 轴刚度 (N/m)
    double kd_stance_y  = 35.0;     ///< 支撑相 Y 轴阻尼 (N*s/m)
    double kp_stance_z  = 800.0;    ///< 支撑相 Z 轴垂直承重刚度 (N/m)
    double kd_stance_z  = 40.0;     ///< 支撑相 Z 轴垂直速度阻尼 (N*s/m)

    // ---- 执行器峰值力矩限幅 ----
    double tau_max      = 1000.0;   ///< 执行器输出力矩安全硬限幅 (Nm)

    // ---- 笛卡尔误差积分项 (Anti-windup) ----
    double ki_swing     = 0.0;
    double ki_stance_x  = 0.0;
    double ki_stance_y  = 0.0;
    double ki_stance_z  = 0.0;
    double ki_windup_n  = 15.0;     ///< 积分力饱和上限 (N)

    // ---- 矢状面操作空间惯量整形 (WBC-lite OSIC) ----
    bool   wbc_lite       = true;   ///< 矢状面操作空间惯量解耦使能
    double wbc_swing_gate = 0.5;    ///< 激活整形的最大支撑权重阈值
    double wbc_damp       = 1e-3;   ///< 奇异阻尼因子
    double wbc_blend      = 0.5;    ///< 标量阻抗与惯量整形混合权重 alpha
};

/**
 * @brief 单腿笛卡尔空间阻抗控制与雅可比力矩映射计算
 * @param cfg 力控参数配置
 * @param cal 腿部结构与电机极性标定
 * @param kin 五连杆运动学几何解算器
 * @param math_act 当前实测物理关节角 (rad)
 * @param p_des_local 期望足端局部坐标 (m)
 * @param v_des_local 期望足端局部运动速度 (m/s)
 * @param p_act_local 实际足端局部坐标 (m)
 * @param v_act_local 实际足端局部运动速度 (m/s)
 * @param F_body 机身坐标系下分配给该腿的目标反作用力 (N)
 * @param support_weight 当前腿支撑连续权重因子 [0.0, 1.0]
 * @param F_int_local 笛卡尔空间积分误差力状态 [in/out]
 * @return 映射到三个执行器电机的期望前馈力矩 (Nm)
 */
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
                                Eigen::Vector3d& F_int_local);

}  // namespace force_control
}  // namespace motor_test_cpp
