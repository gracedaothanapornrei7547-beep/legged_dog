/**
 * @file force_control.cpp
 * @brief 笛卡尔空间虚拟模型力控制 (VMC) 与雅可比转置映射实现
 * 
 * 核心理论与控制流程:
 * 1. 笛卡尔空间阻抗控制律 (Cartesian Impedance Law):
 *    - 依据支撑权重 support_weight 连续插补摆动相与支撑相刚度/阻尼矩阵 Kp, Kd
 *    - 计算位移与速度跟踪误差: F_imp = Kp * (p_des - p_act) + Kd * (v_des - v_act)
 * 2. 矢状面操作空间惯量解耦 (WBC-lite OSIC in Sagittal Plane):
 *    - 仅在矢状面 (x, z) 提取 2x2 雅可比矩阵 Jsag 并计算局部惯量椭球 (Jsag * Jsag^T)^-1
 *    - 引入奇异阻尼与归一化缩放，混合原生阻抗与操作空间惯量整形矩阵
 * 3. 接触反作用力合成 (Contact Reaction Force Formulation):
 *    - 基于牛顿第三定律修正符号: F_local = F_imp - F_body_local
 * 4. 虚功原理力矩映射与硬件方向对齐 (Virtual Work Mapping & Actuator Clamping):
 *    - 基于实测物理构型计算解析雅可比: tau_math = J(math_act)^T * F_local
 *    - 物理电机旋向标定转换与执行器力矩硬限幅保护: tau_motor = clamp(cal.math_to_motor(tau_math), ±tau_max)
 */

#include "motor_test/dynamics/force_control.hpp"

#include <algorithm>
#include <cmath>
#include <Eigen/Dense>

namespace motor_test_cpp {
namespace force_control {

using kinematics::FKResult;
using kinematics::Mat3x3;

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
                                Eigen::Vector3d& F_int_local) {

    // 0. 基于实测关节构型提前解算正运动学与解析雅可比矩阵
    const FKResult fk = kin.forward_kinematics(math_act[0], math_act[1], math_act[2]);
    Mat3x3 J;
    Eigen::Vector3d v_unused;
    kin.jacobian(math_act[0], math_act[1], math_act[2],
                 0.0, 0.0, 0.0, fk, J, v_unused);

    // 1. 笛卡尔空间阻抗控制项计算 (基于支撑权重连续插补)
    Eigen::Vector3d F_imp = Eigen::Vector3d::Zero();
    if (cfg.imp_enabled) {
        const double w = std::clamp(support_weight, 0.0, 1.0);

        // 摆动相各向同性跟踪刚度
        Eigen::Vector3d Kp_swing_vec, Kd_swing_vec;
        Kp_swing_vec << cfg.kp_swing, cfg.kp_swing, cfg.kp_swing;
        Kd_swing_vec << cfg.kd_swing, cfg.kd_swing, cfg.kd_swing;

        // 支撑相空间阻抗增益
        Eigen::Vector3d Kp_stance, Kd_stance;
        Kp_stance << cfg.kp_stance_x, cfg.kp_stance_y, cfg.kp_stance_z;
        Kd_stance << cfg.kd_stance_x, cfg.kd_stance_y, cfg.kd_stance_z;

        // 连续加权混合增益
        const Eigen::Vector3d Kp = (1.0 - w) * Kp_swing_vec + w * Kp_stance;
        const Eigen::Vector3d Kd = (1.0 - w) * Kd_swing_vec + w * Kd_stance;

        const Eigen::Vector3d dp = p_des_local - p_act_local;
        const Eigen::Vector3d dv = v_des_local - v_act_local;
        const Eigen::Vector3d fb = Kp.cwiseProduct(dp) + Kd.cwiseProduct(dv);

        // 矢状面操作空间惯量整形 (WBC-lite OSIC)
        if (cfg.wbc_lite && support_weight < cfg.wbc_swing_gate) {
            Eigen::Matrix2d Jsag;
            Jsag << J(0, 1), J(0, 2),
                    J(2, 1), J(2, 2);
            Eigen::Matrix2d JJt = Jsag * Jsag.transpose();
            JJt += cfg.wbc_damp * JJt.trace() * Eigen::Matrix2d::Identity();
            const Eigen::Matrix2d Linv = JJt.inverse();
            const double det = Linv.determinant();
            const double s = (det > 1e-12) ? std::sqrt(det) : 1.0;
            const double a = std::clamp(cfg.wbc_blend, 0.0, 1.0);
            const Eigen::Matrix2d Leff =
                (1.0 - a) * Eigen::Matrix2d::Identity() + a * (Linv / s);
            const Eigen::Vector2d Fxz = Leff * Eigen::Vector2d(fb(0), fb(2));
            F_imp = Eigen::Vector3d(Fxz(0), fb(1), Fxz(1));
        } else {
            F_imp = fb;
        }

        // 笛卡尔误差积分项 (消除稳态静差并施加抗饱和限幅)
        {
            constexpr double dt_ctrl = 0.002;
            Eigen::Vector3d Ki_swing_vec{cfg.ki_swing, cfg.ki_swing, cfg.ki_swing};
            Eigen::Vector3d Ki_stance_vec{cfg.ki_stance_x, cfg.ki_stance_y, cfg.ki_stance_z};
            const Eigen::Vector3d Ki = (1.0 - w) * Ki_swing_vec + w * Ki_stance_vec;
            F_int_local += Ki.cwiseProduct(dp) * dt_ctrl;
            for (int k = 0; k < 3; ++k) {
                F_int_local[k] = std::clamp(F_int_local[k], -cfg.ki_windup_n, cfg.ki_windup_n);
            }
            F_imp += F_int_local;
        }
    }

    // 2. 机身系分配支撑力投影至支链局部坐标系
    const Eigen::Vector3d F_body_local = cal.body_to_local(F_body);

    // 3. 接触力学反作用力合成 (地反力对机身作用反力取负)
    const Eigen::Vector3d F_local = F_imp - F_body_local;

    // 4. 虚功原理映射至关节空间数学力矩: tau_math = J^T * F_local
    const Eigen::Vector3d tau_math = J.transpose() * F_local;

    // 5. 按照电机安装拓扑转换至物理执行器旋向
    Eigen::Vector3d tau_motor = cal.math_torque_to_motor(tau_math[0], tau_math[1], tau_math[2]);

    // 6. 执行器峰值力矩硬件安全钳位
    for (int k = 0; k < 3; ++k) {
        tau_motor[k] = std::clamp(tau_motor[k], -cfg.tau_max, cfg.tau_max);
    }
    return tau_motor;
}

}  // namespace force_control
}  // namespace motor_test_cpp
