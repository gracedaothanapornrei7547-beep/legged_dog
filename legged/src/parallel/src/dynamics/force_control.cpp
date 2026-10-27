// =============================================================================
// 文件: force_control.cpp
// 模块: parallel::force_control (动力学与虚拟模型控制)
// 职责: 单腿期望接触力与笛卡尔阻抗映射至关节力矩 (虚功原理与雅可比转置映射)
//
// 控制管线流程:
//   1. 笛卡尔空间虚拟阻抗 (Cartesian Impedance):
//      F_imp = Kp * (p_des - p_act) + Kd * (v_des - v_act)
//      增益矩阵按连续支撑权重 support_weight 在摆动相与支撑相之间平滑过渡插值。
//   2. 矢状面操作空间惯量整形 (WBC-lite OSIC):
//      计算 2x2 矢状面操作空间惯量矩阵 Lambda_sag = (J_sag * J_sag^T)^(-1)，
//      对摆动反馈力进行自适应构型拉伸，补偿 5-bar 并联机构在特定构型下的各向异性缺陷。
//   3. 刚体静力学平衡投影 (Principle of Virtual Work):
//      tau_math = J(q_act)^T * (F_imp - F_body_local)
//      注意: 机身分配力项取负符合牛顿第三定律反作用力约定。雅可比必须基于实测关节角
//      q_act 进行计算，确保几何映射反映当前瞬时位形，避免在跟踪误差较大时失稳。
//   4. 坐标系转换与力矩限幅:
//      tau_motor = clamp(cal.math_torque_to_motor(tau_math), ±tau_max)
// =============================================================================

#include "parallel/dynamics/force_control.hpp"

#include <algorithm>
#include <cmath>          // std::cbrt (WBC-lite det^(1/3) 归一化)
#include <Eigen/Dense>    // Matrix3d .inverse()/.determinant() (WBC-lite Λ)

namespace parallel {
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

    // ===== J(math_act) 提前算（WBC-lite Λ 整形 与 步骤4 投影 共用）=====
    //   原来 J 在步骤4才算; WBC-lite 需要它在步骤1整形 F_imp, 故上提, 步骤4直接复用。
    const FKResult fk = kin.forward_kinematics(math_act[0], math_act[1], math_act[2]);
    Mat3x3 J;
    Eigen::Vector3d v_unused;   // jacobian() 顺便算 v_local（这里用不到）
    kin.jacobian(math_act[0], math_act[1], math_act[2],
                 0.0, 0.0, 0.0, fk, J, v_unused);

    // ===== 步骤 1: 笛卡尔阻抗 F_imp（local 系）=====
    //
    // F_imp = Kp · (p_des - p_act) + Kd · (v_des - v_act)
    //
    // 这是个"软弹簧 + 阻尼器"，让脚跟踪 p_des / v_des。
    //
    // ★ 关键：Kp/Kd 按 support_weight ∈ [0,1] 在 swing/stance 间插值：
    //         w = 0: swing 增益（追轨迹，三轴同 Kp_swing）
    //         w = 1: stance 增益（z 通常为 0，让 F_body 主导承重）
    //         中间：smooth 切换，避免 phase 边界 Kp 阶跃 → 命令突变 → 抖
    Eigen::Vector3d F_imp = Eigen::Vector3d::Zero();
    if (cfg.imp_enabled) {
        const double w = std::clamp(support_weight, 0.0, 1.0);

        // swing 增益（三轴同值，等向追踪 swing 抛物线）
        Eigen::Vector3d Kp_swing_vec, Kd_swing_vec;
        Kp_swing_vec << cfg.kp_swing, cfg.kp_swing, cfg.kp_swing;
        Kd_swing_vec << cfg.kd_swing, cfg.kd_swing, cfg.kd_swing;

        // stance 增益（三轴独立，z 默认 0 让 F_body 顶住）
        Eigen::Vector3d Kp_stance, Kd_stance;
        Kp_stance << cfg.kp_stance_x, cfg.kp_stance_y, cfg.kp_stance_z;
        Kd_stance << cfg.kd_stance_x, cfg.kd_stance_y, cfg.kd_stance_z;

        // 在 swing(w=0) 和 stance(w=1) 之间线性插值
        const Eigen::Vector3d Kp = (1.0 - w) * Kp_swing_vec + w * Kp_stance;
        const Eigen::Vector3d Kd = (1.0 - w) * Kd_swing_vec + w * Kd_stance;

        // PD 公式（反馈项）
        const Eigen::Vector3d dp = p_des_local - p_act_local;
        const Eigen::Vector3d dv = v_des_local - v_act_local;
        const Eigen::Vector3d fb = Kp.cwiseProduct(dp) + Kd.cwiseProduct(dv);

        // WBC-lite 矢状面操作空间惯量整形:
        // 在摆动相通过 2x2 矢状面操作空间惯量 Lambda_sag = (Jsag * Jsag^T)^(-1) 对反馈力进行自适应拉伸，
        // 补偿五连杆机构在向前伸展位形下的力学各向异性（解决领头腿摆动跟踪欠抬问题）。
        // 保持侧向 (y 轴) 原阻抗不变，避免三维全矩阵求逆在 th3 斜轴弱轴产生过大条件数导致横滚发散。
        if (cfg.wbc_lite && support_weight < cfg.wbc_swing_gate) {
            Eigen::Matrix2d Jsag;
            Jsag << J(0, 1), J(0, 2),    // ∂x/∂th1, ∂x/∂th2
                    J(2, 1), J(2, 2);    // ∂z/∂th1, ∂z/∂th2
            Eigen::Matrix2d JJt = Jsag * Jsag.transpose();
            JJt += cfg.wbc_damp * JJt.trace() * Eigen::Matrix2d::Identity();   // Tikhonov 奇异阻尼项
            const Eigen::Matrix2d Linv = JJt.inverse();                        // 操作空间惯量形状
            const double det = Linv.determinant();
            const double s = (det > 1e-12) ? std::sqrt(det) : 1.0;             // 行列式平方根归一化
            const double a = std::clamp(cfg.wbc_blend, 0.0, 1.0);
            const Eigen::Matrix2d Leff =
                (1.0 - a) * Eigen::Matrix2d::Identity() + a * (Linv / s);
            const Eigen::Vector2d Fxz = Leff * Eigen::Vector2d(fb(0), fb(2));
            F_imp = Eigen::Vector3d(Fxz(0), fb(1), Fxz(1));
        } else {
            F_imp = fb;
        }

        // 笛卡尔空间误差积分 (带抗饱和抗风积 Anti-Windup 钳位)
        // 仅在支撑相生效，消除稳态位置误差；摆动相积分增益置零。
        {
            constexpr double dt_ctrl = 0.002;   // @500Hz
            const double w = std::clamp(support_weight, 0.0, 1.0);
            Eigen::Vector3d Ki_swing_vec{cfg.ki_swing, cfg.ki_swing, cfg.ki_swing};
            Eigen::Vector3d Ki_stance_vec{cfg.ki_stance_x, cfg.ki_stance_y, cfg.ki_stance_z};
            const Eigen::Vector3d Ki = (1.0 - w) * Ki_swing_vec + w * Ki_stance_vec;
            const Eigen::Vector3d dp = p_des_local - p_act_local;
            F_int_local += Ki.cwiseProduct(dp) * dt_ctrl;
            for (int k = 0; k < 3; ++k)
                F_int_local[k] = std::clamp(F_int_local[k], -cfg.ki_windup_n, cfg.ki_windup_n);
            F_imp += F_int_local;
        }
    }

    // ===== 步骤 2: F_body (body 系) → F_local =====
    //
    // body_to_local 按 LegCalibration 的 x_sign/y_sign/z_sign 翻号
    // 不同腿的 local 系朝向不同（前/后腿绕 z 转 180°）
    const Eigen::Vector3d F_body_local = cal.body_to_local(F_body);

    // ===== 步骤 3: 足端期望力合成 (局部坐标系) =====
    // F_local = F_imp (笛卡尔虚拟阻抗) - F_body_local (机身承重与平衡力分配)
    // 根据作用力与反作用力定律，为向机身提供 F_body 的支撑反力，足端推地力必须取负号。
    const Eigen::Vector3d F_local = F_imp - F_body_local;

    // ===== 步骤 4: 基于实测位形计算虚功映射 (tau_math = J^T * F) =====
    // 雅可比矩阵基于瞬时实测关节角 math_act 计算，准确反映实际机构构型所能产生的虚功约束。
    const Eigen::Vector3d tau_math = J.transpose() * F_local;   // J/fk 已在函数顶部预先计算

    // ===== 步骤 5: 坐标变换与电机极性映射 =====
    // 根据腿部标定参数中的电机装配方向 (dir_th3, dir_th1, dir_th2) 进行符号映射转换。
    Eigen::Vector3d tau_motor = cal.math_torque_to_motor(tau_math[0], tau_math[1], tau_math[2]);

    // ===== 步骤 6: 关节力矩饱和限幅 (执行器保护) =====
    for (int k = 0; k < 3; ++k) {
        tau_motor[k] = std::clamp(tau_motor[k], -cfg.tau_max, cfg.tau_max);
    }
    return tau_motor;
}

}  // namespace force_control
}  // namespace parallel
