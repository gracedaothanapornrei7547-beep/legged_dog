/**
 * @file body_posture.cpp
 * @brief 浮动基机身姿态与空间扳手 (6D Spatial Wrench) 计算实现
 * 
 * 核心理论与控制流程:
 * 1. 重力前馈与垂直速度阻尼 (Gravity Feedforward & Vertical Velocity Damping):
 *    - 垂直支撑力: Fz = m * g - kd_height * vz_filt
 * 2. 斜坡姿态全向重力补偿 (Quaternion-based Gravity Projection):
 *    - F_body = R(q)^T * [0, 0, mg + damp]^T
 *    - 自动生成斜坡抗下滑沿坡水平切向推力
 * 3. 浮动基三维空间旋转姿态阻抗闭环 (Orientation PD Impedance):
 *    - Roll 轴与 Pitch 轴姿态稳定纠偏力矩
 *    - Yaw 轴偏航角航向保持与角速度跟踪阻尼 (带 [-pi, pi] 角度周期回绕)
 * 4. 控制指令安全限幅 (Moment Clamping Protection):
 *    - 限制输出空间力矩至 [ -moment_clip, +moment_clip ]，防止执行器超限
 */

#include "motor_test/dynamics/body_posture.hpp"

#include <algorithm>
#include <Eigen/Geometry>

namespace motor_test_cpp {
namespace body_posture {

WrenchDes compute_wrench_des(const BodyPostureConfig& cfg,
                             const estimator::EstimatorOutput& est,
                             double yawrate_cmd,
                             double& vz_filt_state) {
    WrenchDes w;

    // 1. 垂直重力前馈与速度阻尼计算
    constexpr double VZ_LPF_ALPHA = 0.091;
    vz_filt_state = VZ_LPF_ALPHA * est.vel_world.z() + (1.0 - VZ_LPF_ALPHA) * vz_filt_state;
    const double fz_damp = std::clamp(-cfg.kd_height * vz_filt_state,
                                      -cfg.fz_fb_clip, cfg.fz_fb_clip);

    const double mg = cfg.mass_kg * cfg.gravity + fz_damp;

    // 2. 空间四元数重力矢量投影 (适应非水平坡面环境)
    if (cfg.gravity_comp) {
        const Eigen::Vector4d& q = est.quat_wxyz;
        Eigen::Quaterniond Q(q[0], q[1], q[2], q[3]);
        Q.normalize();
        Eigen::Vector3d f = Q.conjugate() * Eigen::Vector3d(0.0, 0.0, mg);
        const double hclip = cfg.grav_comp_hclip * cfg.mass_kg * cfg.gravity;
        f.x() = std::clamp(f.x(), -hclip, hclip);
        f.y() = std::clamp(f.y(), -hclip, hclip);
        f.z() = std::clamp(f.z(), cfg.fz_min, cfg.fz_max);
        w.force = f;
    } else {
        w.force.z() = std::clamp(mg, cfg.fz_min, cfg.fz_max);
    }

    // 3. 翻滚角 (Roll) 与俯仰角 (Pitch) 姿态纠偏力矩
    w.moment.x() = +cfg.kp_roll  * (est.roll  - cfg.roll_des)  - cfg.kd_roll  * est.ang_vel_body.x();
    w.moment.y() = +cfg.kp_pitch * (est.pitch - cfg.pitch_des) - cfg.kd_pitch * est.ang_vel_body.y();

    // 4. 偏航角 (Yaw) 航向闭环保持与角速度跟踪
    double yaw_err = est.yaw - cfg.yaw_des;
    while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
    w.moment.z() = -cfg.kp_yaw * yaw_err - cfg.kd_yaw * (est.ang_vel_body.z() - yawrate_cmd);

    // 5. 空间力矩安全幅值钳位
    for (int k = 0; k < 3; ++k) {
        w.moment[k] = std::clamp(w.moment[k], -cfg.moment_clip, cfg.moment_clip);
    }
    return w;
}

}  // namespace body_posture
}  // namespace motor_test_cpp
