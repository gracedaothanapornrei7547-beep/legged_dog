// =============================================================================
// 文件: body_posture.cpp
// 模块: parallel::body_posture (躯干空间姿态与力矩计算)
// 职责: 根据状态估计反馈计算躯干期望 6D 空间旋量 (Wrench Desired)
//
// 空间力与力矩控制律:
//   - 法向力 (Fz): 刚体静力重力前馈 m*g + 垂直速度阻尼 (结合 IMU 世界系加速度积分滤波)
//   - 斜坡重力投影: F_body = R(q)^T * [0, 0, m*g]，自动在倾斜地形产生沿坡水平分力抗下滑
//   - 横滚力矩 (Mx): +kp_roll * (roll - roll_des) - kd_roll * omega_x
//   - 俯仰力矩 (My): +kp_pitch * (pitch - pitch_des) - kd_pitch * omega_y
//   - 偏航力矩 (Mz): -kp_yaw * (yaw - yaw_des) - kd_yaw * (omega_z - yawrate_cmd)
//   所有力矩分量最终经过严格的幅值对称限幅保护 (±moment_clip)。
// =============================================================================

#include "parallel/dynamics/body_posture.hpp"

#include <algorithm>
#include <Eigen/Geometry>   // Quaterniond（重力补偿 R^T·g）

namespace parallel {
namespace body_posture {

WrenchDes compute_wrench_des(const BodyPostureConfig& cfg,
                              const estimator::EstimatorOutput& est,
                              double yawrate_cmd,
                              double& vz_filt_state) {
    WrenchDes w;

    // ===== fz：纯重力前馈 m·g（托住机身）=====
    // fz = 重力前馈 m·g + 垂直速度阻尼。机身高度由 IK stance_z 几何锁定, 不依赖 pos 估计。
    // 重力补偿(斜坡): F_body = R^T·(0,0,mg), 用 IMU quat 旋回 body 系。斜坡上多出沿坡水平
    //   分力≈mg·sinθ→腿抗下滑。水平分力钳 grav_comp_hclip 防坏姿态读数失控。
    // 速度阻尼(楼梯): vz=IMU 加速度短时积分+20ms LPF, 下坠 vz<0→fz_damp>0 支撑腿加力"吊着放下去"。
    constexpr double VZ_LPF_ALPHA = 0.091;   // τ=20ms @500Hz
    vz_filt_state = VZ_LPF_ALPHA * est.vel_world.z() + (1.0 - VZ_LPF_ALPHA) * vz_filt_state;
    const double fz_damp = std::clamp(-cfg.kd_height * vz_filt_state,-cfg.fz_fb_clip, cfg.fz_fb_clip);

    const double mg = cfg.mass_kg * cfg.gravity + fz_damp;
    if (cfg.gravity_comp) {
        const Eigen::Vector4d& q = est.quat_wxyz;            // [w,x,y,z], body→world
        Eigen::Quaterniond Q(q[0], q[1], q[2], q[3]);
        Q.normalize();
        Eigen::Vector3d f = Q.conjugate() * Eigen::Vector3d(0.0, 0.0, mg);  // R^T·(0,0,mg+damp)=世界竖直力在body系
        const double hclip = cfg.grav_comp_hclip * cfg.mass_kg * cfg.gravity;  // 水平分力上限(按纯重力算, 不随阻尼浮动)
        f.x() = std::clamp(f.x(), -hclip, hclip);
        f.y() = std::clamp(f.y(), -hclip, hclip);
        f.z() = std::clamp(f.z(), cfg.fz_min, cfg.fz_max);
        w.force = f;
    } else {
        w.force.z() = std::clamp(mg, cfg.fz_min, cfg.fz_max);   // 平地老行为: 纯 body 系 (0,0,mg)
    }
    // fx/fy 现由重力补偿在斜坡上给(沿坡水平分力); 速度闭环仍删(不依赖 vel 估计)

    // ===== 姿态闭环 (mx, my) =====
    //   mx = +kp_roll·(roll−roll_des) − kd_roll·ωx    (roll 位置项用 +kp: est 约定与 force_alloc 符号验证)
    //   my = +kp_pitch·(pitch−pitch_des) − kd_pitch·ωy
    //   kd ≤ 2 较稳：trot 自然 ω≈±1 rad/s, kd 大→高频力矩抖进 moment_clip。
    //   roll/pitch符号已真机定案(estimator内统一修正)
    w.moment.x() = +cfg.kp_roll  * (est.roll  - cfg.roll_des)  - cfg.kd_roll  * est.ang_vel_body.x();
    w.moment.y() = +cfg.kp_pitch * (est.pitch - cfg.pitch_des) - cfg.kd_pitch * est.ang_vel_body.y();

    // ===== yaw 闭环 (mz)：位置 + 速度 =====
    //   yaw_err = yaw - yaw_des    (需要 wrap 到 [-π, π] 避免跨周期跳变)
    //   mz = -kp_yaw · yaw_err - kd_yaw · (ω_z - yawrate_cmd)
    //
    // yaw_des 由 main 在 startup 结束时 latch 当前 est.yaw（=机身初始航向）
    // 这样开机方向就是"想保持的方向"，狗不会自己慢慢转圈
    double yaw_err = est.yaw - cfg.yaw_des;
    while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
    w.moment.z() = -cfg.kp_yaw * yaw_err - cfg.kd_yaw * (est.ang_vel_body.z() - yawrate_cmd);

    // ===== 力矩限幅 =====
    // mx/my/mz 都钳到 ±moment_clip，避免单点失稳引起大命令
    for (int k = 0; k < 3; ++k) {
        w.moment[k] = std::clamp(w.moment[k], -cfg.moment_clip, cfg.moment_clip);
    }
    return w;
}

}  // namespace body_posture
}  // namespace parallel
