// estimator.cpp —— 机身姿态估计实现（纯 IMU 版）
// =============================================================================
//
// 【这个文件干啥】
//   每拍(500Hz)输出机身姿态: quat / roll·pitch·yaw / 角速度(陀螺)。
//   姿态直接采用 IMU 芯片融合好的 quat(gyro+accel 融合, 重力参考), 不积分。
//
// 【历史: foot-anchor pos/vel 里程计已删 (2026-06-27)】
//   旧版用接触腿"锚点法"估 pos_world/vel_world(IMU 积分 + 足端修正 + bias 学习)。
//   实测输出被每步重锚抖 + 漂污染(est_z 漂到 0.5m), 下游控制不消费(只用姿态),
//   删整摊。pos_world/vel_world/vel_body 留惰性零值(遥测兼容, 见 .hpp)。
// =============================================================================

#include "motor_test/estimation/estimator.hpp"

#include <algorithm>
#include <cmath>

namespace motor_test_cpp {
namespace estimator {

// =============================================================================
// 主入口 —— main 每拍调一次。纯 IMU: 只更新姿态(quat/rpy/角速度)。
//   foot_body / foot_vel_body / dt 形参保留(签名兼容 main 调用)但不再使用。
// =============================================================================
void Estimator::update(const imu::ImuState& imu_state,
                       const std::array<Eigen::Vector3d, N_LEGS>& foot_body,
                       const std::array<Eigen::Vector3d, N_LEGS>& foot_vel_body,
                       const std::array<bool, N_LEGS>& is_contact,
                       double dt) {
    (void)foot_body; (void)foot_vel_body; (void)dt;   // foot-anchor 里程计已删, 不再用

    // 首帧: 用 IMU quat 初始化姿态
    if (first_update_) { init_from_imu(imu_state); first_update_ = false; }

    // ===== 姿态: 直接透传 IMU euler =====
    out_.roll  = -imu_state.euler.roll;
    out_.pitch = -imu_state.euler.pitch;
    out_.yaw   =  imu_state.euler.yaw;
    out_.ang_vel_body = Eigen::Vector3d(-imu_state.gyro.x, -imu_state.gyro.y, imu_state.gyro.z);
    // ★quat 每帧更新: body_posture gravity_comp 靠它做倾斜重力补偿
    out_.quat_wxyz = euler_to_quat(-imu_state.euler.roll, -imu_state.euler.pitch, imu_state.euler.yaw);

    // ===== 诊断量 (pos/vel 保持惰性零值: foot-anchor 已删, 下游不读) =====
    out_.n_active_contacts = (is_contact[0] ? 1 : 0) + (is_contact[1] ? 1 : 0)
                           + (is_contact[2] ? 1 : 0) + (is_contact[3] ? 1 : 0);
    out_.valid = true;   // IMU 姿态恒有效(不再有 foot-anchor watchdog)
}

// =============================================================================
// init_from_imu —— 首帧用 IMU quat 初始化姿态 + 默认机身高度(惰性)
// =============================================================================
void Estimator::init_from_imu(const imu::ImuState& imu) {
    out_.quat_wxyz = Eigen::Vector4d(1,0,0,0);  // 惰性, 不再用
    out_.pos_world = Eigen::Vector3d(0.0, 0.0, cfg_.initial_body_height);  // 惰性遥测初值
    out_.vel_world.setZero();
}

// =============================================================================
// 数学工具（四元数 / 旋转 / Tait-Bryan）
// =============================================================================

// 归一化四元数，强制 w >= 0（统一双覆盖：q 和 -q 表示同一旋转）
Eigen::Vector4d Estimator::normalize_quat(const Eigen::Vector4d& q) {
    const double n = q.norm();
    if (n < 1e-12) return Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);  // 退化保护
    Eigen::Vector4d q2 = q / n;
    if (q2[0] < 0.0) q2 = -q2;  // 强制 w >= 0，避免符号跳变
    return q2;
}

// 四元数 → 3×3 旋转矩阵（标准 Hamilton 公式）
Eigen::Matrix3d Estimator::quat_to_rotation(const Eigen::Vector4d& q) {
    const Eigen::Vector4d qn = normalize_quat(q);
    const double w = qn[0], x = qn[1], y = qn[2], z = qn[3];
    Eigen::Matrix3d R;
    R << 1.0 - 2.0*(y*y + z*z),   2.0*(x*y - z*w),       2.0*(x*z + y*w),
         2.0*(x*y + z*w),         1.0 - 2.0*(x*x + z*z), 2.0*(y*z - x*w),
         2.0*(x*z - y*w),         2.0*(y*z + x*w),       1.0 - 2.0*(x*x + y*y);
    return R;
}

// Tait-Bryan ZYX 欧拉角 → 四元数（R = Rz · Ry · Rx）
Eigen::Vector4d Estimator::euler_to_quat(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll  * 0.5), sr = std::sin(roll  * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw   * 0.5), sy = std::sin(yaw   * 0.5);
    return Eigen::Vector4d(
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy);
}

// 旋转矩阵 → Tait-Bryan ZYX 欧拉角（pitch ≈ ±90° 万向锁，实际机器人远小于没事）
Eigen::Vector3d Estimator::rpy_from_rotation(const Eigen::Matrix3d& R) {
    const double roll  = std::atan2(R(2,1), R(2,2));
    const double pitch = std::asin (std::max(-1.0, std::min(1.0, -R(2,0))));
    const double yaw   = std::atan2(R(1,0), R(0,0));
    return Eigen::Vector3d(roll, pitch, yaw);
}

}  // namespace estimator
}  // namespace motor_test_cpp
