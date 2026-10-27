// estimator.cpp —— 机身状态估计实现 (IMU 姿态 + 互补滤波速度/高度)
// =============================================================================
//
// 【§A 姿态】直接采用 IMU 芯片融合 quat, 转 rpy + 角速度 (remount 180° 取反)。
//
// 【§B 速度/高度互补滤波器】(2026-07-18)
//   高频通路: IMU body 系加速度 → world 系, 扣重力, 积分 → vel_world, height。
//   低频通路: stance 腿足端不动 → v_body = -v_foot_body (body 系),
//             足端 world 几何 → z_meas = -(R·foot_body).z。
//   融合: vel_world 和 height 各用互补系数融合 IMU 积分(快,漂) + 足端测量(准,噪)。
//   接触腿不够时纯 IMU 积分, 不修正(trot 摆动期 ~0.2s, 漂 < 2cm)。
//
//   输出直接喂给 body_posture: vel_world.z → kd_height 竖直阻尼, vel_body → 将来水平阻尼,
//   pos_world.z → 高度监视。不估计水平位置(无约束发散, 控制不用)。
//
// 【历史】旧 foot-anchor 里程计 2026-06-27 已删 (est_z 漂到 0.5m 非物理)。
// =============================================================================

#include "parallel/estimation/estimator.hpp"

#include <algorithm>
#include <cmath>

namespace parallel {
namespace estimator {

// =============================================================================
// update —— 主入口, main 每拍调一次 (500Hz)。
//   §A 姿态: IMU quat → rpy + 角速度 (remount 180° 取反)
//   §B 速度/高度: IMU 加速度积分(高频) + stance 足端反算(低频) 互补滤波
// =============================================================================
void Estimator::update(const imu::ImuState& imu_state,
                       const std::array<Eigen::Vector3d, N_LEGS>& foot_body,
                       const std::array<Eigen::Vector3d, N_LEGS>& foot_vel_body,
                       const std::array<bool, N_LEGS>& is_contact,
                       double dt) {

    if (dt <= 0.0) dt = 0.002;   // 保护: 默认 500Hz

    // 首帧: 用 IMU quat 初始化姿态 + 机身高度初值
    if (first_update_) { init_from_imu(imu_state); first_update_ = false; }

    // =======================================================================
    // §A. 姿态: 每帧直接采用 IMU 芯片融合 quat (gyro+accel 融合, 重力参考)
    //    失败(芯片还没融合好)回退到 euler 反算
    // =======================================================================
    {
        Eigen::Vector4d q_imu(imu_state.quat.q0, imu_state.quat.q1,
                              imu_state.quat.q2, imu_state.quat.q3);
        // 退化保护: 模长太小 (IMU 未就绪) 或 NaN (启动瞬态) → 回退到 euler 反算
        const double qn2 = q_imu.squaredNorm();
        if (qn2 < 1e-6 || !std::isfinite(qn2))
            q_imu = euler_to_quat(imu_state.euler.roll, imu_state.euler.pitch, imu_state.euler.yaw);
        out_.quat_wxyz = normalize_quat(q_imu);
    }
    const Eigen::Matrix3d R   = quat_to_rotation(out_.quat_wxyz);
    const Eigen::Vector3d rpy = rpy_from_rotation(R);
    out_.roll  = -rpy[0];           // IMU remount 180°: 取反 roll/pitch
    out_.pitch = -rpy[1];
    out_.yaw   =  rpy[2];
    out_.ang_vel_body = Eigen::Vector3d(-imu_state.gyro.x, -imu_state.gyro.y, imu_state.gyro.z);

    // =======================================================================
    // §B. 速度/高度互补滤波器
    //
    // 高频通路 (IMU 加速度积分):
    //   将 IMU body 系加速度 (单位 g) 转到 world 系, 扣除重力, 积分得 world 系速度/高度。
    //   响应快(~ms) 但漂移 (acc bias 积分 → 线性发散)。
    //
    // 低频通路 (stance 足端反算):
    //   stance 腿足端在世界系不动 → body 系足端速度的负值 = body 速度 (body 系),
    //   转到 world 系 → v_world_meas。足端 world 坐标 → 机身高度 z_meas。
    //   无漂但噪声大 (关节速度 × Jacobian 放大编码器量化噪声)。
    //
    // 融合: vel_world = IMU 积分 + α_v·(测量 − IMU积分), 等价于高通 IMU + 低通测量。
    //       height    = IMU 积分 + α_h·(测量 − IMU积分), 但 α_h 更慢 (高度变化慢)。
    //       n_stance < min_stance_for_correction 时跳过低频修正 (对角 trot 只有 2 腿),
    //       纯 IMU 积分撑过去 (撑 ~0.2s swing, 漂 < 2cm)。
    //
    // 安全: imu_state.valid=false(IMU 未就绪/断连) → 跳过积分, 保持上一帧值防 NaN 污染。
    // =======================================================================
    if (imu_state.valid) {
        constexpr double G = 9.81;

        // B1. body 系加速度 → world 系, 去重力; NaN 守卫防 IMU 启动瞬态
        Eigen::Vector3d acc_body(imu_state.acc.x, imu_state.acc.y, imu_state.acc.z);
        if (!std::isfinite(acc_body.squaredNorm())) acc_body.setZero();
        acc_body *= G;                                             // g → m/s²
        const Eigen::Vector3d acc_world = R * acc_body;           // body→world 旋转
        const Eigen::Vector3d a_motion  = acc_world - Eigen::Vector3d(0.0, 0.0, G);

        // B2. IMU 积分 (高频通路)
        const Eigen::Vector3d vel_imu = out_.vel_world + a_motion * dt;
        const double          z_imu   = out_.pos_world.z() + out_.vel_world.z() * dt;

        // B3. stance 足端反算测量 (低频通路)
        Eigen::Vector3d v_body_meas(0.0, 0.0, 0.0);
        double          z_meas_sum = 0.0;
        int             n_stance   = 0;
        for (int i = 0; i < N_LEGS; ++i) {
            if (!is_contact[i]) continue;
            // stance 腿足端在地面不动 → body 系下 body 速度 = −足端速度
            v_body_meas -= foot_vel_body[i];
            // 几何关系: foot_world = R·foot_body + pos_world, 着地时 foot_world.z ≈ 0
            // → 机身高度 = −(R·foot_body).z  (不依赖当前 pos_world 估计, 避免自指)
            const Eigen::Vector3d foot_world_dir = R * foot_body[i];
            z_meas_sum -= foot_world_dir.z();
            ++n_stance;
        }

        // B4. 互补融合
        const bool enough_stance = (n_stance >= cfg_.min_stance_for_correction);
        if (enough_stance) {
            v_body_meas /= n_stance;
            z_meas_sum  /= n_stance;

            const Eigen::Vector3d vel_world_meas = R * v_body_meas;  // body→world

            // 速度: 低通融合 stance 足端测量, 消除 IMU 积分漂移
            out_.vel_world = vel_imu + cfg_.vel_complementary_alpha * (vel_world_meas - vel_imu);

            // 高度: 极慢融合 stance 足端测量, 消除二重积分漂移
            out_.pos_world.z() = z_imu + cfg_.height_complementary_alpha * (z_meas_sum - z_imu);

            // body 系速度: LPF 平滑关节速度噪声 (直接供 body_posture 水平阻尼用)
            out_.vel_body = out_.vel_body + cfg_.vel_body_lpf_alpha * (v_body_meas - out_.vel_body);
        } else {
            // 无接触或接触腿不够: 纯 IMU 积分, 不修正; body 速度保持上一帧
            out_.vel_world = vel_imu;
            out_.pos_world.z() = z_imu;
        }

        // B5. world 系水平位置不估计 (纯惯性无约束会发散, 控制不用)
        out_.pos_world.x() = 0.0;
        out_.pos_world.y() = 0.0;
    }

    // =======================================================================
    // §C. 诊断量
    // =======================================================================
    out_.n_active_contacts = (is_contact[0] ? 1 : 0) + (is_contact[1] ? 1 : 0)
                           + (is_contact[2] ? 1 : 0) + (is_contact[3] ? 1 : 0);
    out_.valid = true;   // IMU 姿态恒有效
}

// =============================================================================
// init_from_imu —— 首帧用 IMU quat 初始化姿态 + 默认机身高度(惰性)
// =============================================================================
void Estimator::init_from_imu(const imu::ImuState& imu) {
    Eigen::Vector4d q(imu.quat.q0, imu.quat.q1, imu.quat.q2, imu.quat.q3);
    if (q.squaredNorm() < 1e-6)
        q = euler_to_quat(imu.euler.roll, imu.euler.pitch, imu.euler.yaw);
    out_.quat_wxyz = normalize_quat(q);
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
}  // namespace parallel
