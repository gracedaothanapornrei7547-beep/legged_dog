// estimator.hpp —— 机身姿态估计（纯 IMU 版）
// =============================================================================
//
// 【模块定位】
//   IMU(芯片融合 quat) → ★estimator → body_posture(姿态环) / capture(陀螺)
//
//   只做一件事：每拍(500Hz)输出机身姿态 —— quat / roll·pitch·yaw / 角速度。
//   姿态直接采用 IMU 芯片融合好的 quat(gyro+accel, 重力参考)，不做 open-loop
//   积分(避免 gyro bias 累积漂)。
//
// 【历史：foot-anchor pos/vel 里程计已删 (2026-06-27)】
//   旧版还用接触腿"锚点法"估计 pos_world / vel_world(IMU 积分 + 足端修正 + bias 学习)。
//   实测该里程计输出被估计器每步重锚抖 + 漂污染(est_z 漂到 0.5m 非物理)，且下游控制
//   根本不消费它(只用姿态)，留着纯是误导 + 算力 → 整摊删除。
//   pos_world / vel_world / vel_body 字段保留为【惰性零值】，仅供遥测字段兼容，下游不读。
//
// 【输入 / 输出】
//   输入: IMU snapshot(quat/euler/gyro) + is_contact[4](仅用于数 n_active 诊断)
//         (foot_body / foot_vel_body / dt 形参保留以兼容 main 调用签名，但纯 IMU 版不用)
//   输出: quat / roll·pitch·yaw / ang_vel_body(陀螺)  ；pos/vel = 惰性零值(遥测兼容)
// =============================================================================

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>

#include "motor_test/common/pod_types.hpp"  // N_LEGS
#include "motor_test/driver/imu_reader.hpp"

namespace motor_test_cpp {
namespace estimator {

// ============================================================================
// 配置
// ============================================================================
struct EstimatorConfig {
    // 惰性 pos_world.z 初值(仅遥测字段; 控制不读)
    double initial_body_height = 0.23;
};

// ============================================================================
// 输出
// ============================================================================
struct EstimatorOutput {
    // ---- 姿态(纯 IMU，控制实际消费的就这些)----
    Eigen::Vector4d quat_wxyz{1.0, 0.0, 0.0, 0.0};
    double roll = 0.0, pitch = 0.0, yaw = 0.0;   // rad（来自 IMU quat）
    Eigen::Vector3d ang_vel_body{0.0, 0.0, 0.0}; // rad/s（陀螺, body 系）

    // ---- 惰性零值(foot-anchor 里程计已删; 仅遥测字段兼容, 下游不读)----
    Eigen::Vector3d pos_world{0.0, 0.0, 0.23};
    Eigen::Vector3d vel_world{0.0, 0.0, 0.0};
    Eigen::Vector3d vel_body {0.0, 0.0, 0.0};

    // ---- 诊断 ----
    int  n_active_contacts = 0;   // = 本帧 is_contact 计数
    bool valid             = true;
};

// ============================================================================
// 估计器
// ============================================================================
class Estimator {
public:
    explicit Estimator(const EstimatorConfig& cfg = {}) : cfg_(cfg) { reset(); }

    void reset() {
        out_ = EstimatorOutput{};
        out_.pos_world = Eigen::Vector3d(0.0, 0.0, cfg_.initial_body_height);
        first_update_ = true;
    }

    // 单拍更新（纯 IMU：只更新姿态 quat/rpy/角速度）。
    //   imu_state      : 最新 IMU 快照(quat/euler/gyro)
    //   is_contact[i]  : 仅用于数 n_active_contacts(诊断)
    //   foot_body / foot_vel_body / dt : 形参保留(签名兼容 main)，纯 IMU 版不用
    void update(const imu::ImuState& imu_state,
                const std::array<Eigen::Vector3d, N_LEGS>& foot_body,
                const std::array<Eigen::Vector3d, N_LEGS>& foot_vel_body,
                const std::array<bool, N_LEGS>& is_contact,
                double dt);

    const EstimatorOutput& output() const { return out_; }

private:
    EstimatorConfig cfg_;
    EstimatorOutput out_;
    bool            first_update_ = true;

    // 首帧用 IMU quat 初始化姿态
    void init_from_imu(const imu::ImuState& imu);

    // ---- 数学工具 ----
    static Eigen::Vector4d normalize_quat(const Eigen::Vector4d& q);
    static Eigen::Matrix3d quat_to_rotation(const Eigen::Vector4d& q);
    static Eigen::Vector4d euler_to_quat(double roll, double pitch, double yaw);
    static Eigen::Vector3d rpy_from_rotation(const Eigen::Matrix3d& R);
};

}  // namespace estimator
}  // namespace motor_test_cpp
