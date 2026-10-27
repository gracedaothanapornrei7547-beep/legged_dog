// estimator.hpp —— 机身状态估计 (IMU 姿态 + 互补滤波速度/高度)
// =============================================================================
// 所属模块: parallel::estimator (estimation 流水线第 3 步)
// 职责: 每拍(500Hz)输出:
//         §A 姿态: quat / roll·pitch·yaw / 角速度 (直接采 IMU 融合 quat, 不积分)
//         §B 速度: world 系线速度 (IMU 加速度积分 + stance 足端反算互补滤波)
//         §C 高度: pos_world.z (IMU 垂向二重积分 + stance 足端几何测量互补滤波)
//       vel_world.z → body_posture kd_height 竖直阻尼;
//       vel_body     → 留作水平阻尼 (当前控制未消费);
//       pos_world.z  → 高度监视 (当前控制未消费).
//       world 系水平位置不估计 (无约束发散).
//
// 历史: 旧 foot-anchor 里程计 2026-06-27 删; 2026-07-18 补互补滤波速度/高度.
// =============================================================================

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>

#include "parallel/common/pod_types.hpp"  // N_LEGS
#include "parallel/driver/imu_reader.hpp"

namespace parallel {
namespace estimator {

// ============================================================================
// 配置
// ============================================================================
struct EstimatorConfig {
    // 惰性 pos_world.z 初值(纯遥测字段首次 init 用)
    double initial_body_height = 0.23;

    // ---- 速度互补滤波器 ----
    // vel_world = IMU 加速度积分(高频通路) + stance 足端反算测量(低频通路)的融合。
    // IMU 积分响应快但会漂(dt 累积 bias)，足端测量无漂但噪声大(关节速度×Jacobian)。
    // 互补滤波: 信 IMU 的高频变化 + 信足端测量的低频均值。
    double vel_complementary_alpha = 0.02;   // 速度测量融合系数(τ≈100ms@500Hz)
    double vel_body_lpf_alpha      = 0.05;   // body 系速度 LPF(τ≈40ms@500Hz)，平滑关节速度噪声

    // ---- 高度互补滤波器 ----
    // pos_world.z = IMU 垂向加速度二重积分(高频) + stance 足端高度测量(低频)的融合。
    // 垂向加速度二重积分漂移快(1s 偏 0.01m/s²→5mm)，必须用 stance 足端高度拽回来。
    double height_complementary_alpha = 0.01; // 高度测量融合系数(τ≈200ms@500Hz)，慢融合防踏空抖动

    // ---- 速度方向门控 ----
    // stance 腿数量太少(对角 trot 只有 2 腿)时足端反算的 body 速度噪声大，
    // 低于此阈值时跳过低频修正，纯用 IMU 积分撑过去。
    int min_stance_for_correction = 2;
};

// ============================================================================
// 输出
// ============================================================================
struct EstimatorOutput {
    // ---- 姿态(纯 IMU，控制实际消费的就这些)----
    Eigen::Vector4d quat_wxyz{1.0, 0.0, 0.0, 0.0};
    double roll = 0.0, pitch = 0.0, yaw = 0.0;   // rad（来自 IMU quat）
    Eigen::Vector3d ang_vel_body{0.0, 0.0, 0.0}; // rad/s（陀螺, body 系）

    // ---- 速度/高度 (互补滤波估计, 2026-07-18 新增) ----
    // vel_world.z → body_posture kd_height 竖直阻尼(活跃消费)
    // vel_body     → 留作水平阻尼 (当前控制未消费)
    // pos_world.z  → 高度监视 (当前控制未消费)
    // pos_world.xy → 惰性零值 (纯惯性无约束发散, 控制不用)
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

    // 单拍更新：IMU 姿态(quat/rpy/角速度) + 速度/高度互补滤波。
    //   imu_state        : 最新 IMU 快照(quat/euler/gyro/acc)
    //   foot_body[i]     : 第 i 腿足端 body 系坐标(m)，来自 FK (measure_foot)
    //   foot_vel_body[i] : 第 i 腿足端 body 系速度(m/s)，来自 J·qdq (measure_foot)
    //   is_contact[i]    : 第 i 腿是否接触(ContactEstimator 迟滞滤波后)
    //   dt               : 本拍步长(s)
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
}  // namespace parallel
