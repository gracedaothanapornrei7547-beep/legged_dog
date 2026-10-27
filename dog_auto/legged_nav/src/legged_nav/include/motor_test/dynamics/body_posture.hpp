/**
 * @file body_posture.hpp
 * @brief 浮动基机身姿态全向空间阻抗控制与期望扳手 (6D Spatial Wrench) 接口
 * 
 * 模块定位: Layer 3 动力学控制层 (motor_test_cpp::body_posture)
 * 职责描述:
 *   基于浮动基状态观测器输出的位姿与线/角速度状态，计算使机身保持目标高度、
 *   水平姿态及期望偏航角速度所需施加的 6 维空间力与力矩 (Fx, Fy, Fz, Mx, My, Mz)。
 */

#pragma once

#include <Eigen/Core>

#include "motor_test/estimation/estimator.hpp"

namespace motor_test_cpp {
namespace body_posture {

/**
 * @brief 机身姿态控制参数配置
 */
struct BodyPostureConfig {
    bool   enabled     = true;     ///< 机身姿态全向阻抗闭环总开关

    // ---- 物理惯性参数 ----
    double mass_kg     = 16.0;     ///< 机体标称整机质量 (kg)
    double gravity     = 9.81;     ///< 重力加速度常数 (m/s^2)

    // ---- 坡面重力自适应投影 ----
    bool   gravity_comp    = false;///< 坡面重力自适应分解使能
    double grav_comp_hclip = 0.35; ///< 水平切向重力补偿分量最大比例限制

    // ---- 垂直高度通道阻尼 (fz) ----
    double kd_height  = 0.0;       ///< 垂直线速度阻尼增益 (N*s/m)
    double fz_fb_clip = 60.0;      ///< 垂直反馈力限幅 (N)

    // ---- 姿态环纠偏增益 (Roll / Pitch) ----
    double kp_roll     = 10.0;     ///< 横滚角位置比例增益 (Nm/rad)
    double kd_roll     = 1.0;      ///< 横滚角速度阻尼增益 (Nm*s/rad)
    double kp_pitch    = 10.0;     ///< 俯仰角位置比例增益 (Nm/rad)
    double kd_pitch    = 1.0;      ///< 俯仰角速度阻尼增益 (Nm*s/rad)

    // ---- 航向偏航环增益 (Yaw) ----
    double kp_yaw      = 10.0;     ///< 偏航角位置锁定增益 (Nm/rad)
    double kd_yaw      = 1.0;      ///< 偏航角速度阻尼增益 (Nm*s/rad)
    double ki_yaw      = 0.0;      ///< 偏航误差带泄漏积分增益

    // ---- 输出硬限幅 ----
    double moment_clip = 15.0;     ///< 空间三轴力矩硬限幅阈值 (Nm)
    double fz_min      = 0.0;      ///< 垂直期望合力下限 (N)
    double fz_max      = 300.0;    ///< 垂直期望合力上限 (N)

    // ---- 目标姿态基准 ----
    double roll_des    = 0.0;      ///< 期望机身横滚角 (rad)
    double pitch_des   = 0.0;      ///< 期望机身俯仰角 (rad)
    double yaw_des     = 0.0;      ///< 期望机身初始航向锁角 (rad)
};

/**
 * @brief 期望 6D 空间力系结构体 (Spatial Wrench)
 */
struct WrenchDes {
    Eigen::Vector3d force  = Eigen::Vector3d::Zero();  ///< 机身系合力 [Fx, Fy, Fz]^T (N)
    Eigen::Vector3d moment = Eigen::Vector3d::Zero();  ///< 机身系合力矩 [Mx, My, Mz]^T (Nm)
};

/**
 * @brief 计算浮动基机身期望 6D 空间力系
 * @param cfg 姿态控制器参数
 * @param est 浮动基状态观测器当前时刻输出
 * @param yawrate_cmd 外部期望偏航转向角速度 (rad/s)
 * @param vz_filt_state 垂直速度低通滤波状态 [in/out]
 * @return 期望 6 维空间力与力矩 (机身坐标系)
 */
WrenchDes compute_wrench_des(const BodyPostureConfig& cfg,
                             const estimator::EstimatorOutput& est,
                             double yawrate_cmd,
                             double& vz_filt_state);

}  // namespace body_posture
}  // namespace motor_test_cpp
