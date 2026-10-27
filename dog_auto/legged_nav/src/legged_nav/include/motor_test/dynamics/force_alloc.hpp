/**
 * @file force_alloc.hpp
 * @brief 期望空间力系 (Spatial Wrench) 多足地反力分配算法定义
 * 
 * 模块定位: Layer 3 动力学控制层 (motor_test_cpp::force_alloc)
 * 职责描述:
 *   将机身浮动基期望的 6 维空间力系 (Fx, Fy, Fz, Mx, My, Mz) 解耦并分配至四条支链足端，
 *   结合支撑连续权重因子，施加单侧接触单向受力限制与库仑摩擦锥投影。
 */

#pragma once

#include <Eigen/Core>
#include <array>
#include <string>

#include "motor_test/common/pod_types.hpp"
#include "motor_test/dynamics/body_posture.hpp"
#include "motor_test/kinematics/leg_calibration.hpp"

namespace motor_test_cpp {
namespace force_alloc {

/**
 * @brief 地反力分配算法参数配置
 */
struct ForceAllocConfig {
    double yaw_lambda = 1.5;             ///< 航向力矩转化至水平足端推力的几何杠杆常数 (1/m)
    double mu         = 0.7;             ///< 支撑面库仑摩擦系数 (用于摩擦锥约束 ||F_xy|| <= mu * F_z)
    double fz_min     = 0.0;             ///< 单腿垂直法向力下限 (保证非拉力单侧接触) (N)
    double fz_max     = 300.0;           ///< 单腿垂直法向力上限 (N)
    double fxy_clip   = 600.0;           ///< 单腿水平剪切力硬限幅 (N)
    bool   apply_friction_cone = true;   ///< 是否开启库仑摩擦锥投影
    double swing_support_max   = 0.05;   ///< 摆动相最大允许分配的残余支撑力权重 (确保彻底卸载)
};

/**
 * @brief 启发式闭式几何力矩分配
 * @param cfg 分配参数配置
 * @param wrench 机身期望 6D 空间扳手
 * @param support_weight 四足连续支撑权重数组 [0.0, 1.0]
 * @param hip_body 各腿髋关节在机身坐标系下的安装几何中心位置
 * @return 各腿在机身坐标系下的分配反作用力 F_body (N)
 */
std::array<Eigen::Vector3d, N_LEGS> allocate_heuristic(
    const ForceAllocConfig& cfg,
    const body_posture::WrenchDes& wrench,
    const std::array<double, N_LEGS>& support_weight,
    const std::array<Eigen::Vector3d, N_LEGS>& hip_body);

/**
 * @brief 接触力分配统一调用接口
 */
std::array<Eigen::Vector3d, N_LEGS> allocate(
    const ForceAllocConfig& cfg,
    const body_posture::WrenchDes& wrench_des,
    const std::array<double, N_LEGS>& support_weight,
    const std::array<Eigen::Vector3d, N_LEGS>& hip_body);

}  // namespace force_alloc
}  // namespace motor_test_cpp
