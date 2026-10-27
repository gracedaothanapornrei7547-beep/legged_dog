// =============================================================================
// 文件: force_alloc.hpp
// 模块: parallel/dynamics
// 职责: 6D wrench分配到4条腿足端力——启发式闭式分配（WBC第2步）
//
// 控制链: body_posture → wrench → force_alloc → F_body[4] → force_control → τ
// 方法: 按hip位置启发式分摊fz/fxy (~µs, 原OSQP路径2026-06-07已删)
//
// 分配公式 (body系, z朝上):
//   share = 1/n_stance
//   roll/pitch: fz_i = max(0, fz_total·share + sign(hip_y)·mx_des/n ± sign(hip_x)·my_des/n)
//   yaw:       fx_i = fx_total·share + yaw_lambda·(-hip_y)·mz_des
//              fy_i = fy_total·share + yaw_lambda·(+hip_x)·mz_des
//   swing腿 (support_weight≈0): F_body[i]=0
//
// support_weight[i]∈[0,1] 来自contact confidence LPF(50ms), 平滑换相防F跳变
// =============================================================================

#pragma once

#include <Eigen/Core>
#include <array>
#include <string>

#include "parallel/common/pod_types.hpp"           // N_LEGS
#include "parallel/dynamics/body_posture.hpp"           // WrenchDes
#include "parallel/kinematics/leg_calibration.hpp"  // HIP_BODY_XY

namespace parallel {
namespace force_alloc {

// heuristic闭式分配, ~µs, support_weight平滑换相 (原OSQP路径已删)
struct ForceAllocConfig {
    // mz → 水平力的杠杆系数 (1/m)：
    //   fy_i ≈ yaw_lambda · hip_x · mz_des
    //   实际机身 hip_x = 0.20m，所以 yaw_lambda 1.5 让 1 Nm mz → 0.3 N fy
    //   大了 → 转弯力命令大；小了 → yaw 闭环弱
    double yaw_lambda = 1.5;

    // 摩擦系数 μ, 约束|F_xy|≤μ·F_z: 0.5瓷砖, 0.7橡胶+普通地面(默认), 1.0+防滑垫
    //   过大→命令xy力超实际摩擦→软件与真实力失配
    double mu         = 0.7;

    double fz_min     = 0.0;     // 单腿 z 力下限：避免 QP 给负值 (拉脚离地)
    double fz_max     = 300.0;   // 单腿 z 力上限：保护电机和接触冲击
    double fxy_clip   = 600.0;   // 单腿 xy 力上限（仅 heuristic 用，QP 用摩擦锥约束）

    // 摩擦锥剪裁开关：
    //   true  → 按 |F_xy| ≤ μ·F_z 剪裁（推荐，符合物理）
    //   false → 用 fxy_clip 简单 box 剪裁（仅 debug 用）
    bool   apply_friction_cone = true;

    // 摆动腿承重上限: 低于接触判定阈值使腿真卸载抬起
    double swing_support_max = 0.05;
};

// 启发式分配（连续 support_weight 版）
//   support_weight[i] ∈ [0, 1]：腿 i 的支撑权重（来自 contact confidence LPF）
//     1 = 完全支撑（fz 多分配）
//     0 = 完全脱离（fz 不分配）
//     中间 = 切换过渡（按权重比例分摊）
//   wrench_des  : 来自 body_posture::compute_wrench_des
//   hip_body[i] : 髋关节在 body 系的 (x, y, z) 位置
std::array<Eigen::Vector3d, N_LEGS> allocate_heuristic(
    const ForceAllocConfig& cfg,
    const body_posture::WrenchDes& wrench_des,
    const std::array<double, N_LEGS>& support_weight,
    const std::array<Eigen::Vector3d, N_LEGS>& hip_body);

// 统一分配入口（heuristic）。薄封装 allocate_heuristic, 保留供调用方语义清晰。
std::array<Eigen::Vector3d, N_LEGS> allocate(
    const ForceAllocConfig& cfg,
    const body_posture::WrenchDes& wrench_des,
    const std::array<double, N_LEGS>& support_weight,
    const std::array<Eigen::Vector3d, N_LEGS>& hip_body);

}  // namespace force_alloc
}  // namespace parallel
