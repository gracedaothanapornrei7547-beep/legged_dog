/**
 * @file force_alloc.cpp
 * @brief 期望空间力系 (Spatial Wrench) 多足地反力分配算法实现
 * 
 * 核心理论与分配逻辑:
 * 1. 连续支撑权重归一化 (Continuous Support Weight Normalization):
 *    - share_i = support_weight[i] / sum(support_weight)
 * 2. 空间力矩几何杠杆反作用力解耦 (Geometric Moment Decoupling):
 *    - 垂直反力: Fz_i = Fz * share_i + sy * Mx * share_i - sx * My * share_i
 *    - 切向反力: Fx_i = Fx * share_i - yaw_lambda * hy * Mz
 *               Fy_i = Fy * share_i + yaw_lambda * hx * Mz
 * 3. 库仑摩擦锥投影与切向力饱和约束 (Coulomb Friction Cone Projection):
 *    - 确保满足单侧接触与不打滑约束: ||F_xy|| <= min(fxy_clip, mu * Fz)
 */

#include "motor_test/dynamics/force_alloc.hpp"

#include <algorithm>
#include <cmath>

namespace motor_test_cpp {
namespace force_alloc {

std::array<Eigen::Vector3d, N_LEGS> allocate_heuristic(
    const ForceAllocConfig& cfg,
    const body_posture::WrenchDes& wrench,
    const std::array<double, N_LEGS>& support_weight,
    const std::array<Eigen::Vector3d, N_LEGS>& hip_body) {

    std::array<Eigen::Vector3d, N_LEGS> f_body{};
    for (int i = 0; i < N_LEGS; ++i) f_body[i].setZero();

    // 1. 计算总有效支撑权重
    double sum_w = 0.0;
    for (int i = 0; i < N_LEGS; ++i) sum_w += std::max(0.0, support_weight[i]);
    if (sum_w < 1e-6) return f_body;

    // 2. 遍历各支链进行力与力矩几何投影分配
    for (int i = 0; i < N_LEGS; ++i) {
        const double w_i = std::max(0.0, support_weight[i]);
        if (w_i < 1e-6) continue;
        const double share = w_i / sum_w;

        const double hx = hip_body[i].x();
        const double hy = hip_body[i].y();
        const double sy = (hy > 0.0) ? +1.0 : -1.0;
        const double sx = (hx > 0.0) ? +1.0 : -1.0;

        // 垂直力分配: 重力均摊 + 翻滚/俯仰力矩补偿
        const double df_roll  = +sy * wrench.moment.x() * share;
        const double df_pitch = -sx * wrench.moment.y() * share;
        double fz_i = wrench.force.z() * share + df_roll + df_pitch;
        fz_i = std::clamp(fz_i, cfg.fz_min, cfg.fz_max);

        // 水平力分配: 纵横推力均摊 + 航向偏航力矩切向差动
        double fx_i = wrench.force.x() * share + cfg.yaw_lambda * (-hy) * wrench.moment.z();
        double fy_i = wrench.force.y() * share + cfg.yaw_lambda * ( hx) * wrench.moment.z();

        // 库仑摩擦锥约束投影: ||F_xy|| <= mu * F_z
        if (cfg.apply_friction_cone) {
            const double fxy_max = std::min(cfg.fxy_clip, cfg.mu * std::max(0.0, fz_i));
            const double fxy_norm = std::hypot(fx_i, fy_i);
            if (fxy_norm > fxy_max && fxy_norm > 1e-9) {
                const double s = fxy_max / fxy_norm;
                fx_i *= s;
                fy_i *= s;
            }
        } else {
            fx_i = std::clamp(fx_i, -cfg.fxy_clip, cfg.fxy_clip);
            fy_i = std::clamp(fy_i, -cfg.fxy_clip, cfg.fxy_clip);
        }

        f_body[i] = Eigen::Vector3d(fx_i, fy_i, fz_i);
    }
    return f_body;
}

std::array<Eigen::Vector3d, N_LEGS> allocate(
    const ForceAllocConfig& cfg,
    const body_posture::WrenchDes& wrench_des,
    const std::array<double, N_LEGS>& support_weight,
    const std::array<Eigen::Vector3d, N_LEGS>& hip_body) {

    return allocate_heuristic(cfg, wrench_des, support_weight, hip_body);
}

}  // namespace force_alloc
}  // namespace motor_test_cpp
