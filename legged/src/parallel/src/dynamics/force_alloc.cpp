// =============================================================================
// 文件: force_alloc.cpp
// 模块: parallel::force_alloc (机身接触力分配)
// 职责: 将躯干 6D 期望合力与合力矩 (Wrench) 解析分配至各着地腿足端
//
// 算法原理 (Heuristic Force Allocation):
//   1. 支撑权重归一化:
//      share_i = support_weight[i] / Σ_j support_weight[j]
//   2. 法向支撑力与姿态力矩解耦解算:
//      fz_i = Wrench.fz * share_i + (sy * Wrench.mx * share_i) + (-sx * Wrench.my * share_i)
//      利用左右/前后力矩臂几何符号 (sx, sy) 补偿横滚 (Roll) 与俯仰 (Pitch) 姿态误差。
//   3. 偏航转矩解算:
//      fx_i = Wrench.fx * share_i + yaw_lambda * (-hy) * Wrench.mz
//      fy_i = Wrench.fy * share_i + yaw_lambda * ( hx) * Wrench.mz
//   4. 摩擦锥硬约束投影:
//      sqrt(fx_i^2 + fy_i^2) <= mu * fz_i, 确保地面反作用力处于库仑摩擦锥内，避免打滑。
// =============================================================================

#include "parallel/dynamics/force_alloc.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace parallel {
namespace force_alloc {

// =============================================================================
// allocate_heuristic —— 启发式按 hip 位置分摊
// =============================================================================
//
// 公式（body 系）：
//   share_i = support_weight[i] / Σ_j support_weight[j]    （加权分摊比例）
//
//   fz_i = wrench.fz · share_i
//        + (+sy · wrench.mx · share_i)    ← roll 修正：左腿 +, 右腿 -
//        + (-sx · wrench.my · share_i)    ← pitch 修正：前腿 -my, 后腿 +my
//
//   fx_i = wrench.fx · share_i + yaw_lambda · (-hy) · wrench.mz
//   fy_i = wrench.fy · share_i + yaw_lambda · ( hx) · wrench.mz
//
// 启发式的局限：
//   2-leg stance（trot）下，2 条对角腿无法同时独立满足 fz + mx + my
//   → 强行分配会让 mx/my 命令"撕扯"两条腿 → 自振
//   → 这种情况推荐用 QP
// =============================================================================

std::array<Eigen::Vector3d, N_LEGS> allocate_heuristic(
    const ForceAllocConfig& cfg,
    const body_posture::WrenchDes& wrench,
    const std::array<double, N_LEGS>& support_weight,
    const std::array<Eigen::Vector3d, N_LEGS>& hip_body) {

    std::array<Eigen::Vector3d, N_LEGS> f_body{};
    for (int i = 0; i < N_LEGS; ++i) f_body[i].setZero();

    // ===== 步骤 1: 算总权重 =====
    // 全 swing 时 sum_w ≈ 0 → 直接返回零力（机身不受任何力，会自由下落）
    double sum_w = 0.0;
    for (int i = 0; i < N_LEGS; ++i) sum_w += std::max(0.0, support_weight[i]);
    if (sum_w < 1e-6) return f_body;

    // ===== 步骤 2: 每腿分配 =====
    for (int i = 0; i < N_LEGS; ++i) {
        const double w_i = std::max(0.0, support_weight[i]);
        if (w_i < 1e-6) continue;                  // 该腿权重太小 → 跳过（保持 0）
        const double share = w_i / sum_w;          // 该腿分摊比例

        // ---- hip 位置符号 ----
        // 用于决定该腿在 roll/pitch 中的"贡献方向"
        const double hx = hip_body[i].x();         // 前后位置（FL/FR > 0, BL/BR < 0）
        const double hy = hip_body[i].y();         // 左右位置（FL/BL > 0, FR/BR < 0）
        const double sy = (hy > 0.0) ? +1.0 : -1.0;  // 左 +1, 右 -1
        const double sx = (hx > 0.0) ? +1.0 : -1.0;  // 前 +1, 后 -1

        // ---- fz: 总重力均摊 + roll/pitch 修正 ----
        //
        // 物理推导：要产生 mx > 0（机身绕 +x 转 = 左侧抬高），需要左腿 fz↑、右腿 fz↓
        //   即 Δfz_i = +sy · mx · share_i  （左腿 sy=+1 → Δfz>0; 右腿 sy=-1 → Δfz<0）
        //
        // 同理 my > 0（机身绕 +y 转 = 头抬起）需要前腿 fz↓、后腿 fz↑
        //   即 Δfz_i = -sx · my · share_i  （前腿 sx=+1 → Δfz<0; 后腿 sx=-1 → Δfz>0）
        const double df_roll  = +sy * wrench.moment.x() * share;
        const double df_pitch = -sx * wrench.moment.y() * share;
        double fz_i = wrench.force.z() * share + df_roll + df_pitch;
        fz_i = std::clamp(fz_i, cfg.fz_min, cfg.fz_max);

        // ---- fx, fy: 均摊 + yaw 转矩水平投影 ----
        //
        // 物理：要产生 mz（yaw 转矩），靠每条腿的水平力 (fx,fy) × hip 位置 (hx,hy) 凑：
        //   m_z = sum_i (hx_i·fy_i - hy_i·fx_i)
        //
        // 想 m_z > 0（机身绕 +z 转 = 逆时针）：
        //   fx_i ∝ -hy_i · mz   （hy>0 左腿，给负 fx 推后；hy<0 右腿，给正 fx 推前）
        //   fy_i ∝ +hx_i · mz   （hx>0 前腿，给正 fy 推左；hx<0 后腿，给负 fy 推右）
        //
        // yaw_lambda 是杠杆系数（实际 hip 距 0.2m，1/m 量级）
        double fx_i = wrench.force.x() * share + cfg.yaw_lambda * (-hy) * wrench.moment.z();
        double fy_i = wrench.force.y() * share + cfg.yaw_lambda * ( hx) * wrench.moment.z();

        // ---- 摩擦锥剪裁: |F_xy| ≤ μ · F_z ----
        //
        // 物理：地面对脚的最大摩擦力 = μ × 法向压力
        //   水平力分量超过摩擦极限就会打滑
        //   提前剪裁让命令物理可实现，避免脚滑后机身失稳
        if (cfg.apply_friction_cone) {
            const double fxy_max = std::min(cfg.fxy_clip, cfg.mu * std::max(0.0, fz_i));
            const double fxy_norm = std::hypot(fx_i, fy_i);
            if (fxy_norm > fxy_max && fxy_norm > 1e-9) {
                // 等比例缩放保持方向
                const double s = fxy_max / fxy_norm;
                fx_i *= s;
                fy_i *= s;
            }
        } else {
            // 简单 box 剪裁（不严谨，仅 debug 用）
            fx_i = std::clamp(fx_i, -cfg.fxy_clip, cfg.fxy_clip);
            fy_i = std::clamp(fy_i, -cfg.fxy_clip, cfg.fxy_clip);
        }

        f_body[i] = Eigen::Vector3d(fx_i, fy_i, fz_i);
    }
    return f_body;
}

// =============================================================================
// allocate —— 统一分配入口（heuristic 闭式; OSQP 路径已删, 见 force_alloc.hpp）
// =============================================================================

std::array<Eigen::Vector3d, N_LEGS> allocate(
    const ForceAllocConfig& cfg,
    const body_posture::WrenchDes& wrench_des,
    const std::array<double, N_LEGS>& support_weight,
    const std::array<Eigen::Vector3d, N_LEGS>& hip_body) {

    return allocate_heuristic(cfg, wrench_des, support_weight, hip_body);
}

}  // namespace force_alloc
}  // namespace parallel
