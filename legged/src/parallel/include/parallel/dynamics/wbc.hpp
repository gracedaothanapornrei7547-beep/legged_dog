// wbc.hpp —— 分层任务全身控制 (加权 grasp 伪逆 + 摩擦锥投影)
// =============================================================================
// 所属模块: parallel::wbc (dynamics 流水线, 替代 force_alloc::allocate)
// 职责: 给定机身期望 wrench + 接触状态 + 足端位置, 用 grasp 矩阵加权伪逆最优分配
//       每腿足端力, 然后投影到摩擦锥保证物理可实现。摆动腿不进 QP, 独立走 VMC 阻抗。
// =============================================================================
//
// 【数学原理】(每拍一次, ~0.02ms @ 500Hz)
//
//   step 1. support_weight > swing_threshold → stance (进 QP); 其余 swing
//   step 2. 构建 grasp 矩阵 G (6 × 3·n_stance):
//             G·F_stance = wrench_body
//             G = [     I₃            I₃       ...      I₃     ]
//                 [ [foot_FL]×  [foot_FR]×  ...  [foot_BR]× ]
//             其中 [foot_i]× 是足端 body 坐标的反对称矩阵, 力臂 = foot_body (COM→足端)
//   step 3. 加权最小范数解 (argmin ||F||²_W  s.t. G·F = wrench):
//             F = W⁻¹·Gᵀ·(G·W⁻¹·Gᵀ + λ·I)⁻¹·wrench
//             W = diag(1 / support_weight_i)   — 权重大→力小, swing 腿自然排挤到 ~0
//             λ = reg_weight (Tikhonov, 防 2-leg stance 下 G·W⁻¹·Gᵀ 奇异)
//   step 4. 逐腿摩擦锥投影: fz ∈ [0, fz_max], √(fx²+fy²) ≤ μ·fz
//   step 5. stance 腿返回 WBC 力, swing 腿返回 0 (外部 force_control 补 VMC 阻抗)
//
// 【和 heuristic force_alloc 的核心区别】
//   heuristic: hip 符号 ±1 分摊 fz + roll/pitch 修正, 2-leg stance 下 mx/my "撕扯"自振
//   WBC:       精确 grasp 矩阵, 加权伪逆数学最优, 摩擦锥硬投影保证不打滑
// =============================================================================

#pragma once

#include <array>

#include <Eigen/Core>

#include "parallel/common/pod_types.hpp"              // N_LEGS
#include "parallel/dynamics/body_posture.hpp"         // WrenchDes

namespace parallel {
namespace wbc {

// ============================================================================
// WBC 配置
// ============================================================================

struct WbcConfig {
    // 摩擦锥
    double mu     = 0.5;      // 摩擦系数 (橡胶 ~0.7, 碎石 ~0.4)
    double fz_min = 0.0;      // 最小法向力, 0=不拉地
    double fz_max = 300.0;    // 最大法向力 (N)

    // 正则化 (Tikhonov, 防 pinv 奇异)
    double reg_weight = 0.001;   // 对角加载量, 小值(=信任 G·W⁻¹·Gᵀ), 大值→力分布更均匀但追踪差

    // 门控阈值
    double swing_threshold = 0.05;   // 支撑权重低于该门限判定为摆动相，不参与伪逆接触力分配
};

// ============================================================================
// WBC 输入
// ============================================================================

struct WbcInput {
    body_posture::WrenchDes wrench;                       // 机身期望 wrench (body 系)
    std::array<double, N_LEGS> support_weight;            // 0=swing, 1=full stance
    std::array<Eigen::Vector3d, N_LEGS> foot_body;        // 足端 body 坐标 (来自 FK, 用作力矩臂)
};

// ============================================================================
// WBC 输出
// ============================================================================

struct WbcOutput {
    std::array<Eigen::Vector3d, N_LEGS> f_body;           // 最优足端力 (body 系)
    bool  success = false;                                 // pinv 是否成功
    int   n_stance_used = 0;                               // QP 内的 stance 腿数
    double max_friction_violation = 0.0;                   // 投影前最大摩擦锥违反 (N), 诊断
};

// ============================================================================
// 求解器
// ============================================================================

WbcOutput solve(const WbcConfig& cfg, const WbcInput& inp);

}  // namespace wbc
}  // namespace parallel
