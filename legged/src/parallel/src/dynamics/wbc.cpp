// =============================================================================
// 文件: wbc.cpp
// 模块: parallel::wbc (基于抓取矩阵加权伪逆的接触力分配)
// 职责: 结合各腿支撑权重构建 Grasp 矩阵加权最小范数解，并执行摩擦锥正向投影
// =============================================================================

#include "parallel/dynamics/wbc.hpp"

#include <algorithm>
#include <cmath>

namespace parallel {
namespace wbc {

// =============================================================================
// 3×3 反对称矩阵 [v]× → cross(v, F) ≡ skew(v)·F
// =============================================================================
static Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<    0.0, -v.z(),  v.y(),
         v.z(),    0.0, -v.x(),
        -v.y(),  v.x(),    0.0;
    return S;
}

// =============================================================================
// 摩擦锥投影: 把 (fx,fy,fz) 投影到 { fz∈[fz_min,fz_max], √(fx²+fy²) ≤ μ·fz }
// 返回投影后的力, violation_out = 投影前超出摩擦锥的水平力幅度 (N, 诊断)
// =============================================================================
static Eigen::Vector3d project_friction_cone(
    const Eigen::Vector3d& f_in,
    double mu, double fz_min, double fz_max,
    double& violation_out) {

    Eigen::Vector3d f = f_in;
    violation_out = 0.0;

    // fz 钳到 [fz_min, fz_max], 确保不拉地、不穿透
    f.z() = std::clamp(f.z(), fz_min, fz_max);

    // 摩擦锥: 水平力不能超过 μ × 法向力, 否则脚打滑
    const double fxy_max = mu * std::max(0.0, f.z());
    const double fxy_norm = std::hypot(f.x(), f.y());
    if (fxy_norm > fxy_max && fxy_norm > 1e-9) {
        violation_out = fxy_norm - fxy_max;
        const double scale = fxy_max / fxy_norm;
        f.x() *= scale;
        f.y() *= scale;
    }

    return f;
}

// =============================================================================
// solve —— 主入口: 加权 grasp 伪逆 → 摩擦锥投影
// =============================================================================

WbcOutput solve(const WbcConfig& cfg, const WbcInput& inp) {
    WbcOutput out;

    // ---- step 1: 按 support_weight 分离 stance / swing ----
    int stance_idx[N_LEGS];
    int n_stance = 0;
    for (int i = 0; i < N_LEGS; ++i) {
        if (inp.support_weight[i] > cfg.swing_threshold) {
            stance_idx[n_stance++] = i;
        }
    }
    out.n_stance_used = n_stance;

    // 无 stance 腿 (跳跃腾空期): 全零返回, 外部 force_control 填 VMC 阻抗力
    if (n_stance == 0) {
        for (int i = 0; i < N_LEGS; ++i) out.f_body[i].setZero();
        out.success = true;
        return out;
    }

    const int nv = 3 * n_stance;   // 变量维度 = 每 stance 腿 (fx, fy, fz)

    // ---- step 2: 构建 grasp 矩阵 G (6 × nv) ----
    // G · F_stance = [ ΣF_i ] = [ wrench.force  ]
    //                [ Σr_i×F_i ]   [ wrench.moment ]
    //
    // r_i = foot_body[i] — 足端在 body 系的位置, 即 COM → 足端的力矩臂。
    // 注意: body 系原点在机身几何中心 (≈COM), 所以 foot_body 直接是力臂。
    Eigen::MatrixXd G(6, nv);
    G.setZero();
    for (int k = 0; k < n_stance; ++k) {
        const int i  = stance_idx[k];
        const int c0 = 3 * k;
        // 力分量: 每腿力直接累加 → 总力
        G(0, c0 + 0) = 1.0;  G(1, c0 + 1) = 1.0;  G(2, c0 + 2) = 1.0;
        // 力矩分量: r × F → [r]× · F
        const Eigen::Matrix3d S = skew(inp.foot_body[i]);
        G.block<3,3>(3, c0) = S;
    }

    // ---- step 3: 加权对角矩阵 W (nv × nv) ----
    // W_kk = 1 / support_weight_i  →  cost = Fᵀ·W·F = Σ F_i² / support_weight_i
    // support_weight 大 → W 小 → 成本低 → 该腿分配更多力
    // support_weight 小 → W 大 → 成本高 → 该腿力被压到 ~0
    Eigen::VectorXd W_diag(nv);
    for (int k = 0; k < n_stance; ++k) {
        const int i  = stance_idx[k];
        const double sw = std::max(inp.support_weight[i], 1e-3);  // 防除零
        const double w  = 1.0 / sw;
        W_diag(3*k + 0) = w;
        W_diag(3*k + 1) = w;
        W_diag(3*k + 2) = w;
    }

    // ---- step 4: 加权伪逆 ----
    // F = W⁻¹·Gᵀ·(G·W⁻¹·Gᵀ + λ·I)⁻¹·wrench
    //    = W⁻¹·Gᵀ·y,  其中 y 解自 (G·W⁻¹·Gᵀ + λ·I)·y = wrench

    // 4a. W⁻¹·Gᵀ (nv × 6), 再乘 G 得到 6×6
    Eigen::MatrixXd Winv_Gt(nv, 6);
    for (int k = 0; k < nv; ++k) {
        for (int j = 0; j < 6; ++j) {
            Winv_Gt(k, j) = G(j, k) / W_diag(k);          // W⁻¹·Gᵀ
        }
    }
    Eigen::Matrix<double, 6, 6> GWinvGt = G * Winv_Gt;   // G·W⁻¹·Gᵀ

    // 4b. Tikhonov 对角加载: 防 G·W⁻¹·Gᵀ 奇异 (2-leg 下力矩分量可接近 rank-deficient)
    //     力矩对角 ≈ hip² ≈ 0.02-0.15 (vs 力对角 ≈ 2-4), 用 10×λ 补偿量级差。
    //     λ=0.001 → 力阻尼 0.001 (vs ~2, <0.1%), 力矩阻尼 0.01 (vs ~0.13, ~8%)。
    const double lam   = cfg.reg_weight;
    GWinvGt(0,0) += lam;
    GWinvGt(1,1) += lam;
    GWinvGt(2,2) += lam;
    GWinvGt(3,3) += lam * 10.0;    // 力矩分量量级较小, 稍大阻尼防奇异
    GWinvGt(4,4) += lam * 10.0;
    GWinvGt(5,5) += lam * 10.0;

    // 4c. LDLT 解 (G·W⁻¹·Gᵀ + λ·I)·y = wrench
    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt;
    ldlt.compute(GWinvGt);
    if (ldlt.info() != Eigen::Success) {
        for (int i = 0; i < N_LEGS; ++i) out.f_body[i].setZero();
        out.success = false;
        return out;
    }
    Eigen::Vector<double, 6> wrench_vec;
    wrench_vec << inp.wrench.force.x(),  inp.wrench.force.y(),  inp.wrench.force.z(),
                  inp.wrench.moment.x(), inp.wrench.moment.y(), inp.wrench.moment.z();
    const Eigen::Vector<double, 6> y = ldlt.solve(wrench_vec);

    // 4d. F = W⁻¹·Gᵀ·y
    const Eigen::VectorXd F_stance = Winv_Gt * y;
    out.success = true;

    // ---- step 5: 摩擦锥投影 + 填入输出 ----
    // swing 腿置零; stance 腿逐腿投影
    for (int i = 0; i < N_LEGS; ++i) out.f_body[i].setZero();

    out.max_friction_violation = 0.0;
    for (int k = 0; k < n_stance; ++k) {
        const int i = stance_idx[k];
        const Eigen::Vector3d f_raw(F_stance(3*k+0), F_stance(3*k+1), F_stance(3*k+2));
        double viol = 0.0;
        out.f_body[i] = project_friction_cone(f_raw, cfg.mu, cfg.fz_min, cfg.fz_max, viol);
        out.max_friction_violation = std::max(out.max_friction_violation, viol);
    }

    return out;
}

}  // namespace wbc
}  // namespace parallel
  