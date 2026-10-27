// leg_kinematics.cpp —— FK / Jacobian / VMC / IK 实现。
//
// 浮点路径与 Python motor_test/config/kinematics.py 严格一致：
//   - 同样的运算顺序（加减乘除的次序影响舍入误差，对拍要求差异 < 1e-12）
//   - 同样的钳位阈值（_clamp 用 1e-4 / 2*l2-1e-4 / 1e-6 等）
//   - 同样的 atan2 调用模式

#include "motor_test/kinematics/leg_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace motor_test_cpp {
namespace kinematics {

namespace {

inline double clamp_d(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

}  // namespace

// 二维两圆求交 —— 与 Python _circle_intersections 严格一致。
// 公开是为了让测试直接对拍；RT 路径不需要直接调（IK 内部用）。
namespace detail {

std::vector<std::pair<double, double>> circle_intersections(
    double x0, double z0, double r0,
    double x1, double z1, double r1)
{
    const double dx = x1 - x0;
    const double dz = z1 - z0;
    const double d  = std::hypot(dx, dz);
    if (d > r0 + r1 + 1e-9 || d < std::fabs(r0 - r1) - 1e-9 || d < 1e-12) {
        return {};
    }
    const double a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d);
    const double h_sq = std::max(0.0, r0 * r0 - a * a);
    const double h = std::sqrt(h_sq);
    const double xm = x0 + a * dx / d;
    const double zm = z0 + a * dz / d;
    const double rx = -dz * (h / d);
    const double rz =  dx * (h / d);
    if (h < 1e-12) {
        return {{xm + rx, zm + rz}};
    }
    return {{xm + rx, zm + rz}, {xm - rx, zm - rz}};
}

}  // namespace detail

// ============================================================================
// 构造函数
// ============================================================================

LegKinematics::LegKinematics(double l1, double l2, double l3, double gamma, double branch)
    : l1_(l1), l2_(l2), l3_(l3), gamma_(gamma), branch_(branch),
      kx_(std::sin(gamma)), kz_(std::cos(gamma)) {}

// ============================================================================
// FK
// ============================================================================

FKResult LegKinematics::forward_kinematics(double th3, double th1, double th2) const {
    FKResult fk{};

    // ⚠ 命名约定（与 Python 历史保留 1:1）：
    //   2D 平面建模时用 (x, y) 命名变量，但**这个 y 实际是 local 系的 z 方向**。
    //   3D 提升后才用 (act_x, act_y, act_z)。所以下面的 fk.ya/yb/yc/y_L 都是
    //   "第二平面轴 = z"。FK 末尾 Rodrigues 把 (x_L, 0, z_L) 旋转到 3D。

    // Step 1: A、C 两点（主动杆 l1 末端，按 th1/th2 在 (x, z) 平面绕原点旋转）
    fk.xa = l1_ * std::cos(th1);
    fk.ya = l1_ * std::sin(th1);   // = z_a
    fk.xc = l1_ * std::cos(th2);
    fk.yc = l1_ * std::sin(th2);   // = z_c

    // Step 2: 闭链交点 B = 圆(A, l2) ∩ 圆(C, l2)
    //
    // 数值钳位约定（与 Python 一致）：
    //   d 下限 1e-4：A=C 重合时 atan2(yc-ya, xc-xa) 退化为 atan2(0,0)=0，
    //     导致 phi 完全失意义；钳到 1e-4 让法线方向至少可定义。
    //   d 上限 2*l2-1e-4：A、C 距离超过 2*l2 时两圆相离无交，h² 会变负。
    //     钳到接近 2*l2 让 h 接近 0（B 接近 AC 中点），FK 输出仍连续。
    //
    // **代价**：在边界区域 FK 输出失真但不会 crash。RT 路径接受这个妥协；
    // 真实工作空间应该被上层的 clamp_workspace + JointLimits 卡在边界外。
    double d = std::hypot(fk.xa - fk.xc, fk.ya - fk.yc);
    d = clamp_d(d, 1e-4, 2.0 * l2_ - 1e-4);
    const double h = std::sqrt(std::max(0.0, l2_ * l2_ - (d / 2.0) * (d / 2.0)));
    const double xm = 0.5 * (fk.xa + fk.xc);
    const double ym = 0.5 * (fk.ya + fk.yc);
    const double phi = std::atan2(fk.yc - fk.ya, fk.xc - fk.xa);

    const double nx = -std::sin(phi);
    const double nz =  std::cos(phi);
    fk.xb = xm + branch_ * h * nx;
    fk.yb = ym + branch_ * h * nz;

    fk.d = d;
    fk.h = h;

    // Step 3: 足端 L = A + k_ratio·(B - A)
    const double k = k_ratio();
    fk.x_L = fk.xa + (fk.xb - fk.xa) * k;
    fk.z_L = fk.ya + (fk.yb - fk.ya) * k;

    // Step 4: 绕 60° 斜轴 Rodrigues 旋转
    //
    // 旋转轴 u = (kx, 0, kz) = (sin γ, 0, cos γ)（γ=60°）。绕 u 转 th3，
    // 用 Rodrigues 公式 R = I + sin(θ)[u]× + (1-cos θ) [u]×²
    //
    // ⚠ **只保存 6 个矩阵元**（R12 / R22 / R32 不存）—— 因为输入向量是
    //    (x_L, 0, z_L)，y 分量恒为 0，y 列乘 0 后不参与运算。
    //    如果未来要让 FK 支持任意 3D 输入（非平面），必须补 R12/R22/R32 计算
    //    并修改下面的 act_x/y/z 公式。
    const double c0 = std::cos(th3);
    const double s0 = std::sin(th3);
    const double v0 = 1.0 - c0;     // versine = 1 - cos θ

    fk.kx = kx_;                    // 缓存到 fk，供 jacobian() 复用（避免重算 sin/cos γ）
    fk.kz = kz_;

    fk.R11 = c0 + kx_ * kx_ * v0;   // R[0][0]
    fk.R13 = kx_ * kz_ * v0;        // R[0][2]
    fk.R21 = kz_ * s0;              // R[1][0]
    fk.R23 = -kx_ * s0;             // R[1][2]
    fk.R31 = kx_ * kz_ * v0;        // R[2][0]  注意 R31 == R13（对称项）
    fk.R33 = c0 + kz_ * kz_ * v0;   // R[2][2]

    // (act_x, act_y, act_z) = R · (x_L, 0, z_L)
    fk.act_x = fk.R11 * fk.x_L + fk.R13 * fk.z_L;
    fk.act_y = fk.R21 * fk.x_L + fk.R23 * fk.z_L;
    fk.act_z = fk.R31 * fk.x_L + fk.R33 * fk.z_L;

    return fk;
}

// ============================================================================
// Jacobian
// ============================================================================

void LegKinematics::jacobian(double th3, double th1, double th2,
                             double w3,  double w1,  double w2,
                             const FKResult& fk,
                             Mat3x3& J_out,
                             Eigen::Vector3d& act_v_out) const {
    (void)th3;  // th3 不直接出现在偏导里——通过 fk 中的 R 表达

    // A、C 对主动角的偏导
    const double J_a_11 = -l1_ * std::sin(th1);  // ∂xa/∂th1
    const double J_a_21 =  l1_ * std::cos(th1);  // ∂ya/∂th1
    const double J_c_12 = -l1_ * std::sin(th2);  // ∂xc/∂th2
    const double J_c_22 =  l1_ * std::cos(th2);  // ∂yc/∂th2

    const double r_AB_x = fk.xb - fk.xa;
    const double r_AB_y = fk.yb - fk.ya;
    const double r_CB_x = fk.xb - fk.xc;
    const double r_CB_y = fk.yb - fk.yc;

    // det_M = [AB] × [CB]（2D 叉积，标量）—— 这是闭链约束矩阵的行列式。
    //
    // **det_M ≈ 0 = 闭链奇异姿态**：A、B、C 三点共线（B 在 AC 延长线上）。
    // 此时主动角扰动经闭链放大到无穷大，真实 Jacobian 发散。
    //
    // 实物上这个奇异点只在工作空间边缘出现（A、C 几乎对齐）。RT 路径不能
    // crash 不能 NaN，所以强制 |det_M| ≥ 1e-6——给一个有限但**不可信**的
    // Jacobian。后果：奇异点附近 VMC 力矩可能很大但方向不准；由 Step 5
    // 安全四层（硬限幅 + 变化率）兜底防止打到电机。
    //
    // TODO: 若实测频繁触发此分支，加 telemetry 计数报告上来。
    double det_M = r_AB_x * r_CB_y - r_AB_y * r_CB_x;
    if (std::fabs(det_M) < 1e-6) {
        det_M = (det_M >= 0.0) ? 1e-6 : -1e-6;
    }

    const double N11 = r_AB_x * J_a_11 + r_AB_y * J_a_21;
    const double N22 = r_CB_x * J_c_12 + r_CB_y * J_c_22;

    const double Jk11 = ( r_CB_y / det_M) * N11;
    const double Jk12 = (-r_AB_y / det_M) * N22;
    const double Jk21 = (-r_CB_x / det_M) * N11;
    const double Jk22 = ( r_AB_x / det_M) * N22;

    const double k = k_ratio();
    const double J11_2d = (1.0 - k) * J_a_11 + k * Jk11;
    const double J21_2d = (1.0 - k) * J_a_21 + k * Jk21;
    const double J12_2d = k * Jk12;
    const double J22_2d = k * Jk22;

    // th3 列 = u × p（斜轴角速度产生的足端切向速度）
    //   u = (kx, 0, kz)  斜轴单位向量
    //   p = (act_x, act_y, act_z)  当前足端 3D 位置
    //   u × p = ( -kz·act_y,  kz·act_x - kx·act_z,  kx·act_y )
    // 由 ω_axis = th3·u 推出 ∂p/∂th3 = u × p（绕固定轴旋转的导数）。
    const double J11_3d = -fk.kz * fk.act_y;
    const double J21_3d =  fk.kz * fk.act_x - fk.kx * fk.act_z;
    const double J31_3d =  fk.kx * fk.act_y;

    // 2D → 3D 映射
    const double J12_3d = fk.R11 * J11_2d + fk.R13 * J21_2d;
    const double J22_3d = fk.R21 * J11_2d + fk.R23 * J21_2d;
    const double J32_3d = fk.R31 * J11_2d + fk.R33 * J21_2d;

    const double J13_3d = fk.R11 * J12_2d + fk.R13 * J22_2d;
    const double J23_3d = fk.R21 * J12_2d + fk.R23 * J22_2d;
    const double J33_3d = fk.R31 * J12_2d + fk.R33 * J22_2d;

    J_out << J11_3d, J12_3d, J13_3d,
             J21_3d, J22_3d, J23_3d,
             J31_3d, J32_3d, J33_3d;

    act_v_out.x() = J11_3d * w3 + J12_3d * w1 + J13_3d * w2;
    act_v_out.y() = J21_3d * w3 + J22_3d * w1 + J23_3d * w2;
    act_v_out.z() = J31_3d * w3 + J32_3d * w1 + J33_3d * w2;
}

// ============================================================================
// VMC
// ============================================================================

VMCResult LegKinematics::vmc_control(double des_x, double des_y, double des_z,
                                     const FKResult& fk,
                                     const Mat3x3& J,
                                     const Eigen::Vector3d& act_v,
                                     const VMCParams& vmc) {
    VMCResult r{};
    r.err_x = des_x - fk.act_x;
    r.err_y = des_y - fk.act_y;
    r.err_z = des_z - fk.act_z;

    r.Fx = vmc.kp_x * r.err_x - vmc.kd_x * act_v.x() + vmc.f_ff_x;
    r.Fy = vmc.kp_y * r.err_y - vmc.kd_y * act_v.y() + vmc.f_ff_y;
    r.Fz = vmc.kp_z * r.err_z - vmc.kd_z * act_v.z() + vmc.f_ff_z;

    // τ = JᵀF —— 注意顺序：tau3 用 J 的第 1 列，tau1 用第 2 列，tau2 用第 3 列
    r.tau3 = J(0, 0) * r.Fx + J(1, 0) * r.Fy + J(2, 0) * r.Fz;
    r.tau1 = J(0, 1) * r.Fx + J(1, 1) * r.Fy + J(2, 1) * r.Fz;
    r.tau2 = J(0, 2) * r.Fx + J(1, 2) * r.Fy + J(2, 2) * r.Fz;
    return r;
}

// ============================================================================
// IK 工具：Rodrigues
// ============================================================================

Mat3x3 LegKinematics::rot_tilt_axis(double th3) const {
    const double c = std::cos(th3);
    const double s = std::sin(th3);
    const double v = 1.0 - c;
    Mat3x3 R;
    R << c + kx_*kx_*v,    -kz_*s,             kx_*kz_*v,
         kz_*s,              c,                -kx_*s,
         kx_*kz_*v,          kx_*s,             c + kz_*kz_*v;
    return R;
}

// ============================================================================
// IK —— 多支路求解 + FK 回代验证
// ============================================================================

std::optional<Eigen::Vector3d> LegKinematics::inverse_kinematics(
    double des_x, double des_y, double des_z,
    double th1_min, double th1_max,
    double th2_min, double th2_max,
    double th3_min, double th3_max) const
{
    // 越界量: 角度落在 [lo,hi] 外多少 rad (内则 0)。供 cost 重罚, 选限位内支。
    auto oob = [](double v, double lo, double hi) {
        return std::max(0.0, lo - v) + std::max(0.0, v - hi);
    };
    // Step 1: 解 th3 使逆旋转后 y ≈ 0
    // 推导：R(-th3) 的第 1 行 = [-kz·s, c, kx·s]
    //   q_y = -kz·s·des_x + c·des_y + kx·s·des_z = 0
    //   ⇒ th3 = atan2(des_y, kz·des_x - kx·des_z)
    // 两候选解（tan 周期）：atan2(A,B) 和 atan2(-A,-B)
    // 与 Python motor_test/config/kinematics.py 修正后的实现严格一致
    const double A = des_y;
    const double B = kz_ * des_x - kx_ * des_z;
    const double th3_candidates[2] = {
        std::atan2(A, B),
        std::atan2(-A, -B),
    };

    std::optional<Eigen::Vector3d> best;
    double best_cost = 1e18;

    for (double th3_c : th3_candidates) {
        const Mat3x3 R_inv = rot_tilt_axis(-th3_c);
        const double qx = R_inv(0, 0) * des_x + R_inv(0, 1) * des_y + R_inv(0, 2) * des_z;
        const double qy = R_inv(1, 0) * des_x + R_inv(1, 1) * des_y + R_inv(1, 2) * des_z;
        const double qz = R_inv(2, 0) * des_x + R_inv(2, 1) * des_y + R_inv(2, 2) * des_z;
        const double x_L = qx;
        const double z_L = qz;

        auto A_candidates = detail::circle_intersections(0.0, 0.0, l1_, x_L, z_L, l2_ + l3_);
        for (const auto& [ax, az] : A_candidates) {
            const double lam = l2_ / (l2_ + l3_);
            const double bx = ax + lam * (x_L - ax);
            const double bz = az + lam * (z_L - az);

            auto C_candidates = detail::circle_intersections(0.0, 0.0, l1_, bx, bz, l2_);
            for (const auto& [cx, cz] : C_candidates) {
                const double th1_c = std::atan2(az, ax);
                const double th2_c = std::atan2(cz, cx);

                const FKResult fk_chk = forward_kinematics(th3_c, th1_c, th2_c);
                // C++17 std::hypot 三参数版与 Python math.hypot(a,b,c) 算法同源
                // （都做 max-scaling 防 over/underflow），跨语言 bit 差异 ≤ 1 ULP。
                const double err = std::hypot(fk_chk.act_x - des_x,
                                              fk_chk.act_y - des_y,
                                              fk_chk.act_z - des_z);
                // ★限位惩罚: 任一关节越界 → cost 重罚(×1000)，保证有限位内支时必选它，
                //   杜绝 th3≈±180 / th2 负镜像支。无限位内支时退化为选"最接近限位"的解。
                const double pen = oob(th1_c, th1_min, th1_max)
                                 + oob(th2_c, th2_min, th2_max)
                                 + oob(th3_c, th3_min, th3_max);
                const double cost = err + 10.0 * std::fabs(qy) + 1000.0 * pen;
                if (cost < best_cost) {
                    best_cost = cost;
                    best = Eigen::Vector3d{th3_c, th1_c, th2_c};
                }
            }
        }
    }
    return best;
}

}  // namespace kinematics
}  // namespace motor_test_cpp
