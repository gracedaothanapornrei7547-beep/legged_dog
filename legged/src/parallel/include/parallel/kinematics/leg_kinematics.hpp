// =============================================================================
// 文件: leg_kinematics.hpp
// 所属模块: parallel/kinematics
// 职责: 单腿运动学引擎——th3 斜轴(60°) + th1/th2 平面五杆闭链的 FK/Jacobian/IK/VMC。
//       足端 L = A + k_ratio·(B-A)，k_ratio = (l2+l3)/l2 ≈ 1.34（l2=0.16, l3=0.055）。
//       FKResult/VMCParams/VMCResult 与 Python kinematics.py 严格对齐。
// =============================================================================

#pragma once

#include <Eigen/Core>
#include <optional>
#include <utility>
#include <vector>

namespace parallel {
namespace kinematics {

namespace detail {
// 二维两圆求交，公开仅为测试用——RT 路径不需要直接调。
// 与 parallel/config/kinematics.py:_circle_intersections 浮点路径一致。
std::vector<std::pair<double, double>> circle_intersections(
    double x0, double z0, double r0,
    double x1, double z1, double r1);
}  // namespace detail


// 用 RowMajor 3×3 —— pybind11 暴露给 numpy 时需要 C-order
using Mat3x3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

// ============================================================================
// FKResult —— 正运动学输出（含 Jacobian 复用的中间量）
// ============================================================================

struct FKResult {
    // 平面闭链中间量
    double xa, ya;
    double xb, yb;
    double xc, yc;
    double x_L, z_L;
    double d, h;

    // 3D 足端位置（local 系）
    double act_x, act_y, act_z;

    // 斜轴单位向量分量（Jacobian 第一列复用）
    double kx, kz;

    // 旋转矩阵分量（2D→3D 用，没有 R12/R22/R32 因为输入 y=0）
    double R11, R13;
    double R21, R23;
    double R31, R33;
};

// ============================================================================
// VMCParams / VMCResult
// ============================================================================

struct VMCParams {
    double kp_x = 1200.0, kd_x = 12.0;
    double kp_y = 1000.0, kd_y = 10.0;
    double kp_z = 1500.0, kd_z = 15.0;
    double f_ff_x = 0.0, f_ff_y = 0.0, f_ff_z = 0.0;
};

struct VMCResult {
    double err_x, err_y, err_z;
    double Fx, Fy, Fz;
    double tau3, tau1, tau2;
};

// ============================================================================
// LegKinematics —— 单腿运动学引擎
// ============================================================================

class LegKinematics {
public:
    LegKinematics(double l1, double l2, double l3, double gamma, double branch);

    // ---- 派生量 ----
    double k_ratio() const { return (l2_ + l3_) / l2_; }
    double l1() const { return l1_; }
    double l2() const { return l2_; }
    double l3() const { return l3_; }
    double gamma() const { return gamma_; }
    double branch() const { return branch_; }
    double kx() const { return kx_; }
    double kz() const { return kz_; }

    // ---- FK ----
    FKResult forward_kinematics(double th3, double th1, double th2) const;

    // ---- Jacobian ----
    // 返回 (J_3d 3×3, act_v 3D) —— act_v 为足端速度
    void jacobian(double th3, double th1, double th2,
                  double w3,  double w1,  double w2,
                  const FKResult& fk,
                  Mat3x3& J_out, Eigen::Vector3d& act_v_out) const;

    // ---- VMC ----
    static VMCResult vmc_control(double des_x, double des_y, double des_z,
                                 const FKResult& fk,
                                 const Mat3x3& J,
                                 const Eigen::Vector3d& act_v,
                                 const VMCParams& params);

    // ---- IK ----
    // 返回 std::nullopt 表示无解；返回 (th3, th1, th2)。
    // 限位感知: 传关节限位 [th*_min, th*_max] → 候选支里优先选落在限位内的解
    //   (越界支在 cost 里重罚), 杜绝 th3≈±180 / th2 镜像支这种物理不可达解。
    //   默认 ±1e9 = 不偏好(老行为)。control_chain 传真限位; 这是去掉 joints_in_range 软限位后
    //   保证 IK 永远吐合法支的唯一防线。
    std::optional<Eigen::Vector3d> inverse_kinematics(
        double des_x, double des_y, double des_z,
        double th1_min = -1e9, double th1_max = 1e9,
        double th2_min = -1e9, double th2_max = 1e9,
        double th3_min = -1e9, double th3_max = 1e9) const;

private:
    double l1_, l2_, l3_, gamma_, branch_;
    double kx_, kz_;

    // Rodrigues 旋转（绕斜轴）
    Mat3x3 rot_tilt_axis(double th3) const;
};

}  // namespace kinematics
}  // namespace parallel
