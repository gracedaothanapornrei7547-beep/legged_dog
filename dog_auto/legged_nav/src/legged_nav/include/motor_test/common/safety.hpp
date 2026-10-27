// safety.hpp —— 数值/力矩保护: clamp/finite_or_zero/clamp_torque/clamp_torque_vector.
// 关节限位已统一归 robot::JointLimits.

#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>

#include "motor_test/kinematics/leg_calibration.hpp"  // STANDARD_JOINT_LIMITS

namespace motor_test_cpp {
namespace safety {

// ============================================================================
// 基础数值保护
// ============================================================================

inline double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

inline double finite_or_zero(double value) {
    return std::isfinite(value) ? value : 0.0;
}

// 与 Python clamp_torque 一致：先洗 NaN，再限幅
inline double clamp_torque(double torque, double max_nm) {
    return clamp(finite_or_zero(torque), -max_nm, max_nm);
}

// 三轴力矩成组限幅（顺序 th3, th1, th2）
inline Eigen::Vector3d clamp_torque_vector(const Eigen::Vector3d& tau, double max_nm) {
    return {clamp_torque(tau.x(), max_nm),
            clamp_torque(tau.y(), max_nm),
            clamp_torque(tau.z(), max_nm)};
}

// TorqueRateLimiter 已删(未接入控制环)。力矩安全只靠 force_control tau_max + NaN 清洗。

// ============================================================================
// JointSoftLimits 已合并进 robot::JointLimits。
// control_chain 统一采用 robot::JointLimits&。
// ============================================================================

}  // namespace safety
}  // namespace motor_test_cpp
