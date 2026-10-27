// pod_types.hpp —— C++ POD 套装，与 motor_test/runtime/pod_types.py 1:1 对齐。
//
// 设计原则（与 Python 版严格一致）：
//   1. 所有矩阵都用 Eigen::RowMajor，与 numpy 默认 C-order 一致——pybind11 暴露
//      的 numpy 视图才是零拷贝；如果用 ColMajor，pybind11 会偷偷转置或复制。
//   2. shape/dtype 在编译期固定：Matrix<double, 4, 3> 不是 MatrixXd，禁止运行
//      期 resize。这样 ABI 完全确定，也方便后续做 RT-safety 静态检查。
//   3. 每个 POD 都有 default constructor（零值）+ assignment（trivially-copyable
//      或显式 copy_from）。对应 Python 的 .empty() / .copy_from()。
//   4. 不包含任何方法逻辑——这只是数据；逻辑放在 ControlManager / RobotController。
//
// 与 Python 端的对应关系：
//   pos_body / vel_body / desired_force_body / foot_pos_body : (4,3) double
//   nominal_swing / support_mask / contact                   : (4,)   uint8
//   support_weight / contact_conf / soft_scale               : (4,)   double
//   qpos / qvel / tau_cmd                                    : (12,)  double
//   base_rpy / base_vel / v_cmd_body                         : (3,)   double

#pragma once

#include <Eigen/Core>
#include <cstdint>

namespace motor_test_cpp {

// ============================================================================
// 常量（与 Python N_LEGS / N_JOINTS / N_JOINTS_PER_LEG 一致）
// ============================================================================

constexpr int N_LEGS = 4;
constexpr int N_JOINTS_PER_LEG = 3;
constexpr int N_JOINTS = N_LEGS * N_JOINTS_PER_LEG;  // 12

// ============================================================================
// 矩阵类型别名（统一 RowMajor）
// ============================================================================
//
// 命名约定：MatXxY = (X, Y) RowMajor double；VecN = (N,) double；VecNu8 = (N,) uint8

using Mat4x3   = Eigen::Matrix<double,    N_LEGS,   3, Eigen::RowMajor>;
using Vec4     = Eigen::Matrix<double,    N_LEGS,   1>;
using Vec4u8   = Eigen::Matrix<uint8_t,   N_LEGS,   1>;
using Vec12    = Eigen::Matrix<double,    N_JOINTS, 1>;
using Vec3     = Eigen::Matrix<double,    3,        1>;

// ============================================================================
// Mode —— 顶层控制模式（与 Python IntEnum 同值）
// ============================================================================

enum class Mode : int {
    DISABLE   = 0,  // 全部输出零扭矩，不读 target
    PASSIVE   = 1,  // 零扭矩但保持反馈/遥测
    STAND     = 2,  // 持站姿（忽略 target，内部跑站立目标）
    WALK      = 3,  // 跟随上层 LegTargetBundle + BodyForcePlan
    EMERGENCY = 4   // 立刻进入安全停机
};

// ============================================================================
// SafetyFlag —— telemetry.safety_flags 位掩码（与 Python IntFlag 同值）
// ============================================================================

enum SafetyFlag : int {
    SAFETY_NONE          = 0,
    SAFETY_CMD_STALE     = 1 << 0,  // MotionCommand 超时
    SAFETY_TARGET_STALE  = 1 << 1,  // LegTargetBundle 超时
    SAFETY_PLAN_STALE    = 1 << 2,  // BodyForcePlan 超时
    SAFETY_TORQUE_CLAMP  = 1 << 3,  // 力矩饱和
    SAFETY_FEEDBACK_BAD  = 1 << 4,  // 反馈缺失/异常
    SAFETY_EMERGENCY     = 1 << 5   // 上层触发急停
};


}  // namespace motor_test_cpp
