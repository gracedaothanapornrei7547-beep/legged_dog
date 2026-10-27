// =============================================================================
// 文件:        pod_types.hpp
// 模块:        parallel/common
// 描述:        C++ POD 类型，与 Python runtime/pod_types.py 一一对应。
//              所有矩阵均采用 Eigen::RowMajor（C 序），以便通过 pybind11
//              实现 numpy 视图的零拷贝。形状与数据类型在编译期固定 ——
//              不允许运行时调整大小 —— 从而保证确定性的 ABI 和 RT 安全的
//              静态检查。每个 POD 均具备零初始化默认构造函数和显式
//              copy_from，与 Python 的 .empty() / .copy_from() 对应。
//              仅包含数据，无方法逻辑。
// =============================================================================

#pragma once

#include <Eigen/Core>
#include <cstdint>

namespace parallel {

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
// 命名约定：MatXxY 表示 (X, Y) RowMajor double；VecN 表示 (N,) double；VecNu8 表示 (N,) uint8

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


}  // namespace parallel
