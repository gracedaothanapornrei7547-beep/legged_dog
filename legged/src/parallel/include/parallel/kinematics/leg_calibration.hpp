// =============================================================================
// 文件: leg_calibration.hpp
// 所属模块: parallel/kinematics
// 职责: 单腿硬件标定与坐标变换——raw↔math 角度/速度/力矩、body↔local 坐标、关节限位。
//       所有常量为 constexpr/inline const（与 robot.py 等价），LegCalibration 为 POD 值类型。
// =============================================================================

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>

#include "parallel/common/pod_types.hpp"   // N_LEGS 等

namespace parallel {
namespace robot {

// ============================================================================
// 第一部分：腿索引 & 名字
// ============================================================================

constexpr int LEG_FL = 0;
constexpr int LEG_FR = 1;
constexpr int LEG_BL = 2;
constexpr int LEG_BR = 3;

inline constexpr std::array<std::string_view, N_LEGS> LEG_NAMES = {
    "fl", "fr", "bl", "br"
};

// 字符串 → 腿索引。无效返回 -1（保持纯函数，不抛异常方便 RT）。
inline int leg_index(std::string_view name) 
{
    for (int i = 0; i < N_LEGS; ++i) 
    {
        if (LEG_NAMES[i] == name) return i;
    }
    return -1;
}

// ============================================================================
// 第二部分：连杆几何
//   legged 改型: l2 0.13→0.16, l3 0.13→0.055, k_ratio 2.000→1.344。
//   目的: 降膝电机受力, 最大可达半径 0.390→0.345, ws_max_reach 0.33。
// ============================================================================

constexpr double LEG_LINK_M         = 0.13;                 // l1 主动杆长 (不变)
constexpr double LEG_L1             = LEG_LINK_M;           // 0.13   第一部分: 髋主动杆 (不变)
constexpr double LEG_L2             = 0.16;                 // ★第二部分: 两并联耦合杆 A→B/C→B (0.13→0.16)
constexpr double LEG_L3             = 0.055;               // ★第三部分: B→足端延长杆 (0.13→0.055)
inline const double LEG_GAMMA_RAD   = 60.0 * M_PI / 180.0;  // 60° 斜轴
constexpr double LEG_BRANCH         = -1.0;                 // 闭链装配支路

// ============================================================================
// 第三部分：数学零位偏置（来自实测，严禁修改）
// ============================================================================
// raw=0 时 joint_math = deadzone (即 raw → math: math = dir·raw + deadzone)
inline const double DEADZONE_TH3_RAD = 0.0;
inline const double DEADZONE_TH1_RAD = -45.32 * M_PI / 180.0;
inline const double DEADZONE_TH2_RAD = 154.49 * M_PI / 180.0;

// ============================================================================
// 第四部分：髋关节在 body 系下的物理安装位置 (X_b, Y_b)
// ============================================================================
inline constexpr std::array<std::array<double, 2>, N_LEGS> HIP_BODY_XY = {{
    {+0.201, +0.102},  // fl
    {+0.201, -0.102},  // fr
    {-0.201, +0.102},  // bl
    {-0.201, -0.102}   // br
}};

// ============================================================================
// 第五部分：默认静站姿态
// ============================================================================
constexpr double DEFAULT_STAND_Z_BODY = -0.23;

// ============================================================================
// 第六部分：关节机械限位（与 Python JointLimits 同值）
// ============================================================================
struct JointLimits {
    double th1_min = -47.0 * M_PI / 180.0;
    double th1_max =  80.0 * M_PI / 180.0;

    double th2_min =  90.0 * M_PI / 180.0;
    double th2_max = 156.5 * M_PI / 180.0;
    double th2_soft = 156.5 * M_PI / 180.0;

    double th3_min = -60.0 * M_PI / 180.0;
    double th3_max =  30.0 * M_PI / 180.0;
    
    double soft_margin  = 1.0 * M_PI / 180.0;
    double ws_min_reach = 0.10;     // 足端工作空间最小可达半径（m）。
    double ws_max_reach = 0.33;     // 新几何腿实际最长 0.345, 0.33 留~15mm余量防 IK 奇异/冻脚
    double ws_y_limit   = 0.20;     // 足端工作空间 y 方向限制（m）。
    // 矢状面直下可达 z_local∈[0.16,0.34]: clamp_workspace 加 z_local≥此 floor 防 swing 压太浅。
    double ws_z_min_depth = 0.16;

    // 工作空间裁剪：xz 平面径向 + y 简单限幅 + z 直下深度下限(非冻结, 钳足端目标)。
    // 从 safety::JointSoftLimits 合并到此 = 限位/运动学单一来源。
    // 逻辑与原 safety 版一致(含 1e-6/1e6 钳位, 镜像 Python clamp_workspace)。
    Eigen::Vector3d clamp_workspace(const Eigen::Vector3d& local_xyz) const {
        double lx = local_xyz.x();
        double ly = local_xyz.y();
        double lz = local_xyz.z();

        const double plane_r = std::hypot(lx, lz);
        if (plane_r > ws_max_reach && plane_r > 1e-9) {
            const double scale = ws_max_reach / plane_r;
            lx *= scale;
            lz *= scale;
        } else if (plane_r > 0.0 && plane_r < ws_min_reach) {
            const double scale = std::min(ws_min_reach / std::max(plane_r, 1e-6), 1e6);
            lx *= scale;
            lz *= scale;
        }

        ly = std::clamp(ly, -ws_y_limit, ws_y_limit);
        lz = std::max(lz, ws_z_min_depth);   // 矢状面直下 z<0.16 不可达, 防 swing 压太浅顶 th2=90
        return {lx, ly, lz};
    }
};
inline constexpr JointLimits STANDARD_JOINT_LIMITS{};

// ============================================================================
// 第七部分：LegCalibration —— 单腿硬件标定 + 坐标变换
// ============================================================================
//
// 与 Python 版的方法一一对应：
//   body_to_local / local_to_body         —— 坐标符号翻转
//   raw_to_math_angles / math_to_raw_angles —— raw = dir·(math - deadzone)
//   raw_to_math_velocities / math_to_raw_velocities  —— 速度不含 deadzone
//   math_torque_to_motor                  —— 力矩按 dir 翻向

class LegCalibration {
public:
    // ---- 基础标识 ----
    int leg_idx;
    // canN 通道已移到 task::LEG_CAN_CHANNEL (hardware_manager.hpp), 此处不再存。

    // ---- 电机 ID ----
    int motor_id_th3;
    int motor_id_th1;
    int motor_id_th2;

    // ---- 电机方向号 dir²=1 ----
    double dir_th3;
    double dir_th1;
    double dir_th2;

    // ---- 数学零位偏置 ----
    double math_deadzone_th3;
    double math_deadzone_th1;
    double math_deadzone_th2;

    // ---- body↔local 坐标符号 ----
    double x_sign;
    double y_sign;
    double z_sign;

    // ---- 默认站立偏移 ----
    double stand_y_body;
    double stand_z_body;
    double branch;

    // ================================================================
    // 坐标映射
    // ================================================================

    Eigen::Vector3d body_to_local(const Eigen::Vector3d& body_xyz) const {
        return {body_xyz.x() * x_sign,
                body_xyz.y() * y_sign,
                body_xyz.z() * z_sign};
    }

    Eigen::Vector3d local_to_body(const Eigen::Vector3d& local_xyz) const {
        // sign² = 1 → 对称
        return {local_xyz.x() * x_sign,
                local_xyz.y() * y_sign,
                local_xyz.z() * z_sign};
    }

    // ================================================================
    // raw ↔ math 角度
    // ================================================================
    //
    // 公式：math = dir·raw + deadzone， raw = (math - deadzone)·dir
    // 返回顺序统一 (th3, th1, th2)，跟 Python 一致

    Eigen::Vector3d raw_to_math_angles(double raw_th3,double raw_th1,double raw_th2) const {
        return {wrap_to_pi(raw_th3 * dir_th3 + math_deadzone_th3),
                wrap_to_pi(raw_th1 * dir_th1 + math_deadzone_th1),
                wrap_to_pi(raw_th2 * dir_th2 + math_deadzone_th2)};
    }

    Eigen::Vector3d math_to_raw_angles(double math_th3,double math_th1,double math_th2) const {
        return {(math_th3 - math_deadzone_th3) * dir_th3,
                (math_th1 - math_deadzone_th1) * dir_th1,
                (math_th2 - math_deadzone_th2) * dir_th2};
    }
 
    // ================================================================
    // raw ↔ math 速度（不含 deadzone）
    // ================================================================

    Eigen::Vector3d raw_to_math_velocities(double w3, double w1, double w2) const {
        return {w3 * dir_th3, w1 * dir_th1, w2 * dir_th2};
    }
    Eigen::Vector3d math_to_raw_velocities(double w3, double w1, double w2) const {
        return {w3 * dir_th3, w1 * dir_th1, w2 * dir_th2};
    }

    // ================================================================
    // 力矩映射 math → motor
    // ================================================================
    Eigen::Vector3d math_torque_to_motor(double tau3, double tau1, double tau2) const {
        return {tau3 * dir_th3, tau1 * dir_th1, tau2 * dir_th2};
    }

    // 电机命令顺序 (th3, th1, th2)
    std::array<int, 3> motor_ids_command_order() const {
        return {motor_id_th3, motor_id_th1, motor_id_th2};
    }

private:
    // 把任意角度压到 (-π, π]，与 Python _wrap_to_pi 一致
    static double wrap_to_pi(double x) {
        return std::atan2(std::sin(x), std::cos(x));
    }
};

// ============================================================================
// 第八部分：四腿默认标定表（与 Python PER_LEG_FIXED_PARAMS 同值）
// ============================================================================

// 工厂：按腿索引构造标定对象
LegCalibration make_default_calibration(int leg_idx);

// 全局表的只读访问
const LegCalibration& get_calibration(int leg_idx);
const LegCalibration& get_calibration(std::string_view leg_name);

}  // namespace robot
}  // namespace parallel
