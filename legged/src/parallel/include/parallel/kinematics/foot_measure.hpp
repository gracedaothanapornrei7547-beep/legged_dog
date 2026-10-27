// =============================================================================
// 文件: foot_measure.hpp
// 所属模块: parallel/kinematics
// 职责: 实测关节角(raw)到足端运动学全量(FK+Jacobian+local/body)的统一入口，消除原 dog_control.cpp
//       中 §2.5 与 §2.7 的重复内联算法。依赖 leg_kinematics + leg_calibration。
// =============================================================================
#pragma once

#include <Eigen/Core>

#include "parallel/kinematics/leg_calibration.hpp"   // robot::LegCalibration (raw↔math / local↔body)
#include "parallel/kinematics/leg_kinematics.hpp"    // LegKinematics(FK/Jacobian) + FKResult/Mat3x3

namespace parallel {
namespace kinematics {

// 一条腿一拍的"实测足端运动学"全量。
struct FootMeas {
    Eigen::Vector3d math;     // 实测关节角 (th3, th1, th2)
    Eigen::Vector3d p_local;  // 足端位置 (local 系)
    Eigen::Vector3d v_local;  // 足端速度 (local 系) = J·dq
    Eigen::Vector3d p_body;   // 足端位置 (body 系, 含 hip 偏置)
    Eigen::Vector3d v_body;   // 足端速度 (body 系)
};

// 实测 raw 关节(位置 raw_pos + 速度 raw_vel, 顺序 th3/th1/th2)→ 足端运动学全量。
//   hip_xy = 该腿髋在 body-xy 的偏置(HIP_BODY_XY[leg])。
//   逐字复刻原内联算法, 不改任何系数/顺序 → 纯重构。
inline FootMeas measure_foot(const LegKinematics& kin, const robot::LegCalibration& cal,
                             const Eigen::Vector3d& raw_pos, const Eigen::Vector3d& raw_vel,
                             const Eigen::Vector2d& hip_xy) {
    FootMeas m;
    // raw → math(对称数学约定) → FK
    m.math = cal.raw_to_math_angles(raw_pos[0], raw_pos[1], raw_pos[2]);
    const FKResult fk = kin.forward_kinematics(m.math[0], m.math[1], m.math[2]);
    m.p_local = Eigen::Vector3d(fk.act_x, fk.act_y, fk.act_z);
    // 关节速度 → Jacobian → 足端 local 速度
    const Eigen::Vector3d dq = cal.raw_to_math_velocities(raw_vel[0], raw_vel[1], raw_vel[2]);
    Mat3x3 J;
    kin.jacobian(m.math[0], m.math[1], m.math[2], dq[0], dq[1], dq[2], fk, J, m.v_local);
    // local → body(符号翻转) + hip 偏置
    m.p_body = cal.local_to_body(m.p_local);
    m.p_body.x() += hip_xy[0];
    m.p_body.y() += hip_xy[1];
    m.v_body = cal.local_to_body(m.v_local);
    return m;
}

}  // namespace kinematics
}  // namespace parallel
