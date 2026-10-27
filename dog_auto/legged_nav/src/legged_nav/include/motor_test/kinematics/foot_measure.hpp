// foot_measure.hpp —— 运动学层(Layer2): 实测关节 → 足端运动学全量(FK + Jacobian + local/body)。
// =============================================================================
// 【为什么有它】原 dog_control.cpp 的 §2.5(接触/估计) 与 §2.7(力) 各内联了一套
//   "raw→math→FK→Jacobian→local/body(+hip)" 的测量算法(逐字重复两遍)。收进这里一个入口:
//     ① task 变薄, 只调 measure_foot ② 两处共用同一实现, 消除算法重复
//     ③ 为后续"每拍只算一次 FK、估计/力两层共享"的【计算优化】铺路
//        (现仍 §2.5/§2.7 各调一次 = 纯重构、行为不变; 去重是下一小步)。
// 依赖 leg_kinematics(FK/Jacobian) + leg_calibration(raw↔math / local↔body) → 属运动学层。
// =============================================================================
#pragma once

#include <Eigen/Core>

#include "motor_test/kinematics/leg_calibration.hpp"   // robot::LegCalibration (raw↔math / local↔body)
#include "motor_test/kinematics/leg_kinematics.hpp"    // LegKinematics(FK/Jacobian) + FKResult/Mat3x3

namespace motor_test_cpp {
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
}  // namespace motor_test_cpp
