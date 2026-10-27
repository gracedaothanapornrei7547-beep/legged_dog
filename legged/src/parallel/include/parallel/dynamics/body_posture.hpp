// =============================================================================
// 文件: body_posture.hpp
// 模块: parallel/dynamics
// 职责: 机身姿态/速度闭环——基于estimator状态计算目标6D wrench（WBC第1步）
//
// 控制链: estimator → body_posture → wrench → force_alloc → F_body[4] → force_control → τ
// 等价于WBC QP中formulateBaseAccelTask的reference生成器，这里用PD闭环替代QP求解。
//
// wrench 公式 (body系, z朝上):
//   wrench.force.z  = m·g - kd_height·vz_filt
//   wrench.moment.x = -kp_roll·roll  - kd_roll·ω_x
//   wrench.moment.y = -kp_pitch·pitch - kd_pitch·ω_y
//   wrench.moment.z = -kp_yaw·(yaw-yaw_des) - kd_yaw·(ω_z-yawrate_cmd)
//   姿态项(mx/my/mz)来自IMU恒施加; 高度/速度反馈仅在pos_valid时生效(align chapt2)
//
// 调参要点:
//   - kd_height>2000 → wrench.z触上限 → 腿打滑/电机过热
//   - kp_roll/pitch>200且kd>15 → trot的ω≈±1rad/s放大至±15Nm → 自振, kd_rp<2较稳
//   - yaw_des在startup末latch est.yaw, IMU漂移时机身被强行回拉
//
// 启动期: main内cosine_lerp 0→3s渐入, wrench *= min(1, t/startup_s)
// 关闭方式: --no-body-posture或设所有kp/kd=0
// =============================================================================

#pragma once

#include <Eigen/Core>

#include "parallel/estimation/estimator.hpp"

namespace parallel {
namespace body_posture {

// ============================================================================
// 6 路闭环参数（这里改默认值；CLI --kp-height / --kp-roll 等可覆盖）
// ============================================================================

struct BodyPostureConfig {
    // 短腿5-bar需集中式roll力矩稳定(开源长腿不需要)
    bool   enabled     = true;

    // ---- 姿态力环 A/B 开关 ----
    // ---- 物理 ----
    double mass_kg     = 10.0;     // 机身质量估值（决定 fx/fy 推力比例）
    double gravity     = 9.81;     // 重力加速度

    // ---- 重力补偿 ----
    // true  → F_body = R(roll,pitch)ᵀ·(0,0,mg+damp), 斜坡上水平分力=mg·sinθ 抗下滑
    // false → 纯 body 系 (0,0,mg)
    bool   gravity_comp    = true;
    double grav_comp_hclip = 0.35;  // 水平重力补偿分力上限 = hclip·mg (≈20°坡); 防坏姿态读数→失控水平力

    // ---- 高度通道 (fz) ----
    //   fz = m·g + clamp(−kd_height·vz_filt, ±fz_fb_clip)
    //   速度阻尼: vz=IMU 加速度短时积分+LPF 20ms, 支撑腿"吊着"放机身; 钳 ±fz_fb_clip
    double kd_height  = 0.0;   // 垂直速度阻尼 (N·s/m); 配合20ms vz LPF
    double fz_fb_clip = 60.0;    // 阻尼项上限(N); vz漂移偏置~0.05m/s约9N, 远在钳内

    double kp_roll     = 10.0;    // roll 位置增益 (Nm/rad)
    double kd_roll     = 2.0;      // roll 速度阻尼
    double kp_pitch    = 10.0;    // pitch 位置增益
    double kd_pitch    = 2.0;     

    // ---- yaw 闭环 (mz)：位置 + 速度阻尼 ----
    // 公式: mz = -kp·(yaw - yaw_des) - kd·(ω_z - yawrate_cmd)
    //   yaw_des：main 在 startup 结束时自动 latch est.yaw（=当前航向作零点）
    //   kp_yaw=0 退化为纯 yaw-rate damping（不锁航向，只阻止旋转加速）
    double kp_yaw      = 20.0;     // (Nm/rad)
    double ki_yaw      = 0.0;      // yaw积分增益，消速度耦合稳态yaw漂
    double kd_yaw      = 2.0;      // (Nm·s/rad)

    // ---- 姿态目标(楼梯/地形) ----
    //   力矩环 mx/my 拉向 (roll_des, pitch_des), terrain_adapt 也顶向同一目标
    //   默认 0 = 机身保持水平。单位 rad。
    double roll_des    = 0.0;
    double pitch_des   = 0.0;
    double yaw_des     = 0.0;      // latched at main runtime

    // ---- body系速度跟踪 (fx, fy) ----
    //   vel_body不可靠, 速度环已移除; trot前进靠步态几何, 非此推力

    // ---- 安全限幅 ----
    double fz_min      = 0.0;      // fz >= 0
    double fz_max      = 300.0;    // 单机身 fz 上限
    double fxy_clip    = 15.0;     // 单机身 fx/fy 绝对值上限
    double moment_clip = 25.0;     // 单机身 mx/my/mz 绝对值上限(Nm)
};

// ============================================================================
// 6D wrench: 机身在 body 系下"应该受到的力 + 力矩"
//   force.x/y/z: 机身受力 (N)，body 系
//   moment.x/y/z: 机身受力矩 (Nm)，绕 body x/y/z 轴
// ============================================================================
struct WrenchDes {
    Eigen::Vector3d force {0.0, 0.0, 0.0};   // body 系，z 朝上
    Eigen::Vector3d moment{0.0, 0.0, 0.0};   // body 系，绕 x/y/z
};

// ============================================================================
// 单拍计算 wrench_des
//
//   cfg          : 闭环增益 + 限幅
//   est          : estimator 当前输出（pos_world / rpy / ω_body）
//   yawrate_cmd  : 用户期望 yaw 角速度 (rad/s)
//   vz_filt_state: [in/out] vz 的 20ms LPF 态(调用方持有) —— 喂 kd_height 阻尼
//
// 返回 6D wrench (body 系)。由 force_alloc 分配到各腿。
// strip-to-attitude: 仅保留姿态闭环+重力前馈; vcmd_x/y, pos_valid 等随闭环移除
// ============================================================================
WrenchDes compute_wrench_des(const BodyPostureConfig& cfg,
                              const estimator::EstimatorOutput& est,
                              double yawrate_cmd,
                              double& vz_filt_state);

}  // namespace body_posture
}  // namespace parallel
