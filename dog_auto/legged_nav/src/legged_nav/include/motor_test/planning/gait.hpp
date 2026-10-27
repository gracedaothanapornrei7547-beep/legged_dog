/**
 * @file gait.hpp
 * @brief 对角步态 (Trot Gait) 期望轨迹解析生成器接口
 * 
 * 模块定位: Layer 2 运动规划层 (motor_test_cpp::gait)
 * 职责描述:
 *   基于周期时间参数与相位偏置生成各支链足端在机身坐标系下的期望位置、
 *   解析速度与解析加速度 (p_des, v_des, a_des)，支持 Raibert 落地外推
 *   与崎岖地形自适应高度调节。
 */

#pragma once

#include <Eigen/Core>
#include <array>

#include "motor_test/common/pod_types.hpp"   // N_LEGS

namespace motor_test_cpp {
namespace gait {

// ============================================================================
// 步态参数（默认值在这里改 → 永久生效；CLI --cycle-s 等可单次覆盖）
// ============================================================================

struct GaitConfig {
    // ── 步态周期(s) ──
    //   短→步频快/前进快, 但 swing 时间挤压→欠跟踪恶化/易绊倒。
    //   长→抗扰动好, 但前进慢。楼梯模式需更慢(0.7s)让探地足端有时间探完台阶。
    double cycle_s     = 0.40;   // roll 0.90°唯一解, 估计器盲靠降步高补
    // ── stance 占比 ∈ (0,1) ──
    //   >0.5 = 双支撑期 = 两条对角腿同时贴地接力, 消除纯 swap 瞬间的脉冲推→pitch 抖从 2.1°→1.1°。
    //   =0.5 = 标准 trot(无双支撑, 对角腿瞬间交换)。
    //   0.55-0.60: 平衡稳 vs 速度的双支撑, 每周期 10-20% 时间 4 腿同时贴地。
    double stance_frac = 0.6;

    // 站姿横向加宽：四脚各往外叉开此距离(m)，加宽支撑底面提升侧向稳定
    double stand_y_widen = 0.03;

    // 立高度 (body 系 z，负=脚在身下方)
    double stance_z    = -0.23;

    // ── 姿态→腿高 地形自适应(斜坡/楼梯: 腿伸长缩短把机身顶到命令角) ──
    //   纯几何: stance_z_i = stance_z − s_pitch · hip_x_i − s_roll · hip_y_i
    //   s_pitch = k ∫ (pitch_des − est.pitch) dt  (慢积分 τ≈0.83s, 只 trot 期)
    //   s_roll  = k ∫ (est.roll − roll_des) dt    (再慢积分, 与 body_posture 共用 des)
    //   符号: pitch 用 (des−est)=纠正(2026-06-21 真机一票定案), roll 用 (est−des)=VMC_Control 移植。
    //   两个积分【非对称】是移植继承, 各自对, 别强行对齐。
    //   ★泄漏(leaky): 纯积分器会把平地小偏置(~0.16°)积到钳位绕死→走时前倾/站立回正。
    //   每拍 ta_s *= (1-dt/τ): τ=2s → 平地稳态 dz<1.5mm, 真斜坡(≥5°) err 大仍积满钳=不丢补偿。
    //   ★安全: 深侧安全(th2 方向), 浅侧=th2 死区侧用更紧的钳。与 ws_z_min_depth/th2 软监督兜底。
    bool   terrain_adapt                = true;
    double terrain_adapt_k              = 1.2;    // 积分增益 1/s (τ=0.83s)。>1.2→τ<cycle 跟承重交接打架
    double terrain_adapt_max_m          = 0.07;   // 单腿偏移上限(m): 深侧(安全方向)
    double terrain_adapt_shallow_max_m  = 0.02;   // 浅侧单独钳(浅+aft=th2 死区方向): 最多+2cm
    double terrain_adapt_leak_s         = 2.0;    // 泄漏时间常数(s): 平地<1.5mm绕+斜坡满补偿

    // ★ step_h / step_x / step_y = 运行时工作值, 命令源(shm nav)每拍覆盖。这里的默认值是死值(仅无命令 fallback)。
    //   来源: step_x/y ← nav 的 Vx/Vy ×(cycle_s·stance_frac)(ShmCommandSource); step_h ← MotionShaping.nav_step_h。
    double step_h      = 0.0;   // ← subs_gait.hpp (step_h_high/low)
    double step_x      = 0.0;   // ← subs_gait.hpp (max_step_x)
    double step_y      = 0.0;   // ← subs_gait.hpp (max_step_y)

    //   领头腿: 行进方向那侧(前进→前腿, sign(hip_x)==sign(step_x))抬升被 5-bar 几何压低。
    //   实测前腿抬升只到尾随腿的 65%/后退 63%。lead_lift_scale 单独多抬补回。
    //   ★新几何 k_ratio 降后最优值会缩(各向异性减弱), 需 tools/j_anisotropy.py 重扫重定。
    double lead_lift_scale = 1.0;

    //   给每腿 step 加切向分量: yaw_turn_scale·yawrate·t_stance·(ẑ×r_hip)
    //   ẑ×r_hip = (-hip_y, hip_x) → 左右腿步向相反 → 机身真转(落脚几何, 非脚打滑)。
    double yaw_turn_scale = -1.0;

    // 每腿相位 (0..1)，顺序与 LEG_NAMES 一致：FL, FR, BL, BR
    //   diagonal trot 默认: FL=BR=0.5（半周期偏移）, FR=BL=0
    //   pace:               FL=BL=0,  FR=BR=0.5（左右两两同相）
    //   bound:              FL=FR=0,  BL=BR=0.5（前后两两同相）
    std::array<double, N_LEGS> phase{ 0.5, 0.0, 0.0, 0.5 };
};

// ============================================================================
// 缓启动：cosine 平滑插值 (s ∈ [0,1])
//   s=0 → a, s=1 → b, 两端一阶导=0
//   用于：起始位姿 → stance_center 的平滑过渡（避免初始扭矩冲击）
// ============================================================================
inline Eigen::Vector3d cosine_lerp(const Eigen::Vector3d& a,const Eigen::Vector3d& b,double s) 
{
    s = std::max(0.0, std::min(1.0, s));
    return a + (b - a) * (0.5 - 0.5 * std::cos(M_PI * s));
}

// ============================================================================
// trot 单腿足端目标 (body 系)
//
// 算法（单条腿一个 cycle 内）：
//   phase = fmod(t / cycle_s + phase_offset, 1)
//
//   if phase < stance_frac:   ── stance 反扫（脚在地上"往后撑"）
//     sp = phase / stance_frac           # 局部进度 0..1
//     bx = (0.5 - sp) * step_x  + raibert_x
//        = +step_x/2 (开始)  →  -step_x/2 (结束)
//     by = leg_y_offset + (0.5 - sp) * step_y + raibert_y
//     bz = stance_z                       # 保持站立高度
//
//   else:                      ── swing 前摆（脚悬空"往前抛"）
//     sp = (phase - stance_frac) / (1 - stance_frac)
//     bx = (-0.5 + sp) * step_x + raibert_x
//        = -step_x/2 (开始)  →  +step_x/2 (结束)  ← 这是 touchdown 位置
//     by = leg_y_offset + (-0.5 + sp) * step_y + raibert_y
//     bz = stance_z + step_h * sin²(π·sp)  # = (1−cos2π·sp)/2 软着陆: sp=0.5 峰值, 端点(抬脚/落地)vz=0 不砸地
//
// raibert_x/y (默认 0)：Raibert 反馈控制 落脚位置偏移
//   touchdown = +step_x/2 + raibert_x （shift 整个 stroke，长度不变）
//   speed continuity 保持（Δp/dt 不受 raibert 影响）
//
// 注：leg_y_offset 来自 cal.stand_y_body（每腿的横向静止偏置 ±0.02m）
//     phase_offset 来自 GaitConfig.phase[i]
// ============================================================================
Eigen::Vector3d trot_body_target(double t,
                                  const GaitConfig& cfg,
                                  double leg_y_offset,
                                  double phase_offset,
                                  double raibert_x = 0.0,
                                  double raibert_y = 0.0,
                                  double hip_x     = 0.0);  // 该腿 hip body x（判领头腿；默认0=不补偿）

// ============================================================================
// trot 单腿足端目标 + 解析速度/加速度（body 系）
//
// 和 trot_body_target 走完全同一条轨迹（position 一模一样），额外返回闭式一阶/
// 二阶导数（body 系），用作 force_control 的 v_des / a_des 前馈，替代对 p_des 的
// 数值微分。
//
// 【为什么要它】数值二阶差分（p→v→a）即便 LPF 也既衰减又相位滞后，正好在抛物线
//   峰值（需要最大减速力）时给错力 → 抬腿欠跟踪（实测仅 ~69%）。闭式导数平滑且
//   相位精确，惯性前馈 m·a_des 才能真正抵消峰值惯性。
//
// 【导数推导】sp_dot ≡ d(sp)/dt：
//   swing 段 sp_dot = 1/((1-stance_frac)·cycle_s)
//     bz = step_h·sin²(π·sp)（=(1−cos2π·sp)/2 软着陆轨迹, 见 gait.cpp:124）
//     vz = step_h·π·sin(2π·sp)·sp_dot         （sp=0/0.5/1 处=0：抬脚/顶点/落地都零速→不砸地）
//     az = step_h·2π²·cos(2π·sp)·sp_dot²       （落地 sp=1 为正=减速到零。注: acc_body 当前无消费者, inertia-ff 已删）
//     vx = step_x·sp_dot, vy = step_y·sp_dot, ax = ay = 0（匀速前摆，仅边界换向）
//   stance 段 sp_dot = 1/(stance_frac·cycle_s)
//     z 恒定 → vz = az = 0；x/y 匀速反扫 → vx = -step_x·sp_dot, ax = 0（同理 y）
//
// 注：raibert_x/y 在一步内被 sticky-latch 锁定为常数，其时间导数≈0，故不计入 v/a。
// ============================================================================
Eigen::Vector3d trot_body_target_d(double t,
                                   const GaitConfig& cfg,
                                   double leg_y_offset,
                                   double phase_offset,
                                   double raibert_x,
                                   double raibert_y,
                                   Eigen::Vector3d& vel_body,
                                   Eigen::Vector3d& acc_body,
                                   double hip_x = 0.0);  // 该腿 hip body x（判领头腿；默认0=不补偿）

// ============================================================================
// 判 stance / swing（基于 gait phase，不看实际接触）
//   用途: 1. swing/stance kp 切换  2. stance 重力分配（gait phase 版本）
//
// 注：force_alloc 用的是 contact_estimator 给的 support_weight (∈ [0,1])，
//     而不是这个 bool。这个函数只供"想知道步态意图"的代码调用。
// ============================================================================
bool is_stance(double t, const GaitConfig& cfg, double phase_offset);

}  // namespace gait
}  // namespace motor_test_cpp
