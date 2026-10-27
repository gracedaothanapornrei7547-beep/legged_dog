/**
 * @file gait.cpp
 * @brief 对角步态 (Trot Gait) 期望轨迹解析生成器实现
 * 
 * 核心理论:
 * 1. 相位时钟解算 (Phase Sequencer):
 *    - phase = fmod(t / cycle_s + phase_offset, 1.0)
 * 2. 支撑相正弦/线性反扫 (Stance Phase Retraction):
 *    - 维持名义机身高度，足端在机身坐标系下相对反向匀速后扫
 * 3. 摆动相摆线/正弦平方软着陆轨迹 (Swing Soft-touchdown Trajectory):
 *    - 垂直轨迹采用正弦平方提升曲线: bz = stance_z + step_h * sin^2(pi * sp)
 *    - 起始与着陆端点垂直速度、加速度连续为零 (vz=0, az=0)，最大程度消除落地冲击
 * 4. Raibert 落地外推补偿:
 *    - 结合线速度偏差与捕获点动态调整落脚点偏移
 */

#include "motor_test/planning/gait.hpp"

#include <cmath>

namespace motor_test_cpp {
namespace gait {

// 单腿足端目标（body 系）—— 薄封装，position 由 trot_body_target_d 单一来源算出。
Eigen::Vector3d trot_body_target(double t,
                                  const GaitConfig& cfg,
                                  double leg_y_offset,
                                  double phase_offset,
                                  double raibert_x,
                                  double raibert_y,
                                  double hip_x) {
    Eigen::Vector3d vel_unused, acc_unused;
    return trot_body_target_d(t, cfg, leg_y_offset, phase_offset,
                              raibert_x, raibert_y, vel_unused, acc_unused, hip_x);
}

// 单腿足端目标 + 闭式速度/加速度（body 系）。position 与旧 trot_body_target 逐字节相同；
// v/a 是同一条轨迹的解析导数（见 gait.hpp 头部推导），给 force_control 当前馈用。
Eigen::Vector3d trot_body_target_d(double t,
                                   const GaitConfig& cfg,
                                   double leg_y_offset,
                                   double phase_offset,
                                   double raibert_x,
                                   double raibert_y,
                                   Eigen::Vector3d& vel_body,
                                   Eigen::Vector3d& acc_body,
                                   double hip_x) {
    // ===== 步骤 1: 算当前 phase =====
    // t 从 trot 起点开始计时。每条腿有自己的 phase_offset
    //   diagonal trot: FL=BR=0.5, FR=BL=0  → 对角腿同步
    // fmod 让 phase 在 [0, 1) 内循环
    const double phase = std::fmod(t / cfg.cycle_s + phase_offset, 1.0);

    double bx, by, bz;
    double vx, vy, vz;
    double ax, ay, az;

    // ===== 步骤 2: stance vs swing 分支 =====
    if (phase < cfg.stance_frac) {

        // ---- stance 期：脚在地上反扫 ----
        // sp ∈ [0, 1)，把 phase 在 stance 段内归一化
        const double sp     = phase / cfg.stance_frac;
        const double sp_dot = 1.0 / (cfg.stance_frac * cfg.cycle_s);

        // x: 从 +step_x/2 (sp=0) 线性反扫到 -step_x/2 (sp=1)
        //   raibert_x 整体 shift（保持 stroke 长度）
        bx = (0.5 - sp) * cfg.step_x          + raibert_x;

        // y: 同理，但从 leg_y_offset 出发
        by = leg_y_offset + (0.5 - sp) * cfg.step_y + raibert_y;

        // z: 保持站立高度，脚踩地
        bz = cfg.stance_z;

        // 导数：x/y 匀速反扫（负向），z 不动
        vx = -cfg.step_x * sp_dot;
        vy = -cfg.step_y * sp_dot;
        vz = 0.0;
        ax = 0.0; ay = 0.0; az = 0.0;

    } else {

        // ---- swing 期：脚悬空前摆 ----
        // sp ∈ [0, 1)，把 phase 在 swing 段内归一化
        const double sp     = (phase - cfg.stance_frac) / (1.0 - cfg.stance_frac);
        const double sp_dot = 1.0 / ((1.0 - cfg.stance_frac) * cfg.cycle_s);

        // x: 线性前摆 -step_x/2(liftoff)→+step_x/2(touchdown)
        bx = (-0.5 + sp) * cfg.step_x         + raibert_x;
        by = leg_y_offset + (-0.5 + sp) * cfg.step_y + raibert_y;
        vx = cfg.step_x * sp_dot;
        vy = cfg.step_y * sp_dot;
        ax = 0.0; ay = 0.0;

        // ★ 方向感知领头腿抬升补偿（见 gait.hpp lead_lift_scale）：
        //   领头腿 = sign(hip_x)==sign(step_x)（前进→前腿/后退→后腿），把 step_h 放大补回被
        //   5-bar 几何压低的抬升。step_x≈0（原地）时 hip_x·step_x≈0 → 不补偿。
        double step_h_eff = cfg.step_h;
        if (cfg.lead_lift_scale != 1.0 && hip_x * cfg.step_x > 1e-6)
            step_h_eff = cfg.step_h * cfg.lead_lift_scale;

        // sin²(π·sp)软着陆: 端点vz=0消触地冲击. v/a解析导数保证前馈与轨迹一致.
        const double s = std::sin(M_PI * sp);
        const double c = std::cos(M_PI * sp);
        bz = cfg.stance_z + step_h_eff * (s * s);                                   // sin²(π·sp)
        vz = step_h_eff * M_PI * (2.0 * s * c) * sp_dot;                            // π·sin(2π·sp)·sp_dot
        az = step_h_eff * 2.0 * (M_PI * M_PI) * (c * c - s * s) * (sp_dot * sp_dot); // 2π²·cos(2π·sp)·sp_dot²
    }

    vel_body = {vx, vy, vz};
    acc_body = {ax, ay, az};
    return {bx, by, bz};
}

// 判断当前是否 stance 阶段（bool，基于 gait phase）
//
// 用途：
//   - main 里给电机 PD 选 swing/stance 增益（粗略，主要看 support_weight）
//   - 给 force_alloc 算 n_stance 数量
//
// 注：实际控制现在用连续的 support_weight（来自 contact_estimator 的 LPF），
//     这个 bool 函数主要给"想知道步态意图"的代码用。
bool is_stance(double t, const GaitConfig& cfg, double phase_offset) {
    const double phase = std::fmod(t / cfg.cycle_s + phase_offset, 1.0);
    return phase < cfg.stance_frac;
}

}  // namespace gait
}  // namespace motor_test_cpp
