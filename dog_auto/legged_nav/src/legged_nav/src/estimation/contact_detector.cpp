// contact_detector.cpp —— 单腿接触检测 update() 实现
// =============================================================================
//
// 【这个文件干啥】
//   每一拍（500Hz）对一条腿，回答："这条腿现在踩到地了吗？踩多稳？"
//   输出 confidence ∈ [0,1]，给下游 contact_estimator 做多帧迟滞滤波。
//
// 【输入信号有 2 个 channel】
//   1. τ_residual（力矩残差）—— 主信号
//      电机 τ_meas 减去我们自己 commanded 的 τ_ff，剩下的"τ_ext"理论上是
//      外力（地面反作用 + friction）。但 swing 期间空气中也有"τ_baseline"
//      （来自电机内部 PD/摩擦/惯量误差），需要扣掉才能识别真接触。
//
//      → τ_residual = |τ_ext| - τ_baseline
//      → 大于阈值 → 信号"接触了"
//
//   2. z_error（高度误差）—— 辅信号
//      期望 swing 弧形抛物线让脚到达 des_z（比 stance_z 高 step_h），但
//      如果脚先碰到地了，实际 z（act_z）会被卡在地面高度 → act_z > des_z
//
//      → z_error = act_z - des_z
//      → 连续几帧 > 阈值 → 信号"接触了"
//
// 【为什么要 "armed" 这个中间状态】
//   想象一条腿刚抬起来（swing 开始）：那一刻 act_z ≈ stance_z，des_z 也 ≈
//   stance_z，z_error≈0；τ_residual 也小（脚刚离开）。
//   如果直接判 "z_error 不大 + τ_residual 不大 = 没接触"，下一秒脚抬高了，
//   突然又落地 → 应该报触地。但状态机要从 "刚抬起" 切换到 "可识别触地" 才行。
//
//   armed = "现在可以开始判断有没有重新触地了"。
//   触发 armed 的条件（任一）：
//     (a) hold_ok：swing 已经持续 arm_hold_frames 帧（防止立刻误判）
//     (b) lifted：脚比 swing_entry_z 高出 lift_arm_threshold_m（明确抬起来了）
//     (c) unloaded：τ_residual < unloaded_tau_nm（明确空载，没被地顶住）
//
//   stance 腿（gait 说应该在地上）→ 永远 armed = true，跳过 swing 阶段。
//
// 【为什么 τ_ext = τ_meas - τ_ff】
//   电机 τ_meas 包含 3 部分：
//     τ_meas = τ_ff（我们 force_control 给的力前馈）
//            + kp·pos_err（电机 firmware 位置 PD）
//            + τ_external（地面反作用、friction、动力学误差）
//
//   我们关心 τ_external，但单独提取不出来。扣 τ_ff 后剩下：
//     τ_ext = kp·pos_err + τ_external
//
//   再用 τ_baseline 学习 "swing 期间无接触时的 τ_ext"（约等于 kp·pos_err）
//   → τ_residual = |τ_ext| - τ_baseline 就近似剩下 τ_external 的大小。
//
// =============================================================================

#include "motor_test/estimation/contact_detector.hpp"

#include <algorithm>
#include <cmath>

namespace motor_test_cpp {
namespace contact {

namespace {
// clamp 到 [0, 1]：用于把不同尺度的信号映射成 0~1 的置信度
inline double clamp01(double x) { return std::max(0.0, std::min(1.0, x)); }
}  // namespace

// ---------------------------------------------------------------------------
// enter_swing —— "我刚切到 swing 阶段了，重置状态"
//
// 调用时机：nominal_swing 从 false → true 的那一帧（gait 判定开始抬腿）
//
// 重置的 4 个东西：
//   z_contact_count_  : z_error 连续帧计数器归零（重新开始数）
//   armed_            : 关 armed（防止刚抬起就误判触地）
//   swing_hold_count_ : swing 已持续多少帧的计数器归零
//   swing_entry_z_    : 记下"我抬腿那一刻的 z"，后续用来比较"脚抬高了多少"
// ---------------------------------------------------------------------------
void ContactDetector::enter_swing(double act_z_body) {
    z_contact_count_  = 0;
    armed_            = false;
    swing_hold_count_ = 0;
    swing_entry_z_    = act_z_body;
}

// ---------------------------------------------------------------------------
// update_arm —— "我在 swing 期间，要不要 arm（开始警惕触地）"
//
// 每帧调用（仅当 nominal_swing == true）。
//
// armed 转 true 的条件：hold_ok 必须满足 && (lifted 或 unloaded 任一)
//   hold_ok  : swing 持续够久（≥ arm_hold_frames 帧 ≈ 20ms@500Hz）
//              避免刚抬起就误判
//   lifted   : 脚比 swing_entry_z 抬高超过 lift_arm_threshold_m（默认 3mm）
//              → 物理上确实抬起来了
//   unloaded : τ_residual 很小（< unloaded_tau_nm 默认 0.5 Nm）
//              → 力矩证据说明没被地顶住
//
// 一旦 armed=true，update() 里就会真正去判断 touchdown。
// ---------------------------------------------------------------------------
void ContactDetector::update_arm(const ContactInput& inp, double tau_residual) {
    ++swing_hold_count_;
    const bool hold_ok  = swing_hold_count_ >= cfg_.arm_hold_frames;
    const bool lifted   = (inp.act_z_body - swing_entry_z_) > cfg_.lift_arm_threshold_m;
    const bool unloaded = tau_residual < cfg_.unloaded_tau_nm;
    if (hold_ok && (lifted || unloaded)) {
        armed_ = true;
    }
}

// ---------------------------------------------------------------------------
// update —— 主入口：每拍调一次，返回当前接触估计
//
// 处理流程（5 步）：
//   1. 检测 swing 上升沿 → reset 状态
//   2. 计算 τ_ext，学习 τ_baseline（仅空载时学）
//   3. 计算 τ_residual（基于扣掉 ff 的 τ_ext）
//   4. 更新 armed 状态 + z_count 累计
//   5. 综合 τ_hit + z_hit → touchdown，加权 → confidence
// ---------------------------------------------------------------------------
ContactEstimate ContactDetector::update(const ContactInput& inp) {

    // ===== 步骤 1: swing 上升沿检测，重置状态机 =====
    // 上一帧不是 swing，这帧变成 swing → 刚刚切换到 swing 阶段
    if (inp.nominal_swing && !was_nominal_swing_) {
        enter_swing(inp.act_z_body);
    }
    was_nominal_swing_ = inp.nominal_swing;

    // ===== 步骤 2a: 计算 τ_ext = τ_meas - τ_ff =====
    // τ_ext 是"扣除我们自己的力前馈后剩余的力矩"。
    // 物理意义：地面反作用 + 摩擦 + 动力学误差 + 电机 firmware PD 误差。
    // 这是接触检测的主信号。
    const double tau1_ext = inp.tau1_meas - inp.tau1_ff;
    const double tau2_ext = inp.tau2_meas - inp.tau2_ff;

    // ===== 步骤 2b: 学习 τ_baseline =====
    // τ_baseline = "swing 期间无接触时的 |τ_ext|"
    //              ≈ kp·pos_err 的稳态值（电机 firmware PD 跟踪误差）
    //
    // 学习门控（双重保险，避免学进接触干扰）：
    //   - nominal_swing：gait 期望腿在空中
    //   - |τ_ext| < tau_threshold_nm：力矩本身没异常大（如果腿其实接触了，
    //                                τ_ext 会很大，此时跳过本帧学习）
    //
    // EMA 更新（指数移动平均）：α=0.05 → 时间常数 ~20 帧 ≈ 40ms
    if (inp.nominal_swing) {
        if (std::fabs(tau1_ext) < cfg_.tau_threshold_nm) 
        {
            tau_baseline_1_ = (1.0 - cfg_.tau_baseline_alpha) * tau_baseline_1_
                             + cfg_.tau_baseline_alpha * std::fabs(tau1_ext);
        }
        if (std::fabs(tau2_ext) < cfg_.tau_threshold_nm) 
        {
            tau_baseline_2_ = (1.0 - cfg_.tau_baseline_alpha) * tau_baseline_2_
                             + cfg_.tau_baseline_alpha * std::fabs(tau2_ext);
        }
    }

    // ===== 步骤 3: 计算 τ_residual（两轴取最大）=====
    // residual = |τ_ext| - baseline = "本帧外力比 swing 默认值高出多少"
    //   > 0  → 比平时受力大 → 可能踩到东西
    //   ≤ 0  → 比平时还小 → 没接触
    //
    // 取两轴（th1, th2 = 髋膝两个抬腿关节）的较大值：
    //   只要一个轴受力异常，就视为有信号。
    const double tau_residual = std::max(
        std::fabs(tau1_ext) - tau_baseline_1_,
        std::fabs(tau2_ext) - tau_baseline_2_);

    // ===== 步骤 4a: armed 状态更新 =====
    if (inp.nominal_swing) {
        // swing 期：按 update_arm 的条件慢慢 arm 起来
        update_arm(inp, tau_residual);
    } else {
        // stance 期：gait 说应该在地上，直接强 armed = true
        // （contact_detector 主要是为了识别 swing 期间的早触地）
        armed_ = true;
    }

    // ===== 步骤 4b: z 高度检测累计 =====
    // 期望：swing 弧形让 act_z 比 stance_z 高（des_z 是 swing 抛物线高度）
    // 实际：如果脚提前撞到地，act_z 上不去 → act_z > des_z（意思是脚比期望"浅"）
    //       ⚠️ body 系下 z 朝下越大表示越浅
    //
    // z_error = act_z - des_z（>0 表示脚比期望浅，可能撞地）
    const double z_error = inp.act_z_body - inp.des_z_body;
    bool z_hit;
    if (inp.des_vz_body > 0.02) {
        // swing 上升段（脚正在往上抬）：忽略 z_error，因为这阶段 act 跟随 des
        // 难免有小差距，不算接触
        z_contact_count_ = 0;
        z_hit = false;
    } else {
        // swing 下降段 或 stance 期：z_error 超过阈值才计入
        if (z_error > cfg_.z_error_threshold_m) 
        {
            ++z_contact_count_;            // 这帧 z 异常 → 累加
        } 
        // 这帧 z 正常 → 缓慢衰减（不归零，要"持续异常"才算）
        else {
            z_contact_count_ = std::max(0, z_contact_count_ - 1);
        }
        // 连续 z_contact_frames 帧（默认 3 帧 = 6ms）异常 → 视为 z 检测命中
        z_hit = z_contact_count_ >= cfg_.z_contact_frames;
    }

    // ===== 步骤 5a: 综合触地判定 =====
    // tau_hit: τ_residual 超过阈值（默认 1.5 Nm）→ 力矩信号说接触了
    // touchdown: 必须 armed && (tau_hit 或 z_hit)
    //   armed = 状态机准备好了
    //   tau_hit 或 z_hit = 至少一个 channel 报警
    const bool tau_hit  = tau_residual > cfg_.tau_threshold_nm;
    const bool touchdown = (armed_ && (tau_hit || z_hit));

    // ===== 步骤 5b: 置信度合成（0~1 软值）=====
    //   z_score   : z_count 占阈值的比例（0~1）
    //   tau_score : 1.0（如果超阈值）或 残差/阈值的比例
    //
    //   confidence = 50% tau_score + 30% tau_hit bonus + 20% z_score
    //   逻辑：力矩信号是主导（50% 权重），bonus 推到接近 1，z 辅助
    const double z_score   = clamp01(static_cast<double>(z_contact_count_) / cfg_.z_contact_frames);
    const double tau_score = tau_hit ? 1.0
                                      : clamp01(tau_residual / std::max(cfg_.tau_threshold_nm, 1e-6));
    double confidence = clamp01(0.5 * tau_score + 0.3 * (tau_hit ? 1.0 : 0.0) + 0.2 * z_score);

    // ===== 步骤 5c: stance 腿地板下限 —— 下垂感知 =====
    //   stance 腿被分配下蹬力 F, 脚悬空→F 把脚压到目标下方(F/kp_stance_z, 满 share~2.5cm)→
    //   z_error 显著为负; 触地瞬间地面顶住停沉→z_error≈0。据此区分真接触 vs 悬空步态回声:
    //   z_error < -sag_air_m(1.2cm) = 悬空→压低置信度(≤0.45 防 τ 噪声假触发);
    //   否则 = 真触地→给地板(conf≥0.75 保住估计器锚点)。平地正常承重 sag≈0→零回归。
    bool sag_air = false;   // ★观测: 仅 nominal-stance 腿有意义(悬空=true)
    if (!inp.nominal_swing)
    {
        sag_air = (z_error < -cfg_.sag_air_m);
        if (!sag_air) confidence = std::max(confidence, 0.75);
        else          confidence = std::min(confidence, 0.45);
    }

    return ContactEstimate{confidence, touchdown, armed_, tau_residual, z_error, sag_air};
}

}  // namespace contact
}  // namespace motor_test_cpp
