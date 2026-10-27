// baseline.cpp —— 运动学控制链: body_tgt → raw_cmd (纯运动学, 不算力不闭环)
// =============================================================================
// 步骤: ①body→local ②clamp_workspace ③th2 软监督+限位感知 IK ④math_to_raw。
// 唯一 fallback: IK 几何无解(目标超 l1+l2+l3)→沿用 last_raw。
// 限位感知 IK 是关节合法性唯一防线; 下游仅保留 NaN 清洗 + tau_max 力矩硬钳。
// =============================================================================

#include "parallel/kinematics/baseline.hpp"

#include <algorithm>
#include <cmath>

namespace parallel {
namespace baseline {


// =============================================================================
// control_chain —— body_tgt → raw_cmd + math_des (单腿)
//
// 输入:
//   cal       : 该腿的标定（电机 ID、方向、坐标翻号）
//   kin       : leg kinematics（FK/IK/Jacobian）
//   limits    : 关节软限位 + 工作空间限制
//   body_tgt  : body 系下足端期望位置 (相对 hip 的 xyz)
//   last_raw  : 上一帧 raw（in/out，IK 失败时 fallback）
//
// 输出:
//   raw_out   : 发给电机的 raw 命令 (th3, th1, th2)
//   math_out  : 安全钳后的 math 关节角（喂下游 force_control 算 J^T·F）
//
// 返回:
//   true  : IK 成功，raw_out / math_out / last_raw 都已更新
//   false : 失败，raw_out = last_raw (用旧命令保护)
// =============================================================================

// ★ th2 软监督(工作域安全层, 2026-06-11): 若足端目标会让限位感知 IK 的 th2 超软边界(th2_soft),
//   把目标往前(+x_local)拉、每步 4mm, 直到 th2≤soft 或到 8 步上限。机理: th2 由"深(大z)+后(负x)"
//   推高(可行域解析: stance-aft 是 trot 唯一紧约束), 前移=减后→降 th2, 留 ~2.5° 到 154.49 死区硬停。
//   +x_local 对所有腿都是"前/减后"(body_to_local 已处理每腿 x_sign)。代价: aft 落点被钳→该腿整步
//   stride 略短; 收益: 永不顶死区硬停。返回最终 IK 解(null=几何无解), 并通过引用更新 tgt(下游
//   force_control 用同一目标)。绝大多数 tick th2<soft → 只 1 次 IK。
static std::optional<Eigen::Vector3d> solve_th2_supervised(
        const kinematics::LegKinematics& kin,
        const robot::JointLimits& lim,
        Eigen::Vector3d& tgt) {
    std::optional<Eigen::Vector3d> ik;
    for (int it = 0; it < 8; ++it) {
        ik = kin.inverse_kinematics(tgt.x(), tgt.y(), tgt.z(),
                                    lim.th1_min, lim.th1_max,
                                    lim.th2_min, lim.th2_max,
                                    lim.th3_min, lim.th3_max);
        if (!ik) return std::nullopt;
        if ((*ik)[2] <= lim.th2_soft) break;   // th2 在软边界内 → 不动
        tgt.x() += 0.004;                       // th2 过高 → 前移 4mm/步(减后→降 th2)
    }
    return ik;
}

bool control_chain(const robot::LegCalibration& cal,
                   const kinematics::LegKinematics& kin,
                   const robot::JointLimits& limits,
                   const Eigen::Vector3d& body_tgt,
                   Eigen::Vector3d& last_raw,
                   Eigen::Vector3d& raw_out,
                   Eigen::Vector3d& math_out,
                   ControlChainDebug* dbg) {

    // ===== 步骤 1: body 系 → 单腿 local 系 =====
    // 每条腿装配方向不同（前腿/后腿 hip 朝外侧、左右对称等）
    // body_to_local 按 cal.x_sign/y_sign/z_sign 翻号到该腿自己的坐标系
    Eigen::Vector3d local_tgt = cal.body_to_local(body_tgt);

    // ===== 步骤 2: clamp 工作空间 (球 r∈[min,max] + |y| + ★z深度下限 0.16) =====
    local_tgt = limits.clamp_workspace(local_tgt);

    // ===== 步骤 3: th2 软监督 + 限位感知 IK (工作域安全层) =====
    //   th2 软监督: 把"会顶 th2 死区"的 aft+深目标往前拉, 留余量。
    //   限位感知 IK: 候选支里优先选落在关节限位内的解(th2_max=154.49 硬停)。
    //   仅几何无解(目标超 l1+l2+l3)才 nullopt→沿用上帧。下游保留 NaN 清洗 + tau_max 硬钳。
    auto ik = solve_th2_supervised(kin, limits, local_tgt);   // local_tgt 可能被前移
    if (dbg) dbg->local_tgt = local_tgt;
    if (!ik.has_value()) {
        raw_out = last_raw;
        if (dbg) dbg->fail_step = 1;
        return false;
    }
    const double th3_d = (*ik)[0], th1_d = (*ik)[1], th2_d = (*ik)[2];
    // ★最终安全钳: th2绝不超 deadzone(154.49°)。IK解偶尔会在软监督8步后仍轻微超限,钳住。
    const double th2_safe = std::clamp(th2_d, limits.th2_min, limits.th2_max);
    if (dbg) { dbg->th3 = th3_d; dbg->th1 = th1_d; dbg->th2 = th2_safe; }

    // ===== 步骤 4: math 关节 → 电机 raw (按 dir_th* 翻号 + deadzone) =====
    Eigen::Vector3d new_raw = cal.math_to_raw_angles(th3_d, th1_d, th2_safe);

    last_raw = new_raw;
    raw_out  = new_raw;
    math_out = {th3_d, th1_d, th2_safe};   // math 喂下游 force_control 算 J^T·F
    if (dbg) dbg->fail_step = 0;
    return true;
}

// 注意：旧 compute_torque_ff 已迁到 force_control::compute_leg_tau

}  // namespace baseline
}  // namespace parallel
