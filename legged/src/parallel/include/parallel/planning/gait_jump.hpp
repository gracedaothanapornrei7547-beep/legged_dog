// gait_jump.hpp —— 纯 VMC 力控跳跃步态（垂直/向前）
// =============================================================================
// 所属模块: parallel::gait (planning 流水线)
// 职责: 4 腿同步主动控制足端 z（body 系），通过蹬伸→反作用弹起机身，不依赖 IMU、不算弹道。
//       状态机: IDLE → CROUCH → THRUST → FLIGHT → LAND → SETTLE → DONE → IDLE。
//       设计原理与调参详见 legged/调式日志/jump_design.md。
//       关键硬约束: reach 墙 hypot(x,z)<0.33; th2 死区 z 浅于 −0.16 危险; τ 峰 < 16Nm。
// =============================================================================
#pragma once

#include "parallel/common/pod_types.hpp"   // N_LEGS

namespace parallel {
namespace gait {
// 单位: z/x/reach = m(body 系, z 越负=腿越长); T = 秒; 力 = N; 力矩 = Nm; 电流 = A。
// 参数按跳跃时间轴排列:
//   IDLE → ① CROUCH → ② THRUST → ③ FLIGHT → ④ LAND → ⑤ SETTLE → DONE → IDLE
// z 命令每拍终钳在 [z_min_safe, z_max_safe]。逐参数详解见 jump_design.md §7。
struct JumpConfig {
    // ═══ ⓪ 全程基准 / 安全钳 / 电流(不属某一相, 贯穿整跳)════════════════════════
    double stand_z    = -0.23;  // 站立&收尾基准足端深度(m)。★main 每拍用 gait.stance_z 覆盖同步, 勿手调。
    double z_min_safe = -0.30;  // z 命令下限=腿最长(m): 防全伸闭链奇异(det_M→0, Jᵀ 失效→腿收不回=锁死)。
    double z_max_safe = -0.10;  // z 命令上限=腿最短(m): 防 th2 折进死区 154.49°(z 浅于 −0.16 即危险)。
    double idle_current_a   = 15.0;  // 非跳跃期单电机电流上限(A): 护 CAN(大电流→ERROR-PASSIVE)。
    double thrust_current_a = 23.0;  // 跳跃期电流上限(A): CROUCH 起切到此榨爆发, 落地复位 idle。仅跳跃期高(发热/CAN 代价)。

    // ═══ ① CROUCH 蹲下蓄力 ── 时长 crouch_T ══════════════════════════════════════
    double crouch_z = -0.14;    // 蹲深(m): 腿收短=机身下沉。浅近-0.10 蓄势大但逼 th2 死区(-0.16)
    double crouch_T = 0.3;      // 蹲时(s): 过快蹾/回弹污染蹬伸; 够慢让阻抗平稳压到 crouch_z
    double crouch_forward_x = 0.02; // ★蹲相机身前蹿量(m): 脚踩地锚住→机身真前挪。★唯一接触相搬 CoM 的旋钮。
                                //   crouch_fx + jump_fx ≤ 0.17(th2/reach 墙); 0.02+0.13=0.15 安全(前腿 th2 margin+14°)。

    // ═══ ② THRUST 蹬伸离地 ── 时长 thrust_T ══════════════════════════════════════
    //   z_des 在前 thrust_ramp_T 秒内从 crouch_z 快速冲到 launch_z 并保持；腿伸长蹬地→反作用弹起。
    //   跳高撞电机转速/back-EMF 硬件墙（命令侧已榨干，见 jump_design §2/§6）。
    double launch_z   = -0.27;  // 蹬伸目标深度(m): hypot(jfx,|z|)<reach 墙 0.33
    double thrust_T   = 0.12;   // 蹬伸总时长(s): >0.18 机身已回落
    double thrust_ramp_T = 0.1;  // 前置快斜坡(s)
    double thrust_kp_stance_z = 2200.0; // 蹬伸 z 刚度(N/m), 走 Jᵀ·F 不受 fz_max 钳
    double thrust_kd_z = 20.0;   // 蹬伸 z 阻尼(N·s/m): 低=松刹让腿 snap 伸长
    double thrust_kp_x = 800.0;  // 蹬伸 x 刚度(N/m): =kp_stance_x=不顶=关；硬驱(3000)实测机头栽-40°翻飞，前进/前栽绑死
    double thrust_ff_n    = 0.0; // 加性竖直前馈力(N/腿): 0=纯 kp×深目标蹬
    double thrust_front_w = 1.0; // 蹬伸前重整形: 1.0/1.0=平铺不整形(默认)
    double thrust_late_w  = 1.0;
    //   ── 向前跳 xy: 4 脚向后扫, 反作用推机身前进 ──
    double jump_forward_x    = 0.13; // 后扫量(body -x, m): ≈45°射程最优; >0.17 硬限(th2死区+reach墙)
    double jump_front_x_frac = 1.0;  // 前腿后扫比例: 1.0=4腿全扫; <1 卸前腿杀前进
    double jump_pitch_ff_nm  = 0.0;  // 开环抗前栽力矩(Nm): 抗前栽靠几何非力矩，保持 0

    // ═══ ③ FLIGHT 腾空: 上升收腿→下降伸腿接地 ── 时长上限 flight_T ═══════════════
    double flight_T = 0.50;     // 飞行相时长上限(s): 超时转 LAND。跳跃期接触检测不可信, 靠此超时兜底。
    double tuck_T   = 0.2;      // 收腿时长(s): 前此秒收腿, 之后转伸腿。触地判需 ph_t≥tuck_T 才认。
    double tuck_z   = -0.14;    // 收腿目标深度(m): 浅清障多, 但浅于-0.16 逼 th2 死区(前腿尤甚, 下楼跳易爆)
    double tuck_kp_stance_z = 800.0; // 收腿 z 阻抗(N/m): 腾空不承重故可小于蹬伸
    double reach_z  = -0.18;    // 伸腿目标深度(m, 比站高稍深): 提前触地+留压缩缓冲, 不硬腿砸地

    // ═══ ④ LAND 落地缓冲 ── 时长 land_T ══════════════════════════════════════════
    double land_reach_x_front = 0.12; // 前腿落地前伸量(body +x, m): 前脚伸到前方接前冲
    double land_reach_x_back  = 0.0;  // 后腿落地前伸量(m): 必须 0.0，后腿 +x 进入 th2 死区(0.08→175°, 0.05→168°均破死区)
    double land_T   = 0.35;     // 落地缓冲时长(s): 从触地 z 软回 stand_z, 阻抗吸冲击

    // ═══ ⑤ SETTLE 落稳收脚 ── 时长 settle_T → DONE ═══════════════════════════════
    double settle_T = 1.0;      // 落稳后前脚从前伸位软收回中立的时长(s): 机身顺势骑过前脚回正。

};

// ════════════════════════════════════════════════════════════════════════════
// ★四套跳跃参数(ch4/ch5 组合选, 见 dog_control 跳跃触发)。
//   选: ch5=1000+ch4=1000→上跳楼梯 · ch5=1000+ch4=2000→下楼梯小跳
//       ch5=2000+ch4=1000→沙石地跳高 · ch5=2000+ch4=2000→木桥B长跳
// ════════════════════════════════════════════════════════════════════════════

// ① 上跳楼梯(垂直上台阶) ── ch5=1000 + ch4=1000。
//   垂直起跳，四腿对称落地踩台阶。pitch 手动用 ch6 冻结。
inline JumpConfig jump_stair_up() {
    JumpConfig c;
    // ⓪ 基准/安全/电流
    c.stand_z            = -0.23;    // 站立&收尾基准深度(main 每拍用 gait.stance_z 覆盖)
    c.z_min_safe         = -0.30;    // z 命令下限=腿最长(防全伸奇异锁死)
    c.z_max_safe         = -0.10;    // z 命令上限=腿最短(防 th2 死区)
    c.idle_current_a     = 15.0;     // 非跳期单电机电流限(A, 护 CAN)
    c.thrust_current_a   = 23.0;     // 跳期电流限(A)
    // ① CROUCH 蹲下蓄力
    c.crouch_z           = -0.14;    // 蹲深(腿收短→机身下沉)
    c.crouch_T           = 0.30;     // 蹲时长(s)
    c.crouch_forward_x   = 0.02;     // 蹲相机身前蹿量(m)
    // ② THRUST 蹬伸离地
    c.launch_z           = -0.27;    // 蹬伸目标深度(hypot(jfx,|z|)<reach 墙 0.33)
    c.thrust_T           = 0.1;      // 蹬伸总时长(s, >0.18 机身已回落)
    c.thrust_ramp_T      = 0.08;     // 前置快斜坡(s)
    c.thrust_kp_stance_z = 2500.0;   // 蹬伸 z 刚度(N/m, 走 Jᵀ·F)
    c.thrust_kd_z        = 20.0;     // 蹬伸 z 阻尼(低=松刹让腿 snap 伸长)
    c.thrust_kp_x        = 800.0;    // 蹬伸 x 刚度(=kp_stance_x=关; 硬驱→前栽翻飞)
    c.thrust_ff_n        = 0.0;      // 加性竖直前馈力(N/腿, 0=纯 kp)
    c.thrust_front_w     = 1.0;      // 蹬伸前重整形(1/1=平铺不整形)
    c.thrust_late_w      = 1.0;
    c.jump_forward_x     = 0.11;     // 后扫量(body -x): 垂直为主，几不前扫
    c.jump_front_x_frac  = 1.0;      // 前腿后扫比例(1=4腿全扫)
    c.jump_pitch_ff_nm   = 0.0;      // 开环抗前栽力矩(Nm, 保持 0)
    // ③ FLIGHT 腾空(收腿→伸腿接地)
    c.flight_T           = 0.50;     // 飞行相时长上限(s, 超时转 LAND)
    c.tuck_T             = 0.10;     // 收腿时长(s)
    c.tuck_z             = -0.10;    // 收腿目标深度(⚠浅于 -0.16 逼前腿 th2 死区)
    c.tuck_kp_stance_z   = 1500.0;   // 收腿 z 刚度(腾空不承重故可小)
    c.reach_z            = -0.14;    // 伸腿接地目标深度(提前触地+留压缩缓冲)
    // ④ LAND 落地缓冲  ⑤ SETTLE 落稳收脚
    c.land_reach_x_front = 0.12;     // 前腿落地前伸(body +x): 踩阶面
    c.land_reach_x_back  = 0.08;     // 后腿落地前伸
    c.land_T             = 0.35;     // 落地缓冲时长(s, 软回 stand_z 吸冲击)
    c.settle_T           = 1.0;      // 落稳收脚时长(s)
    return c;
}

// ② 下楼梯小跳 ── ch5=1000 + ch4=2000。
inline JumpConfig jump_stairs_small() {
    JumpConfig c;
    // ⓪ 基准/安全/电流
    c.stand_z            = -0.23;    // 站立&收尾基准深度
    c.z_min_safe         = -0.30;    // z 下限=腿最长(防奇异锁死)
    c.z_max_safe         = -0.10;    // z 上限=腿最短(防 th2 死区)
    c.idle_current_a     = 15.0;     // 非跳期电流限(A)
    c.thrust_current_a   = 23.0;     // 跳期电流限(A)
    // ① CROUCH 蹲下蓄力
    c.crouch_z           = -0.18;    // 小蓄势行程(crouch→launch 仅 8cm)
    c.crouch_T           = 0.30;     // 蹲时长(s)
    c.crouch_forward_x   = 0.02;     // 蹲相前蹿(m)
    // ② THRUST 蹬伸离地
    c.launch_z           = -0.26;    // 浅蹬=小离地(仅比站高 -0.23 深 3cm)
    c.thrust_T           = 0.08;     // 短蹬
    c.thrust_ramp_T      = 0.05;     // ramp<thrust_T 让 z 冲到 launch 再短保持
    c.thrust_kp_stance_z = 1500.0;   // 温和不硬砸
    c.thrust_kd_z        = 20.0;     // 蹬伸 z 阻尼
    c.thrust_kp_x        = 800.0;    // 蹬伸 x 刚度(=关)
    c.thrust_ff_n        = 0.0;      // 加性前馈(0=纯 kp 小跳)
    c.thrust_front_w     = 1.0;      // 前重整形(不整形)
    c.thrust_late_w      = 1.0;
    c.jump_forward_x     = 0.05;     // 微前(清台阶沿，下楼主要往下)
    c.jump_front_x_frac  = 1.0;      // 前腿后扫比例(全扫)
    c.jump_pitch_ff_nm   = 0.0;      // 抗前栽力矩(0)
    // ③ FLIGHT 腾空
    c.flight_T           = 0.30;     // 短腾空
    c.tuck_T             = 0.20;     // 收腿时长(s)
    c.tuck_z             = -0.14;    // 收腿目标深度(⚠浅于 -0.16; 下楼前腿易爆 th2)
    c.tuck_kp_stance_z   = 800.0;    // 收腿 z 刚度
    c.reach_z            = -0.18;    // 伸腿接地目标深度(后腿直下安全)
    // ④ LAND 落地缓冲  ⑤ SETTLE
    c.land_reach_x_front = 0.05;     // 前腿落地前伸
    c.land_reach_x_back  = 0.05;     // 后腿落地前伸(必须 0.0 防 th2 死区)
    c.land_T             = 0.40;     // 落地缓更软
    c.settle_T           = 1.0;      // 落稳收脚时长(s)
    return c;
}


// ③ 沙石地跳高(碎石坑) ── ch5=2000 + ch4=1000。
inline JumpConfig jump_gravel() {
    JumpConfig c;
    // ⓪ 基准/安全/电流
    c.stand_z            = -0.23;    // 站立&收尾基准深度
    c.z_min_safe         = -0.30;    // z 下限=腿最长(防奇异锁死)
    c.z_max_safe         = -0.10;    // z 上限=腿最短(防 th2 死区)
    c.idle_current_a     = 15.0;     // 非跳期电流限(A)
    c.thrust_current_a   = 23.0;     // 跳期电流限(A)
    // ① CROUCH 蹲下蓄力
    c.crouch_z           = -0.14;    // 蹲深
    c.crouch_T           = 0.30;     // 蹲时长(s)
    c.crouch_forward_x   = 0.02;     // 蹲相机身前蹿量(m)
    // ② THRUST 蹬伸离地
    c.launch_z           = -0.27;    // 蹬伸目标深度
    c.thrust_T           = 0.12;     // 蹬伸总时长(s)
    c.thrust_ramp_T      = 0.10;     // 前置快斜坡(s)
    c.thrust_kp_stance_z = 2500.0;   // 蹬伸 z 刚度
    c.thrust_kd_z        = 20.0;     // 蹬伸 z 阻尼
    c.thrust_kp_x        = 800.0;    // 蹬伸 x 刚度(=关)
    c.thrust_ff_n        = 0.0;      // 加性竖直前馈力(N/腿, 0=纯 kp)
    c.thrust_front_w     = 1.0;      // 前重整形(不整形)
    c.thrust_late_w      = 1.0;
    c.jump_forward_x     = 0.10;     // 后扫量(body -x)
    c.jump_front_x_frac  = 1.0;      // 前腿后扫比例(全扫)
    c.jump_pitch_ff_nm   = 0.0;      // 抗前栽力矩(保持 0)
    // ③ FLIGHT 腾空
    c.flight_T           = 0.50;     // 飞行相时长上限(s)
    c.tuck_T             = 0.10;     // 收腿时长(s)
    c.tuck_z             = -0.14;    // 收腿目标深度(⚠浅于 -0.16 逼 th2)
    c.tuck_kp_stance_z   = 800.0;    // 收腿 z 刚度(腾空不承重故可小)
    c.reach_z            = -0.15;    // 伸腿接地目标深度(提前触地+留压缩缓冲)
    // ④ LAND 落地缓冲  ⑤ SETTLE
    c.land_reach_x_front = 0.12;     // 前腿落地前伸
    c.land_reach_x_back  = 0.08;     // 后腿落地前伸
    c.land_T             = 0.35;     // 落地缓冲时长(s)
    c.settle_T           = 1.0;      // 落稳收脚时长(s)
    return c;
}


// ④ 木桥B 长跳 ── ch5=2000 + ch4=2000。
inline JumpConfig jump_bridge_b() {
    JumpConfig c;
    // ⓪ 基准/安全/电流
    c.stand_z            = -0.23;    // 站立&收尾基准深度(main 每拍用 gait.stance_z 覆盖)
    c.z_min_safe         = -0.30;    // z 命令下限=腿最长(防全伸奇异锁死)
    c.z_max_safe         = -0.10;    // z 命令上限=腿最短(防 th2 死区)
    c.idle_current_a     = 15.0;     // 非跳期单电机电流限(A, 护 CAN)
    c.thrust_current_a   = 23.0;     // 跳期电流限(A)
    // ① CROUCH 蹲下蓄力
    c.crouch_z           = -0.14;    // 蹲深
    c.crouch_T           = 0.30;     // 蹲时长(s)
    c.crouch_forward_x   = 0.02;     // 蹲相机身前蹿量(m)
    // ② THRUST 蹬伸离地
    c.launch_z           = -0.26;    // 蹬伸目标深度(hypot(jfx,|z|)<reach 墙 0.33)
    c.thrust_T           = 0.12;     // 蹬伸总时长(s, >0.18 机身已回落)
    c.thrust_ramp_T      = 0.10;     // 前置快斜坡(s)
    c.thrust_kp_stance_z = 2000.0;   // 蹬伸 z 刚度(N/m, 走 Jᵀ·F)
    c.thrust_kd_z        = 20.0;     // 蹬伸 z 阻尼(低=松刹让腿 snap 伸长)
    c.thrust_kp_x        = 800.0;    // 蹬伸 x 刚度(=kp_stance_x=关; 硬驱→前栽翻飞)
    c.thrust_ff_n        = 0.0;      // 加性竖直前馈力(N/腿, 0=纯 kp)
    c.thrust_front_w     = 1.0;      // 蹬伸前重整形(1/1=平铺不整形)
    c.thrust_late_w      = 1.0;
    c.jump_forward_x     = 0.1;      // 后扫量(body -x): ≈45°射程最优, >0.17 硬限
    c.jump_front_x_frac  = 1.0;      // 前腿后扫比例(1=4腿全扫)
    c.jump_pitch_ff_nm   = 0.0;      // 开环抗前栽力矩(Nm, 保持 0)
    // ③ FLIGHT 腾空(收腿→伸腿接地)
    c.flight_T           = 0.50;     // 飞行相时长上限(s, 超时转 LAND)
    c.tuck_T             = 0.10;     // 收腿时长(s)
    c.tuck_z             = -0.14;    // 收腿目标深度(⚠浅于 -0.16 逼前腿 th2 死区)
    c.tuck_kp_stance_z   = 800.0;    // 收腿 z 刚度(腾空不承重故可小)
    c.reach_z            = -0.15;    // 伸腿接地目标深度(提前触地+留压缩缓冲)
    // ④ LAND 落地缓冲  ⑤ SETTLE 落稳收脚
    c.land_reach_x_front = 0.12;     // 前腿落地前伸(body +x, 接前冲)
    c.land_reach_x_back  = 0.08;     // 后腿落地前伸(必须 0.0: 后腿 +x 破 th2 死区→锁死)
    c.land_T             = 0.35;     // 落地缓冲时长(s, 软回 stand_z 吸冲击)
    c.settle_T           = 1.0;      // 落稳收脚时长(s)
    return c;
}



class JumpGait {
public:
    JumpConfig cfg;

    enum class Ph { IDLE, CROUCH, THRUST, FLIGHT, LAND, SETTLE, DONE };

    void trigger();          // 触发一次跳(仅 IDLE/DONE 接受, 跳跃中忽略重复触发)
    bool active() const;     // 是否正在跳(true → main 用本模块接管 body_tgt)
    void reset();

    // 每拍调用。dt(s)。各相靠 crouch_T/thrust_T/flight_T 计时转换。
    // 输出 z_cmd / x_cmd = 4 腿统一足端 z / x(body 系; x>0=前伸, 后腿前伸由 dog_control 按 th2 过滤)。
    // 返回 active()(false → main 回普通 trot/stand)。
    bool update(double dt, double& z_cmd, double& x_cmd);

    const char* phase_name() const;
    Ph phase() const { return ph_; }

    // 蹬伸前重相位整形权重 w(s): 仅 THRUST 相返回 clamp(front·(1−s), late, front), 其余相返回 1.0。
    double thrust_ff_scale() const;

private:
    Ph     ph_   = Ph::IDLE;
    double ph_t_ = 0.0;
    double z_    = -0.23;           // 当前 z 命令(相位间连续, 不跳变)
    double z_land_entry_ = -0.23;   // 进 LAND 那刻的 z(从它缓回站高)
    double x_    = 0.0;             // 当前 x 命令(蹬伸后扫 / 落地前伸)
};

}  // namespace gait
}  // namespace parallel
