// gait_jump.cpp —— JumpGait 状态机实现(CROUCH→THRUST→FLIGHT→LAND→SETTLE)。
// 设计/约束/实验依据见 docs/jump_design.md (reach 墙 0.33 / th2 死区 154.49° / 力撞 back-EMF 墙等)。
#include "parallel/planning/gait_jump.hpp"

#include <algorithm>
#include <cmath>

namespace parallel {
namespace gait {

namespace {
// 平滑插值: s∈[0,1], 两端一阶导=0(half-cosine), 避免命令阶跃冲击
inline double ease(double s) {
    s = std::clamp(s, 0.0, 1.0);
    return 0.5 - 0.5 * std::cos(M_PI * s);
}
inline double lerp(double a, double b, double s) { return a + (b - a) * s; }
}  // namespace

void JumpGait::trigger() {
    if (ph_ == Ph::IDLE || ph_ == Ph::DONE) {
        ph_ = Ph::CROUCH;
        ph_t_ = 0.0;
        z_ = cfg.stand_z;     // 从站高起蹲
    }
}

bool JumpGait::active() const {
    return ph_ != Ph::IDLE && ph_ != Ph::DONE;
}

void JumpGait::reset() {
    ph_ = Ph::IDLE;
    ph_t_ = 0.0;
    z_ = cfg.stand_z;
    z_land_entry_ = cfg.stand_z;
}

bool JumpGait::update(double dt, double& z_cmd, double& x_cmd) {
    ph_t_ += dt;
    x_ = 0.0;   // x(足端前后位,m)默认中立; 仅 THRUST 后扫 / FLIGHT/LAND/SETTLE 前伸 覆盖。

    switch (ph_) {
        case Ph::IDLE:
        case Ph::DONE:
            z_ = cfg.stand_z;
            break;

        case Ph::CROUCH: {
            // 蹲下蓄力: 站高 → crouch_z(机身降, 腿略收)
            const double s = ease(ph_t_ / std::max(1e-3, cfg.crouch_T));
            z_ = lerp(cfg.stand_z, cfg.crouch_z, s);
            // ★机身向前蹿: 蹲时脚向后扫 crouch_forward_x(脚踩地锚住→机身真往前挪)→ 起跳点前移→弹道前移。
            x_ = lerp(0.0, -cfg.crouch_forward_x, s);
            if (ph_t_ >= cfg.crouch_T) { ph_ = Ph::THRUST; ph_t_ = 0.0; }
            break;
        }

        case Ph::THRUST: {
            // 爆发蹬伸: 前置快斜坡, 前 thrust_ramp_T(s 绝对时间) 内 z_des 冲到 launch_z(m,深)并保持。
            //   早段误差大 → kp 硬砸顶满电机(脚慢时能发满力); 后段保持 launch, 靠加性前馈带到全行程。
            const double sr = std::clamp(ph_t_ / std::max(1e-3, cfg.thrust_ramp_T), 0.0, 1.0);
            const double s = ease(sr);
            z_ = lerp(cfg.crouch_z, cfg.launch_z, s);
            // 向前跳: 蹬伸把 4 脚继续向后扫(body −x,m)→ 反作用推机身向前。从蹲相蹿到的 −crouch_forward_x
            //   接续扫到 −(crouch_forward_x+jump_forward_x), 与竖直同步(★滞后版 thrust_x_lag_T 真机少10cm 已删)。
            x_ = lerp(-cfg.crouch_forward_x, -(cfg.crouch_forward_x + cfg.jump_forward_x), s);
            // 退出看计时器(整个跳跃已纯计时驱动, 接触检测从跳跃路径移除)。
            if (ph_t_ >= cfg.thrust_T) {
                ph_ = Ph::FLIGHT; ph_t_ = 0.0;
            }
            break;
        }

        case Ph::FLIGHT: {
            // 腾空: 上升段收腿(launch→tuck_z, 脚往身上收清腿); 下降段伸长接地
            //   (tuck→reach_z, 比站高更长 → 提前触地 + 留压缩余量缓冲冲击, 不用硬腿砸地)。
            const double sf = std::clamp(ph_t_ / std::max(1e-3, cfg.flight_T), 0.0, 1.0);
            if (ph_t_ < cfg.tuck_T)   // ★绝对时间: 前 tuck_T 秒收腿(上升)
                z_ = lerp(cfg.launch_z, cfg.tuck_z, ease(ph_t_ / std::max(1e-3, cfg.tuck_T)));
            else                       // tuck_T 之后到 flight_T: 伸腿接地(下降)
                z_ = lerp(cfg.tuck_z, cfg.reach_z,
                          ease((ph_t_ - cfg.tuck_T) / std::max(1e-3, cfg.flight_T - cfg.tuck_T)));
            // 落地前伸: 脚从后扫位 lerp 到前伸位(body +land_reach_x_front,m)接落地(前脚前扑)。
            //   后腿够不到前方(th2 死区)→ dog_control 对后腿把 x>0 钳回 0(中立); 此处统一目标前伸, 按腿过滤。
            x_ = lerp(-(cfg.crouch_forward_x + cfg.jump_forward_x), cfg.land_reach_x_front, ease(sf));
            // ★落地纯计时器(固定场景): FLIGHT 跑满 flight_T 转 LAND。期间腿先收(tuck_T)再伸到
            //   reach_z, flight_T 末已伸好 → 真触地常落在随后的 LAND 相, 靠阻抗软回站高吸冲击。
            //   接触检测已移除: 跳跃期它不可信(被自身力污染), 且固定场景弹道腾空时间可预测 → 计时器足矣。
            //   ★换跳跃场景(落差不同)→ 调那套预设的 flight_T 配真实腾空时间, 别加回检测。
            if (ph_t_ >= cfg.flight_T) {
                ph_ = Ph::LAND; ph_t_ = 0.0; z_land_entry_ = z_;
            }
            break;
        }

        case Ph::LAND: {
            // 落地缓冲: 从触地那刻的 z 软回站高(慢 = 阻抗吸收冲击)。
            const double s = ease(ph_t_ / std::max(1e-3, cfg.land_T));
            z_ = lerp(z_land_entry_, cfg.stand_z, s);
            // 前伸保持到 LAND 全程(前脚伸在前面接住前冲): flight_T(0.2s) < 真实腾空(~0.38s),
            //   真正触地常落在 LAND 相, 此时不可收回否则看不到前腿接身体。
            x_ = cfg.land_reach_x_front;   // 前腿(dog_control 过滤, 后腿钳 0)伸在前方接落地
            if (ph_t_ >= cfg.land_T) { ph_ = Ph::SETTLE; ph_t_ = 0.0; }
            break;
        }

        case Ph::SETTLE: {
            z_ = cfg.stand_z;
            // 落稳后前脚从前伸位软收回中立(脚已承重 → 身体顺势骑过前脚回正, 不突变)。
            x_ = lerp(cfg.land_reach_x_front, 0.0, ease(ph_t_ / std::max(1e-3, cfg.settle_T)));
            if (ph_t_ >= cfg.settle_T) { ph_ = Ph::DONE; ph_t_ = 0.0; }
            break;
        }
    }

    // 安全钳: z 命令(m,body 系)钳在 [z_min_safe, z_max_safe]; 下限防 IK 奇异, 上限防 th2 死区(浅于 −0.16 危险)。
    z_ = std::clamp(z_, cfg.z_min_safe, cfg.z_max_safe);
    z_cmd = z_;
    x_cmd = x_;
    return active();
}

double JumpGait::thrust_ff_scale() const {
    if (ph_ != Ph::THRUST) return 1.0;
    // s∈[0,1] 蹬伸相位; 前重: s=0→front(腿慢电机能发满), s大→late(撞功率墙)。
    const double s = std::clamp(ph_t_ / std::max(1e-3, cfg.thrust_T), 0.0, 1.0);
    return std::clamp(cfg.thrust_front_w * (1.0 - s), cfg.thrust_late_w, cfg.thrust_front_w);
}

const char* JumpGait::phase_name() const {
    switch (ph_) {
        case Ph::IDLE:   return "IDLE";
        case Ph::CROUCH: return "CROUCH";
        case Ph::THRUST: return "THRUST";
        case Ph::FLIGHT: return "FLIGHT";
        case Ph::LAND:   return "LAND";
        case Ph::SETTLE: return "SETTLE";
        case Ph::DONE:   return "DONE";
    }
    return "?";
}

}  // namespace gait
}  // namespace parallel
