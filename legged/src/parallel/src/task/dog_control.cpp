/**
 * @file dog_control.cpp
 * @brief 十二自由度并联四足机器人实时任务控制器 (DogControl) 实现
 * 
 * 核心控制管道:
 * 1. 运动规划层 (Planning Layer):
 *    - 余弦平滑插值缓启动 (Cosine-lerp Soft Startup)
 *    - 对角步态生成器 (Trot Gait Trajectory Generator & Phase Sequencer)
 *    - 捕获点与 Raibert 落地外推修正 (Capture Point & Raibert Touchdown Heuristic)
 *    - 动力跳跃状态机 (Jump Gait Phase Sequencer)
 *    - 地形自适应斜坡/台阶姿态补偿 (Terrain Adapt Integrator)
 * 2. 运动学解算层 (Kinematics Layer):
 *    - 五连杆并联支链正/逆运动学 (5-Bar Pantograph Forward/Inverse Kinematics)
 *    - 60° 斜向 Roll 轴空间几何解耦 (60° Oblique Roll Axis Kinematics)
 *    - 运动学工作空间与关节限位连续性校验 (Kinematic Workspace & Joint Limit Continuity)
 * 3. 状态估计与接触检测层 (State Estimation Layer):
 *    - 广义力矩残差与运动学跟踪误差足端接触检测 (Contact Detector via Torque Residual & Z-Error)
 *    - 多通道双阈值迟滞接触估计与支撑权重连续滤波 (Support Weight Continuous LPF)
 *    - 浮动基状态观测器 (Floating-base Multi-contact State Estimator)
 * 4. 动力学控制与力分配层 (Dynamics & Allocation Layer):
 *    - 机身姿态全向空间阻抗控制 (Spatial Wrench Impedance Controller)
 *    - 库仑摩擦锥约束下的接触力二次规划/伪逆分配 (Contact Wrench Distribution)
 *    - 笛卡尔空间虚拟模型控制与雅可比力矩映射 (Cartesian VMC & Jacobian Transpose Mapping)
 * 5. 硬件抽象与交错通信调度 (Hardware Abstraction & Interleaved Transmission):
 *    - 3-Slot 时钟切片交错并发传输 (Time-sliced Interleaved Transmission)
 *    - 总线运行状态实时监控与自适应降级保护机制 (Bus Degradation Safe-Hold & E-Stop)
 *    - 零拷贝环形缓冲二进制遥测记录 (Ring-buffered Binary Flight Telemetry)
 */

#include "parallel/task/dog_control.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <thread>
#include <vector>

#include "parallel/common/safety.hpp"
#include "parallel/planning/gait.hpp"
#include "parallel/planning/subs_gait.hpp"
#include "parallel/driver/sbus_reader.hpp"
#include "parallel/kinematics/baseline.hpp"
#include "parallel/kinematics/foot_measure.hpp"
#include "parallel/dynamics/body_posture.hpp"
#include "parallel/dynamics/force_alloc.hpp"
#include "parallel/dynamics/force_control.hpp"
#include "parallel/dynamics/wbc.hpp"

namespace {

/**
 * @brief 高精度纳秒时间累加辅助函数
 */
inline void add_ns(timespec& ts, long ns) {
    ts.tv_nsec += ns;
    while (ts.tv_nsec >= 1'000'000'000L) {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1'000'000'000L;
    }
}

/**
 * @brief PREEMPT_RT 绝对单调时钟高精度休眠
 */
inline void sleep_until(const timespec& ts) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

using parallel::can_bridge::MotorState;
using parallel::can_bridge::BusState;
using parallel::can_bridge::bus_state_name;
using parallel::kinematics::LegKinematics;
using parallel::kinematics::FKResult;
using parallel::robot::LEG_NAMES;
using parallel::N_LEGS;
using parallel::robot::JointLimits;
namespace gait          = parallel::gait;
namespace baseline      = parallel::baseline;
namespace kinematics    = parallel::kinematics;
namespace body_posture  = parallel::body_posture;
namespace force_alloc   = parallel::force_alloc;
namespace force_control = parallel::force_control;
namespace raibert       = parallel::raibert;
namespace wbc           = parallel::wbc;

}  // namespace

namespace parallel {
namespace task {

DogControl::DogControl(interface::RunConfig& a_, HardwareManager& hw_,
                       kinematics::LegKinematics& kin_, robot::JointLimits& limits_,
                       std::array<LegCtx, N_LEGS>& legs_,
                       estimator::Estimator& body_est_,
                       contact::ContactEstimator& contact_est_,
                       std::array<bool, N_LEGS>& is_contact_,
                       std::atomic<bool>& g_stop_)
    : a(a_), hw(hw_), kin(kin_), limits(limits_), legs(legs_),
      body_est(body_est_), contact_est(contact_est_), is_contact(is_contact_), g_stop(g_stop_),
      dt_ns(static_cast<long>(1e9 / std::max(10.0, a_.rate_hz))), step_ns(dt_ns / 3) {
    stance_z_target = a.gait.stance_z;

    // 初始化 SBUS 遥控通信接收器
    if (a.sbus.enabled) {
        hw.sbus() = std::make_unique<sbus::SbusReader>(a.sbus.port, a.sbus.invert);
        if (!hw.sbus()->start()) {
            std::fprintf(stderr, "[DogControl] 警告: SBUS 遥控端口打开失败 (%s)，退回内部默认输入\n",
                         a.sbus.port.c_str());
            hw.sbus().reset();
        }
    }

    // 初始化实时二进制遥测落盘器
    if (!a.telem_log.empty()) {
        telem = std::make_unique<telem::TelemetryLogger>(a.telem_log, a.rate_hz);
        if (!telem->ok()) {
            std::fprintf(stderr, "[DogControl] 警告: 遥测日志文件创建失败: %s\n", a.telem_log.c_str());
            telem.reset();
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &next_tick);
    t0 = std::chrono::steady_clock::now();
}

body_posture::BodyPostureConfig DogControl::resolve_posture_config(bool in_trot) {
    body_posture::BodyPostureConfig bp_cfg = a.body_posture;

    // 对角步态运行期间适当下调俯仰微分增益 kd_pitch，抑制角速度高频测量噪声
    if (in_trot) {
        bp_cfg.kd_pitch = 2.0;
    }

    // 跳跃阶段动力学配置: THRUST (蹬伸) 与 FLIGHT (腾空) 阶段冻结 Roll/Yaw 姿态环约束
    {
        const auto jph = jump_gait.phase();
        using JPh = gait::JumpGait::Ph;
        if (jph == JPh::THRUST || jph == JPh::FLIGHT) {
            bp_cfg.kp_roll = 0.0;
            bp_cfg.kd_roll = 0.0;
            bp_cfg.kp_yaw  = 0.0;
            bp_cfg.kd_yaw  = 0.0;
        }
    }

    // 遥控通道6俯仰刚度开关: 爬坡或非结构化地形通过时解除俯仰刚性约束
    if (!pitch_enabled) {
        bp_cfg.kp_pitch = 0.0;
        bp_cfg.kd_pitch = 0.0;
    }

    // 转向执行期间解除偏航位置比例锁 (kp_yaw=0)，仅保留角速度阻尼；
    // 转向停止后将当前实际航向锁定为期望航向 yaw_des，实现转向后的精准定向保持
    const bool turning_now = in_trot && std::fabs(a.yawrate_cmd) > 1e-3;
    if (turning_now) {
        bp_cfg.kp_yaw = 0.0;
        yaw_was_turning = true;
    } else if (yaw_was_turning) {
        a.body_posture.yaw_des = body_est.output().yaw;
        yaw_was_turning = false;
    }

    return bp_cfg;
}

force_control::ForceCtrlConfig DogControl::resolve_force_config() {
    force_control::ForceCtrlConfig force_cfg_eff = a.force_ctrl;
    if (jump_gait.active()) {
        using JPh = gait::JumpGait::Ph;
        const auto jph = jump_gait.phase();
        if (jph == JPh::THRUST) {
            // 蹬伸阶段配置: 提升垂直刚度以提供充足离地冲量，同时减小垂直阻尼提高末端伸展动态响应
            force_cfg_eff.kp_stance_z = jump_gait.cfg.thrust_kp_stance_z;
            force_cfg_eff.kd_stance_z = jump_gait.cfg.thrust_kd_z;
            force_cfg_eff.kp_stance_x = jump_gait.cfg.thrust_kp_x;
        } else if (jph == JPh::FLIGHT) {
            // 腾空收腿阶段配置: 增大垂直刚度确保足端快速收拢
            force_cfg_eff.kp_stance_z = jump_gait.cfg.tuck_kp_stance_z;
        }
    }
    return force_cfg_eff;
}

void DogControl::reset_legs() {
    for (int i = 0; i < N_LEGS; ++i) {
        // 1. 基于名义站姿目标重新逆解各腿关节角，复位控制参考帧
        const Eigen::Vector3d& sc = legs[i].stance_center;
        auto ik = kin.inverse_kinematics(sc.x(), sc.y(), sc.z(),
                                          limits.th1_min, limits.th1_max,
                                          limits.th2_min, limits.th2_max,
                                          limits.th3_min, limits.th3_max);
        if (ik) {
            legs[i].last_raw = legs[i].cal->math_to_raw_angles((*ik)[0], (*ik)[1], (*ik)[2]);
        }

        // 2. 清除控制链连续性诊断计数器
        legs[i].ik_fail = 0;
        legs[i].cc_step = {};
        legs[i].cc_max_jump = 0.0;
        legs[i].cc_max_fkerr = 0.0;

        // 3. 重置笛卡尔空间误差积分项 (Anti-windup)
        legs[i].F_int_local.setZero();

        // 4. 支撑权重恢复为全支撑状态
        legs[i].support_weight = 1.0;

        // 5. 复位足端接触检测器状态
        legs[i].detector = contact::ContactDetector{};
        legs[i].last_est = contact::ContactEstimate{};
        legs[i].touchdown_count = 0;
        legs[i].prev_body_tgt_z = sc.z();
    }

    // 6. 终止并重置跳跃状态机
    jump_gait = gait::JumpGait{};
    jump_was_active = false;

    // 7. 清除偏航轴误差积分
    yaw_integral = 0.0;
}

void DogControl::tick() {
    auto& sbus_reader = hw.sbus();
    auto& imu         = hw.imu();
    const auto& gait_cfg = a.gait;

    const auto now = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(now - t0).count();
    const bool in_trot = (t >= a.startup_s);

    /* === 1. 足端期望轨迹生成 (规划层) === */
    std::array<Eigen::Vector3d, N_LEGS> body_tgt;
    std::array<Eigen::Vector3d, N_LEGS> body_vel_tgt{}, body_acc_tgt{};
    bool use_analytic_ff = false;

    if (t < a.startup_s) {
        // 缓启动阶段: 余弦插值平滑过渡至名义站姿
        for (int i = 0; i < N_LEGS; ++i) {
            body_tgt[i] = gait::cosine_lerp(legs[i].body0, legs[i].stance_center, t / a.startup_s);
        }
        // 站立初期锁定绝对航向为 0° 参考
        if (t >= 0.02 && !yaw0_latched) {
            a.body_posture.yaw_des = body_est.output().yaw;
            yaw0_latched = true;
        }
    } else {
        if (!startup_done_logged) {
            if (!yaw0_latched) {
                a.body_posture.yaw_des = body_est.output().yaw;
                yaw0_latched = true;
            }
            startup_done_logged = true;
        }
        const double t_trot = t - a.startup_s;

        // SBUS 遥控指令解析与滤波注入
        if (sbus_reader) {
            const auto sb = parallel::sbus::parse_command(
                sbus_reader->snapshot(), t, a.sbus, a.gait.step_h);

            a.body_posture.pitch_des = 0.0;

            // 通道7: 三段高度调节与系统应急复位触发
            if (sb.fresh) {
                const int ch7_raw = sbus_reader->snapshot().channels[7];
                if (ch7_raw < 1750) stance_z_target = sb.stance_z;
            }
            {
                const double mxs = a.sbus.height_slew_mps / std::max(10.0, a.rate_hz);
                a.gait.stance_z += std::clamp(stance_z_target - a.gait.stance_z, -mxs, mxs);
            }

            // 通道4 / 通道5: 动态跳跃步态选择与触发
            if (sb.fresh) {
                const auto& snap = sbus_reader->snapshot();

                // 通道7高档位触发上升沿应急复位
                {
                    const int ch7_raw = snap.channels[7];
                    const bool ch7_high = (ch7_raw > 1750);
                    if (ch7_high && !ch7_was_high) reset_legs();
                    ch7_was_high = ch7_high;
                }

                const int ch4_raw = snap.channels[ch_jump];
                const int ch5_raw = snap.channels[a.sbus.ch_att_ab];
                const bool ch4_low  = (ch4_raw < jump_raw_max);
                const bool ch4_high = (ch4_raw > jump_raw_high);
                const bool in_band  = ch4_low || ch4_high;

                if (in_band && !jump_ch_in_band_prev && !jump_gait.active()) {
                    const bool ch5_high = (ch5_raw > a.sbus.att_ab_raw_min);
                    if      (!ch5_high && ch4_low)  { jump_gait.cfg = gait::jump_stair_up(); }
                    else if (!ch5_high && ch4_high) { jump_gait.cfg = gait::jump_stairs_small(); }
                    else if ( ch5_high && ch4_low)  { jump_gait.cfg = gait::jump_gravel(); }
                    else                            { jump_gait.cfg = gait::jump_bridge_b(); }

                    jump_gait.cfg.stand_z = a.gait.stance_z;
                    jump_gait.trigger();

                    // 清除笛卡尔积分项，防止大位移跳跃过渡导致积分饱和
                    for (int k = 0; k < N_LEGS; ++k) legs[k].F_int_local.setZero();
                }
                jump_ch_in_band_prev = in_band;

                // 通道6: 俯仰姿态阻抗开关 (高档位时冻结俯仰补偿环)
                pitch_enabled = (snap.channels[6] < 1750);
            }

            const bool stick = sb.fresh && sb.gait_enabled;
            if (stick || !a.sbus.smooth_stop) {
                eff_step_x = stick ? sb.step_x  : 0.0;
                eff_step_y = stick ? sb.step_y  : 0.0;
                eff_step_h = stick ? 0.06 : 0.0;
                eff_yaw    = stick ? sb.yawrate : 0.0;
            } else {
                // 平滑回中: 遥控杆回中时平滑过渡指令至中立位
                const double dt_loop = 1.0 / std::max(10.0, a.rate_hz);
                const double k = dt_loop / std::max(0.05, a.sbus.home_s);
                auto slew0 = [](double cur, double maxstep) {
                    if (cur >  maxstep) return cur - maxstep;
                    if (cur < -maxstep) return cur + maxstep;
                    return 0.0;
                };
                eff_step_x = slew0(eff_step_x, a.sbus.max_step_x  * k);
                eff_step_y = slew0(eff_step_y, a.sbus.max_step_y  * k);
                eff_step_h = slew0(eff_step_h, 0.06 * k);
                eff_yaw    = slew0(eff_yaw,    a.sbus.max_yawrate * k);
            }
            a.gait.step_x = eff_step_x;
            a.gait.step_y = eff_step_y;
            a.gait.step_h = eff_step_h;
            a.yawrate_cmd = eff_yaw;

            if (!sb.fresh && !sbus_idle_logged) {
                std::fprintf(stderr, "[DogControl] 遥控信号丢失 (Failsafe) -> 指令平滑归零并保持站立\n");
                sbus_idle_logged = true;
            } else if (sb.fresh) {
                sbus_idle_logged = false;
            }
        }

        use_analytic_ff = true;
        const auto& est = body_est.output();
        const double t_stance = a.gait.cycle_s * a.gait.stance_frac;
        const raibert::RaibertOffset rb_fresh =
            raibert::compute_touchdown(a.raibert, est.ang_vel_body.x());

        // 地形自适应补偿 (斜坡/台阶): 基于机身倾角误差积分调整四足名义高度
        if (a.gait.terrain_adapt) {
            const double dt_ta  = 1.0 / std::max(10.0, a.rate_hz);
            const double sp_max = a.gait.terrain_adapt_max_m / 0.201;
            const double sr_max = a.gait.terrain_adapt_max_m / 0.102;
            const double ta_leak = dt_ta / std::max(0.1, a.gait.terrain_adapt_leak_s);
            ta_s_pitch += a.gait.terrain_adapt_k * (a.body_posture.pitch_des - est.pitch) * dt_ta - ta_leak * ta_s_pitch;
            ta_s_roll  += a.gait.terrain_adapt_k * (est.roll  - a.body_posture.roll_des)  * dt_ta - ta_leak * ta_s_roll;
            ta_s_pitch = std::clamp(ta_s_pitch, -sp_max, sp_max);
            ta_s_roll  = std::clamp(ta_s_roll,  -sr_max, sr_max);
        }

        // 跳跃运动轨迹接管 (四足同步蹲踞-蹬伸-腾空-着陆)
        if (jump_gait.active()) {
            use_analytic_ff = false;
            const double dt_j  = 1.0 / std::max(10.0, a.rate_hz);
            double jz = a.gait.stance_z;
            double jx = 0.0;
            jump_gait.update(dt_j, jz, jx);

            if (!jump_was_active) {
                for (int k = 0; k < N_LEGS; ++k)
                    hw.set_leg_current_limit(k, jump_gait.cfg.thrust_current_a);
            }

            for (int i = 0; i < N_LEGS; ++i) {
                double leg_x = jx * (parallel::robot::HIP_BODY_XY[i][0] < 0.0
                                     ? 1.0 : jump_gait.cfg.jump_front_x_frac);
                if (parallel::robot::HIP_BODY_XY[i][0] < 0.0 && leg_x > 0.0) {
                    const double frac = std::clamp(
                        leg_x / std::max(1e-3, jump_gait.cfg.land_reach_x_front), 0.0, 1.0);
                    leg_x = jump_gait.cfg.land_reach_x_back * frac;
                }
                body_tgt[i] = Eigen::Vector3d(leg_x, legs[i].cal->stand_y_body, jz);
            }
        } else {
            for (int i = 0; i < N_LEGS; ++i) {
                const bool swing_i = !gait::is_stance(t_trot, gait_cfg, a.gait.phase[i]);
                const raibert::RaibertOffset rb = td_latch.update(i, swing_i, rb_fresh);

                // 足端差动偏航转向速度叠加
                const double hip_x_i = parallel::robot::HIP_BODY_XY[i][0];
                const double hip_y_i = parallel::robot::HIP_BODY_XY[i][1];
                gait::GaitConfig leg_cfg = gait_cfg;
                const double yaw_k = gait_cfg.yaw_turn_scale * a.yawrate_cmd * t_stance;
                leg_cfg.step_x += yaw_k * (-hip_y_i);
                leg_cfg.step_y += yaw_k * ( hip_x_i);

                // 地形自适应足高修正
                if (a.gait.terrain_adapt) {
                    const double dz = std::clamp(
                        -ta_s_pitch * hip_x_i - ta_s_roll * hip_y_i,
                        -a.gait.terrain_adapt_max_m, a.gait.terrain_adapt_shallow_max_m);
                    leg_cfg.stance_z = gait_cfg.stance_z + dz;
                }

                body_tgt[i] = gait::trot_body_target_d(
                    t_trot, leg_cfg, legs[i].cal->stand_y_body, a.gait.phase[i],
                    rb.x, rb.y, body_vel_tgt[i], body_acc_tgt[i],
                    hip_x_i);
            }
        }

        // 跳跃完成恢复名义待机电流限
        if (jump_was_active && !jump_gait.active()) {
            for (int k = 0; k < N_LEGS; ++k)
                hw.set_leg_current_limit(k, jump_gait.cfg.idle_current_a);
        }
        jump_was_active = jump_gait.active();
    }

    // 站姿横向展宽补偿 (提升侧向倾覆稳定余量)
    if (a.gait.stand_y_widen != 0.0) {
        const double w = a.gait.stand_y_widen * std::min(1.0, t / a.startup_s);
        for (int i = 0; i < N_LEGS; ++i)
            body_tgt[i].y() += std::copysign(w, legs[i].cal->stand_y_body);
    }

    /* === 2. 运动学逆解与关节角度限位安全校验 === */
    std::array<Eigen::Vector3d, N_LEGS> raw_cmd;
    std::array<Eigen::Vector3d, N_LEGS> math_des;
    std::array<bool, N_LEGS> ik_ok{};
    for (int i = 0; i < N_LEGS; ++i) {
        Eigen::Vector3d out, mout;
        baseline::ControlChainDebug dbg;
        bool ok = baseline::control_chain(*legs[i].cal, kin, limits, body_tgt[i], legs[i].last_raw, out, mout, &dbg);
        raw_cmd[i]  = out;
        math_des[i] = mout;
        ik_ok[i]    = ok;
        if (!ok) {
            ++legs[i].ik_fail;
            if (dbg.fail_step >= 1 && dbg.fail_step <= 4) ++legs[i].cc_step[dbg.fail_step];
            legs[i].cc_max_jump  = std::max(legs[i].cc_max_jump,  dbg.jump);
            legs[i].cc_max_fkerr = std::max(legs[i].cc_max_fkerr, dbg.fk_err);
            if (dbg.fail_step == 2) {
                legs[i].cc_th3_min = std::min(legs[i].cc_th3_min, dbg.th3);
                legs[i].cc_th3_max = std::max(legs[i].cc_th3_max, dbg.th3);
                legs[i].cc_th1_min = std::min(legs[i].cc_th1_min, dbg.th1);
                legs[i].cc_th1_max = std::max(legs[i].cc_th1_max, dbg.th1);
                legs[i].cc_th2_min = std::min(legs[i].cc_th2_min, dbg.th2);
                legs[i].cc_th2_max = std::max(legs[i].cc_th2_max, dbg.th2);
            }
        }
    }

    /* === 2.4 计算各腿理论支撑相标志 === */
    const bool is_stand_mode =
        (a.gait.step_x == 0.0 && a.gait.step_y == 0.0 && a.gait.step_h == 0.0);
    std::array<bool, N_LEGS> is_stance{};
    int n_stance = 0;
    if (!in_trot || is_stand_mode || jump_gait.active()) {
        for (int i = 0; i < N_LEGS; ++i) is_stance[i] = true;
        n_stance = N_LEGS;
    } else {
        const double t_trot = t - a.startup_s;
        for (int i = 0; i < N_LEGS; ++i) {
            const double phase = std::fmod(t_trot / a.gait.cycle_s + a.gait.phase[i], 1.0);
            is_stance[i] = (phase < a.gait.stance_frac);
            if (is_stance[i]) ++n_stance;
        }
    }

    /* === 2.5 足端接触状态检测与浮动基状态估计 === */
    const double dt_s = 1.0 / std::max(10.0, a.rate_hz);
    std::array<double, N_LEGS> confidences{};
    std::array<bool,   N_LEGS> nominal_swing_arr{};
    std::array<Eigen::Vector3d, N_LEGS> foot_body_meas{};
    std::array<Eigen::Vector3d, N_LEGS> foot_vel_body_meas{};
    std::array<kinematics::FootMeas, N_LEGS> foot_meas{};

    for (int i = 0; i < N_LEGS; ++i) {
        const auto js = hw.read_joints(i);
        const MotorState& s1 = js[1];
        const MotorState& s2 = js[2];
        foot_meas[i] = kinematics::measure_foot(
            kin, *legs[i].cal,
            Eigen::Vector3d(js[0].pos, js[1].pos, js[2].pos),
            Eigen::Vector3d(js[0].vel, js[1].vel, js[2].vel),
            Eigen::Vector2d(parallel::robot::HIP_BODY_XY[i][0],
                            parallel::robot::HIP_BODY_XY[i][1]));
        const auto& fm = foot_meas[i];
        const Eigen::Vector3d& act_body = fm.p_body;
        foot_body_meas[i]     = fm.p_body;
        foot_vel_body_meas[i] = fm.v_body;

        bool nominal_swing = false;
        if (is_stand_mode || jump_gait.active())  nominal_swing = false;
        else if (in_trot) {
            const double t_trot = t - a.startup_s;
            const double phase = std::fmod(t_trot / a.gait.cycle_s + a.gait.phase[i], 1.0);
            nominal_swing = (phase >= a.gait.stance_frac);
        } else {
            nominal_swing = true;
        }
        nominal_swing_arr[i] = nominal_swing;

        // 基于广义力矩残差与运动学跟踪误差的多判据接触检测
        parallel::contact::ContactInput inp{};
        inp.nominal_swing = nominal_swing;
        inp.act_z_body    = act_body.z();
        inp.des_z_body    = body_tgt[i].z();
        inp.des_vz_body   = (body_tgt[i].z() - legs[i].prev_body_tgt_z) / dt_s;
        inp.tau1_meas     = s1.tor;
        inp.tau2_meas     = s2.tor;
        inp.tau1_ff       = legs[i].last_tau_ff[1];
        inp.tau2_ff       = legs[i].last_tau_ff[2];
        legs[i].last_est  = legs[i].detector.update(inp);
        if (legs[i].last_est.touchdown) ++legs[i].touchdown_count;

        if (!nominal_swing) {
            if (legs[i].last_est.sag_air) ++legs[i].sag_air_count;
            else                          ++legs[i].sag_contact_count;
        }
        legs[i].prev_body_tgt_z = body_tgt[i].z();
        confidences[i] = legs[i].last_est.confidence;
    }

    // 双阈值迟滞滤波生成多足鲁棒接触判定
    is_contact = contact_est.update(confidences, dt_s);

    // 浮动基状态观测器更新 (融合 IMU 与多足运动学测速)
    if (imu.is_running()) {
        const auto imu_snap = imu.get_state();
        body_est.update(imu_snap, foot_body_meas, foot_vel_body_meas, is_contact, dt_s);
    }

    // 支撑权重连续低通滤波 (一阶滞后平滑换相冲击)
    std::array<double, N_LEGS> support_weight{};
    const bool in_startup_phase = !in_trot;
    const double sw_alpha = dt_s / (0.015 + dt_s);
    for (int i = 0; i < N_LEGS; ++i) {
        double target;
        if (in_startup_phase || is_stand_mode || jump_gait.active()) {
            target = 1.0;
        } else {
            target = std::clamp(confidences[i], 0.0, 1.0);
            if (!is_stance[i]) target = std::min(target, a.force_alloc.swing_support_max);
        }
        legs[i].support_weight = sw_alpha * target + (1.0 - sw_alpha) * legs[i].support_weight;
        support_weight[i] = legs[i].support_weight;
    }

    /* === 2.6 机身空间扳手计算与足端接触力分配 === */
    const double grav_scale = a.baseline.ff_grav
        ? (std::min(1.0, t / std::max(0.1, a.startup_s)))
        : 0.0;

    std::array<Eigen::Vector3d, N_LEGS> hip_body{};
    for (int i = 0; i < N_LEGS; ++i) {
        const auto& hip_xy = parallel::robot::HIP_BODY_XY[i];
        hip_body[i] = Eigen::Vector3d(hip_xy[0], hip_xy[1], 0.0);
    }

    body_posture::WrenchDes wrench_des{};
    std::array<Eigen::Vector3d, N_LEGS> f_body_per_leg{};

    if (a.body_posture.enabled) {
        const double yawrate = a.yawrate_cmd;
        body_posture::BodyPostureConfig bp_cfg = resolve_posture_config(in_trot);
        wrench_des = body_posture::compute_wrench_des(
            bp_cfg, body_est.output(), yawrate, vz_filt_state);

        wrench_des.force  *= grav_scale;
        wrench_des.moment *= grav_scale;

        // 偏航稳态误差带泄漏积分补偿 (Leaky Anti-windup Integrator)
        if (in_trot) {
            double yaw_err = body_est.output().yaw - a.body_posture.yaw_des;
            while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
            while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
            const double yaw_i_leak = dt_s / 2.5;
            yaw_integral = std::clamp((1.0 - yaw_i_leak) * yaw_integral + yaw_err * dt_s,
                                      -0.3, 0.3);
            wrench_des.moment.z() -= a.body_posture.ki_yaw * yaw_integral * grav_scale;
            wrench_des.moment.z() = std::clamp(wrench_des.moment.z(),
                                               -a.body_posture.moment_clip,
                                               a.body_posture.moment_clip);
        } else {
            yaw_integral = 0.0;
        }

        // 跳跃蹬伸期垂直推力前馈与抗前栽俯仰力矩补偿
        if (jump_gait.active()
            && jump_gait.phase() == parallel::gait::JumpGait::Ph::THRUST) {
            wrench_des.force.z() += static_cast<double>(N_LEGS) * jump_gait.cfg.thrust_ff_n
                                    * jump_gait.thrust_ff_scale();
            wrench_des.moment.y() += jump_gait.cfg.jump_pitch_ff_nm * jump_gait.thrust_ff_scale();
        }

        // 地反力最优化分配 (全任务空间 WBC 二次规划或启发式分配)
        if (a.use_wbc) {
            wbc::WbcInput wbc_in{wrench_des, support_weight, foot_body_meas};
            wbc::WbcOutput wbc_out = wbc::solve(a.wbc, wbc_in);
            if (wbc_out.success) {
                f_body_per_leg = wbc_out.f_body;
            } else {
                f_body_per_leg = force_alloc::allocate(a.force_alloc, wrench_des, support_weight, hip_body);
            }
        } else {
            f_body_per_leg = force_alloc::allocate(a.force_alloc, wrench_des, support_weight, hip_body);
        }
    } else {
        // 开环重力前馈兜底模式
        double sum_w = 0.0;
        for (int i = 0; i < N_LEGS; ++i) sum_w += support_weight[i];
        const double F_z_total = grav_scale * a.body_posture.mass_kg * 9.81;
        for (int i = 0; i < N_LEGS; ++i) {
            f_body_per_leg[i].setZero();
            if (sum_w > 1e-6) f_body_per_leg[i].z() = F_z_total * support_weight[i] / sum_w;
        }
        wrench_des.force.z() = F_z_total;
    }

    /* === 2.7 足端笛卡尔空间阻抗控制与雅可比力矩映射 === */
    std::array<Eigen::Vector3d, N_LEGS> tau_ff{};
    const double dt_loop = 1.0 / std::max(10.0, a.rate_hz);
    force_control::ForceCtrlConfig force_cfg_eff = resolve_force_config();

    for (int i = 0; i < N_LEGS; ++i) {
        // 机身分配地反力一阶低通滤波
        const double fa = a.baseline.f_lpf_alpha;
        legs[i].F_body_filtered = fa * f_body_per_leg[i] + (1.0 - fa) * legs[i].F_body_filtered;

        const auto& cal = *legs[i].cal;
        const auto& fm = foot_meas[i];
        const Eigen::Vector3d& math_act    = fm.math;
        const Eigen::Vector3d& p_act_local = fm.p_local;
        Eigen::Vector3d        v_act_local = fm.v_local;

        // 足端实际运动速度低通滤波 (抑制编码器差分高频噪声)
        constexpr double V_ACT_LPF_A = 0.167;
        legs[i].v_act_filtered = V_ACT_LPF_A * v_act_local + (1.0 - V_ACT_LPF_A) * legs[i].v_act_filtered;
        v_act_local = legs[i].v_act_filtered;

        // 期望位置与速度生成
        Eigen::Vector3d p_des_local, v_des_local;
        if (ik_ok[i]) {
            kinematics::FKResult fk_des = kin.forward_kinematics(math_des[i][0], math_des[i][1], math_des[i][2]);
            p_des_local = Eigen::Vector3d(fk_des.act_x, fk_des.act_y, fk_des.act_z);
            if (!legs[i].vmc_seeded) {
                legs[i].prev_p_des_local = p_des_local;
                legs[i].vmc_seeded = true;
            }
            v_des_local = (p_des_local - legs[i].prev_p_des_local) / dt_loop;
            legs[i].prev_p_des_local = p_des_local;
        } else {
            // 逆解失效时的位置保持与速度置零容错策略 (避免力矩突变失稳)
            if (legs[i].vmc_seeded) {
                p_des_local = legs[i].prev_p_des_local;
            } else {
                p_des_local = p_act_local;
            }
            v_des_local.setZero();
        }

        // 步态闭式期望速度前馈注入
        if (use_analytic_ff && ik_ok[i]) {
            v_des_local = cal.body_to_local(body_vel_tgt[i]);
        }

        // 统一力控计算: 阻抗刚度/阻尼力 + 支撑相地反力 -> 雅可比转置映射为电机力矩
        tau_ff[i] = force_control::compute_leg_tau(
            force_cfg_eff, cal, kin, math_act,
            p_des_local, v_des_local,
            p_act_local, v_act_local,
            legs[i].F_body_filtered, support_weight[i],
            legs[i].F_int_local);

        // 积分饱和泄漏
        legs[i].F_int_local *= 0.995;
    }

    /* === 3. 执行器增益插补与交错切片通信调度 === */
    std::array<double, N_LEGS> leg_kp{}, leg_kd{};
    for (int i = 0; i < N_LEGS; ++i) {
        const double w = support_weight[i];
        leg_kp[i] = (1.0 - w) * a.baseline.kp_swing + w * a.baseline.kp_stance;
        leg_kd[i] = (1.0 - w) * a.baseline.kd_swing + w * a.baseline.kd_stance;
    }

    // 跳跃动作特定增益覆盖
    {
        const auto jph = jump_gait.phase();
        if (jph == parallel::gait::JumpGait::Ph::THRUST ||
            jph == parallel::gait::JumpGait::Ph::FLIGHT) {
            for (int i = 0; i < N_LEGS; ++i) {
                leg_kp[i] = a.baseline.kp_jump;
                leg_kd[i] = a.baseline.kd_jump;
            }
        }
    }

    // 控制环计算耗时探针
    {
        timespec t_s0;
        clock_gettime(CLOCK_MONOTONIC, &t_s0);
        const double c_ms = ((t_s0.tv_sec - next_tick.tv_sec) * 1e9
                           + (t_s0.tv_nsec - next_tick.tv_nsec)) / 1e6;
        comp_sum_ms += c_ms;
        comp_max_ms  = std::max(comp_max_ms, c_ms);
        if (c_ms > step_ns / 1e6)       ++comp_over_1;
        if (c_ms > 2.0 * step_ns / 1e6) ++comp_over_2;
    }

    // CAN 总线状态诊断与自愈/安全停机状态机
    {
        BusState worst = BusState::Active;
        std::array<BusState, N_LEGS> bs{};
        for (int i = 0; i < N_LEGS; ++i) {
            bs[i] = hw.bus_state(i);
            BusState eff = (bs[i] == BusState::Unknown) ? BusState::Active : bs[i];
            if (static_cast<int>(eff) > static_cast<int>(worst)) worst = eff;
        }
        const bool degraded = (worst == BusState::Passive ||
                               worst == BusState::BusOff  ||
                               worst == BusState::Down);

        if (degraded) {
            ++degraded_frames;
            if (degraded_since_t < 0) {
                degraded_since_t = t;
                ++degraded_episodes;
            }
            if (BUS_SAFEHOLD_ENABLED) {
                if ((t - degraded_since_t) >= SAFE_HOLD_DEBOUNCE_S) {
                    safehold_engaged = true;
                    for (int i = 0; i < N_LEGS; ++i) {
                        leg_kp[i]  = SAFE_HOLD_KP;
                        leg_kd[i]  = SAFE_HOLD_KD;
                        raw_cmd[i] = legs[i].last_raw;
                        tau_ff[i].setZero();
                    }
                }
                if (!estop_triggered && (t - degraded_since_t) > DEGRADED_ESTOP_S) {
                    estop_triggered = true;
                    std::fprintf(stderr,
                        "[DogControl] 紧急停机: CAN 总线降级持续超过 %.1fs，触发主动停机保护\n",
                        DEGRADED_ESTOP_S);
                    g_stop.store(true);
                }
            }
        } else {
            if (degraded_since_t >= 0) {
                degraded_since_t = -1.0;
                safehold_engaged = false;
            }
        }
    }

    // 3-Slot 交错时钟切片并发下发驱动指令 (降低总线瞬态峰值负载)
    timespec sub = next_tick;
    // Slot 0: 四腿 θ3 髋关节 Roll 电机
    hw.send_slot(0, raw_cmd, leg_kp, leg_kd, tau_ff);
    add_ns(sub, step_ns);
    sleep_until(sub);

    // Slot 1: 四腿 θ1 前五连杆主动臂电机
    hw.send_slot(1, raw_cmd, leg_kp, leg_kd, tau_ff);
    add_ns(sub, step_ns);
    sleep_until(sub);

    // Slot 2: 四腿 θ2 后五连杆主动臂电机
    hw.send_slot(2, raw_cmd, leg_kp, leg_kd, tau_ff);

    sent_frames += N_LEGS * 3;
    ++frames;

    for (int i = 0; i < N_LEGS; ++i) legs[i].last_tau_ff = tau_ff[i];

    // 逐帧二进制遥测记录 (Ring buffer 非阻塞无锁写入)
    if (telem) {
        const auto& est = body_est.output();
        parallel::telem::TelemRecord r{};
        r.t            = t;
        r.est_z        = static_cast<float>(est.pos_world.z());
        r.est_vz_world = static_cast<float>(est.vel_world.z());
        r.est_vx_body  = static_cast<float>(est.vel_body.x());
        r.est_vy_body  = static_cast<float>(est.vel_body.y());
        r.est_roll     = static_cast<float>(est.roll);
        r.est_pitch    = static_cast<float>(est.pitch);
        r.est_yaw      = static_cast<float>(std::remainder(est.yaw - a.body_posture.yaw_des, 2.0 * M_PI));
        r.wrench_fz    = static_cast<float>(wrench_des.force.z());
        r.wrench_mx    = static_cast<float>(wrench_des.moment.x());
        r.wrench_my    = static_cast<float>(wrench_des.moment.y());
        r.wrench_mz    = static_cast<float>(wrench_des.moment.z());
        float max_tau = 0.0f;
        for (int i = 0; i < N_LEGS; ++i) {
            r.support_w[i]   = static_cast<float>(support_weight[i]);
            r.fbody_z[i]     = static_cast<float>(f_body_per_leg[i].z());
            r.confidence[i]  = static_cast<float>(legs[i].last_est.confidence);
            r.body_tgt_z[i]  = static_cast<float>(body_tgt[i].z());
            r.foot_meas_z[i] = static_cast<float>(foot_body_meas[i].z());
            r.bus_state[i]   = static_cast<uint8_t>(hw.bus_state(i));

            const Eigen::Vector3d v_total = foot_vel_body_meas[i]
                                          + est.ang_vel_body.cross(foot_body_meas[i]);
            r.foot_body_vel[i] = static_cast<float>(v_total.norm());
            max_tau = std::max(max_tau, static_cast<float>(tau_ff[i].cwiseAbs().maxCoeff()));

            r.tau_leg[i][0] = static_cast<float>(tau_ff[i][0]);
            r.tau_leg[i][1] = static_cast<float>(tau_ff[i][1]);
            r.tau_leg[i][2] = static_cast<float>(tau_ff[i][2]);

            const auto jm = hw.read_joints(i);
            r.tau_meas[i][0] = static_cast<float>(jm[0].tor);
            r.tau_meas[i][1] = static_cast<float>(jm[1].tor);
            r.tau_meas[i][2] = static_cast<float>(jm[2].tor);
            if (is_contact[i]) r.contact_bits |= static_cast<uint8_t>(1u << i);
        }
        r.max_tau_ff = max_tau;
        r.n_contacts = static_cast<uint8_t>(est.n_active_contacts);
        r.valid      = est.valid ? 1 : 0;
        r.phase      = (t < a.startup_s) ? 0 : (is_stand_mode ? 3 : 1);
        telem->push(r);
    }

    // 维持主控制周期节拍 (PREEMPT_RT 单调绝对时钟等待)
    add_ns(next_tick, dt_ns);
    sleep_until(next_tick);
}

void DogControl::print_summary() {
    // 停止并落盘二进制遥测记录器
    if (telem) {
        telem->stop();
    }

    // 终止 SBUS 遥控接收线程
    auto& sbus_reader = hw.sbus();
    if (sbus_reader) {
        sbus_reader->stop();
    }
}

}  // namespace task
}  // namespace parallel
