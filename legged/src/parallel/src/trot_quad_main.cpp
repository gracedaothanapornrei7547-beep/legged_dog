// =============================================================================
// 项目: parallel (12-DOF Parallel Quadruped Robot Control System)
// 文件: trot_quad_main.cpp
// 职责: 四足机器人主控制进程入口
//       - PREEMPT_RT 实时调度配置与 CPU 亲和性隔离
//       - 硬件总线 (SocketCAN/IMU/SBUS) 初始化与预检自检 (Preflight Check)
//       - 执行器批量使能校准 (Batch Motor Arming)
//       - 500Hz 确定性主控制循环调度 (DogControl Tick Loop)
//       - 系统信号中断与失能力矩平滑释放 (Robust Teardown)
// =============================================================================

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// POSIX / Linux 实时系统调用
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <time.h>

// 核心控制与硬件管理
#include "parallel/driver/can_bridge.hpp"
#include "parallel/estimation/contact_estimator.hpp"
#include "parallel/estimation/estimator.hpp"
#include "parallel/kinematics/leg_calibration.hpp"
#include "parallel/kinematics/leg_kinematics.hpp"
#include "parallel/task/dog_control.hpp"
#include "parallel/task/hardware_manager.hpp"
#include "parallel/task/run_config.hpp"

namespace {

using parallel::can_bridge::CanBridge;
using parallel::can_bridge::MotorState;
using parallel::kinematics::FKResult;
using parallel::kinematics::LegKinematics;
using parallel::robot::get_calibration;
using parallel::robot::JointLimits;
using parallel::robot::LegCalibration;
using parallel::robot::LEG_BRANCH;
using parallel::robot::LEG_GAMMA_RAD;
using parallel::robot::LEG_L1;
using parallel::robot::LEG_L2;
using parallel::robot::LEG_L3;
using parallel::robot::LEG_NAMES;
using parallel::task::LegCtx;
using parallel::N_LEGS;

// 全局安全停止标志
std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

// =============================================================================
// 系统实时性与调度配置 (Real-Time Configuration)
// =============================================================================

// 提升主线程至 SCHED_FIFO 实时调度，降低主控制循环的休眠唤醒抖动
void setup_realtime_scheduling(int prio = 80) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::fprintf(stderr, "[rt] mlockall 失败: %s (继续运行，无 memlock 保护)\n", std::strerror(errno));
    }
    struct sched_param sp{};
    sp.sched_priority = prio;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        std::fprintf(stderr, "[rt] SCHED_FIFO 实时调度设置失败: %s (降级为普通进程调度)\n", std::strerror(errno));
        return;
    }
    std::printf("[rt] 实时调度 SCHED_FIFO (优先级 %d) 及内存锁定已生效 ✓\n", prio);
}

// 核心线程亲和性绑定：主控制线程绑定 CPU3，隔离非实时任务保障 500Hz 确定性执行
void setup_cpu_affinity(int cpu_core = 3) {
    cpu_set_t cs;
    CPU_ZERO(&cs);
    CPU_SET(cpu_core, &cs);
    if (sched_setaffinity(0, sizeof(cs), &cs) != 0) {
        std::fprintf(stderr, "[rt] 主线程 CPU 亲和性绑定失败: %s (继续运行)\n", std::strerror(errno));
        return;
    }
    std::printf("[rt] 线程调度亲和性绑定成功: 控制线程绑定 CPU%d ✓\n", cpu_core);
}

// =============================================================================
// 通信重置与日志初始化
// =============================================================================

// 启动前复位 CAN 适配器，清除错误计数器 (TEC/REC) 与历史错误状态，确保通信处于初始健康状态
void reset_can_interfaces_if_configured(const parallel::interface::RunConfig& config) {
    if (!config.can_reset_on_start) return;

    const char* env_script = std::getenv("CAN_RESET_SCRIPT");
    std::string script_path;
    if (env_script && *env_script) {
        script_path = env_script;
    } else if (std::filesystem::exists("scripts/can-reset.sh")) {
        script_path = "scripts/can-reset.sh";
    }

    if (script_path.empty()) {
        std::fprintf(stderr, "[trot_quad] 提示: 未检测到 can-reset.sh 脚本，跳过通信硬件重置。\n");
    } else {
        std::fprintf(stderr, "[trot_quad] 执行通信硬件重置: sudo %s\n", script_path.c_str());
        std::string cmd = "sudo " + script_path + " 1>&2";
        int rc = std::system(cmd.c_str());
        if (rc != 0) {
            std::fprintf(stderr, "[trot_quad] 重置脚本返回码=%d，将在自检阶段再次验证总线状态。\n", rc);
        } else {
            std::fprintf(stderr, "[trot_quad] CAN 硬件重置成功 ✓\n");
        }
    }
}

// 初始化运行日志与二进制遥测文件路径
void setup_logging(parallel::interface::RunConfig& config) {
    std::time_t now = std::time(nullptr);
    std::tm tmv{};
    localtime_r(&now, &tmv);
    char ts[24];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tmv);

    std::error_code ec;
    std::filesystem::create_directories("log", ec);
    if (config.log_file.empty())  config.log_file  = std::string("log/trot_") + ts + ".log";
    if (config.telem_log.empty()) config.telem_log = std::string("log/trot_") + ts + ".bin";

    if (!config.print_to_terminal) {
        if (std::freopen(config.log_file.c_str(), "w", stdout)) {
            std::setvbuf(stdout, nullptr, _IOLBF, 0);
            std::fprintf(stderr, "[log] 运行日志 -> %s ; 逐帧遥测 -> %s\n",
                         config.log_file.c_str(), config.telem_log.c_str());
        } else {
            std::fprintf(stderr, "[log] 警告: 无法打开日志文件 %s (%s)，回退至终端输出\n",
                         config.log_file.c_str(), std::strerror(errno));
        }
    } else {
        std::fprintf(stderr, "[log] 终端输出模式开启；逐帧遥测写入 %s\n", config.telem_log.c_str());
    }
}

// =============================================================================
// 执行器底层读写与使能管理
// =============================================================================

// 读取单腿三轴电机编码器原始弧度值
bool read_leg_feedback(CanBridge& can, const LegCalibration& cal,
                       double& raw3, double& raw1, double& raw2) {
    MotorState s3 = can.get_motor_state(cal.motor_id_th3);
    MotorState s1 = can.get_motor_state(cal.motor_id_th1);
    MotorState s2 = can.get_motor_state(cal.motor_id_th2);
    if (s3.rx_count == 0 || s1.rx_count == 0 || s2.rx_count == 0) return false;
    raw3 = s3.pos; raw1 = s1.pos; raw2 = s2.pos;
    return true;
}

// 电机安全失能：每个通道间隔 2ms 离散发送，整组重发 3 轮，确保各轴固件均可靠接收指令并释放力矩
void disable_all_motors_robust(std::array<LegCtx, N_LEGS>& legs) {
    for (auto& leg : legs) {
        leg.F_int_local.setZero();  // 失能前复位笛卡尔积分力，避免接触力残留引起突跳
    }
    for (int round = 0; round < 3; ++round) {
        for (auto& leg : legs) {
            leg.can->private_disable(leg.cal->motor_id_th3);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            leg.can->private_disable(leg.cal->motor_id_th1);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            leg.can->private_disable(leg.cal->motor_id_th2);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

// 批量使能 12 轴电机并执行基于正运动学的初始位形校准
bool arm_and_calibrate_legs(std::array<LegCtx, N_LEGS>& legs,
                            const LegKinematics& kin,
                            const parallel::interface::RunConfig& config) {
    constexpr int MAX_ARM_RETRIES = 3;
    std::array<bool, N_LEGS> leg_ok{};

    // 单腿测试模式下，非测试腿直接标记为就绪并赋予标称站立位形
    for (int i = 0; i < N_LEGS; ++i) {
        if (config.only_leg >= 0 && i != config.only_leg) {
            leg_ok[i] = true;
            legs[i].stance_center = {0.0, legs[i].cal->stand_y_body, config.gait.stance_z};
            legs[i].body0 = legs[i].stance_center;
        }
    }

    bool all_read = false;
    for (int retry = 0; retry < MAX_ARM_RETRIES && !all_read; ++retry) {
        if (retry > 0) {
            std::fprintf(stderr, "[arm] 重试第 %d/%d 次使能未就绪腿部: ", retry, MAX_ARM_RETRIES - 1);
            for (int i = 0; i < N_LEGS; ++i) {
                if (!leg_ok[i]) std::fprintf(stderr, "%s ", LEG_NAMES[i].data());
            }
            std::fprintf(stderr, "\n");
        }

        // 收集尚未响应使能的电机
        std::vector<std::pair<CanBridge*, int>> to_arm;
        for (int i = 0; i < N_LEGS; ++i) {
            if (leg_ok[i]) continue;
            to_arm.push_back({legs[i].can, legs[i].cal->motor_id_th3});
            to_arm.push_back({legs[i].can, legs[i].cal->motor_id_th1});
            to_arm.push_back({legs[i].can, legs[i].cal->motor_id_th2});
        }
        parallel::can_bridge::arm_private_vmc_batch(to_arm);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        // 验证各腿反馈并执行初始几何正运动学解算
        all_read = true;
        for (int i = 0; i < N_LEGS; ++i) {
            if (leg_ok[i]) continue;
            double raw3 = 0.0, raw1 = 0.0, raw2 = 0.0;
            if (!read_leg_feedback(*legs[i].can, *legs[i].cal, raw3, raw1, raw2)) {
                if (retry == MAX_ARM_RETRIES - 1) {
                    const auto& cal = *legs[i].cal;
                    auto dead = [&](int id) {
                        return legs[i].can->get_motor_state(id).rx_count == 0;
                    };
                    std::fprintf(stderr, "❌ [%s] 未接收到反馈 (离线电机:", LEG_NAMES[i].data());
                    if (dead(cal.motor_id_th3)) std::fprintf(stderr, " th3=%d", cal.motor_id_th3);
                    if (dead(cal.motor_id_th1)) std::fprintf(stderr, " th1=%d", cal.motor_id_th1);
                    if (dead(cal.motor_id_th2)) std::fprintf(stderr, " th2=%d", cal.motor_id_th2);
                    std::fprintf(stderr, ")\n");
                }
                all_read = false;
                continue;
            }

            legs[i].last_raw = {raw3, raw1, raw2};
            Eigen::Vector3d math0 = legs[i].cal->raw_to_math_angles(raw3, raw1, raw2);
            FKResult fk0 = kin.forward_kinematics(math0[0], math0[1], math0[2]);
            legs[i].body0 = legs[i].cal->local_to_body({fk0.act_x, fk0.act_y, fk0.act_z});
            legs[i].stance_center = {0.0, legs[i].cal->stand_y_body, config.gait.stance_z};
            legs[i].rx0_th3 = legs[i].can->get_motor_state(legs[i].cal->motor_id_th3).rx_count;
            legs[i].rx0_th1 = legs[i].can->get_motor_state(legs[i].cal->motor_id_th1).rx_count;
            legs[i].rx0_th2 = legs[i].can->get_motor_state(legs[i].cal->motor_id_th2).rx_count;

            std::printf("  [%s]%s 初始足端 body0=(%+.3f, %+.3f, %+.3f) -> 标称站姿 stance=(%+.3f, %+.3f, %+.3f)\n",
                        LEG_NAMES[i].data(), retry > 0 ? " (重试成功)" : "",
                        legs[i].body0.x(), legs[i].body0.y(), legs[i].body0.z(),
                        legs[i].stance_center.x(), legs[i].stance_center.y(), legs[i].stance_center.z());
            leg_ok[i] = true;
        }
    }
    return all_read;
}

}  // namespace

// =============================================================================
// 主控制进程入口 (Main Entrypoint)
// =============================================================================

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    std::signal(SIGINT, on_sigint);

    // 1. 系统参数聚合配置与硬件接口复位
    parallel::interface::RunConfig config;
    reset_can_interfaces_if_configured(config);
    setup_logging(config);

    // 2. 实时内核调度与 CPU 亲和性绑定 (CPU3 用于 500Hz 控制主线程)
    setup_realtime_scheduling(80);
    setup_cpu_affinity(3);

    // 3. 运动学引擎与状态估计器实例化
    LegKinematics kin(LEG_L1, LEG_L2, LEG_L3, LEG_GAMMA_RAD, LEG_BRANCH);
    JointLimits limits;
    parallel::contact::ContactEstimator contact_est{};
    std::array<bool, N_LEGS> is_contact{};
    parallel::estimator::Estimator body_est{};

    // 4. 硬件管理器初始化与传感器后台服务启动
    parallel::task::HardwareManager hw;
    auto& imu = hw.imu();
    if (imu.start()) {
        std::printf("[imu] 惯性测量单元已连接 (/dev/wheeltec_IMU @ 460800) ✓\n");
    } else {
        std::fprintf(stderr, "[imu] 警告: IMU 串口打开失败，系统将继续在纯运动学状态下运行\n");
    }

    // 5. 4 腿 CAN 通道建立与标定参数注入
    std::array<LegCtx, N_LEGS> legs{};
    for (int i = 0; i < N_LEGS; ++i) {
        legs[i].cal = &parallel::robot::get_calibration(i);
        legs[i].can = hw.make_can(i);
        hw.set_calibration(i, legs[i].cal);

        if (!legs[i].can->connect()) {
            if (config.only_leg >= 0 && i != config.only_leg) {
                continue;  // 单腿测试模式下放行未连通道
            }
            std::fprintf(stderr, "❌ [%s] CAN 通道连接失败 (%s)\n",
                         LEG_NAMES[i].data(), parallel::task::LEG_CAN_CHANNEL[i]);
            return 1;
        }
        std::printf("  [%s] 通道=%s 电机ID=[%d, %d, %d] 坐标极性=[%+0.0f, %+0.0f, %+0.0f]\n",
                    LEG_NAMES[i].data(), parallel::task::LEG_CAN_CHANNEL[i],
                    legs[i].cal->motor_id_th3, legs[i].cal->motor_id_th1, legs[i].cal->motor_id_th2,
                    legs[i].cal->x_sign, legs[i].cal->y_sign, legs[i].cal->z_sign);
    }
    std::printf("\n");

    // 6. 硬件上电前自检 (Preflight Check: 总线状态与端口映射完整性)
    if (int rc = hw.preflight(config.only_leg)) {
        return rc;
    }

    // 7. 全关节电机批量使能与编码器零位校准
    if (!arm_and_calibrate_legs(legs, kin, config)) {
        std::fprintf(stderr, "❌ 初始化失败: 部分腿在多次重试后仍未接收到电机编码器反馈，请检查总线通信或电源供电。\n");
        disable_all_motors_robust(legs);
        return 1;
    }

    // 8. 启动任务层主控制器 (500Hz 实时循环)
    std::printf("\n启动序列开始: 缓起立时间 %.1fs 插值至站姿 stance，随后进入 trot 步态；按 Ctrl+C 安全停止\n\n", config.startup_s);
    parallel::task::DogControl controller{config, hw, kin, limits, legs,
                                              body_est, contact_est, is_contact, g_stop};
    while (!g_stop.load()) {
        controller.tick();
    }
    controller.print_summary();

    // 9. 停机安全保护与总线释放
    std::printf("\n正在执行 12 轴电机安全失能 (离散 3 轮确认发送) ...\n");
    disable_all_motors_robust(legs);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    for (auto& leg : legs) {
        leg.can->disconnect();
    }

    std::printf("✓ 控制器正常退出，电机安全失能完成\n");
    return 0;
}
