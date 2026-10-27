/**
 * @file nav_quad_main.cpp
 * @brief 自主导航四足机器人顶层实时主程序 (Nav Quad Main)
 * 
 * 系统架构与执行管道:
 * 1. 信号处理与命令行解析 (Signal Handling & Arguments)
 * 2. SocketCAN 网络接口拓扑锁定与底层重置 (CAN Device Reset & Configuration)
 * 3. 飞行遥测与运行日志重定向 (Flight Telemetry & Logging Setup)
 * 4. Linux PREEMPT_RT 实时调度与 CPU 核心亲和性配置 (Real-time Scheduling & CPU Pinning)
 * 5. 运动学解算器、多接触状态估计器与硬件管理器实例化 (Kinematics & Estimators Initialization)
 * 6. SocketCAN 链路建立与上电通信预检 (CAN Connection & Hardware Preflight Check)
 * 7. 十二电机批量流水线使能与初始位姿标定 (Pipeline Motor Arming & Pose Calibration)
 * 8. 实时闭环主控制循环调度 (Real-time Closed-loop Control Loop Execution)
 * 9. 安全停机与外设资源释放 (Shutdown Sequence & Hardware Disconnection)
 */

#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <sched.h>
#include <sys/mman.h>

#include "motor_test/kinematics/baseline.hpp"
#include "motor_test/dynamics/body_posture.hpp"
#include "motor_test/estimation/contact_detector.hpp"
#include "motor_test/estimation/contact_estimator.hpp"
#include "motor_test/estimation/estimator.hpp"
#include "motor_test/driver/can_bridge.hpp"
#include "motor_test/task/run_config.hpp"
#include "motor_test/task/hardware_manager.hpp"
#include "motor_test/task/dog_control.hpp"
#include "motor_test/kinematics/leg_calibration.hpp"
#include "motor_test/kinematics/leg_kinematics.hpp"

namespace {

using motor_test_cpp::can_bridge::CanBridge;
using motor_test_cpp::can_bridge::MotorState;
using motor_test_cpp::kinematics::LegKinematics;
using motor_test_cpp::kinematics::FKResult;
using motor_test_cpp::robot::LegCalibration;
using motor_test_cpp::robot::LEG_L1;
using motor_test_cpp::robot::LEG_L2;
using motor_test_cpp::robot::LEG_L3;
using motor_test_cpp::robot::LEG_GAMMA_RAD;
using motor_test_cpp::robot::LEG_BRANCH;
using motor_test_cpp::robot::LEG_NAMES;
using motor_test_cpp::N_LEGS;
using motor_test_cpp::robot::JointLimits;
using motor_test_cpp::task::LegCtx;

std::atomic<bool> g_stop{false};

void on_sigint(int) {
    g_stop.store(true);
}

/**
 * @brief 启用 Linux PREEMPT_RT 实时调度策略并锁定当前及未来内存页
 * @param prio 实时 FIFO 优先级 (默认 80)
 */
void setup_realtime_scheduling(int prio = 80) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::fprintf(stderr, "[rt] 警告: 内存锁定 (mlockall) 失败: %s (降级为无物理内存锁定运行)\n",
                     std::strerror(errno));
    }
    struct sched_param sp{};
    sp.sched_priority = prio;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        std::fprintf(stderr, "[rt] 警告: SCHED_FIFO 实时调度设置失败: %s (降级为普通 SCHED_OTHER 运行)\n",
                     std::strerror(errno));
        return;
    }
    std::printf("[rt] SCHED_FIFO 实时调度已配置 (prio=%d) 并锁定内存页 ✓\n", prio);
}

/**
 * @brief 绑定主控制线程 CPU 核心亲和性 (默认绑定至 CPU3，隔离系统中断与外设通信线程)
 */
void setup_cpu_affinity(int cpu = 3) {
    cpu_set_t cs;
    CPU_ZERO(&cs);
    CPU_SET(cpu, &cs);
    if (sched_setaffinity(0, sizeof(cs), &cs) != 0) {
        std::fprintf(stderr, "[rt] 警告: 主控制线程设置 CPU%d 亲和性失败: %s\n",
                     cpu, std::strerror(errno));
        return;
    }
    std::printf("[rt] 主控制线程已绑定至 CPU%d ✓\n", cpu);
}

/**
 * @brief 启动前重置与配置 CAN 总线链路状态 (清空 TEC/REC 并重置适配器)
 */
void reset_can_interfaces_if_configured(const motor_test_cpp::interface::RunConfig& a) {
    const char* env_lock = std::getenv("CAN_LOCK_SCRIPT");
    std::string lock_path;
    if (env_lock && *env_lock) {
        lock_path = env_lock;
    } else if (std::filesystem::exists("scripts/lock-can-by-usb.sh")) {
        lock_path = "scripts/lock-can-by-usb.sh";
    }
    if (!lock_path.empty()) {
        std::string cmd = "sudo " + lock_path + " 1>&2";
        std::system(cmd.c_str());
    }

    if (std::filesystem::exists("scripts/setup-can.sh")) {
        std::string cmd = "sudo scripts/setup-can.sh 1>&2";
        std::system(cmd.c_str());
    }

    if (a.can_reset_on_start) {
        const char* env_reset = std::getenv("CAN_RESET_SCRIPT");
        std::string script_path;
        if (env_reset && *env_reset) {
            script_path = env_reset;
        } else if (std::filesystem::exists("scripts/can-reset.sh")) {
            script_path = "scripts/can-reset.sh";
        }

        if (!script_path.empty()) {
            std::string cmd = "sudo " + script_path + " 1>&2";
            int rc = std::system(cmd.c_str());
            if (rc != 0) {
                std::fprintf(stderr, "[CAN] 警告: 底层总线重置脚本退出码异常 (rc=%d)\n", rc);
            }
        }
    }
}

/**
 * @brief 配置文件日志重定向与飞行遥测数据存储路径
 */
void setup_logging(motor_test_cpp::interface::RunConfig& a) {
    std::time_t now = std::time(nullptr);
    std::tm tmv{};
    localtime_r(&now, &tmv);
    char ts[24];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tmv);
    std::error_code ec;
    std::filesystem::create_directories("log", ec);
    if (a.log_file.empty())  a.log_file  = std::string("log/nav_") + ts + ".log";
    if (a.telem_log.empty()) a.telem_log = std::string("log/nav_") + ts + ".bin";

    if (!a.print_to_terminal) {
        if (std::freopen(a.log_file.c_str(), "w", stdout)) {
            std::setvbuf(stdout, nullptr, _IOLBF, 0);
            std::fprintf(stderr, "[log] 运行日志 -> %s; 逐帧遥测 -> %s\n",
                         a.log_file.c_str(), a.telem_log.c_str());
        } else {
            std::fprintf(stderr, "[log] 警告: 无法打开日志文件 %s，保持标准控制台输出\n",
                         a.log_file.c_str());
        }
    }
}

/**
 * @brief 读取指定腿的三个驱动电机位置反馈状态
 */
bool read_three(CanBridge& can, const LegCalibration& cal,
                double& raw3, double& raw1, double& raw2) {
    MotorState s3 = can.get_motor_state(cal.motor_id_th3);
    MotorState s1 = can.get_motor_state(cal.motor_id_th1);
    MotorState s2 = can.get_motor_state(cal.motor_id_th2);
    if (s3.rx_count == 0 || s1.rx_count == 0 || s2.rx_count == 0) return false;
    raw3 = s3.pos; raw1 = s1.pos; raw2 = s2.pos;
    return true;
}

/**
 * @brief 批量使能全机关节电机并完成初始位姿标定
 */
bool arm_and_calibrate_legs(std::array<LegCtx, N_LEGS>& legs,
                            const LegKinematics& kin,
                            const motor_test_cpp::interface::RunConfig& a) {
    constexpr int MAX_ARM_RETRIES = 3;
    std::array<bool, N_LEGS> leg_ok{};

    for (int i = 0; i < N_LEGS; ++i) {
        if (a.only_leg >= 0 && i != a.only_leg) {
            leg_ok[i] = true;
            legs[i].stance_center = {0.0, legs[i].cal->stand_y_body, a.gait.stance_z};
            legs[i].body0 = legs[i].stance_center;
        }
    }

    bool all_read = false;
    for (int retry = 0; retry < MAX_ARM_RETRIES && !all_read; ++retry) {
        std::vector<std::pair<CanBridge*, int>> to_arm;
        for (int i = 0; i < N_LEGS; ++i) {
            if (leg_ok[i]) continue;
            to_arm.push_back({legs[i].can, legs[i].cal->motor_id_th3});
            to_arm.push_back({legs[i].can, legs[i].cal->motor_id_th1});
            to_arm.push_back({legs[i].can, legs[i].cal->motor_id_th2});
        }
        motor_test_cpp::can_bridge::arm_private_vmc_batch(to_arm);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        all_read = true;
        for (int i = 0; i < N_LEGS; ++i) {
            if (leg_ok[i]) continue;
            double raw3, raw1, raw2;
            if (!read_three(*legs[i].can, *legs[i].cal, raw3, raw1, raw2)) {
                all_read = false;
                continue;
            }
            legs[i].last_raw = {raw3, raw1, raw2};
            Eigen::Vector3d math0 = legs[i].cal->raw_to_math_angles(raw3, raw1, raw2);
            FKResult fk0 = kin.forward_kinematics(math0[0], math0[1], math0[2]);
            legs[i].body0 = legs[i].cal->local_to_body({fk0.act_x, fk0.act_y, fk0.act_z});
            legs[i].stance_center = {0.0, legs[i].cal->stand_y_body, a.gait.stance_z};
            legs[i].rx0_th3 = legs[i].can->get_motor_state(legs[i].cal->motor_id_th3).rx_count;
            legs[i].rx0_th1 = legs[i].can->get_motor_state(legs[i].cal->motor_id_th1).rx_count;
            legs[i].rx0_th2 = legs[i].can->get_motor_state(legs[i].cal->motor_id_th2).rx_count;
            leg_ok[i] = true;
        }
    }
    return all_read;
}

/**
 * @brief 鲁棒多轮延时安全失能全机各关节电机
 */
template <typename Legs>
void disable_all_motors_robust(Legs& legs) {
    for (auto& leg : legs) leg.F_int_local.setZero();
    for (int round = 0; round < 3; ++round) {
        for (auto& leg : legs) {
            if (!leg.can) continue;
            leg.can->private_disable(leg.cal->motor_id_th3);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            leg.can->private_disable(leg.cal->motor_id_th1);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            leg.can->private_disable(leg.cal->motor_id_th2);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    // 1. 信号处理与命令行配置初始化
    motor_test_cpp::interface::RunConfig a;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        if (k == "-h" || k == "--help") {
            return 0;
        }
    }
    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);
    std::signal(SIGHUP,  on_sigint);

    // 2. 底层总线重置与通信端口配置
    reset_can_interfaces_if_configured(a);

    // 3. 文件日志与实时遥测配置
    setup_logging(a);

    // 4. PREEMPT_RT 实时调度及 CPU 亲和性隔离
    setup_realtime_scheduling(80);
    setup_cpu_affinity(3);

    // 5. 核心模型、状态观测器与硬件管理器实例化
    LegKinematics kin(LEG_L1, LEG_L2, LEG_L3, LEG_GAMMA_RAD, LEG_BRANCH);
    JointLimits limits;
    motor_test_cpp::contact::ContactEstimator contact_est{};
    std::array<bool, N_LEGS> is_contact{};
    motor_test_cpp::estimator::Estimator body_est{};

    motor_test_cpp::task::HardwareManager hw;
    auto& imu = hw.imu();
    if (!imu.start()) {
        std::fprintf(stderr, "[imu] 警告: IMU 惯导连接失败，系统将继续在无 IMU 状态下运行\n");
    }

    // 6. 建立 SocketCAN 通信链路并执行硬件环境预检
    std::array<LegCtx, N_LEGS> legs;
    for (int i = 0; i < N_LEGS; ++i) {
        legs[i].cal = &motor_test_cpp::robot::get_calibration(i);
        legs[i].can = hw.make_can(i);
        hw.set_calibration(i, legs[i].cal);
        if (!legs[i].can->connect()) {
            if (a.only_leg >= 0 && i != a.only_leg) continue;
            std::fprintf(stderr, "[CAN] 错误: 支链 %s SocketCAN 建立失败 (%s)\n",
                         LEG_NAMES[i].data(), motor_test_cpp::task::LEG_CAN_CHANNEL[i]);
            return 1;
        }
    }

    if (int rc = hw.preflight(a.only_leg)) {
        return rc;
    }

    // 7. 流水线批量使能十二电机并校验初始位姿
    if (!arm_and_calibrate_legs(legs, kin, a)) {
        std::fprintf(stderr, "[arm] 错误: 部分电机未获得反馈，已触发安全终止\n");
        disable_all_motors_robust(legs);
        return 1;
    }

    // 8. 实时闭环主控制循环调度
    motor_test_cpp::task::DogControl ctrl{a, hw, kin, limits, legs,
                                          body_est, contact_est, is_contact, g_stop};
    while (!g_stop.load()) {
        ctrl.tick();
    }

    // 9. 执行控制任务收尾 (遥测落盘、硬件停机与资源释放)
    ctrl.print_summary();
    disable_all_motors_robust(legs);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    for (auto& leg : legs) {
        if (leg.can) leg.can->disconnect();
    }

    return 0;
}
