/**
 * @file dog_control.hpp
 * @brief 自主导航四足机器人实时任务控制器 (DogControl)
 * 
 * 模块定位: Layer 4 任务编排层 (motor_test_cpp::task)
 * 职责描述:
 *   作为顶层实时控制周期的核心调度器，持有并管理单个控制拍 (tick) 内的全部管道流程:
 *   共享内存导航指令消费 -> 足端目标规划 -> 运动学逆解 -> 接触状态估计 -> 
 *   浮动基阻抗控制 -> 接触力分配 -> 笛卡尔 VMC 力矩解算 -> 3-Slot 时钟切片交错通信 -> 飞行遥测记录。
 */

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <ctime>
#include <memory>

#include <Eigen/Core>

#include "motor_test/common/pod_types.hpp"
#include "motor_test/common/telemetry_log.hpp"
#include "motor_test/estimation/contact_detector.hpp"
#include "motor_test/estimation/contact_estimator.hpp"
#include "motor_test/estimation/estimator.hpp"
#include "motor_test/planning/raibert.hpp"
#include "motor_test/driver/can_bridge.hpp"
#include "motor_test/kinematics/leg_calibration.hpp"
#include "motor_test/kinematics/leg_kinematics.hpp"
#include "motor_test/task/run_config.hpp"
#include "motor_test/task/hardware_manager.hpp"
#include "motor_test/task/command_source.hpp"
#include "motor_test/dynamics/body_posture.hpp"
#include "motor_test/dynamics/force_control.hpp"

namespace motor_test_cpp {
namespace task {

/**
 * @brief 单腿运行时控制上下文 (Leg Context)
 */
struct LegCtx {
    const robot::LegCalibration* cal = nullptr;   ///< 腿部几何结构与电机映射标定
    can_bridge::CanBridge* can = nullptr;         ///< 对应腿 SocketCAN 驱动句柄 (所有权在 HardwareManager)
    Eigen::Vector3d body0;                        ///< 启动初始时刻足端机身系坐标 (由 FK 算出)
    Eigen::Vector3d stance_center;                ///< 名义支撑中心足端机身系坐标
    Eigen::Vector3d last_raw{0, 0, 0};            ///< 上一周期物理关节角 (逆解奇异时的保底指令)
    int   ik_fail = 0;                            ///< 逆解失败总计数
    std::array<int, 5> cc_step{};                 ///< 控制链阶段失效统计: [1]=无解 [2]=超限 [3]=FK残差 [4]=跳变
    double cc_max_jump = 0.0, cc_max_fkerr = 0.0; ///< 控制链关节角最大阶跃与 FK 最大复核残差
    double cc_th3_min = 1e9, cc_th3_max = -1e9;   ///< 逆解关节超界监测范围
    double cc_th1_min = 1e9, cc_th1_max = -1e9;
    double cc_th2_min = 1e9, cc_th2_max = -1e9;
    int   rx0_th3 = 0, rx0_th1 = 0, rx0_th2 = 0;  ///< 各关节通信初始帧计数

    contact::ContactDetector detector{};          ///< 足端接触检测器 (基于力矩残差与运动学误差)
    double prev_body_tgt_z = 0.0;                 ///< 上一拍期望足端 Z 坐标 (用于数值差分速度估计)
    int    touchdown_count = 0;                   ///< 触地事件累计计数
    int    sag_air_count = 0;                      ///< 名义支撑相下沉悬空判定计数
    int    sag_contact_count = 0;                  ///< 名义支撑相真实受力触地计数
    contact::ContactEstimate last_est{};          ///< 最新一拍接触估计输出

    Eigen::Vector3d F_body_filtered{0.0, 0.0, 0.0}; ///< 滤波后的机身分配支撑力 (机身系)
    Eigen::Vector3d v_act_filtered {0.0, 0.0, 0.0}; ///< 滤波后的足端实际局部运动速度
    Eigen::Vector3d F_int_local     {0.0, 0.0, 0.0}; ///< 笛卡尔空间误差积分项 (带抗饱和泄漏)

    Eigen::Vector3d prev_p_des_local{0.0, 0.0, 0.0};///< 上一拍期望局部足端坐标
    bool            vmc_seeded = false;             ///< VMC 前馈初始化标志

    Eigen::Vector3d last_tau_ff{0.0, 0.0, 0.0};     ///< 上一拍输出前馈力矩
    double support_weight = 1.0;                    ///< 支撑权重连续因子 [0.0, 1.0]
};

/**
 * @brief 自主导航四足机器人实时任务主控类
 */
class DogControl {
public:
    DogControl(interface::RunConfig& a, HardwareManager& hw,
               kinematics::LegKinematics& kin, robot::JointLimits& limits,
               std::array<LegCtx, N_LEGS>& legs,
               estimator::Estimator& body_est,
               contact::ContactEstimator& contact_est,
               std::array<bool, N_LEGS>& is_contact,
               std::atomic<bool>& g_stop);

    /**
     * @brief 执行单拍实时控制循环 (500 Hz / PREEMPT_RT 周期节拍)
     */
    void tick();

    /**
     * @brief 执行控制器停止收尾与后台资源安全注销
     */
    void print_summary();

private:
    /**
     * @brief 根据当前步态模式解析机身姿态阻抗配置
     */
    body_posture::BodyPostureConfig resolve_posture_config(bool in_trot);

    /**
     * @brief 解析足端笛卡尔空间力控配置
     */
    force_control::ForceCtrlConfig  resolve_force_config();

    // ---- 外部共享资源引用 ----
    interface::RunConfig&            a;
    HardwareManager&                 hw;
    kinematics::LegKinematics&       kin;
    robot::JointLimits&              limits;
    std::array<LegCtx, N_LEGS>&      legs;
    estimator::Estimator&            body_est;
    contact::ContactEstimator&       contact_est;
    std::array<bool, N_LEGS>&        is_contact;
    std::atomic<bool>&               g_stop;

    // ---- 控制器内部运行时跨周期状态 ----
    int    frames = 0, sent_frames = 0;
    double comp_sum_ms = 0.0, comp_max_ms = 0.0;
    long   comp_over_1 = 0, comp_over_2 = 0;
    bool   startup_done_logged = false;
    bool   cmd_idle_logged = false;
    double eff_step_x = 0.0, eff_step_y = 0.0, eff_step_h = 0.0, eff_yaw = 0.0;
    double degraded_since_t  = -1.0;
    bool   safehold_engaged  = false;
    int    degraded_frames = 0, degraded_episodes = 0;
    bool   estop_triggered = false;
    bool   yaw0_latched = false;
    raibert::TouchdownLatch td_latch;
    double yaw_integral = 0.0;
    double ta_s_pitch = 0.0, ta_s_roll = 0.0;
    double stance_z_target = -0.23;
    double vz_filt_state = 0.0;
    bool   yaw_was_turning = false;
    const long dt_ns;
    const long step_ns;
    std::unique_ptr<telem::TelemetryLogger> telem;
    std::unique_ptr<ShmCommandSource> cmd_source_;
    timespec next_tick{};
    std::chrono::steady_clock::time_point t0;

    // ---- 总线通信降级保护阈值 ----
    static constexpr bool   BUS_SAFEHOLD_ENABLED = false;
    static constexpr double DEGRADED_ESTOP_S     = 3.0;
    static constexpr double SAFE_HOLD_KP         = 5.0;
    static constexpr double SAFE_HOLD_KD         = 1.5;
    static constexpr double SAFE_HOLD_DEBOUNCE_S = 0.06;
};

}  // namespace task
}  // namespace motor_test_cpp
