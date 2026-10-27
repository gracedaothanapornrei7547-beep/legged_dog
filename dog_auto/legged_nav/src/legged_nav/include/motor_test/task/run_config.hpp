/**
 * @file run_config.hpp
 * @brief 机器人全局运行时配置与参数聚合定义 (RunConfig)
 * 
 * 模块定位: Layer 4 任务配置层 (motor_test_cpp::interface)
 * 职责描述:
 *   集中汇聚控制系统各子模块的配置结构体 (步态规划、逆运动学连续性约束、
 *   虚拟模型力控、机身姿态全向空间阻抗、接触力二次规划分配、Raibert 落脚点外推及运动指令整形)，
 *   并维护主时钟周期、通信重置与数据记录等运行时全局参数。
 */

#pragma once

#include <string>

#include "motor_test/planning/gait.hpp"
#include "motor_test/planning/raibert.hpp"
#include "motor_test/kinematics/baseline.hpp"
#include "motor_test/dynamics/body_posture.hpp"
#include "motor_test/dynamics/force_alloc.hpp"
#include "motor_test/dynamics/force_control.hpp"

namespace motor_test_cpp {
namespace interface {

/**
 * @brief 运动指令滤波与动态整形参数 (Motion Shaping)
 */
struct MotionShaping {
    bool   smooth_stop     = true;    ///< 指令超时/失步时开启一阶平滑减速归零 (避免急停机械冲击)
    double home_s          = 0.35;    ///< 平滑收停过渡时间常数 (s)
    double height_slew_mps = 0.12;    ///< 站姿高度切换斜坡限速 (m/s)
    double nav_step_h      = 0.06;    ///< 自主导航步态固定摆动足抬腿高度 (m)
    double vel_to_step     = 0.5;     ///< 机体线速度至足端步幅映射线性增益
    double max_step_x      = 0.15;    ///< 纵向最大安全步幅限制 (m)
    double max_step_y      = 0.15;    ///< 横向最大安全步幅限制 (m)
    double max_yawrate     = 1.0;     ///< 航向转向角速度最大限制 (rad/s)
};

/**
 * @brief 机器人整机全系统运行时配置结构体
 */
struct RunConfig {
    // ---- 各子系统配置 ----
    gait::GaitConfig                gait{};           ///< 步态周期与占空比配置
    baseline::BaselineConfig        baseline{};       ///< 控制链连续性阈值与执行器刚度
    force_control::ForceCtrlConfig  force_ctrl{};     ///< 笛卡尔空间阻抗刚度与力矩限幅
    body_posture::BodyPostureConfig body_posture{};   ///< 机身浮动基姿态空间扳手控制器配置
    force_alloc::ForceAllocConfig   force_alloc{};    ///< 接触力分配与摩擦锥约束参数
    raibert::RaibertConfig          raibert{};        ///< Raibert 落地外推与捕获点增益
    MotionShaping                   motion{};         ///< 运动指令平滑整形配置

    // ---- 运行时实时运动指令 (机身系) ----
    double vcmd_x_body = 0.0;
    double vcmd_y_body = 0.0;
    double yawrate_cmd = 0.0;

    // ---- 实时控制时钟与调度参数 ----
    double rate_hz    = 500.0;    ///< 控制环与 CAN 发送调度频率 (Hz)
    double startup_s  = 3.0;      ///< 缓启动余弦平滑过渡时长 (s)
    int    only_leg   = -1;       ///< 支链调试模式 (-1: 全部四腿协同; 0..3: 指定单腿调试)

    // ---- 飞行遥测与运行日志 ----
    std::string telem_log;                 ///< 逐帧二进制遥测日志路径 (.bin)
    std::string log_file;                  ///< 运行状态文本输出重定向路径 (.log)
    bool        print_to_terminal = false; ///< 是否保持终端控制台输出 (调试模式)

    // ---- 总线通信初始化 ----
    bool        can_reset_on_start = true; ///< 启动时自动调用底层总线重置脚本清空错误计数
};

}  // namespace interface
}  // namespace motor_test_cpp
