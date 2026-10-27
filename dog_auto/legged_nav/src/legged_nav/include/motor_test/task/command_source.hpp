/**
 * @file command_source.hpp
 * @brief 自主导航底盘运动指令输入接口与共享内存 (SHM) 消费器
 * 
 * 模块定位: Layer 4 任务指令层 (motor_test_cpp::task)
 * 职责描述:
 *   实现基于 POSIX 共享内存 (Shared Memory) 的进程级跨域通信隔离架构。
 *   实时控制进程以零拷贝、无系统调用、无锁 Seqlock 快照方式从共享内存区域
 *   读取上层 ROS2 导航算法 (SLAM / Nav2) 发送的机身三自由度速度指令 (Vx, Vy, ω)，
 *   并将其线性映射为支撑相足端摆动位移 (Foot Displacement) 与差动偏航角速度。
 */

#pragma once

#include "motor_test/task/nav_cmd_shm.hpp"

namespace motor_test_cpp {
namespace task {

/**
 * @brief 机器人底盘瞬态运动指令结构体
 */
struct ChassisCommand {
    bool   fresh        = false;   ///< 指令有效性标志 (通过 Seqlock 与看门狗校验)
    bool   gait_enabled = false;   ///< 步态执行使能状态 (True: 持续行走; False: 保持站立)
    double step_x   = 0.0;         ///< 纵向期望步幅 (m)
    double step_y   = 0.0;         ///< 侧向期望步幅 (m)
    double yawrate  = 0.0;         ///< 期望自转角速度 (rad/s)
    double step_h   = 0.0;         ///< 摆动相最大抬腿高度 (m)
    double stance_z = -0.23;       ///< 名义支撑相站姿机身高度 (m)
};

/**
 * @brief 基于 POSIX 共享内存的跨进程导航指令消费器
 */
class ShmCommandSource {
public:
    /**
     * @brief 构造导航共享内存接收接口
     * @param step_per_v 速度至步幅转换因子 (cycle_s * stance_frac)
     * @param fixed_step_h 固定摆动抬足高度 (m)
     * @param fixed_stance_z 固定站姿名义高度 (m)
     * @param timeout_s 软件看门狗超时时间阈值 (s)
     */
    ShmCommandSource(double step_per_v, double fixed_step_h, double fixed_stance_z, double timeout_s);

    ~ShmCommandSource();

    ShmCommandSource(const ShmCommandSource&) = delete;
    ShmCommandSource& operator=(const ShmCommandSource&) = delete;

    /**
     * @brief 周期性单调时钟看门狗校验并执行 Seqlock 无锁读取
     * @return 当前周期的底盘运动指令快照
     */
    ChassisCommand read();

    /**
     * @brief 检查共享内存描述符是否正常挂载
     */
    bool attached() const { return shm_ != nullptr; }

private:
    NavCmdShm* shm_ = nullptr;     ///< 只读映射的共享内存结构体指针
    int        fd_  = -1;          ///< 共享内存文件描述符
    double     step_per_v_;        ///< 速度转换系数
    double     fixed_step_h_;      ///< 固定抬腿高度
    double     fixed_stance_z_;    ///< 固定站姿高度
    double     timeout_s_;         ///< 看门狗阈值
};

}  // namespace task
}  // namespace motor_test_cpp
