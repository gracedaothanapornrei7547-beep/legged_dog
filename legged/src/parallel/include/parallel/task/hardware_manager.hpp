/**
 * @file hardware_manager.hpp
 * @brief 机器人底层硬件抽象层管理器 (HardwareManager)
 * 
 * 模块定位: Layer 4 硬件抽象与编排 (parallel::task)
 * 职责描述:
 *   集中管理与调度四足机器人的全部外设硬件资源 (九轴 IMU、SBUS 遥控通信接收器、4 路独立 SocketCAN 总线)，
 *   封装关节电机 ID 寻址与 3-Slot 空间插补并发通信，并提供开机通信自检与拓扑一致性校验。
 */

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Core>

#include "parallel/common/pod_types.hpp"
#include "parallel/driver/can_bridge.hpp"
#include "parallel/driver/imu_reader.hpp"
#include "parallel/driver/sbus_reader.hpp"
#include "parallel/kinematics/leg_calibration.hpp"

namespace parallel {
namespace task {

/**
 * @brief 四足各支链 CAN 网络设备名与物理 USB 端口固定拓扑绑定关系
 * 支链顺序: FL(0), FR(1), BL(2), BR(3)
 */
inline constexpr std::array<const char*, N_LEGS> LEG_CAN_CHANNEL =
    { "can5", "can3", "can7", "can6" };
inline constexpr std::array<const char*, N_LEGS> LEG_USB_PORT =
    { "3-1.4", "3-1.3", "3-2.4", "3-2.3" };

/**
 * @brief 机器人底层外设硬件抽象管理器
 */
class HardwareManager {
public:
    /**
     * @brief 获取九轴高频 IMU 惯导读取器句柄
     */
    imu::ImuReader& imu();

    /**
     * @brief 获取 SBUS 遥控接收驱动句柄
     */
    std::unique_ptr<sbus::SbusReader>& sbus();

    /**
     * @brief 构建并初始化指定腿的 SocketCAN 通信桥接器
     * @param leg 支链索引 [0..3] (使用拓扑表 LEG_CAN_CHANNEL 自动寻址)
     */
    can_bridge::CanBridge* make_can(int leg);

    /**
     * @brief 显式参数创建 SocketCAN 桥接器
     */
    can_bridge::CanBridge* make_can(int i, const std::string& channel,
                                    std::uint8_t host_id = 0xFD, bool verbose = false);

    /**
     * @brief 获取指定腿的 SocketCAN 驱动句柄
     */
    can_bridge::CanBridge* can(int i);

    /**
     * @brief 上电使能前硬件通信环境预检
     * 检验项:
     * 1. 4 条 CAN 总线底层控制器状态 (确保处于 ERROR-ACTIVE / WARNING / PASSIVE，阻断 BUS-OFF / DOWN)
     * 2. USB 物理总线拓扑一致性 (核验各腿 CAN 通道与物理 USB 端点映射，防止指令错发)
     * @param only_leg 若 >= 0 则仅测试单腿，跳过其余支链
     * @return 0: 检查通过; 3: CAN 总线故障; 4: 物理拓扑端口错位
     */
    int preflight(int only_leg = -1);

    /**
     * @brief 注入单腿标定参数 (用于 motor_id 到逻辑关节的自动寻址)
     */
    void set_calibration(int leg, const robot::LegCalibration* cal) { cal_[leg] = cal; }

    /**
     * @brief 读取指定支链 3 关节最新反馈状态 {θ3, θ1, θ2}
     */
    std::array<can_bridge::MotorState, 3> read_joints(int leg) const;

    /**
     * @brief 按照切片 Slot 并发下发四腿对应电机的运动控制指令
     * @param slot 切片序号: 0 = θ3 侧摆关节, 1 = θ1 前连杆, 2 = θ2 后连杆
     * @param raw  物理电机期望位置数组 (rad)
     * @param kp   位置比例增益数组
     * @param kd   速度微分增益数组
     * @param tau  前馈附加力矩数组 (Nm)
     */
    void send_slot(int slot,
                   const std::array<Eigen::Vector3d, N_LEGS>& raw,
                   const std::array<double, N_LEGS>& kp,
                   const std::array<double, N_LEGS>& kd,
                   const std::array<Eigen::Vector3d, N_LEGS>& tau);

    /**
     * @brief 设置指定支链 3 关节电机的相电流限制阈值
     */
    void set_leg_current_limit(int leg, float amps);

    /**
     * @brief 获取指定支链 CAN 控制器当前总线状态
     */
    can_bridge::BusState bus_state(int leg) const;

    /**
     * @brief 获取指定支链累计通信错误帧数
     */
    std::uint64_t err_frame_count(int leg) const;

private:
    imu::ImuReader imu_{"/dev/wheeltec_IMU", 460800, false};
    std::unique_ptr<sbus::SbusReader> sbus_;
    std::array<std::unique_ptr<can_bridge::CanBridge>, N_LEGS> can_;
    std::array<const robot::LegCalibration*, N_LEGS> cal_{};
};

}  // namespace task
}  // namespace parallel
