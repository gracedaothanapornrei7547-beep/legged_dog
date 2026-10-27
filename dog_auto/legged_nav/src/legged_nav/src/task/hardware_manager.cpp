/**
 * @file hardware_manager.cpp
 * @brief 自主导航底层硬件抽象层管理器 (HardwareManager) 实现
 */

#include "motor_test/task/hardware_manager.hpp"

#include <cstdio>
#include <cstring>
#include <map>
#include <string>

#include "motor_test/driver/usb_can.hpp"
#include "motor_test/kinematics/leg_calibration.hpp"

namespace motor_test_cpp {
namespace task {

namespace {

/**
 * @brief 通过 Linux 系统 ip 工具读取指定 CAN 接口底层运行状态
 * @param chan 接口设备名 (如 "can0", "can1")
 * @return 状态字符串 (如 "ERROR-ACTIVE", "BUS-OFF")，若读取失败返回空字符串
 */
std::string read_can_state_via_ip(const std::string& chan) {
    std::string cmd = "ip -d link show " + chan + " 2>/dev/null";
    FILE* p = ::popen(cmd.c_str(), "r");
    if (!p) return "";
    std::string out;
    char buf[512];
    while (std::fgets(buf, sizeof(buf), p)) out += buf;
    ::pclose(p);
    auto pos = out.find("can state ");
    if (pos == std::string::npos) return "";
    pos += std::strlen("can state ");
    auto end = out.find_first_of(" \t\n", pos);
    return out.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
}

}  // namespace

imu::ImuReader& HardwareManager::imu() {
    return imu_;
}

can_bridge::CanBridge* HardwareManager::make_can(int i, const std::string& channel,
                                                 std::uint8_t host_id, bool verbose) {
    can_[i] = std::make_unique<can_bridge::CanBridge>(channel, host_id, verbose);
    return can_[i].get();
}

can_bridge::CanBridge* HardwareManager::make_can(int leg) {
    return make_can(leg, LEG_CAN_CHANNEL[leg]);
}

can_bridge::CanBridge* HardwareManager::can(int i) {
    return can_[i].get();
}

int HardwareManager::preflight(int only_leg) {
    // 1. CAN 总线控制器通信状态校验
    std::printf("[preflight] 校验 %s CAN 通信总线控制器状态...\n",
                only_leg >= 0 ? "单腿测试链路" : "四足全部总线链路");
    bool any_fatal = false;
    for (int i = 0; i < N_LEGS; ++i) {
        if (only_leg >= 0 && i != only_leg) continue;
        const std::string ch = LEG_CAN_CHANNEL[i];
        std::string state = read_can_state_via_ip(ch);
        const char* mark = "✓";
        if (state == "ERROR-ACTIVE") {
            mark = "✓";
        } else if (state == "ERROR-WARNING" || state == "ERROR-PASSIVE") {
            mark = "⚠";
        } else if (state == "BUS-OFF") {
            mark = "❌ BUS-OFF";
            any_fatal = true;
        } else {
            mark = "❌ DOWN/UNAVAILABLE";
            any_fatal = true;
        }
        std::printf("  [%s] %s (%s): %s\n",
                    mark, robot::LEG_NAMES[i].data(), ch.c_str(),
                    state.empty() ? "(无法获取)" : state.c_str());
    }
    if (any_fatal) {
        std::fprintf(stderr,
            "[preflight] 错误: 检测到 CAN 接口处于 BUS-OFF 或未就绪状态，已终止控制器使能。\n"
            "建议执行: sudo systemctl restart can-watchdog.service 或重新连接 USB-CAN 适配器。\n");
        for (int i = 0; i < N_LEGS; ++i) if (can_[i]) can_[i]->disconnect();
        return 3;
    }
    std::printf("[preflight] CAN 总线通信状态正常\n\n");

    // 2. 物理端口拓扑一致性校验 (防止 CAN 设备重命名导致支链映射错乱)
    std::printf("[preflight] 校验四足物理 USB 端口与逻辑支链映射一致性...\n");
    std::map<std::string, std::string> if2usb;
    for (const auto& it : usb_can::scan_usb_can_interfaces())
        if2usb[it.ifname] = it.usb_port;

    bool usb_fatal = false;
    for (int i = 0; i < N_LEGS; ++i) {
        if (only_leg >= 0 && i != only_leg) continue;
        const std::string ch = LEG_CAN_CHANNEL[i];
        const char* want_usb  = LEG_USB_PORT[i];
        auto it = if2usb.find(ch);
        std::string got_usb = (it == if2usb.end()) ? std::string{} : it->second;
        if (got_usb == want_usb) {
            std::printf("  [✓] %s -> %s @ USB %s\n",
                        robot::LEG_NAMES[i].data(), ch.c_str(), got_usb.c_str());
        } else {
            std::printf("  [❌] %s -> %s @ USB %s (预期端口: %s)\n",
                        robot::LEG_NAMES[i].data(), ch.c_str(),
                        got_usb.empty() ? "(未找到设备)" : got_usb.c_str(), want_usb);
            usb_fatal = true;
        }
    }
    if (usb_fatal) {
        std::fprintf(stderr,
            "[preflight] 错误: CAN 设备名称与底层物理 USB 端口拓扑不一致，已终止使能以防止误动。\n"
            "建议执行: sudo scripts/lock-can-by-usb.sh 重新固化端口规则。\n");
        for (int i = 0; i < N_LEGS; ++i) if (can_[i]) can_[i]->disconnect();
        return 4;
    }
    std::printf("[preflight] 硬件拓扑与总线通信自检通过\n\n");
    return 0;
}

std::array<can_bridge::MotorState, 3> HardwareManager::read_joints(int leg) const {
    const auto& cal = *cal_[leg];
    return { can_[leg]->get_motor_state(cal.motor_id_th3),
             can_[leg]->get_motor_state(cal.motor_id_th1),
             can_[leg]->get_motor_state(cal.motor_id_th2) };
}

void HardwareManager::send_slot(int slot,
                                const std::array<Eigen::Vector3d, N_LEGS>& raw,
                                const std::array<double, N_LEGS>& kp,
                                const std::array<double, N_LEGS>& kd,
                                const std::array<Eigen::Vector3d, N_LEGS>& tau) {
    for (int i = 0; i < N_LEGS; ++i) {
        const auto& cal = *cal_[i];
        const int id = (slot == 0) ? cal.motor_id_th3
                     : (slot == 1) ? cal.motor_id_th1
                                   : cal.motor_id_th2;
        can_[i]->private_motion_control(id, raw[i][slot], 0.0, kp[i], kd[i], tau[i][slot]);
    }
}

void HardwareManager::set_leg_current_limit(int leg, float amps) {
    const auto& cal = *cal_[leg];
    can_[leg]->set_current_limit(cal.motor_id_th3, amps);
    can_[leg]->set_current_limit(cal.motor_id_th1, amps);
    can_[leg]->set_current_limit(cal.motor_id_th2, amps);
}

can_bridge::BusState HardwareManager::bus_state(int leg) const {
    return can_[leg]->bus_state();
}

std::uint64_t HardwareManager::err_frame_count(int leg) const {
    return can_[leg]->err_frame_count();
}

}  // namespace task
}  // namespace motor_test_cpp
