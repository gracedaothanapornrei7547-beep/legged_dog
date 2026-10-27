// ============================================================================
// 文件: usb_can.hpp
// 模块: parallel::driver (CAN 驱动层)
// 职责: 4 腿 USB-CAN 复合驱动 (candleLight x2, gs_usb) —— 运行时扫描
//       /sys/class/net/can* 获取稳定 USB 物理口标识，按 LEG_BINDINGS 映射表
//       将 USB 口绑定到腿，避免内核 canN 命名不稳定导致腿间串控。
//       空口不构造 CanBridge；start() 检测到任何一条腿缺失则全部释放、返回 false。
//       SocketCAN 接口由外部脚本拉起，本类不管理 link up / bitrate / restart-ms。
// ============================================================================

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "parallel/driver/can_bridge.hpp"

namespace parallel {
namespace usb_can {

// ----------------------------------------------------------------------------
// Leg 枚举: 下标 0..3, 与 LegCalibration::leg_idx (LEG_FL/FR/BL/BR) 完全一致,
// 方便直接当 std::array 下标用。改这个枚举值必须同步改 LEG_BINDINGS 顺序。
// ----------------------------------------------------------------------------
enum class Leg : int {
    FL = 0,
    FR = 1,
    BL = 2,
    BR = 3,
};

inline constexpr int N_MOTOR_LEGS = 4;

// 名字 (打 log 用)。
inline const char* leg_name(Leg l) {
    switch (l) {
        case Leg::FL: return "FL";
        case Leg::FR: return "FR";
        case Leg::BL: return "BL";
        case Leg::BR: return "BR";
    }
    return "??";
}

// 每条腿对应的 USB 物理口 + 电机 ID 三元组。硬件层常量, 编译期就锁死。
// 与上面注释中的物理布局一一对应。改这张表 == 改硬件接线。
struct LegBinding {
    Leg         leg;
    const char* usb_port;       // 如 "3-1.1", 对应 /sys 路径里的 KERNELS=
    const char* label;          // 打 log 友好名
    int         motor_id_th3;   // hip-z   (绕 z 的髋转)
    int         motor_id_th1;   // hip-y   (大腿摆)
    int         motor_id_th2;   // knee
};

// 4 条带电机的腿绑定表。空口 (3-1.3/4, 3-2.1/2) 不在表里 — 永远不碰。
// 顺序必须与 Leg 枚举的整数值一一对应 (FL=0 在第 0 行, FR=1 在第 1 行, ...);
// 见 usb_can.cpp 顶部的 static_assert 强制检查。
inline constexpr std::array<LegBinding, N_MOTOR_LEGS> LEG_BINDINGS = {{
    // leg,        usb,       label, th3, th1, th2
    { Leg::FL,    "3-1.4",    "FL",   4,   5,   6 },
    { Leg::FR,    "3-1.3",    "FR",   1,   2,   3 },
    { Leg::BL,    "3-2.3",    "BL",   7,   8,   9 },
    { Leg::BR,    "3-2.4",    "BR",  11,  10,  12 },
}};

// ----------------------------------------------------------------------------
// 工具: 扫描 /sys/class/net/can*, 返回 USB 口 → 接口名 的映射。
//
// 拆出来当独立函数有 2 个用处:
//   (1) UsbCanDriver::start 内部用。
//   (2) 单元测试 / 诊断 CLI 可以直接调, 不强依赖 UsbCanDriver。
//
// 返回 vector<UsbCanIf>。usb 口形如 "3-1.1", can 名形如 "can0"。
// 如果 /sys 读不动 (极少见, 权限问题) 返回空 vector。
// 注: 不打 stderr (库函数保持纯净); 调用方根据返回结果决定是否打 log。
// ----------------------------------------------------------------------------
struct UsbCanIf {
    std::string usb_port;       // "3-1.1"
    std::string ifname;         // "can0"
};
std::vector<UsbCanIf> scan_usb_can_interfaces();

}  // namespace usb_can
}  // namespace parallel
