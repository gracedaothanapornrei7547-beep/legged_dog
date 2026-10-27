// usb_can.hpp —— 4 腿 USB-CAN 复合驱动 (candleLight × 2 板,按 USB 物理口绑腿)
//
// ============================================================================
// 这个文件存在的理由
// ============================================================================
//
// 项目里有 2 块 candleLight (gs_usb 1d50:606f) USB-CAN 板, 每板 4 路,
// 全机一共 8 个 SocketCAN 接口 (can0..can7)。其中只有 4 路接了电机:
//
//   板 B (USB hub @ 3-1)
//     3-1.1  →  FR  电机 ID 1,2,3
//     3-1.2  →  FL  电机 ID 4,5,6
//     3-1.3  →  (空)
//     3-1.4  →  (空)
//   板 A (USB hub @ 3-2)
//     3-2.1  →  (空)
//     3-2.2  →  (空)
//     3-2.3  →  BL  电机 ID 7,8,9
//     3-2.4  →  BR  电机 ID 11,10,12
//
// 旧实现: 在 LegCalibration 里硬编码 "can0/can1/can6/can7", 依赖外部
// udev 规则 (70-candlelight-can.rules) 或 lock-can-by-usb.sh 把内核动态
// 分配的 canN 名重命名成约定名。问题: rmmod gs_usb / modprobe 循环之后,
// 内核分配顺序不稳定; 如果 udev 没跑通或 lock 脚本失败, 硬编码的 "can0"
// 就指错了人 — 会把 FR 的命令发到 BL 电机上, 瞬间翻车。
//
// 新方案: 不再相信任何 canN 名字, 运行时主动扫描 /sys/class/net/can*,
// 通过 readlink ../device 拿到 USB 物理路径 (形如 .../usb3/3-1/3-1.1/3-1.1:1.0),
// 截掉末尾的 ":N.M" interface 段, 得到稳定 USB 口标识 "3-1.1"。然后按上面
// 的固定映射表把 USB 口 → 腿。只要主板上接两块板的 USB-A 口不动, rmmod /
// reboot / udev 失败都不会让我们把腿打串。
//
// ============================================================================
// 4-vs-8 拆分
// ============================================================================
//
// 8 个 USB 口里只有 4 个有电机。空口 (3-1.3 / 3-1.4 / 3-2.1 / 3-2.2) 即使
// 内核给它分配了 canN 也完全不碰: 不构造 CanBridge, 不 connect, 不查状态。
// 用 std::array<unique_ptr<CanBridge>, 4> (编译期长度 4), 下标直接是 Leg 枚举。
//
// ============================================================================
// 缺口或错位怎么办
// ============================================================================
//
// start() 扫描时, 如果 4 个有电机的 USB 口里**任何一个**找不到对应的 can
// 接口 (USB 没插 / 板子枚举失败 / lsusb 都看不见), start() 返回 false,
// stderr 打印缺失的 USB 口和腿名。已成功打开的 CanBridge 会被一并释放,
// main 直接 return (不进入控制循环), 不让一只"瘸腿狗"跑控制。
//
// ============================================================================
// 假设
// ============================================================================
//
//   * SocketCAN 接口已经被外部脚本拉起来 (setup-can.sh / can-up.service)。
//     本类不做 `ip link set canN up`, 不设 bitrate, 不设 restart-ms。
//   * 用户进程能读 /sys/class/net (普通用户即可, 无需 root)。
//   * gs_usb 驱动模块已加载 (modprobe gs_usb), 否则 /sys 里根本没有 can*。
//
// ============================================================================
// 线程模型
// ============================================================================
//
// 与 CanBridge 一致: 管理线程 (构造 / start / stop) 单线程;
// bridge(Leg) 返回的引用上发送函数可在 RT 线程调用,
// CanBridge 内部 rx_thread 独立跑、状态读取加锁。
// 注意: stop() 期间不要从 RT 线程持有 bridge() 引用 (会被析构掉)。
//
// ============================================================================
// 如何扩展
// ============================================================================
//
//   * 加一条腿 / 换板子接线 → 改 LEG_BINDINGS 表。Leg 枚举的整数值必须
//     和数组下标一一对应 (cpp 里有 static_assert 强制), 顺序不能乱。
//   * 换 master_id / 开 debug → 调用 start(master_id, debug)。
//   * 不依赖本类做诊断 → 直接调 scan_usb_can_interfaces()。

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "motor_test/driver/can_bridge.hpp"

namespace motor_test_cpp {
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
    { Leg::FL,    "3-1.2",    "FL",   4,   5,   6 },
    { Leg::FR,    "3-1.1",    "FR",   1,   2,   3 },
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
}  // namespace motor_test_cpp
