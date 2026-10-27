// usb_can.cpp —— UsbCanDriver 实现: 扫 /sys/class/net 把 USB 口绑到 CanBridge
//
// ============================================================================
// USB 物理口扫描原理 (与 scripts/lock-can-by-usb.sh 同款套路)
// ============================================================================
//
//   /sys/class/net/can0/device
//      → 软链接, readlink 解全后形如:
//        /sys/devices/pci0000:00/0000:00:14.0/usb3/3-1/3-1.1/3-1.1:1.0
//        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  ^^^^^^^
//                          USB hub + port 路径                 interface 段
//
//   末尾的 "3-1.1:1.0" 是 USB *interface* 描述符 (":bConfig.bInterface"),
//   每次重新枚举可能变; 前面的 "3-1.1" 才是 USB *port* — 只要主板上的 USB-A
//   口不动, rmmod / modprobe / reboot 之后这值都不变。
//
//   做法: 取 dirname (去掉最后那个 ":N.M" 段), 再取 basename。
//
// ============================================================================
// 容错策略
// ============================================================================
//
//   * /sys/class/net 不存在 / 读不动 → 返回空 vector, start() 自然失败。
//   * 某个 can* 没有 device 符号链接 (虚拟接口, vcan 之类) → 跳过, 不报错。
//   * 扫描结果里出现重复 USB 口 → start() 拒绝启动 (硬件理论上不可能,
//     真出现说明 /sys 状态损坏或多绑同口, 不应继续)。
//   * 4 腿映射表里某个 USB 口在扫描结果里查不到 → start() 报错, 列出
//     缺谁; 已 connect 的 bridge 一并回收。
//   * CanBridge::connect() 失败 (接口 down / 不存在) → 同上。
//
// 这套设计的目标是 "要么 4 条腿全好, 要么直接拒跑"。控制循环不接受
// 部分初始化。

#include "motor_test/driver/usb_can.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>      // ::realpath
#include <cstring>
#include <set>
#include <stdexcept>

#include <dirent.h>
#include <limits.h>      // PATH_MAX
#include <unistd.h>

namespace motor_test_cpp {
namespace usb_can {

// ----------------------------------------------------------------------------
// 编译期强约束: LEG_BINDINGS 的下标 i 必须对应 Leg::FL/FR/BL/BR (整数值 0..3)。
// 如果未来有人把表项调换顺序但忘了同步 Leg 枚举值, 这里直接编译失败。
// ----------------------------------------------------------------------------
static_assert(LEG_BINDINGS[0].leg == Leg::FL, "LEG_BINDINGS[0] 必须是 FL");
static_assert(LEG_BINDINGS[1].leg == Leg::FR, "LEG_BINDINGS[1] 必须是 FR");
static_assert(LEG_BINDINGS[2].leg == Leg::BL, "LEG_BINDINGS[2] 必须是 BL");
static_assert(LEG_BINDINGS[3].leg == Leg::BR, "LEG_BINDINGS[3] 必须是 BR");
static_assert(static_cast<int>(Leg::FL) == 0, "Leg::FL 必须 == 0");
static_assert(static_cast<int>(Leg::FR) == 1, "Leg::FR 必须 == 1");
static_assert(static_cast<int>(Leg::BL) == 2, "Leg::BL 必须 == 2");
static_assert(static_cast<int>(Leg::BR) == 3, "Leg::BR 必须 == 3");

namespace {

// ----------------------------------------------------------------------------
// 小 RAII: 保证 opendir 的 DIR* 一定被 closedir, 不管中途 throw 还是 return。
// 不用 unique_ptr<DIR, ...> 的写法是为了避免依赖 deleter 模板, 读着更直白。
// ----------------------------------------------------------------------------
struct DirGuard {
    DIR* d = nullptr;
    explicit DirGuard(DIR* p) : d(p) {}
    ~DirGuard() { if (d) ::closedir(d); }
    DirGuard(const DirGuard&) = delete;
    DirGuard& operator=(const DirGuard&) = delete;
};

// ----------------------------------------------------------------------------
// 从形如 ".../usb3/3-1/3-1.1/3-1.1:1.0" 的全路径里抽出 "3-1.1"。
//
// 规则: 取末尾路径段; 如果该段含 ':' → 它是 USB interface 段 (如 "3-1.1:1.0"),
// 把 ":..." 砍掉。否则原样返回 (一般不该走到, 防御性兜底)。
//
// 例:
//   "/sys/devices/.../3-1/3-1.1/3-1.1:1.0"  → "3-1.1"   (典型 USB interface)
//   "/sys/devices/.../3-1/3-1.1"            → "3-1.1"   (USB device 自身, 已无 ":")
//   "/sys/devices/.../usbN"                 → "usbN"    (USB root, 返回但合法性检查会过滤)
// ----------------------------------------------------------------------------
std::string extract_usb_port(const std::string& full_path) {
    auto slash = full_path.find_last_of('/');
    std::string last = (slash == std::string::npos) ? full_path
                                                     : full_path.substr(slash + 1);
    auto colon = last.find(':');
    if (colon != std::string::npos) {
        // "3-1.1:1.0" → "3-1.1"
        last.erase(colon);
    }
    return last;
}

// ----------------------------------------------------------------------------
// readlink 全解 (递归 resolve symlink)。失败返回空串。
//
// 用 realpath(3) 是最稳的: 内核给出来的 /sys/class/net/canN/device 是
// 相对链接 (一般指向 "../../../<usb-path>"), realpath 会一次性解到根。
// ----------------------------------------------------------------------------
std::string resolve_realpath(const std::string& path) {
    char buf[PATH_MAX];
    char* r = ::realpath(path.c_str(), buf);
    if (!r) return {};
    return std::string(r);
}

// ----------------------------------------------------------------------------
// 给定 ifname (如 "can0"), 返回它对应的 USB 物理口 (如 "3-1.1")。
// 不是 USB 设备 (vcan, tunnel) → 返回空串。
// ----------------------------------------------------------------------------
std::string ifname_to_usb_port(const std::string& ifname) {
    std::string device_link = "/sys/class/net/" + ifname + "/device";
    std::string real = resolve_realpath(device_link);
    if (real.empty()) return {};

    // 此时 real 形如 ".../usb3/3-1/3-1.1/3-1.1:1.0"。
    // 先剥掉末尾 USB interface 段:
    std::string port = extract_usb_port(real);

    // 简单合法性: 必须形如 "<bus>-<port>[.<port>]*", 至少含一个 '-'。
    // 这一步把 vcan / loopback 等明显不像 USB 的过滤掉, 也把 root hub
    // 的 "usbN" 之类的过滤掉。
    if (port.find('-') == std::string::npos) return {};
    return port;
}

// ----------------------------------------------------------------------------
// 接口名是否形如 "canN" (N 是十进制数字, 至少 1 位)。
// 把 "cantmp_xxx" (lock-can-by-usb.sh 临时名), "canu0" (历史 fallback 名)
// 等都过滤掉, 避免误抓。
// ----------------------------------------------------------------------------
bool looks_like_can_name(const std::string& name) {
    if (name.rfind("can", 0) != 0) return false;     // 不以 "can" 开头
    if (name.size() <= 3)         return false;      // 单纯 "can" 没意义
    return std::all_of(name.begin() + 3, name.end(),
                       [](unsigned char c) { return std::isdigit(c) != 0; });
}

}  // namespace

// ============================================================================
// scan_usb_can_interfaces —— 扫 /sys/class/net 列所有 can* 接口
// ============================================================================
std::vector<UsbCanIf> scan_usb_can_interfaces() {
    std::vector<UsbCanIf> out;

    DIR* raw = ::opendir("/sys/class/net");
    if (!raw) {
        // /sys 读不动 — 系统级问题; 库函数保持纯净, 不打 stderr。
        // 上层调用者 (UsbCanDriver::start) 看到空结果会打日志。
        return out;
    }
    DirGuard guard(raw);

    for (struct dirent* e; (e = ::readdir(guard.d)) != nullptr; ) {
        std::string name = e->d_name;
        if (!looks_like_can_name(name)) continue;

        std::string usb_port = ifname_to_usb_port(name);
        if (usb_port.empty()) {
            // 该接口不是 USB (或 device 链接残废), 不放进结果。
            continue;
        }
        out.push_back({std::move(usb_port), std::move(name)});
    }
    return out;
}

}  // namespace usb_can
}  // namespace motor_test_cpp
