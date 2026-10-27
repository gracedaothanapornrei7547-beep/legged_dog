// sbus_reader.hpp —— 后台线程 SBUS 接收器（无 ROS 依赖、纯 termios2）
//
// 与 Python motor_test/2.7_遥控位控/sbus_receiver.py + sbus_parser.py 的合并版本：
//   - 100000 波特、8E2、可选电平翻转
//   - 后台 reader 线程读串口 → 解析 25 字节 SBUS 帧 → 更新原子帧缓存
//   - 主控环 get_latest() 拿最近一帧（线程安全，按帧粒度 swap）
//
// 用法：
//   SbusReader r{"/dev/ttyUSB0", /*invert=*/true};
//   if (!r.start()) { ... }
//   per tick：
//     auto f = r.snapshot();
//     if (!f.fresh(now_s, 0.3)) ... // failsafe
//
// 设计：
//   - 单分配缓冲，frame_ 受 mutex 保护
//   - 通道值原始 [172..1811] / 中心 992；snapshot.normalized[i] ∈ [-1,1]
//   - 容错：串口打开失败 → 后台循环每 reconnect_interval_s 重试，外部 snapshot()
//     无信号时 fresh() 自动判 false

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace motor_test_cpp {
namespace sbus {

constexpr int  SBUS_FRAME_LEN    = 25;
constexpr uint8_t SBUS_HEADER    = 0x0F;
constexpr uint8_t SBUS_FOOTER    = 0x00;
constexpr int  SBUS_NUM_CHANNELS = 16;
constexpr int  SBUS_MIN          = 172;
constexpr int  SBUS_MAX          = 1811;
constexpr int  SBUS_CENTER       = 992;

enum class FailsafeStatus : uint8_t {
    OK        = 0,
    LOST      = 1,   // bit2 of flag byte
    FAILSAFE  = 3,   // bit3 of flag byte
};

struct FrameSnapshot {
    std::array<int, SBUS_NUM_CHANNELS> channels{};
    FailsafeStatus status = FailsafeStatus::FAILSAFE;
    bool   valid = false;
    double t_rx_s = 0.0;          // monotonic seconds of last rx

    bool fresh(double now_s, double timeout_s) const {
        return valid && (now_s - t_rx_s) <= timeout_s;
    }

    // 归一化通道 ∈ [-1, 1]（CENTER → 0）
    double normalized(int ch) const {
        const double v = static_cast<double>(channels[ch] - SBUS_CENTER)
                       / static_cast<double>(SBUS_MAX - SBUS_CENTER);
        if (v >  1.0) return  1.0;
        if (v < -1.0) return -1.0;
        return v;
    }
};

// 串口模式: E2=8E2(SBUS标准) / N1=8N1(CP2102兼容, 默认)
enum class SerialMode : uint8_t { E2 = 0, N1 = 1 };

// 自动扫描 /dev/ttyUSB* + /dev/ttySBUS,找出第一个能输出合法 SBUS 帧的端口。
// 同时尝试 invert=false 和 invert=true 两种极性,命中后写入 *out_invert。
// mode 控制串口配置(N1 对 CP2102 更兼容)。
std::string find_sbus_port(bool* out_invert = nullptr,
                           int per_port_timeout_ms = 400,
                           SerialMode mode = SerialMode::N1);

class SbusReader {
public:
    // port = "auto" 或 "" → 启动时调 find_sbus_port() 自动扫描;每次断开重连也会重扫,
    //                       USB 拔插换 ttyUSB 号也能自适应。
    explicit SbusReader(const std::string& port = "auto",
                        bool invert = true,
                        double reconnect_interval_s = 3.0,
                        SerialMode smode = SerialMode::N1);
    ~SbusReader();

    SbusReader(const SbusReader&) = delete;
    SbusReader& operator=(const SbusReader&) = delete;

    // 启动后台线程；返回 true 表示线程已启动（串口打开失败也算"已启动"，
    // 后台会循环重试）
    bool start();
    void stop();

    // 主控环每拍调；返回当前帧快照（如果从来没收到过，返回 valid=false）
    FrameSnapshot snapshot() const;

    uint64_t good_count() const { return good_count_.load(); }
    uint64_t bad_count()  const { return bad_count_.load(); }

private:
    // 后台主循环：打开串口 + 批量 read + 滑动缓冲解析
    void read_loop_();
    bool open_serial_();
    void close_serial_();

    // 缓冲区中尝试找一帧（消费首帧后返回 true）
    bool try_parse_one_frame_(std::vector<uint8_t>& buf, FrameSnapshot& out);

    std::string port_;
    bool        invert_;
    double      reconnect_interval_s_;
    SerialMode  smode_ = SerialMode::N1;
    bool        auto_detect_ = false;   // port_ == "auto"/"" 时为 true,open_serial_ 走扫描分支

    int                serial_fd_ = -1;
    std::atomic<bool>  running_{false};
    std::thread        thread_;

    mutable std::mutex frame_mtx_;
    FrameSnapshot      latest_;   // valid 仅在第一帧成功后置 true

    std::atomic<uint64_t> good_count_{0};
    std::atomic<uint64_t> bad_count_{0};
};

// ============================================================================
// ★ 高层「遥控通道 ↔ 步态指令」映射 (SbusMap / SbusCmd / parse_command / banner)
//   已拆到 → motor_test/planning/subs_gait.hpp。本文件只留低层 SBUS 接收/解帧。
// ============================================================================

}  // namespace sbus
}  // namespace motor_test_cpp
