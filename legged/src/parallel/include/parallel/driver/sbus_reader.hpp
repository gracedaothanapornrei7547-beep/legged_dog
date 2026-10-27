// ============================================================================
// 文件: sbus_reader.hpp
// 模块: parallel::driver (遥控驱动层)
// 职责: 后台线程 SBUS 接收器 —— 100000/8E2 串口读取、25 字节帧解析、原子帧缓存，
//       支持自动扫描端口 (find_sbus_port) 与信号丢失检测 (fresh / FailsafeStatus)。
//       高层遥控通道至步态指令映射已拆分至 planning/subs_gait.hpp。
// ============================================================================

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace parallel {
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

// 自动扫描 /dev/ttyUSB*,找出第一个能输出合法 SBUS 帧的端口。
// 同时尝试 invert=false 和 invert=true 两种极性,命中后写入 *out_invert。
// 找到返回端口路径(如 "/dev/ttyUSB1");全部失败返回空字符串。
// per_port_timeout_ms 是每个候选端口的最长嗅探时间(双极性平分)。
std::string find_sbus_port(bool* out_invert = nullptr,
                           int per_port_timeout_ms = 400);

class SbusReader {
public:
    // port = "auto" 或 "" → 启动时调 find_sbus_port() 自动扫描;每次断开重连也会重扫,
    //                       USB 拔插换 ttyUSB 号也能自适应。
    explicit SbusReader(const std::string& port = "auto",
                        bool invert = true,
                        double reconnect_interval_s = 3.0);
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
    bool        auto_detect_ = false;   // port_ == "auto"/"" 时为 true,open_serial_ 走扫描分支

    int                serial_fd_ = -1;
    std::atomic<bool>  running_{false};
    std::thread        thread_;

    mutable std::mutex frame_mtx_;
    FrameSnapshot      latest_;   // valid 仅在第一帧成功后置 true

    std::atomic<uint64_t> good_count_{0};
    std::atomic<uint64_t> bad_count_{0};
};

}  // namespace sbus
}  // namespace parallel
