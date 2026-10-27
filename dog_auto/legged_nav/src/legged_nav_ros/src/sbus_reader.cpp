// sbus_reader.cpp —— 后台线程 SBUS 接收 + 滑动缓冲解析
//
// 与 Python motor_test/2.7_遥控位控/sbus_receiver.py + sbus_parser.py 合并：
//   - 100000 baud、8E2、可选电平翻转
//   - 后台 reader 线程读串口 → 解析 25 字节 SBUS 帧 → 更新原子帧缓存
//   - 主控环 snapshot() 拿最近一帧（线程安全；fresh(timeout) 判超时）
//
// 串口打开：调外部 extern "C" sbus_open_100k_8e2()（在 sbus_serial_linux.cpp）。
// 这样 <asm/termbits.h> 完全不污染本 TU 的 C++ 标准库。

#include "motor_test/driver/sbus_reader.hpp"
#include "motor_test/planning/subs_gait.hpp"   // parse_command / banner / SbusMap 声明已拆到此

#include <algorithm>  // std::sort (find_sbus_port)
#include <cerrno>
#include <chrono>
#include <cmath>      // std::fabs (parse_command)
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include <dirent.h>   // opendir/readdir (find_sbus_port)
#include <unistd.h>

extern "C" int sbus_open_100k_8e2(const char* port);
extern "C" int sbus_open_100k_8n1(const char* port);

namespace motor_test_cpp {
namespace sbus {

static inline int sbus_open_by_mode(const char* port, SerialMode mode) {
    return (mode == SerialMode::N1) ? sbus_open_100k_8n1(port) : sbus_open_100k_8e2(port);
}

namespace {

inline double monotonic_seconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// 把 22 字节负载按 11 bit/通道分割（与 Python decode_channels 一致）。
// 朴素位流：维护一个 32-bit 寄存器，不够 11 位就吃下一个字节，够了就读出来。
inline std::array<int, SBUS_NUM_CHANNELS> decode_channels_simple(const uint8_t* p) {
    std::array<int, SBUS_NUM_CHANNELS> ch{};
    uint32_t reg = 0;
    int      reg_bits = 0;
    int      byte_idx = 0;
    for (int i = 0; i < SBUS_NUM_CHANNELS; ++i) {
        while (reg_bits < 11) {
            reg |= static_cast<uint32_t>(p[byte_idx++]) << reg_bits;
            reg_bits += 8;
        }
        ch[i] = static_cast<int>(reg & 0x7FF);
        reg >>= 11;
        reg_bits -= 11;
    }
    return ch;
}

inline FailsafeStatus decode_flags(uint8_t flag_byte) {
    if (flag_byte & (1u << 2)) return FailsafeStatus::LOST;
    if (flag_byte & (1u << 3)) return FailsafeStatus::FAILSAFE;
    return FailsafeStatus::OK;
}

// 列 /dev/ttyUSB* (按字典序),用于扫描候选端口。
inline std::vector<std::string> list_ttyusb() {
    std::vector<std::string> out;
    DIR* d = ::opendir("/dev");
    if (!d) return out;
    while (struct dirent* de = ::readdir(d)) {
        const std::string name = de->d_name;
        if (name.rfind("ttyUSB", 0) == 0) out.push_back("/dev/" + name);
    }
    ::closedir(d);
    std::sort(out.begin(), out.end());
    return out;
}

// 在已打开的 fd 上嗅探 timeout_ms,尝试找一帧合法 SBUS(头 0x0F + 尾 0x00,25 字节)。
// invert=true 时按位取反(兼容反相接收机)。找到返回 true。
inline bool probe_sbus_on_fd(int fd, bool invert, int timeout_ms) {
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    std::vector<uint8_t> buf;
    buf.reserve(4 * SBUS_FRAME_LEN);
    while (true) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            clock::now() - t0).count();
        if (elapsed >= timeout_ms) return false;
        uint8_t chunk[64];
        ssize_t n = ::read(fd, chunk, sizeof(chunk));
        if (n > 0) {
            if (invert) for (ssize_t i = 0; i < n; ++i) chunk[i] = ~chunk[i];
            buf.insert(buf.end(), chunk, chunk + n);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        // 滑窗找一帧
        while (static_cast<int>(buf.size()) >= SBUS_FRAME_LEN) {
            size_t idx = 0;
            for (; idx < buf.size(); ++idx) if (buf[idx] == SBUS_HEADER) break;
            if (idx > 0) buf.erase(buf.begin(), buf.begin() + idx);
            if (static_cast<int>(buf.size()) < SBUS_FRAME_LEN) break;
            if (buf[SBUS_FRAME_LEN - 1] == SBUS_FOOTER) return true;
            buf.erase(buf.begin());                       // 伪帧头,跳一字节继续
        }
    }
}

}  // namespace

// ============================================================================
// 自动扫描 (优先 /dev/ttySBUS, 然后 /dev/ttyUSB*), 找 SBUS 端口。
std::string find_sbus_port(bool* out_invert, int per_port_timeout_ms, SerialMode mode) {
    // 优先检查 udev 固定绑定
    {
        const int fd = sbus_open_by_mode("/dev/ttySBUS", mode);
        if (fd >= 0) {
            for (bool inv : {false, true}) {
                if (probe_sbus_on_fd(fd, inv, per_port_timeout_ms / 2)) {
                    ::close(fd);
                    if (out_invert) *out_invert = inv;
                    std::fprintf(stderr, "[sbus-auto] ✓ 命中 /dev/ttySBUS (invert=%d mode=%s)\n",
                                 inv ? 1 : 0, mode == SerialMode::N1 ? "8N1" : "8E2");
                    return "/dev/ttySBUS";
                }
            }
            ::close(fd);
        }
    }
    const auto cands = list_ttyusb();
    if (cands.empty()) {
        std::fprintf(stderr, "[sbus-auto] 没找到任何 /dev/ttyUSB*,检查接收机是否插好\n");
        return "";
    }
    std::fprintf(stderr, "[sbus-auto] 扫描:");
    for (const auto& p : cands) std::fprintf(stderr, " %s", p.c_str());
    std::fprintf(stderr, "\n");
    const int half = per_port_timeout_ms / 2;
    for (const auto& port : cands) {
        const int fd = sbus_open_by_mode(port.c_str(), mode);
        if (fd < 0) {
            std::fprintf(stderr, "[sbus-auto]   %s 打开失败,跳过\n", port.c_str());
            continue;
        }
        for (bool inv : {false, true}) {
            if (probe_sbus_on_fd(fd, inv, half)) {
                ::close(fd);
                if (out_invert) *out_invert = inv;
                std::fprintf(stderr, "[sbus-auto] ✓ 命中 %s (invert=%d)\n",
                             port.c_str(), inv ? 1 : 0);
                return port;
            }
        }
        ::close(fd);
        std::fprintf(stderr, "[sbus-auto]   %s 无 SBUS 帧\n", port.c_str());
    }
    std::fprintf(stderr, "[sbus-auto] ✗ 所有端口都没收到 SBUS 信号("
                          "检查遥控器是否开机/接收机 LED 是否常亮)\n");
    return "";
}

// ============================================================================
SbusReader::SbusReader(const std::string& port, bool invert, double reconnect_interval_s, SerialMode smode)
    : port_(port), invert_(invert), reconnect_interval_s_(reconnect_interval_s),
      smode_(smode), auto_detect_(port == "auto" || port.empty()) {}

SbusReader::~SbusReader() { stop(); }

bool SbusReader::start() {
    if (running_.load()) return true;
    running_.store(true);
    thread_ = std::thread(&SbusReader::read_loop_, this);
    return true;
}

void SbusReader::stop() {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
    close_serial_();
}

FrameSnapshot SbusReader::snapshot() const {
    std::lock_guard<std::mutex> lk(frame_mtx_);
    return latest_;
}

bool SbusReader::open_serial_() {
    // auto 模式:每次(重)连都重新扫一遍,USB 拔插换号也能跟上
    if (auto_detect_) {
        bool inv = invert_;
        const std::string p = find_sbus_port(&inv, 400, smode_);
        if (p.empty()) return false;
        invert_ = inv;
        serial_fd_ = sbus_open_by_mode(p.c_str(), smode_);
        if (serial_fd_ < 0) return false;
        std::fprintf(stderr, "[sbus] 打开串口 %s 成功 (100000 %s, invert=%d)\n",
                     p.c_str(), smode_ == SerialMode::N1 ? "8N1" : "8E2", inv ? 1 : 0);
        return true;
    }
    serial_fd_ = sbus_open_by_mode(port_.c_str(), smode_);
    if (serial_fd_ < 0) return false;
    std::fprintf(stderr, "[sbus] 打开串口 %s 成功 (100000 %s)\n",
                 port_.c_str(), smode_ == SerialMode::N1 ? "8N1" : "8E2");
    return true;
}

void SbusReader::close_serial_() {
    if (serial_fd_ >= 0) { ::close(serial_fd_); serial_fd_ = -1; }
}

bool SbusReader::try_parse_one_frame_(std::vector<uint8_t>& buf, FrameSnapshot& out) {
    while (static_cast<int>(buf.size()) >= SBUS_FRAME_LEN) {
        // 找帧头
        size_t idx = 0;
        for (; idx < buf.size(); ++idx) {
            if (buf[idx] == SBUS_HEADER) break;
        }
        if (idx > 0) {
            // ★诊断: 打印跳过的非帧头字节(仅首次)
            static bool once1 = false;
            if (!once1) {
                std::fprintf(stderr, "[sbus-diag] 跳过 %zu 字节非帧头数据, 前几字节: ", idx);
                for (size_t k = 0; k < (idx < 16 ? idx : 16); ++k)
                    std::fprintf(stderr, "%02x ", buf[k]);
                std::fprintf(stderr, "\n");
                once1 = true;
            }
            buf.erase(buf.begin(), buf.begin() + idx);
        }
        if (static_cast<int>(buf.size()) < SBUS_FRAME_LEN) return false;

        if (buf[SBUS_FRAME_LEN - 1] != SBUS_FOOTER) {
            // 伪帧头，跳过 1 字节
            static bool once2 = false;
            if (!once2) {
                std::fprintf(stderr, "[sbus-diag] 发现疑似帧头 0x0F 但尾部=%02x(期望00), 跳过1字节继续\n",
                             buf[SBUS_FRAME_LEN - 1]);
                once2 = true;
            }
            buf.erase(buf.begin());
            bad_count_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        out.channels = decode_channels_simple(&buf[1]);
        out.status   = decode_flags(buf[SBUS_FRAME_LEN - 2]);
        out.valid    = (out.status == FailsafeStatus::OK);
        out.t_rx_s   = monotonic_seconds();

        static bool once3 = false;
        if (!once3) {
            std::fprintf(stderr, "[sbus-diag] ✓ 解析到第一帧 SBUS! status=%d valid=%d ch0=%d ch1=%d ch2=%d\n",
                         static_cast<int>(out.status), out.valid ? 1 : 0,
                         out.channels[0], out.channels[1], out.channels[2]);
            once3 = true;
        }

        buf.erase(buf.begin(), buf.begin() + SBUS_FRAME_LEN);
        good_count_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }
    return false;
}

void SbusReader::read_loop_() {
    std::vector<uint8_t> buf;
    buf.reserve(4 * SBUS_FRAME_LEN);

    uint64_t read_ok = 0, read_eagain = 0, read_err = 0;
    auto last_report = std::chrono::steady_clock::now();

    while (running_.load()) {
        if (serial_fd_ < 0) {
            if (!open_serial_()) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(
                        static_cast<int>(reconnect_interval_s_ * 1000)));
                continue;
            }
        }

        uint8_t chunk[256];
        ssize_t n = ::read(serial_fd_, chunk, sizeof(chunk));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                read_eagain++;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                // 每 2 秒汇报一次 read 统计
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report).count() >= 2000) {
                    std::fprintf(stderr, "[sbus-diag] read统计: ok=%lu EAGAIN=%lu err=%lu buf=%zu字节\n",
                                 read_ok, read_eagain, read_err, buf.size());
                    last_report = now;
                }
                continue;
            }
            read_err++;
            close_serial_();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    static_cast<int>(reconnect_interval_s_ * 1000)));
            continue;
        }
        if (n == 0) {
            read_eagain++;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        read_ok++;  // n > 0: 读到了数据!

        if (invert_) {
            for (ssize_t i = 0; i < n; ++i) chunk[i] = ~chunk[i];
        }
        buf.insert(buf.end(), chunk, chunk + n);

        FrameSnapshot parsed{};
        bool got_any = false;
        while (try_parse_one_frame_(buf, parsed)) got_any = true;

        if (got_any) {
            std::lock_guard<std::mutex> lk(frame_mtx_);
            latest_ = parsed;
        }

        if (buf.size() > 4 * SBUS_FRAME_LEN) {
            buf.erase(buf.begin(), buf.end() - SBUS_FRAME_LEN);
        }
    }
}

// ============================================================================
// 遥控指令解析 / CLI / banner（封装在 sbus 模块，cli_io 与 trot_quad 只调用）
// ============================================================================

SbusCmd parse_command(const FrameSnapshot& f, double now_s,
                      const SbusMap& m, double default_step_h) {
    SbusCmd c;
    c.step_h = default_step_h;
    c.fresh  = f.fresh(now_s, m.timeout_s);
    if (!c.fresh) { c.step_h = 0.0; return c; }      // 失帧 failsafe → 全 0(站立)

    // 摇杆轴(三态量, 仅 ch0 自转用)：通道值过死区 → 输出 ±sign（方向），否则 0
    auto axis = [&](int ch, double sign) -> double {
        const double v = f.normalized(ch);
        return (v > m.deadzone) ? sign : (v < -m.deadzone ? -sign : 0.0);
    };
    // ★2026-06-20 ch2/ch3 摇杆改【比例】(原三态): raw 过中心 ±stick_deadzone_raw 死区(防误按)后,
    //   按推杆深度线性映射步幅比例 ∈ [stick_scale_min, stick_scale_max] = [0.1, 1.0], 带方向符号。
    //   死区内返回 0(站立)。死区外满量程跨度 = (SBUS_MAX-CENTER) - 死区, 两侧不对称由 clamp 兜住。
    auto stick = [&](int ch, double sign) -> double {
        const int delta  = f.channels[ch] - SBUS_CENTER;        // 带符号偏移(中心=0)
        const int adelta = delta < 0 ? -delta : delta;
        if (adelta <= m.stick_deadzone_raw) return 0.0;         // 死区内不动(防误按)
        const int span = (SBUS_MAX - SBUS_CENTER) - m.stick_deadzone_raw;  // 死区外满跨度
        double frac = static_cast<double>(adelta - m.stick_deadzone_raw)
                    / static_cast<double>(span);
        frac = std::clamp(frac, 0.0, 1.0);                       // 推杆深度 ∈ [0,1]
        const double scale = m.stick_scale_min
                           + (m.stick_scale_max - m.stick_scale_min) * frac;  // 0.1→1.0
        return (delta > 0 ? sign : -sign) * scale;
    };
    // ★2026-06-12 旋钮改连续比例: 归一化 -1(最小)..+1(最大) 线性映射 ×0.3..×1.0, 钳 [0.3,1.0]
    auto knob = [](double v) { return std::clamp(0.65 + 0.35 * v, 0.3, 1.0); };

    const double sx = stick(m.ch_forward, m.sign_forward);   // ch2 比例(带方向, 0 或 ±[0.1,1.0])
    const double sy = stick(m.ch_lateral, m.sign_lateral);   // ch3 比例(同上)
    c.step_x  = sx * knob(f.normalized(m.ch_scale_x)) * m.max_step_x;  // 摇杆比例 ×旋钮档 ×满量程
    c.step_y  = sy * knob(f.normalized(m.ch_scale_y)) * m.max_step_y;
    c.yawrate = axis(m.ch_turn,    m.sign_turn)    * m.max_yawrate;
    c.step_h  = (f.normalized(m.ch_posture) > m.posture_thresh) ? m.step_h_high : m.step_h_low;
    // ★ch7 三段开关 → 身体站高 stance_z(身高)。归一化三分: <-0.5=低/中位=中/>0.5=高。
    //   仅 fresh 帧有效(本函数已过失帧 early-return); 调用方失帧保持上次身高(不归 0)。
    {
        const double hv = f.normalized(m.ch_height);
        c.stance_z = (hv < -0.5) ? m.height_low
                   : (hv >  0.5) ? m.height_high
                   :               m.height_mid;
    }

    // 起步使能 = ch2/ch3 任一过比例死区, 或 ch0 自转过三态死区
    c.gait_enabled = (sx != 0.0 || sy != 0.0 ||
                      std::fabs(f.normalized(m.ch_turn)) > m.deadzone);
    if (!c.gait_enabled) { c.step_x = c.step_y = c.yawrate = 0.0; c.step_h = 0.0; }  // 松手→站立
    return c;
}

std::string banner(const SbusMap& m) {
    if (!m.enabled) return "  SBUS 遥控: off\n";
    char l1[256], l2[320], l3[256];
    std::snprintf(l1, sizeof l1, "    串口=%s invert=%d dz=%.2f timeout=%.2fs\n",
                  m.port.c_str(), m.invert ? 1 : 0, m.deadzone, m.timeout_s);
    std::snprintf(l2, sizeof l2,
                  "    前后=ch%d(×%+g,满%.2f) 侧移=ch%d(×%+g,满%.2f) 自转=ch%d(×%+g,满%.2f) "
                  "姿态=ch%d 比例旋钮=ch%d/ch%d\n",
                  m.ch_forward, m.sign_forward, m.max_step_x,
                  m.ch_lateral, m.sign_lateral, m.max_step_y,
                  m.ch_turn,    m.sign_turn,    m.max_yawrate,
                  m.ch_posture, m.ch_scale_x, m.ch_scale_y);
    std::snprintf(l3, sizeof l3,
                  "    ★ch%d/ch%d 比例推杆: 过中心±%d死区 → 步幅 ×%.1f~%.1f(推杆深度线性)\n",
                  m.ch_forward, m.ch_lateral, m.stick_deadzone_raw,
                  m.stick_scale_min, m.stick_scale_max);
    char l4[200];
    std::snprintf(l4, sizeof l4,
                  "    ★ch%d 三段身高: 低%.0f/中%.0f/高%.0f cm (stance_z, slew %.2fm/s)\n",
                  m.ch_height, -m.height_low * 100, -m.height_mid * 100,
                  -m.height_high * 100, m.height_slew_mps);
    return std::string("  SBUS 遥控: ★ON\n") + l1 + l2 + l3 + l4;
}

}  // namespace sbus
}  // namespace motor_test_cpp
