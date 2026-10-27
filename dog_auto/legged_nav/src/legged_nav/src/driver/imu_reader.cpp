// imu_reader.cpp —— WHEELTEC/Yesense H30 串口驱动实现
//
// 单线程拓扑：rx_thread_ 阻塞读串口 → 拼包 → CRC 校验 → 分发 → 加锁更新 state_
// 主线程通过 get_*() 加锁读 state_

#include "motor_test/driver/imu_reader.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace motor_test_cpp {
namespace imu {

namespace {

// ---- 单位换算系数（与 Python 一致）----
constexpr double TEMP_FACTOR   = 0.01;       // ℃
constexpr double ACC_FACTOR    = 1e-6;       // g
constexpr double GYRO_FACTOR   = 1e-6;       // deg/s（再 deg→rad）
constexpr double EULER_FACTOR  = 1e-6;       // deg
constexpr double QUAT_FACTOR   = 1e-6;       // 归一化系数

constexpr double DEG2RAD = M_PI / 180.0;

inline int32_t rd_i32_le(const uint8_t* p) {
    return static_cast<int32_t>(
        (static_cast<uint32_t>(p[0]))       |
        (static_cast<uint32_t>(p[1]) <<  8) |
        (static_cast<uint32_t>(p[2]) << 16) |
        (static_cast<uint32_t>(p[3]) << 24));
}
inline int16_t rd_i16_le(const uint8_t* p) {
    return static_cast<int16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}
inline uint32_t rd_u32_le(const uint8_t* p) {
    return  static_cast<uint32_t>(p[0])        |
           (static_cast<uint32_t>(p[1]) <<  8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

// 把数字波特率映射到 termios B*** 常量
speed_t baudrate_to_speed_t(int b) {
    switch (b) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 921600:  return B921600;
        default:      return B460800;
    }
}

}  // namespace

// ============================================================================
// 生命周期
// ============================================================================

ImuReader::ImuReader(const std::string& port, int baudrate, bool debug)
    : port_(port), baudrate_(baudrate), debug_(debug) {
    decode_buffer_.reserve(DECODE_BUF_CAP);
}

ImuReader::~ImuReader() { stop(); }

bool ImuReader::start() {
    if (running_.load()) return true;
    if (!open_serial()) return false;
    running_.store(true);
    rx_thread_ = std::thread(&ImuReader::rx_loop, this);
    // 钉 IMU rx 到 CPU1(与 CAN rx 同核, I/O 线程全放狗 RT 隔离区, 不沾 nav 算力)
    cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(1, &cs);
    pthread_setaffinity_np(rx_thread_.native_handle(), sizeof(cs), &cs);
    return true;
}

void ImuReader::stop() {
    if (!running_.exchange(false)) {
        // 不曾启动；如有半开 fd 直接关
        close_serial();
        return;
    }
    if (rx_thread_.joinable()) rx_thread_.join();
    close_serial();
}

// ============================================================================
// 串口
// ============================================================================

bool ImuReader::open_serial() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        if (debug_) std::fprintf(stderr, "[IMU:%s] open() failed: %s\n",
                                  port_.c_str(), std::strerror(errno));
        return false;
    }

    struct termios tio{};
    if (::tcgetattr(fd_, &tio) != 0) {
        if (debug_) std::fprintf(stderr, "[IMU:%s] tcgetattr failed: %s\n",
                                  port_.c_str(), std::strerror(errno));
        close_serial();
        return false;
    }

    cfmakeraw(&tio);                                    // 8N1 raw mode
    const speed_t sp = baudrate_to_speed_t(baudrate_);
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    tio.c_cflag |=  CLOCAL | CREAD;
    tio.c_cflag &= ~CRTSCTS;                            // 无流控
    tio.c_cflag &= ~PARENB;                             // 无校验
    tio.c_cflag &= ~CSTOPB;                             // 1 stop bit
    tio.c_cflag &= ~CSIZE; tio.c_cflag |= CS8;          // 8 data bits

    // 阻塞 read 行为：最少 1 字节，超时 20ms
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 2;                                // 0.2s timeout

    if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
        if (debug_) std::fprintf(stderr, "[IMU:%s] tcsetattr failed: %s\n",
                                  port_.c_str(), std::strerror(errno));
        close_serial();
        return false;
    }

    // 切回阻塞模式（cfmakeraw 不动 O_NONBLOCK，我们手动清掉）
    int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags >= 0) ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    // 清接收缓冲（丢弃可能积压的旧数据）
    ::tcflush(fd_, TCIFLUSH);
    return true;
}

void ImuReader::close_serial() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

// ============================================================================
// 后台线程主循环
// ============================================================================

void ImuReader::rx_loop() {
    std::vector<uint8_t> tmp(UART_READ_CHUNK);

    while (running_.load()) {
        const ssize_t n = ::read(fd_, tmp.data(), tmp.size());
        if (n < 0) {
            if (errno == EINTR) continue;
            // 偶发错误：休 5ms 再来
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        if (n == 0) {
            // 超时无数据
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 缓冲溢出保护
        if (decode_buffer_.size() + static_cast<size_t>(n) > DECODE_BUF_CAP) {
            decode_buffer_.clear();
        }
        decode_buffer_.insert(decode_buffer_.end(), tmp.data(), tmp.data() + n);

        // 拉完所有完整帧
        while (true) {
            int st_idx = 0, end_idx = 0;
            ImuState frame_state{};
            int ret = parse_one_frame(st_idx, end_idx, frame_state);

            if (ret == ANALYSIS_OK) {
                // 写入共享 state_
                frame_state.last_rx_time = monotonic_seconds();
                frame_state.valid = true;
                {
                    std::lock_guard<std::mutex> lk(state_mtx_);
                    // 字段级合并（同一帧只更新本帧带的字段）
                    if (frame_state.euler.roll != 0.0 || frame_state.euler.pitch != 0.0 || frame_state.euler.yaw != 0.0)
                        state_.euler = frame_state.euler;
                    if (frame_state.gyro.x != 0.0 || frame_state.gyro.y != 0.0 || frame_state.gyro.z != 0.0)
                        state_.gyro  = frame_state.gyro;
                    if (frame_state.acc.x  != 0.0 || frame_state.acc.y  != 0.0 || frame_state.acc.z  != 0.0)
                        state_.acc   = frame_state.acc;
                    if (frame_state.quat.q0 != 1.0 || frame_state.quat.q1 != 0.0 ||
                        frame_state.quat.q2 != 0.0 || frame_state.quat.q3 != 0.0)
                        state_.quat = frame_state.quat;
                    if (frame_state.sample_timestamp != 0)
                        state_.sample_timestamp = frame_state.sample_timestamp;
                    if (frame_state.sensor_temp != 0.0)
                        state_.sensor_temp = frame_state.sensor_temp;
                    state_.last_rx_time = frame_state.last_rx_time;
                    state_.valid        = true;
                }
                ++rx_frame_count_;

                // 移除已处理字节
                if (end_idx >= 0 && static_cast<size_t>(end_idx) < decode_buffer_.size()) {
                    decode_buffer_.erase(decode_buffer_.begin(),
                                          decode_buffer_.begin() + end_idx + 1);
                }
                continue;  // 继续拉下一帧
            }

            if (ret == DATA_LEN_ERR) break;  // 数据不够，等下次 read

            if (ret == CRC_ERR) {
                ++crc_err_count_;
                if (end_idx >= 0 && static_cast<size_t>(end_idx) < decode_buffer_.size()) {
                    decode_buffer_.erase(decode_buffer_.begin(),
                                          decode_buffer_.begin() + end_idx + 1);
                }
                continue;
            }

            if (ret == NO_HEADER) {
                ++no_header_count_;
                decode_buffer_.clear();
                break;
            }

            break;
        }
    }
}

// ============================================================================
// 帧解析
// ============================================================================

int ImuReader::parse_one_frame(int& st_idx, int& end_idx, ImuState& out) {
    const auto& buf = decode_buffer_;
    const size_t len = buf.size();

    // 1. 扫描帧头 0x59 0x53
    size_t i = 0;
    while (i + 1 < len) {
        if (buf[i] == HDR1 && buf[i+1] == HDR2) break;
        ++i;
    }
    if (i + 1 >= len) {
        st_idx = 0;
        end_idx = static_cast<int>(len) - 1;
        return NO_HEADER;
    }
    st_idx = static_cast<int>(i);

    // 2. 至少要有 MIN_FRAME_LEN = 7 字节
    if (len - i < MIN_FRAME_LEN) {
        end_idx = static_cast<int>(len) - 1;
        return DATA_LEN_ERR;
    }

    const uint8_t payload_len = buf[i + 4];
    const size_t frame_len = static_cast<size_t>(payload_len) + MIN_FRAME_LEN;
    if (frame_len > len - i) {
        end_idx = static_cast<int>(len) - 1;
        return DATA_LEN_ERR;
    }

    // 3. CRC 校验：TID(2) + Len(1) + Payload(N)
    const uint16_t crc_calc = crc16(&buf[i + 2], static_cast<size_t>(payload_len) + 3);
    const uint16_t crc_recv = static_cast<uint16_t>(buf[i + 5 + payload_len]) |
                              (static_cast<uint16_t>(buf[i + 6 + payload_len]) << 8);
    end_idx = static_cast<int>(i + frame_len - 1);

    if (crc_calc != crc_recv) return CRC_ERR;

    // 4. 分发 payload 数据块
    size_t pos = i + PAYLOAD_POS;
    int remain = payload_len;
    while (remain > 0) {
        if (pos + 2 > len) break;
        const uint8_t data_id  = buf[pos];
        const uint8_t data_len = buf[pos + 1];
        if (pos + 2 + data_len > len) break;

        int ret = parse_payload_block(data_id, data_len, &buf[pos + 2], out);
        if (ret != ANALYSIS_OK) {
            // 未知/长度错：跳 1 字节继续找
            pos    += 1;
            remain -= 1;
        } else {
            pos    += 2 + data_len;
            remain -= 2 + data_len;
        }
    }
    return ANALYSIS_OK;
}

int ImuReader::parse_payload_block(uint8_t id, uint8_t len,
                                    const uint8_t* d, ImuState& out) {
    switch (id) {
        case ID_TEMP: {
            if (len != 2) return DATA_LEN_ERR;
            out.sensor_temp = rd_i16_le(d) * TEMP_FACTOR;
            return ANALYSIS_OK;
        }
        case ID_ACCEL: {
            if (len != 12) return DATA_LEN_ERR;
            out.acc.x = rd_i32_le(d)     * ACC_FACTOR;
            out.acc.y = rd_i32_le(d + 4) * ACC_FACTOR;
            out.acc.z = rd_i32_le(d + 8) * ACC_FACTOR;
            return ANALYSIS_OK;
        }
        case ID_GYRO: {
            if (len != 12) return DATA_LEN_ERR;
            // 协议 deg/s → 转 rad/s
            out.gyro.x = rd_i32_le(d)     * GYRO_FACTOR * DEG2RAD;
            out.gyro.y = rd_i32_le(d + 4) * GYRO_FACTOR * DEG2RAD;
            out.gyro.z = rd_i32_le(d + 8) * GYRO_FACTOR * DEG2RAD;
            return ANALYSIS_OK;
        }
        case ID_EULER: {
            if (len != 12) return DATA_LEN_ERR;
            // 协议顺序 pitch, roll, yaw（与 Python 一致）→ 转 rad
            const double pitch = rd_i32_le(d)     * EULER_FACTOR * DEG2RAD;
            const double roll  = rd_i32_le(d + 4) * EULER_FACTOR * DEG2RAD;
            const double yaw   = rd_i32_le(d + 8) * EULER_FACTOR * DEG2RAD;
            out.euler.pitch = pitch;
            out.euler.roll  = roll;
            out.euler.yaw   = yaw;
            return ANALYSIS_OK;
        }
        case ID_QUAT: {
            if (len != 16) return DATA_LEN_ERR;
            out.quat.q0 = rd_i32_le(d)      * QUAT_FACTOR;
            out.quat.q1 = rd_i32_le(d + 4)  * QUAT_FACTOR;
            out.quat.q2 = rd_i32_le(d + 8)  * QUAT_FACTOR;
            out.quat.q3 = rd_i32_le(d + 12) * QUAT_FACTOR;
            return ANALYSIS_OK;
        }
        case ID_SAMPLE_TIMESTAMP: {
            if (len != 4) return DATA_LEN_ERR;
            out.sample_timestamp = rd_u32_le(d);
            return ANALYSIS_OK;
        }
        // 暂不处理：mag, utc, location, speed, status
        default:
            return DATA_LEN_ERR;
    }
}

// 双累加和校验（Fletcher-like 8-bit cascade）
uint16_t ImuReader::crc16(const uint8_t* data, size_t len) {
    uint8_t ck1 = 0, ck2 = 0;
    for (size_t i = 0; i < len; ++i) {
        ck1 = static_cast<uint8_t>(ck1 + data[i]);
        ck2 = static_cast<uint8_t>(ck2 + ck1);
    }
    return static_cast<uint16_t>((static_cast<uint16_t>(ck2) << 8) | ck1);
}

// ============================================================================
// 工具
// ============================================================================

double ImuReader::monotonic_seconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ============================================================================
// Thread-safe getter
// ============================================================================

EulerData ImuReader::get_euler() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return state_.euler;
}
Vec3 ImuReader::get_gyro() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return state_.gyro;
}
Vec3 ImuReader::get_acc() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return state_.acc;
}
QuatData ImuReader::get_quat() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return state_.quat;
}
ImuState ImuReader::get_state() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return state_;
}

}  // namespace imu
}  // namespace motor_test_cpp
