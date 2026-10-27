// imu_reader.hpp —— WHEELTEC / Yesense H30 惯导 IMU 串口驱动
//
// 与 motor_test/motor_test/driver/imu.py 1:1 对齐：
//   - 协议：`59 53 + TID(2B) + Payload_Len(1B) + Payload(NB) + CRC(2B)`
//   - CRC：双累加和 (ck1 = Σbyte, ck2 = Σck1)，16-bit (ck2<<8 | ck1)
//   - 后台线程独立读串口、拼包、校验、分发；主线程加锁读最新一帧
//   - 默认 /dev/wheeltec_IMU @ 460800
//
// 用法（main 里）：
//   ImuReader imu;
//   if (!imu.start()) { ... }
//   ...
//   auto e = imu.get_euler();   // rad
//   auto g = imu.get_gyro();    // rad/s
//   auto a = imu.get_acc();     // g
//   imu.stop();

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace motor_test_cpp {
namespace imu {

// ============================================================================
// 数据容器
// ============================================================================

struct EulerData {                          // 单位 rad
    double roll  = 0.0;
    double pitch = 0.0;
    double yaw   = 0.0;
};

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

struct QuatData {
    double q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
};

// 完整快照（主线程通过 get_state() 一次拿到所有最新值）
struct ImuState {
    EulerData euler;                        // rad
    Vec3      gyro;                         // rad/s
    Vec3      acc;                          // g（默认协议输出）
    QuatData  quat;
    double    sensor_temp = 0.0;            // ℃
    uint32_t  sample_timestamp = 0;         // 设备端 ms
    double    last_rx_time    = 0.0;        // monotonic seconds（C++ 端）
    bool      valid           = false;
};

// ============================================================================
// IMU Reader
// ============================================================================

class ImuReader {
public:
    explicit ImuReader(const std::string& port = "/dev/wheeltec_IMU",
                        int baudrate = 460800,
                        bool debug = false);
    ~ImuReader();

    // 不可复制（持有 fd + thread）
    ImuReader(const ImuReader&) = delete;
    ImuReader& operator=(const ImuReader&) = delete;

    // 打开串口 + 启动后台线程。返回 false 表示打开/配置失败。
    bool start();
    void stop();
    bool is_running() const { return running_.load(); }
    const std::string& port() const { return port_; }

    // 线程安全 getter
    EulerData get_euler() const;
    Vec3      get_gyro()  const;
    Vec3      get_acc()   const;
    QuatData  get_quat()  const;
    ImuState  get_state() const;            // 整个快照

    // 诊断
    uint64_t rx_frame_count()  const { return rx_frame_count_.load(); }
    uint64_t crc_err_count()   const { return crc_err_count_.load(); }
    uint64_t no_header_count() const { return no_header_count_.load(); }

private:
    // ---- 协议常数（与 Python 严格一致）----
    static constexpr uint8_t  HDR1 = 0x59;
    static constexpr uint8_t  HDR2 = 0x53;
    static constexpr size_t   MIN_FRAME_LEN = 7;     // 头2+TID2+Len1+CRC2
    static constexpr size_t   PAYLOAD_POS   = 5;
    static constexpr size_t   DECODE_BUF_CAP = 1024;
    static constexpr size_t   UART_READ_CHUNK = 512;

    // Payload data IDs（部分常用）
    static constexpr uint8_t ID_TEMP             = 0x01;
    static constexpr uint8_t ID_ACCEL            = 0x10;
    static constexpr uint8_t ID_GYRO             = 0x20;
    static constexpr uint8_t ID_MAG_NORM         = 0x30;
    static constexpr uint8_t ID_MAG_RAW          = 0x31;
    static constexpr uint8_t ID_EULER            = 0x40;
    static constexpr uint8_t ID_QUAT             = 0x41;
    static constexpr uint8_t ID_UTC              = 0x50;
    static constexpr uint8_t ID_SAMPLE_TIMESTAMP = 0x51;
    static constexpr uint8_t ID_DATA_READY_TS    = 0x52;
    static constexpr uint8_t ID_LOCATION         = 0x68;
    static constexpr uint8_t ID_SPEED            = 0x70;
    static constexpr uint8_t ID_STATUS           = 0x80;

    // 解析返回码
    enum ParseRet {
        ANALYSIS_OK   =  0,
        DATA_LEN_ERR  = -2,
        CRC_ERR       = -3,
        NO_HEADER     =  5,
    };

    // ---- 串口配置 ----
    bool open_serial();
    void close_serial();

    // ---- 后台线程 ----
    void rx_loop();

    // ---- 解析核心 ----
    // 在 decode_buffer_ 上原地解析一帧；返回 ANALYSIS_OK 时写入 out。
    // 用 st_idx / end_idx 告知调用方需要从缓冲移除的范围。
    int parse_one_frame(int& st_idx, int& end_idx, ImuState& out);
    int parse_payload_block(uint8_t id, uint8_t len,
                             const uint8_t* data, ImuState& out);
    static uint16_t crc16(const uint8_t* data, size_t len);

    // ---- 工具 ----
    static double monotonic_seconds();

    // ---- 状态 ----
    std::string port_;
    int  baudrate_;
    bool debug_;

    int  fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread       rx_thread_;

    std::vector<uint8_t> decode_buffer_;

    mutable std::mutex state_mtx_;
    ImuState state_;

    std::atomic<uint64_t> rx_frame_count_{0};
    std::atomic<uint64_t> crc_err_count_{0};
    std::atomic<uint64_t> no_header_count_{0};
};

}  // namespace imu
}  // namespace motor_test_cpp
