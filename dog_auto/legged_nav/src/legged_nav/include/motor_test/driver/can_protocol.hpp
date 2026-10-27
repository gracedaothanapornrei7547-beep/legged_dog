// can_protocol.hpp —— CAN 协议层（纯函数）。
//
// 与 motor_test/driver/can.py 中 _uint_to_float / _float_to_uint /
// MIT control 编码、私有扩展帧 ID 构造逐位对齐。
//
// 设计原则：
//   - 全部 inline / constexpr / 自由函数；没有状态；没有 I/O
//   - 测试可以纯算 byte-by-byte 对拍 Python，与硬件解耦

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>

namespace motor_test_cpp {
namespace can_proto {

// ============================================================================
// 物理量范围（与 Python 同值）
// ============================================================================

constexpr double P_MIN  = -12.5,  P_MAX  = 12.5;
constexpr double V_MIN  = -44.0,  V_MAX  = 44.0;
constexpr double T_MIN  = -17.0,  T_MAX  = 17.0;
constexpr double KP_MIN = 0.0,    KP_MAX = 500.0;
constexpr double KD_MIN = 0.0,    KD_MAX = 5.0;

// ============================================================================
// 通信类型 / 寄存器 / 模式（与 Python COMM_*、REG_*、MODE_* 同值）
// ============================================================================

constexpr uint8_t COMM_GET_ID           = 0x00;
constexpr uint8_t COMM_MOTION_CONTROL   = 0x01;
constexpr uint8_t COMM_MOTOR_REQUEST    = 0x02;
constexpr uint8_t COMM_MOTOR_ENABLE     = 0x03;
constexpr uint8_t COMM_MOTOR_STOP       = 0x04;
constexpr uint8_t COMM_SET_POS_ZERO     = 0x06;
constexpr uint8_t COMM_CAN_ID           = 0x07;
constexpr uint8_t COMM_GET_SINGLE_PARAM = 0x11;
constexpr uint8_t COMM_SET_SINGLE_PARAM = 0x12;
constexpr uint8_t COMM_ERROR_FEEDBACK   = 0x15;
constexpr uint8_t COMM_PROACTIVE_REPORT = 0x18;
constexpr uint8_t COMM_MOTOR_MODE_SET   = 0x19;

constexpr uint16_t REG_RUN_MODE       = 0x7005;
constexpr uint16_t REG_CURRENT_LIMIT  = 0x7018;

constexpr uint8_t MODE_MOTION = 0;
constexpr uint8_t MODE_POSITION = 1;
constexpr uint8_t MODE_SPEED = 2;
constexpr uint8_t MODE_CURRENT = 3;
constexpr uint8_t MODE_ZERO = 4;
constexpr uint8_t MODE_CSP = 5;

// MIT 特殊指令（标准帧 8 字节）
inline constexpr std::array<uint8_t, 8> MIT_ENABLE_CMD   = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
inline constexpr std::array<uint8_t, 8> MIT_DISABLE_CMD  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
inline constexpr std::array<uint8_t, 8> MIT_SET_ZERO_CMD = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

// 默认主控 ID
constexpr uint8_t MASTER_ID = 0xFD;

// ============================================================================
// 浮点 ↔ 定点（与 Python 严格一致）
// ============================================================================

inline uint32_t float_to_uint(double x, double x_min, double x_max, int bits) {
    // 注意：与 Python 一致——先 clamp 再缩放再 int()
    x = std::max(x_min, std::min(x, x_max));
    const uint32_t span = (1u << bits) - 1;
    return static_cast<uint32_t>((x - x_min) * span / (x_max - x_min));
}

inline double uint_to_float(uint32_t x, double x_min, double x_max, int bits) {
    const uint32_t span = (1u << bits) - 1;
    x &= span;
    return x_min + (static_cast<double>(x) * (x_max - x_min) / span);
}

inline double wrap_to_pi(double rad) {
    // 与 Python (rad+pi) % (2pi) - pi 一致——注意 std::fmod 行为
    double r = std::fmod(rad + M_PI, 2.0 * M_PI);
    if (r < 0.0) r += 2.0 * M_PI;
    return r - M_PI;
}

// ============================================================================
// MIT 标准帧编码：8 字节 data
// ============================================================================
//
// 字节布局（与 Python mit_control 一致）：
//   d[0] = (p >> 8) & 0xFF        ← p 16-bit 高字节
//   d[1] = p & 0xFF
//   d[2] = (v >> 4) & 0xFF        ← v 12-bit
//   d[3] = ((v & 0x0F) << 4) | ((kp >> 8) & 0x0F)   ← v 低 4 + kp 高 4
//   d[4] = kp & 0xFF
//   d[5] = (kd >> 4) & 0xFF
//   d[6] = ((kd & 0x0F) << 4) | ((t >> 8) & 0x0F)
//   d[7] = t & 0xFF

inline std::array<uint8_t, 8> mit_encode(
    double pos, double vel, double kp, double kd, double torque) {
    const uint32_t p_i  = float_to_uint(pos,    P_MIN,  P_MAX,  16);
    const uint32_t v_i  = float_to_uint(vel,    V_MIN,  V_MAX,  12);
    const uint32_t kp_i = float_to_uint(kp,     KP_MIN, KP_MAX, 12);
    const uint32_t kd_i = float_to_uint(kd,     KD_MIN, KD_MAX, 12);
    const uint32_t t_i  = float_to_uint(torque, T_MIN,  T_MAX,  12);

    return {
        static_cast<uint8_t>((p_i >> 8) & 0xFF),
        static_cast<uint8_t>( p_i        & 0xFF),
        static_cast<uint8_t>((v_i >> 4) & 0xFF),
        static_cast<uint8_t>(((v_i & 0x0F) << 4) | ((kp_i >> 8) & 0x0F)),
        static_cast<uint8_t>( kp_i       & 0xFF),
        static_cast<uint8_t>((kd_i >> 4) & 0xFF),
        static_cast<uint8_t>(((kd_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F)),
        static_cast<uint8_t>( t_i        & 0xFF),
    };
}

// ============================================================================
// MIT 应答帧解码：8 字节 → (motor_id, pos, vel, tor, temp)
// ============================================================================
//
// MIT 应答指令 1：
//   d[0]      = motor_id
//   d[1..2]   = pos 16-bit
//   d[3..4 高] = vel 12-bit
//   d[4 低..5]= tor 12-bit
//   d[6..7]   = temp ×10 16-bit
//
// 故障帧：d[3..7] 全 0 时表示故障（不在此处理，调用方处理）

struct MitFeedback {
    uint8_t motor_id;
    double  pos;
    double  vel;
    double  tor;
    double  temp;
    bool    is_fault;
    uint16_t fault_code;  // 故障帧的 d[1..2]
};

inline MitFeedback mit_decode(const std::array<uint8_t, 8>& d) {
    MitFeedback fb{};
    fb.motor_id = d[0];

    // 故障检测：d[3..7] 全 0
    if (d[3] == 0 && d[4] == 0 && d[5] == 0 && d[6] == 0 && d[7] == 0) {
        fb.is_fault = true;
        fb.fault_code = (static_cast<uint16_t>(d[1]) << 8) | d[2];
        return fb;
    }

    const uint32_t pos_raw  = (static_cast<uint32_t>(d[1]) << 8) | d[2];
    const uint32_t vel_raw  = (static_cast<uint32_t>(d[3]) << 4) | (d[4] >> 4);
    const uint32_t tor_raw  = ((static_cast<uint32_t>(d[4]) & 0x0F) << 8) | d[5];
    const uint32_t temp_raw = (static_cast<uint32_t>(d[6]) << 8) | d[7];

    fb.pos      = wrap_to_pi(uint_to_float(pos_raw, P_MIN, P_MAX, 16));
    fb.vel      = uint_to_float(vel_raw, V_MIN, V_MAX, 12);
    fb.tor      = uint_to_float(tor_raw, T_MIN, T_MAX, 12);
    fb.temp     = temp_raw * 0.1;
    fb.is_fault = false;
    return fb;
}

// ============================================================================
// 私有扩展帧 ID 构造（与 Python _ext_id 一致）
// ============================================================================
//
// ID 29-bit：comm_type[28:24] | middle_16[23:8] | motor_id[7:0]

inline uint32_t private_ext_id(uint8_t comm_type, uint8_t motor_id, uint16_t middle_16 = 0) {
    return (static_cast<uint32_t>(comm_type) << 24)
         | (static_cast<uint32_t>(middle_16) << 8)
         | (static_cast<uint32_t>(motor_id));
}

// 私有协议运动控制帧 ID：torque 编码进 middle_16
inline uint32_t private_motion_id(uint8_t motor_id, double torque) {
    const uint32_t t_u16 = float_to_uint(torque, T_MIN, T_MAX, 16);
    return (static_cast<uint32_t>(COMM_MOTION_CONTROL) << 24)
         | ((t_u16 & 0xFFFF) << 8)
         | static_cast<uint32_t>(motor_id);
}

// 私有协议运动控制帧 data：pos/vel/kp/kd 各 16-bit big-endian
inline std::array<uint8_t, 8> private_motion_data(double pos, double vel, double kp, double kd) {
    const uint32_t p_i  = float_to_uint(pos, P_MIN, P_MAX, 16);
    const uint32_t v_i  = float_to_uint(vel, V_MIN, V_MAX, 16);
    const uint32_t kp_i = float_to_uint(kp, KP_MIN, KP_MAX, 16);
    const uint32_t kd_i = float_to_uint(kd, KD_MIN, KD_MAX, 16);
    return {
        static_cast<uint8_t>((p_i >> 8) & 0xFF), static_cast<uint8_t>(p_i & 0xFF),
        static_cast<uint8_t>((v_i >> 8) & 0xFF), static_cast<uint8_t>(v_i & 0xFF),
        static_cast<uint8_t>((kp_i >> 8) & 0xFF), static_cast<uint8_t>(kp_i & 0xFF),
        static_cast<uint8_t>((kd_i >> 8) & 0xFF), static_cast<uint8_t>(kd_i & 0xFF),
    };
}

// ============================================================================
// 私有协议反馈帧解码（COMM_MOTOR_REQUEST）
// ============================================================================
//
// 扩展 ID 高位：comm_type[28:24] | pattern[23:22] | error[21:16] | motor_id[7:0]
// data 8 字节：4 个 16-bit 大端字段 (pos, vel, tor, temp)

struct PrivateFeedback {
    uint8_t  motor_id;
    uint8_t  comm_type;
    uint8_t  pattern;
    uint8_t  error_code;
    double   pos;
    double   vel;
    double   tor;
    double   temp;
};

inline PrivateFeedback private_decode(uint32_t arb_id, const std::array<uint8_t, 8>& d) {
    PrivateFeedback fb{};
    fb.comm_type  = (arb_id >> 24) & 0x3F;
    fb.pattern    = (arb_id >> 22) & 0x03;
    fb.error_code = (arb_id >> 16) & 0x3F;
    fb.motor_id   = (arb_id >>  8) & 0xFF;

    const uint32_t pos_raw  = (static_cast<uint32_t>(d[0]) << 8) | d[1];
    const uint32_t vel_raw  = (static_cast<uint32_t>(d[2]) << 8) | d[3];
    const uint32_t tor_raw  = (static_cast<uint32_t>(d[4]) << 8) | d[5];
    const uint32_t temp_raw = (static_cast<uint32_t>(d[6]) << 8) | d[7];

    fb.pos  = wrap_to_pi(uint_to_float(pos_raw, P_MIN, P_MAX, 16));
    fb.vel  = uint_to_float(vel_raw, V_MIN, V_MAX, 16);
    fb.tor  = uint_to_float(tor_raw, T_MIN, T_MAX, 16);
    fb.temp = temp_raw * 0.1;
    return fb;
}

}  // namespace can_proto
}  // namespace motor_test_cpp
