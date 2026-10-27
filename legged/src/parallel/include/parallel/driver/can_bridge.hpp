// ============================================================================
// 文件: can_bridge.hpp
// 模块: parallel::driver (CAN 驱动层)
// 职责: 单通道 SocketCAN 桥接 —— 后台接收线程解析电机状态，同步发送控制帧，
//       通过内核错误帧实时推导总线状态 (Active/Warning/Passive/BusOff)。
// ============================================================================

#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>   // std::pair (arm_private_vmc_batch)
#include <vector>    // std::vector (arm_private_vmc_batch)

// 前置声明 SocketCAN 帧（避免头文件引入 <linux/can.h> 内核头，仅在 .cpp 中包含完整定义）
struct can_frame;

namespace parallel {
namespace can_bridge {

// ============================================================================
// CAN 总线状态（从内核错误帧实时推导）
// ============================================================================
//
// 来源：开启 CAN_RAW_ERR_FILTER 后，rx_loop 会收到带 CAN_ERR_FLAG 的错误帧。
// 解析其中 CAN_ERR_CRTL 子类的 controller-state 位 → 推出当前状态。
//
// 状态机：
//   Active   (TEC<128 且 REC<128)  ← 健康
//   Warning  (TEC>=96 或 REC>=96)   ← 接近 Passive，已开始累积错误
//   Passive  (TEC>=128 或 REC>=128) ← 受罚状态，TX 被插延时，带宽暴跌
//   BusOff   (TEC>=256)              ← 彻底死，必须 restart 才能恢复
//   Down                              ← 接口被关
//   Unknown                           ← 还没收到过任何状态指示（启动初）
//
// 控制器对 Passive/BusOff 的响应（main 里实现）：
//   - Passive → 立刻切 safe-hold，少发激进命令，让 TEC 衰减回 Active
//   - BusOff  → 等 restart-ms 100 自动 up；若 3s 不恢复 → E-STOP
enum class BusState : uint8_t {
    Active  = 0,
    Warning = 1,
    Passive = 2,
    BusOff  = 3,
    Down    = 4,
    Unknown = 5,
};

inline const char* bus_state_name(BusState s) {
    switch (s) {
        case BusState::Active:  return "ACTIVE";
        case BusState::Warning: return "WARNING";
        case BusState::Passive: return "PASSIVE";
        case BusState::BusOff:  return "BUS-OFF";
        case BusState::Down:    return "DOWN";
        default:                return "UNKNOWN";
    }
}

// ============================================================================
// 电机状态缓存（对应 Python MotorState）
// ============================================================================

struct MotorState {
    int      motor_id = 0;
    std::string protocol = "unknown";  // "private" / "mit" / "unknown"
    double   pos = 0.0;
    double   vel = 0.0;
    double   tor = 0.0;
    double   temp = 0.0;
    std::optional<int> pattern;
    std::optional<int> error_code;
    std::optional<int> run_mode;
    std::optional<uint32_t> raw_id;
    double   last_rx_time = 0.0;       // monotonic seconds
    bool     last_tx_ok = true;
    int      tx_fail_count = 0;
    int      rx_count = 0;
    std::string note;
};

// ============================================================================
// CanBridge —— 单通道
// ============================================================================

class CanBridge {
public:
    explicit CanBridge(const std::string& channel = "can0",
                       uint8_t master_id = 0xFD,
                       bool debug = false);
    ~CanBridge();

    // 不可复制（持有 socket fd + thread）
    CanBridge(const CanBridge&) = delete;
    CanBridge& operator=(const CanBridge&) = delete;

    // ---- 生命周期 ----
    bool connect();
    void disconnect();

    bool is_connected() const { return socket_fd_ >= 0; }
    const std::string& channel() const { return channel_; }

    // ---- 状态查询（线程安全）----
    MotorState get_motor_state(int motor_id) const;

    // ---- 发送：私有扩展帧 ----
    bool private_enable(int motor_id);
    bool private_disable(int motor_id, uint8_t clear_error = 0);
    bool set_motor_mode(int motor_id, uint8_t mode);
    bool set_current_limit(int motor_id, float limit_a);
    bool set_proactive_report(int motor_id, bool enable = true);
    // 私有协议运控帧（运动控制）：torque 进扩展 ID，pos/vel/kp/kd 各 16-bit 大端进 data。
    //   pos/vel/kp/kd/torque 5 参数 = 经典 MIT 运控语义；编码三方核验（参考 Python + 官方 RS01 + can_protocol.hpp）。
    bool private_motion_control(int motor_id, double pos, double vel,
                                double kp, double kd, double torque);

    // ---- arm 序列 ----
    // 私有协议运控 arm（对应 Python arm_private_vmc，6 步 + 开主动上报）。阻塞 ~1.7s。
    void arm_private_vmc(int motor_id, float current_limit = 20.0f);

    // ---- 诊断 ----
    uint64_t tx_raw_count() const { return tx_raw_count_.load(); }
    uint64_t rx_raw_count() const { return rx_raw_count_.load(); }

    // ---- 总线状态（实时，来自内核错误帧）----
    // 用法：main 每帧或 monitor 线程读，看到 Passive/BusOff 切 safe-hold。
    BusState bus_state() const {
        return static_cast<BusState>(bus_state_.load(std::memory_order_acquire));
    }
    // 累计错误帧个数（每收到一条错误帧 +1），便于跑后诊断
    uint64_t err_frame_count() const { return err_frame_count_.load(); }

private:
    // 帧发送底层
    bool send_frame(uint32_t arb_id, bool extended,
                    const uint8_t data[8], int dlc, int motor_id_for_tag);

    // rx 线程主循环
    void rx_loop();
    void parse_private_frame(uint32_t arb_id, const uint8_t data[8], int dlc);
    void parse_mit_frame(uint32_t arb_id, const uint8_t data[8], int dlc);
    // 错误帧（CAN_ERR_FLAG 置位）→ 更新 bus_state_。
    // 用全局 ::can_frame 避免在 namespace 内引入未定义的同名 struct；
    // cpp 里 #include <linux/can.h> 提供完整定义。
    void handle_error_frame(const ::can_frame& f);

    // 确保状态表里有该 motor_id
    MotorState& ensure_state(int motor_id);  // 必须在持锁时调用

    std::string channel_;
    uint8_t     master_id_;
    bool        debug_;

    int                socket_fd_ = -1;
    std::atomic<bool>  running_{false};
    std::thread        rx_thread_;

    mutable std::mutex state_mtx_;
    std::map<int, MotorState> motor_states_;

    std::atomic<uint64_t> tx_raw_count_{0};
    std::atomic<uint64_t> rx_raw_count_{0};

    // 总线状态（rx_loop 解析错误帧时更新；外部只读）。
    // 初值 Unknown：还没收到任何错误帧时不假设健康，main 的 pre-flight 走 netlink 直接读。
    std::atomic<uint8_t>  bus_state_{static_cast<uint8_t>(BusState::Unknown)};
    std::atomic<uint64_t> err_frame_count_{0};
};

// ============================================================================
// arm_private_vmc_batch —— 多电机流水线 arm（替代逐个串行 arm，总耗时从 ~21s 降至 ~2s）
// ============================================================================
// 对每个电机执行 arm_private_vmc 的 6 步，但所有电机同步走每一步、共用一次 sleep，
// 每个电机经历的步间时序、安全限流、重发次数与单电机 arm 完全一致。
// motors: (CanBridge*, motor_id) 列表，支持跨多条 CAN 通道。
// ============================================================================
void arm_private_vmc_batch(const std::vector<std::pair<CanBridge*, int>>& motors,
                           float current_limit = 15.0f);

}  // namespace can_bridge
}  // namespace parallel
