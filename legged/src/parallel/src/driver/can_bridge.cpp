// can_bridge.cpp —— SocketCAN 实现。
//
// 直接走 PF_CAN raw socket，与 python-can 等价的 Linux SocketCAN 路径。
//
// ============================================================================
// 线程安全约束
// ============================================================================
//
// 1. **生命周期函数（connect/disconnect/析构）只能从单一管理线程调用**。
//    并发调用 disconnect 之间存在 close-after-close race（运行期会得到 EBADF
//    或错关到别人的 fd）。本仓 Python 节点用 SingleThreadedExecutor，天然满足。
//
// 2. **send_*（mit_control / private_* / arm_*）必须晚于 connect、早于 disconnect**。
//    socket_fd_ 不是 atomic；send_frame 读它时若同时 disconnect 把它置 -1，
//    会写到已关闭的 fd。同上：单管理线程约束自然规避。
//
// 3. **get_motor_state / motor 状态读取可跨线程**。state_mtx_ 保护。
//
// 4. **rx 线程是 bridge 自己内部起的**，外部不要触碰。
//
// ============================================================================
// RT 路径风险（与 Python 等价）
// ============================================================================
//
// ::write(socket_fd_, ...) 默认是阻塞 write。CAN 总线拥塞或队列满时，
// write 会阻塞或返回 -1/ENOBUFS。Python python-can 行为相同。
// 如果未来要把发送从 RT 线程发起（slice #5/#6 集成时），考虑：
//   a) 改非阻塞 + 重试 + 丢帧策略
//   b) 把发送丢到独立 tx 线程（与 RT 线程解耦）
//   c) 接受阻塞但容忍 RT 抖动（< 几十 μs）
// 当前实现走 (c)，与 Python 一致。

#include "parallel/driver/can_bridge.hpp"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cmath>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>     // CAN_ERR_FLAG / CAN_ERR_CRTL / CAN_ERR_BUSOFF...
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "parallel/driver/can_protocol.hpp"

namespace parallel {
namespace can_bridge {

namespace cp = parallel::can_proto;

namespace {
inline double monotonic_seconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}
}  // namespace

CanBridge::CanBridge(const std::string& channel, uint8_t master_id, bool debug)
    : channel_(channel), master_id_(master_id), debug_(debug) {}

CanBridge::~CanBridge() {
    disconnect();
}

bool CanBridge::connect() {
    if (socket_fd_ >= 0) return true;

    int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        if (debug_) std::fprintf(stderr, "[CAN:%s] socket() failed: %s\n",
                                  channel_.c_str(), std::strerror(errno));
        return false;
    }

    // 找接口 ifindex
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, channel_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        if (debug_) std::fprintf(stderr, "[CAN:%s] SIOCGIFINDEX failed: %s\n",
                                  channel_.c_str(), std::strerror(errno));
        ::close(fd);
        return false;
    }

    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        if (debug_) std::fprintf(stderr, "[CAN:%s] bind() failed: %s\n",
                                  channel_.c_str(), std::strerror(errno));
        ::close(fd);
        return false;
    }

    // 加大 socket 接收缓冲，应对 1kHz × N 电机的突发接收
    // 优先 SO_RCVBUFFORCE（需 CAP_NET_ADMIN，可绕过 rmem_max）；回退到 SO_RCVBUF
    {
        int rcvbuf_bytes = 4 * 1024 * 1024;  // 4MB → ~250k MIT 帧
        if (::setsockopt(fd, SOL_SOCKET, SO_RCVBUFFORCE,
                          &rcvbuf_bytes, sizeof(rcvbuf_bytes)) < 0) {
            // 非 root → 回退到 SO_RCVBUF（被钳到 rmem_max）
            ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF,
                          &rcvbuf_bytes, sizeof(rcvbuf_bytes));
        }
        if (debug_) {
            int actual = 0; socklen_t sz = sizeof(actual);
            ::getsockopt(fd, SOL_SOCKET, SO_RCVBUF, &actual, &sz);
            std::fprintf(stderr, "[CAN:%s] SO_RCVBUF=%d bytes (kernel doubled)\n",
                          channel_.c_str(), actual);
        }
    }

    // ★ 开启 CAN 错误帧上报（CAN_RAW_ERR_FILTER）
    // 内核在控制器状态变化（Warning/Passive/BusOff/RX overflow 等）时，
    // 通过 socket 推一条带 CAN_ERR_FLAG 的"错误帧"过来。rx_loop 解析这些帧
    // 实时更新 bus_state_，main 看到 Passive 就切 safe-hold。
    {
        can_err_mask_t err_mask = CAN_ERR_MASK;   // 接收所有类别错误
        if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                          &err_mask, sizeof(err_mask)) < 0) {
            std::fprintf(stderr, "[CAN:%s] CAN_RAW_ERR_FILTER 失败: %s (继续，但 bus_state 拿不到实时更新)\n",
                          channel_.c_str(), std::strerror(errno));
        }
    }

    socket_fd_ = fd;
    running_ = true;
    rx_thread_ = std::thread(&CanBridge::rx_loop, this);

    // ★ rx 线程 RT 优先级 + CPU 亲和性
    //   prio 70 (低于主控制环 80，高于普通线程)：保证错误帧/电机回复及时处理，
    //   但绝不抢占主 RT 循环。
    //   affinity CPU 1-3：避开 CPU 0（主 RT 循环跑在 CPU 0），杜绝主循环被
    //   rx 线程同核挤占。N150 是 4 核，4 个 bridge 的 rx 线程平摊到 1-3。
    {
        struct sched_param sp{};
        sp.sched_priority = 70;
        int rc = ::pthread_setschedparam(rx_thread_.native_handle(),
                                          SCHED_FIFO, &sp);
        if (rc != 0 && debug_) {
            std::fprintf(stderr, "[CAN:%s] rx 线程 SCHED_FIFO 失败: %s (继续，普通调度)\n",
                          channel_.c_str(), std::strerror(rc));
        }
        cpu_set_t cs;
        CPU_ZERO(&cs);
        // rx 线程亲和 CPU1-2（main 控制线程在 CPU3，CPU0 留给系统/sshd）
        CPU_SET(1, &cs); CPU_SET(2, &cs);
        rc = ::pthread_setaffinity_np(rx_thread_.native_handle(),
                                       sizeof(cs), &cs);
        if (rc != 0 && debug_) {
            std::fprintf(stderr, "[CAN:%s] rx 线程 affinity 失败: %s (继续)\n",
                          channel_.c_str(), std::strerror(rc));
        }
    }

    return true;
}

void CanBridge::disconnect() {
    if (!running_.exchange(false)) {
        // 还没启动过；关 fd 但不 join
        if (socket_fd_ >= 0) { ::close(socket_fd_); socket_fd_ = -1; }
        return;
    }
    if (rx_thread_.joinable()) rx_thread_.join();
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

// ============================================================================
// 状态查询
// ============================================================================

MotorState CanBridge::get_motor_state(int motor_id) const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    auto it = motor_states_.find(motor_id);
    if (it == motor_states_.end()) {
        MotorState s;
        s.motor_id = motor_id;
        return s;
    }
    return it->second;
}

MotorState& CanBridge::ensure_state(int motor_id) {
    auto it = motor_states_.find(motor_id);
    if (it == motor_states_.end()) {
        MotorState s;
        s.motor_id = motor_id;
        auto [iter, _] = motor_states_.emplace(motor_id, std::move(s));
        return iter->second;
    }
    return it->second;
}

// ============================================================================
// 底层帧发送
// ============================================================================

bool CanBridge::send_frame(uint32_t arb_id, bool extended,
                            const uint8_t data[8], int dlc, int motor_id_for_tag) {
    if (socket_fd_ < 0) return false;
    struct can_frame frame {};
    frame.can_id = arb_id;
    if (extended) frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = static_cast<__u8>(std::min(dlc, 8));
    std::memcpy(frame.data, data, frame.can_dlc);

    // 非阻塞发送 (MSG_DONTWAIT): 总线降级时 TX 队列满立即返回 EAGAIN/ENOBUFS 而非阻塞。
    //   本帧丢失计 tx_fail, 主循环立刻继续下一帧——偶尔丢帧远优于阻塞卡死整个控制环。
    ssize_t n = ::send(socket_fd_, &frame, sizeof(frame), MSG_DONTWAIT);
    if (n != static_cast<ssize_t>(sizeof(frame))) {
        // -1 + errno = EAGAIN/EWOULDBLOCK/ENOBUFS → TX 队列满，本帧丢
        // 其它 errno（EBADF/EFAULT/...）极少见，统一计 tx_fail
        std::lock_guard<std::mutex> lk(state_mtx_);
        auto& st = ensure_state(motor_id_for_tag);
        st.last_tx_ok = false;
        ++st.tx_fail_count;
        // note 区分原因，方便诊断
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS)) {
            st.note = "tx_full";   // CAN 总线节流（PASSIVE 等）
        } else {
            st.note = "send_fail"; // 其它写失败
        }
        return false;
    }
    ++tx_raw_count_;
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        auto& st = ensure_state(motor_id_for_tag);
        st.last_tx_ok = true;
    }
    return true;
}

// ============================================================================
// rx 线程
// ============================================================================

void CanBridge::rx_loop() {
    while (running_.load()) {
        struct pollfd pfd { socket_fd_, POLLIN, 0 };
        int pr = ::poll(&pfd, 1, /*timeout_ms=*/10);
        if (pr <= 0) continue;
        if (!(pfd.revents & POLLIN)) continue;

        // 批量排空：1kHz × 3 电机 → 短时间内可能堆 3+ 帧；一次 poll 唤醒
        // 后用 MSG_DONTWAIT 把全部已就绪帧读完，再回去 poll，避免帧
        // 在内核环形缓冲区排队导致 last_rx_time 滞后。
        while (running_.load()) {
            struct can_frame frame {};
            ssize_t n = ::recv(socket_fd_, &frame, sizeof(frame), MSG_DONTWAIT);
            if (n < 0) {
                // EAGAIN / EWOULDBLOCK = 环形缓冲区 空了，回 poll
                break;
            }
            if (n != sizeof(frame)) break;
            ++rx_raw_count_;

            // ★ 错误帧分流：CAN_ERR_FLAG 置位 = 内核报告的控制器状态变化
            // 不进入 mit/private 解析（IDs 没含义），只用于更新 bus_state_。
            if (frame.can_id & CAN_ERR_FLAG) {
                handle_error_frame(frame);
                continue;
            }

            const bool is_ext = (frame.can_id & CAN_EFF_FLAG) != 0;
            const uint32_t arb = is_ext
                ? (frame.can_id & CAN_EFF_MASK)
                : (frame.can_id & CAN_SFF_MASK);

            if (is_ext) parse_private_frame(arb, frame.data, frame.can_dlc);
            else        parse_mit_frame(arb, frame.data, frame.can_dlc);
        }
    }
}

// ============================================================================
// 错误帧解析：内核 → BusState
//   错误类别在 frame.can_id 低位（CAN_ERR_BUSOFF / CAN_ERR_CRTL / CAN_ERR_RESTARTED ...）
//   控制器状态位（Warning/Passive 等）在 frame.data[1]
// ============================================================================
void CanBridge::handle_error_frame(const struct can_frame& f) {
    err_frame_count_.fetch_add(1, std::memory_order_relaxed);
    const uint32_t cls = f.can_id;   // 错误帧 ID 字段编码错误类别

    BusState new_state = bus_state();   // 先取当前；下面按类别可能改

    if (cls & CAN_ERR_BUSOFF) {
        new_state = BusState::BusOff;
    } else if (cls & CAN_ERR_RESTARTED) {
        // 内核自动重启完成（restart-ms 触发）→ 回到 Active
        new_state = BusState::Active;
    } else if (cls & CAN_ERR_CRTL) {
        // 控制器状态变化：看 data[1] 位
        const uint8_t s = f.data[1];
        if (s & (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE)) {
            new_state = BusState::Passive;
        } else if (s & (CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING)) {
            new_state = BusState::Warning;
        } else if (s == CAN_ERR_CRTL_ACTIVE || s == 0) {
            // 显式 ACTIVE 或所有错误位清零 → 恢复 Active
            new_state = BusState::Active;
        }
        // 其它位（RX/TX overflow）不改状态，只是计数
    }
    // CAN_ERR_PROT / CAN_ERR_ACK / CAN_ERR_LOSTARB 等不改状态（这些是单帧错误，
    // 累计到一定量后内核会再发 CAN_ERR_CRTL 状态变化帧）。

    bus_state_.store(static_cast<uint8_t>(new_state), std::memory_order_release);

    if (debug_) {
        std::fprintf(stderr, "[CAN:%s] ★error frame cls=0x%x data[1]=0x%02x → state=%s\n",
                      channel_.c_str(), cls, f.data[1], bus_state_name(new_state));
    }
}

void CanBridge::parse_private_frame(uint32_t arb_id, const uint8_t data[8], int dlc) {
    const uint8_t comm_type = (arb_id >> 24) & 0x3F;
    if (comm_type == cp::COMM_MOTOR_REQUEST) {
        if (dlc != 8) return;
        std::array<uint8_t, 8> d;
        for (int i = 0; i < 8; ++i) d[i] = data[i];
        auto fb = cp::private_decode(arb_id, d);

        std::lock_guard<std::mutex> lk(state_mtx_);
        auto& st = ensure_state(fb.motor_id);
        st.protocol = "private";
        st.pos      = fb.pos;
        st.vel      = fb.vel;
        st.tor      = fb.tor;
        st.temp     = fb.temp;
        st.pattern  = fb.pattern;
        st.error_code = fb.error_code;
        st.raw_id   = arb_id;
        st.last_rx_time = monotonic_seconds();
        ++st.rx_count;
        if (fb.error_code != 0) st.note = "private_error=" + std::to_string(fb.error_code);
        else if (fb.pattern != 2) st.note = "private_pattern=" + std::to_string(fb.pattern);
        else st.note.clear();
        return;
    }

    // COMM_ERROR_FEEDBACK：故障反馈帧
    //
    // 私有协议设计：故障信息**全部编码在 arb_id**（高位 bit field）里：
    //   bit[28:24] = comm_type (= 0x15)
    //   bit[23:22] = pattern
    //   bit[21:16] = error_code (6-bit)
    //   bit[15:8]  = motor_id
    // data 字段厂商自定义不解析（手册没说清楚，留原始数据不动）。
    //
    // 上层通过 get_motor_state(motor_id).error_code / pattern 读出来诊断。
    if (comm_type == cp::COMM_ERROR_FEEDBACK) {
        const uint8_t motor_id   = (arb_id >> 8) & 0xFF;
        const uint8_t pattern    = (arb_id >> 22) & 0x03;
        const uint8_t error_code = (arb_id >> 16) & 0x3F;
        std::lock_guard<std::mutex> lk(state_mtx_);
        auto& st = ensure_state(motor_id);
        st.protocol = "private";
        st.pattern  = pattern;
        st.error_code = error_code;
        st.raw_id   = arb_id;
        st.last_rx_time = monotonic_seconds();
        ++st.rx_count;
        st.note = "fault_feedback code=" + std::to_string(error_code);
    }
}

// ----------------------------------------------------------------------------
// MIT 协议有两种应答帧布局（StdId 区分）：
//
//   应答 1（正常运动反馈）：
//     StdId  = master_id (0xFD)
//     data[0]    = motor_id（电机自己填）
//     data[1..7] = pos/vel/tor/temp 编码
//     故障变体：data[3..7] 全 0 → data[1..2] 是 16-bit 故障码
//
//   应答 2（ACK2，特殊命令回复）：
//     StdId  = motor_id（电机自己的 ID）
//     data[0..7] = 64-bit MCU UID（厂家烧的唯一标识符）
//     用途：设零位 / 改 CAN ID / 切协议 后的确认
//
// 注意：本桥是**全局 rx 监听器**（一个 socket 收所有帧），
//   - ACK1 用 StdId == master_id 过滤——天然唯一
//   - ACK2 用 StdId == motor_id 过滤——但总线上随便一个标准帧（比如另一台
//     电机发的 ACK1，其 StdId 也可能撞 motor_id 命名空间）会被误判
//     → 所以下面用"该 motor_id 必须已在 motor_states_ map 里"做过滤，
//       未知 motor_id 的 ACK2 直接丢弃
//   对照 Python：python-can 版本在 逐电机实例里调，天然只看自己的帧；
//   C++ 全局监听器必须显式做这个 已知限定过滤。
// ----------------------------------------------------------------------------
void CanBridge::parse_mit_frame(uint32_t arb_id, const uint8_t data[8], int dlc) {
    if (dlc != 8) return;
    const uint16_t std_id = arb_id & 0x7FF;

    // 应答 1: StdId == master_id（0xFD）→ 数据帧或故障帧
    if ((std_id & 0xFF) == master_id_) {
        std::array<uint8_t, 8> d;
        for (int i = 0; i < 8; ++i) d[i] = data[i];
        auto fb = cp::mit_decode(d);

        std::lock_guard<std::mutex> lk(state_mtx_);
        auto& st = ensure_state(fb.motor_id);
        st.protocol = "mit";
        st.raw_id = arb_id;
        st.last_rx_time = monotonic_seconds();
        ++st.rx_count;
        if (fb.is_fault) {
            st.error_code = fb.fault_code;
            char buf[64];
            std::snprintf(buf, sizeof(buf), "mit_fault=0x%04x", fb.fault_code);
            st.note = buf;
        } else {
            st.pos  = fb.pos;
            st.vel  = fb.vel;
            st.tor  = fb.tor;
            st.temp = fb.temp;
            st.error_code = 0;
            st.note.clear();
        }
        return;
    }

    // 应答 2: StdId == motor_id（ACK2）
    //   仅对**已知** motor_id 处理，避免把无关帧污染到状态表里
    //   (见 parse_mit_frame 顶部注释关于全局监听器 vs 逐电机实例的差异)
    const uint8_t motor_id = std_id & 0xFF;
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (motor_states_.find(motor_id) == motor_states_.end()) return;
    auto& st = motor_states_[motor_id];
    st.protocol = "mit";
    st.raw_id   = arb_id;
    st.last_rx_time = monotonic_seconds();
    ++st.rx_count;
    st.note = "mit_ack2";
    // 注：ACK2 的 data[0..7] 是 MCU UID，可解析记录但当前无人消费
}

// ============================================================================
// 私有协议 API
// ============================================================================

bool CanBridge::private_enable(int motor_id) {
    const uint32_t id = cp::private_ext_id(cp::COMM_MOTOR_ENABLE,
                                            static_cast<uint8_t>(motor_id),
                                            master_id_);
    uint8_t d[8] = {0};
    return send_frame(id, /*ext=*/true, d, 8, motor_id);
}

bool CanBridge::private_disable(int motor_id, uint8_t clear_error) {
    const uint32_t id = cp::private_ext_id(cp::COMM_MOTOR_STOP,
                                            static_cast<uint8_t>(motor_id),
                                            master_id_);
    uint8_t d[8] = {clear_error, 0, 0, 0, 0, 0, 0, 0};
    return send_frame(id, true, d, 8, motor_id);
}

// 私有协议 "SET_SINGLE_PARAM"（COMM 类型 0x12）的 8 字节 data 布局：
//   d[0..1] = 寄存器地址 (16-bit little-endian)
//   d[2..3] = 保留 (0)
//   d[4..7] = 参数值（类型按寄存器定）
//
// REG_RUN_MODE = 0x7005 → d[0]=0x05, d[1]=0x70；mode 是 uint8 → d[4]
bool CanBridge::set_motor_mode(int motor_id, uint8_t mode) {
    const uint32_t id = cp::private_ext_id(cp::COMM_SET_SINGLE_PARAM,
                                            static_cast<uint8_t>(motor_id),
                                            master_id_);
    uint8_t d[8] = {0x05, 0x70, 0x00, 0x00, mode, 0x00, 0x00, 0x00};
    bool ok = send_frame(id, true, d, 8, motor_id);
    if (ok) {
        // 发送成功只代表帧已上总线，**不代表**电机模式已切换；
        // 真正切换的确认应等 rx 回 COMM_MOTOR_REQUEST 看 pattern 字段。
        // 这里乐观更新 run_mode 缓存仅为诊断之用。
        std::lock_guard<std::mutex> lk(state_mtx_);
        auto& st = ensure_state(motor_id);
        st.run_mode = mode;
    }
    return ok;
}

// REG_CURRENT_LIMIT = 0x7018 → d[0]=0x18, d[1]=0x70；参数是 float32 → d[4..7]
//
// ⚠ 假设 host 是 little-endian。x86_64 / aarch64（Raspberry Pi、Jetson 等
//   常见嵌入式平台）都是 LE，本仓不打算支持 大端序主机；若必须支持，
//   把 memcpy 改成手动字节拆分即可。
bool CanBridge::set_current_limit(int motor_id, float limit_a) {
    limit_a = std::max(0.0f, std::min(limit_a, 23.0f));  // 与 Python 一致硬限 23A
    const uint32_t id = cp::private_ext_id(cp::COMM_SET_SINGLE_PARAM,
                                            static_cast<uint8_t>(motor_id),
                                            master_id_);
    uint8_t d[8] = {0x18, 0x70, 0x00, 0x00, 0, 0, 0, 0};
    std::memcpy(&d[4], &limit_a, 4);
    return send_frame(id, true, d, 8, motor_id);
}

// COMM_PROACTIVE_REPORT = 0x18：开/关电机主动上报状态。
//   data = {0x01,0x02,0x03,0x04,0x05,0x06, enable?1:0, 0x08}（与 Python set_proactive_report 一致）。
//   私有运控下电机据此持续回 COMM_MOTOR_REQUEST 反馈帧，rx 线程异步接入 motor_states_。
bool CanBridge::set_proactive_report(int motor_id, bool enable) {
    const uint32_t id = cp::private_ext_id(cp::COMM_PROACTIVE_REPORT,
                                            static_cast<uint8_t>(motor_id),
                                            master_id_);
    uint8_t d[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                    static_cast<uint8_t>(enable ? 0x01 : 0x00), 0x08};
    return send_frame(id, /*ext=*/true, d, 8, motor_id);
}

// 私有协议运控帧（运动控制）——MIT mit_control 的私有等价。
//   torque 编码进扩展 ID 的中段 16-bit；pos/vel/kp/kd 各 16-bit 大端进 8 字节 data。
//   编码逐字节三方核验：参考 Python send_motion_control_cmd + 官方 RS01 RobStride_Motor_move_control。
bool CanBridge::private_motion_control(int motor_id, double pos, double vel,
                                        double kp, double kd, double torque) {
    // 清洗 NaN/Inf（防 float_to_uint UB → 编码出乱帧），与 mit_control 同策略。
    auto sanitize = [](double x) { return std::isfinite(x) ? x : 0.0; };
    pos = sanitize(pos); vel = sanitize(vel);
    kp  = sanitize(kp);  kd  = sanitize(kd);
    torque = sanitize(torque);

    const uint32_t id = cp::private_motion_id(static_cast<uint8_t>(motor_id), torque);
    const auto d = cp::private_motion_data(pos, vel, kp, kd);
    return send_frame(id, /*ext=*/true, d.data(), 8, motor_id);
}

// ============================================================================
// arm 序列
// ============================================================================
//
// arm_private_vmc：私有协议进入 VMC 运控模式（对照官方 Robstride01.cpp 初始化流程 +
//   已验证参考 Function_can.py arm_private_vmc）。6 步，步间 sleep 等电机状态机切换：
//     1) 失能 + 清错  2) 设运控模式(0x7005=MODE_MOTION)  3) 设电流限制(0x7018)
//     4) 再确认一次运控模式  5) 使能  6) 开主动上报 + 查 pattern==2
//   阻塞约 1.7s。**不要在 RT 路径里调** —— init 阶段一次性 arm。
//   ⚠ MIT arm 是 3 步 150ms；私有 arm 慢得多（寄存器写需电机处理），sleep 与验证过的参考一致。
void CanBridge::arm_private_vmc(int motor_id, float current_limit) {
    using std::chrono::milliseconds;
    // Step 1: 失能 + 清错
    private_disable(motor_id, /*clear_error=*/1);
    std::this_thread::sleep_for(milliseconds(300));
    // Step 2: 设运控模式
    set_motor_mode(motor_id, cp::MODE_MOTION);
    std::this_thread::sleep_for(milliseconds(300));
    // Step 3: 设电流限制 —— ★安全关键。send_frame 非阻塞(TX满会丢帧)，限流只发一次若丢=
    //   电机跑在无限流态(危险)。故重发 3 次(写同一寄存器幂等)，并检查至少一帧上了总线；
    //   全失败则大声告警(电机可能无限流，别继续盲跑)。
    bool cl_sent = false;
    for (int k = 0; k < 3; ++k) {
        if (set_current_limit(motor_id, current_limit)) cl_sent = true;
        std::this_thread::sleep_for(milliseconds(20));
    }
    if (!cl_sent) {
        std::fprintf(stderr, "[CAN:%s] ⚠⚠ arm_private_vmc motor %d: 限流帧 3 次都没发出(TX满)! "
                      "电机可能无电流限制，请检查总线/重试 arm！\n", channel_.c_str(), motor_id);
    }
    std::this_thread::sleep_for(milliseconds(260));   // 配齐原 ~300ms 步间间隔（3×20 + 260≈320）
    // Step 4: 再确认一次运控模式
    set_motor_mode(motor_id, cp::MODE_MOTION);
    std::this_thread::sleep_for(milliseconds(300));
    // Step 5: 使能
    private_enable(motor_id);
    std::this_thread::sleep_for(milliseconds(500));
    // Step 6: 开主动上报（私有运控靠它持续回反馈），再查 pattern
    set_proactive_report(motor_id, true);
    std::this_thread::sleep_for(milliseconds(50));

    MotorState st = get_motor_state(motor_id);
    if (st.rx_count == 0) {
        std::fprintf(stderr, "[CAN:%s] ⚠ arm_private_vmc motor %d: 无回复! 查接线/ID/通道/上电\n",
                      channel_.c_str(), motor_id);
    } else if (st.pattern.value_or(-1) == 2) {
        if (debug_) std::fprintf(stderr, "[CAN:%s] arm_private_vmc motor %d: 使能成功 pattern=2\n",
                                  channel_.c_str(), motor_id);
    } else {
        std::fprintf(stderr, "[CAN:%s] arm_private_vmc motor %d: pattern=%d (期望2)，重试使能...\n",
                      channel_.c_str(), motor_id, st.pattern.value_or(-1));
        private_enable(motor_id);
        std::this_thread::sleep_for(milliseconds(300));
    }
}

// arm_private_vmc_batch：多电机【流水线】私有 arm。每步对所有电机各发一遍, 再共用一次 sleep
//   → 总耗时 ~2s 而非 12×1.7s≈21s。每电机步间时序与单个 arm_private_vmc 一致(见声明注释)。
void arm_private_vmc_batch(const std::vector<std::pair<CanBridge*, int>>& motors,
                           float current_limit) {
    using std::chrono::milliseconds;
    if (motors.empty()) return;
    auto nap = [](int ms) { std::this_thread::sleep_for(milliseconds(ms)); };

    // Step 1: 失能 + 清错（all）
    for (const auto& pm : motors) pm.first->private_disable(pm.second, /*clear_error=*/1);
    nap(300);
    // Step 2: 设运控模式（all）
    for (const auto& pm : motors) pm.first->set_motor_mode(pm.second, cp::MODE_MOTION);
    nap(300);
    // Step 3: 设电流限制（all, ★安全关键）—— 重发 3 次(幂等), 逐电机检查至少一帧上总线
    std::vector<bool> cl_sent(motors.size(), false);
    for (int k = 0; k < 3; ++k) {
        for (std::size_t i = 0; i < motors.size(); ++i)
            if (motors[i].first->set_current_limit(motors[i].second, current_limit)) cl_sent[i] = true;
        nap(20);
    }
    for (std::size_t i = 0; i < motors.size(); ++i)
        if (!cl_sent[i])
            std::fprintf(stderr, "[CAN] ⚠⚠ arm_batch motor %d: 限流帧 3 次都没发出(TX满)! 电机可能无限流!\n",
                          motors[i].second);
    nap(260);
    // Step 4: 再确认运控模式（all）
    for (const auto& pm : motors) pm.first->set_motor_mode(pm.second, cp::MODE_MOTION);
    nap(300);
    // Step 5: 使能（all）
    for (const auto& pm : motors) pm.first->private_enable(pm.second);
    nap(500);
    // Step 6: 开主动上报（all）
    for (const auto& pm : motors) pm.first->set_proactive_report(pm.second, true);
    nap(50);

    // 逐电机检查 + 重试（与单个 arm 一致）
    for (const auto& pm : motors) {
        MotorState st = pm.first->get_motor_state(pm.second);
        if (st.rx_count == 0) {
            std::fprintf(stderr, "[CAN] ⚠ arm_batch motor %d: 无回复! 查接线/ID/通道/上电\n", pm.second);
        } else if (st.pattern.value_or(-1) != 2) {
            std::fprintf(stderr, "[CAN] arm_batch motor %d: pattern=%d (期望2)，重试使能...\n",
                          pm.second, st.pattern.value_or(-1));
            pm.first->private_enable(pm.second);
        }
    }
    nap(300);   // 给重试的电机稳定下来
}

}  // namespace can_bridge
}  // namespace parallel
