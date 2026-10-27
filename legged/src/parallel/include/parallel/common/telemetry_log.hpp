// =============================================================================
// 文件:        telemetry_log.hpp
// 模块:        parallel/common
// 描述:        用于 500 Hz 控制环的帧同步二进制遥测记录器 ——
//              零格式化、零阻塞。无锁 SPSC 环形缓冲区将 RT 控制线程
//              （生产者，每帧一次 memcpy）与后台写入线程（消费者，
//              批量 fwrite 至本地 SSD）解耦。缓冲区溢出时丢弃记录
//              并递增计数器，从不会阻塞控制环。
//
//              文件格式：TelemHeader (32 B) + N × TelemRecord (256 B)。
//              字段顺序与类型必须与 tools/parse_telem.py 保持同步。
//
//              SPSC 正确性：head 仅由生产者写入，tail 仅由消费者写入；
//              release/acquire 内存序保证已写入记录对消费者的可见性。
// =============================================================================

#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <thread>
#include <vector>

namespace parallel {
namespace telem {

// ============================================================================
// 单帧记录（定长 POD，256B）。字段顺序与 tools/parse_telem.py 严格对应。
// ============================================================================
struct TelemRecord {
    double  t;              // 运行时刻 (s)

    // ---- estimator ----
    float   est_z;          // pos_world.z (m)         机身估计高度
    float   est_vz_world;   // vel_world.z (m/s)       下沉速率
    float   est_vx_body;    // vel_body.x  (m/s)
    float   est_vy_body;    // vel_body.y  (m/s)
    float   est_roll;       // rad
    float   est_pitch;      // rad
    float   est_yaw;        // rad

    // ---- body_posture wrench ----
    float   wrench_fz;      // N      高度 PD 总力，用于诊断是否触顶/饱和
    float   wrench_mx;      // Nm
    float   wrench_my;      // Nm
    float   wrench_mz;      // Nm

    // ---- per-leg（顺序 FL FR BL BR）----
    float   support_w[4];   // support_weight ∈ [0,1]
    float   fbody_z[4];     // force_alloc 分到每腿的 fz (body 系, N)
    float   confidence[4];  // 接触置信度
    float   body_tgt_z[4];  // 命令足端 z (body 系, m)
    float   foot_meas_z[4]; // 实测足端 z (body 系, m)

    float   max_tau_ff;     // 4 腿 τ_ff 绝对值最大 (Nm)

    // ---- 标志位 ----
    uint8_t n_contacts;     // est.n_active_contacts，0 表示换相估计盲区
    uint8_t valid;          // est.valid
    uint8_t phase;          // 0=startup 1=trot 2=stand-hold 3=stand-mode
    uint8_t contact_bits;   // is_contact，bit0=FL bit1=FR bit2=BL bit3=BR

    // [v2] 4 路 CAN bus 实时状态（顺序 FL FR BL BR），用于诊断 bus 降级时序
    //   0=ACTIVE 1=WARNING 2=PASSIVE 3=BUS-OFF 4=DOWN 5=UNKNOWN
    uint8_t bus_state[4];

    // [v3] 4 腿 foot_body_vel 模 (m/s)，用于诊断 F2 误触发（阈值 > 0.6 m/s）。
    //   记录 stand-hold 期每帧值可还原降级前各腿超阈的持续帧数。
    float foot_body_vel[4];     // 顺序 FL FR BL BR

    // [v4] 4 腿 × 3 电机 的 τ_ff (Nm)。腿序 FL FR BL BR; 每腿内 (τ3, τ1, τ2)，与 raw_cmd 同序。
    //   保留完整 12 值签名用于分析单腿三电机扭矩分布。
    float tau_leg[4][3];

    // [v5] 4 腿 × 3 电机 的实测力矩 (Nm)，来自电机私有协议回报 s.tor。腿序/关节序与 tau_leg 同
    //   ((τ3,τ1,τ2))。与 tau_leg（指令 τ_ff）配对可对比指令 vs 实发，实发封顶值即电机真实上限。
    float tau_meas[4][3];
};
static_assert(sizeof(TelemRecord) == 256, "TelemRecord 必须 256B(v5)，改字段要同步 parse_telem.py / decompose_pitch.py");

// 文件头：magic + 版本 + 单条记录大小，供解析器校验
struct TelemHeader {
    char     magic[4];      // "MTTL"
    uint32_t version;       // 当前 5（v5: 加入 tau_meas）
    uint32_t record_size;   // sizeof(TelemRecord)，供解析器按大小判断版本
    uint32_t rate_hz_x100;  // 控制频率 ×100，解析时除以 100 得 Hz
    double   t0_unix;       // 起始 UNIX 时间，用于对齐墙钟
    uint32_t _pad[2];
};
static_assert(sizeof(TelemHeader) == 32, "TelemHeader 必须 32B");

// ============================================================================
// 记录器（header-only）
// ============================================================================
class TelemetryLogger {
public:
    // capacity_pow2 必须是 2 的幂；65536 × 256B ≈ 16 MB 缓冲，足够掩盖落盘抖动。
    explicit TelemetryLogger(const std::string& path,
                             double rate_hz = 500.0,
                             size_t capacity_pow2 = (1u << 16))
        : cap_(capacity_pow2), mask_(capacity_pow2 - 1), buf_(capacity_pow2) {

        fp_ = std::fopen(path.c_str(), "wb");
        if (!fp_) return;
        // 1MB 全缓冲，减少 write syscall 次数
        std::setvbuf(fp_, nullptr, _IOFBF, 1u << 20);

        TelemHeader h{};
        std::memcpy(h.magic, "MTTL", 4);
        h.version     = 5;  // v5: 加入 tau_meas[4][3]（逐电机实测力矩），record 208→256B；解析器按 record_size 判版本
        h.record_size = sizeof(TelemRecord);
        h.rate_hz_x100 = static_cast<uint32_t>(rate_hz * 100.0 + 0.5);
        h.t0_unix     = static_cast<double>(std::time(nullptr));
        std::fwrite(&h, sizeof(h), 1, fp_);

        running_.store(true);
        writer_ = std::thread([this] { writer_loop(); });
    }

    ~TelemetryLogger() { stop(); }

    bool ok() const { return fp_ != nullptr; }

    // 控制线程每帧调；非阻塞，缓冲满则丢并计数。
    inline void push(const TelemRecord& r) {
        const uint64_t h = head_.load(std::memory_order_relaxed);
        const uint64_t t = tail_.load(std::memory_order_acquire);
        if (h - t >= cap_) {                      // 满 → 丢，绝不阻塞 RT
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        buf_[h & mask_] = r;
        head_.store(h + 1, std::memory_order_release);
    }

    // 停 writer：置 flag → join → flush → close。幂等。
    void stop() {
        if (!running_.exchange(false)) {
            if (writer_.joinable()) writer_.join();
            return;
        }
        if (writer_.joinable()) writer_.join();
        if (fp_) { std::fflush(fp_); std::fclose(fp_); fp_ = nullptr; }
    }

    uint64_t written() const { return written_.load(std::memory_order_relaxed); }
    uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

private:
    void writer_loop() {
        // running_ 为真或缓冲中仍有未落盘记录时继续循环
        while (running_.load(std::memory_order_relaxed) ||
               head_.load(std::memory_order_acquire) != tail_.load(std::memory_order_relaxed)) {

            const uint64_t h = head_.load(std::memory_order_acquire);
            uint64_t t       = tail_.load(std::memory_order_relaxed);
            if (t == h) {                          // 缓冲空，sleep 2ms 后重试
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            // 批量写：分段处理环形回绕，每段连续 fwrite
            while (t < h) {
                const size_t idx       = static_cast<size_t>(t & mask_);
                const uint64_t to_end   = cap_ - idx;            // 到缓冲末尾的连续槽数
                const uint64_t avail     = h - t;
                const size_t n          = static_cast<size_t>(avail < to_end ? avail : to_end);
                if (fp_) std::fwrite(&buf_[idx], sizeof(TelemRecord), n, fp_);
                written_.fetch_add(n, std::memory_order_relaxed);
                t += n;
            }
            tail_.store(t, std::memory_order_release);
        }
        if (fp_) std::fflush(fp_);
    }

    std::FILE*  fp_ = nullptr;
    const size_t cap_;
    const size_t mask_;
    std::vector<TelemRecord> buf_;

    std::atomic<uint64_t> head_{0};     // 仅生产者写
    std::atomic<uint64_t> tail_{0};     // 仅消费者写
    std::atomic<uint64_t> written_{0};
    std::atomic<uint64_t> dropped_{0};
    std::atomic<bool>     running_{false};
    std::thread           writer_;
};

}  // namespace telem
}  // namespace parallel
