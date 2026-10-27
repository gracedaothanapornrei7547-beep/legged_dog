// sbus_teleop.cpp —— ROS2 遥控 teleop 节点: 读 SBUS 串口 → 摇杆映射 Vx/Vy/ω → 发布 /cmd_vel。
// =============================================================================
//   ★站 nav 的位: 它发 /cmd_vel(Twist), 跟真 Point-LIO/nav2 一样。手动用遥控器模拟速度发布。
//   在【狗上】跑(SBUS 接收机接狗串口)。50Hz 发布(回中也发零, 保 shm 看门狗新鲜)。
//   ★失帧/失控(遥控器没发/关机/出界) → status≠OK 或不 fresh → 发零速 → 狗站立(failsafe)。
//
//   映射(可 launch 参数覆盖): ch2 前后→Vx, ch3 侧移→Vy, ch0 自转→ω;
//     满杆 × max_v* ; |归一化|<deadzone 视为 0。
//
//   ★ch5 使能开关(ch5≈1000=遥控有效, ch5≈2000=遥控禁用):
//     禁用时 完全不发 /cmd_vel → 狗端 shm 看门狗 0.3s 过期 → 自动站立。
//     SBUS 物理不在时同样不发。操作纪律: 比赛前拔 SBUS 或确认 ch5=2000。
// =============================================================================

#include <cmath>
#include <ctime>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "motor_test/driver/sbus_reader.hpp"   // 复活自 legged_vmc: SbusReader/FrameSnapshot/find_sbus_port

using namespace std::chrono_literals;
namespace sbus = motor_test_cpp::sbus;

static double mono_now_s() {
    timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_nsec) * 1e-9;
}

class SbusTeleop : public rclcpp::Node {
public:
    SbusTeleop() : Node("sbus_teleop") {
        max_vx_    = declare_parameter<double>("max_vx", 0.30);     // m/s 满杆前后
        max_vy_    = declare_parameter<double>("max_vy", 0.15);     // m/s 满杆侧移
        max_omega_ = declare_parameter<double>("max_omega", 1.00);  // rad/s 满杆自转
        ch_fwd_   = declare_parameter<int>("ch_forward", 2);  sign_fwd_  = declare_parameter<double>("sign_forward",  1.0);
        ch_lat_   = declare_parameter<int>("ch_lateral", 3);  sign_lat_  = declare_parameter<double>("sign_lateral", -1.0);
        ch_turn_  = declare_parameter<int>("ch_turn",    0);  sign_turn_ = declare_parameter<double>("sign_turn",     1.0);
        deadzone_ = declare_parameter<double>("deadzone", 0.08);
        timeout_  = declare_parameter<double>("sbus_timeout", 0.30);
        // ★摇杆行程标定: 中心(992)→满杆 的通道差。归一化 = (channel-992)/stick_half_range, 钳[-1,1]。
        //   遥控器满杆通常到不了 SBUS 电气满量程(1811, 即 819), 用实测满杆值才能让"满杆=max_v"。
        //   先用调试打印看你满杆 channel, 再把这个设成 (满杆channel - 992)。
        stick_half_range_   = declare_parameter<double>("stick_half_range", 511.0);
        debug_channels_     = declare_parameter<bool>("debug_channels", false);
        enable_channel_     = declare_parameter<int>("enable_channel", 5);     // 通道5 (0~9)
        enable_threshold_   = declare_parameter<int>("enable_threshold", 1500); // <1500=遥控有效, >=1500=遥控禁用
        std::string port    = declare_parameter<std::string>("port", "auto");
        bool invert         = declare_parameter<bool>("invert", false);
        std::string smode_s = declare_parameter<std::string>("serial_mode", "N1");
        sbus::SerialMode smode = (smode_s == "E2") ? sbus::SerialMode::E2 : sbus::SerialMode::N1;

        if (port == "auto") {
            bool inv = false;
            port = sbus::find_sbus_port(&inv, 400, smode);
            invert = inv;
            if (port.empty()) {
                RCLCPP_ERROR(get_logger(),
                    "★找不到 SBUS 端口(接收机没接/没通电?) → 不发 /cmd_vel(狗端看门狗站立)");
                // reader_ 保持 nullptr, tick() 里直接 return 不发消息
            }
        }
        if (!port.empty()) {
            reader_ = std::make_unique<sbus::SbusReader>(port, invert, 3.0, smode);
            if (!reader_->start()) RCLCPP_ERROR(get_logger(), "★SBUS start 失败 → snapshot 返回旧/零值帧, 走失帧逻辑发零速(狗站立)");
            else RCLCPP_INFO(get_logger(), "SBUS 接收启动: port=%s invert=%d ✓", port.c_str(), invert ? 1 : 0);
        }

        pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = create_wall_timer(20ms, [this] { tick(); });   // 50Hz
        RCLCPP_INFO(get_logger(),
            "sbus_teleop: SBUS → /cmd_vel  max v=(%.2f,%.2f) ω=%.2f  ch%d< %d=使能 >=%d=禁用(不发)"
            "  (失帧→零速→站立)",
            max_vx_, max_vy_, max_omega_,
            enable_channel_, enable_threshold_, enable_threshold_);
    }

private:
    void tick() {
        geometry_msgs::msg::Twist m;   // 默认全 0 = 站立

        // ── 情况 1: SBUS 物理不在 → 完全不发 /cmd_vel(狗端看门狗站立) ──
        if (!reader_) {
            return;  // 不发任何消息
        }

        const sbus::FrameSnapshot f = reader_->snapshot();

        // ── 情况 2: ch5 <阈值 → 使能, ch5 >= 阈值 → 禁用 ──
        const int  ch_en    = f.channels[enable_channel_];
        const bool disabled = (ch_en >= enable_threshold_);

        if (disabled) {
            // ch5>=阈值(≈1800) → 遥控禁用, 完全不发 /cmd_vel
            // (狗端 shm 看门狗 0.3s 过期 → 收腿站立)
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "ch%d=%d >= %d → teleop DISABLED(不发 /cmd_vel)",
                enable_channel_, ch_en, enable_threshold_);
            return;  // 不发任何消息
        }

        // ── 情况 3: ch5 < 阈值(≈200) → 遥控有效, 正常发速度 ──
        const bool ok = f.fresh(mono_now_s(), timeout_) && f.status == sbus::FailsafeStatus::OK;
        if (ok) {
            // ★用实测摇杆行程归一化: (channel-中心)/stick_half_range, 钳[-1,1]。满杆→±1→±max_v。
            auto norm = [this, &f](int ch) {
                const double v = (static_cast<double>(f.channels[ch]) - sbus::SBUS_CENTER) / stick_half_range_;
                return std::clamp(v, -1.0, 1.0);
            };
            auto dz = [this](double v) { return (std::fabs(v) < deadzone_) ? 0.0 : v; };
            m.linear.x  = dz(sign_fwd_  * norm(ch_fwd_))  * max_vx_;
            m.linear.y  = dz(sign_lat_  * norm(ch_lat_))  * max_vy_;
            m.angular.z = dz(sign_turn_ * norm(ch_turn_)) * max_omega_;
        }
        // else: 遥控使能但失帧/失控 → m 保持全 0 → 狗站立(和原来一样)

        pub_->publish(m);

        // 调试: 节流打印原始通道 + 速度(给 stick_half_range 定标用)
        if (debug_channels_ && (++dbg_n_ % 25 == 0)) {   // ~2Hz
            RCLCPP_INFO(get_logger(),
                "ch[en%d fwd%d lat%d turn%d]=(%d,%d,%d,%d) ok=%d → Vx=%.3f Vy=%.3f ω=%.3f",
                enable_channel_, ch_fwd_, ch_lat_, ch_turn_,
                ch_en, f.channels[ch_fwd_], f.channels[ch_lat_], f.channels[ch_turn_],
                ok ? 1 : 0, m.linear.x, m.linear.y, m.angular.z);
        }
    }

    std::unique_ptr<sbus::SbusReader> reader_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double max_vx_, max_vy_, max_omega_, deadzone_, timeout_;
    double stick_half_range_ = 511.0;
    bool   debug_channels_ = false;
    uint64_t dbg_n_ = 0;
    int    ch_fwd_, ch_lat_, ch_turn_;
    double sign_fwd_, sign_lat_, sign_turn_;
    int enable_channel_   = 5;    // 通道5 (0~9)
    int enable_threshold_ = 1500; // <1500=使能, >=1500=禁用
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SbusTeleop>());
    rclcpp::shutdown();
    return 0;
}
