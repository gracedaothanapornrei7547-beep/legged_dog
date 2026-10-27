// cmd_bridge.cpp —— ROS2 桥节点: 订阅 /cmd_vel(Twist) → seqlock 写共享内存 /legged_nav_cmd。
// =============================================================================
//   nav(或 sbus_teleop)发布 Vx/Vy/ω(linear.x/y, angular.z); 狗 RT 进程(nav_quad)读 shm。
//   ★狗完全不碰 DDS —— DDS/ROS2 隔离在本节点; 狗只 mmap 只读 shm(进程级隔离)。
//   桥是 shm 的【写端/创建者】(O_CREAT), 故须先于狗启动。
//
//   gait_enabled 由速度大小推导(Twist 不带该位): 任一分量过死区→1(走), 全近零→0(站)。
//     → nav 到点发零速 / teleop 回中 / 失帧发零 → 狗站立。
//   看门狗在狗端(0.3s): 桥停写(节点崩/nav停发)→ write_t_s 变旧 → 狗站立。
// =============================================================================

#include <cmath>
#include <ctime>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "motor_test/task/nav_cmd_shm.hpp"   // legged_nav 共享布局(单一真源)

using motor_test_cpp::task::NavCmdShm;
using motor_test_cpp::task::NAV_CMD_SHM_NAME;

static double mono_now_s() {
    timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_nsec) * 1e-9;
}

class CmdBridge : public rclcpp::Node {
public:
    CmdBridge() : Node("cmd_bridge") {
        // shm 写端: 创建/打开 → ftruncate → mmap 读写。
        fd_ = shm_open(NAV_CMD_SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (fd_ < 0 || ftruncate(fd_, sizeof(NavCmdShm)) != 0) {
            RCLCPP_FATAL(get_logger(), "shm_open/ftruncate '%s' 失败", NAV_CMD_SHM_NAME);
            throw std::runtime_error("shm init failed");
        }
        void* p = mmap(nullptr, sizeof(NavCmdShm), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (p == MAP_FAILED) { RCLCPP_FATAL(get_logger(), "mmap 失败"); throw std::runtime_error("mmap failed"); }
        shm_ = static_cast<NavCmdShm*>(p);
        write_cmd(0, 0, 0, 0);   // 初始化为站立(零速)

        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr m) { on_cmd(*m); });
        RCLCPP_INFO(get_logger(), "cmd_bridge: /cmd_vel → shm %s  ✓ (狗须后于本节点启动)", NAV_CMD_SHM_NAME);
    }
    ~CmdBridge() override {
        if (shm_) { write_cmd(0, 0, 0, 0); munmap(shm_, sizeof(NavCmdShm)); }
        if (fd_ >= 0) close(fd_);
    }

private:
    void on_cmd(const geometry_msgs::msg::Twist& m) {
        const double vx = m.linear.x, vy = m.linear.y, om = m.angular.z;
        const uint32_t ge = (std::fabs(vx) > 0.01 || std::fabs(vy) > 0.01 || std::fabs(om) > 0.02) ? 1u : 0u;
        write_cmd(vx, vy, om, ge);
    }
    void write_cmd(double vx, double vy, double om, uint32_t ge) {
        if (!shm_) return;
        const uint32_t seq = shm_->seq;
        shm_->seq = seq + 1; __sync_synchronize();          // 奇=写中
        shm_->vx = vx; shm_->vy = vy; shm_->omega = om;
        shm_->gait_enabled = ge; shm_->write_t_s = mono_now_s();
        __sync_synchronize(); shm_->seq = seq + 2;           // 偶=稳定
    }

    int        fd_  = -1;
    NavCmdShm* shm_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdBridge>());
    rclcpp::shutdown();
    return 0;
}
