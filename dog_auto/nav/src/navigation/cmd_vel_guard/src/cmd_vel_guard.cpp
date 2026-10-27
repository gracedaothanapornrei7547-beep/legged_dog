// cmd_vel_guard.cpp —— nav 侧「安全带」节点
// =============================================================================
//   位置: fake_vel_transform ─ /cmd_vel_raw ─▶ [cmd_vel_guard] ─▶ /cmd_vel ─▶ cmd_bridge(狗接口)
//
//   本版职责(解决 A + B):
//     A 速度硬钳(代码级, 不依赖任何 params): vx/vy/ω 越界即钳, NaN/Inf → 0
//     B 固定频率重发 + 上游超时兜底: 定时器以 publish_rate_hz 发「最后一条(已钳)命令」,
//       与上游抖动/高频 odom 彻底解耦(shm 永远新鲜); 上游 > input_timeout_s 无新命令
//       (到点/丢路/节点崩)→ 发零速 → 狗站立。
//
//   设计: onTick() 做成「分级处理 + 预留扩展位」, 后续可无痛加:
//     · 平移优先仲裁(问题 D, 已埋参数开关, 默认关)
//     · e-stop 订阅 / 斜率(加速度)限 / gait 迟滞 / 多源仲裁(见 onTick 内 TODO)
//   不动 cmd_bridge、不动狗、不动 Point-LIO。
// =============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class CmdVelGuard : public rclcpp::Node
{
public:
  CmdVelGuard() : rclcpp::Node("cmd_vel_guard")
  {
    input_topic_   = declare_parameter<std::string>("input_topic", "cmd_vel_raw");
    output_topic_  = declare_parameter<std::string>("output_topic", "cmd_vel");
    vx_max_        = declare_parameter<double>("vx_max", 0.30);   // 狗安全包络(前后)
    vy_max_        = declare_parameter<double>("vy_max", 0.15);   // 横向弱轴, 更小
    w_max_         = declare_parameter<double>("w_max", 0.50);    // 自转
    rate_hz_       = declare_parameter<double>("publish_rate_hz", 50.0);
    input_timeout_ = declare_parameter<double>("input_timeout_s", 0.30);  // 上游超时→归零
    // ── 扩展位(默认关): 平移优先仲裁(平移超阈值时禁自转; 待问题 D 决策) ──
    translate_priority_ = declare_parameter<bool>("translate_priority", false);
    translate_thresh_   = declare_parameter<double>("translate_speed_threshold", 0.05);

    if (rate_hz_ <= 0.0) rate_hz_ = 50.0;

    pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, rclcpp::QoS(1));
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&CmdVelGuard::onCmd, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelGuard::onTick, this));

    RCLCPP_INFO(
      get_logger(),
      "cmd_vel_guard: %s → 钳[vx±%.2f vy±%.2f w±%.2f] @%.0fHz → %s "
      "(上游超时 %.2fs 归零; 平移优先=%s)",
      input_topic_.c_str(), vx_max_, vy_max_, w_max_, rate_hz_, output_topic_.c_str(),
      input_timeout_, translate_priority_ ? "on" : "off");
  }

private:
  static double clampFinite(double v, double lim)
  {
    if (!std::isfinite(v)) return 0.0;               // NaN/Inf → 0 (安全)
    return std::clamp(v, -lim, lim);
  }

  void onCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    last_ = *msg;
    last_rx_ = now();
    have_cmd_ = true;
  }

  void onTick()
  {
    geometry_msgs::msg::Twist out;                   // 默认全 0 = 站立
    bool   fresh  = false;
    double age_ms = -1.0;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      // have_cmd_ 短路在前: 从未收到过命令时不做时间相减(避免时钟类型不一致)。
      const double age = have_cmd_ ? (now() - last_rx_).seconds() : 1e9;
      fresh  = have_cmd_ && age <= input_timeout_;
      age_ms = have_cmd_ ? age * 1e3 : -1.0;
      if (fresh) {
        // ── A: 硬钳(代码级保证, 不依赖上游 params) ──
        out.linear.x  = clampFinite(last_.linear.x,  vx_max_);
        out.linear.y  = clampFinite(last_.linear.y,  vy_max_);
        out.angular.z = clampFinite(last_.angular.z, w_max_);

        // ── 扩展位: 平移优先仲裁(默认关) ──
        if (translate_priority_ &&
            std::hypot(out.linear.x, out.linear.y) > translate_thresh_) {
          out.angular.z = 0.0;
        }
        // ── 扩展位(后续): e-stop 订阅 / 斜率(加速度)限 / gait 迟滞 / 多源仲裁 在此加 ──
      }
      // else: 上游超时(到点/丢路/崩) → out 保持全 0 → 狗站立
    }
    // B: 无论如何都以固定频率发 → cmd_bridge/shm 永远新鲜, 与上游抖动解耦
    pub_->publish(out);
    ++pub_count_;

    // 心跳(每 2s 一行): 让人看得见它活着 + 在发什么。静默不代表没干活。
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "活着@%.0fHz 已发%lu帧 | 上游 %s(age=%.0fms) | 出 vx=%+.3f vy=%+.3f w=%+.3f",
      rate_hz_, pub_count_, fresh ? "FRESH" : "STALE→归零", age_ms,
      out.linear.x, out.linear.y, out.angular.z);
  }

  std::string input_topic_, output_topic_;
  double vx_max_, vy_max_, w_max_, rate_hz_, input_timeout_, translate_thresh_;
  bool translate_priority_ = false;

  std::mutex mtx_;
  geometry_msgs::msg::Twist last_;
  rclcpp::Time last_rx_;
  bool have_cmd_ = false;
  unsigned long pub_count_ = 0;   // 已发帧计数(心跳日志用)

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelGuard>());
  rclcpp::shutdown();
  return 0;
}
