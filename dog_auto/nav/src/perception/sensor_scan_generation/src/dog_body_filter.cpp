// dog_body_filter.cpp —— 狗身体长方体过滤 (C++ 版, 替代 Python 版, 提速数十倍)
// =============================================================================
//   livox → /livox/lidar(CustomMsg) → dog_body_filter → /livox/lidar_filtered → point_lio
//
//   以 base_footprint 为中心, 滤掉落在狗自身机体长方体内的点。逻辑与原 Python 版一字不差:
//     · 静态挂载 (无 TF 依赖): 点 p(lidar 帧) → base_footprint 帧 = R·p + t
//     · 长方体判据 (base_footprint 帧): 落在 [x/y/z_min, x/y/z_max] 内的点滤掉
//     · 保全 CustomMsg 全部字段 (header/timebase/lidar_id/rsvd + 每个点的所有字段)
//
//   为什么快: Python 版逐点走对象循环 (每帧几万次 Python 调用 → 98% CPU 追不上雷达);
//   C++ 直接对 msg->points 连续内存迭代, 同样逻辑, 微秒级。
//
//   参数 (与 Python 版同名同默认, 可 launch 覆盖):
//     x_min/x_max/y_min/y_max/z_min/z_max, mount_xyz[3], mount_rpy[3],
//     input_topic(/livox/lidar), output_topic(/livox/lidar_filtered)
// =============================================================================
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace sensor_scan_generation
{

using CustomMsg = livox_ros_driver2::msg::CustomMsg;

class DogBodyFilterNode : public rclcpp::Node
{
public:
  explicit DogBodyFilterNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("dog_body_filter", options)
  {
    x_min_ = declare_parameter<double>("x_min", -0.35);
    x_max_ = declare_parameter<double>("x_max", 0.35);
    y_min_ = declare_parameter<double>("y_min", -0.175);
    y_max_ = declare_parameter<double>("y_max", 0.175);
    z_min_ = declare_parameter<double>("z_min", -0.30);
    z_max_ = declare_parameter<double>("z_max", 0.50);

    const std::vector<double> mount_xyz =
      declare_parameter<std::vector<double>>("mount_xyz", {0.13, 0.0, 0.06});
    const std::vector<double> mount_rpy =
      declare_parameter<std::vector<double>>("mount_rpy", {0.0, 0.0, 0.0});
    t_ = {0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < 3u && i < mount_xyz.size(); ++i) {
      t_[i] = mount_xyz[i];
    }
    const double roll = mount_rpy.size() > 0u ? mount_rpy[0] : 0.0;
    const double pitch = mount_rpy.size() > 1u ? mount_rpy[1] : 0.0;
    const double yaw = mount_rpy.size() > 2u ? mount_rpy[2] : 0.0;
    computeR(roll, pitch, yaw);

    const std::string in_topic =
      declare_parameter<std::string>("input_topic", "/livox/lidar");
    const std::string out_topic =
      declare_parameter<std::string>("output_topic", "/livox/lidar_filtered");

    const auto qos = rclcpp::SensorDataQoS();
    pub_ = create_publisher<CustomMsg>(out_topic, qos);
    sub_ = create_subscription<CustomMsg>(
      in_topic, qos,
      std::bind(&DogBodyFilterNode::callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "狗长方体过滤(C++, CustomMsg, 静态挂载, 无TF): "
      "X=[%.3f,%.3f] Y=[%.3f,%.3f] Z=[%.3f,%.3f] t=[%.3f,%.3f,%.3f]  %s -> %s",
      x_min_, x_max_, y_min_, y_max_, z_min_, z_max_,
      t_[0], t_[1], t_[2], in_topic.c_str(), out_topic.c_str());
  }

private:
  // rpy → R = Rz(yaw)·Ry(pitch)·Rx(roll) (与 Python rpy_to_matrix 完全一致)
  void computeR(double roll, double pitch, double yaw)
  {
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    R_[0][0] = cy * cp; R_[0][1] = cy * sp * sr - sy * cr; R_[0][2] = cy * sp * cr + sy * sr;
    R_[1][0] = sy * cp; R_[1][1] = sy * sp * sr + cy * cr; R_[1][2] = sy * sp * cr - cy * sr;
    R_[2][0] = -sp;     R_[2][1] = cp * sr;                R_[2][2] = cp * cr;
  }

  void callback(const CustomMsg::SharedPtr msg)
  {
    const uint32_t n = msg->point_num;
    if (n == 0u || msg->points.empty()) {
      pub_->publish(*msg);   // 空帧原样转发
      return;
    }

    CustomMsg out;
    out.header = msg->header;
    out.timebase = msg->timebase;
    out.lidar_id = msg->lidar_id;   // ★保全 lidar_id / rsvd
    out.rsvd = msg->rsvd;
    out.points.reserve(msg->points.size());

    for (const auto & p : msg->points) {
      const double x = p.x, y = p.y, z = p.z;
      const double bx = R_[0][0] * x + R_[0][1] * y + R_[0][2] * z + t_[0];
      const double by = R_[1][0] * x + R_[1][1] * y + R_[1][2] * z + t_[1];
      const double bz = R_[2][0] * x + R_[2][1] * y + R_[2][2] * z + t_[2];
      const bool inside =
        bx >= x_min_ && bx <= x_max_ &&
        by >= y_min_ && by <= y_max_ &&
        bz >= z_min_ && bz <= z_max_;
      if (!inside) {
        out.points.push_back(p);   // 保留整点(含 offset_time/reflectivity/tag/line)
      }
    }
    out.point_num = static_cast<uint32_t>(out.points.size());
    pub_->publish(out);

    if (!logged_) {
      logged_ = true;
      RCLCPP_INFO(
        get_logger(), "首帧过滤: %u -> %u 点 (滤掉狗身 %u)",
        n, out.point_num, n - out.point_num);
    }
  }

  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
  std::array<double, 3> t_{{0.0, 0.0, 0.0}};
  double R_[3][3]{};
  bool logged_ = false;
  rclcpp::Publisher<CustomMsg>::SharedPtr pub_;
  rclcpp::Subscription<CustomMsg>::SharedPtr sub_;
};

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::DogBodyFilterNode)
