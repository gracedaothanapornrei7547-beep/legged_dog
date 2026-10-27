#!/usr/bin/env python3
"""硬编码路径点导航 + 过缝隙 cmd_vel 直行.

用法:
  ros2 run nav_bringup waypoint_gap_navigator

依赖: Nav2 已启动 (AMCL + costmap + planner + controller).

启动时自动发布 /initialpose 给 AMCL, 等协方差收敛后开始走路径点.
"""

import math
import time
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw_deg: float
    is_gap_entry: bool = False   # 到达后切 cmd_vel 直行
    gap_distance: float = 0.0     # 直行距离 (m), 仅 is_gap_entry=True 时有效


# ═══════════════════════════════════════════════════════════════
# ★ 起始位姿 + 路径点 (建图后根据实际场地标定修改)
# ═══════════════════════════════════════════════════════════════
START_X = 0.0
START_Y = 0.0
START_YAW_DEG = 0.0

WAYPOINTS = [
    # 示例: 4m×6m 围墙空间, 缝隙在 (1.5, 2.5)→(1.5, 3.1), 宽 600mm
    Waypoint("start",       0.5, 0.5, 0.0),
    Waypoint("pre_gap",     1.5, 2.0, 90.0),
    Waypoint("gap_entry",   1.5, 2.5, 90.0,
             is_gap_entry=True, gap_distance=0.6),
    Waypoint("post_gap",    1.5, 3.5, 90.0),
    Waypoint("goal",        3.0, 5.0, 0.0),
]

# 容差
XY_TOLERANCE = 0.05
YAW_TOLERANCE = 0.05
GAP_XY_TOLERANCE = 0.03

# cmd_vel 直行
GAP_LINEAR_SPEED = 0.15
GAP_CMD_FREQ = 50.0

# AMCL 收敛
AMCL_COV_THRESHOLD = 0.5
AMCL_WAIT_TIMEOUT = 30.0


class WaypointGapNavigator(Node):
    def __init__(self):
        super().__init__("waypoint_gap_navigator")

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)

        self._goal_done = False
        self._amcl_converged = False
        self._amcl_cov = float("inf")

        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, 10)

        self.get_logger().info("路径点导航器已就绪, 等待 AMCL 收敛 ...")

    # ── AMCL 收敛监听 ──────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        pos_cov = max(cov[0], cov[7])
        self._amcl_cov = pos_cov
        if pos_cov < AMCL_COV_THRESHOLD:
            self._amcl_converged = True

    # ── 初始位姿 ───────────────────────────────────────────

    def set_initial_pose(self):
        yaw = math.radians(START_YAW_DEG)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = START_X
        msg.pose.pose.position.y = START_Y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance = [
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.0685,
        ]
        self._init_pose_pub.publish(msg)
        self.get_logger().info(
            f"已发布 initialpose: ({START_X:.2f}, {START_Y:.2f}, {START_YAW_DEG:.0f}°)")

    def wait_for_amcl(self, timeout=AMCL_WAIT_TIMEOUT):
        self.get_logger().info("等待 AMCL 收敛 ...")
        t0 = time.time()
        while rclpy.ok() and not self._amcl_converged:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t0 > timeout:
                self.get_logger().warn(
                    f"AMCL 收敛超时 ({timeout}s), 当前协方差={self._amcl_cov:.3f}, 继续")
                return
        self.get_logger().info(
            f"AMCL 已收敛 (协方差={self._amcl_cov:.3f}, 耗时 {time.time()-t0:.1f}s)")

    # ── Nav2 导航 ──────────────────────────────────────────

    def _nav_feedback(self, feedback_msg):
        pass

    def _nav_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 拒绝导航目标!")
            self._goal_done = True
            return
        self.get_logger().info("Nav2 已接受目标")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 到达目标")
        else:
            self.get_logger().warn(f"Nav2 结束 (状态码={status})")
        self._goal_done = True

    def nav_to(self, x: float, y: float, yaw_deg: float, tolerance: float = XY_TOLERANCE):
        yaw = math.radians(yaw_deg)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(f"→ ({x:.2f}, {y:.2f}, {yaw_deg:.0f}°) 容差={tolerance:.3f}")

        self._goal_done = False
        send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback)
        send_future.add_done_callback(self._nav_response)

        while rclpy.ok() and not self._goal_done:
            rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.2)

    # ── cmd_vel 直行 ───────────────────────────────────────

    def straight_line(self, distance: float, speed: float = GAP_LINEAR_SPEED):
        duration = distance / speed
        rate = self.create_rate(GAP_CMD_FREQ)
        t0 = time.time()
        twist = Twist()
        twist.linear.x = speed

        self.get_logger().info(f"⚡ 闭眼直行 {distance:.2f}m @{speed:.2f}m/s (~{duration:.1f}s)")

        while rclpy.ok() and (time.time() - t0) < duration:
            self._cmd_pub.publish(twist)
            rate.sleep()

        self._cmd_pub.publish(Twist())
        time.sleep(0.3)
        self.get_logger().info("⚡ 直行完成")

    # ── 主循环 ────────────────────────────────────────────

    def run(self):
        # ① 发布初始位姿
        self.set_initial_pose()
        # ② 等 AMCL 收敛
        self.wait_for_amcl()

        self.get_logger().info(
            f"开始 {len(WAYPOINTS)} 个路径点, "
            f"{sum(1 for w in WAYPOINTS if w.is_gap_entry)} 段缝隙")

        for i, wp in enumerate(WAYPOINTS):
            self.get_logger().info(f"[{i+1}/{len(WAYPOINTS)}] {wp.name}")
            tol = GAP_XY_TOLERANCE if wp.is_gap_entry else XY_TOLERANCE
            self.nav_to(wp.x, wp.y, wp.yaw_deg, tolerance=tol)
            if wp.is_gap_entry and wp.gap_distance > 0:
                self.straight_line(wp.gap_distance)

        self.get_logger().info("✅ 全部完成")


def main():
    rclpy.init()
    node = WaypointGapNavigator()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
