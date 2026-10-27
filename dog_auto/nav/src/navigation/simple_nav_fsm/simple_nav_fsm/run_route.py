#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_route —— 按顺序调用 navigate_to_pose Action，串联跑完整路线。

用法:
  ros2 run simple_nav_fsm run_route <wp1> <wp2> ...     # 直接给路点名序列
  ros2 run simple_nav_fsm run_route demo                 # 读 waypoints.yaml 的 routes 段

waypoints.yaml 格式:
  waypoints:
    pickup_a: {x: 1.0, y: 0.5, yaw: 90.0, final_heading: true}
    corner_1: {x: 2.0, y: 1.0, final_heading: false}   # 中途点不做终点朝向
  routes:
    demo: [pickup_a, corner_1, place_a, home]
"""
import argparse
import math
import sys
import os
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import yaml

WAYPOINTS_FILE = Path(os.path.expanduser("~")) / ".simple_nav_fsm_waypoints.yaml"


class RouteRunner(Node):
    def __init__(self, waypoints):
        super().__init__('route_runner')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._waypoints = waypoints
        self._current_goal = None
        self._result = None

    def send_goal(self, wp):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action 不可用，'
                                   'simple_nav_fsm 是否已启动？')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])

        yaw_deg = float(wp.get('yaw', 0.0))
        yaw_rad = math.radians(yaw_deg)
        goal.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        if not wp.get('final_heading', True):
            goal.behavior_tree = 'xy_only'

        self.get_logger().info(
            f'发送 goal: x={wp["x"]:.3f} y={wp["y"]:.3f} yaw={yaw_deg:.1f}° '
            f'终点朝向={"不要" if goal.behavior_tree == "xy_only" else "要"}')

        self._result = None
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)
        return True

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal 被拒绝')
            self._result = 'rejected'
            return
        self.get_logger().info('Goal 已接受，执行中...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:   # 修正: 原为 status==0(UNKNOWN)
            self.get_logger().info('Goal 成功')
        else:
            self.get_logger().error(f'Goal 失败，状态码={status}')
        self._result = status


def load_route(route_spec):
    """解析命令行为 route 名或路点名序列。"""
    if not WAYPOINTS_FILE.exists():
        print(f'找不到 {WAYPOINTS_FILE}，请先用 record_waypoint 打点', file=sys.stderr)
        return None

    data = yaml.safe_load(open(WAYPOINTS_FILE)) or {}
    wp_db = data.get('waypoints', {})
    routes_db = data.get('routes', {})

    # 若只有一个参数且匹配 route 名 -> 展开
    if len(route_spec) == 1 and route_spec[0] in routes_db:
        names = routes_db[route_spec[0]]
    else:
        names = route_spec

    waypoints = []
    for name in names:
        if name not in wp_db:
            print(f'路点 "{name}" 不存在于 {WAYPOINTS_FILE}', file=sys.stderr)
            return None
        w = dict(wp_db[name])
        w['_name'] = name
        waypoints.append(w)
    return waypoints


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='跑路线：依次发送 navigate_to_pose goal')
    parser.add_argument('names', nargs='+', help='路点名序列 或 routes 中定义的路线名')
    from rclpy.utilities import remove_ros_args
    argv = remove_ros_args(args=sys.argv)   # ★只剥离 ROS 参数, 保留 names
    parsed, unknown = parser.parse_known_args(argv[1:])

    waypoints = load_route(parsed.names)
    if waypoints is None:
        rclpy.shutdown()
        return 1

    print(f'路线: {" -> ".join(w["_name"] for w in waypoints)}')
    node = RouteRunner(waypoints)

    for i, wp in enumerate(waypoints):
        print(f'\n[{i+1}/{len(waypoints)}] {wp["_name"]}...')
        if not node.send_goal(wp):
            node.destroy_node()
            rclpy.shutdown()
            return 1

        # 等待完成
        while rclpy.ok() and node._result is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node._result != GoalStatus.STATUS_SUCCEEDED:
            print(f'✗ 路点 {wp["_name"]} 失败，路线中止', file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 1

    print('\n✓ 路线完成！')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
