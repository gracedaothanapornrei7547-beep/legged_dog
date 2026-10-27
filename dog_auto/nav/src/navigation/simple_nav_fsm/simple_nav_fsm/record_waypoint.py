#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_waypoint —— 采样1秒定位取均值，写入 waypoints.yaml。

用法:
  ros2 run simple_nav_fsm record_waypoint <name> [--xy-only] [--force]

  --xy-only : 不记录朝向（final_heading: false）
  --force   : 方差超标仍强制写入
"""
import argparse
import math
import sys
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
import yaml

WAYPOINTS_FILE = Path(os.path.expanduser("~")) / ".simple_nav_fsm_waypoints.yaml"


def yaw_of(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder', parameter_overrides=[])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.poses = []

    def sample(self):
        try:
            tfm = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
        except Exception:
            return False
        t = tfm.transform.translation
        yaw = yaw_of(tfm.transform.rotation)
        self.poses.append((t.x, t.y, yaw))
        return True

    def wait_for_tf(self, timeout=8.0):
        """等 map->base_footprint 出现(TF buffer 预热)。返回 True/False。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                if self.tf_buffer.can_transform("map", "base_footprint", Time()):
                    return True
            except Exception:
                pass
        return False

    def frames_dump(self):
        try:
            return self.tf_buffer.all_frames_as_string()
        except Exception:
            return "(无法列出 frames)"

    def stats(self):
        n = len(self.poses)
        if n < 2:
            return None
        xs = [p[0] for p in self.poses]
        ys = [p[1] for p in self.poses]
        yaws = [p[2] for p in self.poses]
        mx, my = sum(xs) / n, sum(ys) / n
        myaw = sum(yaws) / n
        vx = sum((x - mx) ** 2 for x in xs) / n
        vy = sum((y - my) ** 2 for y in ys) / n
        return {'x': mx, 'y': my, 'yaw': myaw, 'var_x': vx, 'var_y': vy, 'samples': n}


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='记录导航路点')
    parser.add_argument('name', help='路点名')
    parser.add_argument('--xy-only', action='store_true', help='不记录朝向')
    parser.add_argument('--force', action='store_true', help='方差超标仍写入')
    from rclpy.utilities import remove_ros_args
    argv = remove_ros_args(args=sys.argv)   # ★只剥离 ROS 参数(--ros-args...), 保留用户的 name/--xy-only/--force

    parsed, unknown = parser.parse_known_args(argv[1:])

    node = WaypointRecorder()
    print('等待定位 TF (map -> base_footprint) ...', flush=True)
    if not node.wait_for_tf(timeout=8.0):
        print('✗ 8s 内拿不到 map->base_footprint。定位没起全或没收敛。', file=sys.stderr)
        print('  排查:', file=sys.stderr)
        print('   1) 终端2 定位在跑吗: dog_navigation.launch.py map_name:=... run_nav2:=False', file=sys.stderr)
        print('   2) 另开终端: ros2 run tf2_ros tf2_echo map base_footprint  能否拿到', file=sys.stderr)
        print('   3) ros2 node list | grep -E "point_lio|small_gicp|loam_interface|robot_state_publisher"', file=sys.stderr)
        print('  当前 TF 树:\n' + node.frames_dump(), file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print('采集中... 保持狗静止 1 秒', flush=True)
    misses = 0
    for _ in range(40):                 # ~1.6s, 容忍偶发丢帧
        rclpy.spin_once(node, timeout_sec=0.04)
        if not node.sample():
            misses += 1
        if len(node.poses) >= 30:
            break
    if len(node.poses) < 10:
        print(f'✗ 采样太少 ({len(node.poses)} 帧, 丢 {misses} 帧), TF 不稳定', file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    s = node.stats()
    node.destroy_node()
    rclpy.shutdown()

    if s is None:
        print('采样不足', file=sys.stderr)
        return 1

    var_max = max(s['var_x'], s['var_y'])
    thresh = 0.001   # 1mm^2
    if var_max > thresh and not parsed.force:
        print(f'方差超标 var_xy={var_max:.6f} > {thresh}，可能是刚挪完/TF 还在跳。'
              f'请停稳后重试，或加 --force 强制写入', file=sys.stderr)
        return 1

    entry = {'x': round(s['x'], 4), 'y': round(s['y'], 4)}
    if not parsed.xy_only:
        entry['yaw'] = round(math.degrees(s['yaw']), 2)
        entry['final_heading'] = True
    else:
        entry['final_heading'] = False

    # 加载已有文件
    data = {}
    if WAYPOINTS_FILE.exists():
        data = yaml.safe_load(open(WAYPOINTS_FILE)) or {}

    if 'waypoints' not in data:
        data['waypoints'] = {}
    data['waypoints'][parsed.name] = entry

    with open(WAYPOINTS_FILE, 'w') as f:
        yaml.safe_dump(data, f, allow_unicode=True, default_flow_style=False)

    print(f'✓ 路点 "{parsed.name}" 已写入 {WAYPOINTS_FILE}')
    print(f'  x={entry["x"]:.4f}  y={entry["y"]:.4f}', end='')
    if 'yaw' in entry:
        print(f'  yaw={entry["yaw"]:.1f}°')
    else:
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
