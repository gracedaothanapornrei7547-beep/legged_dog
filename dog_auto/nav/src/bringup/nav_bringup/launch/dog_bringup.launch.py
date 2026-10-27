#!/usr/bin/env python3
"""狗底盘独立启动: cmd_bridge + nav_quad + shm_monitor.

不含 SBUS 遥控, 不含雷达/导航。用于:
  - 建图时: 另开终端跑 dog_mapping.launch.py (雷达)
  - 导航时: 另开终端跑 dog_navigation.launch.py (Nav2)
  - 结束后 Ctrl+C 不会影响本终端, 狗保持站立。

用法:
  ros2 launch nav_bringup dog_bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # shm 写端（创建者），必须先于 nav_quad
    cmd_bridge_node = Node(
        package="legged_nav_ros", executable="cmd_bridge", name="cmd_bridge",
        output="screen",
    )

    # nav_quad: 必须从 legged_nav 目录跑 (内部 scripts/ 相对路径)
    nav_quad_cmd = ExecuteProcess(
        cmd=["bash", "-c",
             "cd ~/dog_auto/legged_nav && exec ros2 run legged_nav nav_quad"],
        output="screen",
    )

    # shm 实时监视器
    shm_monitor_cmd = ExecuteProcess(
        cmd=["ros2", "run", "legged_nav", "shm_monitor", "0.30", "50"],
        output="screen",
    )

    return LaunchDescription([
        cmd_bridge_node,
        nav_quad_cmd,
        shm_monitor_cmd,
    ])
