#!/usr/bin/env python3
"""
mission_director 启动文件 —— 只起决策机自己的节点。

前置（另开终端按序启动，本 launch 不代起）：
  1. 定位栈（建图或重定位）:
     ros2 launch pb2025_nav_bringup navigation_launch.py map_name:=arena run_nav2:=False
     (起 point_lio + small_gicp_relocalization + loam_interface + robot_state_publisher)
  2. FSM 导航状态机:
     ros2 launch simple_nav_fsm simple_nav.launch.py
     (接管 /navigate_to_pose action，发布 /cmd_vel_raw → guard → /cmd_vel → 底盘)

用法：
  ros2 launch mission_director mission.launch.py                 # 实机
  ros2 launch mission_director mission.launch.py dry_run:=true   # 本机仿真（不开串口+导航仅仿真）
  ros2 launch mission_director mission.launch.py auto_start:=false  # 手动触发（调 ~/start 服务）
  ros2 launch mission_director mission.launch.py params_file:=/path/to/my.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    dry_run = LaunchConfiguration("dry_run")
    auto_start = LaunchConfiguration("auto_start")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("mission_director"),
                "config", "mission.yaml"]),
            description="参数文件路径"),
        DeclareLaunchArgument(
            "dry_run", default_value="false",
            description="true=不开串口+导航仿真(本机跑流程)"),
        DeclareLaunchArgument(
            "auto_start", default_value="true",
            description="false=等 ~/start 服务再开跑"),
        Node(
            package="mission_director",
            executable="mission_director_node",
            name="mission_director",
            output="screen",
            emulate_tty=True,
            parameters=[params_file, {
                "dry_run": dry_run,
                "auto_start": auto_start,
            }],
        ),
    ])
