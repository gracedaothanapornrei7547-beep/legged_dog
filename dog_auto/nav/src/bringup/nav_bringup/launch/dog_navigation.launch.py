#!/usr/bin/env python3
"""全自主导航: 狗体过滤 + 箱子过滤 + MID360 + point_lio + AMCL + Nav2.

前置: 另开终端先起狗底盘.
用法:
  终端1: ros2 launch nav_bringup dog_bringup.launch.py
  终端2: ros2 launch nav_bringup dog_navigation.launch.py map_name:=arena

初始位姿: AMCL 通过 set_initial_pose 自动初始化, 坐标在 nav2_params_realcar.yaml 的
  amcl.initial_pose 中配置. waypoint_gap_navigator 启动时会再次发布 /initialpose 并等待收敛.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_SHARE_DIR = get_package_share_directory("sensor_scan_generation")
_LIB_DIR = os.path.join(_SHARE_DIR, "..", "..", "lib", "sensor_scan_generation")
_BODY_FILTER = os.path.join(_LIB_DIR, "dog_body_filter.py")
_BOX_FILTER = os.path.join(_LIB_DIR, "box_filter.py")


def generate_launch_description():
    map_name = LaunchConfiguration("map_name")
    use_rviz = LaunchConfiguration("use_rviz")
    use_composition = LaunchConfiguration("use_composition")
    run_nav2 = LaunchConfiguration("run_nav2")
    slam = LaunchConfiguration("slam")

    nav_bringup_pkg = PathJoinSubstitution([
        FindPackageShare("pb2025_nav_bringup"),
    ])
    nav_launch = PathJoinSubstitution([
        nav_bringup_pkg, "launch",
        "rm_navigation_reality_mapping_launch.py",
    ])

    # ★C++ 版狗体过滤 (替代原 Python 脚本, 提速数十倍, 治 /livox/lidar_filtered 断流)
    body_filter_cmd = Node(
        package="sensor_scan_generation",
        executable="dog_body_filter_node",
        name="dog_body_filter",
        output="screen",
    )

    box_filter_cmd = ExecuteProcess(
        cmd=["python3", "-u", _BOX_FILTER],
        output="screen", name="box_filter",
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_name", default_value="arena"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_composition", default_value="False",
                              description="狗用普通节点模式更稳"),
        DeclareLaunchArgument("run_nav2", default_value="True",
                              description="Nav2 模式=True; FSM 模式设 False(只起定位+基础设施+guard, 不起 Nav2)"),
        DeclareLaunchArgument("slam", default_value="False",
                              description="建图模式=True 使用 SLAM; 定位模式=False 使用 small_gicp"),
        body_filter_cmd,
        box_filter_cmd,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments={
                "slam": slam,
                "world": map_name,
                "use_rviz": use_rviz,
                "use_composition": use_composition,
                "run_nav2": run_nav2,
            }.items(),
        ),
    ])
