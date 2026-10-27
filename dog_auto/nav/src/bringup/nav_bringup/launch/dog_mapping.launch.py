#!/usr/bin/env python3
"""建图: MID360驱动 + 狗体长方体过滤(CustomMsg) + point_lio.

前置: 另开终端先起狗底盘.
用法:
  source ~/dog_auto/setup-env.sh
  ~/dog_auto/desnap-ros.sh ros2 launch nav_bringup dog_mapping.launch.py rviz:=True
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_FILTER_SCRIPT = os.path.join(
    get_package_share_directory("sensor_scan_generation"),
    "..", "..", "lib", "sensor_scan_generation", "dog_body_filter.py")


def generate_launch_description():
    rviz = LaunchConfiguration("rviz", default="False")

    livox_launch = PathJoinSubstitution([
        FindPackageShare("livox_ros_driver2"), "launch",
        "msg_MID360_launch.py",
    ])

    # 狗体长方体过滤: /livox/lidar(CustomMsg) → /livox/lidar_filtered(CustomMsg)
    body_filter_cmd = ExecuteProcess(
        cmd=["python3", "-u", _FILTER_SCRIPT],
        output="screen", name="dog_body_filter",
    )

    # point_lio: 订阅过滤后的 CustomMsg
    point_lio_cfg = PathJoinSubstitution([
        FindPackageShare("point_lio"), "config", "mid360.yaml",
    ])
    point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        parameters=[point_lio_cfg, {
            "common.lid_topic": "livox/lidar_filtered",
        }],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    rviz_cfg = PathJoinSubstitution([
        FindPackageShare("point_lio"), "rviz_cfg", "loam_livox.rviz",
    ])
    rviz_node = Node(
        condition=IfCondition(rviz),
        package="rviz2", executable="rviz2", name="rviz",
        arguments=["-d", rviz_cfg],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="False"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(livox_launch)),
        body_filter_cmd,
        point_lio_node,
        rviz_node,
    ])
