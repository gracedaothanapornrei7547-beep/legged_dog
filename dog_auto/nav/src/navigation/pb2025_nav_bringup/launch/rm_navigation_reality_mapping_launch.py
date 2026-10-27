# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# Consolidated launch file for real-robot mapping and localization.
# Wraps rm_navigation_reality_launch.py + msg_MID360_launch.py into one entry.
# odom -> base_footprint is published dynamically by sensor_scan_generation;
# map -> odom is published by slam_launch (SLAM mode) or small_gicp_relocalization (localization mode).
#
# Usage:
#   建图:   ros2 launch pb2025_nav_bringup rm_navigation_reality_mapping_launch.py
#   重定位: ros2 launch pb2025_nav_bringup rm_navigation_reality_mapping_launch.py slam:=False world:=<map_name>

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    livox_driver2_dir = get_package_share_directory("livox_ros_driver2")
    livox_launch_dir = os.path.join(livox_driver2_dir, "launch")

    use_rviz = LaunchConfiguration("use_rviz")
    use_composition = LaunchConfiguration("use_composition")
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    params_file = LaunchConfiguration("params_file")
    run_nav2 = LaunchConfiguration("run_nav2")

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to start RVIZ",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="True",
        description="Whether to run SLAM (True) or localization (False)",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmul_2024",
        description="Map name (without extension); resolves map/<world>.yaml and pcd/<world>.pcd in localization mode",
    )

    # ★狗默认用 realcar 配置 (base_footprint), 小车须显式传 nav2_params.yaml
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "reality", "nav2_params_realcar.yaml"),
        description="Full path to the ROS2 parameters file for all launched nodes",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Whether to use composed bringup (dog defaults to False)",
    )

    # Step 1: Navigation bringup with SLAM and robot_state_publisher
    # Disable built-in livox node since we use msg_MID360_launch.py instead
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "rm_navigation_reality_launch.py")
        ),
        launch_arguments={
            "slam": slam,
            "world": world,
            "params_file": params_file,
            "use_robot_state_pub": "True",
            "use_livox_driver": "False",
            "use_sim_time": "False",
            "use_rviz": use_rviz,
            "use_composition": use_composition,
            "run_nav2": run_nav2,
        }.items(),
    )

    # Step 2: Livox MID360 driver (with proper MID360_config.json)
    livox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_launch_dir, "msg_MID360_launch.py")
        ),
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(DeclareLaunchArgument(
        "run_nav2", default_value="True",
        description="是否启动 Nav2 规划/控制/BT; FSM 模式设 False"))
    ld.add_action(navigation_cmd)
    ld.add_action(livox_cmd)

    return ld
