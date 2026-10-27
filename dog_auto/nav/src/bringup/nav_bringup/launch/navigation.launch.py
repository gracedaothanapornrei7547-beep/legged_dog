#!/usr/bin/env python3
"""重定位 + 路径规划 + 导航。

加载建图阶段保存的 <map_name>.pcd / .yaml:
  - point_lio 出实时里程计 (odom)
  - sensor_scan_generation 出 odom->base TF
  - small_gicp_relocalization 用 <map_name>.pcd 配准出 map->odom TF
  - Nav2 (costmap + planner + omni-pid-pursuit 控制器) 规划并输出 /cmd_vel

用法:
  ros2 launch nav_bringup navigation.launch.py map_name:=arena
  (可加 use_rviz:=false 无头运行)

说明: 复用 pb2025_nav_bringup 的 rm_navigation_reality_mapping_launch.py (slam:=False = 重定位模式),
它会拉起驱动 + point_lio + 感知 + small_gicp + Nav2。地图按 world=<map_name> 从
pb2025_nav_bringup/{map,pcd}/reality/ 解析 (建图阶段已写入此处)。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_name = LaunchConfiguration("map_name")
    use_rviz = LaunchConfiguration("use_rviz")
    params_file = LaunchConfiguration("params_file")
    run_nav2 = LaunchConfiguration("run_nav2")

    bringup_pkg = PathJoinSubstitution([FindPackageShare("pb2025_nav_bringup")])
    nav_launch = PathJoinSubstitution(
        [bringup_pkg, "launch", "rm_navigation_reality_mapping_launch.py"])

    # ★狗默认用 realcar 配置 (base_footprint), 小车须显式传 params_file
    dog_params_default = PathJoinSubstitution(
        [bringup_pkg, "config", "reality", "nav2_params_realcar.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("map_name", default_value="arena",
                              description="要加载的地图名 (建图阶段保存的)"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=dog_params_default,
                              description="Nav2 参数文件; 狗默认 nav2_params_realcar.yaml"),
        DeclareLaunchArgument("run_nav2", default_value="True",
                              description="Nav2 模式=True; FSM 模式设 False(不起 Nav2 节点)"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments={
                "slam": "False",          # False = 重定位+导航模式
                "world": map_name,
                "use_rviz": use_rviz,
                "params_file": params_file,
                "run_nav2": run_nav2,
            }.items(),
        ),
    ])
