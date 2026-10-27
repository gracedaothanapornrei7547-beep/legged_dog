#!/usr/bin/env python3
"""建图核心 (驱动 + point_lio LIO-SLAM)。

注意: 推荐用 `ros2 run nav_bringup run_mapping` 启动建图——它会在结束(Ctrl+C)时
确定性地自动保存点云+栅格图。本 launch 只起建图核心, 单独跑不会自动保存。

  ros2 launch nav_bringup mapping.launch.py        # 仅核心, 手动保存
  ros2 run   nav_bringup run_mapping --map-name arena   # 推荐: 结束自动保存
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    livox_launch = PathJoinSubstitution(
        [FindPackageShare("livox_ros_driver2"), "launch", "msg_MID360_launch.py"])
    point_lio_launch = PathJoinSubstitution(
        [FindPackageShare("point_lio"), "launch", "point_lio.launch.py"])

    return LaunchDescription([
        DeclareLaunchArgument("map_name", default_value="arena",
                              description="(仅记录用; 实际保存由 run_mapping 完成)"),
        # ① MID360 驱动 (10Hz, 配置自带 IP .146)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(livox_launch)),
        # ② point_lio LIO 建图 (mid360.yaml 已开 pcd_save_en, 退出写 PCD/scans.pcd)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(point_lio_launch)),
    ])
