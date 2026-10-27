#!/usr/bin/env python3
"""
极简导航状态机启动文件。

启动 fsm_node (simple_nav_fsm 节点) + cmd_vel_guard (安全带), 替代 Nav2 的
planner_server / bt_navigator / controller_server / costmap 层。

链路 (FSM 模式, 不跑 Nav2):
    fsm_node → /cmd_vel_raw → [cmd_vel_guard] → /cmd_vel → cmd_bridge(狗接口)

★为什么本 launch 自带 cmd_vel_guard:
  guard 原本写在 navigation_launch.py 里, FSM 模式不跑 Nav2 就会连 guard 一起丢掉
  → 命令无钳位/无 NaN 拦截/无超时归零地直连狗。这里内联狗安全包络(0.15/0.1/0.3),
  不依赖 nav2 yaml (guard 代码默认是 0.30/0.15/0.50, 是狗包络两倍, 漏传参=钳位翻倍)。
  不启动 fake_vel_transform: 狗 init_spin_speed=0 且无独立云台, 它对狗是恒等透传,
  FSM 输出已是 base_footprint 系, 直接发 /cmd_vel_raw 即可。

用法:
  ros2 launch simple_nav_fsm simple_nav.launch.py                    # 狗平台 (默认 params_dog.yaml)
  ros2 launch simple_nav_fsm simple_nav.launch.py params_file:=params.yaml  # 哨兵/小车平台
  ros2 launch simple_nav_fsm simple_nav.launch.py params_file:=/path/to/my_params.yaml
  ros2 launch simple_nav_fsm simple_nav.launch.py start_guard:=false # guard 已由别处常驻时

前置条件: 定位 (point_lio + loam_interface + small_gicp_relocalization) 和
  狗底盘 (dog_bringup, 含 cmd_bridge + nav_quad) 已运行。
  ★不要同时跑 navigation_launch.py 的 Nav2 节点 (planner/controller/bt): 会和 FSM 抢
    /navigate_to_pose action, 且 fake_vel_transform 会抢 /cmd_vel_raw。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_nav_fsm')
    params_file = LaunchConfiguration('params_file')
    start_guard = LaunchConfiguration('start_guard')

    declare_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('simple_nav_fsm'), 'config', 'params_dog.yaml']),
        description='参数文件路径; 狗平台默认 params_dog.yaml, '
                    '哨兵/小车用 params.yaml')

    declare_guard_cmd = DeclareLaunchArgument(
        'start_guard', default_value='true',
        description='是否随 FSM 一起启动 cmd_vel_guard; '
                    '若 guard 已由底盘/其它 launch 常驻则设 false')

    fsm_node = Node(
        package='simple_nav_fsm',
        executable='fsm_node',
        name='simple_nav_fsm',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'info'],
        # ★ 与 Nav2 互斥: 本节点接管 /navigate_to_pose action, 并发布 /cmd_vel_raw,
        #   不要同时跑 Nav2 的 planner_server / controller_server / bt_navigator
    )

    # 安全带: 与 FSM 同起, 内联狗包络 (不依赖 nav2 yaml, 避免代码默认 0.30/0.15/0.50 漏钳)
    guard_node = Node(
        package='cmd_vel_guard',
        executable='cmd_vel_guard',
        name='cmd_vel_guard',
        output='screen',
        condition=IfCondition(start_guard),
        parameters=[{
            'input_topic': 'cmd_vel_raw',
            'output_topic': 'cmd_vel',
            'vx_max': 0.30,
            'vy_max': 0.2,
            'w_max': 0.6,
            'publish_rate_hz': 50.0,
            'input_timeout_s': 0.3,
            'translate_priority': False,   # 直行时不禁自转 (FSM 靠小 wz 纠偏)
        }],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_cmd)
    ld.add_action(declare_guard_cmd)
    ld.add_action(fsm_node)
    ld.add_action(guard_node)
    return ld
