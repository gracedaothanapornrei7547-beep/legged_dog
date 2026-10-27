"""简化决策机 launch 文件。

用法：
  ros2 launch simple_dicision simple_mission.launch.py
  ros2 launch simple_dicision simple_mission.launch.py dry_run:=True
  ros2 launch simple_dicision simple_mission.launch.py auto_start:=False

前置依赖（另开终端）：
  · 定位 + FSM + 底盘
  · 路点已打点（~/.simple_nav_fsm_waypoints.yaml）
  · 路点映射配置文件 config/mission.yaml 已按实际打点名称填写

路点映射配置（config/mission.yaml 中的 route）：
  start        起始点
  grab         抓取点（gap12 缝中站位）
  cross        过减速带点
  place_green  食品区(绿) 放置点
  place_gray   工具区(灰) 放置点
  place_blue   仪器区(蓝) 放置点
  place_red    药品区(红) 放置点
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 路点映射配置文件（自动定位到包内的 config/mission.yaml）
    pkg_share = get_package_share_directory('simple_dicision')
    default_route_config = os.path.join(pkg_share, 'config', 'mission.yaml')

    route_config = LaunchConfiguration('route_config',
                                       default=default_route_config)
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyX5')
    serial_baud = LaunchConfiguration('serial_baud', default='115200')
    nav_timeout = LaunchConfiguration('nav_timeout_sec', default='60.0')
    grab_timeout = LaunchConfiguration('grab_timeout_sec', default='60.0')
    place_timeout = LaunchConfiguration('place_timeout_sec', default='60.0')
    arm_timeout = LaunchConfiguration('arm_timeout', default='180.0')
    nav_retry = LaunchConfiguration('nav_retry', default='1')
    waypoints_file = LaunchConfiguration('waypoints_file',
                                         default='~/.simple_nav_fsm_waypoints.yaml')
    left_color = LaunchConfiguration('left_color', default='')
    right_color = LaunchConfiguration('right_color', default='')
    auto_start = LaunchConfiguration('auto_start', default='True')
    start_delay = LaunchConfiguration('start_delay', default='2.0')
    dry_run = LaunchConfiguration('dry_run', default='False')
    dry_run_step_sec = LaunchConfiguration('dry_run_step_sec', default='0.3')
    dry_run_nav_sec = LaunchConfiguration('dry_run_nav_sec', default='0.5')

    return LaunchDescription([
        DeclareLaunchArgument('route_config', default_value=default_route_config,
                              description='路点映射配置文件路径'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyX5'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('nav_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('grab_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('place_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('arm_timeout', default_value='180.0'),
        DeclareLaunchArgument('nav_retry', default_value='1'),
        DeclareLaunchArgument('waypoints_file',
                              default_value='~/.simple_nav_fsm_waypoints.yaml'),
        DeclareLaunchArgument('auto_start', default_value='True'),
        DeclareLaunchArgument('start_delay', default_value='2.0'),
        DeclareLaunchArgument('left_color', default_value='',
                              description='左侧箱子颜色 (green/gray/blue/red)'),
        DeclareLaunchArgument('right_color', default_value='',
                              description='右侧箱子颜色 (green/gray/blue/red)'),
        DeclareLaunchArgument('dry_run', default_value='False'),
        DeclareLaunchArgument('dry_run_step_sec', default_value='0.3'),
        DeclareLaunchArgument('dry_run_nav_sec', default_value='0.5'),

        Node(
            package='simple_dicision',
            executable='simple_mission_node',
            name='simple_dicision',
            output='screen',
            parameters=[{
                'route_config': route_config,
                'serial_port': serial_port,
                'serial_baud': serial_baud,
                'nav_timeout_sec': nav_timeout,
                'grab_timeout_sec': grab_timeout,
                'place_timeout_sec': place_timeout,
                'arm_timeout': arm_timeout,
                'nav_retry': nav_retry,
                'waypoints_file': waypoints_file,
                'auto_start': auto_start,
                'start_delay': start_delay,
                'left_color': left_color,
                'right_color': right_color,
                'dry_run': dry_run,
                'dry_run_step_sec': dry_run_step_sec,
                'dry_run_nav_sec': dry_run_nav_sec,
            }],
        ),
    ])
