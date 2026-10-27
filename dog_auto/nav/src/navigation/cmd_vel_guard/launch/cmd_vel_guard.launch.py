# cmd_vel_guard.launch.py —— 单独起安全带节点(测试 / 独立用)。
#   生产环境已随 pb2025_nav_bringup 的 navigation_launch.py 自动起(参数走 nav2_params.yaml),
#   本 launch 仅供单独调试 / 本地自测。
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmd_vel_guard', executable='cmd_vel_guard', name='cmd_vel_guard',
            output='screen',
            parameters=[{
                'input_topic': 'cmd_vel_raw',
                'output_topic': 'cmd_vel',
                'vx_max': 0.30,
                'vy_max': 0.15,
                'w_max': 0.50,
                'publish_rate_hz': 50.0,
                'input_timeout_s': 0.30,
                'translate_priority': False,
                'translate_speed_threshold': 0.05,
            }],
        ),
    ])
