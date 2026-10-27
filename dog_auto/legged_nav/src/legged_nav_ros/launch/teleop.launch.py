from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # shm 写端(创建者), 须先起
        Node(
            package='legged_nav_ros', executable='cmd_bridge', name='cmd_bridge',
            output='screen',
        ),
        # 遥控器 → /cmd_vel(站 nav 的位)
        Node(
            package='legged_nav_ros', executable='sbus_teleop', name='sbus_teleop',
            output='screen',
            parameters=[{
                'max_vx': 0.30, 'max_vy': 0.15, 'max_omega': 1.0,
                'ch_forward': 2, 'sign_forward': 1.0,
                'ch_lateral': 3, 'sign_lateral': -1.0,
                'ch_turn': 0, 'sign_turn': 1.0,
                'deadzone': 0.08, 'sbus_timeout': 0.30,
                'port': '/dev/ttySBUS', 'invert': False, 'serial_mode': 'N1',
            }],
        ),
    ])
