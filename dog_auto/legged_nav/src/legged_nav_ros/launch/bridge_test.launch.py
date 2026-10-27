# bridge_test.launch.py —— nav → 狗接口 桥链路联调（不接真狗）
# =============================================================================
#   起两件东西:
#     1) cmd_bridge   —— 订 /cmd_vel(Twist) → seqlock 写 shm /legged_nav_cmd（shm 创建者）
#     2) shm_monitor  —— 临时「狗接口」接收端：只读 shm，用与真狗 ShmCommandSource 完全一致的
#                        seqlock + 0.3s 看门狗打印收到的 Vx/Vy/ω/gait（验证格式字节级符合）
#
#   配合 nav 侧联调（你运行时接 MID360 + RViz 发 goal）:
#     终端A:  ros2 launch nav_bringup navigation.launch.py map_name:=<你的地图>   # MID360+Nav2+RViz
#     终端B:  ros2 launch legged_nav_ros bridge_test.launch.py                    # 本文件
#     RViz:   用「2D Goal Pose」发 goal → 终端B 的 shm_monitor 打印出符合狗接口的速度信号
#
#   纯本地自测(不用 nav): 终端B 起本 launch，另一端 `ros2 run legged_nav shm_feed 0.15 0 0.3`。
#
#   注: cmd_bridge 订的是全局 /cmd_vel；若 nav 跑在命名空间下，给 cmd_bridge 加 remap 或把
#       nav 的 fake_vel_transform 输出对到 /cmd_vel。
# =============================================================================
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # shm 写端（创建者），必须先起
        Node(
            package='legged_nav_ros', executable='cmd_bridge', name='cmd_bridge',
            output='screen',
        ),
        # 临时「狗接口」接收端。shm_monitor 是普通二进制（非 rclcpp 节点），用 ExecuteProcess
        # 直接跑，避免 launch_ros 给它注入 --ros-args 污染它的位置参数。
        ExecuteProcess(
            cmd=['ros2', 'run', 'legged_nav', 'shm_monitor'],
            output='screen',
        ),
    ])
