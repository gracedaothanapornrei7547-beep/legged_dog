#!/usr/bin/env python3
"""狗身体长方体过滤: 以 base_footprint 为中心, 滤掉狗自身机体的点.

  livox → /livox/lidar(CustomMsg) → dog_body_filter → /livox/lidar_filtered(CustomMsg) → point_lio

★为什么不查 TF (2026-07-04 修):
  MID360 是刚性挂载, base_footprint→front_mid360 是 URDF 里的固定值
  (mid360_x/y/z + yaw, 见 pb2025_sentry_robot.sdf.xmacro), 不随时间变。
  旧版在回调里 lookup_transform(timeout=1.0s), 而建图时没起 robot_state_publisher
  → 该 TF 永不到来 → 每帧阻塞满 1 秒 → 单线程 executor 卡死 → point_lio 饿死
  → "建图无点云"。改为硬编码静态挂载参数, 无 TF 依赖, 建图/导航都能用。
  挂载若改, 同步改 URDF 与这里的 mount_* 参数 (或用 launch 覆盖)。

箱体(base_footprint 帧, 狗几何中心=原点), 可 launch 覆盖:
  前后 X=[-0.35, 0.35], 左右 Y=[-0.175, 0.175], 上下 Z=[-0.30, 0.50]
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from livox_ros_driver2.msg import CustomMsg


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF/SDF 约定的 rpy → 旋转矩阵 R = Rz(yaw)·Ry(pitch)·Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


class DogBodyFilter(Node):
    def __init__(self):
        super().__init__("dog_body_filter")

        # 箱体范围 (base_footprint 帧)
        self.x_min = self.declare_parameter("x_min", -0.35).value
        self.x_max = self.declare_parameter("x_max", 0.35).value
        self.y_min = self.declare_parameter("y_min", -0.175).value
        self.y_max = self.declare_parameter("y_max", 0.175).value
        self.z_min = self.declare_parameter("z_min", -0.30).value
        self.z_max = self.declare_parameter("z_max", 0.50).value

        # 静态挂载 base_footprint←front_mid360 (URDF 值: pitch=0°平放无倾斜, yaw=-90°)
        mount_xyz = self.declare_parameter("mount_xyz", [0.13, 0.0, 0.06]).value
        mount_rpy = self.declare_parameter("mount_rpy", [0.0, 0.0, 0.0]).value
        self._t = np.asarray(mount_xyz, dtype=np.float64)
        self._R = rpy_to_matrix(*mount_rpy)

        in_topic = self.declare_parameter("input_topic", "/livox/lidar").value
        out_topic = self.declare_parameter(
            "output_topic", "/livox/lidar_filtered").value

        self.sub = self.create_subscription(
            CustomMsg, in_topic, self.callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            CustomMsg, out_topic, qos_profile_sensor_data)

        self._logged = False
        self.get_logger().info(
            f"狗长方体过滤(CustomMsg, 静态挂载, 无TF): "
            f"X=[{self.x_min},{self.x_max}] Y=[{self.y_min},{self.y_max}] "
            f"Z=[{self.z_min},{self.z_max}]  t={self._t.tolist()}  "
            f"{in_topic} → {out_topic}")

    def callback(self, msg: CustomMsg):
        n = msg.point_num
        if n == 0 or not msg.points:
            self.pub.publish(msg)
            return

        # 向量化取 xyz (lidar 帧) → 变换到 base_footprint 帧
        xyz = np.array([(p.x, p.y, p.z) for p in msg.points], dtype=np.float64)
        pb = xyz @ self._R.T + self._t   # (N,3)
        inside = (
            (pb[:, 0] >= self.x_min) & (pb[:, 0] <= self.x_max) &
            (pb[:, 1] >= self.y_min) & (pb[:, 1] <= self.y_max) &
            (pb[:, 2] >= self.z_min) & (pb[:, 2] <= self.z_max))
        keep = ~inside

        if keep.all():                    # 无点落在箱内, 原样转发(零拷贝)
            self.pub.publish(msg)
            return

        # 布尔索引选出保留的点 (向量化, 不做逐点 Python 循环)
        kept_pts = np.asarray(msg.points, dtype=object)[keep].tolist()
        out = CustomMsg()
        out.header = msg.header
        out.timebase = msg.timebase
        out.lidar_id = msg.lidar_id       # ★保全字段, 别丢 lidar_id/rsvd
        out.rsvd = msg.rsvd
        out.point_num = len(kept_pts)
        out.points = kept_pts
        self.pub.publish(out)

        if not self._logged:
            self._logged = True
            self.get_logger().info(
                f"首帧过滤: {n} → {out.point_num} 点 (滤掉狗身 {n - out.point_num})")


def main():
    rclpy.init()
    node = DogBodyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
