#!/usr/bin/env python3
"""箱子高度过滤: CustomMsg → z轴裁剪 → PointCloud2.

  订阅 /livox/lidar_filtered (狗体过滤后, front_mid360 帧)
  变换到 base_footprint 帧, 按 z 轴裁剪 (剔除箱子高度范围的激光点)
  发布 /filtered_points (PointCloud2, front_mid360 帧)

  使用硬编码挂载参数 (同 URDF), 无 TF 依赖, 建图/导航均可用。

  箱子高度参数可通过 launch 覆盖, 建图时 box_max_height=0 关闭过滤。
"""

import math
import struct

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF/SDF 约定的 rpy → 旋转矩阵 R = Rz(yaw)·Ry(pitch)·Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


# 预定义 PointField (xyz + intensity)
# CustomMsg.reflectivity (uint8 0-255) → float32 intensity
_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]
_POINT_STEP = 16  # 4 × float32


class BoxFilter(Node):
    def __init__(self):
        super().__init__("box_filter")

        # 挂载参数 (必须与 URDF 一致)
        self.declare_parameter("mount_xyz", [0.13, 0.0, 0.06])
        self.declare_parameter("mount_rpy", [0.0, 0.0, 0.0])
        # 箱子过滤参数
        self.declare_parameter("box_max_height", 0.3)
        self.declare_parameter("box_min_height", 0.05)

        mount_xyz = self.get_parameter("mount_xyz").value
        mount_rpy = self.get_parameter("mount_rpy").value
        self._box_max = float(self.get_parameter("box_max_height").value)
        self._box_min = float(self.get_parameter("box_min_height").value)

        self._t = np.asarray(mount_xyz, dtype=np.float64)
        self._R = _rpy_to_matrix(*mount_rpy)

        self._sub = self.create_subscription(
            CustomMsg, "/livox/lidar_filtered", self._cb, qos_profile_sensor_data)
        self._pub = self.create_publisher(
            PointCloud2, "/filtered_points", qos_profile_sensor_data)

        self._logged = False
        self.get_logger().info(
            f"箱子高度过滤: z∈[{self._box_min:.2f},{self._box_max:.2f}]m 剔除 → /filtered_points")

    def _cb(self, msg: CustomMsg):
        n = msg.point_num
        if n == 0 or not msg.points:
            return

        # 向量化: (N,3) xyz + (N,) reflectivity  front_mid360 帧
        pts = np.array([(p.x, p.y, p.z) for p in msg.points], dtype=np.float64)
        ref = np.array([p.reflectivity for p in msg.points], dtype=np.float32)

        # 变换到 base_footprint 帧 (z=0 为地面)
        pb = pts @ self._R.T + self._t  # (N,3)  R=Rz(-90°), p@Rᵀ = R*p (matches URDF: Rz(-90°)*p_mid360→p_base)

        # 剔除箱子高度区间内的点 (boxes)
        keep = (pb[:, 2] >= self._box_max) | (pb[:, 2] <= self._box_min)
        kept_idx = np.where(keep)[0]
        m = kept_idx.size

        if m == 0:
            return

        # 选取保留的点 (front_mid360 帧原始坐标) + intensity
        kept_xyz = pts[kept_idx].astype(np.float32)
        kept_ref = ref[kept_idx]  # uint8→float32, 0-255

        # 组装 PointCloud2 (xyz + intensity 交错)
        out = np.empty((m, 4), dtype=np.float32)
        out[:, :3] = kept_xyz
        out[:, 3] = kept_ref

        # 组装 PointCloud2
        pc2 = PointCloud2()
        pc2.header = msg.header                     # frame_id = "front_mid360"
        pc2.height = 1
        pc2.width = m
        pc2.fields = _FIELDS
        pc2.point_step = _POINT_STEP
        pc2.row_step = _POINT_STEP * m
        pc2.is_bigendian = False
        pc2.is_dense = True
        pc2.data = out.tobytes()

        self._pub.publish(pc2)

        if not self._logged:
            self._logged = True
            self.get_logger().info(f"首帧: {n} → {m} 点 (过滤 {n - m})")


def main():
    rclpy.init()
    node = BoxFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
