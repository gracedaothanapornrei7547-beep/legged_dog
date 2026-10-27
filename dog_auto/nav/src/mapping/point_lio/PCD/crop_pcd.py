import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("scans.pcd")
points = np.asarray(pcd.points)

# ===== 修改这里（你的室内范围）=====
xmin, xmax = -6, 6
ymin, ymax = -6, 6
zmin, zmax = -2, 2
# ================================

mask = (
    (points[:,0] > xmin) & (points[:,0] < xmax) &
    (points[:,1] > ymin) & (points[:,1] < ymax) &
    (points[:,2] > zmin) & (points[:,2] < zmax)
)

cropped = o3d.geometry.PointCloud()
cropped.points = o3d.utility.Vector3dVector(points[mask])

o3d.io.write_point_cloud("room.pcd", cropped)

print("裁剪完成，输出 room.pcd")
