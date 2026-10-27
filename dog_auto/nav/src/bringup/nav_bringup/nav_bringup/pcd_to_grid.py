#!/usr/bin/env python3
"""PCD -> 2D occupancy grid (.pgm + .yaml).

把 point_lio 建出的 3D 点云地图投影成 Nav2 用的 2D 占据栅格图:
取某一高度带内的点 (默认 0.1~0.8m, 滤掉地面/天花板) 投到 XY 平面, 有点的格=占据,
扫描覆盖到、但无障碍点的格=空闲, 其余=未知。

与 small_gicp 用的 PCD 同源, 所以栅格图与点云地图坐标系/原点天然对齐。

可独立运行:
  ros2 run nav_bringup pcd_to_grid --pcd scans.pcd --out /path/arena \
      --resolution 0.05 --zmin 0.1 --zmax 0.8
"""
import argparse
import struct
import sys

import numpy as np

_TYPE_MAP = {
    ("F", 4): "f4", ("F", 8): "f8",
    ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4", ("U", 8): "u8",
    ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4", ("I", 8): "i8",
}


def read_pcd_xyz(path):
    """读 PCD (ascii / binary / binary_compressed 不支持后者), 动态解析头, 返回 Nx3 float32 (x,y,z)."""
    with open(path, "rb") as f:
        fields, sizes, types, counts = [], [], [], []
        npoints = None
        data_fmt = None
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD 头未结束就到文件尾")
            text = line.decode("ascii", "replace").strip()
            if not text or text.startswith("#"):
                continue
            key, _, rest = text.partition(" ")
            key = key.upper()
            toks = rest.split()
            if key == "FIELDS":
                fields = toks
            elif key == "SIZE":
                sizes = [int(t) for t in toks]
            elif key == "TYPE":
                types = toks
            elif key == "COUNT":
                counts = [int(t) for t in toks]
            elif key == "POINTS":
                npoints = int(toks[0])
            elif key == "WIDTH" and npoints is None:
                npoints = int(toks[0])
            elif key == "DATA":
                data_fmt = toks[0].lower()
                break
        if not counts:
            counts = [1] * len(fields)

        # 组装结构化 dtype (展开 count>1 的字段)
        names, formats = [], []
        for name, sz, ty, cnt in zip(fields, sizes, types, counts):
            np_ty = _TYPE_MAP.get((ty.upper(), sz))
            if np_ty is None:
                raise ValueError(f"不支持的字段类型 {ty}{sz}")
            if cnt == 1:
                names.append(name)
                formats.append(np_ty)
            else:
                for k in range(cnt):
                    names.append(f"{name}_{k}")
                    formats.append(np_ty)
        dtype = np.dtype({"names": names, "formats": [np.dtype(x) for x in formats]})

        if data_fmt == "binary":
            buf = f.read(npoints * dtype.itemsize)
            arr = np.frombuffer(buf, dtype=dtype, count=npoints)
            xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)
        elif data_fmt == "ascii":
            rows = []
            col = {n: i for i, n in enumerate(names)}
            for _ in range(npoints):
                ln = f.readline().split()
                if not ln:
                    break
                rows.append((float(ln[col["x"]]), float(ln[col["y"]]), float(ln[col["z"]])))
            xyz = np.asarray(rows, dtype=np.float32)
        else:
            raise ValueError(f"不支持的 DATA 格式: {data_fmt} (binary_compressed 请先转 binary)")

    # 去掉 NaN/Inf
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return xyz


def project_to_grid(xyz, resolution=0.05, z_min=0.1, z_max=0.8, margin=1.0):
    """高度带过滤 + XY 投影 -> 占据栅格 (Nav2 约定: 0=空闲,100=占据,-1=未知 的 PGM 编码)."""
    if xyz.shape[0] == 0:
        raise ValueError("点云为空")
    # 用全部点确定地图范围 (含地面, 保证空闲区覆盖到走过的地方)
    xmin, ymin = xyz[:, 0].min() - margin, xyz[:, 1].min() - margin
    xmax, ymax = xyz[:, 0].max() + margin, xyz[:, 1].max() + margin
    w = int(np.ceil((xmax - xmin) / resolution))
    h = int(np.ceil((ymax - ymin) / resolution))

    # 占据: 高度带内的点
    band = xyz[(xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)]
    # 空闲: 任意高度有点扫到的列 (近似可通行/已观测)
    seen = xyz

    def idx(pts):
        cx = ((pts[:, 0] - xmin) / resolution).astype(np.int32)
        cy = ((pts[:, 1] - ymin) / resolution).astype(np.int32)
        ok = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        return cx[ok], cy[ok]

    grid = np.full((h, w), -1, dtype=np.int16)   # 未知
    sx, sy = idx(seen)
    grid[sy, sx] = 0                              # 观测到 -> 空闲
    ox, oy = idx(band)
    grid[oy, ox] = 100                            # 障碍 -> 占据
    return grid, xmin, ymin, resolution


def write_map(grid, origin_x, origin_y, resolution, out_base):
    """写 <out_base>.pgm + <out_base>.yaml (Nav2 occupancy_grid 约定)."""
    h, w = grid.shape
    # PGM: 254=空闲(白) 0=占据(黑) 205=未知(灰); 行优先, 图像第一行=最大 y -> 上下翻转
    img = np.full((h, w), 205, dtype=np.uint8)
    img[grid == 0] = 254
    img[grid == 100] = 0
    img = np.flipud(img)

    pgm = out_base + ".pgm"
    with open(pgm, "wb") as f:
        f.write(b"P5\n")
        f.write(f"{w} {h}\n255\n".encode("ascii"))
        f.write(img.tobytes())

    yaml = out_base + ".yaml"
    pgm_name = pgm.rsplit("/", 1)[-1]
    with open(yaml, "w") as f:
        f.write(f"image: {pgm_name}\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0]\n")
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n")
    return pgm, yaml


def pcd_to_map(pcd_path, out_base, resolution=0.05, z_min=0.1, z_max=0.8):
    xyz = read_pcd_xyz(pcd_path)
    grid, ox, oy, res = project_to_grid(xyz, resolution, z_min, z_max)
    pgm, yaml = write_map(grid, ox, oy, res, out_base)
    return pgm, yaml, xyz.shape[0], grid.shape


def main():
    ap = argparse.ArgumentParser(description="PCD -> 2D occupancy grid")
    ap.add_argument("--pcd", required=True)
    ap.add_argument("--out", required=True, help="输出前缀, 生成 <out>.pgm/.yaml")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--zmin", type=float, default=0.1)
    ap.add_argument("--zmax", type=float, default=0.8)
    a = ap.parse_args()
    pgm, yaml, n, shape = pcd_to_map(a.pcd, a.out, a.resolution, a.zmin, a.zmax)
    print(f"[pcd_to_grid] {n} 点 -> 栅格 {shape[1]}x{shape[0]} @{a.resolution}m")
    print(f"[pcd_to_grid] 已写 {pgm}  +  {yaml}")


if __name__ == "__main__":
    sys.exit(main())
