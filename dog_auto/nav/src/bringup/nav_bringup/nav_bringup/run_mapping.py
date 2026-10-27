#!/usr/bin/env python3
"""建图程序 (确定性自动保存)。

启动建图核心 (MID360 驱动 + 狗体长方体过滤 dog_body_filter + point_lio),
用 SBUS 遥控 (另开终端 `ros2 run legged_nav_ros sbus_teleop`) 把狗在场地开一圈;
**按 Ctrl+C 结束建图, 自动保存两份 (缺一重定位就会偏)**:
  - 点云  -> pb2025_nav_bringup/pcd/reality/<map>.pcd        (small_gicp 重定位用)
  - 栅格图 -> pb2025_nav_bringup/map/reality/<map>.pgm/.yaml (Nav2 costmap 用)
  src 与 install/share 两处都写, 且两份同源, 之后导航无需再 colcon build。

关键: 本程序做父进程, Ctrl+C 时先让 point_lio 退出并写完 scans.pcd, 等它真正结束后再投影栅格图,
不受 ros2 launch 收尾超时影响 (这是直接 ros2 launch dog_mapping 做不到的 —— 那样只落 scans.pcd,
不会自动转栅格 + 拷重定位 pcd)。

  ros2 run nav_bringup run_mapping --map-name arena [--rviz] [--resolution 0.05 --zmin 0.1 --zmax 0.8]
  RViz(--rviz)须在非 snap 终端或用 desnap-ros.sh 起, 否则 RViz 崩。
"""
import argparse
import os
import shutil
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory

from nav_bringup.pcd_to_grid import pcd_to_map

NAV_SRC = os.path.expanduser("~/dog_auto/nav/src")
POINT_LIO_PCD = os.path.join(NAV_SRC, "mapping/point_lio/PCD/scans.pcd")
BRINGUP_SRC = os.path.join(NAV_SRC, "navigation/pb2025_nav_bringup")


def out_dirs():
    share = get_package_share_directory("pb2025_nav_bringup")
    pcd_dirs = [os.path.join(share, "pcd", "reality"),
                os.path.join(BRINGUP_SRC, "pcd", "reality")]
    grid_dirs = [os.path.join(share, "map", "reality"),
                 os.path.join(BRINGUP_SRC, "map", "reality")]
    return pcd_dirs, grid_dirs


def save_map(map_name, resolution, z_min, z_max):
    if not os.path.isfile(POINT_LIO_PCD):
        print(f"\n[run_mapping] ❌ 没找到 {POINT_LIO_PCD}")
        print("[run_mapping]    说明 point_lio 没扫到数据 (检查雷达连接/驱动 bind) 或建图时间太短。")
        return False
    pcd_dirs, grid_dirs = out_dirs()
    ok = True
    for d in pcd_dirs:
        try:
            os.makedirs(d, exist_ok=True)
            dst = os.path.join(d, f"{map_name}.pcd")
            shutil.copyfile(POINT_LIO_PCD, dst)
            print(f"[run_mapping] ✅ 点云  -> {dst}")
        except Exception as e:  # noqa: BLE001
            print(f"[run_mapping] ⚠ 写 {d} 失败: {e}"); ok = False
    for d in grid_dirs:
        try:
            os.makedirs(d, exist_ok=True)
            base = os.path.join(d, map_name)
            pgm, yaml, n, shape = pcd_to_map(POINT_LIO_PCD, base, resolution, z_min, z_max)
            print(f"[run_mapping] ✅ 栅格图 -> {pgm} ({shape[1]}x{shape[0]} @{resolution}m, {n} 点)")
        except Exception as e:  # noqa: BLE001
            print(f"[run_mapping] ⚠ 投影 {d} 失败: {e}"); ok = False
    if ok:
        print(f"[run_mapping] 🎉 地图 '{map_name}' 已保存(栅格+重定位pcd同源), 可直接导航: "
              f"ros2 launch nav_bringup dog_navigation.launch.py map_name:={map_name}")
    return ok


def main():
    ap = argparse.ArgumentParser(description="MID360 建图 + 结束自动保存")
    ap.add_argument("--map-name", default="arena")
    ap.add_argument("--rviz", action="store_true", help="建图时开 RViz(须非 snap 终端/desnap)")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--zmin", type=float, default=0.1)
    ap.add_argument("--zmax", type=float, default=0.8, help="栅格高度带上界(围墙0.8)")
    a = ap.parse_args()

    # 建图前清掉上一轮的 scans.pcd, 避免误存旧图
    try:
        if os.path.isfile(POINT_LIO_PCD):
            os.remove(POINT_LIO_PCD)
    except OSError:
        pass

    # ★带狗体过滤的建图 (dog_mapping.launch.py: MID360 + dog_body_filter + point_lio)
    cmd = ["ros2", "launch", "nav_bringup", "dog_mapping.launch.py",
           f"rviz:={'True' if a.rviz else 'False'}"]
    print(f"[run_mapping] 启动建图: {' '.join(cmd)}")
    print("[run_mapping] 另开终端遥控开车: ros2 run legged_nav_ros sbus_teleop (ch5低位使能)")
    print("[run_mapping] 走遍全场后按 Ctrl+C 结束并自动保存(栅格图 + 重定位 pcd)。\n")
    # 子进程独立进程组, 这样 Ctrl+C 只命中本程序, 由本程序可控地转发
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)

    try:
        proc.wait()
    except KeyboardInterrupt:
        print("\n[run_mapping] 收到 Ctrl+C, 正在结束建图并等待 point_lio 写出点云 ...")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)  # 让 point_lio 走退出存盘路径
        except ProcessLookupError:
            pass
        # 等 launch 整体退出 (= point_lio 已写完 scans.pcd); 最多等 60s, 超时再升级
        deadline = time.time() + 60
        while proc.poll() is None and time.time() < deadline:
            time.sleep(0.3)
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            time.sleep(2)

    print("[run_mapping] 建图核心已退出, 开始保存地图 ...")
    return 0 if save_map(a.map_name, a.resolution, a.zmin, a.zmax) else 1


if __name__ == "__main__":
    sys.exit(main())
