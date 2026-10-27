#!/usr/bin/env bash
# 一键搭建 nav 工作区(新设备首次部署 / 依赖变更后运行)
# 用法: ./setup.sh
# 说明: small_gicp 已 vendored 进仓库,build 全程不需要访问 GitHub。
#       仅需:apt(系统库) + PyPI(xmacro)。
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash

echo "==> [1/3] 系统依赖 (rosdep)"
sudo rosdep init 2>/dev/null || true
if ! rosdep update; then
  echo "⚠ rosdep update 失败(国内常见:raw.githubusercontent.com 被墙)。"
  echo "  解决见 README 第八节:用 fishros 一键配置 或 清华 tuna 镜像,再重跑 ./setup.sh"
fi
rosdep install --from-paths src -y --ignore-src

echo "==> [2/3] Python 依赖 (sdformat_tools 运行时需要 xmacro)"
pip install "xmacro==1.2.1"

echo "==> [3/3] 编译 (Release + symlink-install) —— small_gicp 已 vendored,无需 GitHub"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "✅ 完成。每个新终端执行: source ~/dog_auto/nav/install/setup.bash"
echo "   建图: ros2 run nav_bringup run_mapping --map-name arena"
echo "   导航: ros2 launch nav_bringup navigation.launch.py map_name:=arena"
