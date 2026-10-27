#!/usr/bin/env bash
# 冒烟测试:快速验证"包齐全 + launch 可解析",拦截"悄悄缺包"类问题。
# (本次 sdformat_tools / loam_interface 缺失,这一步能立刻发现。)
# 用法: ./smoke_test.sh
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "==> [1/3] colcon build"
colcon build --symlink-install >/dev/null 2>&1 && echo "    ✅ build OK"

echo "==> [2/3] 建图 launch 解析"
timeout 30 ros2 launch nav_bringup mapping.launch.py --show-args >/dev/null 2>&1 \
  && echo "    ✅ mapping.launch.py OK"

echo "==> [3/3] 导航 launch 解析 (会触发 sdformat_tools / 描述生成的 import)"
timeout 45 ros2 launch nav_bringup navigation.launch.py --show-args >/dev/null 2>&1 \
  && echo "    ✅ navigation.launch.py OK"

echo "🎉 冒烟测试通过"
