#!/bin/bash
# setup-env.sh —— 一次 source 整个 dog_auto 环境(ROS2 + nav + legged_nav)
# =============================================================================
#   用法(每个新终端):  source ~/dog_auto/setup-env.sh
#   之后 nav / legged_nav 的所有节点、可执行、launch 都能直接 ros2 run / ros2 launch。
#   各子工作区仍各自 `colcon build`(独立编译), 本脚本只负责把它们的 install 叠加 source。
# =============================================================================

# 脚本自身所在目录 = dog_auto 根(不写死 ~/dog_auto, 换位置也能用)
_DOG_AUTO="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"

source /opt/ros/humble/setup.bash

_srcd=""
# 优先: 各子工作区独立 install; 回退: 顶层合并 install
for _ws in nav legged_nav; do
  _f="${_DOG_AUTO}/${_ws}/install/setup.bash"
  if [ -f "$_f" ]; then source "$_f"; _srcd="${_srcd} ${_ws}"; fi
done
# 回退: 若顶层合并 install(setup.bash)存在且单独 install 不全, 补 source
if [ -f "${_DOG_AUTO}/install/setup.bash" ]; then
  source "${_DOG_AUTO}/install/setup.bash"
  _srcd="${_srcd} (顶层合并)"
fi

echo "[dog_auto] 已 source: ROS2 humble +${_srcd:- (还没编译? 先 colcon build)}"
unset _DOG_AUTO _srcd _ws _f
