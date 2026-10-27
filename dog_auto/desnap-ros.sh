#!/usr/bin/env bash
# desnap-ros.sh —— 在 snap 版 VSCode 终端里, 用"去 snap 污染"的干净环境跑 ROS 图形程序(尤其 RViz)。
# ---------------------------------------------------------------------------
# 起因: VSCode 是 snap(classic) 安装, 其集成终端把 core20 基座的 glibc2.31 的
#       libpthread / GTK / locale 等塞进环境; 原生 RViz(系统 glibc2.35) 加载后
#       符号冲突崩溃: "symbol lookup error ... __libc_pthread_init GLIBC_PRIVATE"。
#       本脚本摘掉这些 snap 注入 + 清 LD_LIBRARY_PATH 里的 /snap, 再 source ROS 并执行命令。
#
# 用法(把要跑的命令原样跟在后面):
#   ~/dog_auto/desnap-ros.sh ros2 launch pb2025_nav_bringup rviz_launch.py
#   ~/dog_auto/desnap-ros.sh ros2 launch nav_bringup navigation.launch.py map_name:=arena use_rviz:=true
#   ~/dog_auto/desnap-ros.sh rviz2
# ---------------------------------------------------------------------------

# 1) 摘掉 snap 注入的环境变量(这些让 ld/GTK 去 /snap/core20 找库)
unset SNAP SNAP_NAME SNAP_INSTANCE_NAME SNAP_REVISION SNAP_VERSION SNAP_ARCH \
      SNAP_LIBRARY_PATH SNAP_DATA SNAP_USER_DATA SNAP_USER_COMMON SNAP_COMMON \
      SNAP_CONTEXT SNAP_COOKIE SNAP_REAL_HOME SNAP_EUID SNAP_UID SNAP_LAUNCHER_ARCH_TRIPLET \
      GTK_PATH GTK_EXE_PREFIX LOCPATH GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR \
      GSETTINGS_SCHEMA_DIR GTK_IM_MODULE_FILE
export XDG_DATA_DIRS="/usr/share/ubuntu:/usr/local/share:/usr/share:/var/lib/snapd/desktop"

# 2) source ROS + 两个工作区(nav / legged_nav)
source /opt/ros/humble/setup.bash
[ -f "$HOME/dog_auto/nav/install/setup.bash" ]        && source "$HOME/dog_auto/nav/install/setup.bash"
[ -f "$HOME/dog_auto/legged_nav/install/setup.bash" ] && source "$HOME/dog_auto/legged_nav/install/setup.bash"

# 3) LD_LIBRARY_PATH 去掉 /snap 与 /var/lib/snapd(核心: 别让 RViz 用 core20 的 libpthread)
export LD_LIBRARY_PATH="$(printf '%s' "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -vE '/snap/|/var/lib/snapd' | paste -sd:)"

# 4) 本机有 mihomo, DDS 发现走回环(与 nav/cmd_bridge 一致才能互相看到)
export ROS_LOCALHOST_ONLY=1

exec "$@"
