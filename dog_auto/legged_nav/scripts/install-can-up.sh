#!/bin/bash
# install-can-up.sh —— 一键安装：CAN setup + watchdog + 免密码 sudo
#
# 装上之后：
#   - 开机自动拉起 4 条 CAN 接口（一次）
#   - 后台 watchdog 持续监视，DOWN 立刻拉回 UP，UP 绝不 cycle（防赤乌死锁）
#   - 用户 rc 所有 sudo 命令免密码（开发流程畅通）
#
# 用法（狗端）：
#   cd ~/Desktop/legged_control_lite/scripts && sudo ./install-can-up.sh
#
# 看运行状态：
#   systemctl status can-watchdog.service
#   journalctl -u can-watchdog.service -f          # 实时日志
#
# 卸载：
#   sudo systemctl disable --now can-watchdog.service
#   sudo rm /etc/systemd/system/can-watchdog.service
#   sudo rm /usr/local/bin/setup-can.sh /usr/local/bin/can-watchdog.sh
#   sudo rm /etc/sudoers.d/rc-nopasswd     # 如果想恢复密码 sudo

set -e
HERE="$(cd "$(dirname "$0")" && pwd)"

if [[ $EUID -ne 0 ]]; then
    echo "请用 sudo 跑这条脚本" >&2
    exit 1
fi

echo "[1/6] 装 setup-can.sh → /usr/local/bin/"
install -m 755 "$HERE/setup-can.sh" /usr/local/bin/setup-can.sh

echo "[2/6] 装 can-watchdog.sh → /usr/local/bin/"
install -m 755 "$HERE/can-watchdog.sh" /usr/local/bin/can-watchdog.sh

echo "[3/6] 装 can-watchdog.service → /etc/systemd/system/"
install -m 644 "$HERE/can-watchdog.service" /etc/systemd/system/can-watchdog.service

echo "[4/6] 装 rc-nopasswd → /etc/sudoers.d/  (rc 用户免密码 sudo)"
install -m 0440 "$HERE/rc-nopasswd" /etc/sudoers.d/rc-nopasswd
# 校验语法，错了立刻卸载防 brick
if ! visudo -c -f /etc/sudoers.d/rc-nopasswd; then
    echo "❌ sudoers 语法错，回滚" >&2
    rm -f /etc/sudoers.d/rc-nopasswd
    exit 1
fi

echo "[5/6] systemd reload + enable + start"
systemctl daemon-reload
# 老的 can-up.service 已经不用了，卸载
systemctl disable can-up.service 2>/dev/null || true
rm -f /etc/systemd/system/can-up.service
# 启用 watchdog（开机自启 + 立即跑）
systemctl enable can-watchdog.service
systemctl restart can-watchdog.service
sleep 1   # 给 watchdog 1 秒去把 CAN 拉起来

echo "[6/6] 验证状态"
for c in can0 can1 can6 can7; do  # 2026-05-30: udev locks fr=can0 fl=can1 bl=can6 br=can7
    ip -d link show "$c" 2>/dev/null | grep -E "can state|state UP|state DOWN" | head -1 | sed "s/^/  $c: /"
done
echo
echo "✓ 装好了。"
echo "  - 开机自动起 watchdog + 拉 CAN"
echo "  - rc 用户 sudo 全免密码"
echo "  - 看运行日志: journalctl -u can-watchdog.service -f"
echo "  - 手动重启: sudo systemctl restart can-watchdog.service"
