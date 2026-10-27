#!/bin/bash

# install-autostart.sh —— 一键装开机自启: CAN看门狗 + 狗控制
# 用法: cd ~/Desktop/legged && sudo scripts/install-autostart.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"

if [[ $EUID -ne 0 ]]; then
    echo "请用 sudo: sudo $0" >&2; exit 1
fi

echo "=== [1/5] 装 setup-can.sh / can-watchdog.sh / lock-can-by-usb.sh ==="
install -m 755 "$HERE/setup-can.sh"          /usr/local/bin/
install -m 755 "$HERE/can-watchdog.sh"       /usr/local/bin/
install -m 755 "$HERE/lock-can-by-usb.sh"    /usr/local/bin/
cp -f "$HERE/lock-can-by-usb.sh"             /usr/local/bin/

echo "=== [2/5] 装 systemd service 文件 ==="
cat > /etc/systemd/system/can-watchdog.service << 'SVC'
[Unit]
Description=CAN watchdog (can5/can3/can7/can6)
After=systemd-udev-settle.service usb.target

[Service]
Type=simple
ExecStartPre=/usr/local/bin/setup-can.sh
ExecStart=/usr/local/bin/can-watchdog.sh
Restart=always
RestartSec=3
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
SVC

cat > /etc/systemd/system/trot-quad.service << 'SVC'
[Unit]
Description=Dog VMC trot controller
After=can-watchdog.service

[Service]
Type=simple
User=rc
WorkingDirectory=/home/rc/Desktop/legged
Environment=CAN_RESET_SCRIPT=/home/rc/Desktop/legged/scripts/can-reset.sh
ExecStart=/home/rc/Desktop/legged/install/parallel/lib/parallel/trot_quad
Restart=no
LimitRTPRIO=99
LimitMEMLOCK=infinity
KillSignal=SIGINT
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
SVC

echo "=== [3/5] 免密sudo ==="
install -m 0440 "$HERE/rc-nopasswd" /etc/sudoers.d/rc-nopasswd
visudo -c -f /etc/sudoers.d/rc-nopasswd

echo "=== [4/5] enable + start ==="
systemctl daemon-reload
systemctl disable can-up.service 2>/dev/null || true
systemctl enable can-watchdog.service
systemctl enable trot-quad
systemctl restart can-watchdog.service
sleep 2
systemctl start trot-quad 2>/dev/null || true

echo "=== [5/5] 验证 ==="
echo "  can-watchdog: $(systemctl is-active can-watchdog)"
echo "  trot-quad:    $(systemctl is-active trot-quad)"
for can in can5 can3 can7 can6; do
  st=$(cat /sys/class/net/$can/operstate 2>/dev/null || echo MISSING)
  echo "  $can: $st"
done
echo ""
echo "✓ 装好了。sudo reboot 验证开机自启。"
echo "  紧急停: sudo systemctl stop trot-quad"
echo "  日志:   journalctl -u trot-quad -f"
