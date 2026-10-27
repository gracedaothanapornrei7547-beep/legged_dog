#!/bin/bash

# can-reset.sh —— 每次跑 trot 前 reset 4 条 CAN bus（清 TEC + 重置赤乌固件状态）
# =============================================================================
#
# 【用途】跑前手动调用，确保 4 条 bus TEC=0 状态启动，避免"预热已 WARNING/PASSIVE"
#   累积到下一次跑时直接崩。
#
# 【方法】rmmod + modprobe 驱动，比 ip link down/up 更彻底（USB 层重枚举 → 适配器
#   固件状态完全归零），且只算 1 次驱动 cycle，不计入赤乌 "6 次 down/up brick" 计数器。
#
# 【为什么不用 ip link down/up】
#   赤乌科技 USB-CAN 适配器固件 bug：ip link down/up 累积 > 6 次后 FL adapter 死锁。
#   driver reload 是 USB 层操作，不会触发同一计数器。
#
# 【使用】
#   sudo ./can-reset.sh         # 手动跑一次
#   ./can-reset.sh              # 配合 rc-nopasswd 也能直接跑
#
# 【与 can-watchdog.service 的互动】
#   watchdog 每 2s 检查 + 拉 DOWN 接口。reset 期间接口短暂消失会触发 watchdog 尝试拉起，
#   但因为 driver 刚卸还没装回来，watchdog 会失败并重试。本脚本会先 stop watchdog，
#   reset 完再 start，避免抢锁。
# =============================================================================

set -u

CHANS=(can5 can3 can7 can6)  # ★2026-07-10 灵足02: FL=can5 FR=can3 BL=can7 BR=can6
BITRATE=1000000
TXQ=1000

# 必须 root（rmmod/modprobe + ip link）
if [[ $EUID -ne 0 ]]; then
    echo "[can-reset] 需要 root：用 sudo 调用，或配 rc-nopasswd" >&2
    exec sudo "$0" "$@"
fi

# ---- 0. 跳过条件：4 条 bus 都已 ERROR-ACTIVE = 完全干净，rmmod 反而干扰电机 ----
#
# 实测发现 (2026-05-27)：当电机已经 arm 好处于正常 idle 状态时，rmmod gs_usb 会
# 让 bxCAN 短暂停发几十 ms → 电机看到 CAN 沉默 → 部分 motor 进"链路丢失"自保状态
# → 下次 arm 全员失能（4 条全都 ❌ 未拿到反馈）。
#
# 所以只在 bus 真的需要 reset 时（WARNING/PASSIVE/BUS-OFF/DOWN）才做。如果全是
# ERROR-ACTIVE，直接退出 0 让 trot 启动。
all_clean=1
for c in "${CHANS[@]}"; do
    state=$(ip -d link show "$c" 2>/dev/null | grep -oE 'can state [A-Z-]+' | awk '{print $3}')
    if [[ "$state" != "ERROR-ACTIVE" ]]; then
        all_clean=0
        echo "[can-reset] $c 状态=$state，需要 reset"
        break
    fi
    # 注: 不检查 restart-ms — gs_usb STM32F0 固件不支持 ("Device doesn't support
    # restart from Bus Off"), bring-up 时已 fallback 到无 restart-ms 路径, 始终为 0 是正常。
done
if [[ $all_clean -eq 1 ]]; then
    echo "[can-reset] ✓ 4 条 bus 全部 ERROR-ACTIVE，跳过 reset (避免干扰已 arm 的电机)"
    exit 0
fi

# ---- 1. 自动探测当前 CAN 驱动（赤乌 一般是 gs_usb）----
DRIVER=""
for c in "${CHANS[@]}"; do
    if [[ -L "/sys/class/net/$c/device/driver" ]]; then
        DRIVER=$(basename "$(readlink "/sys/class/net/$c/device/driver")")
        break
    fi
done
# 探测失败：fallback 到 gs_usb（CANable / 赤乌 标准）
[[ -z "$DRIVER" ]] && DRIVER="gs_usb"
echo "[can-reset] 探测到 CAN 驱动: $DRIVER"

# ---- 2. 先停 watchdog 避免抢拉接口 ----
WATCHDOG_WAS_ACTIVE=0
if systemctl is-active --quiet can-watchdog.service 2>/dev/null; then
    echo "[can-reset] 暂停 can-watchdog.service"
    systemctl stop can-watchdog.service
    WATCHDOG_WAS_ACTIVE=1
fi

# ---- 3. 卸驱动 → 装驱动 ----
# rmmod 会让 4 个 USB-CAN 适配器从 socketCAN 完全断开 + 内部 firmware reset
echo "[can-reset] rmmod $DRIVER"
modprobe -r "$DRIVER" 2>&1 | grep -v "not currently loaded" || true
sleep 0.5

echo "[can-reset] modprobe $DRIVER"
modprobe "$DRIVER"

# 等内核 enumerate 出 8 个 gs_usb 接口 (按 USB 设备数算,不按目标名 — 名字得 lock 后才对)
echo "[can-reset] 等待 USB 重新枚举..."
for i in {1..30}; do
    sleep 0.1
    n=$(ls /sys/bus/usb/drivers/gs_usb/ 2>/dev/null | grep -c ":1.0$" || echo 0)
    [[ $n -ge 8 ]] && break
done

# 2026-05-30: udev NAME= 对并发 net device 会撞 EEXIST 静默失败 → 用 ip link 两段式 rename
# 用 lock-can-by-usb.sh 把名字按 USB 物理口锁死
HERE_DIR="$(dirname "$(readlink -f "$0")")"
if [[ -x "$HERE_DIR/lock-can-by-usb.sh" ]]; then
    echo "[can-reset] 调 lock-can-by-usb.sh 按 USB 口锁名"
    "$HERE_DIR/lock-can-by-usb.sh" | sed 's/^/  /'
else
    echo "[can-reset] ⚠ lock-can-by-usb.sh 缺失,跳过 — can 名可能错位" >&2
fi

# ---- 4. 检查 + 拉起 + 设 bitrate ----
fail=0
for c in "${CHANS[@]}"; do
    if ! ip link show "$c" &>/dev/null; then
        echo "[can-reset] ❌ $c 没回来（USB-CAN 适配器没枚举到，检查物理插拔）" >&2
        fail=1
        continue
    fi
    # 接口存在但可能是 DOWN，拉起来
    # 2026-05-30 实测: gs_usb STM32F0 固件拒绝 restart-ms (Device doesn't support restart),
    # 尝试一次留作未来固件升级钩子, 失败就 fallback 不带 restart-ms。
    ip link set "$c" up type can bitrate "$BITRATE" restart-ms 100 2>/dev/null || \
        ip link set "$c" up type can bitrate "$BITRATE" 2>/dev/null
    if ip link show "$c" 2>/dev/null | grep -q "state UP"; then
        ip link set "$c" txqueuelen "$TXQ" 2>/dev/null || true
        state=$(ip -d link show "$c" 2>/dev/null | grep -oE 'can state [A-Z-]+' | awk '{print $3}')
        tec_rec=$(ip -d -s link show "$c" 2>/dev/null | grep -oE 'tx errors [0-9]+ [0-9]+' | head -1)
        echo "[can-reset] ✓ $c UP  bitrate=$BITRATE  txq=$TXQ  can_state=$state  ${tec_rec:-(无 TEC 信息)}"
    else
        echo "[can-reset] ❌ $c ip link up 失败" >&2
        fail=1
    fi
done

# ---- 5. 恢复 watchdog ----
if [[ $WATCHDOG_WAS_ACTIVE -eq 1 ]]; then
    echo "[can-reset] 重启 can-watchdog.service"
    systemctl start can-watchdog.service
fi

# ---- 6. 总结 ----
if [[ $fail -ne 0 ]]; then
    echo "[can-reset] ⚠ 有接口失败，可能需要物理拔插 USB 重试" >&2
    exit 1
fi

echo "[can-reset] ✓ 4 条 CAN 全部 reset 完毕（TEC=0, 赤乌固件状态新鲜，可启动 trot）"
exit 0
