#!/bin/bash

# setup-can.sh —— 把 4 条 USB-CAN 拉成可用状态（★ 幂等：已 UP 绝不 cycle）
#
# ★ 关键设计（2026-05-26 重写）：
#   赤乌科技 USB-CAN 适配器有固件 bug —— 反复 down/up 超过 6 次后 FL adapter
#   再也无法发送（hardware brick）。所以本脚本设计为：
#     1. 检查接口当前 state
#     2. 已经 UP 的：完全不动，连 bitrate 都不重设（重设也会 cycle）
#     3. 只对 DOWN/不存在 的接口做一次 up
#   配合 can-watchdog.service 保证开机后 4 条 CAN 一直 UP，trot 跑完后也不 down。
#
# 设置：
#   bitrate 1000000      —— 1Mbit/s（电机协议固定）
#   txqueuelen 1000      —— TX 队列从默认 10 拉到 1000
#   restart-ms 100         —— 2026-05-30: 尝试设 (BUS-OFF 自动 restart hook,
#                             未来固件升级可启用); gs_usb STM32F0 固件实测拒绝
#                             ("Device doesn't support restart"),失败 fallback 不带。
#
# 退出码：0=4 条都 UP 成功；1=有接口起不来

set -u
CHANS=(can5 can3 can7 can6)  # ★2026-07-10 灵足02: FL=can5 FR=can3 BL=can7 BR=can6
BITRATE=1000000
TXQ=1000

fail=0
for c in "${CHANS[@]}"; do
    # 接口存在吗？
    if ! ip link show "$c" &>/dev/null; then
        echo "[setup-can] ❌ $c: 接口不存在（USB-CAN 适配器没插好？）" >&2
        fail=1
        continue
    fi

    # 当前 link state（UP / DOWN / UNKNOWN）
    link_state=$(ip link show "$c" | grep -oE 'state [A-Z]+' | awk '{print $2}')

    if [[ "$link_state" == "UP" ]]; then
        # ★ 已 UP —— 完全不动！避免 赤乌 adapter 反复 cycle 后死锁
        can_state=$(ip -d link show "$c" 2>/dev/null | grep -oE 'can state [A-Z-]+' | awk '{print $3}')
        echo "[setup-can] ✓ $c: 已 UP (can state=$can_state)，跳过 cycle 防 赤乌 adapter 死锁"
    else
        # DOWN 或异常 → 拉起来
        # 2026-05-30: 优先带 restart-ms 100 (BUS-OFF 后内核自动 restart),老固件不支持就 fallback
        ip link set "$c" up type can bitrate "$BITRATE" restart-ms 100 2>/dev/null
        if ip link show "$c" 2>/dev/null | grep -q "state UP"; then :; else
            ip link set "$c" up type can bitrate "$BITRATE" 2>/dev/null
        fi
        if ip link show "$c" 2>/dev/null | grep -q "state UP"; then
            ip link set "$c" txqueuelen "$TXQ" 2>/dev/null || true
            can_state=$(ip -d link show "$c" 2>/dev/null | grep -oE 'can state [A-Z-]+' | awk '{print $3}')
            echo "[setup-can] ✓ $c: brought up   bitrate=$BITRATE  txqueuelen=$TXQ  can_state=$can_state"
        else
            echo "[setup-can] ❌ $c: ip link up 失败（赤乌 adapter 可能已死锁，需物理拔插 USB）" >&2
            fail=1
        fi
    fi
done

if [[ $fail -ne 0 ]]; then
    echo "[setup-can] ⚠ 有 CAN 接口未拉起；检查 USB 接线或拔插适配器" >&2
    exit 1
fi
echo "[setup-can] ✓ 4 条 CAN 接口全部 UP（保留状态，未 cycle）"
exit 0
