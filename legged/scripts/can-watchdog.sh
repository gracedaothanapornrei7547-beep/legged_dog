#!/bin/bash

# can-watchdog.sh —— 后台守护进程，保证 4 条 CAN 接口一直 UP
#
# 设计逻辑：
#   每 2 秒检查一次 4 条 CAN：
#     - link state == UP    → 啥也不做（绝不 cycle，防赤乌 adapter 死锁）
#     - link state == DOWN  → 立刻 ip link up
#     - 接口不存在            → 等下次轮询（USB 还没枚举完）
#
#   核心原则：UP 的接口绝不动。这是为了适配 赤乌科技 USB-CAN 反复 cycle 后死锁的 bug。
#   只有"曾经 UP 然后被外部 down 了"或者"开机后还没起来"的情况会触发 up。
#
# 状态变化日志：仅在 DOWN→UP / UP→DOWN 时打印（避免每 2s 重复打印）。
# journal 看：  journalctl -u can-watchdog.service -f

set -u
CHANS=(can5 can3 can7 can6)  # ★2026-07-10 灵足02: FL=can5 FR=can3 BL=can7 BR=can6
BITRATE=1000000
TXQ=1000
INTERVAL=2

# 记录上一轮各接口的 state，只在变化时打日志
declare -A prev_state
for c in "${CHANS[@]}"; do prev_state["$c"]="INIT"; done

# 优雅退出：systemctl stop 时收到 SIGTERM
trap 'echo "[$(date "+%F %T")] [can-watchdog] received SIGTERM, exiting"; exit 0' TERM INT

echo "[$(date "+%F %T")] [can-watchdog] started, interval=${INTERVAL}s, target chans=${CHANS[*]}"

while true; do
    for c in "${CHANS[@]}"; do
        if ! ip link show "$c" &>/dev/null; then
            cur="MISSING"
        else
            cur=$(ip link show "$c" | grep -oE 'state [A-Z]+' | awk '{print $2}')
        fi

        # 状态变化打日志
        if [[ "${prev_state[$c]}" != "$cur" ]]; then
            echo "[$(date "+%F %T")] [can-watchdog] $c state: ${prev_state[$c]} → $cur"
            prev_state["$c"]="$cur"
        fi

        # 仅 DOWN 状态需要拉起来（UP 一律不动）
        if [[ "$cur" == "DOWN" ]] || [[ "$cur" == "UNKNOWN" && "${prev_state[$c]}" == "INIT" ]]; then
            # 2026-05-30: 优先 restart-ms 100,老固件不支持 fallback 不带
            ip link set "$c" up type can bitrate "$BITRATE" restart-ms 100 2>/dev/null || \
                ip link set "$c" up type can bitrate "$BITRATE" 2>/dev/null
            if ip link show "$c" 2>/dev/null | grep -q "state UP"; then
                ip link set "$c" txqueuelen "$TXQ" 2>/dev/null || true
                echo "[$(date "+%F %T")] [can-watchdog] $c: brought UP (bitrate=$BITRATE txqueuelen=$TXQ)"
                prev_state["$c"]="UP"
            else
                echo "[$(date "+%F %T")] [can-watchdog] ⚠ $c: ip link up 失败（adapter 可能死锁，需物理拔插）"
            fi
        fi
    done
    sleep "$INTERVAL"
done
