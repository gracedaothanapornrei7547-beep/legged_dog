#!/bin/bash

# lock-can-by-usb.sh - 把 8 个 candleLight (gs_usb) can 接口按 USB 物理口锁名
#
# 为什么不用 udev: udev NAME= 对 net device 在多设备并发枚举时撞 EEXIST,
# rename 静默失败 (2026-05-30 实测: rule 匹配正确,但 8 设备 7 个 rename 没成)。
# 用 ip link 两段式 rename: 先全转 cantmp_*, 再按 USB 口转目标名,绕开碰撞。
#
# 运行时机: modprobe gs_usb 后, setup-can.sh / can-reset.sh / 启动时调一次。
# 需要 root (ip link set name)。

set -u

# USB 物理口 -> 目标 can 名
declare -A USB_TO_CAN=(
  ["3-1.4"]="can5"   # FL
  ["3-1.3"]="can3"   # FR
  ["3-2.4"]="can7"   # BL
  ["3-2.3"]="can6"   # BR
  ["3-1.1"]="can0"   # 空
  ["3-1.2"]="can1"   # 空
  ["3-2.1"]="can2"   # 空
  ["3-2.2"]="can4"   # 空
)

if [[ $EUID -ne 0 ]]; then
    echo "[lock-can] 需要 root" >&2
    exec sudo "$0" "$@"
fi

# 收集当前 can / canu 接口 (跳过已是 cantmp_ 的)
mapfile -t IFACES < <(ls /sys/class/net/ 2>/dev/null | grep -E "^(can[0-9]+|canu[0-9]+)$" || true)
if [[ ${#IFACES[@]} -eq 0 ]]; then
    echo "[lock-can] 没找到 can 接口,退" >&2
    exit 1
fi

echo "[lock-can] 阶段 1: 全部 down + 转成 cantmp_ 临时名"
declare -A TMP_TO_USB
for c in "${IFACES[@]}"; do
    dev=/sys/class/net/$c/device
    if [[ ! -e "$dev" ]]; then
        echo "[lock-can] ⚠ $c 没有 device 链接,跳"
        continue
    fi
    usb=$(basename $(readlink -f "$dev" | sed 's|/[^/]*:[0-9.]*$||'))
    tmp="cantmp_${usb//./_}"
    ip link set "$c" down 2>/dev/null || true
    if ip link set "$c" name "$tmp" 2>/dev/null; then
        TMP_TO_USB[$tmp]=$usb
        echo "[lock-can]   $c (usb=$usb) -> $tmp"
    else
        echo "[lock-can] ❌ $c -> $tmp 失败" >&2
    fi
done

echo "[lock-can] 阶段 2: cantmp_ -> 目标 can 名"
fail=0
for tmp in "${!TMP_TO_USB[@]}"; do
    usb="${TMP_TO_USB[$tmp]}"
    target="${USB_TO_CAN[$usb]:-}"
    if [[ -z "$target" ]]; then
        echo "[lock-can] ⚠ $tmp (usb=$usb) 没在表里,留 $tmp 名" >&2
        continue
    fi
    if ip link set "$tmp" name "$target" 2>/dev/null; then
        echo "[lock-can]   $tmp -> $target  (usb=$usb)"
    else
        echo "[lock-can] ❌ $tmp -> $target 失败" >&2
        fail=1
    fi
done

echo "[lock-can] 最终状态:"
for c in can0 can1 can2 can3 can4 can5 can6 can7; do
    if [[ -e /sys/class/net/$c ]]; then
        usb=$(basename $(readlink -f /sys/class/net/$c/device | sed 's|/[^/]*:[0-9.]*$||'))
        target_expected="${USB_TO_CAN[$usb]:-?}"
        if [[ "$target_expected" == "$c" ]]; then
            printf "[lock-can]   ✓ %-5s usb=%s\n" "$c" "$usb"
        else
            printf "[lock-can]   ❌ %-5s usb=%s (期望该 USB->%s)\n" "$c" "$usb" "$target_expected"
            fail=1
        fi
    else
        printf "[lock-can]   ⚠ %-5s MISSING\n" "$c"
    fi
done

if [[ $fail -eq 0 ]]; then
    echo "[lock-can] ✓ 8 个接口全部按 USB 口锁好"
    exit 0
else
    echo "[lock-can] ⚠ 有错位,看上面" >&2
    exit 1
fi
