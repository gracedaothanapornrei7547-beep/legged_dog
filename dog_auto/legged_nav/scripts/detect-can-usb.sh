#!/bin/bash
# detect-can-usb.sh —— 搬机后探测本机 can 接口 <-> USB 物理口映射, 生成可粘贴的
#   lock 表 + udev 规则骨架。解决"换主机(N150->7845HS)后 USB 口路径会变"的适配问题。
# =============================================================================
# 【背景 · 参考狗子代码】
#   lock-can-by-usb.sh / 70-candlelight-can.rules / hardware_manager.hpp 的
#   LEG_CAN_CHANNEL 都按 N150 的 USB 口写死: 3-1.1=can0(FR) 3-1.2=can1(FL)
#   3-2.3=can6(BL) 3-2.4=can7(BR)。换到 7845HS, USB 控制器/hub 编号不同, 插同样的板子
#   物理口路径几乎一定不一样 → 必须重测。映射错 = 命令发到错误电机 = 上电瞬间翻车。
#
# 【本脚本只读 /sys, 不改任何东西】
#   1. 列出当前每个 can 接口解析出的 USB 物理口 + 驱动 + can state
#   2. 和 N150 期望口对比, 标出变没变
#   3. 打印可粘贴的 USB_TO_CAN 填空模板 + udev 规则骨架
#   4. 提示下一步: 用 function_can_test 逐条 bus 靠电机 id 确认"哪条腿在哪条 bus"
#
# 【用法】 ./detect-can-usb.sh     (不需要 root)
# =============================================================================
set -u

# N150 期望(来自 70-candlelight-can.rules), 仅用于对比
declare -A N150=( [3-1.1]="can0(FR)" [3-1.2]="can1(FL)" [3-2.3]="can6(BL)" [3-2.4]="can7(BR)" )

port_of() {  # $1=iface -> USB 物理口(与 lock-can-by-usb.sh 完全同一算法)
    local dev=/sys/class/net/$1/device
    [[ -e "$dev" ]] || { echo "-"; return; }
    basename "$(readlink -f "$dev" | sed 's|/[^/]*:[0-9.]*$||')"
}

mapfile -t IFACES < <(ls /sys/class/net/ 2>/dev/null | grep -E "^(can[0-9]+|canu[0-9]+|cantmp_.*)$" || true)
if [[ ${#IFACES[@]} -eq 0 ]]; then
    echo "[detect] 没找到 can 接口。先插好 USB-CAN 板 / modprobe gs_usb 再跑。" >&2
    exit 1
fi

echo "[detect] 当前 can 接口 <-> USB 物理口:"
printf "  %-12s %-10s %-8s %s\n" IFACE USB口 驱动 CAN状态
declare -A FOUND   # usb -> iface
for c in "${IFACES[@]}"; do
    usb=$(port_of "$c")
    drv="-"; [[ -L /sys/class/net/$c/device/driver ]] && drv=$(basename "$(readlink /sys/class/net/$c/device/driver)")
    st=$(ip -d link show "$c" 2>/dev/null | grep -oE 'can state [A-Z-]+' | awk '{print $3}'); st=${st:-?}
    printf "  %-12s %-10s %-8s %s\n" "$c" "$usb" "$drv" "$st"
    [[ "$usb" != "-" ]] && FOUND[$usb]=$c
done

echo ""
echo "[detect] 与 N150 期望 USB 口对比:"
changed=0
for p in "${!N150[@]}"; do
    if [[ -n "${FOUND[$p]:-}" ]]; then
        echo "  ✓ USB $p 仍在(当前 ${FOUND[$p]}; N150 上是 ${N150[$p]})"
    else
        echo "  ✗ USB $p 不在了 —— 本机 USB 口和 N150 不同, 需重定映射"; changed=1
    fi
done
[[ $changed -eq 1 ]] && echo "  → 结论: 这不是 N150 的拓扑, 按下面模板重定映射。" \
                     || echo "  → 结论: USB 口与 N150 一致, 现有映射可直接用。"

echo ""
echo "[detect] 检测到的 USB 口(有 CAN 板的): ${!FOUND[*]}"
echo ""
echo "[detect] === 填空模板 A: lock-can-by-usb.sh 的 USB_TO_CAN(把 <USB口> 换成上面检测到的) ==="
cat <<'TPL'
  declare -A USB_TO_CAN=(
    ["<USB口>"]="can0"   # FR  确认: 该 bus 上电机 id = 1/2/3
    ["<USB口>"]="can1"   # FL  确认: 电机 id = 4/5/6
    ["<USB口>"]="can6"   # BL  确认: 电机 id = 7/8/9
    ["<USB口>"]="can7"   # BR  确认: 电机 id = 10/11/12
    # (空口可不填或随意给 can2/3/4/5)
  )
TPL
echo ""
echo "[detect] === 填空模板 B: 70-candlelight-can.rules(同样把 <USB口> 换掉) ==="
cat <<'TPL'
  SUBSYSTEM=="net", ACTION=="add", KERNEL=="can*", KERNELS=="<USB口FR>", NAME="can0"
  SUBSYSTEM=="net", ACTION=="add", KERNEL=="can*", KERNELS=="<USB口FL>", NAME="can1"
  SUBSYSTEM=="net", ACTION=="add", KERNEL=="can*", KERNELS=="<USB口BL>", NAME="can6"
  SUBSYSTEM=="net", ACTION=="add", KERNEL=="can*", KERNELS=="<USB口BR>", NAME="can7"
TPL

echo ""
echo "[detect] 下一步 —— 确认「哪条 bus 是哪条腿」只能靠电机 id, 不能猜:"
echo "  1) 电机上电, 逐条 bus 跑:  ros2 run legged_nav function_can_test <fl|fr|bl|br> private step"
echo "     看哪条 bus 的电机 id 应答(FR=1/2/3 FL=4/5/6 BL=7/8/9 BR=10/11/12)。"
echo "  2) 据此把 USB 口 <-> 腿 <-> canN 填进这 4 处并保持一致:"
echo "       - scripts/lock-can-by-usb.sh 的 USB_TO_CAN"
echo "       - scripts/70-candlelight-can.rules"
echo "       - src/legged_nav/include/motor_test/task/hardware_manager.hpp 的 LEG_CAN_CHANNEL(★权威)"
echo "       - (可选) src/legged_nav/include/motor_test/driver/usb_can.hpp 的 LEG_BINDINGS"
echo "  3) colcon build; 再用 function_can_test 逐腿点动确认「命令哪条腿=动哪条腿」, 才整机上电。"
echo "  提示: nav_quad 启动的 preflight() 会查 canN<->USB 一致性, 但前提是上面映射表本身是对的。"
