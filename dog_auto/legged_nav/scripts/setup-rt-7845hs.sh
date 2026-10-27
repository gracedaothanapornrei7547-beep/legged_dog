#!/bin/bash
# setup-rt-7845hs.sh —— 7845HS 任务赛目标机的实时(RT)调优一键脚本
# =============================================================================
# 【目的】让 nav_quad(狗 RT 主进程, 500Hz)跑稳: 隔离它固定占用的 CPU 核、锁频/关深
#   C-state、装 rtprio/memlock limits、关 irqbalance。搬机到 7845HS 后跑一次。
#
# 【为什么隔离这几个核】nav_quad 的 CPU 亲和是写死在源码里的(参考狗子代码):
#     - 主线程      -> CPU3   (nav_quad_main.cpp: pin_main_to_cpu3, SCHED_FIFO 80)
#     - 4 条 CAN rx -> CPU1-2 (can_bridge.cpp::connect, SCHED_FIFO 70)
#     - CPU0 留系统/sshd
#   代码实际只用 CPU0-3。外部 taskset/cset 移不动这些线程(进程内 sched_setaffinity
#   会覆盖), 所以正确做法是"把代码已经在用的 CPU1/2/3 (+SMT 兄弟)从内核调度隔离出来",
#   让 nav 全栈/系统别抢, 把 500Hz 抖动压到最低。
#   注: 若日后改代码把亲和挪到别的核(路线 B), 把下面 RT_CORES 改成新核即可。
#
# 【做什么】(幂等, 可反复跑)
#   1. 探测本机 CPU 拓扑, 算出要隔离的核 = CPU1/2/3 + 它们的 SMT 兄弟
#   2. 装 /etc/security/limits.d/99-realtime.conf (rtprio 99 + memlock unlimited)
#   3. 所有核设 performance governor
#   4. 关 irqbalance
#   5. 打印要加进 GRUB 的 cmdline(isolcpus 只能开机时设, 需 reboot 生效)
#      —— 默认只打印; 加 --write-grub 才自动写入 /etc/default/grub(带备份)
#   6. 打印验证命令
#
# 【用法】
#   sudo ./setup-rt-7845hs.sh              # 应用运行时项 + 打印 GRUB 建议(不改 GRUB)
#   sudo ./setup-rt-7845hs.sh --write-grub # 额外自动把 cmdline 追加进 /etc/default/grub
# =============================================================================
set -u

WRITE_GRUB=0
[[ "${1:-}" == "--write-grub" ]] && WRITE_GRUB=1

if [[ $EUID -ne 0 ]]; then echo "[rt] 需要 root, 用 sudo 重跑"; exec sudo "$0" "$@"; fi
RUN_USER="${SUDO_USER:-$USER}"

# ---- 1. 拓扑: 要隔离的 RT 核 = CPU1,2,3 + 它们的 SMT 兄弟 ----
RT_CORES=(1 2 3)   # 与代码写死的亲和一致(main=CPU3, rx=CPU1-2); 改了代码就改这里
declare -A ISO
for c in "${RT_CORES[@]}"; do
    sib=/sys/devices/system/cpu/cpu$c/topology/thread_siblings_list
    if [[ -r "$sib" ]]; then
        # 形如 "1,9" (SMT 开) 或 "1" (SMT 关) 或 "1-1"
        for tok in $(tr ',' ' ' < "$sib"); do
            if [[ "$tok" == *-* ]]; then
                for ((x=${tok%-*}; x<=${tok#*-}; x++)); do ISO[$x]=1; done
            else ISO[$tok]=1; fi
        done
    else ISO[$c]=1; fi
done
NPROC=$(nproc --all)
declare -A HK
for ((i=0; i<NPROC; i++)); do [[ -z "${ISO[$i]:-}" ]] && HK[$i]=1; done
ISO_LIST=$(printf '%s\n' "${!ISO[@]}" | sort -n | paste -sd, -)
HK_LIST=$(printf '%s\n' "${!HK[@]}" | sort -n | paste -sd, -)

echo "[rt] CPU 总数=$NPROC"
echo "[rt] 隔离(RT)核 = $ISO_LIST   (nav_quad 主线程 CPU3 + CAN rx CPU1-2, 连 SMT 兄弟)"
echo "[rt] housekeeping = $HK_LIST   (系统/IRQ/nav 全栈 Point-LIO/Nav2/GICP)"
echo "[rt] 当前已隔离(isolcpus 生效中) = $(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo '(空)')"
[[ "$ISO_LIST" == "1,2,3" ]] && echo "[rt] 提示: 没测到 SMT 兄弟(可能是 4 核机/关了 SMT), 隔离 1,2,3 也对。"
echo ""

# ---- 2. limits.d rtprio/memlock ----
LIM=/etc/security/limits.d/99-realtime.conf
cat > "$LIM" <<EOF
# 由 setup-rt-7845hs.sh 生成: SCHED_FIFO(rtprio) + mlockall(memlock)。
# nav_quad 的 try_enable_rt() 需要它, 否则降级普通调度(500Hz 抖动大)。
@dialout - rtprio 99
@dialout - memlock unlimited
$RUN_USER - rtprio 99
$RUN_USER - memlock unlimited
EOF
chmod 0644 "$LIM"
echo "[rt] ✓ 写 $LIM (@dialout + $RUN_USER: rtprio 99 + memlock unlimited) — 重登录生效"
if ! id -nG "$RUN_USER" 2>/dev/null | grep -qw dialout; then
    usermod -aG dialout "$RUN_USER" && echo "[rt] ✓ 把 $RUN_USER 加进 dialout 组(重登录生效)"
fi

# ---- 3. performance governor + 关深 C-state 提示 ----
gov=0
for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    [[ -w "$g" ]] && echo performance > "$g" 2>/dev/null && gov=1
done
command -v cpupower >/dev/null 2>&1 && cpupower frequency-set -g performance >/dev/null 2>&1 || true
[[ $gov -eq 1 ]] && echo "[rt] ✓ 所有核 governor=performance" \
                 || echo "[rt] ⚠ 没能设 governor(无 cpufreq? 也可在 BIOS 关节能/Cool'n'Quiet)"

# ---- 4. 关 irqbalance ----
if systemctl list-unit-files 2>/dev/null | grep -q '^irqbalance'; then
    systemctl disable --now irqbalance >/dev/null 2>&1 && echo "[rt] ✓ 关闭 irqbalance(避免把 IRQ 迁到隔离核)"
fi

# ---- 5. GRUB cmdline ----
CMDLINE="isolcpus=$ISO_LIST nohz_full=$ISO_LIST rcu_nocbs=$ISO_LIST irqaffinity=$HK_LIST processor.max_cstate=1"
echo ""
echo "[rt] ★ 隔离核需要 GRUB cmdline(reboot 才生效)。在 /etc/default/grub 的"
echo "      GRUB_CMDLINE_LINUX_DEFAULT 里追加:"
echo "        $CMDLINE"
echo "      然后: sudo update-grub && sudo reboot"

if [[ $WRITE_GRUB -eq 1 ]]; then
    GRUBF=/etc/default/grub
    if grep -q 'isolcpus=' "$GRUBF" 2>/dev/null; then
        echo "[rt] ⚠ $GRUBF 里已有 isolcpus=..., 不自动改, 请手动核对上面的 cmdline"
    else
        cp "$GRUBF" "$GRUBF.bak.$(date +%Y%m%d%H%M%S)"
        sed -i -E "s|^(GRUB_CMDLINE_LINUX_DEFAULT=\")(.*)(\")|\1\2 $CMDLINE\3|" "$GRUBF"
        echo "[rt] ✓ 已追加进 $GRUBF (备份 $GRUBF.bak.*)。跑: sudo update-grub && sudo reboot"
    fi
fi

# ---- 6. 验证 ----
echo ""
echo "[rt] 验证(reboot 后):"
echo "  cat /sys/devices/system/cpu/isolated                  # 应显示 $ISO_LIST"
echo "  ps -eLo psr,rtprio,comm | grep nav_quad               # 主线程 psr=3, rx psr=1/2, rtprio 80/70"
echo "  cyclictest -p80 -a3 -t1 -m -D 60                      # 隔离前后对比抖动(sudo apt install rt-tests)"
echo "  # 以及 nav_quad 跑完自带 print_summary(): 看实际控制率/slot 被挤比例, 确认 500Hz 达成度"
echo "[rt] 完成(GRUB 部分需 reboot)。"
