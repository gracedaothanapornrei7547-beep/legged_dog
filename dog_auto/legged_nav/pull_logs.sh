#!/usr/bin/env bash
# 把狗上每次运行生成的 log/trot_<时间戳>.{log,bin} 拉回本地 log/ 供查看。
# 控制器跑在狗上 → 日志生在狗的 ~/Desktop/legged_control_lite/log/；
# 本机用这个脚本拉过来（只拉 trot_*，不动 colcon 的 build_*/test_* 日志）。
set -euo pipefail

DOG="rc@RC.local"
DOG_LOG="~/Desktop/legged_control_lite/log/"
LOCAL_LOG="$HOME/legged_control/legged_control_lite/log/"

mkdir -p "$LOCAL_LOG"
rsync -avz \
  --include='trot_*.log' --include='trot_*.bin' --exclude='*' \
  -e "ssh -o BatchMode=yes" \
  "$DOG":"$DOG_LOG" "$LOCAL_LOG"

echo ""
echo "最新 5 个文本日志:"
ls -t "$LOCAL_LOG"trot_*.log 2>/dev/null | head -5 || echo "  (暂无)"
echo ""
echo "看数据:"
echo "  文本:   tail -40 $LOCAL_LOG<最新>.log    (或 grep '★窗口' / 'Estimator:')"
echo "  逐帧图: python3 src/motor_test/tools/parse_telem.py $LOCAL_LOG<最新>.bin --plot"
