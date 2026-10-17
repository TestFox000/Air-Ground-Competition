#!/bin/bash
# 真正抓取 VNC 显示 :1 的截图并转 PNG（供外部验证）
# 用法：bash /tmp/capture_vnc1.sh [输出名]
set +u
NAME="${1:-vnc1}"
OUT="/tmp/${NAME}.png"

DISPLAY=:1 timeout 25 xwd -root -silent -out "/tmp/${NAME}.xwd" 2>&1 | head -3
ls -la "/tmp/${NAME}.xwd" 2>/dev/null || echo "(xwd failed)"

python3 /tmp/xwd2png.py "/tmp/${NAME}.xwd" "$OUT" 1400
ls -la "$OUT" 2>/dev/null || echo "(png failed)"

echo
echo "=== windows on :1 (tree, depth 2) ==="
DISPLAY=:1 timeout 10 xwininfo -root -tree 2>/dev/null | head -28

echo
echo "=== rviz log tail ==="
tail -12 /tmp/dzlink_rviz.log 2>/dev/null

echo
echo "=== bridge STAT (subs_points should be >=1 now) ==="
grep -c STAT /tmp/dzlink_bridge_cpp.log 2>/dev/null
tail -3 /tmp/dzlink_bridge_cpp.log 2>/dev/null
