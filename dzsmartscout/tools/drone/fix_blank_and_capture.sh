#!/bin/bash
# 关掉 :1 的黑屏/DPMS，保持点云可见；然后真拍一张
set +u

echo "=== 1. X screen-saver / DPMS off on :1 ==="
DISPLAY=:1 timeout 10 xset s off 2>&1
DISPLAY=:1 timeout 10 xset s noblank 2>&1
DISPLAY=:1 timeout 10 xset -dpms 2>&1
DISPLAY=:1 timeout 10 xset q 2>&1 | grep -A2 -i 'screen saver' | head -6

echo
echo "=== 2. xfconf: 关闭自动锁定/黑屏（--create，属性此前不存在）==="
for p in /saver/enabled /saver/idle-activation/enabled /lock/enabled; do
  xfconf-query -c xfce4-screensaver -p "$p" -n -t bool -s false --create 2>&1
done
echo "-- verify --"
xfconf-query -c xfce4-screensaver -p /saver/enabled 2>&1
xfconf-query -c xfce4-screensaver -p /lock/enabled 2>&1

echo
echo "=== 3. fullscreen windows on :1 (谁还挡着屏) ==="
DISPLAY=:1 timeout 10 xwininfo -root -tree 2>/dev/null | grep -E '1920x1200|1900x11' | head -10

echo
echo "=== 4. capture :1 ==="
bash /tmp/capture_vnc1.sh rviz2

echo
echo "=== 5. screensaver process state ==="
pgrep -af xfce4-screensaver || echo "(screensaver not running - good)"
echo "=== DONE ==="
