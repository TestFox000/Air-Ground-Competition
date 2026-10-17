#!/bin/bash
# VNC 点云显示的最终活体验收（进程 + 速率 + 全分辨率截图）
set +u

echo "=== 1. 关键进程 ==="
pgrep -af livox_ros_driver2_node | head -1 || echo "driver MISSING"
pgrep -af pc2_bridge | grep -v pgrep | head -1 || echo "bridge MISSING"
pgrep -af rviz2 | grep -v pgrep | head -1 || echo "rviz MISSING"
pgrep -af Xtigervnc | head -1 || echo "vnc MISSING"
pgrep -af xfce4-screensaver >/dev/null && echo "screensaver: RUNNING (可能再挡屏)" || echo "screensaver: not running"

echo
echo "=== 2. VNC 监听 ==="
ss -lntp | grep 5901 || echo "(5901 not listening)"

echo
echo "=== 3. 话题与速率 ==="
source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"
source "$HOME/dzlink_ws/install/setup.bash"
ros2 topic list -t 2>/dev/null | grep -E 'livox|dzlink'
timeout 10 stdbuf -oL ros2 topic hz /livox/points 2>&1 | head -3

echo
echo "=== 4. 桥 STAT（subs_points=RViz 订阅者） ==="
tail -2 /tmp/dzlink_bridge_cpp.log 2>/dev/null

echo
echo "=== 5. 全分辨率截图 :1 ==="
DISPLAY=:1 timeout 25 xwd -root -silent -out /tmp/vnc1_full.xwd 2>&1 | head -2
python3 /tmp/xwd2png.py /tmp/vnc1_full.xwd /tmp/vnc1_full.png 1920
ls -la /tmp/vnc1_full.png

echo
echo "=== 6. rviz 窗口几何 ==="
DISPLAY=:1 timeout 10 xwininfo -root -tree 2>/dev/null | grep -i 'RViz' | head -3
echo "=== DONE ==="
