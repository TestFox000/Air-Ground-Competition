#!/bin/bash
# 探测无人机 VNC 桌面与可视化能力（只读，不改任何东西）
# 用法：bash /tmp/probe_vnc_gui.sh
set +u

echo "=== 1. display :1 geometry ==="
DISPLAY=:1 xrandr 2>&1 | head -4

echo
echo "=== 2. desktop session processes on :1 ==="
pgrep -af 'gnome-session|gnome-shell|xfce4-session|xfwm4|metacity|marco|mutter|pcmanfm|nautilus|lxqt|openbox' | head -12

echo
echo "=== 3. screenshot / X tools ==="
command -v import scrot xwd ffmpeg gnome-screenshot xdotool wmctrl xrandr

echo
echo "=== 4. GL capability on :1 ==="
if command -v glxinfo >/dev/null; then
  DISPLAY=:1 glxinfo -B 2>&1 | head -12
else
  echo "glxinfo MISSING (mesa-utils not installed)"
fi

echo
echo "=== 5. ROS + rviz2 ==="
source /opt/ros/humble/setup.bash
echo "rviz2_bin=$(command -v rviz2)"
echo "rviz_pkgs=$(ros2 pkg list 2>/dev/null | grep -c -e rviz)"
ls -d /opt/ros/humble/share/rviz* 2>/dev/null

echo
echo "=== 6. python GUI libs ==="
python3 - <<'EOF'
for m in ['numpy', 'cv2', 'matplotlib', 'tkinter', 'PIL']:
    try:
        __import__(m)
        print(m, 'OK')
    except Exception as e:
        print(m, 'MISSING', type(e).__name__)
EOF

echo
echo "=== 7. current ROS nodes / livox topics ==="
ros2 node list 2>/dev/null
ros2 topic list -t 2>/dev/null | grep -iE 'livox|dzlink' || echo '(no livox/dzlink topics)'

echo
echo "=== 8. lidar + bridge processes ==="
pgrep -af livox_ros_driver2_node || echo '(driver not running)'
pgrep -af dzlink || echo '(bridge not running)'

echo
echo "=== 9. vnc-related systemd units ==="
systemctl list-units --type=service --all 2>/dev/null | grep -i -e vnc -e vncserver || echo '(none)'

echo
echo "=== 10. vnc xstartup ==="
cat "$HOME/.vnc/xstartup" 2>/dev/null || echo '(no ~/.vnc/xstartup)'

echo "=== DONE ==="
