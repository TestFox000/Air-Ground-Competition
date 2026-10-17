#!/bin/bash
# 探测挡屏 polkit 弹窗来源 + 可用窗口操作手段，然后拉起 RViz
set +u

echo "=== A. color / polkit processes ==="
pgrep -af 'colord|gcm|polkit|color-manager|gnome-color' | head -20 || echo "(none)"

echo
echo "=== B. X window/input tools ==="
command -v xdotool xte xwininfo xprop wmctrl xvkbd xinput xrandr
echo "-- libXtst / libX11 --"
ls -la /usr/lib/aarch64-linux-gnu/libXtst.so.6 /usr/lib/aarch64-linux-gnu/libX11.so.6 2>/dev/null

echo
echo "=== C. window manager + clients on :1 ==="
DISPLAY=:1 xprop -root _NET_SUPPORTING_WM_CHECK 2>&1 | head -2
DISPLAY=:1 xprop -root _NET_CLIENT_LIST 2>&1 | head -2

echo
echo "=== D. launch rviz on :1 ==="
bash /tmp/start_rviz_display.sh
