#!/bin/bash
# 无人机侧：在 VNC 桌面 (:1) 上打开 RViz2 显示 Livox MID-360 点云
# 用法：bash /tmp/start_rviz_display.sh
set +u

pkill -f 'rviz2' 2>/dev/null
sleep 2

cat > /tmp/dzlink_rviz_inner.sh <<'EOS'
#!/bin/bash
source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"
source "$HOME/dzlink_ws/install/setup.bash"
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
exec rviz2 -d /tmp/dzlink_view.rviz
EOS
chmod +x /tmp/dzlink_rviz_inner.sh
setsid nohup /tmp/dzlink_rviz_inner.sh > /tmp/dzlink_rviz.log 2>&1 < /dev/null &
sleep 18

echo "=== 1. rviz process ==="
pgrep -af rviz2 | grep -v pgrep | head -3 || echo "NOT RUNNING"

echo
echo "=== 2. rviz log (tail 30) ==="
tail -30 /tmp/dzlink_rviz.log 2>/dev/null

echo
echo "=== 3. windows on :1 ==="
DISPLAY=:1 timeout 10 xwininfo -root -children 2>/dev/null | grep -v '^ *$' | tail -15
