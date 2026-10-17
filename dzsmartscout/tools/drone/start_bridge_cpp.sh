#!/bin/bash
# 无人机侧：后台启动 C++ 点云桥（CustomMsg -> PointCloud2）
# 幂等：已在运行则不重复启动。用法：bash /tmp/start_bridge_cpp.sh
set +u

PAT='dzlink_pc2_bridge/lib/dzlink_pc2_bridge/pc2_bridge'
if pgrep -f "$PAT" >/dev/null 2>&1; then
  echo "ALREADY_RUNNING pid=$(pgrep -f "$PAT" | tr '\n' ' ')"
else
  cat > /tmp/start_bridge_inner.sh <<'EOS'
#!/bin/bash
source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"
source "$HOME/dzlink_ws/install/setup.bash"
exec ros2 run dzlink_pc2_bridge pc2_bridge
EOS
  chmod +x /tmp/start_bridge_inner.sh
  setsid nohup /tmp/start_bridge_inner.sh > /tmp/dzlink_bridge_cpp.log 2>&1 < /dev/null &
  sleep 8
fi

echo "=== 1. bridge process ==="
pgrep -af pc2_bridge | grep -v pgrep || echo "NOT RUNNING"

echo
echo "=== 2. bridge log ==="
tail -15 /tmp/dzlink_bridge_cpp.log 2>/dev/null

echo
echo "=== 3. topics ==="
source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"
source "$HOME/dzlink_ws/install/setup.bash"
ros2 topic list -t 2>/dev/null | grep -E 'livox|dzlink'

echo
echo "=== 4. /livox/points rate (12s) ==="
timeout 12 stdbuf -oL ros2 topic hz /livox/points 2>&1 | head -4
