#!/bin/bash
# 在无人机上启动 Livox MID-360 驱动（脱离 SSH 会话，断开连接不停）
# 幂等：已在运行则不重复启动。用法：bash /tmp/start_lidar_bg.sh
# 注意：必须 set +u —— ROS 的 setup.bash 会引用未绑定变量（AMENT_TRACE_SETUP_FILES 等），
#       set -u 会让 source 直接失败，末尾的速率检查就白跑了（2026-10-07 实测踩到）。
set +u

# 幂等：已在运行则不重复启动。
# 注意：内核把 comm 名截断到 15 字符（livox_ros_drive），所以 pgrep -x 全名永远匹配不到；
#       必须用 pgrep -f 匹配完整路径片段。
# 2026-10-07 实测踩坑：pgrep -f 会偶发返回 0（成功）但输出为空 —— 那是瞬时自匹配，
# 结果是打出 "ALREADY_RUNNING pid="（空 pid）的假阳性，雷达明明没起却说已在运行，
# 而下游只看到 /livox/points 没数据，会误判成雷达故障。
# 修正：必须真的拿到非空 pid 才算在跑。
LIDAR_PAT='livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'
LIDAR_PIDS=$(pgrep -f "$LIDAR_PAT" 2>/dev/null | tr '\n' ' ')
if [ -n "${LIDAR_PIDS// /}" ]; then
  echo "ALREADY_RUNNING pid=$LIDAR_PIDS"
  exit 0
fi

cat > /tmp/start_livox_inner.sh <<'EOS'
#!/bin/bash
source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"
exec ros2 launch livox_ros_driver2 msg_MID360_launch.py
EOS
chmod +x /tmp/start_livox_inner.sh

setsid nohup /tmp/start_livox_inner.sh > /tmp/livox_launch.log 2>&1 < /dev/null &
sleep 12

echo "=== 1. driver process ==="
pgrep -af livox_ros_driver2_node | head -3 || echo "NOT RUNNING"

echo
echo "=== 2. launch log tail ==="
tail -12 /tmp/livox_launch.log

echo
echo "=== 3. init result ==="
grep -c "Init lds lidar success" /tmp/livox_launch.log

echo
echo "=== 4. error/fail lines ==="
grep -icE "error|fail" /tmp/livox_launch.log

echo
echo "=== 5. /livox/lidar publish rate (12s sample) ==="
source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"
timeout 12 ros2 topic hz /livox/lidar 2>&1 | head -4

echo
echo "=== 6. topic types ==="
source /opt/ros/humble/setup.bash
source "$HOME/dzpx4/install/setup.bash"
ros2 topic list -t 2>/dev/null | grep -i livox
