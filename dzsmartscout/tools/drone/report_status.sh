#!/usr/bin/env bash
# 无人机状态汇报（严格只读：不启动/不停止/不修改任何东西）
export LC_ALL=C
echo "########## [1] 主机与网络 ##########"
echo "hostname   : $(hostname)"
echo "date       : $(date '+%F %T %Z')"
echo "uptime     :$(uptime -p 2>/dev/null) （启动于 $(uptime -s 2>/dev/null)）"
echo "load       :$(cat /proc/loadavg)"
echo "--- IPv4 接口 ---"
ip -4 -br addr show | grep -Ev '^(docker|veth|br-|lo)'
echo "--- 默认路由 ---"
ip route | grep -E '^default'
echo "--- 雷达网口 eno1 ---"
echo "carrier=$(cat /sys/class/net/eno1/carrier 2>/dev/null)  operstate=$(cat /sys/class/net/eno1/operstate 2>/dev/null)"
echo "--- ping 雷达 192.168.3.105 ---"
ping -c 2 -W 1 192.168.3.105 2>&1 | tail -2
echo "--- ping 小车 10.240.174.188 ---"
ping -c 2 -W 1 10.240.174.188 2>&1 | tail -2
echo "--- ping 网关 10.240.174.68 ---"
ping -c 2 -W 1 10.240.174.68 2>&1 | tail -2

echo
echo "########## [2] 关键进程 ##########"
for pat in livox_ros_driver2_node dzlink_pc2_bridge beacon.sh; do
  echo "--- $pat ---"
  ps -eo pid,etime,pcpu,pmem,cmd | grep -F "$pat" | grep -v grep || echo "(未运行)"
done

echo
echo "########## [3] ROS2 话题与频率 ##########"
source /opt/ros/humble/setup.bash
[ -f "$HOME/dzpx4/install/setup.bash" ] && source "$HOME/dzpx4/install/setup.bash"
[ -f "$HOME/dzlink_ws/install/setup.bash" ] && source "$HOME/dzlink_ws/install/setup.bash"
echo "ROS_DISTRO=$ROS_DISTRO  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset=0>}  RMW=${RMW_IMPLEMENTATION:-<default>}"
echo "--- 当前可见节点（含跨机发现的） ---"
timeout 8 ros2 node list 2>/dev/null | sort || echo "(超时)"
echo "--- livox / dzlink 话题 ---"
timeout 8 ros2 topic list -t 2>/dev/null | grep -E 'livox|dzlink' || echo "(无)"
echo "--- 发布/订阅计数 ---"
for t in /livox/lidar /livox/points /livox/imu /dzlink/seq; do
  pi=$(timeout 8 ros2 topic info "$t" 2>/dev/null | awk -F': *' '/Publisher count/{print $2}')
  si=$(timeout 8 ros2 topic info "$t" 2>/dev/null | awk -F': *' '/Subscription count/{print $2}')
  echo "$t  pub=${pi:-?}  sub=${si:-?}"
done
echo "--- 实测频率（每项 8s） ---"
for t in /livox/lidar /livox/points /livox/imu /dzlink/seq; do
  echo "[$t]"
  timeout 8 stdbuf -oL ros2 topic hz "$t" 2>&1 | tail -2
done
echo "--- 雷达点云内容抽检（/livox/lidar 单帧） ---"
timeout 10 python3 - <<'PY' 2>&1 | tail -6
import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
rclpy.init()
n = Node('dz_report_probe')
got = {}
def cb(m):
    got['m'] = m
n.create_subscription(CustomMsg, '/livox/lidar', cb, 10)
import time
t0 = time.time()
while time.time() - t0 < 8 and 'm' not in got:
    rclpy.spin_once(n, timeout_sec=0.3)
m = got.get('m')
if m is None:
    print("RESULT: 未收到点云")
else:
    age = n.get_clock().now().nanoseconds/1e9 - (m.timebase/1e9 if m.timebase else 0)
    print(f"RESULT: point_num={m.point_num} lidar_id={m.lidar_id} frame_id={m.header.frame_id} age={age:.3f}s")
n.destroy_node(); rclpy.shutdown()
PY

echo
echo "########## [4] 日志与实证文件 ##########"
ls -la /tmp/dzlink*.log /tmp/livox*.log /tmp/*.md5 2>/dev/null || echo "(无匹配日志)"
for f in /tmp/dzlink*.log; do
  [ -f "$f" ] || continue
  echo "--- $f : $(wc -l < "$f") 行 ---"
  echo "   对端 HELLO 计数 = $(grep -ac 'HELLO from duzhong' "$f" 2>/dev/null)"
  echo "   本机 HELLO 计数 = $(grep -ac 'HELLO from dzsmart' "$f" 2>/dev/null)"
  echo "   尾部 3 行:"
  tail -3 "$f" | sed 's/^/     /'
done
echo "--- 桥 STAT 最近记录 ---"
grep -ah 'STAT' /tmp/*.log 2>/dev/null | tail -4 | sed 's/^/   /' || echo "   (无 STAT 记录)"
echo "--- 驱动日志错误计数 ---"
for f in /tmp/livox*.log; do
  [ -f "$f" ] || continue
  echo "   $f: error/fail/timeout 行数 = $(grep -aicE 'error|fail|timeout' "$f")，总行数 = $(wc -l < "$f")"
  grep -aiE 'error|fail|timeout' "$f" | tail -3 | sed 's/^/     /'
done

echo
echo "########## [5] 资源 ##########"
free -h | head -2
df -h / /tmp 2>/dev/null | grep -v '^Filesystem' | sed 's/^/   /'
echo "--- 温度 ---"
for z in /sys/devices/virtual/thermal/thermal_zone*/; do
  t=$(cat "$z/temp" 2>/dev/null); n=$(cat "$z/type" 2>/dev/null)
  [ -n "$t" ] && echo "   $n = $((t/1000))C"
done

echo
echo "########## [6] 与本次链路相关的持久化配置 ##########"
echo "--- ~/.bashrc 中的 ROS 相关行 ---"
grep -nE 'ROS_|ros2|rmw|dzlink|livox' "$HOME/.bashrc" 2>/dev/null || echo "   (无)"
echo "--- 是否存在开机自启单元 ---"
ls -1 /etc/systemd/system/ 2>/dev/null | grep -iE 'livox|dzlink|ros' || echo "   (无 systemd 单元)"
echo "--- ~/dzlink_ws 目录 ---"
ls -la "$HOME/dzlink_ws" 2>/dev/null | head -8 || echo "   (不存在)"
echo
echo "########## 汇报结束 ##########"
