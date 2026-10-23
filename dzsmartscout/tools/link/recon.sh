#!/usr/bin/env bash
# 无人机<->小车 直连侦察（只读，不做任何修改）
# 用法: bash recon.sh [对端IP]
PEER="${1:-}"
echo "### host=$(hostname) user=$(whoami) time=$(date -Is)"
echo "### kernel: $(uname -sr)"
[ -r /etc/os-release ] && { . /etc/os-release; echo "### os: ${PRETTY_NAME}"; }
echo "### uptime: $(uptime -p 2>/dev/null || uptime)"
echo
echo "--- IPv4 地址 ---"
ip -4 addr show | awk '/^[0-9]+:/{ifc=$2} /inet /{print ifc, $2}'
echo
echo "--- 默认路由 ---"
ip route show default
echo
echo "--- /opt/ros 安装 ---"
ls -d /opt/ros/*/ 2>/dev/null || echo "(无 /opt/ros)"
echo
echo "--- ROS 环境(登录shell) ---"
bash -lc 'for d in /opt/ros/*/setup.bash; do [ -r "$d" ] && source "$d"; done
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset(默认0)}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset(默认rmw_fastrtps_cpp)}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}"
echo "ros2=$(command -v ros2 || echo missing)"' 2>&1 | sed 's/^/  /'
echo
echo "--- ROS2 当前节点 ---"
bash -lc 'for d in /opt/ros/*/setup.bash; do [ -r "$d" ] && source "$d"; done
timeout 12 ros2 node list 2>&1 | head -25' 2>&1 | sed 's/^/  /'
echo
echo "--- ROS2 当前话题(前15) ---"
bash -lc 'for d in /opt/ros/*/setup.bash; do [ -r "$d" ] && source "$d"; done
timeout 12 ros2 topic list 2>&1 | head -15' 2>&1 | sed 's/^/  /'
echo
echo "--- 工作区 ---"
for w in "$HOME/dzrobot/install/setup.bash" "$HOME/dzpx4/install/setup.bash" "$HOME/dzuav/install/setup.bash"; do
  if [ -r "$w" ]; then echo "  [有] $w"; else echo "  [无] $w"; fi
done
echo
if [ -n "$PEER" ]; then
  echo "--- 直连对端 $PEER ---"
  ping -c 3 -W 2 "$PEER" 2>&1 | tail -4 | sed 's/^/  /'
  echo "  ARP: $(ip neigh show "$PEER" 2>/dev/null)"
  echo "--- 到对端 traceroute(1跳期望) ---"
  timeout 10 tracepath -m 3 "$PEER" 2>&1 | head -5 | sed 's/^/  /'
fi
echo
echo "--- 组播能力(网卡是否 UP/MULTICAST) ---"
ip -o link show | awk -F': ' '{print $2}' | while read -r i; do
  fl=$(cat "/sys/class/net/$i/flags" 2>/dev/null)
  st=$(cat "/sys/class/net/$i/operstate" 2>/dev/null)
  echo "  $i state=$st flags=$fl"
done
echo
echo "### recon done"
