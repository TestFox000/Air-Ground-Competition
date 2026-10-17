#!/bin/bash
# t4 两端环境取证（只读）。在无人机与小车各跑一遍，输出逐项对照。
# 关键：区分「登录 shell 值」与「非交互 ssh 命令值」——历史坑就在这里。
echo "=== 0. identity ==="
echo "hostname    : $(hostname)"
echo "whoami      : $(whoami)"
echo "date        : $(date -Is)"
echo "kernel      : $(uname -sr)"
echo "arch        : $(uname -m)"
echo "uptime      : $(uptime -p 2>/dev/null || uptime)"
[ -r /etc/os-release ] && . /etc/os-release && echo "os          : ${PRETTY_NAME}"
echo "uptime_sec  : $(cut -d. -f1 /proc/uptime)"

echo
echo "=== 1. 网络 ==="
ip -4 addr show | awk '/^[0-9]+:/{ifc=$2} /inet /{print "  ifc="ifc" addr="$2}'
echo "  default route: $(ip route show default | head -1)"
echo "  hostname -I  : $(hostname -I 2>/dev/null)"
echo "  /etc/hosts   :"
grep -vE '^\s*#' /etc/hosts | grep -v '^\s*$' | sed 's/^/    /'
echo "  avahi/mDNS   : $(systemctl is-active avahi-daemon 2>/dev/null || echo n/a)"

echo
echo "=== 2. ROS 安装 ==="
ls -d /opt/ros/*/ 2>/dev/null | sed 's/^/  /' || echo "  (无 /opt/ros)"
for d in /opt/ros/*/setup.bash; do [ -r "$d" ] && { echo "  setup: $d"; break; }; done

echo
echo "=== 3. ROS 环境 —— 非交互 ssh 命令（ssh host 'cmd' 的真实环境）==="
bash -c '
  for d in /opt/ros/*/setup.bash; do [ -r "$d" ] && source "$d"; done
  echo "  ROS_DISTRO               = ${ROS_DISTRO:-<unset>}"
  echo "  ROS_DOMAIN_ID            = ${ROS_DOMAIN_ID:-<unset → 默认 0>}"
  echo "  RMW_IMPLEMENTATION       = ${RMW_IMPLEMENTATION:-<unset → 默认 rmw_fastrtps_cpp>}"
  echo "  ROS_LOCALHOST_ONLY       = ${ROS_LOCALHOST_ONLY:-<unset>}"
  echo "  ROS_AUTOMATIC_DISCOVERY_RANGE = ${ROS_AUTOMATIC_DISCOVERY_RANGE:-<unset>}"
  echo "  ROS_STATIC_PEERS         = ${ROS_STATIC_PEERS:-<unset>}"
  echo "  FASTRTPS_DEFAULT_PROFILES_FILE = ${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}"
  echo "  CYCLONEDDS_URI           = ${CYCLONEDDS_URI:-<unset>}"
  echo "  ros2                     = $(command -v ros2 || echo MISSING)"
  echo "  python3                  = $(command -v python3 || echo MISSING)"
'

echo
echo "=== 4. ROS 环境 —— 登录 shell（bash -lc，会读 ~/.bashrc/~/.profile）==="
bash -lc '
  for d in /opt/ros/*/setup.bash; do [ -r "$d" ] && source "$d"; done
  echo "  ROS_DISTRO        = ${ROS_DISTRO:-<unset>}"
  echo "  ROS_DOMAIN_ID     = ${ROS_DOMAIN_ID:-<unset → 默认 0>}"
  echo "  RMW_IMPLEMENTATION= ${RMW_IMPLEMENTATION:-<unset>}"
  echo "  DZ_ROS_DOMAIN     = ${DZ_ROS_DOMAIN:-<unset>}"
  echo "  DZ_DOMAIN         = ${DZ_DOMAIN:-<unset>}"
'

echo
echo "=== 5. 所有 ROS_DOMAIN_ID / RMW 的声明点（配置真相源）==="
echo "-- ~/.bashrc / ~/.profile / ~/.bash_profile --"
grep -nE "ROS_DOMAIN_ID|RMW_IMPLEMENTATION|DZ_ROS_DOMAIN|DZ_DOMAIN|ROS_LOCALHOST_ONLY" \
  ~/.bashrc ~/.profile ~/.bash_profile 2>/dev/null | sed 's/^/  /'
echo "(以上为全部命中；空 = 没声明)"
echo "-- 项目 env 文件 --"
for f in ~/dzrobot/dzros.env ~/dzlink_tools/dzlink.env ~/dzpx4/dzpx4.env ~/dzlink.env; do
  if [ -r "$f" ]; then echo "  [有] $f"; sed 's/^/      /' "$f"; else echo "  [无] $f"; fi
done
echo "-- /etc/environment --"
grep -nE "ROS_|RMW_" /etc/environment 2>/dev/null | sed 's/^/  /' || echo "  (无命中)"
echo "-- systemd 用户/系统 unit 里的 ROS 声明 --"
grep -rlnE "ROS_DOMAIN_ID" /etc/systemd/system 2>/dev/null | sed 's/^/  /' || echo "  (无)"
crontab -l 2>/dev/null | grep -nE "ROS_|dz|robot" | sed 's/^/  cron: /' || echo "  (无 crontab)"

echo
echo "=== 6. 工作区 ==="
for w in "$HOME/dzrobot" "$HOME/dzrobot_v2" "$HOME/dzpx4" "$HOME/dzuav" "$HOME/dzlink_tools" "$HOME/dzlink_ws"; do
  if [ -d "$w" ]; then echo "  [有] $w  ($(du -sh "$w" 2>/dev/null | cut -f1))"; else echo "  [无] $w"; fi
done
for s in "$HOME/dzrobot/install/setup.bash" "$HOME/dzpx4/install/setup.bash"; do
  [ -r "$s" ] && echo "  [setup] $s" || echo "  [无setup] $s"
done

echo
echo "=== 7. 当前 ROS 图（冷态应为空/极少）==="
bash -lc '
  for d in /opt/ros/*/setup.bash; do [ -r "$d" ] && source "$d"; done
  echo "  -- node list --";  timeout 10 ros2 node list 2>&1 | sed "s/^/    /"
  echo "  -- topic list --"; timeout 10 ros2 topic list 2>&1 | sed "s/^/    /"
  echo "  -- 进程 --";        pgrep -af "ros2|dzbrain|path_executor|dzlink|fastlio|MicroXRCEAgent" | grep -v pgrep | sed "s/^/    /" || echo "    (无)"
'

echo
echo "=== 8. 防火墙 / DDS 端口占用 ==="
echo "-- ufw --"; (command -v ufw >/dev/null && ufw status 2>&1 | head -5) || echo "  (无 ufw)"
echo "-- iptables filter --"; (command -v iptables >/dev/null && sudo -n iptables -L -n 2>/dev/null | head -8) || echo "  (需要 sudo，跳过；非 root 只读会话不取)"
echo "-- 域号相关 UDP 绑定 (7400..37200) --"
ss -lunp 2>/dev/null | awk 'NR==1 || ($5 ~ /:(7[4-9][0-9][0-9]|3[0-7][0-9][0-9])$/)' | sed 's/^/  /'
echo "-- 全部 UDP LISTEN（前 20）--"
ss -lunp 2>/dev/null | head -20 | sed 's/^/  /'

echo
echo "=== 9. 组播能力（网卡 MULTICAST 位 + 路由）==="
ip -o link show | while read -r n _ rest; do
  ifc=${n%:}
  st=$(cat /sys/class/net/$ifc/operstate 2>/dev/null)
  fl=$(cat /sys/class/net/$ifc/flags 2>/dev/null)
  mcast=$(( (0x$fl >> 12) & 1 ))
  echo "  $ifc state=$st flags=0x$fl multicast_bit=$mcast"
done
echo "-- ip maddr（组播成员）--"
ip maddr show | head -12 | sed 's/^/  /'

echo
echo "=== 10. 时间同步（跨机 ts 比较的前提）==="
timedatectl 2>/dev/null | sed 's/^/  /' || date -Is

echo "### t4_env done on $(hostname)"
