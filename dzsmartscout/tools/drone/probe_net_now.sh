#!/usr/bin/env bash
# 无人机网络/链路现状探测（严格只读）
export LC_ALL=C
echo "=== [A] 当前 WiFi 链路 ==="
iw dev wlP1p1s0 link 2>/dev/null || echo "(iw 不可用)"
echo "--- 活动连接 ---"
nmcli -t -f NAME,DEVICE,TYPE,STATE con show --active 2>/dev/null
echo "--- 已保存的 WiFi（配置名 -> SSID） ---"
nmcli -t -f NAME,TYPE con show 2>/dev/null | awk -F: '$2 ~ /wireless/ {print $1}' | while read -r c; do
  s=$(nmcli -t -f 802-11-wireless.ssid con show "$c" 2>/dev/null | cut -d: -f2-)
  echo "  [$c] -> SSID=$s"
done
echo "--- 网卡 ---"
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION dev status 2>/dev/null
echo "--- 可见 WiFi（SSID:信号:信道，前 12） ---"
nmcli -t -f SSID,SIGNAL,CHAN dev wifi list 2>/dev/null | head -12

echo
echo "=== [B] 邻居与本地网段 ==="
ip -4 -br addr show | grep -Ev '^(docker|veth|br-)'
echo "--- ARP 邻居 ---"
ip neigh show | grep -v FAILED
echo "--- mDNS: 网段上的 workstation 主机 ---"
timeout 10 avahi-browse -rtp _workstation._tcp 2>/dev/null | grep -E '^\+|^=' | head -20 || echo "(无输出)"
echo "--- mDNS 解析 duzhong.local ---"
timeout 5 avahi-resolve -n duzhong.local 2>&1 | head -3 || echo "(失败)"
echo "--- mDNS 解析 dzsmart.local ---"
timeout 5 avahi-resolve -n dzsmart.local 2>&1 | head -3 || echo "(失败)"

echo
echo "=== [C] 路由与跨网段可达性 ==="
ip route
for t in 10.240.174.68 10.240.174.188 10.158.138.202 10.158.138.1; do
  printf "  ping %-16s " "$t"
  ping -c 1 -W 1 "$t" >/dev/null 2>&1 && echo "通" || echo "不通"
done

echo
echo "=== [D] 同网段存活主机（ping sweep 10.158.138.0/24） ==="
seq 1 254 | xargs -P 64 -I{} sh -c 'ping -c 1 -W 1 10.158.138.{} >/dev/null 2>&1 && echo 10.158.138.{}' 2>/dev/null | sort -t. -k4 -n | tr '\n' ' '
echo
echo "--- 存活主机的 ARP/MAC ---"
ip neigh show | grep -v FAILED | grep -v '^10.158.138.104 '

echo
echo "=== [E] 雷达与雷达网口 ==="
echo "  eno1 carrier=$(cat /sys/class/net/eno1/carrier 2>/dev/null)  ip=$(ip -4 -br addr show eno1 | awk '{print $3}')"
ping -c 2 -W 1 192.168.3.105 2>&1 | tail -2
echo
echo "=== 探测结束 ==="
