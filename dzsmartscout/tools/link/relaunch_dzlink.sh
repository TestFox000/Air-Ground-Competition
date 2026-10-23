#!/usr/bin/env bash
# 重启无人机侧长时信标（幂等；文件名刻意不含被 pkill 的模式，避免自伤）
echo "=== 清理前 ==="
pgrep -af 'beaco[n]\.sh|dzlin[k]_hello' | head -8

pkill -f 'beaco[n]\.sh' 2>/dev/null
pkill -f 'dzlin[k]_hello' 2>/dev/null
sleep 3

echo "=== 清理后（空=干净） ==="
pgrep -af 'beaco[n]\.sh|dzlin[k]_hello' | head -8

rm -f /tmp/dzlink_dzsmart.log /tmp/dzlink_dzsmart_run.log
setsid nohup bash /tmp/beacon.sh 3600 dzsmart > /tmp/dzlink_dzsmart_run.log 2>&1 < /dev/null &
sleep 8

echo "=== 新进程 ==="
pgrep -af 'beaco[n]\.sh|dzlin[k]_hello' | head -8
echo "=== 运行日志 ==="
cat /tmp/dzlink_dzsmart_run.log
echo "=== 已收条数 ==="
grep -c 'HELLO from' /tmp/dzlink_dzsmart.log 2>/dev/null
