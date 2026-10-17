#!/bin/bash
# t18 · hub 版本沿革取证（只读）：备份目录里有哪些版本、当前是什么、f4151410 能否 diff
WS=/home/duzhong/dzlink_tools
echo "=== 1) 当前 hub 文件 ==="
ls -l --time-style=+%Y-%m-%dT%H:%M:%S "$WS"/dzlink_supervisor.py "$WS"/dz_send_waypoint.py \
      "$WS"/test_dzlink_supervisor.py "$WS"/test_dz_send_waypoint.py 2>/dev/null
md5sum "$WS"/dzlink_supervisor.py "$WS"/dz_send_waypoint.py "$WS"/test_dzlink_supervisor.py "$WS"/test_dz_send_waypoint.py 2>/dev/null
echo
echo "=== 2) 备份目录（t12/t16 各一次部署都留了旧文件）==="
ls -ld --time-style=+%Y-%m-%dT%H:%M:%S "$WS"/_backups/*/ 2>/dev/null
for d in "$WS"/_backups/*/; do
  [ -f "$d/dzlink_supervisor.py" ] && {
    printf '%s  ' "$(md5sum "$d/dzlink_supervisor.py" | cut -d' ' -f1)"
    stat -c '%y  %n' "$d/dzlink_supervisor.py"
  }
done
echo
echo "=== 3) f4151410（t12 部署版）能否 diff：用 t16 备份里的那份 ==="
BAK=$(ls -d "$WS"/_backups/t16_* 2>/dev/null | tail -1)
echo "  取备份: $BAK"
if [ -n "$BAK" ] && [ -f "$BAK/dzlink_supervisor.py" ]; then
  md5sum "$BAK/dzlink_supervisor.py"
  echo "  --- unified diff（备份 f4151410 → 当前）只打前 120 行 ---"
  diff -u "$BAK/dzlink_supervisor.py" "$WS/dzlink_supervisor.py" | head -120
  echo "  --- diff 统计 ---"
  diff -u "$BAK/dzlink_supervisor.py" "$WS/dzlink_supervisor.py" | grep -c '^+' 
  diff -u "$BAK/dzlink_supervisor.py" "$WS/dzlink_supervisor.py" | grep -c '^-'
else
  echo "  !! 备份里没有 dzlink_supervisor.py，无法 diff"
fi
echo
echo "=== 4) 同目录是否还有别处的 supervisor 副本（防多版本）==="
find /home/duzhong -name "dzlink_supervisor.py" -not -path "*/__pycache__/*" 2>/dev/null | while read -r f; do
  printf '%s  %s\n' "$(md5sum "$f" | cut -d' ' -f1)" "$f"
done
echo "### done"
