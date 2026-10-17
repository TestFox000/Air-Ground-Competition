#!/bin/bash
# Read-only: inspect the logs of the last stack run (19:13-19:20).
L="$HOME/robot_logs"
for f in 4_det 5b_aimshim 6_judgment 8_camview 8_browser; do
  echo "########## $f.log ($(wc -c < "$L/$f.log" 2>/dev/null) bytes) ##########"
  cat "$L/$f.log" 2>/dev/null | head -25
  echo
done
echo "########## 7_path.log 全文（循迹节点）##########"
cat "$L/7_path.log" 2>/dev/null
echo
echo "########## 2_camera.log 尾部 ##########"
tail -8 "$L/2_camera.log" 2>/dev/null
echo
echo "########## 3_localization.log 尾部 ##########"
tail -6 "$L/3_localization.log" 2>/dev/null
echo
echo "########## pids 目录 ##########"
ls -l "$L/pids/" 2>/dev/null
