#!/bin/bash
# t4 车侧：/nav/waypoint 到底谁能写（双写者判定，只读）
WS="$HOME/dzrobot"
echo "=== A. dzbrain 源码里 /nav/waypoint 的 create_publisher ==="
grep -rn "waypoint" "$WS/src/dzbrain/dzbrain/io/" 2>/dev/null | head -20
echo
echo "=== B. ros_io.py 里所有 create_publisher（谁写什么）==="
grep -n -A2 "create_publisher" "$WS/src/dzbrain/dzbrain/io/ros_io.py" 2>/dev/null | head -40
echo
echo "=== C. 想给 /nav/waypoint 发东西的脚本（潜在第二写者）==="
grep -rn "topic pub.*nav/waypoint\|publish.*nav/waypoint" "$WS" --include="*.sh" --include="*.py" 2>/dev/null \
  | grep -v "/install/\|/build/\|/history/" | head -10
echo
echo "=== D. contract.py 里 waypoint_cmd 的 direction 字段 ==="
grep -n -B1 -A2 '"waypoint_cmd"' "$WS/src/dzbrain/dzbrain/io/contract.py"
echo
echo "=== E. 车上有哪些别的进程可能写 /dzlink/drone_to_car ==="
grep -rn "drone_in" "$WS/src/dzbrain/dzbrain/io/" 2>/dev/null | head -10
echo "### done"
