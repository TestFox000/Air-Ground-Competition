#!/bin/bash
# t4：车侧桥洞情报的驻留方式（单槽 vs 分半场）—— 只读
WS="$HOME/dzrobot/src/dzbrain/dzbrain"
echo "=== A. bridge_of / bridge_half / bridge_block 的出现位置 ==="
grep -rn 'bridge_of' "$WS" 2>/dev/null | head -20 || echo "(无 bridge_of)"
echo "--- 半场变量 ---"
grep -rn 'bridge_half\|bridge_block\|_bridge' "$WS/core/state.py" 2>/dev/null | head -20 || echo "(state.py 无)"
echo
echo "=== B. DroneTask 写半场的那两行（上下文）==="
grep -n -B6 -A6 'self.bridge_half' "$WS/tasks/drone.py" | head -40
echo
echo "=== C. DroneState 定义（state.py 里的桥洞字段）==="
grep -n -A30 'class DroneState' "$WS/core/state.py" | head -50
echo
echo "=== D. Mission 是否读 bridge_of / EV_BRIDGE_INFO ==="
grep -rn 'bridge_of\|EV_BRIDGE_INFO\|cross_lower\|cross_upper' "$WS/tasks/mission.py" 2>/dev/null | head -20 || echo "(mission.py 无)"
echo "### done"
