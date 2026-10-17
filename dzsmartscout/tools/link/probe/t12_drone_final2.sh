#!/bin/bash
# t12 机侧补充取证：单测计数 + _run_ros 修复点行号（只读）
WS=/home/duzhong/dzlink_tools
echo "=== A) 单测：只取结论行 ==="
cd "$WS" || exit 1
echo "--- test_dzlink_supervisor.py ---"
python3 test_dzlink_supervisor.py 2>&1 | grep -E "^Ran |^OK$|^FAILED|^ERROR" 
echo "--- test_dz_send_waypoint.py ---"
python3 test_dz_send_waypoint.py 2>&1 | grep -E "^Ran |^OK$|^FAILED|^ERROR"
echo
echo "=== B) _run_ros 修复点行号（udp_src 必须在 try 之前）==="
python3 - <<'PY'
import re
p = "/home/duzhong/dzlink_tools/dzlink_supervisor.py"
lines = open(p, encoding="utf-8").read().splitlines()
start = next(i for i, l in enumerate(lines) if l.startswith("def _run_ros("))
print("def _run_ros 在第", start + 1, "行")
for i in range(start, start + 30):
    l = lines[i]
    if "udp_src = None" in l or l.strip() == "try:" or l.strip().startswith("if mode =="):
        print("  %5d: %s" % (i + 1, l))
PY
echo
echo "=== C) run 分支注入调用（第 1913 行往下）==="
sed -n '1909,1922p' "$WS/dzlink_supervisor.py"
echo
echo "=== D) 发送器：target/field 关键行 ==="
sed -n '296,312p' "$WS/dz_send_waypoint.py"
echo "### done"
