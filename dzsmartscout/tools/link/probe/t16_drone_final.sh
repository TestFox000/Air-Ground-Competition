#!/bin/bash
# t16 无人机侧取证：部署物 md5 / 单测计数 / contract 横幅真值（只读）
WS=/home/duzhong/dzlink_tools
echo "=== 1) 部署物 md5 ==="
md5sum "$WS/dzlink_supervisor.py" "$WS/test_dzlink_supervisor.py"
echo "=== 2) 机上单测（结论行）==="
cd "$WS" || exit 1
python3 test_dzlink_supervisor.py 2>&1 | grep -E "^Ran |^OK$|^FAILED|^ERROR"
echo "=== 3) contract 横幅：时序段（应为 2s / K10(下)/K5(上) / 45s）==="
python3 dzlink_supervisor.py contract 2>&1 | sed -n '/时序约束/,/域号：/p'
echo "=== 4) 修复点自证：bridge_round_due（两半场同轮成对）==="
grep -n "bridge_round_due" "$WS/dzlink_supervisor.py"
echo "=== 5) CAR_CONTRACT 现值 ==="
sed -n '/^CAR_CONTRACT = {/,/^}/p' "$WS/dzlink_supervisor.py"
echo "=== 6) 数据侧：t16 三个相位的 run 日志是否留档 ==="
ls -l /tmp/t16_sup_*.jsonl /tmp/t16_car_recv_*.jsonl 2>/dev/null | head -8
echo "### done"
