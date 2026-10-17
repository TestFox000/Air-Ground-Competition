#!/bin/bash
# t12 无人机侧取证：部署物 md5 / 单测 / run 日志直方图 / 修复点自证（只读）
WS=/home/duzhong/dzlink_tools
echo "=== 1) 部署物 md5 ==="
md5sum "$WS/dzlink_supervisor.py" "$WS/dz_send_waypoint.py" \
       "$WS/test_dzlink_supervisor.py" "$WS/test_dz_send_waypoint.py"
echo "=== 2) 单测（两个文件）==="
cd "$WS" || exit 1
echo "--- test_dzlink_supervisor.py ---"
python3 test_dzlink_supervisor.py 2>&1 | tail -3
echo "--- test_dz_send_waypoint.py ---"
python3 test_dz_send_waypoint.py 2>&1 | tail -3
echo "=== 3) /tmp/t12_run.jsonl 的 kind 直方图（D2 生效的直接证据）==="
python3 - <<'PY'
import json, collections
c = collections.Counter()
kinds = {}
for line in open('/tmp/t12_run.jsonl', encoding='utf-8'):
    line = line.strip()
    if not line:
        continue
    try:
        d = json.loads(line)
    except Exception:
        continue
    k = d.get('kind', '?')
    c[k] += 1
    if k in ('intel_emit', 'route_emit') and k not in kinds:
        kinds[k] = d
print(dict(c))
for k, v in kinds.items():
    print(k, '=>', json.dumps(v, ensure_ascii=False)[:220])
PY
echo "=== 4) 修复点自证：udp_src 初始化位置 ==="
grep -n "udp_src = None" "$WS/dzlink_supervisor.py"
grep -n "^    try:" "$WS/dzlink_supervisor.py" | head -3
echo "--- run 分支是否调用注入函数 ---"
grep -n -A6 'if args.cmd == "run":' "$WS/dzlink_supervisor.py"
echo "=== 5) 发送器修复点 ==="
grep -n "BLOCK_SEP\|def split_blocks\|blocks_opt" "$WS/dz_send_waypoint.py" | head -12
echo "### done"
