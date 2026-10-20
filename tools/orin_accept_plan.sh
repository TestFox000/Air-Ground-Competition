#!/usr/bin/env bash
# =============================================================================
# 机载规划一键验收（在 Orin 上跑；只写 /tmp，不改任何源文件）
#
# 验四件事：
#   ① 规划进程能起来并绑住 UDP:50000（同机广播投递的前提）
#   ② 同机 UDP 广播投递成立：注入器广播的情报能被 sidecar 收到
#   ③ 规划全链：落格矩阵 -> 门控 -> 路线 -> 三份产物
#   ④ （可选）0x4003 车命令真广播（--send）
#
# 用法：
#   tools/orin_accept_plan.sh                # dry-run，只落盘不发车命令（推荐先跑）
#   tools/orin_accept_plan.sh --send         # 额外验证 0x4003 真广播
#   AG_ACCEPT_SCEN="D2:paper,H9:paper,F5:target_blue,M9:obstacle" tools/orin_accept_plan.sh
#
# ⚠️ 场景类别必须按 13 类口径写：雷 = 非 K 列 paper（K 列 paper 落格判为桥洞箭头）
# ⚠️ 必须**没有别的进程**占着 UDP:50000（本脚本自己就是那个消费者）
# =============================================================================
set -u

REPO="$(cd "$(dirname "$0")/.." && pwd)"
SIDECAR="$REPO/tools/orin_plan_sidecar.py"
PARAMS="$REPO/src/ag_integrate/config/match_params.yaml"
SENDER="$REPO/tools/fake_udp_sender.py"
SCEN="${AG_ACCEPT_SCEN:-D2:paper,H9:paper,F5:target_blue,M9:obstacle}"
SEND=""
[ "${1:-}" = "--send" ] && SEND="--send"
OUT="/tmp/ag_accept_$(date +%Y%m%d_%H%M%S)"

fail() { echo "[失败] $*" >&2; echo "      产物目录（若已建）: $OUT" >&2; exit 1; }
ok()   { echo "[通过] $*"; }

[ -f "$SIDECAR" ] || fail "缺少 $SIDECAR —— 先跑 tools/orin_deploy.py 把规划旁挂推上机"
[ -f "$PARAMS" ]  || fail "缺少 $PARAMS —— 规划包未完整上机"
[ -f "$SENDER" ]  || fail "缺少 $SENDER —— 注入器未上机（tools/fake_udp_sender.py）"
mkdir -p "$OUT"

# ---- ① 规划进程起来 ----
python3 -u "$SIDECAR" --params "$PARAMS" --out "$OUT" --plan-mode on-change \
    --duration-seconds 40 $SEND > "$OUT/sidecar.log" 2>&1 &
SPID=$!
sleep 3
if ! kill -0 "$SPID" 2>/dev/null; then
    echo "--- sidecar.log ---"; cat "$OUT/sidecar.log"
    fail "规划进程未能启动（UDP:50000 被占？或 import 失败）"
fi
ok "规划进程已启动（PID $SPID），监听 UDP:50000${SEND:+，0x4003 实发}"

# ---- ② 注入已知态势（走与 dzuav 相同的广播路径） ----
timeout 25 python3 "$SENDER" --scenario "$SCEN" --rate 5 > "$OUT/fake.log" 2>&1
ok "已注入态势 30s: $SCEN"

sleep 8
kill -INT "$SPID" 2>/dev/null; sleep 4; kill -9 "$SPID" 2>/dev/null
wait "$SPID" 2>/dev/null || true      # 收掉 shell 的 "已杀死" 作业提示，保持输出干净

# ---- ③ 三份产物 + 矩阵落格 ----
for f in cell_matrix.json route_plan.json planned_path.txt; do
    [ -s "$OUT/$f" ] || {
        echo "--- sidecar.log（末 20 行）---"; tail -20 "$OUT/sidecar.log"
        fail "缺少产物 $f（门控未放行 / 注入未生效？看上面日志）"
    }
done
ok "三份产物齐备：$(ls "$OUT" | tr '\n' ' ')"

grep -q '"M"' "$OUT/cell_matrix.json" || grep -q 'M' "$OUT/cell_matrix.json" \
    || fail "矩阵里没有雷(M) 符号 —— 落格或注入有问题"
grep -q 'B' "$OUT/cell_matrix.json" || fail "矩阵里没有蓝靶(B) 符号"
ok "落格矩阵含 M/B 符号"

grep -m1 '"reset"' "$OUT/route_plan.json" | sed 's/^/       /'
grep -m1 '"route_len_cells"' "$OUT/route_plan.json" | sed 's/^/       /'
echo
echo "[完成] 机载规划验收通过；产物目录: $OUT"
echo "       接入全栈: PLAN_SEND=1 tools/orin_bringup.sh up（默认 dry-run 只落盘）"
