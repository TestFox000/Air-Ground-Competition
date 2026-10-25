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
#   AG_ACCEPT_SCEN="D2:paper,..." tools/orin_accept_plan.sh
#
# ⚠️ 场景类别按 14 类口径写（ft_v1，2026-10-26 起）：雷 = 非 K 列 paper，
#    K 列 paper 落格判为桥洞箭头。**K 列箭头是必需元素**：桥门规则下
#    无箭头则门全关，N10 摆位是孤岛（连通域仅 15 格），门控过后规划
#    必报"goal N5 不可达"——2026-10-26 机载验收实测踩坑，双箭头 K7+K2
#    才能同时满足"东→西扫雷→北门回 N5"的完整可达拓扑。
# 端口：优先 50000；若已被全栈里的 plan 窗口占用，自动改用 5010x 空闲端口，
#       本脚本自己的 sidecar 与注入器一起换——**不必先停全栈**（2026-10-22 实测：
#       按手册"先起全栈再验收"，原实现直接 bind 失败 Address already in use）。
# =============================================================================
set -u

REPO="$(cd "$(dirname "$0")/.." && pwd)"
SIDECAR="$REPO/tools/orin_plan_sidecar.py"
PARAMS="$REPO/src/ag_integrate/config/match_params.yaml"
SENDER="$REPO/tools/fake_udp_sender.py"
SCEN="${AG_ACCEPT_SCEN:-D2:paper,H9:paper,F5:target_blue,M9:obstacle,K7:paper,K2:paper}"
SEND=""
[ "${1:-}" = "--send" ] && SEND="--send"
OUT="/tmp/ag_accept_$(date +%Y%m%d_%H%M%S)"

fail() { echo "[失败] $*" >&2; echo "      产物目录（若已建）: $OUT" >&2; exit 1; }
ok()   { echo "[通过] $*"; }

[ -f "$SIDECAR" ] || fail "缺少 $SIDECAR —— 先跑 tools/orin_deploy.py 把规划旁挂推上机"
[ -f "$PARAMS" ]  || fail "缺少 $PARAMS —— 规划包未完整上机"
[ -f "$SENDER" ]  || fail "缺少 $SENDER —— 注入器未上机（tools/fake_udp_sender.py）"
mkdir -p "$OUT"

# ---- ⓪ 栈一致性：一键启动实际会用哪套二进制/引擎（错版本不会报错，只会静默错结果） ----
BIN="$REPO/onboard/dzuav/build/dzuav"
[ -f "$BIN" ] || fail "缺少 dzuav 二进制 $BIN —— 先 bash tools/orin_build_verify.sh"
# 类别名表可直接在二进制里查：13/14 类都含 uav_end/paper，7 类旧版含 mine/arrow。
# 这条是 2026-10-20 查出的真实事故形态：一键启动曾指向 9 月的 7 类老二进制。
# （ft_v1 起的"target"整体板名是 target_red 的子串，grep 无法单独判 14 类；
#   通道契约由 HbbDetector 构造期形状校验兜底，18≠17 直接 fail-fast。）
grep -qa uav_end "$BIN" || fail "dzuav 二进制未见类别名 uav_end——疑似 7 类旧版，跑起来会与协议错位"
grep -qa paper "$BIN" || fail "dzuav 二进制未见类别名 paper"
ok "dzuav 二进制类目表正常（含 uav_end / paper 类别名，13/14 类同判）"
for e in hbb_detector_d13_fp16 cell_classifier_v4d_fp16; do
    [ -f "$HOME/dzuav/engines/$e.engine" ] || \
        fail "缺少 canonical 引擎 $e.engine（文件名沿用 d13 旧名，内容随 current 指针——2026-10-26 起为 ft_v1）"
done
ok "canonical 引擎齐备（hbb_detector_d13_fp16.engine 现为 ft_v1 内容 + cell_classifier_v4d）"

# ---- ① 规划进程起来 ----
# 选端口：50000 空闲就用它；被占（全栈在跑）则换 5010x，两边一起换端口。
free_port() {
    python3 -c 'import socket, sys
for p in (50000, 50100, 50101, 50102, 50103, 50104, 50105):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("", p)); s.close(); print(p); sys.exit(0)
    except OSError:
        s.close()
sys.exit(1)'
}
PORT=$(free_port) || fail "找不到空闲 UDP 端口（50000/5010x 都被占用）"
if [ "$PORT" != "50000" ]; then
    echo "[提示] UDP:50000 已被占用（多半是全栈的 plan 窗口在跑），"
    echo "       本次验收改用 UDP:$PORT，sidecar 与注入器同步换端口，不影响正在运行的规划。"
fi

python3 -u "$SIDECAR" --params "$PARAMS" --out "$OUT" --plan-mode on-change \
    --udp-port "$PORT" --duration-seconds 40 $SEND > "$OUT/sidecar.log" 2>&1 &
SPID=$!
sleep 3
if ! kill -0 "$SPID" 2>/dev/null; then
    echo "--- sidecar.log ---"; cat "$OUT/sidecar.log"
    fail "规划进程未能启动（UDP:$PORT 被占？或 import 失败）"
fi
ok "规划进程已启动（PID $SPID），监听 UDP:$PORT${SEND:+，0x4003 实发}"

# ---- ② 注入已知态势（走与 dzuav 相同的编码路径，回环投递最稳） ----
timeout 25 python3 "$SENDER" --scenario "$SCEN" --rate 5 --port "$PORT" \
    --to 127.0.0.1 > "$OUT/fake.log" 2>&1
SND_RC=$?
# 124 = timeout 到期（循环模式的正常结束）；其它非 0 一律是注入失败。
# 2026-10-22 实测：机上旧版注入器不认 --to，直接 usage 退出，而脚本仍打"已注入"。
if [ "$SND_RC" -ne 0 ] && [ "$SND_RC" -ne 124 ]; then
    echo "--- fake.log（末 8 行）---"; tail -8 "$OUT/fake.log"
    fail "注入器退出码 $SND_RC（多为机上 fake_udp_sender.py 版本旧/参数不兼容），注入未发生"
fi
ok "已注入态势 30s: $SCEN（UDP:$PORT）"

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

# 断言写在**明细**（spots 的 kind）上，不写符号图例——图例里本来就含 M/B，
# 拿它做断言等于永远为真（2026-10-20 复查发现，属假绿）。
grep -q '"kind": "paper"'       "$OUT/cell_matrix.json" || fail "矩阵明细里没有 paper（雷）落格 —— 注入未生效或落格有问题"
grep -q '"kind": "target_blue"' "$OUT/cell_matrix.json" || fail "矩阵明细里没有 target_blue（蓝靶）"
ok "矩阵明细含 paper(雷) 与 target_blue(蓝靶)"

grep -m1 '"reset"' "$OUT/route_plan.json" | sed 's/^/       /'
grep -m1 '"route_len_cells"' "$OUT/route_plan.json" | sed 's/^/       /'
echo
echo "[完成] 机载规划验收通过；产物目录: $OUT"
echo "       接入全栈: PLAN_SEND=1 tools/orin_bringup.sh up（默认 dry-run 只落盘）"
