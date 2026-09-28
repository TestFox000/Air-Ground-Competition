#!/usr/bin/env bash
# 车端「全链路干跑 + 黑匣子」：在车上把整条链串起来跑一遍，但**只算不发**：
#
#   fake_intel_node ─/air/field_intel─▶ air_link_node ─UDP─▶ ground_link_node
#        ─/link/field_intel─▶ car_node(dry_run=true) ─/car/route_status─▶ 看路线
#
# 不会让车动一下：不启动任何渡众节点、不碰串口/相机/底盘，car_node 在 dry_run 下
# 一条 /nav/waypoint 都不发。结束时只 kill 本脚本自己起的 PID。
#
# 为什么要有这个脚本：
#   实车一局跑完最痛的是"不知道发生了什么"。这里用假情报把真实链路（含 UDP 收发）
#   跑通，顺便验证黑匣子日志确实落到了 $AG_LOG_DIR，取回本地就能出复盘报告：
#
#     python tools/remote_run.py --host <IP> --user duzhong --script tools/car_log_rehearsal.sh
#     python tools/remote_run.py --host <IP> --user duzhong --get /tmp/ag_logs --to logs/car
#     python tools/log_report.py --dir logs/car
#
# 这一步过了，说明"日志 → 取回 → 复盘"这条调试路是通的；真跑一局时照搬即可。

WS="${AG_WS:-/tmp/ag_ws}"
export AG_LOG_DIR="${AG_LOG_DIR:-/tmp/ag_logs}"
export AG_LOG_SESSION="${AG_LOG_SESSION:-rehearsal}"
RUN_S="${RUN_S:-25}"          # 跑多久（秒），要给 UDP 与会话落盘留足时间
LOG_ROOT="$AG_LOG_DIR/$AG_LOG_SESSION"

source /opt/ros/humble/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null

# ⚠️ 这里必须杀**进程组**，不能只杀 `ros2 run` 的 pid：
#    `ros2 run` 自己是个壳，真正干活的是它 fork 出来的子进程。
#    只 kill 壳 → 节点变成孤儿继续跑（实测踩过：上一轮的 air_link/ground_link 还活着，
#    和这一轮的两套节点同时收发，日志里就冒出"456 帧乱序/重复"这种假故障）。
PGRPS=""
hr() { echo "----------------------------------------------------"; }
cleanup() {
  for pg in $PGRPS; do
    kill -INT -"$pg" 2>/dev/null    # 负号 = 整个进程组
  done
  sleep 1
  for pg in $PGRPS; do
    kill -9 -"$pg" 2>/dev/null
  done
  # 兜底再扫一遍本工作空间的节点（setsid 万一没能独立成组时也退得干净）
  pkill -9 -f "$WS/install/" 2>/dev/null
  [ -n "$PGRPS" ] && echo "  已停止本脚本启动的进程组: $PGRPS"
}
trap cleanup EXIT

# 只杀本工作空间里的节点：命令行里带 "$WS/install/" 的进程只可能是我们起的
kill_leftovers() {
  pkill -f "$WS/install/" 2>/dev/null && sleep 1 && return 0
  return 1
}

echo "==================== 车端全链路干跑（不动车） ===================="
date
echo "工作区 : $WS"
echo "日志目录: $LOG_ROOT   （会话 $AG_LOG_SESSION）"
echo

hr; echo "## 0. 前置检查：车上现在有哪些节点（避免与官方栈互相干扰）"
NODES="$(timeout 10 ros2 node list 2>/dev/null)"
echo "$NODES" | sed 's/^/  /'
earlier=$(echo "$NODES" | grep -cE "^/(air_link_node|ground_link_node|fake_intel_node|car_node)$")
if [ "${earlier:-0}" -gt 0 ]; then
  echo
  echo "  [警告] 检测到 $earlier 个本项目的残留节点（多半是上一次没退干净）。"
  echo "         两套节点同时跑会让日志看起来像'链路坏了'（重复帧、帧数翻倍），"
  echo "         所以这里先清掉它们，再来一次干净的。"
  kill_leftovers && echo "  已清理残留节点" || echo "  （没清掉，继续跑，但结论可能被污染）"
  sleep 2
else
  echo "  [OK] 没有本项目的残留节点"
fi
echo

hr; echo "## 1. 清掉上次的同名日志（避免新旧混在一份报告里）"
if [ -d "$LOG_ROOT" ]; then
  ls -1 "$LOG_ROOT" | sed 's/^/  删除 /'
  rm -f "$LOG_ROOT"/*.jsonl
else
  mkdir -p "$LOG_ROOT"
fi
echo "  日志目录就绪: $LOG_ROOT"
echo

hr; echo "## 2. 启动链路与节点"
start() {  # start <名字> <命令行...>
  local name="$1"; shift
  setsid "$@" > "/tmp/ag_${name}.log" 2>&1 &   # setsid：让节点自成进程组，便于整组回收
  local pg=$!
  PGRPS="$PGRPS $pg"
  echo "  $name 进程组=$pg -> /tmp/ag_${name}.log"
}

# 走环回而不是广播：不往现场网络里发东西
start ground_link ros2 run ag_bridge ground_link_node --ros-args \
  -p bind_port:=9901 -p peer_ip:=127.0.0.1 -p peer_port:=9902
start air_link ros2 run ag_bridge air_link_node --ros-args \
  -p bind_port:=9902 -p peer_ip:=127.0.0.1 -p peer_port:=9901
sleep 2
start fake_intel ros2 run ag_bridge fake_intel_node --ros-args \
  -p mode:=full -p period_s:=0.5 -p intel_topic:=/air/field_intel
start car_node ros2 run ag_car car_node --ros-args \
  -p dry_run:=true -p min_votes:=2 -p min_mines:=1 \
  -p require_target_face:=true -p start_cell:=N10 -p goal_cell:=N11

sleep 4
alive=0
for pg in $PGRPS; do kill -0 "$pg" 2>/dev/null && alive=$((alive + 1)); done
echo "  存活 $alive / 4 个进程"
if [ "$alive" -lt 4 ]; then
  echo "  [FAIL] 有节点没起来，日志："
  for f in /tmp/ag_ground_link.log /tmp/ag_air_link.log \
           /tmp/ag_fake_intel.log /tmp/ag_car_node.log; do
    echo "  --- $f"; tail -6 "$f" 2>/dev/null | sed 's/^/    /'
  done
  exit 1
fi
echo "  [OK] 四个节点都在跑"
echo

hr; echo "## 3. 让链路跑 ${RUN_S}s（假情报 -> UDP -> car_node）"
sleep "$RUN_S"
echo "  完成"
echo

hr; echo "## 4. car_node 算出的路线（dry_run，只算不发）"
timeout 15 ros2 topic echo --once /car/route_status --field data 2>/dev/null | sed 's/^/  /'
echo

hr; echo "## 5. 反证：dry_run 下**一条路线命令都没发**"
got=$(timeout 6 ros2 topic echo --once /nav/waypoint --field data 2>/dev/null)
if [ -z "$got" ]; then
  echo "  [OK] 没收到任何 /nav/waypoint 消息 —— 车不会动"
else
  echo "  [FAIL] 竟然收到了: $got"
fi
echo

hr; echo "## 6. 黑匣子：落了哪些文件"
ls -l "$LOG_ROOT" 2>/dev/null | sed 's/^/  /'
echo

hr; echo "## 7. 黑匣子：事件概览（这台车上的第一手复盘素材）"
python3 - "$LOG_ROOT" <<'PY'
import glob, json, os, sys
from collections import Counter

root = sys.argv[1]
events, bad = [], 0
for f in sorted(glob.glob(os.path.join(root, "*.jsonl"))):
    with open(f, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except Exception:
                bad += 1
events.sort(key=lambda r: (r.get("ts", 0), r.get("seq", 0)))

if not events:
    print("  [FAIL] 一条事件都没有 —— 日志没写进来。"
          "检查 AG_LOG_DIR 是否可写、节点参数 log_enabled 是否为 true")
    sys.exit(1)

t0 = events[0].get("ts", 0)
print(f"  共 {len(events)} 条事件，{len({r.get('node') for r in events})} 个节点，坏行 {bad}")
print("  各节点条数: " + ", ".join(
    f"{n}×{c}" for n, c in Counter(r.get("node") for r in events).most_common()))
print()
# 只打关键事件，文本日志(mirror)单独汇总，避免刷屏
key = {"session.begin", "intel.recv", "intel.stable", "plan.built",
       "route.publish", "route.blocked", "route.reject", "link.tx", "link.rx",
       "link.drop", "aim.pass", "aim.block", "referee.state", "node.error"}
for r in events:
    ev = r.get("ev")
    if ev not in key:
        continue
    t = r.get("ts", t0) - t0
    body = {k: v for k, v in r.items()
            if k not in ("ts", "mono", "seq", "node", "ev", "lvl", "session")}
    print(f"  T+{t:6.1f}s {r.get('node','?'):<18} {ev:<14} {json.dumps(body, ensure_ascii=False)[:150]}")

texts = [r for r in events if r.get("ev") == "ros.log"]
if texts:
    print()
    print(f"  -- 文本日志 {len(texts)} 条（去重后；完整内容见本地报告）--")
    seen = set()
    for r in texts:
        msg = str(r.get("msg", ""))
        if msg in seen:
            continue
        seen.add(msg)
        print(f"     [{r.get('lvl')}] {r.get('node')}: {msg[:130]}")
PY
echo

echo "==================== 结束（节点将自动停止） ===================="
echo "取回日志："
echo "  python tools/remote_run.py --host <IP> --user duzhong --get $AG_LOG_DIR --to logs/car"
echo "出报告　：python tools/log_report.py --dir logs/car"
