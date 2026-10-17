#!/usr/bin/env bash
# ==============================================================================
#  smoke_ros.sh —— dzlink_supervisor 的**真机 ROS 冒烟**（在无人机上跑）
#
#  目的：证明「rclpy 层」真的能收发 DDS —— 这是 Windows 本机单测**证明不了**的部分。
#
#  怎么证明（不依赖小车、不碰小车任何东西）：
#    ① 起本机**临时订阅者**冒充小车，订阅 /dzlink/drone_to_car（BEST_EFFORT）
#       —— 收心跳与情报；打印**实际收到的 JSON**（判据就是这些报文本身）。
#    ② 起本机临时订阅者订阅 /nav/waypoint（RELIABLE）—— 收路线命令。
#    ③ 喂三种感知输入：文件尾随（JSONL）+ UDP JSON 包 + 命令文件（route/bridge）。
#    ④ 反向闭环：把收到的**心跳 seq** 回一条 ack 到 /dzlink/car_to_drone，
#       再看 supervisor 日志里有没有 `"kind":"car_ack"` —— 证明回传方向也通。
#    ⑤ 只跑十几秒，退出码 0=全过 / 3=发得出去但没人订阅 / 2=没有 ROS 环境。
#
#  只在 $TMPDIR（/tmp）与 ~/dzlink_tools/ 下写文件；不动任何既有配置，不装包。
# ==============================================================================
set -u

DOMAIN="${DOMAIN:-118}"
DZ_REMOTE_DIR="${DZ_REMOTE_DIR:-$HOME/dzlink_tools}"
SUP="$DZ_REMOTE_DIR/dzlink_supervisor.py"
RUN_S="${RUN_S:-16}"
WORK="$(mktemp -d /tmp/dzsmoke.XXXXXX)"
PERC="$WORK/perception.jsonl"      # ② 文件输入的样本
CMD="$WORK/cmd.jsonl"              # ③ 命令文件的样本
ACCEPTED="$WORK/accepted.jsonl"    # 冒充小车收到的东西
LOG="$WORK/supervisor.jsonl"       # supervisor 自己的日志
SUPOUT="$WORK/supervisor.stdout"
UDP_PORT="${UDP_PORT:-39042}"

SUP_PID=""
ACCEPT_PID=""
ACK_PID=""

cleanup() {
  for p in "$ACK_PID" "$ACCEPT_PID" "$SUP_PID"; do
    [ -n "$p" ] && kill "$p" 2>/dev/null
  done
  sleep 0.5
  for p in "$ACK_PID" "$ACCEPT_PID" "$SUP_PID"; do
    [ -n "$p" ] && kill -9 "$p" 2>/dev/null
  done
  return 0
}
trap 'cleanup' EXIT INT TERM

step() { printf '\n=== %s ===\n' "$*"; }
ok()   { printf '  [OK]   %s\n' "$*"; }
bad()  { printf '  [FAIL] %s\n' "$*"; FAILED="$*"; }
FAILED=""
note() { printf '  %s\n' "$*"; }

# ------------------------------------------------------------------ 前置 ----
step "0. 环境"
if [ ! -f "$SUP" ]; then echo "ERROR: 找不到 $SUP" >&2; exit 1; fi
if [ ! -f /opt/ros/humble/setup.bash ]; then echo "ERROR: /opt/ros/humble/setup.bash 不存在" >&2; exit 2; fi
# ★ 顺序很讲究：先 source ROS，再 source 域号文件（域号文件要最后生效）
# shellcheck disable=SC1091
set +u
. /opt/ros/humble/setup.bash
set -u
if [ -f "$DZ_REMOTE_DIR/dzlink.env" ]; then
  # shellcheck disable=SC1091
  set +u; . "$DZ_REMOTE_DIR/dzlink.env"; set -u
fi
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY=0
note "ROS_DOMAIN_ID = $ROS_DOMAIN_ID（本次统一用 $DOMAIN）"
note "工作目录     = $WORK"
note "supervisor   = $SUP"
python3 -c 'import rclpy; print("  rclpy OK:", rclpy.__file__)' || { echo "ERROR: 没有 rclpy" >&2; exit 2; }

# --------------------------------------------------------- 造输入样本 ----
step "1. 准备三种感知输入 + 命令文件"
cat > "$PERC" <<'EOF'
{"type":"bridge","direction":"left","hole":"K9","confidence":0.93}
{"type":"field_verify","mines":["A3"],"obstacles":["K9"]}
EOF
cat > "$CMD" <<'EOF'
{"type":"target_zone","blocks":["G10","H10"]}
{"type":"route","data":"N10 M10 L10 L9 K9"}
EOF
note "perception.jsonl:"; sed 's/^/    /' "$PERC"
note "cmd.jsonl:";        sed 's/^/    /' "$CMD"
ok "样本就绪（文件=2 条，命令=2 条，另加 1 个 UDP 包）"

# --------------------------------------------------- 冒充小车的订阅者 ----
step "2. 起本机临时订阅者（冒充小车；只订阅，不发运动指令）"
cat > "$WORK/accept.py" <<'PYEOF'
import json, os, sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

def qos(rel):
    return QoSProfile(depth=10,
                      reliability=ReliabilityPolicy.RELIABLE if rel == "reliable" else ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)

out = open(os.environ["ACCEPTED"], "a", buffering=1)
rclpy.init()
n = rclpy.create_node("dzsmoke_fake_car")
state = {"to_car": 0, "route": 0, "hb_seq": [], "types": {}}

def on_to_car(m):
    state["to_car"] += 1
    try:
        t = json.loads(m.data).get("type", "?")
        state["types"][t] = state["types"].get(t, 0) + 1
        if t == "heartbeat":
            state["hb_seq"].append(json.loads(m.data).get("seq"))
    except Exception:
        t = "non-json"
    out.write(json.dumps({"topic": "/dzlink/drone_to_car", "type": t, "data": m.data}) + "\n")
    print(f"  ← to_car #{state['to_car']} [{t}] {m.data[:150]}", flush=True)

def on_route(m):
    state["route"] += 1
    out.write(json.dumps({"topic": "/nav/waypoint", "type": "route", "data": m.data}) + "\n")
    print(f"  ← waypoint #{state['route']} {m.data[:150]}", flush=True)

n.create_subscription(String, "/dzlink/drone_to_car", on_to_car, qos("best_effort"))
n.create_subscription(String, "/nav/waypoint", on_route, qos("reliable"))
print(f"  [fake-car] 已订阅 /dzlink/drone_to_car(BEST_EFFORT) 与 /nav/waypoint(RELIABLE)，"
      f"域={os.environ.get('ROS_DOMAIN_ID')}", flush=True)
end = time.time() + float(os.environ.get("ACCEPT_S", "20"))
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
out.write(json.dumps({"summary": state}) + "\n")
print(f"  [fake-car] 汇总: {state}", flush=True)
n.destroy_node()
rclpy.shutdown()
PYEOF

ACCEPTED="$ACCEPTED" ACCEPT_S=$((RUN_S + 6)) python3 "$WORK/accept.py" > "$WORK/accept.log" 2>&1 &
ACCEPT_PID=$!
sleep 2.5
grep -q "已订阅" "$WORK/accept.log" && ok "临时订阅者起来了" || bad "临时订阅者没起来"
sed 's/^/    /' "$WORK/accept.log"

# ------------------------------------------------------- 起 supervisor ----
step "3. 起 dzlink_supervisor run（$RUN_S 秒）"
python3 "$SUP" run \
  --domain "$DOMAIN" \
  --perception-file "$PERC" \
  --perception-from-start \
  --perception-udp "$UDP_PORT" \
  --cmd-file "$CMD" \
  --cmd-from-start \
  --log-file "$LOG" \
  --status-period 5 \
  --bridge-period 2 \
  --duration "$RUN_S" > "$SUPOUT" 2>&1 &
SUP_PID=$!
sleep 3

# ③-b UDP 输入：给 supervisor 那 3 秒后再发一包
python3 - "$UDP_PORT" <<'PYEOF'
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(b'{"type":"bridge_passable","half":"upper","block":"K4","confidence":0.88}',
         ("127.0.0.1", int(sys.argv[1])))
print(f"  [udp] 已向 127.0.0.1:{sys.argv[1]} 发一包 bridge_passable upper=K4")
PYEOF
note "supervisor 前 3 秒输出："
sed 's/^/    /' "$SUPOUT" | head -30

# ------------------------------------------------------- 反向 ack ----
step "4. 冒充小车回 ack（证明回传方向与台账都对）"
HB_SEQ="$(python3 - "$ACCEPTED" <<'PYEOF'
import json, sys
best = None
for line in open(sys.argv[1], encoding="utf-8"):
    try:
        rec = json.loads(line)
    except Exception:
        continue
    if rec.get("type") == "heartbeat":
        try:
            best = json.loads(rec["data"]).get("seq")
        except Exception:
            pass
print(best if best is not None else "")
PYEOF
)"
if [ -z "$HB_SEQ" ]; then
  bad "没抓到 supervisor 的心跳 seq（订阅者没收到心跳？）"
else
  ok "抓到心跳 seq=$HB_SEQ"
  python3 - "$HB_SEQ" "$DOMAIN" <<'PYEOF' > "$WORK/ack.log" 2>&1
import sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
seq = int(sys.argv[1])
rclpy.init()
n = rclpy.create_node("dzsmoke_fake_car_ack")
q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
               durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
p = n.create_publisher(String, "/dzlink/car_to_drone", q)
end = time.time() + 4
sent = 0
while time.time() < end:
    m = String(); m.data = '{"type":"ack","of":%d,"ts":%.3f}' % (seq, time.time())
    p.publish(m); sent += 1
    rclpy.spin_once(n, timeout_sec=0.4)
print(f"  [fake-car] 已向 /dzlink/car_to_drone 发 {sent} 条 ack of={seq}")
n.destroy_node(); rclpy.shutdown()
PYEOF
  sed 's/^/    /' "$WORK/ack.log"
fi

step "5. 等 supervisor 跑完（最多 $((RUN_S + 25)) 秒）"
WAITED=0
while kill -0 "$SUP_PID" 2>/dev/null && [ "$WAITED" -lt $((RUN_S + 25)) ]; do
  sleep 1; WAITED=$((WAITED + 1))
done
kill "$SUP_PID" 2>/dev/null
sleep 1

# ------------------------------------------------------------ 判据 ----
step "6. 判据核对"
note "supervisor stdout（尾部）："; sed 's/^/    /' "$SUPOUT" | tail -25
note "冒充小车收到（accept.log 尾部）："; sed 's/^/    /' "$WORK/accept.log" | tail -20

python3 - "$ACCEPTED" "$LOG" "$DOMAIN" <<'PYEOF' | sed 's/^/  /'
import json, sys
accepted, logp, domain = sys.argv[1], sys.argv[2], sys.argv[3]

got = {"heartbeat": 0, "bridge_passable": 0, "target_zone": 0, "field_verify": 0, "route": 0}
halves = set()
hello = []
for line in open(accepted, encoding="utf-8"):
    try:
        rec = json.loads(line)
    except Exception:
        continue
    if "summary" in rec:
        continue
    t = rec.get("type")
    if rec.get("topic") == "/nav/waypoint":
        got["route"] += 1
        hello.append(("route", rec.get("data", "")[:80]))
        continue
    if t in got:
        got[t] += 1
    if t == "bridge_passable":
        try:
            halves.add(json.loads(rec["data"])["half"])
        except Exception:
            pass
    if len(hello) < 6:
        hello.append((t, rec.get("data", "")[:110]))

kinds = set()
acks = []
intel_acked = hb_acked = 0
suspicious = False
for line in open(logp, encoding="utf-8"):
    try:
        rec = json.loads(line)
    except Exception:
        continue
    kinds.add(rec.get("kind"))
    if rec.get("kind") == "car_ack":
        acks.append(rec.get("of"))
    if rec.get("kind") in ("status", "final_status"):
        al = rec.get("ack_ledger") or {}
        intel_acked = al.get("intel_acked", intel_acked)
        hb_acked = al.get("hb_acked", hb_acked)
        suspicious = al.get("suspicious_mismatch", suspicious)

print(f"域={domain}")
print(f"fake-car 收到：{got}   桥洞半场={sorted(halves)}")
print(f"supervisor 日志种类：{sorted(k for k in kinds if k)}")
print(f"car_ack 条数={len(acks)} of序列={sorted(set(acks))[:10]}")
print(f"台账：intel_acked={intel_acked} hb_acked={hb_acked} "
      f"suspicious_mismatch={suspicious}")
for t, d in hello:
    print(f"  样本 [{t}] {d}")

fails = []
if got["heartbeat"] < 3:
    fails.append(f"心跳只收到 {got['heartbeat']} 条（应 ≥3）")
if got["bridge_passable"] < 1:
    fails.append("没收到 bridge_passable")
if got["target_zone"] < 1:
    fails.append("没收到 target_zone（命令文件没被读？）")
if got["field_verify"] < 1:
    fails.append("没收到 field_verify（文件尾随没生效？）")
if got["route"] < 1:
    fails.append("没收到 /nav/waypoint 路线（RELIABLE 通道没通？）")
if halves != {"lower", "upper"}:
    fails.append(f"桥洞半场不全：{sorted(halves)}（文件给 lower、UDP 给 upper，两条都要到）")
# ★ 回传判据必须看"对得上我们 seq"的回执，不能只看"收到过 ack"：
#   本轮的 ack 是心跳的（intel 无 ack 是正常的，冒烟里没人吃 intel）。
if not (intel_acked or hb_acked):
    fails.append(f"没有任何对得上的车回执（intel_acked=0 hb_acked=0；收到 {len(acks)} 条 ack）"
                 + ("—— 且出现 suspicious_mismatch（域号不一致？）" if suspicious else ""))
print("RESULT: " + ("PASS" if not fails else "FAIL - " + "; ".join(fails)))
sys.exit(0 if not fails else 1)
PYEOF
VERDICT=$?

step "7. 收尾"
note "临时目录：$WORK（脚本退出时已停掉三个临时进程；目录留着供你看日志）"
if [ "$VERDICT" = "0" ] && [ -z "$FAILED" ]; then
  echo "==> SMOKE PASS（域 $DOMAIN）"
  exit 0
fi
echo "==> SMOKE FAIL：${FAILED:-见上面判据}" >&2
exit 3
