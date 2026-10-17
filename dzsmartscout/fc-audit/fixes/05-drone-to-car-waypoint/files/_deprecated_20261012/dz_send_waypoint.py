#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dz_send_waypoint —— 无人机侧「给小车发航点 / 发情报」发送器。

设计依据（全部读小车侧源码得到，不是猜的）：
  · 小车新决策层 dzbrain 订阅两个入口：
      /nav/waypoint          std_msgs/String  RELIABLE    depth 10   ← 直接下发路线
      /dzlink/drone_to_car   std_msgs/String  BEST_EFFORT depth 10   ← JSON 情报
    证据：小车 `src/dzbrain/dzbrain/io/contract.py:73,94`（话题表 + QoS 规格）
  · 路线命令语法来自 `src/dzbrain/dzbrain/tasks/mission.py:870-943`
  · JSON 报文字段来自 `src/dzbrain/dzbrain/tasks/drone.py:204-327`

为什么要重复发几次：两个入口都是 **VOLATILE**（非 latched）QoS。如果发的时候
小车的订阅端还没被发现/还没起栈，那一条就**永久丢了**，不会补发。所以默认发 3 次。

用法（在无人机上，先 source ROS 环境）：

  # ① 直接下发一条路线（最直白的"发航点"）
  python3 dz_send_waypoint.py route "N10 M10 L10 L9 K9 J9 J10 I10 H10 G10* F10 F11 E11 N5"

  # ② 发桥洞情报（新核据此选穿洞格；每局随机、只有无人机看得到桥顶标识）
  python3 dz_send_waypoint.py bridge lower=K9 upper=K4

  # ③ 保持心跳（3s 不上线就判离线；跑长任务时挂着它）
  python3 dz_send_waypoint.py heartbeat --duration 120

  # ④ 监听小车回传（car_state / ack）—— 证明"车收到了且活着"
  python3 dz_send_waypoint.py listen --duration 30

  # 干跑：只打印报文不发（先确认格式）
  python3 dz_send_waypoint.py bridge lower=K9 --dry-run

退出码：0 正常 / 2 无 ROS 环境 / 3 发出去了但**没有任何小车订阅者**（链路或起栈问题）
"""
from __future__ import annotations

import argparse
import json
import sys
import time

TOPIC_ROUTE = "/nav/waypoint"           # 车：路线命令入口
TOPIC_TO_CAR = "/dzlink/drone_to_car"   # 车：无人机情报入口（JSON）
TOPIC_FROM_CAR = "/dzlink/car_to_drone"  # 车：回传 car_state / ack

BANNER = """=== dz_send_waypoint ===
车侧入口   : {r}  (String, RELIABLE, depth10)
             {t}  (String, BEST_EFFORT, depth10)
车侧回传   : {f}
""".format(r=TOPIC_ROUTE, t=TOPIC_TO_CAR, f=TOPIC_FROM_CAR)


# --------------------------------------------------------------------------- #
#  QoS：必须与车侧订阅端一致，否则"发出去收不到"
# --------------------------------------------------------------------------- #
def make_qos(reliability: str):
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                           HistoryPolicy)
    rel = (ReliabilityPolicy.RELIABLE if reliability == "reliable"
           else ReliabilityPolicy.BEST_EFFORT)
    return QoSProfile(depth=10, reliability=rel,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST)


def now_s() -> float:
    return time.time()


# --------------------------------------------------------------------------- #
#  报文构造（纯函数，可 --dry-run 本地验证）
# --------------------------------------------------------------------------- #
def route_payload(text: str) -> str:
    """路线命令就是一段裸文本，不加 JSON 包装。"""
    body = " ".join(str(text).replace(";", " ").replace("|", " ").split())
    if not body:
        raise ValueError("路线为空")
    return body


def bridge_payload(half: str, block: str, confidence: float = 0.9,
                   seq: int | None = None) -> str:
    half = str(half).strip().lower()
    if half not in ("lower", "upper"):
        raise ValueError(f"half 只能是 lower/upper，收到 {half!r}")
    block = str(block).strip().upper()
    if not block:
        raise ValueError("block 不能为空（如 K9）")
    body = {"type": "bridge_passable", "half": half, "block": block,
            "confidence": float(confidence), "ts": round(now_s(), 3)}
    if seq is not None:
        body["seq"] = int(seq)
    return json.dumps(body, ensure_ascii=False, separators=(",", ":"))


def heartbeat_payload(seq: int | None = None) -> str:
    body = {"type": "heartbeat", "ts": round(now_s(), 3)}
    if seq is not None:
        body["seq"] = int(seq)
    return json.dumps(body, ensure_ascii=False, separators=(",", ":"))


def blocks_payload(kind: str, blocks: list[str], seq: int | None = None) -> str:
    """kind = target_zone | field_verify（后者还要 mines/obstacles 分开传）。"""
    body = {"type": kind, "blocks": [str(b).strip().upper() for b in blocks],
            "ts": round(now_s(), 3)}
    if seq is not None:
        body["seq"] = int(seq)
    return json.dumps(body, ensure_ascii=False, separators=(",", ":"))


def field_payload(mines: list[str] | None, obstacles: list[str] | None,
                  seq: int | None = None) -> str:
    body: dict = {"type": "field_verify", "ts": round(now_s(), 3)}
    if mines is not None:
        body["mines"] = [str(b).strip().upper() for b in mines]
    if obstacles is not None:
        body["obstacles"] = [str(b).strip().upper() for b in obstacles]
    if seq is not None:
        body["seq"] = int(seq)
    return json.dumps(body, ensure_ascii=False, separators=(",", ":"))


# --------------------------------------------------------------------------- #
#  发送
# --------------------------------------------------------------------------- #
def send(topic: str, reliability: str, texts: list[str], *, repeat: int,
         period: float, wait_match_s: float, label: str) -> int:
    """把 texts 依次发 repeat 遍；返回 0=正常，3=没有订阅者。"""
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("dz_send_waypoint")
    pub = node.create_publisher(String, topic, make_qos(reliability))
    try:
        # 等 DDS 匹配：冷启动时发现是异步的，立刻发第一条大概率丢
        deadline = time.time() + max(0.0, wait_match_s)
        while time.time() < deadline and pub.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)

        # ★ 再给发现过程 1.5s 让计数收敛：实测第一帧常常只看到 1 个订阅者，
        #   而稳定态是 2 个（path_executor_node + serial_reader_node，都是 RELIABLE）。
        #   之前直接采样会把 2 报成 1，让人误以为只匹配上一个。
        settle_end = time.time() + 1.5
        while time.time() < settle_end:
            rclpy.spin_once(node, timeout_sec=0.1)

        matched = pub.get_subscription_count()
        print(f"[{label}] topic={topic} 订阅者匹配数={matched}")
        if matched == 0:
            print("  ⚠️ 没有车侧订阅者 —— 两种可能：")
            print("     ① 小车没起栈（第 7 步 `7_path` 没跑）→ 车侧看 pgrep -af dzbrain")
            print("     ② 两端 DDS 没互相发现（域名/网段/RMW/组播）→ 见 APPLY.md 第五节")
            print("     仍然按 repeat 次数发出，供你判断是否需要重试。")

        for i in range(max(1, repeat)):
            for text in texts:
                msg = String()
                msg.data = text
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.05)
                print(f"  → [{i + 1}/{max(1, repeat)}] {text[:160]}")
            if i + 1 < max(1, repeat):
                end = time.time() + max(0.0, period)
                while time.time() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
        print(f"[{label}] 发送完成：{len(texts) * max(1, repeat)} 条")
        return 0 if matched else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


def listen(duration: float) -> int:
    """订阅 /dzlink/car_to_drone，证明车收到并活着。"""
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("dz_listen_car")
    sub = node.create_subscription(String, TOPIC_FROM_CAR, lambda m: _on_car(m),
                                   make_qos("best_effort"))
    got = {"n": 0, "last": "", "kinds": {}}

    def _on_car(m):
        got["n"] += 1
        got["last"] = m.data
        try:
            k = json.loads(m.data).get("type", "?")
        except Exception:
            k = "非JSON"
        got["kinds"][k] = got["kinds"].get(k, 0) + 1
        print(f"  ← [{got['n']}] {m.data[:200]}")

    print(f"[listen] 监听 {TOPIC_FROM_CAR} 共 {duration:.0f}s（BEST_EFFORT depth10）…")
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f"[listen] 收到 {got['n']} 条；按类型：{got['kinds'] or '无'}")
    if got["n"] == 0:
        print("  ⚠️ 一条都没收到 —— 车可能没起栈，或链路没通。")
        print("     小车侧会 1~2Hz 发 car_state（前提是 dzbrain 在跑且 drone.enabled=true）。")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if got["n"] else 3


# --------------------------------------------------------------------------- #
def parse_kv(pairs: list[str]) -> dict:
    out = {}
    for p in pairs or []:
        if "=" not in p:
            raise SystemExit(f"参数要写成 key=value，收到 {p!r}（例：lower=K9）")
        k, v = p.split("=", 1)
        out[k.strip().lower()] = v.strip()
    return out


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(
        description="无人机→小车：发路线(/nav/waypoint) 或 JSON 情报(/dzlink/drone_to_car)")
    sub = ap.add_subparsers(dest="cmd")

    def common(p):
        p.add_argument("--repeat", type=int, default=3, help="重复发送遍数（默认 3）")
        p.add_argument("--period", type=float, default=0.5, help="遍间隔秒（默认 0.5）")
        p.add_argument("--wait-match", type=float, default=3.0,
                       help="等待 DDS 匹配订阅者的秒数（默认 3）")
        p.add_argument("--dry-run", action="store_true", help="只打印不发")

    p = sub.add_parser("route", help="下发路线命令")
    p.add_argument("route", help='如 "N10 M10 L10 ... N5" 或 "多点 N10 M10" 或 "reset:N10,y"')
    common(p)

    p = sub.add_parser("bridge", help="发桥洞情报（新核据此选穿洞格）")
    p.add_argument("kv", nargs="+", help="lower=K9 upper=K4")
    p.add_argument("--confidence", type=float, default=0.9)
    p.add_argument("--seq", type=int, default=None, help="带上 seq 会让车回 ack（验收到）")
    common(p)

    p = sub.add_parser("heartbeat", help="周期心跳（默认 1Hz）")
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--hz", type=float, default=1.0)
    common(p)

    p = sub.add_parser("target", help="发靶区（blocks）")
    p.add_argument("blocks", nargs="+")
    common(p)

    p = sub.add_parser("field", help="发雷/障核对")
    p.add_argument("--mines", nargs="*", default=None)
    p.add_argument("--obstacles", nargs="*", default=None)
    common(p)

    p = sub.add_parser("listen", help="监听小车回传 car_state/ack")
    p.add_argument("--duration", type=float, default=30.0)

    sub.add_parser("contract", help="打印车侧接口契约摘要")

    args = ap.parse_args(argv)

    if not args.cmd or args.cmd == "contract":
        print(BANNER)
        print("车侧报文契约（读源码核实）:")
        print("  /nav/waypoint 路线命令语法：")
        print("    reset:N10,y            复位信念位姿 + 清路线，回 IDLE")
        print("    load:<文件名>           读路径文件并出发")
        print("    单点|single|goto N10,y  单点")
        print("    多点|multi|route A B,y  多航点（朝向 ,x/,y 只能挂最后一个 token）")
        print("    裸 token 列表  G8 A10 D2*    ← 最常用；* = 该点停车打靶")
        print("    分隔符：空格 / 分号 / 竖线")
        print("  /dzlink/drone_to_car JSON：")
        print('    {"type":"heartbeat"}')
        print('    {"type":"bridge_passable","half":"lower|upper","block":"K9","confidence":0.9}')
        print('    {"type":"target_zone","blocks":["G11","H11"]}')
        print('    {"type":"field_verify","mines":[...],"obstacles":[...]}')
        print("    带 seq 字段 → 车回 ack 到 /dzlink/car_to_drone")
        return 0

    # ---- 构造报文 ----
    if args.cmd == "route":
        texts, topic, rel, label = [route_payload(args.route)], TOPIC_ROUTE, "reliable", "route"
    elif args.cmd == "bridge":
        kv = parse_kv(args.kv)
        unknown = set(kv) - {"lower", "upper"}
        if unknown:
            raise SystemExit(f"bridge 只认 lower=/upper=，多了 {sorted(unknown)}")
        if not kv:
            raise SystemExit("至少要给一个 half，如 lower=K9")
        texts = [bridge_payload(h, b, args.confidence, args.seq)
                 for h, b in kv.items()]
        topic, rel, label = TOPIC_TO_CAR, "best_effort", "bridge"
    elif args.cmd == "heartbeat":
        texts = [heartbeat_payload()]
        topic, rel, label = TOPIC_TO_CAR, "best_effort", "heartbeat"
    elif args.cmd == "target":
        texts = [blocks_payload("target_zone", args.blocks)]
        topic, rel, label = TOPIC_TO_CAR, "best_effort", "target"
    elif args.cmd == "field":
        if args.mines is None and args.obstacles is None:
            raise SystemExit("field 至少要给 --mines 或 --obstacles")
        texts = [field_payload(args.mines, args.obstacles)]
        topic, rel, label = TOPIC_TO_CAR, "best_effort", "field"
    elif args.cmd == "listen":
        return listen(args.duration)
    else:
        ap.print_help()
        return 0

    if getattr(args, "dry_run", False):
        print(f"[dry-run] topic={topic} qos={rel} depth=10")
        for t in texts:
            print(f"  {t}")
        return 0

    if args.cmd == "heartbeat":
        # 心跳要按周期连着发，不走 send() 的 repeat 逻辑
        import rclpy
        from std_msgs.msg import String
        rclpy.init()
        node = rclpy.create_node("dz_send_waypoint")
        pub = node.create_publisher(String, topic, make_qos(rel))
        end_match = time.time() + args.wait_match
        while time.time() < end_match and pub.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        matched = pub.get_subscription_count()
        print(f"[heartbeat] topic={topic} 订阅者匹配数={matched}，"
              f"发 {args.duration:.0f}s @ {args.hz:.1f}Hz")
        period = 1.0 / max(0.1, args.hz)
        end = time.time() + args.duration
        n = 0
        while time.time() < end:
            m = String()
            m.data = heartbeat_payload(seq=n)
            pub.publish(m)
            n += 1
            t_end = time.time() + period
            while time.time() < t_end:
                rclpy.spin_once(node, timeout_sec=0.05)
        print(f"[heartbeat] 共发 {n} 条")
        node.destroy_node()
        rclpy.shutdown()
        return 0 if matched else 3

    try:
        return send(topic, rel, texts, repeat=args.repeat, period=args.period,
                    wait_match_s=args.wait_match, label=label)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except (ModuleNotFoundError, ImportError) as exc:
        print(f"ERROR: 没有可用的 ROS 2 Python 环境（{exc}）", file=sys.stderr)
        print("       先执行: source /opt/ros/humble/setup.bash", file=sys.stderr)
        raise SystemExit(2)
