#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t4_car_consumer.py —— 车侧**只读**消费者（放在小车 /tmp 跑，不改车任何文件）

它不是 dzbrain，而是按 dzbrain `tasks/drone.py` 的真实解析逻辑复刻的一只"眼睛"：
  · 订阅 /dzlink/drone_to_car，QoS 与车侧契约一致：KEEP_LAST(10)/BEST_EFFORT/VOLATILE
  · 逐条 json.loads → 按 type 分派，抽出 dzbrain 真正会读的字段
  · 复刻它的合法性判定（bridge half∈{lower,upper} 且 block 非空；blocks/mines/obstacles 必须是非空列表）
  · 把「收到/未收到 + 逐字段值」打成 JSONL + stdout，作为双向对账的车侧证据

用法：
  python3 t4_car_consumer.py --duration 30 --out /tmp/t4_car_recv.jsonl
可选 --qos reliable 用来做**反向不兼容**对照（车侧真实档位是 best_effort）。
"""
import argparse
import json
import os
import socket
import sys
import time

from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from std_msgs.msg import String

KNOWN = ("heartbeat", "bridge_passable", "target_zone", "field_verify")
BRIDGE_HALVES = ("lower", "upper")


def qos_of(kind):
    rel = (ReliabilityPolicy.RELIABLE if kind == "reliable"
           else ReliabilityPolicy.BEST_EFFORT)
    return QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                      reliability=rel, durability=DurabilityPolicy.VOLATILE)


def parse_like_dzbrain(raw):
    """返回 (rec, valid, note)。rec 里是 dzbrain 真会取到的字段。"""
    rec = {"raw": raw}
    try:
        payload = json.loads(raw)
    except Exception as e:
        return {"raw": raw, "error": "bad_json:%s" % type(e).__name__}, False, "bad_json"
    if not isinstance(payload, dict):
        return {"raw": raw, "error": "not_object"}, False, "bad_shape"
    t = str(payload.get("type") or "").strip()
    rec["type"] = t
    rec["seq"] = payload.get("seq")
    rec["ts"] = payload.get("ts")
    if t == "heartbeat":
        return rec, True, "ok"
    if t == "bridge_passable":
        half = str(payload.get("half") or "").strip().lower()
        block = str(payload.get("block") or "").strip().upper()
        rec["half"], rec["block"] = half, block
        rec["confidence"] = payload.get("confidence")
        ok = (half in BRIDGE_HALVES) and bool(block)
        return rec, ok, ("ok" if ok else "missing_half_or_block")
    if t == "target_zone":
        b = payload.get("blocks")
        rec["blocks"] = list(b) if isinstance(b, (list, tuple)) else b
        ok = isinstance(b, (list, tuple)) and len(b) > 0
        return rec, ok, ("ok" if ok else "missing_blocks")
    if t == "field_verify":
        rec["mines"] = payload.get("mines")
        rec["obstacles"] = payload.get("obstacles")
        rec["diff"] = payload.get("diff")
        has = isinstance(rec["mines"], (list, tuple)) or isinstance(rec["obstacles"], (list, tuple))
        return rec, has, ("ok" if has else "missing_mines_and_obstacles")
    return rec, False, "unknown_type"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/dzlink/drone_to_car")
    ap.add_argument("--topic-out", default="/dzlink/car_to_drone")
    ap.add_argument("--qos", choices=["reliable", "best_effort"], default="best_effort")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--domain", type=int, default=None)
    ap.add_argument("--auto-ack", action="store_true",
                    help="复刻 dzbrain `_maybe_ack`：带 seq 的消息立刻回 {\"type\":\"ack\",\"of\":seq} "
                         "到 /dzlink/car_to_drone（BEST_EFFORT），用来验证机的 ack 台账闭环")
    a = ap.parse_args()

    if a.domain is not None:
        os.environ["ROS_DOMAIN_ID"] = str(a.domain)

    import rclpy
    rclpy.init()
    node = rclpy.create_node("t4_car_consumer")
    out = open(a.out, "w") if a.out else None

    stats = {"host": socket.gethostname(), "topic": a.topic,
             "qos_requested": a.qos,
             "domain_id": os.environ.get("ROS_DOMAIN_ID", "<unset→0>"),
             "rmw": os.environ.get("RMW_IMPLEMENTATION", "<default>"),
             "t_start": round(time.time(), 3),
             "counts": {}, "valid": 0, "invalid": 0, "received": 0,
             "publishers_max": 0, "multi_publisher_seen": False, "notes": {}}
    t0 = time.time()
    last_pubcount = [None]

    ack_pub = None
    if a.auto_ack:
        # 与 dzbrain ros_io 一致：/dzlink/car_to_drone, KEEP_LAST(10)/BEST_EFFORT/VOLATILE
        ack_pub = node.create_publisher(String, a.topic_out, qos_of("best_effort"))

    def cb(msg):
        raw = msg.data
        rec, valid, note = parse_like_dzbrain(raw)
        rec["t_recv"] = round(time.time(), 3)
        rec["valid"] = valid
        rec["note"] = note
        stats["received"] += 1
        stats["counts"][rec.get("type", "?")] = stats["counts"].get(rec.get("type", "?"), 0) + 1
        stats["notes"][note] = stats["notes"].get(note, 0) + 1
        if valid:
            stats["valid"] += 1
        else:
            stats["invalid"] += 1
        # ★ 复刻 dzbrain drone.py:_maybe_ack —— 只对带 seq 的消息回执
        if ack_pub is not None and isinstance(rec.get("seq"), int):
            body = json.dumps({"type": "ack", "of": rec["seq"],
                               "ts": round(time.time(), 3)},
                              ensure_ascii=False, separators=(",", ":"))
            m = String(); m.data = body
            ack_pub.publish(m)
            stats["acks_sent"] = stats.get("acks_sent", 0) + 1
            rec["acked"] = body
        tag = "OK " if valid else "BAD"
        print("[rx %s] %s" % (tag, json.dumps(rec, ensure_ascii=False)), flush=True)
        if out:
            out.write(json.dumps(rec, ensure_ascii=False) + "\n")
            out.flush()

    node.create_subscription(String, a.topic, cb, qos_of(a.qos))
    print("== t4_car_consumer == host=%s topic=%s qos=%s domain=%s duration=%ss"
          % (stats["host"], a.topic, a.qos, stats["domain_id"], a.duration), flush=True)
    while time.time() - t0 < a.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        c = node.count_publishers(a.topic)
        if c != last_pubcount[0]:
            print("[subs] publishers on %s = %d @%.2fs" % (a.topic, c, time.time() - t0),
                  flush=True)
            last_pubcount[0] = c
        stats["publishers_max"] = max(stats["publishers_max"], c)
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        stats["publishers_max"] = max(stats["publishers_max"], node.count_publishers(a.topic))

    stats["publishers_final"] = node.count_publishers(a.topic)
    stats["multi_publisher_seen"] = stats["publishers_max"] > 1
    stats["t_end"] = round(time.time(), 3)
    stats["verdict"] = ("RECEIVED" if stats["received"] > 0 else
                        ("MATCHED_BUT_NO_DATA" if stats["publishers_max"] >= 1 else "NOT_RECEIVED"))
    print("== SUMMARY ==", flush=True)
    print(json.dumps(stats, ensure_ascii=False, indent=2), flush=True)
    if out:
        with open((a.out or "/tmp/x") + ".summary.json", "w") as f:
            json.dump(stats, f, ensure_ascii=False, indent=2)
        out.close()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
