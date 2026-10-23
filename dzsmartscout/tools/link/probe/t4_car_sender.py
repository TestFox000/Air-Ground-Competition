#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t4_car_sender.py —— 车侧**只读**模拟发送端（放小车 /tmp 跑，不改车任何文件）

它按 dzbrain `tasks/drone.py` 的真实代码逐字复刻两种上行报文体：
  · ack        : {"type":"ack","of":<seq>,"ts":<round(now,3)>}
                 （drone.py:_maybe_ack，compact separators）
  · car_state  : {"type":"car_state","block":B,"yaw_deg":Y,"phase":P,"ts":T,"seq":N}
                 （drone.py:_send_car_state，seq 是**车自己的**单调计数，不回声无人机 seq）
发布话题 /dzlink/car_to_drone，QoS = 车侧契约 KEEP_LAST(10)/BEST_EFFORT/VOLATILE。

用途：车栈保持冷态时，用它在真机上产生**真实形状**的上行流量，
      让无人机侧用生产代码（dzlink_supervisor listen）验证解析。

用法：
  python3 t4_car_sender.py --ack-of 7 --car-state --period 0.7 --duration 20 --out /tmp/t4_car_sent.jsonl
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/dzlink/car_to_drone")
    ap.add_argument("--qos", choices=["reliable", "best_effort"], default="best_effort")
    ap.add_argument("--ack-of", type=int, default=None, help="要回的 seq（可多次）", action="append")
    ap.add_argument("--car-state", action="store_true")
    ap.add_argument("--block", default="K9")
    ap.add_argument("--yaw-deg", type=float, default=180.0)
    ap.add_argument("--phase", default="DRIVE")
    ap.add_argument("--period", type=float, default=0.7)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--domain", type=int, default=None)
    a = ap.parse_args()

    if a.domain is not None:
        os.environ["ROS_DOMAIN_ID"] = str(a.domain)

    import rclpy
    rclpy.init()
    node = rclpy.create_node("t4_car_sender")
    qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE,
                     reliability=(ReliabilityPolicy.RELIABLE if a.qos == "reliable"
                                  else ReliabilityPolicy.BEST_EFFORT))
    pub = node.create_publisher(String, a.topic, qos)
    fh = open(a.out, "w") if a.out else None

    sent = []
    t0 = time.time()
    subs_max = 0
    seq = 1
    next_state = t0
    ack_queue = list(a.ack_of or [])

    while time.time() - t0 < a.duration:
        now = time.time()
        subs_max = max(subs_max, node.count_subscribers(a.topic))
        if ack_queue:
            of = ack_queue.pop(0)
            body = json.dumps({"type": "ack", "of": of, "ts": round(now, 3)},
                              ensure_ascii=False, separators=(",", ":"))
            m = String(); m.data = body; pub.publish(m)
            sent.append(body)
            print("[tx ack ] " + body, flush=True)
            next_state = now + 0.4
        elif a.car_state and now >= next_state:
            body = json.dumps({"type": "car_state", "block": a.block,
                               "yaw_deg": round(a.yaw_deg, 1), "phase": a.phase,
                               "ts": round(now, 3), "seq": seq},
                              ensure_ascii=False, separators=(",", ":"))
            m = String(); m.data = body; pub.publish(m)
            sent.append(body)
            print("[tx state] " + body, flush=True)
            seq += 1
            next_state = now + a.period
        if fh:
            fh.write(body + "\n")
            fh.flush()
        rclpy.spin_once(node, timeout_sec=0.05)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
        subs_max = max(subs_max, node.count_subscribers(a.topic))

    summary = {"host": socket.gethostname(), "topic": a.topic, "qos": a.qos,
               "domain_id": os.environ.get("ROS_DOMAIN_ID", "<unset→0>"),
               "sent": len(sent), "bodies": sent,
               "subscribers_max": subs_max,
               "verdict": "MATCHED" if subs_max >= 1 else "NO_MATCH",
               "duration_actual": round(time.time() - t0, 2)}
    print("== SUMMARY ==", flush=True)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    if a.out:
        with open(a.out + ".summary.json", "w") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        fh.close()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
