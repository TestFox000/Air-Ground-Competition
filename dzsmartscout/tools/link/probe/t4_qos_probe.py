#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t4_qos_probe.py —— DDS QoS 组合变异探针（真机用，只做一次收发，不碰任何配置）

为什么需要它：
  ROS 2 里 publisher/subscriber 的 ReliabilityPolicy **不匹配时不会报任何错**：
  匹配表（DDS 规范）：
      pub RELIABLE    + sub RELIABLE     -> 兼容，收到
      pub RELIABLE    + sub BEST_EFFORT  -> 兼容，收到（降级到 BEST_EFFORT）
      pub BEST_EFFORT + sub BEST_EFFORT  -> 兼容，收到
      pub BEST_EFFORT + sub RELIABLE     -> ★不兼容，静默收不到★
  本探针把「收到几条」和「DDS 匹配数」同时打成 JSON，两个数字一起才构成证据：
  匹配数 0 = 压根没配上（QoS 不兼容 / 域号不一致 / 组播不通）；
  匹配数 ≥1 但收到 0 = 另一类问题（比如进程没起、话题名写错）。

用法：
  t4_qos_probe.py pub --topic /t4qos --qos reliable    --rate 5 --duration 8 --tag DRONE
  t4_qos_probe.py sub --topic /t4qos --qos best_effort --duration 10 --out /tmp/r.json
  （两端各自跑；--out 写 JSON 结果，stdout 同时打印人类可读摘要）
"""
import argparse
import json
import os
import socket
import sys
import time


def build_qos(kind):
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                           DurabilityPolicy)
    rel = (ReliabilityPolicy.RELIABLE if kind == "reliable"
           else ReliabilityPolicy.BEST_EFFORT)
    return QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                      reliability=rel, durability=DurabilityPolicy.VOLATILE)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="role", required=True)
    for name in ("pub", "sub"):
        p = sub.add_parser(name)
        p.add_argument("--topic", default="/t4qos/probe")
        p.add_argument("--qos", choices=["reliable", "best_effort"], required=True)
        p.add_argument("--duration", type=float, default=8.0)
        p.add_argument("--out", default=None)
        p.add_argument("--domain", type=int, default=None)
        p.add_argument("--rate", type=float, default=5.0)
        p.add_argument("--tag", default=socket.gethostname())
        p.add_argument("--payload", default=None,
                       help="显式指定要发的内容（默认 '{tag}#i'）")
    a = ap.parse_args()

    if a.domain is not None:
        os.environ["ROS_DOMAIN_ID"] = str(a.domain)

    import rclpy
    from std_msgs.msg import String

    qos = build_qos(a.qos)
    rclpy.init()
    node = rclpy.create_node("t4_qos_%s" % a.role)
    res = {
        "role": a.role, "host": socket.gethostname(), "topic": a.topic,
        "qos_requested": a.qos, "domain_id": os.environ.get("ROS_DOMAIN_ID", "<unset→0>"),
        "rmw": os.environ.get("RMW_IMPLEMENTATION", "<default rmw_fastrtps_cpp>"),
        "peer_count_timeline": [], "peer_count_max": 0,
        "sent": 0, "received": 0, "samples": [], "tags_seen": [],
        "t_start": time.time(),
    }
    t0 = time.time()

    if a.role == "pub":
        pub = node.create_publisher(String, a.topic, qos)
        i = 0
        next_s = t0
        while time.time() - t0 < a.duration:
            c = node.count_subscribers(a.topic)
            res["peer_count_timeline"].append([round(time.time() - t0, 2), c])
            res["peer_count_max"] = max(res["peer_count_max"], c)
            now = time.time()
            if now >= next_s:
                m = String()
                body = a.payload if a.payload is not None else "%s#%d" % (a.tag, i)
                m.data = body
                pub.publish(m)
                if res["sent"] == 0:
                    res["samples"].append(body)
                i += 1
                res["sent"] += 1
                next_s += 1.0 / max(a.rate, 0.01)
            rclpy.spin_once(node, timeout_sec=0.02)
        for _ in range(20):     # 收尾期再采样几轮，等匹配稳定
            rclpy.spin_once(node, timeout_sec=0.05)
            c = node.count_subscribers(a.topic)
            res["peer_count_timeline"].append([round(time.time() - t0, 2), c])
            res["peer_count_max"] = max(res["peer_count_max"], c)
    else:
        seen = []

        def cb(msg):
            res["received"] += 1
            seen.append(msg.data)
            if len(res["samples"]) < 5:
                res["samples"].append(msg.data)

        node.create_subscription(String, a.topic, cb, qos)
        while time.time() - t0 < a.duration:
            c = node.count_publishers(a.topic)
            res["peer_count_timeline"].append([round(time.time() - t0, 2), c])
            res["peer_count_max"] = max(res["peer_count_max"], c)
            rclpy.spin_once(node, timeout_sec=0.05)
        # 再给 1 秒把在途消息收干净
        end = time.time() + 1.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        tags = []
        for s in seen:
            t = s.split("#")[0]
            if t not in tags:
                tags.append(t)
        res["tags_seen"] = tags

    res["t_end"] = time.time()
    res["duration_actual"] = round(res["t_end"] - res["t_start"], 2)
    node.destroy_node()
    rclpy.shutdown()

    # 判据（脚本自己下结论，不留解释空间）
    if a.role == "pub":
        res["verdict"] = ("MATCHED" if res["peer_count_max"] >= 1 else "NO_MATCH")
    else:
        if res["received"] > 0:
            res["verdict"] = "RECEIVED"
        elif res["peer_count_max"] >= 1:
            res["verdict"] = "MATCHED_BUT_NO_DATA"
        else:
            res["verdict"] = "NOT_RECEIVED"

    txt = json.dumps(res, ensure_ascii=False, indent=2)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt)
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
