#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t4_qos_info.py —— 把「对端端点实际声明的 QoS」抓下来（DDS 发现层直接证据）

为什么比"匹配数"可靠：
  `node.count_subscribers()` 是**发现层**计数 —— 只要对端端点被 SPDP/SEDP 发现到就会 +1，
  哪怕 QoS 不兼容（BEST_EFFORT 发 → RELIABLE 收）也照样显示 1。所以"匹配数=1"不能证明能收到。
  本工具用 `get_publishers_info_by_topic()` / `get_subscriptions_info_by_topic()` 把**对端
  端点携带的 QoSProfile 原样打印**（reliability / durability / history / depth），
  再配合 t4_qos_probe.py 的实收条数，两者一起才能完整证明"契约档位在真机上成立"。

用法：
  python3 t4_qos_info.py --topic /dzlink/drone_to_car --role sub --qos best_effort --duration 12
  python3 t4_qos_info.py --topic /nav/waypoint       --role sub --qos reliable    --duration 12
"""
import argparse
import json
import os
import socket
import sys
import time


def qos_of(kind):
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                           DurabilityPolicy)
    rel = (ReliabilityPolicy.RELIABLE if kind == "reliable"
           else ReliabilityPolicy.BEST_EFFORT)
    return QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                      reliability=rel, durability=DurabilityPolicy.VOLATILE)


def ep_to_dict(e):
    q = e.qos_profile
    return {"node_name": e.node_name, "node_namespace": e.node_namespace,
            "topic_type": e.topic_type,
            "reliability": str(q.reliability).split(".")[-1],
            "durability": str(q.durability).split(".")[-1],
            "history": str(q.history).split(".")[-1],
            "depth": q.depth, "gid": list(e.endpoint_gid)[:8]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True)
    ap.add_argument("--role", choices=["sub", "pub"], default="sub")
    ap.add_argument("--qos", choices=["reliable", "best_effort"], required=True)
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--msg-type", default="std_msgs/String")
    ap.add_argument("--out", default=None)
    ap.add_argument("--domain", type=int, default=None)
    a = ap.parse_args()
    if a.domain is not None:
        os.environ["ROS_DOMAIN_ID"] = str(a.domain)

    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node("t4_qos_info")
    qos = qos_of(a.qos)
    if a.role == "sub":
        node.create_subscription(String, a.topic, lambda m: None, qos)
    else:
        node.create_publisher(String, a.topic, qos)

    res = {"host": socket.gethostname(), "topic": a.topic, "role": a.role,
           "self_qos": a.qos, "domain_id": os.environ.get("ROS_DOMAIN_ID", "<unset→0>"),
           "publishers_seen": [], "subscribers_seen": []}
    t0 = time.time()
    while time.time() - t0 < a.duration:
        try:
            res["publishers_seen"] = [ep_to_dict(e) for e in
                                      node.get_publishers_info_by_topic(a.topic)]
            res["subscribers_seen"] = [ep_to_dict(e) for e in
                                       node.get_subscriptions_info_by_topic(a.topic)]
        except Exception as e:
            res["error"] = "%s: %s" % (type(e).__name__, e)
        time.sleep(0.5)
    res["graph_publisher_count"] = node.count_publishers(a.topic)
    res["graph_subscriber_count"] = node.count_subscribers(a.topic)
    txt = json.dumps(res, ensure_ascii=False, indent=2)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt)
    print(txt)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
