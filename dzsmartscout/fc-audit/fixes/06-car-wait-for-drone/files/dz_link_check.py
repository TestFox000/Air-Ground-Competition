#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dz_link_check —— 无人机侧「发指令前后」的链路体检 + 小车回执抓取。

为什么要有它：
  小车 `path_executor.py:586` 自己执行了
      self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
  所以它所有 info() 级日志（`Odometry reset to N10 facing y`、
  `Path Executor Node Initialized`、`Loaded N path points`）在 7_path.log 里
  【完全看不到】。现场可见的只有 warn/error 两级，例如：
      ★自动出发【已关闭】...        (warn)
      >>>  任 务 开 始  <<<        (warn)
      Path file not found: ...      (error)
  本工具从 /rosout 远程抓这些回执，不必 SSH 到小车 tail 日志。

三种用法：
  # ① 全流程体检（默认）：看订阅者数量 + 发一条【不产生位移】的探针 + 抓回执
  python3 dz_link_check.py

  # ② 只监听小车日志 N 秒（在另一个终端发完路线后用这个看收据）
  python3 dz_link_check.py --watch 60

  # ③ 只报订阅者数量（快速判断车起栈没有）
  python3 dz_link_check.py --count

判据：
  · 订阅者数量：稳定态 = 2（path_executor_node + serial_reader_node，都 RELIABLE）
  · 探针回执：小车日志出现 `Path file not found: /tmp/dzlink_preflight_no_such_file.txt`
    ⇒ 说明「无人机 → /nav/waypoint → 小车回调」整条链路真的通了

退出码：0 通过 / 3 未通过（无订阅者或没抓到回执）/ 2 无 ROS 环境
"""
from __future__ import annotations

import argparse
import sys
import time

TOPIC_ROUTE = "/nav/waypoint"
TOPIC_ROSOUT = "/rosout"
PROBE_TEXT = "load:/tmp/dzlink_preflight_no_such_file.txt"   # 只会触发 error 日志，不产生任何位移


def make_qos(reliability: str):
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                           HistoryPolicy)
    rel = (ReliabilityPolicy.RELIABLE if reliability == "reliable"
           else ReliabilityPolicy.BEST_EFFORT)
    return QoSProfile(depth=200, reliability=rel,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST)


def rosout_topic():
    return TOPIC_ROSOUT


def main(argv):
    ap = argparse.ArgumentParser(description="无人机侧链路体检 + 小车回执抓取")
    ap.add_argument("--watch", type=float, default=0.0, help="只监听小车日志 N 秒")
    ap.add_argument("--count", action="store_true", help="只报订阅者数量")
    ap.add_argument("--settle", type=float, default=3.0, help="等 DDS 发现收敛的秒数")
    ap.add_argument("--capture", type=float, default=6.0, help="发探针后抓回执的秒数")
    ap.add_argument("--all", action="store_true",
                    help="连 INFO 级小车日志一起显示（默认只看 warn/error，避免 serial_reader 5Hz 刷屏）")
    a = ap.parse_args(argv)

    import rclpy
    from std_msgs.msg import String
    from rcl_interfaces.msg import Log

    rclpy.init()
    node = rclpy.create_node("dz_link_check")

    got = {"car_logs": [], "probe_hit": False, "quiet": 0}

    def on_log(msg):
        name = getattr(msg, "name", "") or ""
        text = getattr(msg, "msg", "") or ""
        if "path_executor" not in name and "serial_reader" not in name:
            return
        # 探针识别放在过滤之前，保证任何级别都能识别
        if "Path file not found" in text and "dzlink_preflight" in text:
            got["probe_hit"] = True
        # 默认只显示 warn(30)/error(40)/fatal(50)：
        # serial_reader 会以 5Hz 打 INFO 位置行，不过滤会把输出淹掉。
        lvl = int(getattr(msg, "level", 20) or 20)
        if lvl < 30 and not a.all:
            got["quiet"] += 1
            return
        got["car_logs"].append("[%s] %s" % (name, text))
        print("  ← 小车日志 [%s] %s" % (name, text))

    try:
        from rclpy.qos import qos_profile_rosout_default
        sub_qos = qos_profile_rosout_default
    except ImportError:
        sub_qos = make_qos("reliable")
    node.create_subscription(Log, rosout_topic(), on_log, sub_qos)

    pub = node.create_publisher(String, TOPIC_ROUTE, make_qos("reliable"))

    # ---- 等发现收敛 -------------------------------------------------------
    end = time.time() + max(0.0, a.settle)
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    n = pub.get_subscription_count()
    print("[1] %s 订阅者数量 = %d   （期望 2：path_executor_node + serial_reader_node）" % (TOPIC_ROUTE, n))
    ok_count = n >= 1
    if n == 0:
        print("    ✗ 一个订阅者都没有 —— 小车没起栈，或两端 ROS_DOMAIN_ID 不一致")
        print("      车侧起栈后 7_path.log 开头应出现：★自动出发【已关闭】...")
    elif n == 1:
        print("    ⚠️ 只看到 1 个（发现尚未收敛，或 serial_reader 未起）")
    else:
        print("    ✓ 两个订阅者都在")

    if a.count:
        node.destroy_node()
        rclpy.shutdown()
        return 0 if ok_count else 3

    # ---- 只监听模式 -------------------------------------------------------
    if a.watch > 0:
        print("[watch] 监听小车日志 %.0fs …（小车 warn/error 级日志会实时打在这里）" % a.watch)
        end = time.time() + a.watch
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        print("[watch] 抓到 %d 条小车日志（另有 %d 条 INFO 被过滤，加 --all 可看）"
              % (len(got["car_logs"]), got["quiet"]))
        node.destroy_node()
        rclpy.shutdown()
        return 0

    # ---- 探针模式：发一条不产生位移的命令，看小车是否回执 -------------------
    print("[2] 发探针 %r（只触发 error 日志，车不会动）" % PROBE_TEXT)
    for _ in range(3):
        m = String()
        m.data = PROBE_TEXT
        pub.publish(m)
        rclpy.spin_once(node, timeout_sec=0.2)
    end = time.time() + max(1.0, a.capture)
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()

    print("[3] 结果：（本次另有 %d 条 INFO 级小车日志被过滤）" % got["quiet"])
    if got["probe_hit"]:
        print("    ✓ PASS —— 小车回执已抓到：「无人机 → /nav/waypoint → path_executor 回调」整条链路是通的")
        return 0
    print("    ✗ FAIL —— 没抓到小车对探针的回执。逐条排查：")
    print("      ① 小车起栈了吗？  ros2 topic info /nav/waypoint  （Subscription 应为 2）")
    print("      ② 域号一致吗？    车 start_all.sh 打印的 ROS_DOMAIN_ID 应为 118")
    print("      ③ 组播可用吗？    ping 通不等于 DDS 通；见 APPLY.md 第五节（可换单播配置）")
    print("      ④ 也可能 /rosout 跨机不可见：这时到车上 tail -f ~/robot_logs/7_path.log 再看")
    return 3


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except (ModuleNotFoundError, ImportError) as exc:
        print("ERROR: 没有可用的 ROS 2 Python 环境（%s）" % exc, file=sys.stderr)
        print("       先执行: source /opt/ros/humble/setup.bash", file=sys.stderr)
        raise SystemExit(2)
