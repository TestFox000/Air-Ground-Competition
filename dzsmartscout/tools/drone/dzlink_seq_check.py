#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dzlink_seq_check —— 序号信标「逐条」丢包精确对账（跨机，两端通用）

原理：对端 50Hz 发单调递增 Int64，本端只看序号本身，不依赖两端时钟同步：
    span = max_seq - min_seq + 1
    丢包率 = 1 - 去重后收到数 / span
并给出最长连续丢失段、丢失段个数、最大到达间隔（WiFi 卡顿证据）。

用法: python3 dzlink_seq_check.py [观测秒数, 默认 300] [话题, 默认 /dzlink/seq]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Int64


class SeqCheck(Node):
    def __init__(self, topic):
        super().__init__('dzlink_seq_check')
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Int64, topic, self.cb, qos)
        self.seen = set()
        self.t_first = None
        self.t_last = None
        self.last_t = None
        self.max_gap = 0.0
        self.max_gap_at = None

    def cb(self, m):
        now = time.time()
        v = int(m.data)
        self.seen.add(v)
        if self.t_first is None:
            self.t_first = now
        if self.last_t is not None:
            g = now - self.last_t
            if g > self.max_gap:
                self.max_gap = g
                self.max_gap_at = v
        self.last_t = now
        self.t_last = now


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    topic = sys.argv[2] if len(sys.argv) > 2 else '/dzlink/seq'

    rclpy.init()
    node = SeqCheck(topic)
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.1)
    el = time.time() - t0

    print(f'=== {topic} 序号对账（窗口 {el:.1f}s）===')
    if not node.seen:
        print('  ❌ 一条都没收到（信标未运行 / 类型或 QoS 不匹配 / 链路断了）')
        node.destroy_node()
        rclpy.shutdown()
        return

    smin, smax = min(node.seen), max(node.seen)
    span = smax - smin + 1
    recv = len(node.seen)
    loss = (1.0 - recv / span) * 100.0

    # 接收窗口：从首条到末条。若用总窗口 el，会把订阅发现握手时间算进去，
    # 导致到达率虚低（发布端 50Hz 会被报成 ~44Hz），故两者分开给出。
    win = (node.t_last - node.t_first) if (node.t_first and node.t_last) else el
    rate = recv / win if win > 0 else 0.0

    # 连续丢失段统计
    runs = []
    run = 0
    for s in range(smin, smax + 1):
        if s in node.seen:
            if run:
                runs.append(run)
                run = 0
        else:
            run += 1
    if run:
        runs.append(run)
    runs.sort(reverse=True)

    print(f'  收到(去重)={recv}  seq 范围=[{smin}, {smax}]  span={span}')
    print(f'  丢包率 = {loss:.3f}%   有效到达率 = {rate:.2f} Hz'
          f'（接收窗口 {win:.1f}s / 总窗口 {el:.1f}s）')
    print(f'  丢失段个数 = {len(runs)}   最长连续丢失 = {runs[0] if runs else 0} 条'
          f'   前 5 长丢失段 = {runs[:5]}')
    print(f'  最大到达间隔 = {node.max_gap*1000:.1f} ms (在 seq≈{node.max_gap_at} 处)')

    verdict = 'PASS' if loss <= 1.0 and (runs[0] if runs else 0) <= 10 else 'FAIL'
    print(f'  判定(丢包≤1% 且 最长连续丢失≤10 条) => {verdict}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
