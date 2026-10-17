#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dzlink_seq_beacon —— 极轻量单调序号信标（跨机丢包精确对账用）

为什么需要它：ROS 2 的 std_msgs/Header 已无 seq 字段，IMU/PointCloud2 本身
也不带序号，无法做「逐条」丢包统计。这里发一路 50Hz 的 Int64 递增计数：

  对端收到后：span = max_seq - min_seq + 1
             丢包率 = 1 - recv / span        ← 精确，不依赖时钟

只做发布（无重负载回调），CPU 占用可忽略，绝不反压驱动。
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Int64


class SeqBeacon(Node):
    def __init__(self):
        super().__init__('dzlink_seq_beacon')
        self.declare_parameter('seq_rate', 50.0)
        self.declare_parameter('seq_topic', '/dzlink/seq')

        rate = float(self.get_parameter('seq_rate').value)
        topic = self.get_parameter('seq_topic').value

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Int64, topic, qos)

        self.seq = 0
        self.t0 = time.time()
        self.last_log = self.t0
        self.last_seq = 0
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('seq beacon up: %s @ %.1f Hz' % (topic, rate))

    def tick(self):
        m = Int64()
        m.data = self.seq
        self.pub.publish(m)
        self.seq += 1

        now = time.time()
        if now - self.last_log >= 5.0:
            dur = now - self.last_log
            hz = (self.seq - self.last_seq) / dur
            self.get_logger().info(
                'STAT seq=%d pub_hz=%.2f subs=%d uptime=%.0fs'
                % (self.seq, hz, self.pub.get_subscription_count(), now - self.t0))
            self.last_log = now
            self.last_seq = self.seq


def main():
    rclpy.init()
    node = SeqBeacon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
