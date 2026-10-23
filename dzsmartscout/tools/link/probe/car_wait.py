#!/usr/bin/env python3
"""车侧探针：订阅 /nav/waypoint，收到一条就打印并退出（不依赖发现时序）。

用法：ROS_DOMAIN_ID=118 python3 car_wait.py [等待秒数]
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TIMEOUT = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
DOMAIN = os.environ.get('ROS_DOMAIN_ID', 'unset')

got = []


def main():
    rclpy.init()
    node = Node('dz_car_probe')

    def cb(msg):
        got.append(msg.data)
        print('RECEIVED: %s' % msg.data, flush=True)

    node.create_subscription(String, '/nav/waypoint', cb, 10)
    print('car probe listening: domain=%s topic=/nav/waypoint timeout=%.0fs'
          % (DOMAIN, TIMEOUT), flush=True)
    t0 = time.time()
    while time.time() - t0 < TIMEOUT and not got:
        rclpy.spin_once(node, timeout_sec=0.2)
    print('RESULT received=%d waited=%.1fs' % (len(got), time.time() - t0), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
