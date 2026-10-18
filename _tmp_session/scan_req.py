#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""发 /nav/scan_request（std_msgs/UInt8）。
用法: python3 scan_req.py <value> <hz> <seconds>
  value: 0=停扫/回正  1=单趟全向  2=持续扫描（起转寻靶）
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


def main():
    val = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    dur = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0
    rclpy.init()
    n = Node('scan_req_pub')
    pub = n.create_publisher(UInt8, '/nav/scan_request', 10)
    msg = UInt8()
    msg.data = val
    n0 = 0
    end = time.time() + dur
    period = 1.0 / hz
    while time.time() < end:
        pub.publish(msg)
        n0 += 1
        time.sleep(period)
    print('published %d x scan_request=%d' % (n0, val))
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
