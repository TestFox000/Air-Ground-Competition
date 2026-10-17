#!/usr/bin/env python3
"""无人机侧探针：先等 /nav/waypoint 出现订阅者（打印匹配数），再发若干条。

用法：ROS_DOMAIN_ID=118 python3 drone_send.py "N10 M10 L10 L9 K9" [等待秒数] [发送次数]
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PAYLOAD = sys.argv[1] if len(sys.argv) > 1 else 'N10 M10 L10 L9 K9'
WAIT = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
COUNT = int(sys.argv[3]) if len(sys.argv) > 3 else 10
DOMAIN = os.environ.get('ROS_DOMAIN_ID', 'unset')


def main():
    rclpy.init()
    node = Node('dz_drone_probe')
    pub = node.create_publisher(String, '/nav/waypoint', 10)

    t0 = time.time()
    matched = 0
    while time.time() - t0 < WAIT and matched == 0:
        rclpy.spin_once(node, timeout_sec=0.2)
        matched = pub.get_subscription_count()
    print('MATCHED_SUBSCRIPTIONS=%d after %.1fs (domain=%s)'
          % (matched, time.time() - t0, DOMAIN), flush=True)

    for i in range(COUNT):
        msg = String()
        msg.data = PAYLOAD
        pub.publish(msg)
        print('PUBLISHED#%d %r subs=%d'
              % (i + 1, PAYLOAD, pub.get_subscription_count()), flush=True)
        time.sleep(0.5)
    print('DONE sent=%d matched=%d' % (COUNT, pub.get_subscription_count()), flush=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
