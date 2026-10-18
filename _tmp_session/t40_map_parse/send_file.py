#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, time
import rclpy
from std_msgs.msg import String
path = sys.argv[1]
msg = open(path, encoding='utf-8').read().strip()
print('load len', len(msg), 'head', msg[:40], 'tail', msg[-20:])
rclpy.init()
node = rclpy.create_node('t22_route_sender')
pub = node.create_publisher(String, '/nav/waypoint', 10)
t0 = time.time()
while time.time() - t0 < 6.0:
    rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() > 0:
        break
subs = pub.get_subscription_count()
for _ in range(5):
    pub.publish(String(data=msg))
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.25)
print('sent len=%d subs=%d' % (len(msg), subs))
node.destroy_node()
rclpy.shutdown()

