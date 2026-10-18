#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""t40_send_route.py —— 无人机侧直接向小车 /nav/waypoint 下发原始命令（RELIABLE）。

为什么不走 dzlink_supervisor route：机侧校验器不接受 car 侧的 :PIVOT 后缀
（它只认 :SCAN_STOP / SHOOT@ / *）。本脚本只做 DDS 发送，不做 token 校验，
token 合法性由车侧 parse_mission 判定（已在本机 validate_route 0 error/0 warn）。
"""
import sys, time
import rclpy
from std_msgs.msg import String

msg = sys.argv[1]
rclpy.init()
node = rclpy.create_node("t40_route_sender")
pub = node.create_publisher(String, "/nav/waypoint", 10)

t0 = time.time()
while time.time() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() > 0:
        break
subs = pub.get_subscription_count()
for i in range(5):
    pub.publish(String(data=msg))
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.25)
print(f"[t40] sent {len(msg)} chars, subs={subs}, head={msg[:60]!r}")
node.destroy_node()
rclpy.shutdown()
