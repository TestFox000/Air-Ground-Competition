import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from std_msgs.msg import String

GOT = []


def on_msg(msg):
    GOT.append(msg.data)
    print('RECEIVED: ' + str(msg.data), flush=True)


def main():
    rclpy.init()
    node = Node('dz_link_probe')
    qos = QoSProfile(depth=10,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(String, '/nav/waypoint', on_msg, qos)
    print('PROBE_READY domain=' + str(os.environ.get('ROS_DOMAIN_ID')), flush=True)
    end = time.time() + 120
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
    print('RESULT received=' + str(len(GOT)), flush=True)
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    print('PROBE_EXIT', flush=True)


main()
