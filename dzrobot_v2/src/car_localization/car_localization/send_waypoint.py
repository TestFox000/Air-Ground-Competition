#! /usr/bin/env python3
"""
Send waypoint commands to the path planner.

Usage:
  python3 send_waypoint.py M10,y
  python3 send_waypoint.py M7,x
  python3 send_waypoint.py K7,y
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WaypointSender(Node):
    def __init__(self, waypoint_cmd):
        super().__init__('waypoint_sender')
        self.pub = self.create_publisher(String, '/nav/waypoint', 10)
        self.timer = self.create_timer(0.1, self.send_once)
        self.sent = False
        self.waypoint_cmd = waypoint_cmd

    def send_once(self):
        if self.sent:
            self.timer.cancel()
            rclpy.shutdown()
            return

        msg = String()
        msg.data = self.waypoint_cmd
        self.pub.publish(msg)
        self.get_logger().info(f"Sent waypoint: {msg.data}")
        self.sent = True


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 send_waypoint.py BLOCK_NAME,HEADING")
        print("  Example: python3 send_waypoint.py M10,y")
        print("  Heading: 'x' = X axis positive, 'y' = Y axis positive")
        sys.exit(1)

    waypoint_cmd = f"{sys.argv[1]},{sys.argv[2]}"

    rclpy.init()
    node = WaypointSender(waypoint_cmd)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
