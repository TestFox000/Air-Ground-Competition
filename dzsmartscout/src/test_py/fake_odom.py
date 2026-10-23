#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.02, self.timer_cb)  # 50Hz

    def timer_cb(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 1.0

        msg.pose.pose.orientation.w = 1.0

        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
