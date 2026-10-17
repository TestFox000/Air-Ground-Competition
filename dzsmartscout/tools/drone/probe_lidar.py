#!/usr/bin/env python3
"""One-shot probe: subscribe to /livox/lidar (CustomMsg), report point count + frame + freshness."""
import time
import rclpy
from livox_ros_driver2.msg import CustomMsg


def main():
    rclpy.init()
    node = rclpy.create_node("lidar_probe")
    result = {}

    def cb(msg: CustomMsg):
        result["points"] = msg.point_num
        result["frame"] = msg.header.frame_id
        result["stamp"] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        result["timebase"] = msg.timebase

    node.create_subscription(CustomMsg, "/livox/lidar", cb, 10)

    t0 = time.time()
    while time.time() - t0 < 15 and "points" not in result:
        rclpy.spin_once(node, timeout_sec=0.2)

    if "points" in result:
        now = time.time()
        age = now - result["stamp"]
        print(f"OK points_per_msg={result['points']} frame={result['frame']} "
              f"timebase={result['timebase']} msg_age={age:.3f}s")
    else:
        print("FAIL no message received within 15s")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
