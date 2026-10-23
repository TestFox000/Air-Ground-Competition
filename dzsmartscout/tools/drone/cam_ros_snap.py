#!/usr/bin/env python3
# ============================================================================
# cam_ros_snap.py —— 从 ROS 话题抓一帧存成 PNG（无人值守/无头验证用）
#
# 为什么需要它：rqt_image_view 的窗口默认只有 200x105，而机上没有
# wmctrl / xdotool / python-Xlib，没法从外部把它撑大；截图出来根本看不清。
# 直接从话题取帧存盘，才是"ROS 通路真的在传画面"的直接证据。
#
# 用法（机上，先 source 两句）：
#   source /opt/ros/humble/setup.bash
#   python3 /tmp/cam_ros_snap.py [话题] [输出png]
# 默认：/image_raw  ->  /tmp/cam_ros_frame.png
# ============================================================================
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/image_raw"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/cam_ros_frame.png"


class GrabOnce(Node):
    def __init__(self) -> None:
        super().__init__("cam_ros_snap")
        self.bridge = CvBridge()
        self.done = False
        # 用 sensor_data(BEST_EFFORT) 订阅：它能兼容 RELIABLE 和 BEST_EFFORT 两种发布者，
        # 反过来则不成立 —— 这是唯一"怎么都能收到"的 QoS 选择。
        self.create_subscription(Image, TOPIC, self.on_img, qos_profile_sensor_data)
        self.count = 0

    def on_img(self, msg: Image) -> None:
        self.count += 1
        if self.done:
            return
        if not msg.data:
            print("收到空帧（data 为空），继续等")
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok = cv2.imwrite(OUT, img)
        print(f"收到第 {self.count} 帧: encoding_in={msg.encoding} shape={img.shape} "
              f"step={msg.step} 写盘={'OK' if ok else '失败'}")
        print(f"像素统计: min={int(img.min())} max={int(img.max())} "
              f"mean_bgr={[round(float(v), 1) for v in img.reshape(-1, 3).mean(axis=0)]}")
        self.done = bool(ok)


def main() -> int:
    rclpy.init()
    node = GrabOnce()
    t0 = time.time()
    while rclpy.ok() and not node.done and (time.time() - t0) < 25:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    if node.done:
        print(f"结果: 成功 -> {OUT}")
        return 0
    print("结果: 超时，25 秒内没拿到可用帧")
    return 1


if __name__ == "__main__":
    sys.exit(main())
