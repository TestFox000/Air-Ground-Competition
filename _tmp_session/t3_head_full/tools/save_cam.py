# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os

class SaveImg(Node):
    def __init__(self):
        super().__init__("save_camera_image")
        self.create_subscription(Image, "/image_raw", self.cb, 5)
        self.cnt = 0
        os.makedirs("camera_dataset", exist_ok=True)

    def cb(self, msg):
        # 你的摄像头固定YUYV 双通道，直接解码
        raw_yuyv = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 2)
        # YUYV转BGR用于保存图片
        img_bgr = cv2.cvtColor(raw_yuyv, cv2.COLOR_YUV2BGR_YUY2)
        
        path = f"camera_dataset/img_{self.cnt:04d}.jpg"
        cv2.imwrite(path, img_bgr)
        self.get_logger().info(f"Saved: {path}")
        self.cnt += 1

def main():
    rclpy.init()
    node = SaveImg()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
