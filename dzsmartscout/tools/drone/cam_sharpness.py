#!/usr/bin/env python3
# ============================================================================
# cam_sharpness.py —— 定量测"画面糊不糊"，并存下最清晰的一帧
#
# 指标：拉普拉斯方差（Laplacian variance）。对焦准确的图像边缘强 -> 方差大；
#       糊掉的图像边缘被抹平 -> 方差小。这是工业上最常用的对焦评价函数之一。
#       数量级参考（同场景同分辨率）：方差 <10 极糊 / 10~100 糊 / >100 基本清晰。
#
# 用法（机上，先 source）：
#   python3 /tmp/cam_sharpness.py [话题] [帧数] [输出png]
# 默认：/image_raw 15 /tmp/cam_sharpest.png
# ============================================================================
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/image_raw"
WANT = int(sys.argv[2]) if len(sys.argv) > 2 else 15
OUT = sys.argv[3] if len(sys.argv) > 3 else "/tmp/cam_sharpest.png"


class Sharp(Node):
    def __init__(self) -> None:
        super().__init__("cam_sharpness")
        self.bridge = CvBridge()
        self.scores: list[float] = []
        self.best = None
        self.best_score = -1.0
        self.create_subscription(Image, TOPIC, self.on_img, qos_profile_sensor_data)

    @property
    def done(self) -> bool:
        return len(self.scores) >= WANT

    def on_img(self, msg: Image) -> None:
        if self.done or not msg.data:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        score = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        self.scores.append(score)
        if score > self.best_score:
            self.best_score = score
            self.best = img


def main() -> int:
    rclpy.init()
    node = Sharp()
    t0 = time.time()
    while rclpy.ok() and not node.done and (time.time() - t0) < 30:
        rclpy.spin_once(node, timeout_sec=0.5)

    if not node.scores:
        print("结果: 30 秒内没收到帧")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    arr = np.array(node.scores)
    print(f"帧数={len(arr)}  拉普拉斯方差: mean={arr.mean():.1f} "
          f"min={arr.min():.1f} max={arr.max():.1f} 中位数={np.median(arr):.1f}")
    if node.best is not None:
        cv2.imwrite(OUT, node.best)
        print(f"最清晰一帧已存: {OUT}  (score={node.best_score:.1f}, shape={node.best.shape})")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
