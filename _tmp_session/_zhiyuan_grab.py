#!/usr/bin/env python3
# 纸鸢夜巡：抓一帧相机画面到 /tmp（只读探针，不碰任何节点）。
import os
os.environ.setdefault('ROS_DOMAIN_ID', '118')
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
try:
    from cv_bridge import CvBridge
    import cv2
    HAVE_CV = True
except Exception:
    HAVE_CV = False

class Grab(Node):
    def __init__(self):
        super().__init__('zhiyuan_grab')
        self.bridge = CvBridge() if HAVE_CV else None
        self.sub = self.create_subscription(Image, '/image_raw', self.cb, 10)
        self.got = False
    def cb(self, msg):
        if self.got:
            return
        if HAVE_CV:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            ok = cv2.imwrite('/tmp/_zhiyuan_frame.jpg', img)
            print('SAVED_JPG', img.shape, 'ok=', ok)
        else:
            with open('/tmp/_zhiyuan_frame.raw', 'wb') as f:
                f.write(bytes(msg.data))
            print('SAVED_RAW', msg.width, 'x', msg.height, 'enc=', msg.encoding)
        self.got = True

rclpy.init()
g = Grab()
try:
    while rclpy.ok() and not g.got:
        rclpy.spin_once(g, timeout_sec=1.0)
finally:
    g.destroy_node()
    rclpy.shutdown()
print('DONE')
