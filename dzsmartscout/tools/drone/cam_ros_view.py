#!/usr/bin/env python3
# ============================================================================
# cam_ros_view.py —— 在 VNC 桌面上实时看 ROS 相机画面（窗口可缩放，不占设备）
#
# 为什么不用 rqt_image_view：它的窗口默认 200x105，而机上没有 wmctrl /
# xdotool / python-Xlib，无法从外部调整窗口大小 -> 截图上根本看不清。
# 这里用 OpenCV 的 WINDOW_NORMAL + resizeWindow，尺寸由脚本自己定。
#
# 它是**订阅者**：不打开 /dev/video0，所以可以和 v4l2_camera_node 同时跑。
#
# 用法（机上）：
#   source /opt/ros/humble/setup.bash
#   DISPLAY=:1 python3 /tmp/cam_ros_view.py [话题] [宽] [高]
# 默认：/image_raw 1280 720      按 q 或 ESC 退出，Ctrl-C 也行
# ============================================================================
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/image_raw"
WIN_W = int(sys.argv[2]) if len(sys.argv) > 2 else 1280
WIN_H = int(sys.argv[3]) if len(sys.argv) > 3 else 720
WIN = "cam_ros_view"


class Viewer(Node):
    def __init__(self) -> None:
        super().__init__("cam_ros_view")
        self.bridge = CvBridge()
        self.frames = 0
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, WIN_W, WIN_H)
        self.create_subscription(Image, TOPIC, self.on_img, qos_profile_sensor_data)
        self.create_timer(2.0, self.report)

    def on_img(self, msg: Image) -> None:
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 - 转换失败不该让窗口死掉
            self.get_logger().warn(f"转换失败: {exc}")
            return
        self.frames += 1
        cv2.putText(img, f"{TOPIC}  {msg.width}x{msg.height} {msg.encoding}  #{self.frames}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(WIN, img)
        cv2.waitKey(1)

    def report(self) -> None:
        if self.frames == 0:
            self.get_logger().info(f"还没收到 {TOPIC} 的帧，等待中…")


def main() -> int:
    rclpy.init()
    node = Viewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
