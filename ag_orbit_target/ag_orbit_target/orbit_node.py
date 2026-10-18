"""绕标靶圆周录像节点。

流程：
    SEARCH  未检测到标靶 → 原地慢转搜索
    APPROACH 检测到标靶 → 估算距离，开到最佳打击距离
    ORBIT   到达最佳距离 → 绕靶做圆周运动，全程录像
    LOST    绕靶中丢靶 → 停车，短时间内重新搜索

订阅：
    /offset_center  (std_msgs/Int32MultiArray, 8项)
        [框中心x偏移, 框中心y偏移, 类别(1蓝/2红/0无), 框宽, 框高, 关键点x, 关键点y, 关键点conf×100]
    /image_raw      (sensor_msgs/Image)  用于录像
发布：
    /cmd_vel        (geometry_msgs/Twist)  底盘速度

安全：enable 参数默认 false，未启用时不发布任何 /cmd_vel（只录像、打日志）。
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

try:
    from cv_bridge import CvBridge
    import cv2
    _HAS_CV = True
except ImportError:
    _HAS_CV = False


class State(Enum):
    SEARCH = "search"
    APPROACH = "approach"
    ORBIT = "orbit"
    LOST = "lost"
    STOP = "stop"


@dataclass
class Detection:
    cx_offset: int   # 框中心x相对图像中心偏移（像素）
    cy_offset: int
    cls: int         # 1=蓝 2=红 0=无
    w: int           # 框宽（像素）
    h: int
    valid: bool


class OrbitNode(Node):
    def __init__(self) -> None:
        super().__init__("orbit_node")

        # ---- 参数 ----
        self.declare_parameter("enable", False)             # 安全闸：false 时不发任何 /cmd_vel
        self.declare_parameter("target_real_width", 0.20)   # 标靶实际宽度 m
        self.declare_parameter("focal_length", 600.0)       # 相机焦距 px（640x480 典型值）
        self.declare_parameter("best_distance", 1.0)        # 最佳打击距离 m
        self.declare_parameter("distance_tolerance", 0.15)  # 到达判定容差 m
        self.declare_parameter("orbit_speed", 0.15)         # 绕行线速度 m/s
        self.declare_parameter("search_angular_speed", 0.3) # 搜索角速度 rad/s
        self.declare_parameter("approach_speed", 0.12)      # 接近线速度 m/s
        self.declare_parameter("k_visual", 0.004)           # 视觉偏移→角速度增益
        self.declare_parameter("k_dist", 0.5)               # 距离误差→线速度增益
        self.declare_parameter("max_angular", 0.8)          # 角速度上限 rad/s
        self.declare_parameter("max_linear", 0.25)          # 线速度上限 m/s
        self.declare_parameter("lost_timeout", 3.0)         # 丢靶超时 s（转回搜索）
        self.declare_parameter("search_timeout", 30.0)      # 总搜索超时 s（超时停车）
        self.declare_parameter("max_runtime", 120.0)        # 最大运行时长 s
        self.declare_parameter("record_dir", "~/orbit_videos")
        self.declare_parameter("record_fps", 20.0)          # 录像帧率 fps
        self.declare_parameter("det_freshness", 0.5)        # 目标新鲜度 s（超时视为无目标）
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("detection_topic", "/offset_center")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("target_cls", 1)             # 1=蓝靶 2=红靶

        p = lambda k: self.get_parameter(k).value
        self.enabled = bool(p("enable"))
        self.target_real_width = float(p("target_real_width"))
        self.focal_length = float(p("focal_length"))
        self.best_distance = float(p("best_distance"))
        self.distance_tolerance = float(p("distance_tolerance"))
        self.orbit_speed = float(p("orbit_speed"))
        self.search_angular_speed = float(p("search_angular_speed"))
        self.approach_speed = float(p("approach_speed"))
        self.k_visual = float(p("k_visual"))
        self.k_dist = float(p("k_dist"))
        self.max_angular = float(p("max_angular"))
        self.max_linear = float(p("max_linear"))
        self.lost_timeout = float(p("lost_timeout"))
        self.search_timeout = float(p("search_timeout"))
        self.max_runtime = float(p("max_runtime"))
        self.record_dir = os.path.expanduser(str(p("record_dir")))
        self.record_fps = float(p("record_fps"))
        self.det_freshness = float(p("det_freshness"))
        self.image_topic = str(p("image_topic"))
        self.detection_topic = str(p("detection_topic"))
        self.cmd_vel_topic = str(p("cmd_vel_topic"))
        self.target_cls = int(p("target_cls"))

        # ---- 状态 ----
        self.state = State.SEARCH
        self.latest_det: Detection | None = None
        self.last_det_time: float = 0.0
        self.state_enter_time: float = time.time()
        self.start_time: float = time.time()
        self.last_distance: float = 0.0
        self._recording = False
        self._last_enable_warn: float = 0.0   # "等待启用"日志节流
        self._last_bad_det_warn: float = 0.0  # 畸形帧日志节流

        # ---- 通信 ----
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.det_sub = self.create_subscription(
            Int32MultiArray, self.detection_topic, self._on_detection, 10
        )
        # 无条件初始化：无 cv2 环境下 _stop_recording/destroy_node 也能安全判 None
        self._video_writer = None
        self._video_failed = False  # 录像打开失败（codec 缺失）后不再每帧重试
        if _HAS_CV:
            self.bridge = CvBridge()
            self.img_sub = self.create_subscription(
                Image, self.image_topic, self._on_image, 10
            )
            os.makedirs(self.record_dir, exist_ok=True)
        else:
            self.bridge = None
            self.get_logger().warn("cv_bridge/cv2 不可用，录像功能关闭")

        self.timer = self.create_timer(0.1, self._on_tick)  # 10Hz 控制
        self.get_logger().info(
            f"orbit_node 就绪: 目标类={self.target_cls} "
            f"最佳距离={self.best_distance}m 焦距={self.focal_length}px "
            f"录像={'开' if _HAS_CV else '关'} 目录={self.record_dir} "
            f"enable={'是' if self.enabled else '否（只录像不发 /cmd_vel）'}"
        )

    # ---------------- 检测回调 ----------------
    def _on_detection(self, msg: Int32MultiArray) -> None:
        data = msg.data
        if len(data) < 8:
            # 协议约定 8 项，畸形帧直接丢弃（节流防刷日志）
            now = time.time()
            if now - self._last_bad_det_warn > 2.0:
                self._last_bad_det_warn = now
                self.get_logger().warn(f"畸形检测帧: len={len(data)}，协议应为 8 项，已丢弃")
            return
        cls = data[2]
        valid = cls == self.target_cls and data[3] > 0 and data[4] > 0
        self.latest_det = Detection(
            cx_offset=int(data[0]),
            cy_offset=int(data[1]),
            cls=cls,
            w=int(data[3]),
            h=int(data[4]),
            valid=valid,
        )
        if valid:
            self.last_det_time = time.time()

    def _on_image(self, msg: Image) -> None:
        if not _HAS_CV:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return
        if self._video_writer is None:
            if self._video_failed:
                return
            h, w = frame.shape[:2]
            ts = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self.record_dir, f"orbit_{ts}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(path, fourcc, self.record_fps, (w, h))
            if not writer.isOpened():
                writer.release()
                self._video_failed = True
                self.get_logger().warn(f"录像打开失败（可能缺 mp4 编码器），本次运行不再录像: {path}")
                return
            self._video_writer = writer
            self._recording = True
            self.get_logger().info(f"开始录像: {path}")
        if self._video_writer is not None:
            # 叠加状态信息
            self._draw_overlay(frame)
            self._video_writer.write(frame)

    def _draw_overlay(self, frame) -> None:
        h, w = frame.shape[:2]
        info = f"state={self.state.value}  dist={self.last_distance:.2f}m"
        if self.latest_det and self.latest_det.valid:
            info += f"  target=({self.latest_det.cx_offset},{self.latest_det.cy_offset})"
        cv2.putText(frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # 画中心十字
        cv2.line(frame, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (0, 0, 255), 1)
        cv2.line(frame, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (0, 0, 255), 1)

    # ---------------- 距离估算 ----------------
    def _estimate_distance(self, det: Detection) -> float:
        """由框宽像素估算距离：d = f * W_real / w_px。"""
        if det.w <= 0:
            return 0.0
        return self.focal_length * self.target_real_width / det.w

    # ---------------- 状态机 ----------------
    def _on_tick(self) -> None:
        now = time.time()

        # 安全闸：未启用时不发任何 /cmd_vel，周期提示等待启用
        if not self.enabled:
            if now - self._last_enable_warn > 5.0:
                self._last_enable_warn = now
                self.get_logger().warn("enable=false，不发 /cmd_vel；确认安全后以 -p enable:=true 启用")
            return

        # 全局超时保护（进入 STOP 后不再重复触发）
        if self.state != State.STOP and now - self.start_time > self.max_runtime:
            self.state = State.STOP
            self._publish_zero()
            self._stop_recording()
            self.get_logger().warn(f"达到最大运行时长 {self.max_runtime}s，停止")
            return

        has_target = (
            self.latest_det is not None
            and self.latest_det.valid
            and (now - self.last_det_time) < self.det_freshness
        )

        if self.state == State.SEARCH:
            self._do_search(now, has_target)
        elif self.state == State.APPROACH:
            self._do_approach(now, has_target)
        elif self.state == State.ORBIT:
            self._do_orbit(now, has_target)
        elif self.state == State.LOST:
            self._do_lost(now, has_target)
        elif self.state == State.STOP:
            self._publish_zero()

    def _do_search(self, now: float, has_target: bool) -> None:
        if has_target:
            self.state = State.APPROACH
            self.state_enter_time = now
            self.get_logger().info("检测到标靶，进入接近")
            return
        if now - self.state_enter_time > self.search_timeout:
            self.state = State.STOP
            self._publish_zero()
            self._stop_recording()
            self.get_logger().warn(f"搜索超时 {self.search_timeout}s，停止")
            return
        # 原地慢转搜索
        self._publish_twist(0.0, self.search_angular_speed)

    def _do_approach(self, now: float, has_target: bool) -> None:
        if not has_target:
            self.state = State.LOST
            self.state_enter_time = now
            return
        det = self.latest_det
        dist = self._estimate_distance(det)
        self.last_distance = dist

        if abs(dist - self.best_distance) <= self.distance_tolerance:
            self.state = State.ORBIT
            self.state_enter_time = now
            self.get_logger().info(f"到达最佳距离 {dist:.2f}m，进入绕行")
            return

        # 距离修正线速度 + 视觉修正角速度
        v = self.k_dist * (dist - self.best_distance)
        v = max(-self.approach_speed, min(self.approach_speed, v))
        w = -self.k_visual * det.cx_offset  # 目标偏左→左转
        w = max(-self.max_angular, min(self.max_angular, w))
        self._publish_twist(v, w)

    def _do_orbit(self, now: float, has_target: bool) -> None:
        if not has_target:
            if now - self.last_det_time > self.lost_timeout:
                self.state = State.LOST
                self.state_enter_time = now
                self.get_logger().warn("绕行中丢靶，进入丢失态")
            else:
                # 短暂丢靶，保持当前圆周运动
                w = self.orbit_speed / self.best_distance
                self._publish_twist(self.orbit_speed, w)
            return

        det = self.latest_det
        dist = self._estimate_distance(det)
        self.last_distance = dist

        # 基础圆周角速度
        w_base = self.orbit_speed / self.best_distance
        # 视觉修正：目标偏左(cx<0)→增大左转角速度
        w_visual = -self.k_visual * det.cx_offset
        w = w_base + w_visual
        w = max(-self.max_angular, min(self.max_angular, w))

        # 距离修正线速度
        v = self.orbit_speed + self.k_dist * (dist - self.best_distance)
        v = max(0.0, min(self.max_linear, v))  # 绕行不后退

        self._publish_twist(v, w)

    def _do_lost(self, now: float, has_target: bool) -> None:
        if has_target:
            self.state = State.ORBIT
            self.state_enter_time = now
            self.get_logger().info("重新捕获标靶，回到绕行")
            return
        if now - self.state_enter_time > self.lost_timeout:
            self.state = State.SEARCH
            self.state_enter_time = now
            self.get_logger().info("丢失超时，回到搜索")
            return
        self._publish_zero()  # 丢靶时停车等待

    # ---------------- 速度发布 ----------------
    def _publish_twist(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _publish_zero(self) -> None:
        self._publish_twist(0.0, 0.0)

    def _stop_recording(self) -> None:
        if self._video_writer is not None:
            self._video_writer.release()
            self._video_writer = None
            self._recording = False
            self.get_logger().info("录像已停止并保存")

    def destroy_node(self) -> bool:
        if self.enabled:
            self._publish_zero()  # 启用过才可能发过速度，退出前补零停车
        self._stop_recording()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OrbitNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
