"""调试用情报源：在无机载端/无真机时，周期发布模拟侦察情报。

两种模式：
    full        每帧都发完整情报（8 雷 / 2 桥洞 / 敌我区）
    progressive 逐帧揭示，并掺入偶发误报格号，用于验证车端多帧投票是否生效

可选：同时发布一个「蓝面目标」的 DetectionArray，让仿真里的
target_node -> referee_node 打击链路也能闭环（sim.launch.py 默认打开）。

发布话题由 intel_topic 指定：
    接 air_link_node 时用 /air/field_intel（走 UDP 全链路）
    直接验证车端时用 /link/field_intel（跳过无线链路）
"""

from __future__ import annotations

import random

import rclpy
from rclpy.node import Node

from ag_interfaces.msg import Detection, DetectionArray, FieldIntelMsg

from ag_core.blackbox import BlackBox

ALL_MINES = ["B2", "C3", "D4", "E5", "F6", "G7", "H8", "I9"]
OBSTACLES = ["A5", "J7"]
NOISE_MINES = ["A9", "K2", "D11"]


class FakeIntelNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_intel_node")

        self.declare_parameter("period_s", 1.0)
        self.declare_parameter("mode", "progressive")
        self.declare_parameter("noise_ratio", 0.2)
        self.declare_parameter("seed", 20260927)
        self.declare_parameter("intel_topic", "/air/field_intel")
        self.declare_parameter("publish_detections", False)
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("target_cls", 0)      # 0=蓝面(敌, 可打) 1=红面(友, 禁打)
        # ---- 黑匣子 ----
        self.declare_parameter("log_enabled", True)
        self.declare_parameter("log_dir", "")
        self.declare_parameter("log_level", "info")

        p = lambda k: self.get_parameter(k).value  # noqa: E731

        self.period = float(p("period_s"))
        self.mode = str(p("mode"))
        self.noise_ratio = float(p("noise_ratio"))
        self.rng = random.Random(int(p("seed")))
        self.target_cls = int(p("target_cls"))
        self.seq = 0
        self.frame = 0

        self.pub = self.create_publisher(FieldIntelMsg, p("intel_topic"), 10)
        if bool(p("publish_detections")):
            self.pub_det = self.create_publisher(DetectionArray, p("detections_topic"), 10)
        else:
            self.pub_det = None
        self.create_timer(self.period, self.cb_timer)

        self.bb = BlackBox.from_params(self)
        self.get_logger().info(
            f"fake_intel_node 就绪: mode={self.mode} -> {p('intel_topic')}"
            + (f" + 检测 -> {p('detections_topic')}" if self.pub_det else "")
        )

    def cb_timer(self) -> None:
        self.frame += 1
        self.seq += 1
        mines = list(ALL_MINES)
        if self.mode == "progressive":
            revealed = min(len(ALL_MINES), max(2, self.frame))
            mines = ALL_MINES[:revealed]
            if self.rng.random() < self.noise_ratio:
                mines = mines + [self.rng.choice(NOISE_MINES)]

        m = FieldIntelMsg()
        m.header.stamp = self.get_clock().now().to_msg()
        m.mines = mines
        m.obstacles = OBSTACLES
        m.bridge_a = "K4"
        m.bridge_b = "K9"
        m.target_cells = ["D2", "E2"]
        m.target_face = "blue" if self.target_cls == 0 else "red"
        m.seq = self.seq
        self.pub.publish(m)

        if self.pub_det is not None:
            self.pub_det.publish(self._fake_detections(m.header))

    def _fake_detections(self, header) -> DetectionArray:
        """画面正中一个高置信目标：关键点即中心，用于跑通瞄准/开火链路。"""
        d = Detection()
        d.cls = self.target_cls
        d.score = 0.95
        d.x1, d.y1, d.x2, d.y2 = 300.0, 220.0, 340.0, 260.0
        d.kpt_x, d.kpt_y, d.kpt_conf = 320.0, 240.0, 0.90
        out = DetectionArray()
        out.header = header
        out.detections = [d]
        return out


def main(args=None):
    rclpy.init(args=args)
    node = FakeIntelNode()
    try:
        rclpy.spin(node)
    finally:
        node.bb.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
