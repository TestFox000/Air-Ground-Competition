"""车端集成节点：接收 dzuav 二进制 UDP:50000 -> 官方 path_executor。

    UDP 广播 (dzuav UdpBroadcaster)
        -> 后台收包线程
             parse -> WorldModel -> 门控 -> 路由+适配
        -> /nav/waypoint  先发一次 reset，再发路线（官方 path_executor 消费）

安全默认：dry_run=True（只计算、打日志，不发命令）；上车验证后再置 false。
"""

from __future__ import annotations

import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .intel_protocol import parse
from .pipeline import build_mission, evaluate_gate
from .world_model import WorldModel


class IntegrateNode(Node):
    def __init__(self) -> None:
        super().__init__("integrate_node")

        self.declare_parameter("udp_port", 50000)
        self.declare_parameter("waypoint_topic", "/nav/waypoint")
        self.declare_parameter("start_cell", "N10")
        self.declare_parameter("goal_cell", "N11")
        self.declare_parameter("start_heading", "y")
        self.declare_parameter("final_heading", "y")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("publish_reset", True)
        self.declare_parameter("min_mines", 1)
        self.declare_parameter("require_face", True)

        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.start_cell = str(p("start_cell"))
        self.goal_cell = str(p("goal_cell"))
        self.start_heading = str(p("start_heading"))
        fh = str(p("final_heading")).strip().lower()
        self.final_heading = fh if fh in ("x", "y") else None
        self.dry_run = bool(p("dry_run"))
        self.publish_reset = bool(p("publish_reset"))
        self.min_mines = int(p("min_mines"))
        self.require_face = bool(p("require_face"))

        self.model = WorldModel()
        self.reset_sent = False
        self.last_route = ""

        self.pub = self.create_publisher(String, str(p("waypoint_topic")), 10)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._receive, args=(int(p("udp_port")),), daemon=True)
        self._thread.start()

        mode = "DRY_RUN（只算不发）" if self.dry_run else "实发"
        self.get_logger().info(
            f"integrate_node 就绪: 监听 UDP:{p('udp_port')} -> "
            f"{self.start_cell}->{self.goal_cell}, {mode}"
        )

    # ---------------- UDP 接收 ----------------
    def _receive(self, port: int) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", port))
        sock.settimeout(0.5)
        while not self._stop.is_set():
            try:
                data, _addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                frame = parse(data)
            except ValueError as exc:
                self.get_logger().warning(f"丢弃坏帧: {exc}")
                continue
            snapshot = self.model.update(frame)
            self._handle(snapshot)
        sock.close()

    def _handle(self, snapshot) -> None:
        gate = evaluate_gate(snapshot, self.min_mines, self.require_face)
        if not gate.ok:
            self._log_hold_once(gate.reason)
            return

        commands = build_mission(
            snapshot, self.start_cell, self.goal_cell,
            self.start_heading, self.final_heading,
        )
        if commands.route == self.last_route:
            return
        self.last_route = commands.route

        if self.dry_run:
            self.get_logger().info(
                f"[dry_run] {commands.reset} | {commands.route}"
            )
            return

        if self.publish_reset and not self.reset_sent:
            self._publish(commands.reset)
            self.reset_sent = True
        self._publish(commands.route)
        self.get_logger().info(f"已下发路线: {commands.route}")

    def _log_hold_once(self, reason: str) -> None:
        if reason != getattr(self, "_last_hold", None):
            self.get_logger().info(f"暂不发车：{reason}")
            self._last_hold = reason

    def _publish(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub.publish(msg)

    def destroy_node(self) -> bool:
        self._stop.set()
        self._thread.join(timeout=2)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IntegrateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
