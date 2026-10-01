"""车端集成节点：接收 dzuav 二进制 UDP:50000 -> 官方 path_executor。

    UDP 广播 (dzuav UdpBroadcaster)
        -> 后台收包线程（只做 收包/解析/态势归并）
        -> 最新帧队列（容量 1，决策慢时自动丢旧帧——态势数据只要最新）
        -> ROS timer 回调（executor 线程）：门控 -> 路由+适配 -> 发布
        -> /nav/waypoint  先发一次 reset，再发路线（官方 path_executor 消费）

线程模型：规划与发布都在 ROS executor 线程的 timer 回调里做，
后台线程只把最新 WorldSnapshot 递进来，不跨线程调用 ROS API。

安全默认：dry_run=True（只计算、打日志，不发命令）；上车验证后再置 false。
"""

from __future__ import annotations

import queue
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
        self.declare_parameter("cross_gates", [""])
        self.declare_parameter("mid_boundary", 5)

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
        self.cross_gates = tuple(n.strip() for n in p("cross_gates") if n.strip())
        self.mid_boundary = int(p("mid_boundary"))

        self.model = WorldModel()
        self.reset_sent = False
        self.last_route = ""

        # 收包线程 -> 决策回调的最新帧交接：容量 1，满了丢旧帧。
        # 态势数据只要最新值，决策慢于到达率时自动降级为"跳帧"，不积压。
        self._frames: queue.Queue = queue.Queue(maxsize=1)

        self.pub = self.create_publisher(String, str(p("waypoint_topic")), 10)
        # 决策/发布都在 executor 线程（timer 回调）里做，收包线程不碰 ROS。
        self.create_timer(0.1, self._on_tick)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._receive, args=(int(p("udp_port")),), daemon=True)
        self._thread.start()

        mode = "DRY_RUN（只算不发）" if self.dry_run else "实发"
        self.get_logger().info(
            f"integrate_node 就绪: 监听 UDP:{p('udp_port')} -> "
            f"{self.start_cell}->{self.goal_cell}, {mode}"
        )

    # ---------------- UDP 接收（后台线程，只解析不入 ROS） ----------------
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
            self._submit(self.model.update(frame))
        sock.close()

    def _submit(self, snapshot) -> None:
        """把最新态势放入交接队列；满了先丢旧帧再放（单生产者，无竞态）。"""
        try:
            self._frames.put_nowait(snapshot)
        except queue.Full:
            try:
                self._frames.get_nowait()
                self._frames.put_nowait(snapshot)
            except (queue.Empty, queue.Full):
                pass

    # ---------------- 决策（executor 线程） ----------------
    def _on_tick(self) -> None:
        try:
            snapshot = self._frames.get_nowait()
        except queue.Empty:
            return
        self._handle(snapshot)

    def _handle(self, snapshot) -> None:
        gate = evaluate_gate(snapshot, self.min_mines, self.require_face)
        if not gate.ok:
            self._log_hold_once(gate.reason)
            return

        commands = build_mission(
            snapshot, self.start_cell, self.goal_cell,
            self.start_heading, self.final_heading,
            self.cross_gates, self.mid_boundary,
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
