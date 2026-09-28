"""机载端链路节点：订阅本机汇总出的侦察情报 -> UDP 下发给地面车。

订阅：
    /air/field_intel (FieldIntelMsg)  —— 由机载端的情报汇总/融合节点产出
    /perception/detections (DetectionArray) —— 可选，用于打印检测统计

无新情报时按 heartbeat_period_s 发心跳，方便车端判断链路是否存活。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from ag_core.blackbox import EV, BlackBox
from ag_core.link_protocol import FieldIntel
from ag_interfaces.msg import FieldIntelMsg

from .udp_link import UdpLink


class AirLinkNode(Node):
    def __init__(self) -> None:
        super().__init__("air_link_node")

        self.declare_parameter("bind_port", 9902)
        self.declare_parameter("peer_ip", "255.255.255.255")
        self.declare_parameter("peer_port", 9901)
        self.declare_parameter("heartbeat_period_s", 1.0)
        self.declare_parameter("intel_topic", "/air/field_intel")
        # ---- 黑匣子 ----
        self.declare_parameter("log_enabled", True)
        self.declare_parameter("log_dir", "")
        self.declare_parameter("log_level", "info")

        p = lambda k: self.get_parameter(k).value  # noqa: E731

        self.link = UdpLink(
            bind_port=int(p("bind_port")),
            peer=(p("peer_ip"), int(p("peer_port"))),
            broadcast=True,
        )
        self.seq = 0
        self.create_subscription(FieldIntelMsg, p("intel_topic"), self.cb_intel, 10)
        self.create_timer(float(p("heartbeat_period_s")), self.cb_heartbeat)

        self.bb = BlackBox.from_params(self)

        self.get_logger().info(
            f"air_link_node 就绪: -> {p('peer_ip')}:{p('peer_port')} (bind {p('bind_port')})"
        )

    def cb_intel(self, msg: FieldIntelMsg) -> None:
        intel = FieldIntel(
            mines=list(msg.mines),
            obstacles=list(msg.obstacles),
            bridge_a=msg.bridge_a or None,
            bridge_b=msg.bridge_b or None,
            target_cells=list(msg.target_cells),
            target_face=msg.target_face or None,
            seq=int(msg.seq) or self.seq,
        )
        self.seq = max(self.seq, intel.seq)
        try:
            n = self.link.send_intel(intel)
        except OSError as e:
            # 发送失败 = 车端必然收不到，这是"车上没情报"的第一现场
            self.bb.log(EV.LINK_DROP, level="error", why="机载端发送失败",
                        error=f"{type(e).__name__}: {e}")
            self.get_logger().warning(f"发送失败: {e}")
            return
        self.bb.log(EV.LINK_TX, kind="intel", seq=int(intel.seq), bytes=n,
                    mines=list(intel.mines), obstacles=list(intel.obstacles),
                    bridge_a=intel.bridge_a, bridge_b=intel.bridge_b,
                    target_cells=list(intel.target_cells),
                    target_face=intel.target_face)

    def cb_heartbeat(self) -> None:
        try:
            n = self.link.send_heartbeat(self.seq, role="air")
        except OSError as e:
            self.bb.log(EV.LINK_DROP, level="warn", why="机载端心跳发送失败",
                        error=f"{type(e).__name__}: {e}")
            self.get_logger().warning(f"心跳发送失败: {e}")
            return
        # 心跳 1Hz，逐条记意义不大，但"链路还在发"这件事要留证据
        if self.bb.throttled("hb", 10.0):
            self.bb.log(EV.LINK_TX, kind="heartbeat", seq=self.seq, bytes=n)


def main(args=None):
    rclpy.init(args=args)
    node = AirLinkNode()
    try:
        rclpy.spin(node)
    finally:
        node.link.close()
        node.bb.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
