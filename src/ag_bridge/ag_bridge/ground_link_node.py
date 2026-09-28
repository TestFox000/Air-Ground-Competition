"""车端链路节点：UDP 接收 -> 发布 FieldIntelMsg。

发布：
    /link/field_intel (FieldIntelMsg) —— 交给 mission_node 规划
    /link/link_stats (String, JSON)   —— 收发统计，用于现场排查同频干扰
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ag_core.blackbox import EV, BlackBox
from ag_core.link_protocol import MSG, FieldIntel
from ag_interfaces.msg import FieldIntelMsg

from .udp_link import UdpLink

INTEL_TOPIC = "/link/field_intel"
STATS_TOPIC = "/link/link_stats"


class GroundLinkNode(Node):
    def __init__(self) -> None:
        super().__init__("ground_link_node")

        self.declare_parameter("bind_port", 9901)
        self.declare_parameter("peer_ip", "255.255.255.255")
        self.declare_parameter("peer_port", 9902)
        self.declare_parameter("poll_period_s", 0.02)
        self.declare_parameter("stats_period_s", 2.0)
        # ---- 黑匣子 ----
        self.declare_parameter("log_enabled", True)
        self.declare_parameter("log_dir", "")
        self.declare_parameter("log_level", "info")

        p = lambda k: self.get_parameter(k).value  # noqa: E731

        self.link = UdpLink(
            bind_port=int(p("bind_port")),
            peer=(p("peer_ip"), int(p("peer_port"))),
            broadcast=True,
            timeout_s=float(p("poll_period_s")),
        )
        self.pub = self.create_publisher(FieldIntelMsg, INTEL_TOPIC, 10)
        self.pub_stats = self.create_publisher(String, STATS_TOPIC, 10)
        self.create_timer(float(p("poll_period_s")), self.cb_recv)
        self.create_timer(float(p("stats_period_s")), self.cb_stats)

        self.bb = BlackBox.from_params(self)
        self._bad_crc = 0

        self.get_logger().info(f"ground_link_node 就绪: bind {p('bind_port')}")

    def cb_recv(self) -> None:
        st = self.link.stats
        d = self.link.recv()

        # bad_crc 是**累计量**：必须用增量判断，否则要么漏报要么重复报同一帧。
        # 坏帧多 = 无线环境差，是"车没收到情报"最上游的原因。
        if st.bad_crc != self._bad_crc:
            self.bb.log(EV.LINK_DROP, level="warn", why="crc32 校验失败",
                        dropped=st.bad_crc - self._bad_crc, bad_crc_total=st.bad_crc)
            self._bad_crc = st.bad_crc

        if d is None:
            return
        if d.get("t") != MSG.INTEL:
            # 心跳/状态帧：不需要每条都进时间轴，但"链路是活的"这个证据要留
            if self.bb.throttled("non_intel", 5.0):
                self.bb.log(EV.LINK_RX, kind=str(d.get("t")), ok=True,
                            seq=d.get("seq"), note="非情报帧（心跳/状态）")
            return

        intel = FieldIntel.from_dict(d)
        m = FieldIntelMsg()
        m.header.stamp = self.get_clock().now().to_msg()
        m.mines = list(intel.mines)
        m.obstacles = list(intel.obstacles)
        m.bridge_a = intel.bridge_a or ""
        m.bridge_b = intel.bridge_b or ""
        m.target_cells = list(intel.target_cells)
        m.target_face = intel.target_face or ""
        m.seq = int(intel.seq)
        self.pub.publish(m)
        self.bb.log(EV.LINK_RX, kind=MSG.INTEL, ok=True, seq=int(intel.seq),
                    mines=list(intel.mines), obstacles=list(intel.obstacles),
                    bridge_a=intel.bridge_a, bridge_b=intel.bridge_b,
                    target_cells=list(intel.target_cells),
                    target_face=intel.target_face, bad_crc_total=st.bad_crc)

    def cb_stats(self) -> None:
        out = String()
        out.data = json.dumps(self.link.stats.as_dict())
        self.pub_stats.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GroundLinkNode()
    try:
        rclpy.spin(node)
    finally:
        node.link.close()
        node.bb.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
