"""机载端情报融合节点：像素级检测结果 -> 格号情报 -> 下发给地面车。

前置：必须先标定单应矩阵（tools/calib_homography.py），否则像素无法落到格号。

订阅（各自可配置，留空则不启用）：
    /perception/mines      (DetectionArray)  rest_clean：地雷/爆炸物
    /perception/detections (DetectionArray)  best_new：敌我目标（blue/red）
    /perception/obstacles  (DetectionArray)  障碍物（可选）
    /perception/bridges    (DetectionArray)  桥洞通行标识（可选）

发布：
    /air/field_intel (FieldIntelMsg) —— 交给 air_link_node 走数传下发
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node

from ag_core.blackbox import EV, BlackBox
from ag_core.grid_map import FieldGrid
from ag_interfaces.msg import DetectionArray, FieldIntelMsg

from .intel_accumulator import KIND, IntelAccumulator

INTEL_TOPIC = "/air/field_intel"


def _flat_to_H(vals) -> np.ndarray:
    if vals is None or len(vals) != 9:
        raise ValueError(f"homography 需要 9 个数，当前 {0 if vals is None else len(vals)}")
    H = np.asarray(vals, dtype=np.float64).reshape(3, 3)
    # 未标定（全零）或退化矩阵都会让映射结果无意义，必须显式报错而不是静默输出错误格号
    if abs(np.linalg.det(H)) < 1e-12:
        raise ValueError("homography 为奇异矩阵（是否仍是全零默认值？）")
    return H


class IntelFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("intel_fusion_node")

        # ---- 场地与标定 ----
        # 14 列（A~N，沿 5 m 边）x 11 行（1~11，沿 4 m 边），与车端 path_executor 常量一致
        self.declare_parameter("cols", 14)
        self.declare_parameter("rows", 11)
        self.declare_parameter("cell_size", 0.35)
        self.declare_parameter("homography", [0.0] * 9)

        # ---- 累积 ----
        self.declare_parameter("min_votes", 3)
        self.declare_parameter("ttl_s", 20.0)
        self.declare_parameter("publish_period_s", 1.0)

        # ---- 过滤 ----
        self.declare_parameter("min_score", 0.50)

        # ---- 话题 ----
        self.declare_parameter("mines_topic", "/perception/mines")
        self.declare_parameter("targets_topic", "/perception/detections")
        self.declare_parameter("obstacles_topic", "")
        self.declare_parameter("bridges_topic", "")
        self.declare_parameter("output_topic", INTEL_TOPIC)
        self.declare_parameter("only_when_ready", True)
        # ---- 黑匣子 ----
        self.declare_parameter("log_enabled", True)
        self.declare_parameter("log_dir", "")
        self.declare_parameter("log_level", "info")

        p = lambda k: self.get_parameter(k).value  # noqa: E731

        self.grid = FieldGrid(
            cols=int(p("cols")), rows=int(p("rows")), cell_size=float(p("cell_size"))
        )
        self._h_ok = True
        try:
            self.grid.set_homography(_flat_to_H(p("homography")))
        except Exception as e:
            self._h_ok = False
            self.get_logger().error(f"单应矩阵无效({e})，节点将无法输出格号")

        self.acc = IntelAccumulator(
            min_votes=int(p("min_votes")),
            ttl_s=float(p("ttl_s")),
        )
        self.min_score = float(p("min_score"))
        self.only_when_ready = bool(p("only_when_ready"))
        self.seq = 0

        self.pub = self.create_publisher(FieldIntelMsg, p("output_topic"), 10)
        self._subscribe(p("mines_topic"), KIND.MINE)
        self._subscribe(p("obstacles_topic"), KIND.OBSTACLE)
        self._subscribe(p("bridges_topic"), KIND.BRIDGE)
        # 敌我目标按 cls 分流：0=blue(敌) 1=red(友)
        self._subscribe(p("targets_topic"), None)

        self.create_timer(float(p("publish_period_s")), self.cb_publish)

        self.bb = BlackBox.from_params(self)
        # 单应是整条情报链的闸门：没标定 -> 像素落不到格号 -> 机载端**永远**发不出情报。
        # 而现场表现只是"车端一帧情报都没有"，极容易被误诊成链路/UDP 问题
        # （报告里的 no-intel 会指向网卡和端口，方向全错），所以必须显式留一条。
        if not self._h_ok:
            self.bb.log(EV.HOMOGRAPHY, level="error", ok=False, stage="startup",
                        reason="单应矩阵无效（全零 = 未标定，或退化）",
                        vals=[float(v) for v in list(p("homography"))[:9]])
        else:
            self.bb.log(EV.HOMOGRAPHY, ok=True, stage="startup")
        self.get_logger().info(
            f"intel_fusion_node 就绪: 单应={'已标定' if self.grid.has_homography else '未标定'} "
            f"min_votes={self.acc.min_votes}"
        )

    # ---------------- 订阅 ----------------
    def _subscribe(self, topic: str, kind: Optional[str]) -> None:
        if not topic:
            return
        if kind is not None:
            self.create_subscription(DetectionArray, topic, lambda m: self.cb_dets(m, kind), 10)
        else:
            self.create_subscription(DetectionArray, topic, self.cb_targets, 10)

    # ---------------- 回调 ----------------
    def cb_dets(self, msg: DetectionArray, kind: str) -> None:
        self.acc.observe_many(kind, self._cells_of(msg))

    def cb_targets(self, msg: DetectionArray) -> None:
        """敌我目标：cls 0=蓝面(敌方) 1=红面(友方)，见 target_policy.CLASS_*。"""
        blue, red = [], []
        for d in msg.detections:
            if d.score < self.min_score:
                continue
            c = self._cell_of(d)
            if c is None:
                continue
            (red if int(d.cls) == 1 else blue).append(c)
        self.acc.observe_many(KIND.TARGET_BLUE, blue)
        self.acc.observe_many(KIND.TARGET_RED, red)

    # ---------------- 像素 -> 格号 ----------------
    def _cell_of(self, d) -> str:
        if not self.grid.has_homography:
            # "看得见目标却算不出格号"是典型现场：检测在跑、情报为零。
            # 不打出来，复盘时只会看到"机载端没发情报"，然后去查链路。
            if self.bb.throttled("no_h", 5.0):
                self.bb.log(EV.HOMOGRAPHY, level="error", ok=False, stage="mapping",
                            reason="有检测结果但单应未标定，无法映射到格号")
            return ""
        # 优先用关键点（发光中心），否则退回框底边中心（贴地点更稳）
        if getattr(d, "kpt_conf", 0.0) > 0.0:
            u, v = float(d.kpt_x), float(d.kpt_y)
        else:
            u, v = (float(d.x1) + float(d.x2)) / 2.0, float(d.y2)
        try:
            cid = self.grid.pixel_to_cell(u, v)
        except Exception:
            return ""
        return cid.name if cid else ""

    def _cells_of(self, msg: DetectionArray):
        out = []
        for d in msg.detections:
            if d.score < self.min_score:
                continue
            c = self._cell_of(d)
            if c:
                out.append(c)
        return out

    # ---------------- 发布 ----------------
    def cb_publish(self) -> None:
        if self.only_when_ready and not self.acc.is_ready:
            return
        intel = self.acc.snapshot()
        self.seq += 1
        m = FieldIntelMsg()
        m.header.stamp = self.get_clock().now().to_msg()
        m.mines = list(intel.mines)
        m.obstacles = list(intel.obstacles)
        m.bridge_a = intel.bridge_a or ""
        m.bridge_b = intel.bridge_b or ""
        m.target_cells = list(intel.target_cells)
        m.target_face = intel.target_face or ""
        m.seq = self.seq
        self.pub.publish(m)
        # 1Hz 发布，逐条记即可：这份 snapshot 就是"机载端认为的场地真相"，
        # 车端若收到的东西跟它不一样，问题一定出在链路而不是感知。
        self.bb.log(EV.INTEL_ACCUM, seq=self.seq, mines=list(intel.mines),
                    obstacles=list(intel.obstacles), bridge_a=intel.bridge_a,
                    bridge_b=intel.bridge_b, target_cells=list(intel.target_cells),
                    target_face=intel.target_face)


def main(args=None):
    rclpy.init(args=args)
    node = IntelFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.bb.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
