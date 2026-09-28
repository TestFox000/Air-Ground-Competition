"""敌我识别决策节点：检测结果 -> 云台偏角 + 开火许可。

数据流向：
    /perception/detections (DetectionArray) ─┐
                                             ├─▶ TargetPolicy ─▶ /gimbal/command (GimbalCommand)
    /referee/state (RefereeStateMsg) ────────┘        │
                                                      └─▶ /mission/strike (String, JSON 记录)

安全约束（规则硬约束，代码里不允许绕过）：
    * 只有 TargetPolicy 返回 FIRE 才置 fire=True
    * 红面（友方）一律 AVOID，永不置 fire
    * 开火有冷却（fire_cooldown_s），避免一帧多发打光弹药
"""

from __future__ import annotations

import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ag_core.blackbox import BlackBox
from ag_core.postprocess import Detection
from ag_core.target_policy import FireAction, TargetPolicy
from ag_interfaces.msg import DetectionArray, GimbalCommand, RefereeStateMsg

GIMBAL_TOPIC = "/gimbal/command"
STRIKE_TOPIC = "/mission/strike"


def msg_to_detections(msg: DetectionArray) -> list:
    """DetectionArray -> List[ag_core.Detection]。kpt_conf<=0 视为无关键点。"""
    out = []
    for m in msg.detections:
        kpts = None
        if m.kpt_conf > 0.0:
            kpts = np.array([[m.kpt_x, m.kpt_y, m.kpt_conf]], dtype=np.float32)
        out.append(
            Detection(
                cls=int(m.cls),
                score=float(m.score),
                x1=float(m.x1),
                y1=float(m.y1),
                x2=float(m.x2),
                y2=float(m.y2),
                kpts=kpts,
            )
        )
    return out


class TargetNode(Node):
    def __init__(self) -> None:
        super().__init__("target_node")

        self.declare_parameter("conf_thres", 0.60)
        self.declare_parameter("kpt_conf_thres", 0.40)
        self.declare_parameter("consensus_frames", 3)
        self.declare_parameter("aim_tolerance_px", 25.0)
        self.declare_parameter("ammo_total", 10)
        # 类别 id 映射：重新训练后类别顺序可能变化，必须与训练时的顺序一致
        self.declare_parameter("class_blue", 0)   # 蓝面 = 敌方 = 可打击
        self.declare_parameter("class_red", 1)    # 红面 = 友方 = 禁止打击
        self.declare_parameter("img_w", 640)
        self.declare_parameter("img_h", 480)
        self.declare_parameter("fov_h_deg", 60.0)
        self.declare_parameter("fire_cooldown_s", 0.8)
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("referee_topic", "/referee/state")
        self.declare_parameter("enable_fire", True)
        # ---- 黑匣子 ----
        self.declare_parameter("log_enabled", True)
        self.declare_parameter("log_dir", "")
        self.declare_parameter("log_level", "info")

        p = lambda k: self.get_parameter(k).value  # noqa: E731

        self.policy = TargetPolicy(
            conf_thres=float(p("conf_thres")),
            kpt_conf_thres=float(p("kpt_conf_thres")),
            consensus_frames=int(p("consensus_frames")),
            aim_tolerance_px=float(p("aim_tolerance_px")),
            ammo_total=int(p("ammo_total")),
            img_w=int(p("img_w")),
            img_h=int(p("img_h")),
            fov_h_deg=float(p("fov_h_deg")),
            class_blue=int(p("class_blue")),
            class_red=int(p("class_red")),
        )
        self.cooldown_s = float(p("fire_cooldown_s"))
        self.enable_fire = bool(p("enable_fire"))
        self._last_fire = 0.0

        self.pub_gimbal = self.create_publisher(GimbalCommand, GIMBAL_TOPIC, 10)
        self.pub_strike = self.create_publisher(String, STRIKE_TOPIC, 10)
        self.create_subscription(DetectionArray, p("detections_topic"), self.cb_det, 10)
        self.create_subscription(RefereeStateMsg, p("referee_topic"), self.cb_referee, 10)

        self.bb = BlackBox.from_params(self)

        self.get_logger().info(
            f"target_node 就绪: conf>={self.policy.conf_thres} 一致帧={self.policy.consensus_frames} "
            f"开火={'允许' if self.enable_fire else '禁用(仅瞄准)'}"
        )

    # ---------------- 回调 ----------------
    def cb_det(self, msg: DetectionArray) -> None:
        dets = msg_to_detections(msg)
        decision = self.policy.update(dets)

        cmd = GimbalCommand()
        cmd.header = msg.header
        cmd.yaw_delta = float(decision.yaw_delta)
        cmd.pitch_delta = float(decision.pitch_delta)
        cmd.reason = decision.reason

        now = time.time()
        fire = False
        if decision.action == FireAction.FIRE and self.enable_fire:
            # 双保险：即使策略出错，也绝不对红面开火
            if decision.cls == self.policy.class_red:
                cmd.reason = "安全拦截：红面目标禁止开火"
            elif (now - self._last_fire) < self.cooldown_s:
                cmd.reason = f"开火冷却中（{self.cooldown_s}s）"
            elif self.policy.ammo_left <= 0:
                cmd.reason = "弹药耗尽"
            else:
                fire = True
                self._last_fire = now
        cmd.fire = fire
        self.pub_gimbal.publish(cmd)

        if fire:
            rec = String()
            rec.data = json.dumps(
                {
                    "cls": decision.cls,
                    "score": round(decision.score, 3),
                    "aim_px": [round(v, 1) for v in (decision.aim_px or (0.0, 0.0))],
                    "yaw": round(decision.yaw_delta, 2),
                    "pitch": round(decision.pitch_delta, 2),
                    "ammo_left": self.policy.ammo_left,
                },
                ensure_ascii=False,
            )
            self.pub_strike.publish(rec)
            self.get_logger().info(f"开火: {rec.data}")

    def cb_referee(self, msg: RefereeStateMsg) -> None:
        """用裁判系统权威状态回填本地计数，避免车端与裁判板口径不一致。"""
        self.policy.shots_fired = int(msg.shots)
        self.policy.enemy_hits = int(msg.enemy_count) * self.policy.hits_per_count
        self.policy.friend_hits = int(msg.friend_count) * self.policy.hits_per_count


def main(args=None):
    rclpy.init(args=args)
    node = TargetNode()
    try:
        rclpy.spin(node)
    finally:
        node.bb.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
