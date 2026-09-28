"""裁判系统节点：串口 / 官方 ROS 话题 <-> ROS。

三种数据源（source 参数）：
    dz      **实机推荐**：接渡众自带的 `dzjudgment` 节点（它自己解裁判串口），
            订阅 `/HpAndHitmsg` 等话题，开火则发 `/LaserShot_Command`
    serial  自己解串口帧（官方包不可用时的兜底，帧格式见 referee_protocol）
    topic   订阅上游转出的 JSON 话题（自制转发节点用）

发布：
    /referee/state (RefereeStateMsg) —— 弹药 / 已射击 / 受击方向 / 血量
订阅：
    /gimbal/command (GimbalCommand)  —— fire=True 时下发一次射击指令
    /HpAndHitmsg (UInt8MultiArray)   —— source=dz：官方裁判状态
    /self_Material_Number (UInt8MultiArray) —— source=dz：己方物资
    /referee/state_json (String)     —— source=topic：JSON 状态

dz 模式下本节点**不碰串口**，只做话题转接；真实串口由 dzjudgment 负责。

字段映射（源：dzjudgment/src/judgment.cpp，cmd=0x07）：
    /HpAndHitmsg.data = [targethp, self_hp, Bing_hit, Shooting_Count, ammo]
    Bing_hit: 0x01 左 / 0x02 前 / 0x03 右 / 0x04 后 / 0x10 未被击打
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8, UInt8MultiArray

from ag_core.blackbox import EV, BlackBox
from ag_core.referee_protocol import RefereeState
from ag_interfaces.msg import GimbalCommand, RefereeStateMsg

from .serial_io import RefereeSerial, open_serial

#: JSON 状态字段约定；官方字段名不一致时只改这里
STATE_KEYS = ("ammo", "shots", "enemy_count", "friend_count", "illuminated", "hp")

#: dzjudgment 的 0x07 帧字段顺序
DZ_HP_FIELDS = ("targethp", "self_hp", "hit", "shots", "ammo")

STATE_TOPIC = "/referee/state"


class RefereeNode(Node):
    def __init__(self) -> None:
        super().__init__("referee_node")

        self.declare_parameter("source", "serial")    # dz | serial | topic
        self.declare_parameter("state_json_topic", "/referee/state_json")
        self.declare_parameter("shot_json_topic", "/referee/shot_cmd")
        # ---- source=dz：接渡众自带 dzjudgment 的话题 ----
        self.declare_parameter("hp_topic", "/HpAndHitmsg")
        self.declare_parameter("self_material_topic", "/self_Material_Number")
        self.declare_parameter("shot_topic", "/LaserShot_Command")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("poll_period_s", 0.05)
        self.declare_parameter("min_shot_interval_s", 0.5)
        self.declare_parameter("shot_duration_ms", 200)
        self.declare_parameter("shots_per_command", 1)
        self.declare_parameter("gimbal_topic", "/gimbal/command")
        self.declare_parameter("simulate", False)
        self.declare_parameter("simulate_hit", True)   # simulate 模式下：假定开火即命中敌方
        self.declare_parameter("hits_per_count", 2)    # 规则：每 2 次有效命中，受击侧计数 +1
        self.declare_parameter("max_count", 3)         # 任一侧计到 3 后靶标整体失效
        self.declare_parameter("ammo_total", 10)
        # ---- 黑匣子 ----
        self.declare_parameter("log_enabled", True)
        self.declare_parameter("log_dir", "")
        self.declare_parameter("log_level", "info")

        p = lambda k: self.get_parameter(k).value  # noqa: E731

        self.min_interval = float(p("min_shot_interval_s"))
        self.shot_duration_ms = int(p("shot_duration_ms"))
        self.shots_per_command = int(p("shots_per_command"))
        self.simulate_hit = bool(p("simulate_hit"))
        self.hits_per_count = max(1, int(p("hits_per_count")))
        self.max_count = max(1, int(p("max_count")))
        self.simulate = bool(p("simulate"))
        self.source = str(p("source"))
        self._last_shot = 0.0

        self.state = RefereeState(ammo=int(p("ammo_total")))
        self.link: RefereeSerial
        self.pub_shot = None
        self.pub_shot_u8 = None
        self.target_hp = 0
        self.materials: list = []
        if self.source == "dz":
            # 实机路径：串口解析交给官方的 dzjudgment，本节点只转发话题
            self.link = RefereeSerial(port=None)
            self.create_subscription(UInt8MultiArray, p("hp_topic"), self.cb_dz_hp, 10)
            self.create_subscription(
                UInt8MultiArray, p("self_material_topic"), self.cb_dz_self_material, 10
            )
            self.pub_shot_u8 = self.create_publisher(UInt8, p("shot_topic"), 10)
            self.get_logger().info(
                f"source=dz: 状态来自 {p('hp_topic')}，开火发往 {p('shot_topic')}（串口由 dzjudgment 负责）"
            )
        elif self.source == "topic":
            # 官方包（dzactuator）已经把串口解析做完了，这里只接它转出的 JSON
            self.link = RefereeSerial(port=None)
            self.create_subscription(String, p("state_json_topic"), self.cb_state_json, 10)
            self.pub_shot = self.create_publisher(String, p("shot_json_topic"), 10)
            self.get_logger().info(
                f"source=topic: 状态来自 {p('state_json_topic')}，射击指令发往 {p('shot_json_topic')}"
            )
        elif self.simulate:
            self.link = RefereeSerial(port=None)
            self.get_logger().warning("simulate=True：不打开真实串口，状态由本节点模拟维持")
        else:
            try:
                self.link = RefereeSerial(open_serial(p("port"), int(p("baud"))))
            except Exception as e:
                self.get_logger().error(f"串口打开失败({e})，退化为 simulate 模式")
                self.link = RefereeSerial(port=None)
                self.simulate = True

        self.pub = self.create_publisher(RefereeStateMsg, STATE_TOPIC, 10)
        self.create_subscription(GimbalCommand, p("gimbal_topic"), self.cb_gimbal, 10)
        self.create_timer(float(p("poll_period_s")), self.cb_poll)

        self.bb = BlackBox.from_params(self)
        self._last_state_log = None

        self.get_logger().info(
            f"referee_node 就绪: source={self.source} port={p('port')} simulate={self.simulate}"
        )

    # ---------------- topic 数据源 ----------------
    def cb_state_json(self, msg: String) -> None:
        """接收官方包转出的 JSON 状态（字段名见 STATE_KEYS）。"""
        try:
            d = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            self.get_logger().warning("裁判状态不是合法 JSON，已忽略")
            return
        for k in STATE_KEYS:
            if k in d:
                setattr(self.state, k, int(d[k]))

    # ---------------- dz 数据源（实机） ----------------
    def cb_dz_hp(self, msg: UInt8MultiArray) -> None:
        """官方裁判状态：data = [targethp, self_hp, hit, shots, ammo]。

        规则里"每 2 次有效命中 -> 受击侧计数 +1"是从靶标血量掉的格数推出来的，
        因此这里保存 targethp，计数交给上层（target/strategy）判断，
        本节点只做「事实转述」。
        """
        d = list(msg.data)
        if len(d) < len(DZ_HP_FIELDS):
            self.get_logger().warning(f"{DZ_HP_FIELDS} 需要 5 个字段，收到 {len(d)} 个，已忽略")
            return
        self.target_hp, self.state.hp, self.state.illuminated, self.state.shots, self.state.ammo = \
            d[0], d[1], d[2], d[3], d[4]

    def cb_dz_self_material(self, msg: UInt8MultiArray) -> None:
        self.materials = list(msg.data)

    # ---------------- 轮询 ----------------
    def cb_poll(self) -> None:
        st = self.link.poll()
        if st is not None:
            self.state = st
        self._publish()

    def _publish(self) -> None:
        m = RefereeStateMsg()
        m.header.stamp = self.get_clock().now().to_msg()
        m.ammo = int(self.state.ammo)
        m.shots = int(self.state.shots)
        m.enemy_count = int(self.state.enemy_count)
        m.friend_count = int(self.state.friend_count)
        m.illuminated = int(self.state.illuminated)
        m.hp = int(self.state.hp)
        self.pub.publish(m)

        # 裁判状态是按 poll_period 轮询出来的（默认 20Hz），逐拍记会淹掉时间轴；
        # 只记**变化**：血量/弹药/受击计数一变就是这局的关键转折点。
        try:
            target_hp = int(getattr(self, "target_hp", 0))
        except (TypeError, ValueError):
            target_hp = 0
        snap = (m.ammo, m.shots, m.enemy_count, m.friend_count, m.illuminated, m.hp, target_hp)
        if snap != self._last_state_log:
            self._last_state_log = snap
            self.bb.log(EV.REFEREE_STATE, ammo=m.ammo, shots=m.shots,
                        enemy_count=m.enemy_count, friend_count=m.friend_count,
                        illuminated=m.illuminated, hp=m.hp, target_hp=target_hp)

    # ---------------- 开火 ----------------
    def cb_gimbal(self, msg: GimbalCommand) -> None:
        if not msg.fire:
            return
        self.bb.log(EV.FIRE_REQUEST, reason=str(getattr(msg, "reason", "")),
                    ammo=int(self.state.ammo))
        now = time.time()
        if (now - self._last_shot) < self.min_interval:
            self.get_logger().warning("射击过于密集，已丢弃一次指令")
            self.bb.log(EV.FIRE_DENIED, level="warn", reason="射击过于密集（min_shot_interval_s）")
            return
        if self.state.ammo <= 0:
            self.get_logger().warning("弹药为 0，拒绝下发射击指令")
            self.bb.log(EV.FIRE_DENIED, level="warn", reason="弹药为 0")
            return
        self._last_shot = now
        if self.pub_shot_u8 is not None:
            # 与官方 dzrknn_dataprocess 一致：先发 1 触发，再发 0 收回，避免连发
            on = UInt8()
            on.data = 1
            self.pub_shot_u8.publish(on)
            off = UInt8()
            off.data = 0
            self.pub_shot_u8.publish(off)
            ok = True
        elif self.pub_shot is not None:
            cmd = String()
            cmd.data = json.dumps(
                {"cmd": "shot", "count": self.shots_per_command,
                 "duration_ms": self.shot_duration_ms},
                ensure_ascii=False,
            )
            self.pub_shot.publish(cmd)
            ok = True
        else:
            ok = self.link.send_shot(self.shots_per_command, self.shot_duration_ms)
        if self.simulate:
            # 模拟模式下自行维护计数，方便在没有硬件时跑通整条链路。
            # 真实裁判板会自己回报 enemy_count，这里只能按规则折算。
            self.state.shots += self.shots_per_command
            self.state.ammo = max(0, self.state.ammo - self.shots_per_command)
            if self.simulate_hit and self.state.enemy_count < self.max_count:
                self.state.enemy_count = min(
                    self.max_count, self.state.shots // self.hits_per_count
                )
            ok = True
        self.bb.log(EV.FIRE_SENT, ok=bool(ok), reason=str(msg.reason),
                    shots_total=int(self.state.shots), ammo=int(self.state.ammo))
        self.get_logger().info(f"下发射击指令 ok={ok} reason='{msg.reason}'")


def main(args=None):
    rclpy.init(args=args)
    node = RefereeNode()
    try:
        rclpy.spin(node)
    finally:
        node.bb.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
