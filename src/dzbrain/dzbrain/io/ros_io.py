# -*- coding: utf-8 -*-
r"""ROS 接线（rclpy）—— 唯一允许 import rclpy 的地方之一。

职责边界：
- 订阅回调**只做三件事**：解码（走 `contract.py` 的纯函数）、打时间戳、写进缓存。
  绝不在回调里做决策/计算 —— 决策在任务线程里（避免回调线程与任务线程抢状态）。
- 发布走命名方法，话题名/QoS/单位一律取自 `contract.py`（禁止手写）。
- `rclpy` **惰性导入**：本文件在无 ROS 的机器上也能被 import（只是不能实例化），
  这样 `--help`、单测、静态检查都不会因为缺 rclpy 而失败。

线程说明：rclpy 的 publish/订阅缓存都是线程安全的（内部有锁），
多个任务线程可以并发调用下面的 publish_*，不需要额外加锁。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable

from . import contract as ct
from ..core import geometry as geo


def _qos(spec: ct.QosSpec):
    """QosSpec → rclpy QoSProfile（惰性导入）。"""
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    return QoSProfile(
        depth=spec.depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=(ReliabilityPolicy.BEST_EFFORT if spec.reliability == "best_effort"
                     else ReliabilityPolicy.RELIABLE),
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if spec.durability == "transient_local"
                    else DurabilityPolicy.VOLATILE),
    )


@dataclass
class RosStats:
    """接线层统计（遥测/排障用）。"""

    odom_msgs: int = 0
    imu_msgs: int = 0
    vision_msgs: int = 0
    aim_status_msgs: int = 0
    hit_msgs: int = 0
    ammo_msgs: int = 0
    commands_sent: int = 0
    zero_commands: int = 0
    drone_in_msgs: int = 0
    last_odom_ts: float = 0.0
    last_vision_ts: float = 0.0
    started_at: float = field(default_factory=time.time)


class RosBridge:
    """把 dzbrain 与车上既有话题接起来。"""

    def __init__(self, node_name: str = "dzbrain", *, log: Callable[[str], None] = print) -> None:
        import rclpy  # 惰性导入：无 ROS 环境 import 本模块不会失败
        from rclpy.node import Node

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node: "Node" = Node(node_name)
        self._log = log
        self.stats = RosStats()

        # ---------------- 缓存（回调写、任务读；值都是不可变对象） ----------------
        self._lock = threading.Lock()
        self._pose: geo.Pose | None = None
        self._pose_ts = 0.0
        self._imu: tuple[float, float, float, float, float, float] | None = None
        self._imu_ts = 0.0
        self._vision: ct.OffsetObservation | None = None
        self._aim_status = ct.AimStatus.IDLE
        self._shoot_count = 0
        self._ammo: int | None = None
        self._hit_pending = 0
        self._manual_override = False
        self._manual_gimbal = False
        self._drone_inbox: list[str] = []
        self._waypoint_cmds: list[str] = []

        self._create_subs()
        self._create_pubs()

    # ------------------------------------------------------------------ #
    #  订阅
    # ------------------------------------------------------------------ #
    def _create_subs(self) -> None:
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from std_msgs.msg import Int32, Int32MultiArray, String, UInt8

        node = self.node

        def on_odom(msg: Odometry) -> None:
            # ★ position 是格单位；四元数要 +π/2（换算集中在 contract.odom_to_pose）
            pose = ct.odom_to_pose(msg.pose.pose.position.x,
                                   msg.pose.pose.position.y,
                                   (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
            with self._lock:
                self._pose = pose
                self._pose_ts = time.time()
                self.stats.odom_msgs += 1
                self.stats.last_odom_ts = self._pose_ts

        def on_imu(msg: Imu) -> None:
            with self._lock:
                self._imu = (msg.angular_velocity.x, msg.angular_velocity.y,
                             msg.angular_velocity.z, msg.linear_acceleration.x,
                             msg.linear_acceleration.y, msg.linear_acceleration.z)
                self._imu_ts = time.time()
                self.stats.imu_msgs += 1

        def on_vision(msg: Int32MultiArray) -> None:
            obs = ct.decode_offset_center(msg.data, ts=time.time())
            if obs is None:      # ★ 长度不足 → 视为无观测，绝不猜字段
                return
            with self._lock:
                self._vision = obs
                self.stats.vision_msgs += 1
                self.stats.last_vision_ts = obs.ts

        def on_aim_status(msg: Int32) -> None:
            with self._lock:
                self._aim_status = int(msg.data)
                self.stats.aim_status_msgs += 1

        def on_shoot_count(msg: Int32) -> None:
            with self._lock:
                self._shoot_count = int(msg.data)

        def on_hit(msg: Int32) -> None:
            with self._lock:
                self._hit_pending += 1
                self.stats.hit_msgs += 1

        def on_ammo(msg: Int32) -> None:
            with self._lock:
                self._ammo = int(msg.data)
                self.stats.ammo_msgs += 1

        def on_manual_override(msg: UInt8) -> None:
            with self._lock:
                self._manual_override = bool(msg.data)

        def on_manual_gimbal(msg: UInt8) -> None:
            with self._lock:
                self._manual_gimbal = bool(msg.data)

        def on_drone(msg: String) -> None:
            with self._lock:
                self._drone_inbox.append(str(msg.data))
                if len(self._drone_inbox) > 64:
                    del self._drone_inbox[:-64]
                self.stats.drone_in_msgs += 1

        def on_waypoint(msg: String) -> None:
            with self._lock:
                self._waypoint_cmds.append(str(msg.data))
                if len(self._waypoint_cmds) > 32:
                    del self._waypoint_cmds[:-32]

        self._subs = [
            node.create_subscription(Odometry, ct.topic("odom").name, on_odom,
                                     _qos(ct.topic("odom").qos)),
            node.create_subscription(Imu, ct.topic("imu").name, on_imu,
                                     _qos(ct.topic("imu").qos)),
            node.create_subscription(Int32MultiArray, ct.topic("offset_center").name,
                                     on_vision, _qos(ct.topic("offset_center").qos)),
            node.create_subscription(Int32, ct.topic("aim_status").name, on_aim_status,
                                     _qos(ct.topic("aim_status").qos)),
            node.create_subscription(Int32, ct.topic("shoot_count").name, on_shoot_count,
                                     _qos(ct.topic("shoot_count").qos)),
            node.create_subscription(Int32, ct.topic("referee_hit").name, on_hit,
                                     _qos(ct.topic("referee_hit").qos)),
            node.create_subscription(Int32, ct.topic("referee_ammo").name, on_ammo,
                                     _qos(ct.topic("referee_ammo").qos)),
            node.create_subscription(UInt8, ct.topic("manual_override").name, on_manual_override,
                                     _qos(ct.topic("manual_override").qos)),
            node.create_subscription(UInt8, ct.topic("manual_gimbal").name, on_manual_gimbal,
                                     _qos(ct.topic("manual_gimbal").qos)),
            node.create_subscription(String, ct.topic("drone_in").name, on_drone,
                                     _qos(ct.topic("drone_in").qos)),
            node.create_subscription(String, ct.topic("waypoint_cmd").name, on_waypoint,
                                     _qos(ct.topic("waypoint_cmd").qos)),
        ]

    def _create_pubs(self) -> None:
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Float32, String, UInt8

        node = self.node
        self._pub_pursuit = node.create_publisher(Twist, ct.topic("pursuit_angle").name,
                                                  _qos(ct.topic("pursuit_angle").qos))
        self._pub_scan = node.create_publisher(UInt8, ct.topic("scan_request").name,
                                               _qos(ct.topic("scan_request").qos))
        self._pub_fire_gate = node.create_publisher(UInt8, ct.topic("aim_fire_gate").name,
                                                    _qos(ct.topic("aim_fire_gate").qos))
        self._pub_fire_ready = node.create_publisher(UInt8, ct.topic("aim_fire_ready").name,
                                                     _qos(ct.topic("aim_fire_ready").qos))
        self._pub_stop_signal = node.create_publisher(UInt8, ct.topic("stop_signal").name,
                                                      _qos(ct.topic("stop_signal").qos))
        self._pub_motion = node.create_publisher(UInt8, ct.topic("motion_state").name,
                                                 _qos(ct.topic("motion_state").qos))
        self._pub_voice = node.create_publisher(UInt8, ct.topic("voice_switch").name,
                                                _qos(ct.topic("voice_switch").qos))
        self._pub_drone = node.create_publisher(String, ct.topic("drone_out").name,
                                                _qos(ct.topic("drone_out").qos))
        self._pub_lead = node.create_publisher(Float32, ct.topic("aim_lead").name,
                                               _qos(ct.topic("aim_lead").qos))

    # ------------------------------------------------------------------ #
    #  读（任务线程调用，全部线程安全）
    # ------------------------------------------------------------------ #
    def pose(self) -> geo.Pose | None:
        with self._lock:
            return self._pose

    def pose_age(self) -> float:
        with self._lock:
            return float("inf") if not self._pose_ts else time.time() - self._pose_ts

    def imu(self) -> tuple[float, float, float, float, float, float] | None:
        with self._lock:
            return self._imu

    def imu_age(self) -> float:
        with self._lock:
            return float("inf") if not self._imu_ts else time.time() - self._imu_ts

    def vision(self, max_age_s: float = 0.5) -> ct.OffsetObservation | None:
        """视觉观测。默认只要 0.5 s 内的（过期观测拿去开火就是打空气）。"""
        with self._lock:
            obs = self._vision
        if obs is None:
            return None
        return obs if (time.time() - obs.ts) <= max_age_s else None

    def vision_raw(self) -> ct.OffsetObservation | None:
        with self._lock:
            return self._vision

    def aim_status(self) -> int:
        with self._lock:
            return self._aim_status

    def aim_status_name(self) -> str:
        return ct.AimStatus.name(self.aim_status())

    def shoot_count(self) -> int:
        with self._lock:
            return self._shoot_count

    def ammo(self) -> int | None:
        with self._lock:
            return self._ammo

    def take_hits(self) -> int:
        """取走累计命中确认数（清零），由 AimTask 记账。"""
        with self._lock:
            n = self._hit_pending
            self._hit_pending = 0
            return n

    def manual_override(self) -> bool:
        with self._lock:
            return self._manual_override

    def manual_gimbal(self) -> bool:
        with self._lock:
            return self._manual_gimbal

    def take_drone_messages(self, max_items: int = 16) -> list[str]:
        with self._lock:
            out = self._drone_inbox[:max_items]
            del self._drone_inbox[:max_items]
            return out

    def take_waypoint_commands(self, max_items: int = 8) -> list[str]:
        with self._lock:
            out = self._waypoint_cmds[:max_items]
            del self._waypoint_cmds[:max_items]
            return out

    # ------------------------------------------------------------------ #
    #  写
    # ------------------------------------------------------------------ #
    def publish_command(self, linear_x: float, steer_deg: float) -> None:
        """★ `pursuitAngle`：linear.x = m/s，angular.z = 舵角【度】。"""
        from geometry_msgs.msg import Twist
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(steer_deg)
        self._pub_pursuit.publish(msg)
        with self._lock:
            self.stats.commands_sent += 1

    def publish_brake(self) -> None:
        """零 Twist = 刹车（所有安全路径都走这里）。"""
        self.publish_command(0.0, 0.0)
        with self._lock:
            self.stats.zero_commands += 1

    def publish_scan_request(self, mode: int) -> None:
        """0=停扫 / 1=单趟 / 2=持续扫描。"""
        from std_msgs.msg import UInt8
        msg = UInt8()
        msg.data = int(mode)
        self._pub_scan.publish(msg)

    def publish_fire_gate(self, allow: bool) -> None:
        """★ 新话题：云台补丁会**真正订阅**它（修旧版"放行门是死线"）。"""
        from std_msgs.msg import UInt8
        msg = UInt8()
        msg.data = 1 if allow else 0
        self._pub_fire_gate.publish(msg)
        # 历史话题同步发一份，便于与旧云台节点对拍/回退
        self._pub_stop_signal.publish(msg)

    def publish_fire_ready(self, ready: bool) -> None:
        """★ 两键开火的第二把钥匙：**瞄准合格**（预测落点在开火框内）。

        分工（避免两个任务抢同一个话题）：
          · `fire_gate`  ← MissionTask（任务级：到站、停稳、进入 FIRE 阶段）
          · `fire_ready` ← AimTask（瞄准级：目标确认、预测落点合格、冷却已过）
          云台补丁只在**两者都为 1** 时点亮激光 —— 任一任务误判都不会误开火。
        """
        from std_msgs.msg import UInt8
        msg = UInt8()
        msg.data = 1 if ready else 0
        self._pub_fire_ready.publish(msg)

    def publish_motion_state(self, reverse: bool) -> None:
        from std_msgs.msg import UInt8
        msg = UInt8()
        msg.data = 1 if reverse else 0
        self._pub_motion.publish(msg)

    def publish_voice(self, voice_id: int) -> None:
        from std_msgs.msg import UInt8
        msg = UInt8()
        msg.data = int(voice_id)
        self._pub_voice.publish(msg)

    def publish_drone(self, payload: str) -> None:
        from std_msgs.msg import String
        msg = String()
        msg.data = payload
        self._pub_drone.publish(msg)

    def publish_lead(self, lead_px: float) -> None:
        """提前量（px）→ 云台补丁用（旧版无此能力，0.5 帧预判远不够）。"""
        from std_msgs.msg import Float32
        msg = Float32()
        msg.data = float(lead_px)
        self._pub_lead.publish(msg)

    # ------------------------------------------------------------------ #
    #  生命周期
    # ------------------------------------------------------------------ #
    def spin_async(self, threads: int = 4):
        """启动 MultiThreadedExecutor（ROS 回调在独立线程池里跑，不阻塞任务线程）。"""
        from rclpy.executors import MultiThreadedExecutor
        self._executor = MultiThreadedExecutor(num_threads=threads)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin,
                                             name="ros-executor", daemon=True)
        self._spin_thread.start()
        self._log(f"ROS executor 启动：{threads} 线程")
        return self._spin_thread

    def shutdown(self, *, brake: bool = True) -> None:
        """收尾：默认先发零指令（刹车是唯一安全的默认动作）。"""
        try:
            if brake:
                self.publish_brake()
        except Exception:
            pass
        try:
            if getattr(self, "_executor", None) is not None:
                self._executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            self._rclpy.shutdown()
        except Exception:
            pass

    def summary(self) -> dict[str, Any]:
        with self._lock:
            return {
                "odom_msgs": self.stats.odom_msgs,
                "vision_msgs": self.stats.vision_msgs,
                "commands_sent": self.stats.commands_sent,
                "zero_commands": self.stats.zero_commands,
                "pose_age_s": round(self.pose_age(), 3),
                "aim_status": ct.AimStatus.name(self._aim_status),
                "shoot_count": self._shoot_count,
                "ammo": self._ammo,
                "manual_override": self._manual_override,
            }
