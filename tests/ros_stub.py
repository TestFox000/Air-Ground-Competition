"""测试用的极简 ROS2 替身。

目的：在没有 ROS 的机器（Windows / CI）上也能实例化真实节点类，
调用它们的回调并断言发布出去的消息。这样「节点这一层」不再是测试盲区。

用法：
    import ros_stub            # 必须在 import 节点模块之前
    from ag_mission.mission_node import MissionNode
"""

from __future__ import annotations

import sys
import types
from typing import Callable, Dict, List

# ---------------------------------------------------------------- 消息替身
class Header:
    def __init__(self) -> None:
        self.stamp = None
        self.frame_id = ""


def make_msg_class(name: str, fields: Dict[str, object]):
    """fields: {字段名: 默认值或 callable（用于 list 等可变默认）}"""

    class M:
        _fields = fields

        def __init__(self, **kw):
            for f, d in fields.items():
                val = kw.get(f, d() if callable(d) else d)
                setattr(self, f, val)

        def __repr__(self) -> str:  # pragma: no cover - 调试用
            body = ", ".join(f"{f}={getattr(self, f)!r}" for f in fields)
            return f"{name}({body})"

    M.__name__ = M.__qualname__ = name
    return M


String = make_msg_class("String", {"data": ""})
Image = make_msg_class(
    "Image", {"header": Header, "height": 0, "width": 0, "encoding": "", "data": bytes}
)


class _Point:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Quaternion:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class _Pose:
    def __init__(self) -> None:
        self.position = _Point()
        self.orientation = _Quaternion()


class PoseStamped:
    def __init__(self) -> None:
        self.header = Header()
        self.pose = _Pose()


def _pose_msg_class():
    return PoseStamped
Detection = make_msg_class(
    "Detection",
    {"cls": 0, "score": 0.0, "x1": 0.0, "y1": 0.0, "x2": 0.0, "y2": 0.0,
     "kpt_x": 0.0, "kpt_y": 0.0, "kpt_conf": 0.0},
)
DetectionArray = make_msg_class("DetectionArray", {"header": Header, "detections": list})
FieldIntelMsg = make_msg_class(
    "FieldIntelMsg",
    {"header": Header, "mines": list, "obstacles": list, "bridge_a": "", "bridge_b": "",
     "target_cells": list, "target_face": "", "seq": 0},
)
GimbalCommand = make_msg_class(
    "GimbalCommand", {"header": Header, "yaw_delta": 0.0, "pitch_delta": 0.0,
                      "fire": False, "reason": ""},
)
RefereeStateMsg = make_msg_class(
    "RefereeStateMsg",
    {"header": Header, "ammo": 0, "shots": 0, "enemy_count": 0, "friend_count": 0,
     "illuminated": 0, "hp": 0},
)


def _install(name: str, module: types.ModuleType) -> None:
    sys.modules[name] = module


def _package(pkg: str, mapping: Dict[str, object]) -> None:
    """注册 <pkg> 与 <pkg>.msg 两个模块（模块名必须唯一，否则导入会串台）。"""
    mod = types.ModuleType(pkg)
    msg_mod = types.ModuleType(pkg + ".msg")
    for k, v in mapping.items():
        setattr(msg_mod, k, v)
    mod.msg = msg_mod
    _install(pkg, mod)
    _install(pkg + ".msg", msg_mod)


# ---------------------------------------------------------------- rclpy 替身
class FakeLogger:
    def __init__(self) -> None:
        self.records: List[tuple] = []

    def _log(self, level, msg):
        self.records.append((level, str(msg)))

    def info(self, msg): self._log("info", msg)
    def warn(self, msg): self._log("warn", msg)
    def warning(self, msg): self._log("warning", msg)
    def error(self, msg): self._log("error", msg)
    def debug(self, msg): self._log("debug", msg)


class FakeClock:
    def now(self):
        return types.SimpleNamespace(to_msg=lambda: None)


class FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.published: List[object] = []

    def publish(self, msg) -> None:
        self.published.append(msg)


#: 参数覆盖表：用于在实例化节点前改写 declare_parameter 的默认值
_OVERRIDES: Dict[str, object] = {}


def set_overrides(**kw) -> None:
    _OVERRIDES.update(kw)


def clear_overrides() -> None:
    _OVERRIDES.clear()


class FakeNode:
    """节点基类替身：记录参数、发布器、订阅回调与定时器。"""

    def __init__(self, node_name: str) -> None:
        self.node_name = node_name
        self._params: Dict[str, object] = {}
        self.publishers: Dict[str, FakePublisher] = {}
        self.subscriptions: Dict[str, Callable] = {}
        self.timers: List[Callable] = []
        self.logger = FakeLogger()
        self.clock = FakeClock()

    # ---- 参数 ----
    def declare_parameter(self, name: str, value=None):
        self._params[name] = _OVERRIDES.get(name, value)

    def get_parameter(self, name: str):
        if name not in self._params:
            raise KeyError(f"未声明的参数: {name}")
        return types.SimpleNamespace(value=self._params[name])

    def set_param(self, name: str, value) -> None:
        """测试辅助：直接覆盖已声明的参数。"""
        self._params[name] = value

    # ---- 通信 ----
    def create_publisher(self, msg_type, topic: str, qos: int):
        pub = FakePublisher(topic)
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic: str, cb: Callable, qos: int):
        self.subscriptions[topic] = cb
        return cb

    def create_timer(self, period: float, cb: Callable):
        self.timers.append(cb)
        return cb

    # ---- 其它 ----
    def get_logger(self):
        return self.logger

    def get_clock(self):
        return self.clock

    def destroy_node(self):
        pass

    # ---- 测试辅助 ----
    def deliver(self, topic: str, msg) -> None:
        """把消息投递给订阅该话题的回调。"""
        cb = self.subscriptions.get(topic)
        if cb is None:
            raise AssertionError(f"节点 {self.node_name} 未订阅 {topic}，现有: {list(self.subscriptions)}")
        cb(msg)

    def tick(self) -> None:
        """依次触发所有定时器回调。"""
        for cb in self.timers:
            cb()

    def topic(self, name: str) -> FakePublisher:
        return self.publishers[name]


def install() -> None:
    """把替身装进 sys.modules。重复调用是安全的。"""
    if getattr(sys.modules.get("rclpy"), "_is_stub", False):
        return

    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod._is_stub = True
    rclpy_mod._inited = False
    rclpy_mod.init = lambda args=None: None
    rclpy_mod.shutdown = lambda: None
    rclpy_mod.spin = lambda node, *a, **k: None

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = FakeNode
    rclpy_mod.node = node_mod

    _install("rclpy", rclpy_mod)
    _install("rclpy.node", node_mod)
    _package("std_msgs", {"String": String, "Header": Header,
                          "Int32": make_msg_class("Int32", {"data": 0}),
                          "Int32MultiArray": make_msg_class("Int32MultiArray", {"data": list}),
                          "UInt8": make_msg_class("UInt8", {"data": 0}),
                          "UInt8MultiArray": make_msg_class("UInt8MultiArray", {"data": list}),
                          "Float32": make_msg_class("Float32", {"data": 0.0})})
    _package("sensor_msgs", {"Image": Image})
    _package("geometry_msgs", {"PoseStamped": PoseStamped, "Point": _Point,
                               "Quaternion": _Quaternion, "Pose": _Pose})
    _package(
        "ag_interfaces",
        {
            "Detection": Detection,
            "DetectionArray": DetectionArray,
            "FieldIntelMsg": FieldIntelMsg,
            "GimbalCommand": GimbalCommand,
            "RefereeStateMsg": RefereeStateMsg,
        },
    )


install()
