"""测试用的极简 ROS2 替身。

目的：在没有 ROS 的机器（Windows / CI）上也能实例化真实节点类，
调用它们的回调并断言发布出去的消息。这样「节点这一层」不再是测试盲区。

用法：
    import ros_stub            # 必须在 import 节点模块之前
    from ag_integrate.integrate_node import IntegrateNode
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
            # 真 ROS 消息对未知字段名抛 TypeError；桩若静默吞掉，
            # 字段名写错的测试会一路绿灯到实机才炸。
            unknown = sorted(set(kw) - set(fields))
            if unknown:
                raise TypeError(
                    f"{name}() 没有字段 {unknown}（与真 ROS 消息行为一致）"
                )
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


class _Vector3:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Twist:
    def __init__(self) -> None:
        self.linear = _Vector3()
        self.angular = _Vector3()


class _Pose:
    def __init__(self) -> None:
        self.position = _Point()
        self.orientation = _Quaternion()


class PoseStamped:
    def __init__(self) -> None:
        self.header = Header()
        self.pose = _Pose()


class _PoseWithCovariance:
    def __init__(self) -> None:
        self.pose = _Pose()
        self.covariance = [0.0] * 36


class _TwistWithCovariance:
    def __init__(self) -> None:
        self.twist = Twist()
        self.covariance = [0.0] * 36


class Odometry:
    """nav_msgs/Odometry 替身：path_executor 只读 pose.pose 的 position/
    orientation，协方差等字段占位即可。"""

    def __init__(self) -> None:
        self.header = Header()
        self.child_frame_id = ""
        self.pose = _PoseWithCovariance()
        self.twist = _TwistWithCovariance()


def _pose_msg_class():
    return PoseStamped


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
        self.level = None  # set_level 仅记录，不过滤（测试要断言低级别日志）

    def set_level(self, level) -> None:
        self.level = level

    def _log(self, level, msg):
        self.records.append((level, str(msg)))

    def info(self, msg): self._log("info", msg)
    def warn(self, msg): self._log("warn", msg)
    def warning(self, msg): self._log("warning", msg)
    def error(self, msg): self._log("error", msg)
    def debug(self, msg): self._log("debug", msg)
    def fatal(self, msg): self._log("fatal", msg)


class SimClock:
    """全局可控仿真时钟（默认关闭）。

    启用后所有 FakeClock.now() 都读同一个全局时间，不再自动推进，
    由仿真循环调用 advance() 步进 —— path_executor 的 PIVOT 相位计时、
    integrate_node 的对齐超时等都吃同一个时间源，才能离线快进。
    """

    def __init__(self) -> None:
        self.enabled = False
        self.t = 0.0

    def enable(self, start: float = 0.0) -> None:
        self.enabled = True
        self.t = float(start)

    def disable(self) -> None:
        self.enabled = False

    def advance(self, dt: float) -> float:
        self.t += float(dt)
        return self.t


#: 全局仿真时钟单例
SIM = SimClock()


class FakeClock:
    def __init__(self) -> None:
        self._ns = 1_000_000_000  # 1s 起步，单调递增

    def now(self):
        if SIM.enabled:
            ns = int(SIM.t * 1e9)  # 仿真模式：读全局时间，不自动推进
        else:
            self._ns += 100_000_000  # 每次调用推进 0.1s（旧行为，保持不变）
            ns = self._ns
        return types.SimpleNamespace(
            nanoseconds=ns, to_msg=lambda: None,
            seconds_nanoseconds=lambda: (ns // 1_000_000_000, ns % 1_000_000_000),
        )


class FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.published: List[object] = []

    def publish(self, msg) -> None:
        self.published.append(msg)
        if BUS.active:
            BUS.dispatch(self.topic, msg)


class TopicBus:
    """进程内 topic 总线（默认关闭，仿真测试用 fixture 打开）。

    打开后：create_publisher/create_subscription 登记到总线，publish 时
    同步调用同 topic 的所有订阅回调；count_subscribers/count_publishers
    返回真实数量（integrate_node 的 wait_waypoint_consumer 闸门因此
    走真逻辑）。关闭时这两个 API 抛 RuntimeError —— 兼容
    integrate_node._waypoint_consumer_ready 的 except 兜底（老测试环境
    无订阅者概念，按"已就绪"放行，行为不变）。
    """

    def __init__(self) -> None:
        self.active = False
        self.subs: Dict[str, List[Callable]] = {}
        self.pubs: Dict[str, List[FakePublisher]] = {}

    def reset(self) -> None:
        self.active = True
        self.subs.clear()
        self.pubs.clear()

    def disable(self) -> None:
        self.active = False
        self.subs.clear()
        self.pubs.clear()

    def register_sub(self, topic: str, cb: Callable) -> None:
        self.subs.setdefault(topic, []).append(cb)

    def register_pub(self, topic: str, pub: FakePublisher) -> None:
        self.pubs.setdefault(topic, []).append(pub)

    def dispatch(self, topic: str, msg) -> None:
        # 拷一份订阅列表再回调：回调里可能再注册/发布，避免迭代中被改
        for cb in list(self.subs.get(topic, [])):
            cb(msg)

    def count_subscribers(self, topic: str) -> int:
        if not self.active:
            raise RuntimeError("ros_stub: topic 总线未启用，无法统计订阅者")
        return len(self.subs.get(topic, []))

    def count_publishers(self, topic: str) -> int:
        if not self.active:
            raise RuntimeError("ros_stub: topic 总线未启用，无法统计发布者")
        return len(self.pubs.get(topic, []))


#: 全局 topic 总线单例
BUS = TopicBus()


#: 参数覆盖表：用于在实例化节点前改写 declare_parameter 的默认值
_OVERRIDES: Dict[str, object] = {}


def set_overrides(**kw) -> None:
    _OVERRIDES.update(kw)


def clear_overrides() -> None:
    _OVERRIDES.clear()


class FakeTimer:
    """rclpy Timer 替身：可调用、可取消（cancel()）。

    真 rclpy 的 create_timer 返回 Timer 句柄；现役 path_executor 的
    _auto_start_mission 会调 timer.cancel()（一次性定时器触发即取消）。
    tick() 跳过已取消的定时器，与真语义一致。
    """

    def __init__(self, period: float, cb: Callable) -> None:
        self.timer_period_ns = int(period * 1e9)
        self._cb = cb
        self.cancelled = False

    def __call__(self) -> None:
        self._cb()

    def cancel(self) -> None:
        self.cancelled = True


class FakeNode:
    """节点基类替身：记录参数、发布器、订阅回调与定时器。"""

    def __init__(self, node_name: str) -> None:
        self.node_name = node_name
        self._params: Dict[str, object] = {}
        self.publishers: Dict[str, FakePublisher] = {}
        self.subscriptions: Dict[str, Callable] = {}
        self.timers: List[Callable] = []
        # 与真 rclpy 一致：logger 存 _logger，get_logger() 返回它。
        # （旧桩用独立的 self.logger 属性，blackbox attach() 写 node._logger
        # 时镜像链路静默失效 —— get_logger() 永远拿不到镜像。）
        self._logger = FakeLogger()
        self.clock = FakeClock()

    @property
    def logger(self) -> FakeLogger:
        """兼容测试直读 node.logger.records；与 get_logger() 同一对象。

        attach() 之后拿到的是 _LoggerMirror，records 经 __getattr__ 透传到
        被包裹的 FakeLogger，断言照旧成立。
        """
        return self._logger

    # ---- 参数 ----
    def declare_parameter(self, name: str, value=None):
        self._params[name] = _OVERRIDES.get(name, value)
        # 真 rclpy 返回 Parameter 对象（.value 取值），别返回 None
        return types.SimpleNamespace(name=name, value=self._params[name])

    def get_parameter(self, name: str):
        if name not in self._params:
            raise KeyError(f"未声明的参数: {name}")
        return types.SimpleNamespace(value=self._params[name])

    def set_param(self, name: str, value) -> None:
        """测试辅助：直接覆盖已声明的参数。"""
        self._params[name] = value

    # ---- 通信 ----
    def create_publisher(self, msg_type, topic: str, qos):
        pub = FakePublisher(topic)
        self.publishers[topic] = pub
        if BUS.active:
            BUS.register_pub(topic, pub)
        return pub

    def create_subscription(self, msg_type, topic: str, cb: Callable, qos):
        self.subscriptions[topic] = cb
        if BUS.active:
            BUS.register_sub(topic, cb)
        return cb

    def count_subscribers(self, topic: str) -> int:
        """总线启用时返回真实订阅数；未启用抛 RuntimeError，
        由调用方（如 _waypoint_consumer_ready）的 except 兜底。"""
        return BUS.count_subscribers(topic)

    def count_publishers(self, topic: str) -> int:
        return BUS.count_publishers(topic)

    def create_timer(self, period: float, cb: Callable):
        # 真 rclpy 返回 Timer 对象（带 cancel()）；现役 path_executor 的
        # 自动发车一次性定时器用到 cancel()，桩必须同款语义。
        t = FakeTimer(period, cb)
        self.timers.append(t)
        return t

    # ---- 其它 ----
    def get_logger(self):
        return self._logger

    def get_name(self) -> str:
        return self.node_name

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
        """依次触发所有定时器回调（已 cancel 的跳过，与真 rclpy 一致）。"""
        for cb in self.timers:
            if not getattr(cb, "cancelled", False):
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

    # QoS 替身：path_executor 模块级会构造 CMD_QOS = QoSProfile(...)，
    # 桩只记录参数，不做任何传输语义。
    qos_mod = types.ModuleType("rclpy.qos")

    class QoSProfile:
        def __init__(self, depth: int = 10, reliability=None, **kw) -> None:
            self.depth = depth
            self.reliability = reliability
            self.extra = kw

    class ReliabilityPolicy:
        RELIABLE = 1
        BEST_EFFORT = 2

    qos_mod.QoSProfile = QoSProfile
    qos_mod.ReliabilityPolicy = ReliabilityPolicy
    rclpy_mod.qos = qos_mod

    # 日志级别替身：path_executor 调 get_logger().set_level(LoggingSeverity.WARN)
    logging_mod = types.ModuleType("rclpy.logging")

    class LoggingSeverity:
        DEBUG = 10
        INFO = 20
        WARN = 30
        ERROR = 40
        FATAL = 50

    logging_mod.LoggingSeverity = LoggingSeverity
    rclpy_mod.logging = logging_mod

    _install("rclpy", rclpy_mod)
    _install("rclpy.node", node_mod)
    _install("rclpy.qos", qos_mod)
    _install("rclpy.logging", logging_mod)
    _package("std_msgs", {"String": String, "Header": Header,
                          "Int32": make_msg_class("Int32", {"data": 0}),
                          "Int32MultiArray": make_msg_class("Int32MultiArray", {"data": list}),
                          "UInt8": make_msg_class("UInt8", {"data": 0}),
                          "UInt8MultiArray": make_msg_class("UInt8MultiArray", {"data": list}),
                          "Float32": make_msg_class("Float32", {"data": 0.0})})
    _package("sensor_msgs", {"Image": Image})
    _package("geometry_msgs", {"PoseStamped": PoseStamped, "Point": _Point,
                               "Quaternion": _Quaternion, "Pose": _Pose,
                               "Twist": Twist, "Vector3": _Vector3})
    _package("nav_msgs", {"Odometry": Odometry})


install()
