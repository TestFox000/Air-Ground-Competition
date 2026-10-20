"""Pytest bootstrap: make ag_integrate / dznav importable from src/*/, and
stub out rclpy + ROS message packages so pure-logic modules can be tested on a
dev machine without a ROS2 installation.

The stubs are deliberately minimal and only installed when the real package is
absent, so running this suite on the robot (where rclpy exists) uses the real
thing.

The fake Node is functional rather than a bare `object` because dznav's safety
logic (NaN guard, link watchdog, estop latch) lives in the node itself and had
zero test coverage — these stubs are what make it testable off-robot.
"""
import sys
import types
from pathlib import Path

_ROOT = Path(__file__).parent
for _pkg_root in (_ROOT / "src" / "ag_integrate", _ROOT / "src" / "dznav"):
    _p = str(_pkg_root)
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _have(name: str) -> bool:
    try:
        __import__(name)
        return True
    except Exception:
        return False


def _stub(name: str, **attrs) -> types.ModuleType:
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules[name] = mod
    return mod


class _AnyMsg:
    """Permissive stand-in for a ROS message type."""

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class _AutoMsg(_AnyMsg):
    """Auto-vivifying message: `Twist().linear.x = 1` works like the real thing.

    Only unset attributes auto-create; anything the test assigns stays put.
    Concrete message classes below override this with faithful ROS defaults —
    auto-vivification alone would let `Twist().linear.x` be an object instead
    of 0.0, which silently masks "we published garbage" bugs.
    """

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        v = _AutoMsg()
        object.__setattr__(self, name, v)
        return v


class _Vec3(_AutoMsg):
    def __init__(self, x=0.0, y=0.0, z=0.0, **kw):
        self.x, self.y, self.z = float(x), float(y), float(z)


class _Quaternion(_AutoMsg):
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0, **kw):
        self.x, self.y, self.z, self.w = (float(x), float(y), float(z),
                                          float(w))


class _Point(_AutoMsg):
    def __init__(self, x=0.0, y=0.0, z=0.0, **kw):
        self.x, self.y, self.z = float(x), float(y), float(z)


class _Stamp(_AutoMsg):
    def __init__(self, sec=0, nanosec=0, **kw):
        self.sec, self.nanosec = int(sec), int(nanosec)


class _Header(_AutoMsg):
    def __init__(self, stamp=None, frame_id="", **kw):
        self.stamp = stamp if stamp is not None else _Stamp()
        self.frame_id = frame_id


class _Pose(_AutoMsg):
    def __init__(self, position=None, orientation=None, **kw):
        self.position = position if position is not None else _Point()
        self.orientation = (orientation if orientation is not None
                            else _Quaternion())


class _PoseWithCovariance(_AutoMsg):
    def __init__(self, pose=None, covariance=None, **kw):
        self.pose = pose if pose is not None else _Pose()
        self.covariance = covariance if covariance is not None else [0.0] * 36


class _TwistMsg(_AutoMsg):
    def __init__(self, linear=None, angular=None, **kw):
        self.linear = linear if linear is not None else _Vec3()
        self.angular = angular if angular is not None else _Vec3()


class _TwistWithCovariance(_AutoMsg):
    def __init__(self, twist=None, covariance=None, **kw):
        self.twist = twist if twist is not None else _TwistMsg()
        self.covariance = covariance if covariance is not None else [0.0] * 36


class _OdometryMsg(_AutoMsg):
    def __init__(self, header=None, pose=None, twist=None, child_frame_id="",
                 **kw):
        self.header = header if header is not None else _Header()
        self.pose = pose if pose is not None else _PoseWithCovariance()
        self.twist = twist if twist is not None else _TwistWithCovariance()
        self.child_frame_id = child_frame_id


class _ImuMsg(_AutoMsg):
    def __init__(self, header=None, orientation=None, angular_velocity=None,
                 linear_acceleration=None, **kw):
        self.header = header if header is not None else _Header()
        self.orientation = (orientation if orientation is not None
                            else _Quaternion())
        self.angular_velocity = (angular_velocity if angular_velocity is not None
                                 else _Vec3())
        self.linear_acceleration = (linear_acceleration
                                    if linear_acceleration is not None
                                    else _Vec3())
        self.orientation_covariance = [0.0] * 9
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration_covariance = [0.0] * 9


class _PoseStampedMsg(_AutoMsg):
    def __init__(self, header=None, pose=None, **kw):
        self.header = header if header is not None else _Header()
        self.pose = pose if pose is not None else _Pose()


class _PoseWithCovarianceStampedMsg(_AutoMsg):
    def __init__(self, header=None, pose=None, **kw):
        self.header = header if header is not None else _Header()
        self.pose = pose if pose is not None else _PoseWithCovariance()


def _scalar(default):
    def _init(self, data=None, **kw):
        self.data = default() if data is None else data
    return type("_Scalar", (_AutoMsg,), {"__init__": _init})


#: 消息名 -> (构造器)。未列出的消息退回纯 auto-vivify。
_MSG_TYPES = {
    "geometry_msgs.msg": {
        "Twist": _TwistMsg, "Vector3": _Vec3, "Quaternion": _Quaternion,
        "Point": _Point, "Pose": _Pose, "PoseStamped": _PoseStampedMsg,
        "PoseWithCovariance": _PoseWithCovariance,
        "PoseWithCovarianceStamped": _PoseWithCovarianceStampedMsg,
        "TwistWithCovariance": _TwistWithCovariance,
    },
    "std_msgs.msg": {
        "String": _scalar(lambda: ""), "UInt8": _scalar(int),
        "Int32": _scalar(int), "Int64": _scalar(int),
        "Float32": _scalar(float), "Float64": _scalar(float),
        "Bool": _scalar(bool), "Header": _Header, "Empty": _AutoMsg,
        "ColorRGBA": _scalar(lambda: None),
    },
    "nav_msgs.msg": {"Odometry": _OdometryMsg, "Path": _AutoMsg},
    "sensor_msgs.msg": {"Imu": _ImuMsg, "Image": _AutoMsg,
                        "CameraInfo": _AutoMsg},
}

for _mod, _ctors in _MSG_TYPES.items():
    if not _have(_mod.split(".")[0]):
        # 统一再套一层具名子类，让 repr/报错里显示 UInt8 而不是 _Scalar。
        _stub(_mod, **{name: type(name, (ctor,), {})
                       for name, ctor in _ctors.items()})
        _parent = _mod.split(".")[0]
        if _parent not in sys.modules:
            _stub(_parent)


# ---------------------------------------------------------------------------
# Fake rclpy
# ---------------------------------------------------------------------------
class _EnumMeta(type):
    """Lets `ReliabilityPolicy.BEST_EFFORT` resolve without enumerating members."""

    def __getattr__(cls, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return name


class _Policy(metaclass=_EnumMeta):
    def __init__(self, *a, **k):
        pass


class _FakeTime:
    def __init__(self, ns: int):
        self.nanoseconds = int(ns)


class _FakeClock:
    def __init__(self):
        self._ns = 0

    def now(self):
        return _FakeTime(self._ns)

    def advance(self, seconds: float):
        self._ns += int(round(seconds * 1e9))

    def set(self, seconds: float):
        self._ns = int(round(seconds * 1e9))


class _FakeLogger:
    def __init__(self):
        self.records: list[tuple[str, str]] = []

    def _emit(self, level, message, **kwargs):
        self.records.append((level, str(message)))

    def debug(self, message, **kwargs):
        self._emit("debug", message, **kwargs)

    def info(self, message, **kwargs):
        self._emit("info", message, **kwargs)

    def warn(self, message, **kwargs):
        # 故意不提供：rclpy 的 RcutilsLogger 只有 warning()，warn() 在 humble 里
        # 是标注 "Deprecated in favor of RcutilsLogger.warning" 的别名，到
        # jazzy/rolling 已被删除（实测 rclpy/impl/rcutils_logger.py）。这里跟着
        # 最严的那一版走，谁再写 get_logger().warn(...) 测试当场 AttributeError，
        # 不会等到换发行版才在车上炸。
        raise AttributeError(
            "rclpy Logger 没有 warn()，请用 warning()"
            "（humble 的 warn 是已废弃别名，jazzy 起已删除）")

    def warning(self, message, **kwargs):
        self._emit("warning", message, **kwargs)

    def error(self, message, **kwargs):
        self._emit("error", message, **kwargs)

    def fatal(self, message, **kwargs):
        self._emit("fatal", message, **kwargs)

    def text(self, level=None, contains=""):
        return [m for lv, m in self.records
                if (level is None or lv == level) and contains in m]


class _FakePublisher:
    def __init__(self, topic: str, msg_type):
        self.topic = topic
        self.msg_type = msg_type
        self.sent: list = []

    def publish(self, msg):
        self.sent.append(msg)

    @property
    def last(self):
        return self.sent[-1] if self.sent else None

    def clear(self):
        self.sent.clear()


class _FakeSubscription:
    def __init__(self, topic, msg_type, callback, qos):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback
        self.qos = qos


class _FakeTimer:
    """真 rclpy 的 create_timer 返回带 cancel() 的 Timer 对象，桩必须同款——
    返回 None 会让"一次性定时器"这类逻辑在测试里根本无法表达，也已经导致
    tests/ros_stub.py 另起一套 FakeTimer（两套桩语义分叉，见其 :403 注释）。

    timers 里仍存 (period, cb) 元组以兼容既有消费方
    （tests/test_sim_tracking_dznav.py:115 按二元组解包）；句柄另存一份，
    供 cancel 语义断言。
    """

    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.canceled = False
        self.cancel_count = 0

    def cancel(self):
        self.canceled = True
        self.cancel_count += 1

    def is_canceled(self):
        return self.canceled


class _FakeNode:
    """Just enough of rclpy.node.Node to drive a node body deterministically."""

    def __init__(self, node_name, **kwargs):
        self._node_name = node_name
        self._logger = _FakeLogger()
        self._clock = _FakeClock()
        self.publishers: dict[str, _FakePublisher] = {}
        self.subscriptions: list[_FakeSubscription] = []
        self.timers: list[tuple[float, object]] = []
        self.timer_handles: list[_FakeTimer] = []
        self.destroyed = False

    # --- rclpy surface ---
    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def create_publisher(self, msg_type, topic, qos=10):
        pub = _FakePublisher(topic, msg_type)
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos=10):
        sub = _FakeSubscription(topic, msg_type, callback, qos)
        self.subscriptions.append(sub)
        return sub

    def create_timer(self, period, callback, **kwargs):
        t = _FakeTimer(period, callback)
        self.timers.append((period, callback))
        self.timer_handles.append(t)
        return t

    def count_publishers(self, topic):
        return 1

    def count_subscribers(self, topic):
        return 1

    def declare_parameter(self, name, value=None, descriptor=None):
        return type("P", (), {"value": value})()

    def get_parameter(self, name):
        return type("P", (), {"value": None})()

    def destroy_node(self):
        self.destroyed = True

    # --- test helpers (not part of rclpy) ---
    def sub_for(self, topic):
        for s in self.subscriptions:
            if s.topic == topic:
                return s
        raise KeyError(f"no subscription on {topic!r}")

    def inject(self, topic, msg):
        """Deliver `msg` to the subscription callback on `topic`."""
        self.sub_for(topic).callback(msg)


if not _have("rclpy"):
    _rclpy = _stub("rclpy",
                   init=lambda *a, **k: None,
                   shutdown=lambda *a, **k: None,
                   spin=lambda *a, **k: None,
                   spin_once=lambda *a, **k: None,
                   ok=lambda *a, **k: True)
    _rclpy.on_shutdown = lambda cb, *a, **k: None
    _stub("rclpy.node", Node=_FakeNode)
    _stub("rclpy.qos",
          QoSProfile=_AnyMsg, ReliabilityPolicy=_Policy,
          DurabilityPolicy=_Policy, HistoryPolicy=_Policy,
          QoSPresetProfiles=_Policy,
          qos_profile_sensor_data=_AnyMsg())
    _stub("rclpy.executors", SingleThreadedExecutor=object,
          MultiThreadedExecutor=object)
    _stub("rclpy.callback_groups", ReentrantCallbackGroup=object,
          MutuallyExclusiveCallbackGroup=object)
