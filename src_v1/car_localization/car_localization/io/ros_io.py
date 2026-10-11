"""io/ros_io.py — ROS 话题封装（RosBridge）。

从 path_executor.py V1 __init__ (L198-387) 拆出：
  - 所有话题订阅 / 发布（保持原话题名和 QoS）
  - 所有定时器创建
  - 回调只做三件事：解码 + 时间戳 + 缓存（不写入状态变量）
  - check_odom_publishers 健康检查

设计模式：鸭子类型桥接（惰性导入 rclpy，单测可无 ROS）。
V1 继承 Node → V2 在已有 node 上创建话题，**不创建新 node**，
确保 main() 中 rclpy.spin(node) 可以处理所有话题和定时器。

话题命名与 V1 逐字一致：
  订阅: /odom/localization, /nav/waypoint, /shoot_count, /aim/status
  发布: pursuitAngle, /move_base/stop_signal, /nav/scan_request
  QoS: 控制指令使用 BEST_EFFORT depth=1 (CMD_QOS)
"""

import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, List, Optional


# ── 桥接消息结构 ──────────────────────────────────────────────
# RosBridge 回调解码后的消息体（不依赖 rclpy 消息类型）


@dataclass
class OdomFrame:
    """原始里程计帧（解码后的 /odom/localization）。"""
    raw_x: float        # 块坐标 x（格单位）
    raw_y: float        # 块坐标 y（格单位）
    qx: float           # 四元数 x
    qy: float           # 四元数 y
    qz: float           # 四元数 z
    qw: float           # 四元数 w
    ts: float           # 时间戳 (time.monotonic)


@dataclass
class WaypointMsg:
    """路线命令消息（解码后的 /nav/waypoint）。"""
    data: str           # 命令文本
    ts: float           # 时间戳


@dataclass
class IntMsg:
    """整数消息（/shoot_count, /aim/status 通用）。"""
    data: int
    ts: float


# ── RosBridge ─────────────────────────────────────────────────

class RosBridge:
    """ROS 话题桥接：封装所有订阅/发布，保持 V1 原话题名和 QoS。

    在已有 rclpy.node.Node 上创建话题和定时器，**不创建新 node**。
    确保 main() 中 rclpy.spin(node) 可以处理所有话题和定时器。

    使用方式：
      node = rclpy.node.Node('path_executor_node')
      bridge = RosBridge(node, logger=log)

      # 注入 IO 回调（话题数据 → 业务层）
      bridge.on_odom = lambda frame: handle_odom(frame)
      bridge.on_waypoint = lambda msg: handle_waypoint(msg)

      # 发布指令
      bridge.publish_command(0.3, 15.0)
      bridge.publish_brake()

      # 停止
      bridge.shutdown()
    """

    # V1 L99-103: 控制指令 QoS（BEST_EFFORT depth=1）
    CMD_QOS_DEPTH = 1
    CMD_QOS_RELIABILITY = 'BEST_EFFORT'

    # V1 L306-311: 订阅队列深度
    SUB_QUEUE_DEPTH = 10

    # V1 L358-364: 定时器参数
    CONTROL_PERIOD_S = 0.02   # 20 Hz
    HEALTH_CHECK_DELAY_S = 3.0  # 一次 3s 后

    def __init__(
        self,
        node: Any,
        logger: Optional[Any] = None,
    ):
        """初始化 RosBridge。

        Args:
            node: 已有的 rclpy.node.Node 实例（话题/定时器都在此 node 上创建）
            logger: 日志对象（可选，None 时用 print）
        """
        self._node: Any = node
        self._log: Any = logger
        self._node_name: str = getattr(node, 'get_name', lambda: 'bridge')()
        if hasattr(self._node_name, '__call__'):
            try:
                self._node_name = self._node_name()
            except Exception:
                self._node_name = 'bridge'
        self._log_impl: Callable[..., None] = self._default_logger

        # ── 发布器引用 ──────────────────────────────────────
        self._cmd_pub: Any = None           # pursuitAngle
        self._stop_signal_pub: Any = None   # /move_base/stop_signal
        self._scan_req_pub: Any = None      # /nav/scan_request

        # ── 订阅器引用 ──────────────────────────────────────
        self._odom_sub: Any = None
        self._waypoint_sub: Any = None
        self._shoot_count_sub: Any = None
        self._aim_status_sub: Any = None

        # ── 定时器引用 ──────────────────────────────────────
        self._control_timer: Any = None
        self._health_timer: Any = None
        self._health_timer_done: bool = False

        # ── 缓存（回调解码后的数据，供业务层读取）─────────────
        self._latest_odom: Optional[OdomFrame] = None
        self._latest_waypoint: Optional[WaypointMsg] = None
        self._latest_shoot_count: Optional[IntMsg] = None
        self._latest_aim_status: Optional[IntMsg] = None

        # 缓存保护锁
        self._cache_lock: threading.Lock = threading.Lock()

        # ── IO 回调（业务层注册，数据解码后调用）─────────────
        self._on_odom: Optional[Callable[[OdomFrame], None]] = None
        self._on_waypoint: Optional[Callable[[WaypointMsg], None]] = None
        self._on_shoot_count: Optional[Callable[[IntMsg], None]] = None
        self._on_aim_status: Optional[Callable[[IntMsg], None]] = None

        # 控制循环回调
        self._on_control_tick: Optional[Callable[[], None]] = None

        # 健康检查回调
        self._on_health_check: Optional[Callable[[int], None]] = None

        # ── 立即在 node 上创建话题和定时器 ──────────────────
        self._create_ros_entities()

    def _default_logger(self, msg: str, level: str = 'info') -> None:
        """默认日志。"""
        tag = level.upper() if level else 'INFO'
        print(f"[{tag}][{self._node_name}] {msg}")

    def _log_msg(self, msg: str, level: str = 'info') -> None:
        """统一日志输出。"""
        if self._log is not None:
            try:
                if hasattr(self._log, '__call__'):
                    self._log(msg)
                elif hasattr(self._log, level):
                    getattr(self._log, level)(msg)
                else:
                    self._default_logger(msg, level)
            except Exception:
                self._default_logger(msg, level)
        else:
            self._default_logger(msg, level)

    # ── 属性：IO 回调注册 ────────────────────────────────────

    @property
    def on_odom(self) -> Optional[Callable[[OdomFrame], None]]:
        return self._on_odom

    @on_odom.setter
    def on_odom(self, cb: Optional[Callable[[OdomFrame], None]]) -> None:
        self._on_odom = cb

    @property
    def on_waypoint(self) -> Optional[Callable[[WaypointMsg], None]]:
        return self._on_waypoint

    @on_waypoint.setter
    def on_waypoint(self, cb: Optional[Callable[[WaypointMsg], None]]) -> None:
        self._on_waypoint = cb

    @property
    def on_shoot_count(self) -> Optional[Callable[[IntMsg], None]]:
        return self._on_shoot_count

    @on_shoot_count.setter
    def on_shoot_count(self, cb: Optional[Callable[[IntMsg], None]]) -> None:
        self._on_shoot_count = cb

    @property
    def on_aim_status(self) -> Optional[Callable[[IntMsg], None]]:
        return self._on_aim_status

    @on_aim_status.setter
    def on_aim_status(self, cb: Optional[Callable[[IntMsg], None]]) -> None:
        self._on_aim_status = cb

    @property
    def on_control_tick(self) -> Optional[Callable[[], None]]:
        return self._on_control_tick

    @on_control_tick.setter
    def on_control_tick(self, cb: Optional[Callable[[], None]]) -> None:
        self._on_control_tick = cb

    @property
    def on_health_check(self) -> Optional[Callable[[int], None]]:
        return self._on_health_check

    @on_health_check.setter
    def on_health_check(self, cb: Optional[Callable[[int], None]]) -> None:
        self._on_health_check = cb

    # ── 只读：缓存读取（供业务层用）───────────────────────────

    def get_latest_odom(self) -> Optional[OdomFrame]:
        """获取最新里程计帧（线程安全）。"""
        with self._cache_lock:
            return self._latest_odom

    def get_shoot_count(self) -> int:
        """获取最新射弹数。等价于 V1 self.shoot_count。"""
        with self._cache_lock:
            if self._latest_shoot_count is not None:
                return self._latest_shoot_count.data
            return 0

    def get_aim_status(self) -> int:
        """获取最新云台状态 (0=IDLE 1=SCAN 2=TRACK)。"""
        with self._cache_lock:
            if self._latest_aim_status is not None:
                return self._latest_aim_status.data
            return 0

    def get_waypoint(self) -> Optional[WaypointMsg]:
        """获取最新路线命令。"""
        with self._cache_lock:
            return self._latest_waypoint

    # ── ROS 话题／定时器创建（在已有 node 上）────────────────

    def _create_ros_entities(self) -> None:
        """在已有 node 上创建发布器、订阅器和定时器。

        所有话题名和 QoS 与 V1 完全相同。
        """
        if self._node is None:
            return

        try:
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from nav_msgs.msg import Odometry
            from geometry_msgs.msg import Twist
            from std_msgs.msg import String, UInt8, Int32
        except ImportError:
            return

        # ── 构建 QoS（V1 L99-103）────────────────────────────
        cmd_qos = QoSProfile(
            depth=self.CMD_QOS_DEPTH,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # ── 创建发布器 ──────────────────────────────────────

        # V1 L331-335: pursuitAngle (Twist, CMD_QOS)
        self._cmd_pub = self._node.create_publisher(
            Twist, 'pursuitAngle', cmd_qos)

        # V1 L339-343: /move_base/stop_signal (UInt8, depth=10)
        self._stop_signal_pub = self._node.create_publisher(
            UInt8, '/move_base/stop_signal', 10)

        # V1 L356: /nav/scan_request (UInt8, depth=10)
        self._scan_req_pub = self._node.create_publisher(
            UInt8, '/nav/scan_request', 10)

        # ── 创建订阅 ────────────────────────────────────────

        # V1 L306-311: /odom/localization (Odometry, depth=10)
        self._odom_sub = self._node.create_subscription(
            Odometry, '/odom/localization',
            self._rclpy_odom_callback, 10)

        # V1 L316-321: /nav/waypoint (String, depth=10)
        self._waypoint_sub = self._node.create_subscription(
            String, '/nav/waypoint',
            self._rclpy_waypoint_callback, 10)

        # V1 L346-347: /shoot_count (Int32, depth=10)
        self._shoot_count_sub = self._node.create_subscription(
            Int32, '/shoot_count',
            self._rclpy_shoot_count_callback, 10)

        # V1 L352-353: /aim/status (Int32, depth=10)
        self._aim_status_sub = self._node.create_subscription(
            Int32, '/aim/status',
            self._rclpy_aim_status_callback, 10)

        # ── 创建定时器 ──────────────────────────────────────

        # V1 L359: 20Hz 控制定时器
        self._control_timer = self._node.create_timer(
            self.CONTROL_PERIOD_S, self._rclpy_control_timer_callback)

        # V1 L364: 一次 3s 健康检查
        self._health_timer = self._node.create_timer(
            self.HEALTH_CHECK_DELAY_S, self._rclpy_health_check_callback)

    # ── rclpy 回调（内部使用，负责解码 + 缓存 + 通知）────────

    def _rclpy_odom_callback(self, msg: Any) -> None:
        """/odom/localization 回调（V1 L829-851）。

        只做三件事：解码 + 缓存 + 通知 on_odom。
        """
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        frame = OdomFrame(
            raw_x=pos.x,
            raw_y=pos.y,
            qx=q.x, qy=q.y, qz=q.z, qw=q.w,
            ts=time.monotonic(),
        )
        with self._cache_lock:
            self._latest_odom = frame
        if self._on_odom:
            try:
                self._on_odom(frame)
            except Exception as e:
                self._log_msg(f"odom callback error: {e}", level='error')

    def _rclpy_waypoint_callback(self, msg: Any) -> None:
        """/nav/waypoint 回调（V1 L467-600）。"""
        wm = WaypointMsg(data=msg.data, ts=time.monotonic())
        with self._cache_lock:
            self._latest_waypoint = wm
        if self._on_waypoint:
            try:
                self._on_waypoint(wm)
            except Exception as e:
                self._log_msg(f"waypoint callback error: {e}", level='error')

    def _rclpy_shoot_count_callback(self, msg: Any) -> None:
        """/shoot_count 回调（V1 L739-743）。"""
        im = IntMsg(data=int(msg.data), ts=time.monotonic())
        with self._cache_lock:
            self._latest_shoot_count = im
        if self._on_shoot_count:
            try:
                self._on_shoot_count(im)
            except Exception as e:
                self._log_msg(f"shoot_count callback error: {e}", level='error')

    def _rclpy_aim_status_callback(self, msg: Any) -> None:
        """/aim/status 回调（V1 L745-747）。"""
        im = IntMsg(data=int(msg.data), ts=time.monotonic())
        with self._cache_lock:
            self._latest_aim_status = im
        if self._on_aim_status:
            try:
                self._on_aim_status(im)
            except Exception as e:
                self._log_msg(f"aim_status callback error: {e}", level='error')

    def _rclpy_control_timer_callback(self) -> None:
        """控制定时器回调（V1 L1036-1372 control_loop 的 20Hz 入口）。"""
        if self._on_control_tick:
            try:
                self._on_control_tick()
            except Exception as e:
                self._log_msg(f"control tick error: {e}", level='error')

    def _rclpy_health_check_callback(self) -> None:
        """健康检查回调（V1 L393-416 check_odom_publishers）。"""
        if not self._health_timer_done:
            self._health_timer_done = True
            if self._health_timer is not None:
                try:
                    self._health_timer.cancel()
                except Exception:
                    pass

        if self._node is not None:
            try:
                count = self._node.count_publishers('/odom/localization')
                if self._on_health_check:
                    self._on_health_check(count)
                else:
                    if count > 1:
                        self._log_msg(
                            f"{count} publishers on /odom/localization! "
                            "Kill the extra node, otherwise the pose is corrupted",
                            level='error')
                    elif count == 0:
                        self._log_msg(
                            "No publisher on /odom/localization yet",
                            level='warn')
                    else:
                        self._log_msg(
                            "Localization OK: 1 publisher on /odom/localization")
            except Exception:
                pass

    # ── 发布方法（业务层调用）─────────────────────────────────

    def publish_command(self, linear_x: float, steer_deg: float) -> None:
        """发布控制指令到 pursuitAngle（V1 L1353-1356）。"""
        if self._cmd_pub is None:
            return
        try:
            from geometry_msgs.msg import Twist
            cmd = Twist()
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(steer_deg)
            self._cmd_pub.publish(cmd)
        except ImportError:
            pass

    def publish_brake(self) -> None:
        """发布零控制指令（V1 L1050-1051, L1170, L1394-1395）。"""
        self.publish_command(0.0, 0.0)

    def publish_stop_signal(self, allow: bool = True) -> None:
        """发布停站信号到 /move_base/stop_signal（V1 L820-822）。"""
        if self._stop_signal_pub is None:
            return
        try:
            from std_msgs.msg import UInt8
            sig = UInt8()
            sig.data = 1 if allow else 0
            self._stop_signal_pub.publish(sig)
        except ImportError:
            pass

    def publish_scan_request(self, mode: int = 1) -> None:
        """发布扫描请求到 /nav/scan_request（V1 L760, L1213）。"""
        if self._scan_req_pub is None:
            return
        try:
            from std_msgs.msg import UInt8
            req = UInt8()
            req.data = mode
            self._scan_req_pub.publish(req)
        except ImportError:
            pass

    # ── ROS 生命周期管理 ─────────────────────────────────────

    def shutdown(self, brake: bool = True) -> None:
        """优雅停止：发布刹车 + 取消定时器。

        注意：node 由 PathExecutorNode 管理（main() 中 destroy_node），
        RosBridge 不销毁 node。
        """
        if brake:
            try:
                self.publish_brake()
            except Exception:
                pass

        if self._control_timer is not None:
            try:
                self._control_timer.cancel()
            except Exception:
                pass

        if self._health_timer is not None:
            try:
                self._health_timer.cancel()
            except Exception:
                pass
