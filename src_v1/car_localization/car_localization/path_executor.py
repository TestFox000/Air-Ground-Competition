# -*- coding: utf-8 -*-
"""
Path tracking executor node (path_executor) — V2 薄外壳组合根。

与 V1 保持完全相同的：
  - __init__(self, path_file=None) 签名
  - main(args=None) 入口
  - launch.py 引用路径（car_localization.path_executor）
  - 话题名（/odom/localization, /nav/waypoint, pursuitAngle 等）
  - control_loop 20Hz 行为

内部组合 V2 子模块：
  io/ros_io.py  →  RosBridge（所有 ROS 话题）
  core/blackboard.py → 线程安全共享状态
  core/geometry.py  → 几何工具函数
  route/parser.py   → 路线解析
  pursuit/ → 投影/前视/舵角
  tasks/  → 定位/localization, 循迹/tracker, 任务调度/mission
"""

import math
import sys
import time as time_module

# ---- V2 子模块导入 ----
from .core.geometry import (
    COL_LETTERS, BLOCK_SIZE, RAW_YAW_TO_MAP_YAW_OFFSET,
    euler_from_quaternion, normalize_angle, block_to_meters, heading_to_quaternion,
)
from .core.blackboard import (
    Blackboard,
    P_POSE_X, P_POSE_Y, P_POSE_YAW, P_HAS_ODOM,
    P_SNAP_X_OFF, P_SNAP_Y_OFF, P_SNAP_YAW_OFF,
    P_RAW_X, P_RAW_Y, P_RAW_MAP_YAW,
    P_PATH_POINTS, P_CURRENT_TARGET_IDX, P_PATH_LOADED,
    P_ARRIVED, P_ARRIVE_LOGGED,
    P_SHOOT_FLAGS, P_SHOOT_WAIT, P_SHOOT_WAIT_START,
    P_SHOOT_COUNT_AT_ARRIVAL, P_SHOOT_COUNT,
    P_SCAN_FLAGS, P_SCAN_WAIT, P_SCAN_WAIT_START,
    P_SCAN_SEEN_SCANNING, P_AIM_STATUS,
    P_CORNER_MODE, P_CORNER_TARGET_YAW, P_CORNER_TURN_DIR,
    P_CORNER_PHASE, P_CORNER_PHASE_T, P_CORNER_SEG_IDX,
)
from .pursuit.path import find_nearest_segment_v1
from .pursuit.lookahead import find_lookahead_point_v1_dict
from .pursuit.controller import pure_pursuit_control_v1, brake_command
from .io.ros_io import RosBridge, OdomFrame, WaypointMsg, IntMsg

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String, UInt8, Int32
except ImportError:
    # 当无 ROS 环境时（如测试），定义桩
    class Node:
        """测试桩：替代 rclpy.node.Node。"""
        def __init__(self, *args, **kwargs):
            pass

        def get_logger(self):
            class _Log:
                def info(self, *a, **kw): pass
                def warn(self, *a, **kw): pass
                def error(self, *a, **kw): pass
            return _Log()

        def get_clock(self):
            import time
            class _Clock:
                def now(self):
                    class _Now:
                        def seconds_nanoseconds(self):
                            t = time.time()
                            return int(t), int((t - int(t)) * 1e9)
                    return _Now()
            return _Clock()

        def count_publishers(self, topic):
            return 1

        def destroy_node(self):
            pass

        def create_timer(self, *a, **kw):
            return None


# ##### 薄外壳节点 #####
# PathExecutorNode 保持 V1 的 Node 继承和 __init__ 签名，
# 内部使用 RosBridge 处理 ROS 话题，Blackboard 共享状态，
# 由 control_loop 调度各子模块逻辑。

class PathExecutorNode(Node):
    """路径执行节点（V2 薄外壳组合根）。

    保留 V1 的全部外部接口（__init__ 签名、main 入口、话题名），
    内部逻辑委托给 V2 子模块：
      - RosBridge: 话题订阅/发布
      - Blackboard: 共享状态
      - core/geometry: 数学工具
      - route/parser: 路线解析
      - pursuit: 投影/前视/舵角
      - tasks: 定位/循迹/任务调度
    """

    def __init__(self, path_file=None):
        # ── 兼容 V1 的 Node 初始化 ────────────────────────────
        # 注意：如果在 ROS 环境直接继承 Node；否则使用桩
        if 'Node' in globals() and 'rclpy' in sys.modules:
            # 正式 ROS 环境用此初始化
            super().__init__('path_executor_node')
            try:
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
            except Exception:
                pass
            self._ros_mode = True
        else:
            # 测试环境使用桩
            self._ros_mode = False

        # ── 共享状态 Blackboard ───────────────────────────────
        self._bb = Blackboard()

        # ── 缓存（兼容 V1 直读字段）─────────────────────────
        # 以下字段与 V1 同名，供内部逻辑读取（部分逻辑仍直接访问 self.xxx）
        self.path_points = []        # V1 L212: waypoint list
        self.current_target_idx = 0  # V1 L213
        self.path_loaded = False     # V1 L214
        self.current_x = 0.0         # V1 L217
        self.current_y = 0.0         # V1 L218
        self.current_yaw = 0.0       # V1 L219
        self.has_odom = False        # V1 L220
        self.arrived = False         # V1 L221
        self.arrive_logged = False   # V1 L222

        # 转弯 PIVOT 状态（V1 L231-236）
        self.corner_mode = False
        self.corner_target_yaw = 0.0
        self.corner_turn_dir = 0
        self.corner_phase = 0
        self.corner_phase_t = 0.0
        self.corner_seg_idx = -1

        # 参数（V1 L239-254）
        self.lookahead_dist = 0.15
        self.linear_speed = 0.3
        self.max_angular_speed = 1.5
        self.wheelbase = 0.157
        self.arrival_threshold = 0.10 + (math.pi * 0.073 / 4.0)
        self.approach_dist = 0.6

        # 射击参数（V1 L260-266）
        self.shoot_flags = []
        self.shoot_wait = False
        self.shoot_wait_start = 0.0
        self.shoot_count_at_arrival = 0
        self.SHOOT_COUNT_TARGET = 10
        self.SHOOT_WAIT_TIMEOUT = 150.0
        self.SHOOT_STOP_RADIUS = 0.17

        # 扫描参数（V1 L273-281）
        self.aim_status = 0
        self.scan_flags = []
        self.scan_wait = False
        self.scan_wait_start = 0.0
        self.scan_seen_scanning = False
        self.SCAN_WAIT_TIMEOUT = 8.0
        self.SCAN_STOP_RADIUS = 0.20
        self.mission_scan_mode = 'continuous'

        # 锚定偏移（V1 L290-295）
        self.snap_x_off = 0.0
        self.snap_y_off = 0.0
        self.snap_yaw_off = 0.0
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.raw_map_yaw = 0.0

        # 缓存射弹数
        self.shoot_count = 0

        # 日志计数器（V1 L1360-1362）
        self._log_count = 0

        # ── 创建 RosBridge（在已有 node 上创建话题和定时器）───
        self._bridge = RosBridge(
            node=self if self._ros_mode else None,
            logger=self.get_logger() if self._ros_mode else None,
        )

        if self._ros_mode:
            # ── 注册 IO 回调（桥接解码 → 直写本节点字段）─────
            self._bridge.on_odom = self._on_odom_bridge
            self._bridge.on_waypoint = self._on_waypoint_bridge
            self._bridge.on_shoot_count = self._on_shoot_count_bridge
            self._bridge.on_aim_status = self._on_aim_status_bridge

            # 控制循环由 RosBridge 的定时器驱动（20Hz）
            self._bridge.on_control_tick = self.control_loop

            # 健康检查
            self._bridge.on_health_check = self._on_health_bridge

            # 同步黑盒到 V1 兼容字段（初始写入）
            self._sync_from_bb()

        # 注意：不创建额外线程，main() 中的 rclpy.spin(node)
        # 会处理 RosBridge 创建的订阅/定时器（在同一 node 上）

        # ── 加载路径文件（与 V1 L367-368 相同）─────────────
        if path_file:
            self.load_path(path_file)

        # ── 初始日志（与 V1 L370-387 相同）───────────────────
        if self._ros_mode:
            self._log_info("=" * 50)
            self._log_info("Path Executor Node Initialized")
            self._log_info("Commands:")
            self._log_info("  'M10,y' -> Go to M10, axis-aligned final approach")
            self._log_info("  'G8 A10' -> route: straight lines between")
            self._log_info("          waypoints, DIAGONALS allowed, the")
            self._log_info("          turn angle at each waypoint is")
            self._log_info("          computed automatically")
            self._log_info("  'G8 A10 C2,y' -> route, final stop facing Y+")
            self._log_info("  'G8 D2* E2' -> route, STOP and fire 10")
            self._log_info("          shots at D2, then continue")
            self._log_info("  'A10' -> direct line from current position")
            self._log_info("  'load:/path/to/file.txt' -> Load path file")
            self._log_info("  'reset:L9,x' -> re-anchor odometry after")
            self._log_info("          manually moving/rotating the car")
            self._log_info("=" * 50)
            self._log_info("Note: modeTimerCallback will auto-switch to Pursuit mode")
            self._log_info("when pursuitAngle is the only active topic")

    # ── 日志辅助（兼容 V1 和 RosBridge）────────────────────

    def _log_info(self, msg: str) -> None:
        if self._ros_mode:
            try:
                self.get_logger().info(msg)
            except Exception:
                pass

    def _log_warn(self, msg: str) -> None:
        if self._ros_mode:
            try:
                self.get_logger().warn(msg)
            except Exception:
                pass

    def _log_error(self, msg: str) -> None:
        if self._ros_mode:
            try:
                self.get_logger().error(msg)
            except Exception:
                pass

    # ── 状态同步 ─────────────────────────────────────────────

    def _sync_from_bb(self) -> None:
        """从黑盒同步到平铺字段（保持 V1 字段可读）。"""
        self.current_x = self._bb.get(P_POSE_X, self.current_x)
        self.current_y = self._bb.get(P_POSE_Y, self.current_y)
        self.current_yaw = self._bb.get(P_POSE_YAW, self.current_yaw)
        self.has_odom = self._bb.get(P_HAS_ODOM, self.has_odom)
        self.arrived = self._bb.get(P_ARRIVED, self.arrived)
        self.arrive_logged = self._bb.get(P_ARRIVE_LOGGED, self.arrive_logged)
        self.path_loaded = self._bb.get(P_PATH_LOADED, self.path_loaded)
        self.current_target_idx = self._bb.get(P_CURRENT_TARGET_IDX, self.current_target_idx)
        self.shoot_wait = self._bb.get(P_SHOOT_WAIT, self.shoot_wait)
        self.scan_wait = self._bb.get(P_SCAN_WAIT, self.scan_wait)
        self.corner_mode = self._bb.get(P_CORNER_MODE, self.corner_mode)
        self.corner_phase = self._bb.get(P_CORNER_PHASE, self.corner_phase)
        self.snap_x_off = self._bb.get(P_SNAP_X_OFF, self.snap_x_off)
        self.snap_y_off = self._bb.get(P_SNAP_Y_OFF, self.snap_y_off)
        self.snap_yaw_off = self._bb.get(P_SNAP_YAW_OFF, self.snap_yaw_off)

    def _sync_to_bb(self) -> None:
        """从平铺字段同步到黑盒（保持子模块可读）。"""
        self._bb.update('shell',
            **{
                P_POSE_X: self.current_x,
                P_POSE_Y: self.current_y,
                P_POSE_YAW: self.current_yaw,
                P_HAS_ODOM: self.has_odom,
                P_ARRIVED: self.arrived,
                P_ARRIVE_LOGGED: self.arrive_logged,
                P_PATH_LOADED: self.path_loaded,
                P_CURRENT_TARGET_IDX: self.current_target_idx,
                P_SHOOT_WAIT: self.shoot_wait,
                P_SCAN_WAIT: self.scan_wait,
                P_CORNER_MODE: self.corner_mode,
                P_CORNER_PHASE: self.corner_phase,
                P_SNAP_X_OFF: self.snap_x_off,
                P_SNAP_Y_OFF: self.snap_y_off,
                P_SNAP_YAW_OFF: self.snap_yaw_off,
                P_SHOOT_COUNT: self.shoot_count,
                P_AIM_STATUS: self.aim_status,
            })

    # ── RosBridge IO 回调（桥接 → 直写 V1 兼容字段）──────────

    def _on_odom_bridge(self, frame: OdomFrame) -> None:
        """里程计回调桥接（等价 V1 L829-851 odom_callback）。

        RosBridge 解码后传入 OdomFrame (raw_x/y 是块坐标，未乘 BLOCK_SIZE)。
        """
        # V1 L840-841: 块坐标 → 米
        self.raw_x = frame.raw_x * BLOCK_SIZE
        self.raw_y = frame.raw_y * BLOCK_SIZE

        # V1 L842-843: belief = raw + snap offset
        self.current_x = self.raw_x + self.snap_x_off
        self.current_y = self.raw_y + self.snap_y_off
        self.has_odom = True

        # V1 L846-851: yaw 解析 + 偏移
        q = (frame.qx, frame.qy, frame.qz, frame.qw)
        raw_yaw = euler_from_quaternion(*q)
        self.raw_map_yaw = normalize_angle(raw_yaw + RAW_YAW_TO_MAP_YAW_OFFSET)
        self.current_yaw = normalize_angle(self.raw_map_yaw + self.snap_yaw_off)

        # 同步到黑盒
        self._sync_to_bb()

    def _on_waypoint_bridge(self, msg: WaypointMsg) -> None:
        """路线命令回调桥接（等价 V1 L467-600 waypoint_callback）。"""
        # 直接调用 V1 兼容的 waypoint_callback（通过 Node 方法保留）
        # 构造一个桩消息对象
        class _Msg:
            def __init__(self, d):
                self.data = d
        self.waypoint_callback(_Msg(msg.data))

    def _on_shoot_count_bridge(self, msg: IntMsg) -> None:
        """射弹数回调桥接（等价 V1 L739-743 shoot_count_cb）。"""
        self.shoot_count = msg.data
        self._sync_to_bb()

    def _on_aim_status_bridge(self, msg: IntMsg) -> None:
        """云台状态回调桥接（等价 V1 L745-747 aim_status_cb）。"""
        self.aim_status = msg.data
        self._sync_to_bb()

    def _on_health_bridge(self, count: int) -> None:
        """健康检查回调桥接（等价 V1 L393-416 check_odom_publishers）。"""
        # 完全复制 V1 L397-416 的逻辑
        if count > 1:
            self._log_error(
                f"{count} publishers on /odom/localization! "
                "Kill the extra node (e.g. old C++ car_localization_node), "
                "otherwise the pose is corrupted"
            )
        elif count == 0:
            self._log_warn("No publisher on /odom/localization yet")
        else:
            self._log_info("Localization OK: 1 publisher on /odom/localization")

    # ── get_clock() 适配 ──────────────────────────────────

    def get_clock_now(self) -> float:
        """获取当前时钟秒数（兼容 V1 self.get_clock().now().seconds_nanoseconds()[0]）。"""
        if self._ros_mode:
            try:
                sec, nsec = super().get_clock().now().seconds_nanoseconds()
                return float(sec) + float(nsec) * 1e-9
            except Exception:
                pass
        return time_module.time()

    # ── count_publishers 桥接 ──────────────────────────────

    def count_publishers(self, topic: str) -> int:
        """桥接到 RosBridge（V1 兼容）。"""
        if self._ros_mode:
            try:
                return super().count_publishers(topic)
            except Exception:
                pass
        return 0

    # ── 发布方法（桥接到 RosBridge）────────────────────────────

    def _publish_cmd(self, linear_x: float, steer_deg: float) -> None:
        """发布控制指令（V1 L1353-1356）。"""
        if self._ros_mode:
            self._bridge.publish_command(linear_x, steer_deg)

    def _publish_stop_signal(self, allow: bool) -> None:
        """发布停站信号（V1 L820-822, L1207-1209）。"""
        if self._ros_mode:
            self._bridge.publish_stop_signal(allow)

    def _publish_scan_request(self, mode: int = 1) -> None:
        """发布扫描请求（V1 L760, L1213）。"""
        if self._ros_mode:
            self._bridge.publish_scan_request(mode)

    # ##### 以下是 V1 原样保留的方法 #####
    # 保持与 V1 逐字一致的逻辑，但访问 RosBridge 的发布/订阅

    def check_odom_publishers(self):
        """Check whether /odom/localization has multiple publishers.
        V1 L393-416 保留原样，但通过 RosBridge 的 on_health_check 触发。
        """
        # 由 on_health_bridge 处理
        pass

    def load_path(self, file_path):
        """Load path from file. V1 L421-461 完全一致。"""
        try:
            self.path_points = []
            with open(file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split(',')
                    if len(parts) >= 7:
                        point = {
                            'x': float(parts[0]),
                            'y': float(parts[1]),
                            'z': float(parts[2]),
                            'qx': float(parts[3]),
                            'qy': float(parts[4]),
                            'qz': float(parts[5]),
                            'qw': float(parts[6]),
                        }
                        self.path_points.append(point)
            if self.path_points:
                self.path_loaded = True
                self.current_target_idx = 0
                self.shoot_flags = [False] * len(self.path_points)
                self.scan_flags = [False] * len(self.path_points)
                self._start_mission_scan()
                self._log_info(
                    f"Loaded {len(self.path_points)} path points from {file_path}")
                self._sync_to_bb()
            else:
                self._log_warn(f"No valid points found in {file_path}")

        except FileNotFoundError:
            self._log_error(f"Path file not found: {file_path}")
        except Exception as e:
            self._log_error(f"Failed to load path: {e}")

    def waypoint_callback(self, msg):
        """Handle waypoint command. V1 L467-600 完全一致。"""
        try:
            data = msg.data.strip()

            # load: command (V1 L474-477)
            if data.startswith('load:'):
                file_path = data[5:].strip()
                self.load_path(file_path)
                return

            # reset: command (V1 L485-517)
            if data.startswith('reset:'):
                try:
                    rp = data[6:].split(',')
                    rblock = rp[0].strip().upper()
                    rhead = rp[1].strip().lower()
                    tx, ty = block_to_meters(rblock)
                    tqx, tqy, tqz, tqw = heading_to_quaternion(rhead)
                    tyaw = euler_from_quaternion(tqx, tqy, tqz, tqw)
                    self.snap_x_off = tx - self.raw_x
                    self.snap_y_off = ty - self.raw_y
                    self.snap_yaw_off = normalize_angle(tyaw - self.raw_map_yaw)
                    self.current_x = tx
                    self.current_y = ty
                    self.current_yaw = tyaw
                    self.path_points = []
                    self.shoot_flags = []
                    self.scan_flags = []
                    self.scan_wait = False
                    self.path_loaded = False
                    self.arrived = False
                    self.arrive_logged = False
                    self.shoot_wait = False
                    self.corner_mode = False
                    self.corner_phase = 0
                    self._sync_to_bb()
                    self._log_info(f"Odometry reset to {rblock} facing {rhead}")
                except Exception as e:
                    self._log_error(f"Failed to reset: {e}")
                return

            # Command dispatch (V1 L525-598)
            parts = data.split(',')
            if len(parts) != 2 or ' ' in parts[0]:
                self.follow_route_command(data)
                return

            # Single-target command: "M10,y" or "M7,x" (V1 L530-598)
            block_name = parts[0].strip().upper().rstrip('*')
            heading_dir = parts[1].strip().lower()

            if heading_dir not in ['x', 'y']:
                self._log_error(f"Invalid heading: '{heading_dir}'. Use 'x' or 'y'")
                return

            target_x, target_y = block_to_meters(block_name)
            qx, qy, qz, qw = heading_to_quaternion(heading_dir)

            if heading_dir == 'y':
                axis_x, axis_y = 0.0, 1.0
            else:
                axis_x, axis_y = 1.0, 0.0

            app_x = max(0.05, target_x - axis_x * self.approach_dist)
            app_y = max(0.05, target_y - axis_y * self.approach_dist)

            rel_x = self.current_x - app_x
            rel_y = self.current_y - app_y
            along = rel_x * axis_x + rel_y * axis_y
            lateral = abs(rel_x * axis_y - rel_y * axis_x)
            points = []
            if not (along >= -0.1 and lateral <= 0.15):
                points.append({
                    'x': app_x, 'y': app_y, 'z': 0.0,
                    'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw
                })
            points.append({
                'x': target_x, 'y': target_y, 'z': 0.0,
                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw
            })
            self.path_points = points
            self.current_target_idx = 0
            self.shoot_flags = [False] * len(points)
            self.scan_flags = [True] * len(points)
            if len(self.scan_flags) > 1:
                self.scan_flags[0] = False
            self._start_mission_scan()
            self.path_loaded = True
            self.arrived = False
            self.arrive_logged = False
            self.shoot_wait = False
            self.corner_mode = False
            self.corner_phase = 0
            self._sync_to_bb()
            self._log_info(
                f"Navigating to {block_name} ({target_x:.2f}m, {target_y:.2f}m), "
                f"facing {heading_dir}")
            self._log_info(
                f"Current: ({self.current_x:.2f}m, {self.current_y:.2f}m), "
                f"yaw={self.current_yaw:.2f}rad")

        except Exception as e:
            self._log_error(f"Failed to parse command: {e}")

    def follow_route_command(self, data):
        """Drive through a sequence of straight-line-connected waypoints.
        V1 L607-732 完全一致。"""
        final_heading = None
        body = data
        if ',' in data:
            body, tail = data.rsplit(',', 1)
            final_heading = tail.strip().lower()
            if final_heading not in ('x', 'y'):
                self._log_error(f"Invalid heading: '{tail}'. Use 'x' or 'y'")
                return
        tokens = body.split()
        if not tokens:
            self._log_error(f"Empty route command: '{data}'")
            return

        points = []
        shoot_flags = []
        scan_flags = []
        for tok in tokens:
            t = tok.strip()
            shoot = t.endswith('*')
            b = (t[:-1] if shoot else t).upper()
            if (len(b) < 2 or b[0] not in COL_LETTERS
                    or not b[1:].isdigit()):
                self._log_error(
                    f"Invalid waypoint '{tok}'. Use blocks like 'G8' or "
                    "'D2*' ('*' = mid-route shoot stop), e.g. 'G8 A10' "
                    "or 'G8 D2* E2 C2,y'")
                return
            px, py = block_to_meters(b.upper())
            if points and abs(points[-1]['x'] - px) < 1e-9 \
                    and abs(points[-1]['y'] - py) < 1e-9:
                if shoot:
                    shoot_flags[-1] = True
                continue
            points.append({'x': px, 'y': py, 'z': 0.0,
                           'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})
            shoot_flags.append(shoot)
            scan_flags.append(True)

        if not points:
            self._log_error(f"No valid waypoints in '{data}'")
            return

        if len(points) >= 2:
            last_yaw = math.atan2(points[-1]['y'] - points[-2]['y'],
                                  points[-1]['x'] - points[-2]['x'])
        else:
            last_yaw = self.current_yaw
        if final_heading == 'y':
            final_yaw = math.pi / 2
        elif final_heading == 'x':
            final_yaw = 0.0
        else:
            final_yaw = last_yaw

        if final_heading is not None \
                and abs(math.cos(last_yaw - final_yaw)) < 0.999:
            axis_x, axis_y = ((1.0, 0.0) if final_heading == 'x'
                              else (0.0, 1.0))
            tx, ty = points[-1]['x'], points[-1]['y']
            app_x = max(0.05, tx - axis_x * self.approach_dist)
            app_y = max(0.05, ty - axis_y * self.approach_dist)
            if abs(app_x - tx) > 1e-9 or abs(app_y - ty) > 1e-9:
                points.insert(len(points) - 1,
                              {'x': app_x, 'y': app_y, 'z': 0.0,
                               'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})
                shoot_flags.insert(len(shoot_flags) - 1, False)
                scan_flags.insert(len(scan_flags) - 1, False)

        for i, p in enumerate(points):
            if i + 1 < len(points):
                yw = math.atan2(points[i + 1]['y'] - p['y'],
                                points[i + 1]['x'] - p['x'])
            else:
                yw = final_yaw
            p['qz'] = math.sin(yw / 2.0)
            p['qw'] = math.cos(yw / 2.0)

        self.path_points = points
        self.current_target_idx = 0
        self.shoot_flags = shoot_flags
        self.scan_flags = scan_flags
        self._start_mission_scan()
        self.path_loaded = True
        self.arrived = False
        self.arrive_logged = False
        self.shoot_wait = False
        self.corner_mode = False
        self.corner_phase = 0
        self._sync_to_bb()
        self._log_info(
            f"Route ({len(points)} pts): {' '.join(tokens)} | "
            f"straight lines between waypoints, diagonals allowed")
        self._log_info(
            f"Current: ({self.current_x:.2f}m, {self.current_y:.2f}m), "
            f"yaw={self.current_yaw:.2f}rad")

    def shoot_count_cb(self, msg):
        """Track dzrknn_dataprocess cumulative shot count. V1 L739-743 一致。"""
        self.shoot_count = int(msg.data)
        self._sync_to_bb()

    def aim_status_cb(self, msg):
        """Cache the gimbal aim state machine status. V1 L745-747 一致。"""
        self.aim_status = int(msg.data)
        self._sync_to_bb()

    def enter_scan_stop(self, wp_idx):
        """停车并请求云台全向扫描。V1 L749-763 完全一致。"""
        self._publish_cmd(0.0, 0.0)          # 零 Twist = 刹车
        self.scan_wait = True
        self.scan_wait_start = self.get_clock_now()
        self.scan_seen_scanning = False
        self.scan_flags[wp_idx] = False
        self._publish_scan_request(1)        # 请求云台全向扫描
        self._sync_to_bb()
        self._log_info(
            f"点位 #{wp_idx} 停车，已请求云台全向扫描（最多等 {self.SCAN_WAIT_TIMEOUT:.0f}s）")

    def _start_mission_scan(self):
        """任务加载后，按 mission_scan_mode 决定云台的扫描方式。V1 L765-782 一致。"""
        if self.mission_scan_mode == 'continuous':
            self.scan_flags = [False] * len(self.path_points)
            self._publish_scan_request(2)
            self._log_info("任务扫描模式：持续横扫（边走边扫）")
        elif self.mission_scan_mode == 'waypoint':
            self._publish_scan_request(0)
            self._log_info("任务扫描模式：点位停站扫描")
        else:
            self.scan_flags = [False] * len(self.path_points)
            self._publish_scan_request(0)
            self._log_info("任务扫描模式：不自动扫描")
        self._sync_to_bb()

    def enter_shoot_stop(self, wp_idx):
        """Stop at a mid-route shoot waypoint. V1 L784-827 完全一致。"""
        cmd_brake = (0.0, 0.0)
        self._publish_cmd(*cmd_brake)

        b = self.path_points[wp_idx]
        a = self.path_points[max(0, wp_idx - 1)]
        leg_yaw = math.atan2(b['y'] - a['y'], b['x'] - a['x'])
        drift_x = self.current_x - b['x']
        drift_y = self.current_y - b['y']
        self.snap_x_off -= drift_x
        self.snap_y_off -= drift_y
        self.current_x = b['x']
        self.current_y = b['y']
        yaw_drift_deg = math.degrees(
            normalize_angle(self.current_yaw - leg_yaw))
        self.snap_yaw_off = normalize_angle(
            self.snap_yaw_off - math.radians(yaw_drift_deg))
        self.current_yaw = leg_yaw

        self.shoot_flags[wp_idx] = False
        self.shoot_wait = True
        self.shoot_wait_start = self.get_clock_now()
        self.shoot_count_at_arrival = self.shoot_count
        self._publish_stop_signal(True)
        self._sync_to_bb()
        self._log_info(
            f"Shoot stop at wp#{wp_idx}: waiting for "
            f"{self.SHOOT_COUNT_TARGET} shots (start count "
            f"{self.shoot_count_at_arrival})")

    # ##### 以下方法由 tasks/ 子模块提供（组合注入）##############

    def find_nearest_segment(self):
        """寻找最近段 + 投影点（委托给 pursuit/path.py V1 兼容函数）。

        与 V1 L855-891 逐位一致。
        """
        return find_nearest_segment_v1(self.path_points,
                                       self.current_target_idx,
                                       self.current_x, self.current_y)

    def find_lookahead_point(self, seg):
        """从前视距离找到目标瞄准点（委托给 pursuit/lookahead.py V1 兼容函数）。

        与 V1 L895-970 逐位一致。
        """
        return find_lookahead_point_v1_dict(self.path_points, seg,
                                            self.lookahead_dist)

    def pure_pursuit_control(self, target_point):
        """纯追踪核心：舵角 + 线速度（委托给 pursuit/controller.py V1 兼容函数）。

        与 V1 L974-1026 逐位一致。
        """
        return pure_pursuit_control_v1(
            target_point,
            self.current_x, self.current_y, self.current_yaw,
            self.wheelbase, self.linear_speed,
        )

    # ##### control_loop — V1 原样保留的主控制循环 ##############

    def control_loop(self):
        """Main control loop (20 Hz).
        V1 L1036-1372 完全一致（所有逻辑逐字节保留）。
        """
        if not self.has_odom:
            return

        cmd_zero = (0.0, 0.0)

        # Branch 1: shoot wait (V1 L1047-1072)
        if self.shoot_wait:
            self._publish_cmd(*cmd_zero)
            now_t = self.get_clock_now()
            done = (self.shoot_count - self.shoot_count_at_arrival
                    >= self.SHOOT_COUNT_TARGET)
            timed_out = (now_t - self.shoot_wait_start
                         > self.SHOOT_WAIT_TIMEOUT)
            if done or timed_out:
                self._publish_stop_signal(False)
                self.shoot_wait = False
                if timed_out and not done:
                    self._log_warn("Shoot wait timed out, continuing route")
                else:
                    self._log_info(
                        f"{self.SHOOT_COUNT_TARGET} shots fired, resuming route")
                self._sync_to_bb()
            return

        # Branch 2: scan wait (V1 L1074-1096)
        if self.scan_wait:
            self._publish_cmd(*cmd_zero)
            now_t = self.get_clock_now()
            elapsed = now_t - self.scan_wait_start
            if self.aim_status == 1:
                self.scan_seen_scanning = True

            if self.aim_status == 2:
                self.scan_wait = False
                self._log_info("云台已锁定目标，恢复循迹（边走边跟随）")
            elif self.scan_seen_scanning and self.aim_status == 0:
                self.scan_wait = False
                self._log_info("云台扫描完成、未发现目标，继续前往下一个点位")
            elif elapsed > self.SCAN_WAIT_TIMEOUT:
                self.scan_wait = False
                self._log_warn(
                    f"扫描等待超时({self.SCAN_WAIT_TIMEOUT:.0f}s)，强制继续循迹")
            self._sync_to_bb()
            return

        # Branch 3: no path (V1 L1101-1105)
        if not self.path_loaded or not self.path_points:
            self._publish_cmd(*cmd_zero)
            return

        # Branch 4: nearest segment + lookahead (V1 L1107-1118)
        seg = self.find_nearest_segment()
        if seg is None:
            target_point = self.path_points[-1] if self.path_points else None
        else:
            self.current_target_idx = seg['idx']
            target_point = self.find_lookahead_point(seg)

        if target_point is None:
            self._publish_cmd(*cmd_zero)
            return

        # Branch 5: scan stop detection (V1 L1120-1139)
        for wp_idx in range(len(self.path_points)):
            if (wp_idx >= len(self.scan_flags)
                    or not self.scan_flags[wp_idx]):
                continue
            wp = self.path_points[wp_idx]
            d = math.hypot(wp['x'] - self.current_x,
                           wp['y'] - self.current_y)
            if d <= self.SCAN_STOP_RADIUS:
                if self.aim_status == 2:
                    self.scan_flags[wp_idx] = False
                    self._log_info(
                        f"点位 #{wp_idx} 云台已锁定目标，不停车继续")
                else:
                    self.enter_scan_stop(wp_idx)
                    return

        # Branch 6: shoot stop detection (V1 L1141-1153)
        for wp_idx in range(len(self.path_points)):
            if (wp_idx >= len(self.shoot_flags)
                    or not self.shoot_flags[wp_idx]):
                continue
            wp = self.path_points[wp_idx]
            d = math.hypot(wp['x'] - self.current_x,
                           wp['y'] - self.current_y)
            if d <= self.SHOOT_STOP_RADIUS:
                self.enter_shoot_stop(wp_idx)
                return

        # Branch 7: end arrival (V1 L1155-1215)
        final_point = self.path_points[-1]
        dx = final_point['x'] - self.current_x
        dy = final_point['y'] - self.current_y
        dist_to_final = math.sqrt(dx * dx + dy * dy)

        if dist_to_final < self.arrival_threshold or self.arrived:
            self.arrived = True
            self._publish_cmd(*cmd_zero)

            if not self.arrive_logged:
                self.arrive_logged = True
                target_yaw = euler_from_quaternion(
                    final_point['qx'], final_point['qy'],
                    final_point['qz'], final_point['qw'])
                drift_x = self.current_x - final_point['x']
                drift_y = self.current_y - final_point['y']
                yaw_drift_deg = math.degrees(
                    normalize_angle(self.current_yaw - target_yaw))

                self.snap_x_off -= drift_x
                self.snap_y_off -= drift_y
                self.snap_yaw_off = normalize_angle(
                    self.snap_yaw_off - math.radians(yaw_drift_deg))
                self.current_x = final_point['x']
                self.current_y = final_point['y']
                self.current_yaw = target_yaw
                self._sync_to_bb()

                self._log_info(
                    f"ARRIVED! dist={dist_to_final:.3f}m, "
                    f"drift=({drift_x:+.2f}m, {drift_y:+.2f}m), "
                    f"yaw_drift={yaw_drift_deg:+.1f}deg, stopped")
                self._log_info(
                    "Odometry re-anchored to block center + commanded heading")
                self._publish_stop_signal(True)
                self._log_info("Published stop_signal=1 (enable gimbal+shooting)")
                self._publish_scan_request(1)
                self._log_info("已请求云台在终点执行全向扫描")
            return

        # Branch 8: PIVOT sharp corner in-place turn (V1 L1217-1314)
        SHARP_CORNER_RAD = math.radians(45.0)
        CORNER_TRIGGER_M = 0.25
        CORNER_ALIGN_RAD = math.radians(15.0)
        CORNER_SLOW_M = 0.45

        # PIVOT entry check (V1 L1229-1257)
        if seg is not None and not self.corner_mode \
                and seg['idx'] + 2 < len(self.path_points):
            vertex = self.path_points[seg['idx'] + 1]
            dvx = vertex['x'] - self.current_x
            dvy = vertex['y'] - self.current_y
            dist_to_vertex = math.hypot(dvx, dvy)
            p1 = self.path_points[seg['idx']]
            p2 = self.path_points[seg['idx'] + 1]
            p3 = self.path_points[seg['idx'] + 2]
            d1x = p2['x'] - p1['x']
            d1y = p2['y'] - p1['y']
            d2x = p3['x'] - p2['x']
            d2y = p3['y'] - p2['y']
            cross = d1x * d2y - d1y * d2x
            dot = d1x * d2x + d1y * d2y
            corner_ang = abs(math.atan2(cross, dot))
            if corner_ang > SHARP_CORNER_RAD and dist_to_vertex < CORNER_TRIGGER_M:
                self.corner_mode = True
                self.corner_phase = 0
                self.corner_phase_t = self.get_clock_now()
                self.corner_seg_idx = seg['idx'] + 1
                self.corner_target_yaw = math.atan2(d2y, d2x)
                self.corner_turn_dir = 1 if cross > 0 else -1
                self._sync_to_bb()
                self._log_info(
                    f"[PIVOT] sharp corner at vertex #{seg['idx']+1}, "
                    f"target_yaw={math.degrees(self.corner_target_yaw):.0f}deg "
                    f"dir={'L' if self.corner_turn_dir>0 else 'R'}")

        # PIVOT state machine (V1 L1260-1314)
        if self.corner_mode:
            now_t = self.get_clock_now()
            yaw_err = normalize_angle(self.corner_target_yaw - self.current_yaw)

            if self.corner_phase == 0:  # Phase 0: stop + steer to max
                self._publish_cmd(0.0, 30.0 * self.corner_turn_dir)
                self.corner_phase = 1
                self.corner_phase_t = now_t
                self._log_info("[PIVOT] phase 1: stop + max steer")
                self._sync_to_bb()
                return

            elif self.corner_phase == 1:  # Phase 1: hold steer 0.5s
                if now_t - self.corner_phase_t < 0.5:
                    self._publish_cmd(0.0, 30.0 * self.corner_turn_dir)
                    return
                self.corner_phase = 2
                self.corner_phase_t = now_t
                self._log_info("[PIVOT] phase 2: creep forward")

            # Phase 2: creep until aligned
            if abs(yaw_err) < CORNER_ALIGN_RAD:
                self.corner_mode = False
                self.current_target_idx = self.corner_seg_idx
                self._sync_to_bb()
                self._log_info(
                    f"[PIVOT] aligned (yaw_err={math.degrees(yaw_err):.1f}deg), "
                    f"resume pure pursuit on seg {self.corner_seg_idx}")
            else:
                self._publish_cmd(0.08, 30.0 * self.corner_turn_dir)
                self.arrive_logged = False
                self._log_count += 1
                if self._log_count % 20 == 0:
                    self._log_info(
                        f"[PIVOT] creeping yaw_err={math.degrees(yaw_err):.1f}deg "
                        f"yaw={math.degrees(self.current_yaw):.1f}deg "
                        f"target={math.degrees(self.corner_target_yaw):.1f}deg")
                return

        # Branch 9: normal pure pursuit (V1 L1316-1372)
        linear_vel, steer_deg = self.pure_pursuit_control(target_point)

        # End slowdown (V1 L1323-1324)
        if dist_to_final < 0.5:
            linear_vel = min(linear_vel, max(0.1, 0.35 * dist_to_final))

        # Corner slowdown (V1 L1330-1348)
        if seg is not None and not self.corner_mode \
                and seg['idx'] + 2 < len(self.path_points):
            vertex = self.path_points[seg['idx'] + 1]
            dvx = vertex['x'] - self.current_x
            dvy = vertex['y'] - self.current_y
            dist_to_vertex = math.hypot(dvx, dvy)
            p1 = self.path_points[seg['idx']]
            p2 = self.path_points[seg['idx'] + 1]
            p3 = self.path_points[seg['idx'] + 2]
            d1x = p2['x'] - p1['x']
            d1y = p2['y'] - p1['y']
            d2x = p3['x'] - p2['x']
            d2y = p3['y'] - p2['y']
            cross = d1x * d2y - d1y * d2x
            dot = d1x * d2x + d1y * d2y
            corner_ang = abs(math.atan2(cross, dot))
            if corner_ang > SHARP_CORNER_RAD \
                    and dist_to_vertex < CORNER_SLOW_M:
                linear_vel = min(linear_vel, 0.12)

        # Final output (V1 L1353-1372)
        self._publish_cmd(linear_vel, steer_deg)
        self.arrive_logged = False

        self._log_count += 1
        if self._log_count % 40 == 0:
            seg_disp = str(seg['idx']) if seg else '-'
            self._log_info(
                f"pos=({self.current_x:.2f}m, {self.current_y:.2f}m) "
                f"seg={seg_disp}/{len(self.path_points)} "
                f"dist={dist_to_final:.2f}m "
                f"v={linear_vel:.2f} steer={steer_deg:.1f}deg "
                f"yaw={self.current_yaw:.2f}rad"
            )


# ##### 程序入口 #####
# 完全保持 V1 L1379-1402 的语义

def main(args=None):
    """程序入口（与 V1 L1379-1398 相同）。"""
    path_file = None

    if len(sys.argv) > 1:
        path_file = sys.argv[1]

    rclpy.init(args=args)
    node = PathExecutorNode(path_file)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
        # Publish stop command via bridge (V1 used node.cmd_pub.publish(Twist()))
        try:
            node._bridge.publish_brake()
        except Exception:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
