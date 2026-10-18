# -*- coding: utf-8 -*-
"""
Path executor node - follows waypoint routes on the block grid.

Accepts route commands on /nav/waypoint:
  'M10,y'        -> go to M10 and stop facing Y+
  'M10 L9 K7,y'  -> route through M10, L9, then K7 (final facing Y+)
  'reset:N10,y'  -> re-anchor odometry after manually moving the car
  'load:/path'   -> load a path file (x,y,z,qx,qy,qz,qw per line)

Publishes Twist on 'pursuitAngle' where angular.z is a SERVO ANGLE in
degrees (0 = straight, valid range [-35, +35]), matching dzactuator.

Control architecture (grid navigator):
  - Straight legs: pure pursuit toward a point ahead ON THE CURRENT LEG
    (heading hold + cross-track correction) at STRAIGHT_SPEED, with a
    linear ramp down to TURN_SPEED before every corner.
  - Corners: calibrated OPEN-LOOP arc. Full servo lock + TURN_SPEED for
    a distance S = R * angle. Wheel odometry YAW is unreliable while
    turning, so the arc length - measured by odometry DISTANCE, which is
    accurate on this chassis - decides when the turn is done.
  - After every corner: position snapped to the ideal arc exit point and
    yaw snapped to the next leg direction. Drift can therefore never
    accumulate across more than one leg.
  - The old final-approach pre-point insertion is REMOVED: with 0.35 m
    legs it created backtrack segments that forced real 180 deg turns.
"""

import math
import re
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8, Int32

try:                                    # 脚本方式运行(start_path_executor.sh)：同目录模块
    from cte_pid import CtePidController
except ImportError:                     # 作为 ROS 包模块导入
    from car_localization.cte_pid import CtePidController

# QoS matching dzactuator subscriptions
CMD_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# Map constants
COL_LETTERS = 'ABCDEFGHIJKLMN'
MAP_COLS = 14
MAP_ROWS = 11
BLOCK_SIZE = 0.35  # meters per block

# serial_reader publishes /odom/localization in BLOCK units and passes
# through the raw odometry orientation. Raw odom frame: forward = X+.
# Map frame: forward at start = Y+. Offset applied to raw yaw.
RAW_YAW_TO_MAP_YAW_OFFSET = math.pi / 2

# Bench test 'rotate car LEFT 90deg' decides this flag:
#   yaw belief goes 90 -> 180deg : keep False (correct)
#   yaw belief goes 90 -> 0deg   : set True (IMU yaw is CW-positive)
FLIP_RAW_YAW_SIGN = False

# Car constants (match dztrack steering law)
WHEELBASE = 0.17       # meters
MAX_STEER_DEG = 30.0   # servo limit used by dztrack

HEADING_VECTORS = {
    'x': (1.0, 0.0),
    '-x': (-1.0, 0.0),
    'y': (0.0, 1.0),
    '-y': (0.0, -1.0),
}

HEADING_YAWS = {
    'x': 0.0,
    '-x': math.pi,
    'y': math.pi / 2,
    '-y': -math.pi / 2,
}


def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to yaw angle (radians)."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def block_to_meters(block_name):
    """Convert block name to meters coordinates."""
    letter = block_name[0].upper()
    number = int(block_name[1:])

    col_idx = COL_LETTERS.index(letter)
    row_idx = number - 1

    x = (10.5 - row_idx) * BLOCK_SIZE
    y = (13.5 - col_idx) * BLOCK_SIZE

    return x, y


def heading_yaw(heading_dir):
    """Return map yaw (radians) for a heading token 'x'/'y'/'-x'/'-y'."""
    return HEADING_YAWS.get(heading_dir, 0.0)


def make_point(x, y, heading_dir=None):
    """Build a path point dict; optional heading sets the final quaternion."""
    yaw = heading_yaw(heading_dir)
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return {'x': x, 'y': y, 'z': 0.0,
            'qx': 0.0, 'qy': 0.0, 'qz': qz, 'qw': qw}


class PathExecutorNode(Node):
    def __init__(self, path_file=None):
        super().__init__('path_executor_node')

        # Path data. path_points[0] is the car position at route start,
        # so every segment is well defined for the projection.
        self.path_points = []
        self.route_labels = []
        self.final_heading = None
        self.seg_idx = 0
        self.path_loaded = False

        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False
        self.arrived = False
        self.arrive_logged = False
        self.last_steer = 0.0
        self._log_count = 0

        # CTE 转向修正控制器（吸收 Line_Follower 自适应 PID 的 6 重抗饱和机制）
        self.cte_ctrl = CtePidController()
        self._cte_seg = -1

        # --- Mid-route shoot stops ------------------------------------------
        # Waypoints marked with '*' in the route command (e.g. 'D2*') make
        # the car stop there, enable gimbal scan + shooting via stop_signal,
        # wait for SHOOT_COUNT_TARGET shots on /shoot_count, then continue.
        self.route_shoot = []
        self.shoot_wait_start = None
        self.shoot_count_at_arrival = 0
        self.SHOOT_COUNT_TARGET = 10     # shots to fire before resuming
        self.SHOOT_WAIT_TIMEOUT = 150.0  # s; give up waiting and continue
        self.SHOOT_STOP_RADIUS = 0.17    # m; stop distance from the waypoint

        # --- Speed schedule ------------------------------------------------
        # Remote-control baseline: straight odometry is accurate (<1 cm per
        # block) but yaw drifts badly while turning. So: fast straights,
        # slow short arcs, and never trust yaw across a corner.
        self.STRAIGHT_SPEED = 0.32   # m/s on straight legs
        self.TURN_SPEED = 0.14       # m/s inside corner arcs
        self.R_TURN = WHEELBASE / math.tan(math.radians(MAX_STEER_DEG))
        # TURN_ARC_SCALE calibrates the arc: physical rotation = S / R_phys.
        # If corners systematically over-rotate, lower it (e.g. 0.85);
        # if they under-rotate, raise it.
        # 1.0 -> 110deg overshoot. 0.80 gives ~90deg right, ~111deg left.
        # 2026-09-04 clean run (X-mirror fixed): right arc rotated only
        # ~50deg over 0.38m (R_phys~0.44m) while left arc rotated ~140deg
        # over 0.37m (R_phys~0.15m) at the SAME full-lock command. The
        # servo travel is strongly asymmetric, so a DISTANCE-based arc can
        # never give correct angles on both sides. Arc stop is therefore
        # YAW-based now; TURN_ARC_SCALE is only the travel SAFETY upper
        # bound multiplier (nominal arc * 1.6).
        self.TURN_ARC_SCALE = 1.6
        # Stop the arc this many degrees before the target angle; the
        # remaining few degrees are finished by pure pursuit on the next
        # leg (closed loop), and servo return-lag adds a bit more.
        self.TURN_YAW_LEAD_DEG = 6.0
        self.AIM_DRIVE = 0.35        # aim point distance (shorter = more aggressive correction)
        # STEER_TRIM: small constant RIGHT bias (-1 deg) to cancel the
        # constant mechanical left pull. Safe now: the feedback signs are
        # verified correct, so the cte loop just absorbs the tiny shift.
        # (The old diagonal-cutting failure was the mirrored-odom bug,
        # not the trim itself.) Adjust: more left drift -> -1.5; right
        # drift -> -0.5.
        self.STEER_TRIM = -1.0       # degrees (negative = right)

        # --- Tracking / geometry -------------------------------------------
        self.arrival_threshold = 0.20  # meters
        self.final_ramp_dist = 0.50    # start braking this far before target

        # --- Watchdogs / safety --------------------------------------------
        self.close_radius = 0.40  # orbiting watchdog radius (drive mode only)
        self.close_grace = 5.0    # seconds before forced stop
        self.close_since = None
        self.was_close = False

        self.realign_limit = 2
        self.realign_count = 0

        self.nav_timeout = 120.0   # seconds
        self.nav_start = None

        # --- Mode machine ----------------------------------------------------
        # 'idle'  : no active route, publish zero command
        # 'drive' : follow current leg straight
        # 'turn'  : open-loop arc through the corner ahead
        self.mode = 'idle'
        self.turn_sign = 0.0
        self.turn_arc_len = 0.0
        self.turn_start_travel = 0.0
        self.turn_start_time = 0.0
        self.turn_max_time = 0.0
        self.turn_start_yaw = 0.0
        self.turn_target_ang = 0.0
        self.turn_exit_x = 0.0
        self.turn_exit_y = 0.0
        self.turn_exit_yaw = 0.0

        # --- Raw odometry distance (immune to position snaps) ---------------
        self.raw_travel = 0.0
        self.prev_raw_x = None
        self.prev_raw_y = None

        # --- Odometry snap offsets -------------------------------------------
        self.snap_x_off = 0.0
        self.snap_y_off = 0.0
        self.snap_yaw_off = 0.0
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.raw_map_yaw = 0.0

        # Subscribe to localization (BEST_EFFORT accepts BOTH reliable and
        # best_effort publishers - a QoS mismatch here used to silently
        # deliver zero messages, leaving the car standing).
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom/localization',
            self.odom_callback,
            CMD_QOS
        )

        # Subscribe to waypoint commands (for triggering)
        self.waypoint_sub = self.create_subscription(
            String,
            '/nav/waypoint',
            self.waypoint_callback,
            10
        )

        # Publish control commands
        self.cmd_pub = self.create_publisher(
            Twist,
            'pursuitAngle',
            CMD_QOS
        )

        # Publish arrival signal to dzrknn_dataprocess (enables gimbal+shooting)
        self.stop_signal_pub = self.create_publisher(
            UInt8,
            '/move_base/stop_signal',
            10
        )

        # Shot counter from dzrknn_dataprocess (published once per shot)
        self.shoot_count_sub = self.create_subscription(
            Int32, '/shoot_count', self.shoot_count_cb, 10)
        self.shoot_count = 0

        # Create control timer (20Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # One-shot check: /odom/localization must have exactly ONE publisher.
        self.create_timer(3.0, self.check_odom_publishers)

        # Loud warning if localization data never actually arrives
        # (publisher count alone does not prove messages flow).
        self.create_timer(5.0, self.warn_no_odom)

        # Load path file
        if path_file:
            self.load_path(path_file)

        self.get_logger().info("=" * 50)
        self.get_logger().info("Path Executor Node Initialized")
        self.get_logger().info("Commands:")
        self.get_logger().info("  'M10,y' -> go to M10, stop facing Y+")
        self.get_logger().info("  'M10 L9 K7,y' -> route M10 -> L9 -> K7,")
        self.get_logger().info("          final stop facing Y+")
        self.get_logger().info("  'load:/path/to/file.txt' -> Load path file")
        self.get_logger().info("  'reset:N10,y' -> re-anchor odometry after")
        self.get_logger().info("          manually moving/rotating the car")
        self.get_logger().info("-" * 50)
        self.get_logger().info(
            f"Straight={self.STRAIGHT_SPEED:.2f}m/s, "
            f"turn arc R={self.R_TURN:.2f}m at "
            f"{self.TURN_SPEED:.2f}m/s, scale={self.TURN_ARC_SCALE:.2f}")
        self.get_logger().info("=" * 50)
        self.get_logger().info("Note: modeTimerCallback will auto-switch to Pursuit mode")
        self.get_logger().info("when pursuitAngle is the only active topic")

    def warn_no_odom(self):
        """Periodically shout if no /odom/localization data arrives."""
        if not self.has_odom:
            self.get_logger().error(
                "NO /odom/localization data received! The car will NOT "
                "move. Check: is serial_reader_node running and printing "
                "'pos:' lines? Rebuild with: colcon build --packages-select "
                "car_localization"
            )

    def check_odom_publishers(self):
        """Warn if more than one node publishes /odom/localization."""
        try:
            count = self.count_publishers('/odom/localization')
            if count > 1:
                self.get_logger().error(
                    f"{count} publishers on /odom/localization! "
                    "Kill the extra node, otherwise the pose is corrupted"
                )
            elif count == 0:
                self.get_logger().warning(
                    "No publisher on /odom/localization yet"
                )
            else:
                self.get_logger().info(
                    "Localization OK: 1 publisher on /odom/localization"
                )
        except Exception:
            pass

    def load_path(self, file_path):
        """Load path from file."""
        try:
            file_points = []
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
                            'qw': float(parts[6])
                        }
                        file_points.append(point)

            if file_points:
                self.path_points = ([make_point(self.current_x, self.current_y)]
                                    + file_points)
                self.route_labels = (['START']
                                     + [f"P{i}" for i in range(len(file_points))])
                self.final_heading = None
                self.seg_idx = 0
                self.path_loaded = True
                self.arrived = False
                self.arrive_logged = False
                self.mode = 'drive'
                self.close_since = None
                self.was_close = False
                self.realign_count = 0
                self.nav_start = self.get_clock().now().nanoseconds * 1e-9
                self.get_logger().info(
                    f"Loaded {len(file_points)} path points from {file_path}"
                )
            else:
                self.get_logger().warning(f"No valid points found in {file_path}")

        except FileNotFoundError:
            self.get_logger().error(f"Path file not found: {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load path: {e}")

    def waypoint_callback(self, msg):
        """Handle waypoint commands."""
        try:
            data = msg.data.strip()

            if data.startswith('load:'):
                file_path = data[5:].strip()
                self.load_path(file_path)
                return

            # Manual odometry re-anchor: 'reset:BLOCK,HEADING'
            if data.startswith('reset:'):
                try:
                    rp = data[6:].split(',')
                    rblock = rp[0].strip().upper()
                    rhead = rp[1].strip().lower()
                    if rhead not in HEADING_VECTORS:
                        self.get_logger().error(
                            f"Invalid heading: '{rhead}'. Use x/y/-x/-y")
                        return
                    tx, ty = block_to_meters(rblock)
                    tyaw = heading_yaw(rhead)
                    self.snap_x_off = tx - self.raw_x
                    self.snap_y_off = ty - self.raw_y
                    self.snap_yaw_off = normalize_angle(tyaw - self.raw_map_yaw)
                    self.current_x = tx
                    self.current_y = ty
                    self.current_yaw = tyaw
                    self.path_points = []
                    self.path_loaded = False
                    self.arrived = False
                    self.mode = 'idle'
                    self.get_logger().info(
                        f"Odometry reset to {rblock} facing {rhead}"
                    )
                except Exception as e:
                    self.get_logger().error(f"Failed to reset: {e}")
                return

            # Route command: blocks plus optional final heading.
            # A block may carry a '*' suffix marking a MID-ROUTE SHOOT STOP
            # (e.g. 'L10 L7 C7 C2 D2* E2 E5 N5' -> stop & fire 10x at D2).
            tokens = [t for t in re.split(r'[\s,]+', data) if t]
            blocks = []
            shoot_flags = []
            heading = None
            for tok in tokens:
                t = tok.lower()
                if t in HEADING_VECTORS:
                    heading = t
                elif re.fullmatch(r'[a-n][0-9]{1,2}\*?', t):
                    blocks.append(tok.upper().rstrip('*'))
                    shoot_flags.append(t.endswith('*'))
                else:
                    self.get_logger().error(f"Invalid token: '{tok}'")
                    return

            if not blocks:
                self.get_logger().error(
                    f"Invalid format: '{data}'. "
                    "Expected 'BLOCK[,HEADING]' or 'B1 B2* B3[,HEADING]' "
                    "('*' marks a mid-route shoot stop)")
                return

            # Build waypoint list (dedupe consecutive duplicates).
            # NOTE: no final-approach pre-point insertion. On a 0.35 m grid
            # the inserted point used to land BEHIND the previous waypoint,
            # creating a backtrack segment and a real 180 deg turn.
            pts = []
            labels = []
            shoot_marks = []
            for b, s in zip(blocks, shoot_flags):
                x, y = block_to_meters(b)
                if pts and abs(pts[-1][0] - x) < 1e-6 and abs(pts[-1][1] - y) < 1e-6:
                    if s:
                        shoot_marks[-1] = True
                    continue
                # Skip a LEADING waypoint equal to the current position:
                # path_points[0] is already the car position, and a
                # zero-length first leg (leg_len < 1e-6) stalls the
                # control loop silently - the car never moves.
                if not pts and math.hypot(self.current_x - x,
                                          self.current_y - y) < 0.02:
                    self.get_logger().info(
                        f"Skipping start waypoint {b} "
                        "(car is already there)")
                    continue
                pts.append((x, y))
                labels.append(b)
                shoot_marks.append(s)

            self.path_points = ([make_point(self.current_x, self.current_y)]
                                + [make_point(x, y) for x, y in pts])
            if heading:
                fx, fy = pts[-1]
                self.path_points[-1] = make_point(fx, fy, heading)
            self.route_labels = ['START'] + labels
            self.route_shoot = [False] + shoot_marks
            self.final_heading = heading
            self.seg_idx = 0
            self.path_loaded = True
            self.arrived = False
            self.arrive_logged = False
            self.mode = 'drive'
            self.close_since = None
            self.was_close = False
            self.realign_count = 0
            self.nav_start = self.get_clock().now().nanoseconds * 1e-9

            route_str = ' -> '.join(
                f"{b}*" if s else b for b, s in zip(labels, shoot_marks))
            suffix = f", final facing {heading}" if heading else ""
            self.get_logger().info(f"Route: {route_str}{suffix}")
            self.get_logger().info(
                f"Current: ({self.current_x:.2f}m, {self.current_y:.2f}m), "
                f"yaw={self.current_yaw:.2f}rad"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to parse command: {e}")

    def shoot_count_cb(self, msg):
        """Track cumulative shots fired by dzrknn_dataprocess."""
        self.shoot_count = int(msg.data)

    def odom_callback(self, msg):
        """Update current position and accumulate raw travel distance."""
        self.raw_x = msg.pose.pose.position.x * BLOCK_SIZE
        self.raw_y = msg.pose.pose.position.y * BLOCK_SIZE

        if self.prev_raw_x is not None:
            self.raw_travel += math.hypot(self.raw_x - self.prev_raw_x,
                                          self.raw_y - self.prev_raw_y)
        self.prev_raw_x = self.raw_x
        self.prev_raw_y = self.raw_y

        self.current_x = self.raw_x + self.snap_x_off
        self.current_y = self.raw_y + self.snap_y_off
        self.has_odom = True

        q = msg.pose.pose.orientation
        raw_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        if FLIP_RAW_YAW_SIGN:
            raw_yaw = -raw_yaw
        self.raw_map_yaw = normalize_angle(raw_yaw + RAW_YAW_TO_MAP_YAW_OFFSET)
        self.current_yaw = normalize_angle(self.raw_map_yaw + self.snap_yaw_off)

    def _project_on_segment(self, ax, ay, bx, by):
        """Projection of the car on segment AB. Returns (t clamped [0,1], dist)."""
        abx, aby = bx - ax, by - ay
        seg_len2 = abx * abx + aby * aby
        if seg_len2 < 1e-9:
            return 0.0, math.hypot(self.current_x - ax, self.current_y - ay)
        t = ((self.current_x - ax) * abx
             + (self.current_y - ay) * aby) / seg_len2
        t = max(0.0, min(1.0, t))
        px = ax + t * abx
        py = ay + t * aby
        return t, math.hypot(self.current_x - px, self.current_y - py)

    def corner_radius(self, i):
        """Arc radius for the corner at path_points[i+1]."""
        a = self.path_points[i]
        b = self.path_points[i + 1]
        c = self.path_points[i + 2]
        l1 = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        l2 = math.hypot(c['x'] - b['x'], c['y'] - b['y'])
        return max(0.10, min(self.R_TURN, 0.9 * l1, 0.9 * l2))

    def start_turn(self, r_corner):
        """Enter 'turn' mode for the corner at path_points[seg_idx + 1]."""
        a = self.path_points[self.seg_idx]
        b = self.path_points[self.seg_idx + 1]
        c = self.path_points[self.seg_idx + 2]

        d1x, d1y = b['x'] - a['x'], b['y'] - a['y']
        d2x, d2y = c['x'] - b['x'], c['y'] - b['y']
        # 2026-10-18 修复（G1 对拍发现）：180° 掉头方向显式定义。原式 cross
        # 在对向掉头时恒为 ±0.0，其符号由坐标差的 -0.0 传播**偶然**决定——
        # 实测北↔南掉头右转、东↔西掉头左转（非设计行为，跨列/跨行不一致）。
        # +0.0 把 -0.0 归一为 +0.0（IEEE 754：-0.0+0.0=+0.0，非零值无影响），
        # 掉头一律左转（ang=+π → turn_sign=+1）。规划层 mission_router 的
        # 掉头扫掠检查已按**两侧并集**覆盖（180° 双侧保守），任一侧皆安全。
        # 非 180° 转向 cross 非零，+0.0 为空操作——本修复只影响掉头侧。
        cross = d1x * d2y - d1y * d2x + 0.0
        ang = math.atan2(cross, d1x * d2x + d1y * d2y)

        label = (self.route_labels[self.seg_idx + 1]
                 if self.seg_idx + 1 < len(self.route_labels) else '?')

        if abs(ang) < 0.15:
            # Nearly straight through the waypoint
            self.seg_idx += 1
            self.get_logger().info(f"Passed {label} (straight through)")
            return

        l2 = math.hypot(d2x, d2y)
        if l2 < 1e-6:
            self.seg_idx += 1
            return

        self.mode = 'turn'
        self.turn_sign = 1.0 if ang > 0 else -1.0
        # Travel is only a SAFETY bound now; the arc stops on yaw.
        self.turn_arc_len = r_corner * abs(ang) * self.TURN_ARC_SCALE
        self.turn_start_travel = self.raw_travel
        self.turn_start_yaw = self.current_yaw
        self.turn_target_ang = ang
        # Time-based safety: if turn takes > 1.5x expected, force exit.
        expected_time = self.turn_arc_len / self.TURN_SPEED
        self.turn_start_time = self.get_clock().now().nanoseconds * 1e-9
        self.turn_max_time = expected_time * 1.5
        self.turn_exit_yaw = math.atan2(d2y, d2x)
        # Exit point at one nominal radius past the corner (best single
        # guess; yaw-based stop makes the physical exit radius per-side
        # asymmetric, position snap tolerates the difference).
        r_exit = r_corner
        self.turn_exit_x = b['x'] + r_exit * d2x / l2
        self.turn_exit_y = b['y'] + r_exit * d2y / l2

        self.get_logger().info(
            f"Turn at {label}: {math.degrees(ang):+.0f}deg (yaw-"
            f"closed-loop, lead {self.TURN_YAW_LEAD_DEG:.0f}deg), "
            f"R={r_corner:.2f}m, travel_cap={self.turn_arc_len:.2f}m, "
            f"v={self.TURN_SPEED:.2f}"
        )

    def turn_update(self, cmd):
        """Hold full-lock arc until yaw reaches target (travel = safety)."""
        cmd.linear.x = self.TURN_SPEED
        cmd.angular.z = self.turn_sign * MAX_STEER_DEG
        self.last_steer = cmd.angular.z

        traveled = self.raw_travel - self.turn_start_travel
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now_sec - self.turn_start_time
        time_up = elapsed >= self.turn_max_time

        # Signed rotation magnitude in the turn direction since arc start.
        rot = (self.turn_sign * normalize_angle(
            self.current_yaw - self.turn_start_yaw))
        target_rot = (abs(self.turn_target_ang)
                      - math.radians(self.TURN_YAW_LEAD_DEG))
        yaw_done = rot >= target_rot
        travel_up = traveled >= self.turn_arc_len

        if yaw_done or travel_up or time_up:
            stop_reason = ('yaw' if yaw_done
                           else ('travel' if travel_up else 'timeup'))
            # Arc done: snap BOTH position and yaw to ideal exit. The
            # remaining few degrees (lead + servo return lag) are finished
            # by pure pursuit on the next leg via position feedback.
            yaw_err_deg = math.degrees(
                normalize_angle(self.turn_exit_yaw - self.current_yaw))
            self.snap_x_off += self.turn_exit_x - self.current_x
            self.snap_y_off += self.turn_exit_y - self.current_y
            self.current_x = self.turn_exit_x
            self.current_y = self.turn_exit_y
            self.snap_yaw_off = normalize_angle(
                self.snap_yaw_off + math.radians(yaw_err_deg))
            self.current_yaw = self.turn_exit_yaw

            self.seg_idx += 1
            self.mode = 'drive'
            nxt = (self.route_labels[self.seg_idx + 1]
                   if self.seg_idx + 1 < len(self.route_labels) else 'target')
            self.get_logger().info(
                f"Turn done after {traveled:.2f}m rot={math.degrees(rot):.0f}"
                f"/{math.degrees(abs(self.turn_target_ang)):.0f}deg "
                f"[{stop_reason}] yaw_err {yaw_err_deg:+.0f}deg snapped"
                f", -> {nxt}"
            )

        self.cmd_pub.publish(cmd)

    def pure_pursuit_control(self, aim_x, aim_y, base_speed):
        """Steer toward aim point; speed derated by steering angle."""
        dx = aim_x - self.current_x
        dy = aim_y - self.current_y
        dist = math.hypot(dx, dy)

        if dist < 0.01:
            return 0.0, 0.0

        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        rel_x = dx * cos_yaw + dy * sin_yaw
        rel_y = -dx * sin_yaw + dy * cos_yaw

        alpha = math.atan2(rel_y, rel_x)

        if abs(alpha) > 1.75:
            # Safety only: the arc machine should handle real turns.
            if rel_y > 0.02:
                side = 1.0
            elif rel_y < -0.02:
                side = -1.0
            else:
                side = 1.0 if self.last_steer >= 0 else -1.0
            steer_deg = MAX_STEER_DEG * side
            linear_vel = 0.10
        else:
            steer_deg = math.degrees(
                math.atan(2.0 * WHEELBASE * math.sin(alpha) / max(dist, 0.15))
            )
            steer_deg = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer_deg))
            linear_vel = base_speed * (1.0 - 0.6 * abs(steer_deg) / MAX_STEER_DEG)
            linear_vel = max(0.08, linear_vel)

        self.last_steer = steer_deg
        return linear_vel, steer_deg

    def enter_shoot_stop(self):
        """Stop at a mid-route shoot waypoint, enable gimbal + shooting."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

        b = self.path_points[self.seg_idx + 1]
        label = (self.route_labels[self.seg_idx + 1]
                 if self.seg_idx + 1 < len(self.route_labels) else '?')

        # Snap belief to the waypoint center, yaw to the incoming leg
        # direction so the gimbal faces along the travel direction.
        a = self.path_points[self.seg_idx]
        leg_yaw = math.atan2(b['y'] - a['y'], b['x'] - a['x'])
        drift_x = self.current_x - b['x']
        drift_y = self.current_y - b['y']
        self.snap_x_off -= drift_x
        self.snap_y_off -= drift_y
        self.current_x = b['x']
        self.current_y = b['y']
        yaw_drift_deg = math.degrees(normalize_angle(self.current_yaw - leg_yaw))
        self.snap_yaw_off = normalize_angle(
            self.snap_yaw_off - math.radians(yaw_drift_deg))
        self.current_yaw = leg_yaw

        self.mode = 'shoot_wait'
        self.shoot_wait_start = self.get_clock().now().nanoseconds * 1e-9
        self.shoot_count_at_arrival = self.shoot_count

        sig = UInt8()
        sig.data = 1
        self.stop_signal_pub.publish(sig)
        self.get_logger().info(
            f"Shoot stop at {label}: waiting for "
            f"{self.SHOOT_COUNT_TARGET} shots (start count "
            f"{self.shoot_count_at_arrival})"
        )

    def handle_arrival(self, dist_to_final):
        """Arrival handling with wrong-heading retry."""
        cmd = Twist()

        if (self.final_heading
                and self.realign_count < self.realign_limit
                and not self.arrive_logged):
            target_yaw = heading_yaw(self.final_heading)
            yaw_err_deg = math.degrees(
                normalize_angle(self.current_yaw - target_yaw))
            if abs(yaw_err_deg) > 60.0:
                self.realign_count += 1
                self.path_points = [make_point(self.current_x, self.current_y),
                                    self.path_points[-1]]
                self.route_labels = [self.route_labels[0],
                                     self.route_labels[-1]]
                self.seg_idx = 0
                self.mode = 'drive'
                self.close_since = None
                self.get_logger().info(
                    f"Reached target facing wrong way "
                    f"({yaw_err_deg:+.0f}deg), re-approaching "
                    f"(try {self.realign_count}/{self.realign_limit})"
                )
                return

        self.arrived = True
        self.mode = 'idle'
        self.cmd_pub.publish(cmd)

        if not self.arrive_logged:
            self.arrive_logged = True
            final_point = self.path_points[-1]
            if dist_to_final <= 0.28:
                drift_x = self.current_x - final_point['x']
                drift_y = self.current_y - final_point['y']
                self.snap_x_off -= drift_x
                self.snap_y_off -= drift_y
                self.current_x = final_point['x']
                self.current_y = final_point['y']
                if self.final_heading:
                    target_yaw = heading_yaw(self.final_heading)
                    yaw_drift_deg = math.degrees(
                        normalize_angle(self.current_yaw - target_yaw))
                    self.snap_yaw_off = normalize_angle(
                        self.snap_yaw_off - math.radians(yaw_drift_deg))
                    self.current_yaw = target_yaw
                    self.get_logger().info(
                        f"ARRIVED! dist={dist_to_final:.3f}m, "
                        f"drift=({drift_x:+.2f}m, {drift_y:+.2f}m), "
                        f"yaw_drift={yaw_drift_deg:+.1f}deg, stopped"
                    )
                else:
                    self.get_logger().info(
                        f"ARRIVED! dist={dist_to_final:.3f}m, "
                        f"drift=({drift_x:+.2f}m, {drift_y:+.2f}m), "
                        "stopped (heading kept)"
                    )
            else:
                self.get_logger().info(
                    f"ARRIVED (forced stop, dist={dist_to_final:.2f}m). "
                    "Send 'reset:BLOCK,HEAD' to re-anchor if needed"
                )

            # Signal dzrknn_dataprocess to enable gimbal scanning + shooting
            sig = UInt8()
            sig.data = 1
            self.stop_signal_pub.publish(sig)
            self.get_logger().info("Published stop_signal=1 (enable gimbal+shooting)")

    def control_loop(self):
        """Main control loop (drive/turn mode machine)."""
        if not self.has_odom:
            return

        cmd = Twist()

        if (not self.path_loaded or len(self.path_points) < 2
                or self.mode == 'idle' or self.arrived):
            self.cmd_pub.publish(cmd)
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # Hard timeout for the whole route
        if (self.nav_start is not None
                and now_sec - self.nav_start > self.nav_timeout):
            self.arrived = True
            self.mode = 'idle'
            self.cmd_pub.publish(cmd)
            self.get_logger().error(
                "Navigation timeout, forcing stop. "
                "Send 'reset:BLOCK,HEAD' to re-anchor if needed"
            )
            return

        # ---- SHOOT_WAIT mode: stopped, waiting for N shots -------------
        if self.mode == 'shoot_wait':
            self.cmd_pub.publish(cmd)  # hold position
            # Keep the route timeout from firing while we wait
            self.nav_start = now_sec
            done = (self.shoot_count - self.shoot_count_at_arrival
                    >= self.SHOOT_COUNT_TARGET)
            timed_out = (now_sec - self.shoot_wait_start
                         > self.SHOOT_WAIT_TIMEOUT)
            if done or timed_out:
                sig = UInt8()
                sig.data = 0
                self.stop_signal_pub.publish(sig)
                if timed_out and not done:
                    self.get_logger().warn(
                        "Shoot wait timed out, continuing route")
                else:
                    self.get_logger().info(
                        f"{self.SHOOT_COUNT_TARGET} shots fired, "
                        "resuming route")
                # Handle the corner at this waypoint exactly like a normal
                # pass (straight-through advances, real corner turns).
                self.start_turn(self.corner_radius(self.seg_idx))
                if self.mode == 'shoot_wait':
                    self.mode = 'drive'  # straight-through waypoint
            return

        # ---- TURN mode: open-loop arc ---------------------------------
        if self.mode == 'turn':
            self.turn_update(cmd)
            return

        # ---- DRIVE mode ------------------------------------------------
        final_point = self.path_points[-1]
        dx = final_point['x'] - self.current_x
        dy = final_point['y'] - self.current_y
        dist_to_final = math.hypot(dx, dy)

        # Orbiting watchdog near the final target (drive mode only)
        if dist_to_final < self.close_radius:
            if self.close_since is None:
                self.close_since = now_sec
            elif now_sec - self.close_since > self.close_grace:
                self.get_logger().info(
                    "Close to target but not converging (orbiting), "
                    "forcing stop"
                )
                self.handle_arrival(dist_to_final)
                return
        else:
            self.close_since = None

        # Arrival
        if dist_to_final < self.arrival_threshold:
            self.handle_arrival(dist_to_final)
            return

        # Overshoot guard
        if dist_to_final < 0.5:
            self.was_close = True
        elif self.was_close and dist_to_final > 1.0:
            self.path_points = [make_point(self.current_x, self.current_y),
                                final_point]
            self.route_labels = [self.route_labels[0],
                                 self.route_labels[-1]]
            self.seg_idx = 0
            self.was_close = False
            self.close_since = None
            self.get_logger().info(
                "Overshot the target, re-approaching from current position"
            )

        a = self.path_points[self.seg_idx]
        b = self.path_points[self.seg_idx + 1]
        t, _ = self._project_on_segment(a['x'], a['y'], b['x'], b['y'])
        leg_len = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        if leg_len < 1e-6:
            return
        dist_to_b = (1.0 - t) * leg_len

        # Signed cross-track error, + means belief is LEFT of the leg
        # direction (left = 90deg CCW from travel direction in map frame).
        cte = (((self.current_x - a['x']) * (-(b['y'] - a['y']))
               + (self.current_y - a['y']) * (b['x'] - a['x'])) / leg_len)

        base_speed = self.STRAIGHT_SPEED
        has_corner = self.seg_idx < len(self.path_points) - 2
        is_shoot_wp = (has_corner
                       and self.seg_idx + 1 < len(self.route_shoot)
                       and self.route_shoot[self.seg_idx + 1])

        if is_shoot_wp:
            # Mid-route shoot stop: brake gently and halt near the waypoint
            if dist_to_b < 0.6:
                ramp = max(0.0, dist_to_b / 0.6)
                base_speed = max(0.10, self.STRAIGHT_SPEED * ramp)
            if dist_to_b <= self.SHOOT_STOP_RADIUS:
                self.enter_shoot_stop()
                return
        elif has_corner:
            r_corner = self.corner_radius(self.seg_idx)
            if dist_to_b <= r_corner:
                self.start_turn(r_corner)
                return
            # Ramp down toward the turn start point
            d_turn_start = dist_to_b - r_corner
            if d_turn_start < 0.6:
                ramp = max(0.0, d_turn_start / 0.6)
                base_speed = (self.TURN_SPEED
                              + ramp * (self.STRAIGHT_SPEED - self.TURN_SPEED))
        else:
            # Final leg: brake before the target for a clean stop
            if dist_to_final < self.final_ramp_dist:
                base_speed = max(
                    0.10,
                    self.STRAIGHT_SPEED * dist_to_final / self.final_ramp_dist)

        # Aim point ON the current leg (never around the corner: the
        # dedicated arc handles corners)
        s_ahead = min(t * leg_len + self.AIM_DRIVE, leg_len)
        frac = s_ahead / leg_len
        aim_x = a['x'] + frac * (b['x'] - a['x'])
        aim_y = a['y'] + frac * (b['y'] - a['y'])

        linear_vel, steer_deg = self.pure_pursuit_control(
            aim_x, aim_y, base_speed)

        # Cross-track correction: 纯追踪 + CTE 修正。
        # 吸收 Line_Follower 的自适应 PID：在比例项（原 CTE_GAIN=115）基础上加入
        # 输入滤波/死区/积分分离/积分限幅/微分先行/软限幅 六重抗饱和机制，
        # 消除稳态横向误差并抑制抖动。cte > 0 = 车在航线左侧 → 需向右打舵。
        if self.seg_idx != self._cte_seg:      # 进入新航段：复位积分/微分状态
            self.cte_ctrl.reset()
            self._cte_seg = self.seg_idx
        steer_deg += self.cte_ctrl.update(cte, 0.05)   # 控制周期 20Hz
        steer_deg = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer_deg))

        cmd.linear.x = linear_vel
        cmd.angular.z = steer_deg + self.STEER_TRIM
        self.cmd_pub.publish(cmd)
        self.arrive_logged = False

        # Periodic status log
        self._log_count += 1
        if self._log_count % 40 == 0:  # Every 2 seconds
            aim_label = (self.route_labels[min(self.seg_idx + 1,
                                               len(self.route_labels) - 1)]
                         if self.route_labels else '?')
            self.get_logger().info(
                f"[{self.mode}] pos=({self.current_x:.2f}, "
                f"{self.current_y:.2f})m seg={aim_label} "
                f"dist={dist_to_final:.2f}m cte={cte:+.2f}m "
                f"v={linear_vel:.2f}"
                f"(base={base_speed:.2f}) steer={steer_deg:.1f}deg "
                f"yaw={math.degrees(self.current_yaw):.0f}deg"
            )


def main(args=None):
    path_file = None

    if len(sys.argv) > 1:
        path_file = sys.argv[1]

    rclpy.init(args=args)
    node = PathExecutorNode(path_file)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
