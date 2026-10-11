# -*- coding: utf-8 -*-
"""
Coordinate transformation node - Challenge: Ground-Air Cooperative Reconnaissance

Coordinate system definition:
- Map: 14 columns (A-N) x 11 rows (1-11), each block 35cm x 35cm
- Origin: Bottom-right corner of block N11
- X axis positive: row 11 -> row 1 (bottom to top)
- Y axis positive: col N -> col A (right to left)
- Unit: 1 unit = 1 block = 35cm
- Start position: (1.5, 0.5) at block N10 center, heading Y+

Strategy:
  - Yaw: IMU quaternion from /imu_data (accurate during turns).
  - Position: DEAD RECKONING. dzactuator's integrated pose lives in an
    encoder-yaw frame whose rotation does not match the physical world
    (verified 2026-09-06: after the same 90deg right turn, the raw track
    drifted to opposite sides across two runs and spirals outward
    monotonically), so raw pose deltas are unusable as world
    coordinates. The sample-to-sample travel DISTANCE is
    rotation-invariant and accurate, so it is integrated along the IMU
    map yaw instead. Residual error is removed by path_executor's
    per-corner position snapping.
  - 'reset:BLOCK,HEAD' re-bases the DR direction to HEAD, so the car may
    be placed/rotated freely BEFORE the reset (the pi/2 default only
    assumes it faced Y+ when this node started).
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from copy import deepcopy

# ===== Map constants =====
COL_LETTERS = 'ABCDEFGHIJKLMN'
MAP_COLS = 14
MAP_ROWS = 11
BLOCK_SIZE = 0.35

# ===== Coordinate params =====
START_X = 1.5
START_Y = 0.5

# Heading token -> map yaw (radians). Same semantics as path_executor.
HEADING_YAWS = {'x': 0.0, '-x': math.pi, 'y': math.pi / 2, '-y': -math.pi / 2}

# IMU yaw direction: verified CCW-positive (rotating the car LEFT
# increases the map yaw). Must match FLIP_RAW_YAW_SIGN in
# path_executor.py.
FLIP_RAW_YAW_SIGN = False

# Log tag: proves which version of this node is actually running.
# The launch file imports the INSTALLED copy (site-packages), so a
# stale colcon install silently runs old code - this tag makes that
# visible in every log line.
NODE_VERSION = "v4-dr-reset"

# If no IMU message for this long (seconds), fall back to odom yaw.
IMU_TIMEOUT = 2.0

# Ignore per-sample travel below this (m) to skip stationary jitter.
MIN_STEP_M = 0.0005


def get_block_name(x: float, y: float) -> str:
    row_idx = round(10.5 - x)
    col_idx = round(13.5 - y)
    row_idx = max(0, min(MAP_ROWS - 1, row_idx))
    col_idx = max(0, min(MAP_COLS - 1, col_idx))
    letter = COL_LETTERS[col_idx]
    number = row_idx + 1
    return f"{letter}{number}"


def euler_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class SerialReaderNode(Node):
    def __init__(self):
        super().__init__("serial_reader_node")

        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)
        self.sub_imu = self.create_subscription(
            Imu, "/imu_data", self.imu_callback, 10)
        # reset commands (path_executor listens on the same topic): the
        # dead-reckoning DIRECTION is re-based to the commanded heading.
        # Position re-anchoring stays in path_executor's snap logic, so
        # the two nodes never double-correct each other.
        self.sub_reset = self.create_subscription(
            String, "/nav/waypoint", self.waypoint_callback, 10)

        self.odom_publisher = self.create_publisher(
            Odometry, "/odom/localization", 10)

        self.init_x = None
        self.init_y = None
        self.log_counter = 0

        # IMU state
        self.imu_yaw_init = None
        self.imu_yaw_rel = None
        self.imu_latest_time = None

        # Dead-reckoning state: previous raw pose sample (for travel
        # distance) and the integrated map-frame position in meters,
        # anchored at the N10 start point.
        self.prev_px = None
        self.prev_py = None
        self.dr_x_m = START_X * BLOCK_SIZE
        self.dr_y_m = START_Y * BLOCK_SIZE
        # DR direction: map_yaw = yaw_rel + DR_YAW_OFF. The pi/2 default
        # assumes the car faces Y+ when this node starts; if the car was
        # rotated after startup, integration ran along a rotated direction
        # and the position belief fanned out sideways (immediate turn at
        # route start). 'reset:BLOCK,HEAD' re-bases this offset so the DR
        # follows the true heading. Only the internal DR direction and the
        # diagnostic print use it; the published quaternion keeps the
        # legacy yaw_rel + pi/2 convention that path_executor's snap
        # logic consumes (no cross-node race).
        self.dr_yaw_off = math.pi / 2

        self.get_logger().info("=" * 50)
        self.get_logger().info(
            "Coordinate Transform Node (IMU yaw + dead-reckoned pos) "
            f"[{NODE_VERSION}]")
        self.get_logger().info(f"Map: {MAP_COLS}x{MAP_ROWS}, block={BLOCK_SIZE}m")
        self.get_logger().info(f"Start: ({START_X}, {START_Y}) blocks")
        self.get_logger().info(f"IMU: /imu_data, fallback after {IMU_TIMEOUT}s")
        self.get_logger().info("=" * 50)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        raw_yaw = euler_yaw(q.x, q.y, q.z, q.w)
        if self.imu_yaw_init is None:
            self.imu_yaw_init = raw_yaw
            self.get_logger().info(
                f"IMU yaw init at {math.degrees(raw_yaw):+.1f}deg")
        self.imu_yaw_rel = normalize_angle(raw_yaw - self.imu_yaw_init)
        self.imu_latest_time = self.get_clock().now()

    def waypoint_callback(self, msg: String):
        """Re-base the DR direction on 'reset:BLOCK,HEAD'.

        The reset semantic is: the car is AT block, FACING head, right now.
        So the internal map yaw must equal the head's yaw from this moment
        on, otherwise travel distance gets integrated along a rotated
        direction and the position belief fans out sideways.
        """
        data = msg.data.strip()
        if not data.startswith('reset:'):
            return
        parts = data[6:].split(',')
        if len(parts) < 2:
            self.get_logger().error(
                "reset parse error, expected 'reset:BLOCK,HEAD'")
            return
        head = parts[1].strip().lower()
        target_yaw = HEADING_YAWS.get(head)
        if target_yaw is None:
            self.get_logger().error(f"Invalid reset heading: '{head}'")
            return
        if self.imu_yaw_rel is None:
            self.get_logger().warning(
                "reset ignored: no IMU yaw received yet")
            return
        cur = normalize_angle(self.imu_yaw_rel + self.dr_yaw_off)
        self.dr_yaw_off = normalize_angle(
            self.dr_yaw_off + target_yaw - cur)
        self.get_logger().info(
            f"[{NODE_VERSION}] reset: DR direction re-anchored, "
            f"map_yaw -> {math.degrees(target_yaw):+.0f}deg "
            f"(was {math.degrees(cur):+.0f}deg)")

    def odom_callback(self, msg: Odometry):
        if self.init_x is None:
            self.init_x = msg.pose.pose.position.x
            self.init_y = msg.pose.pose.position.y
            self.get_logger().info(
                f"Raw odom start: ({self.init_x:.3f}, "
                f"{self.init_y:.3f}) m")

        # ---- Yaw: prefer IMU, fallback to odom ----
        now = self.get_clock().now()
        imu_fresh = (
            self.imu_yaw_rel is not None
            and self.imu_latest_time is not None
            and (now - self.imu_latest_time).nanoseconds * 1e-9 < IMU_TIMEOUT
        )
        if imu_fresh:
            yaw_rel = self.imu_yaw_rel
            yaw_src = "imu"
        else:
            q = msg.pose.pose.orientation
            yaw_rel = normalize_angle(euler_yaw(q.x, q.y, q.z, q.w))
            yaw_src = "odom"
        if FLIP_RAW_YAW_SIGN:
            yaw_rel = -yaw_rel

        # ---- Position: dead-reckon travel distance along map yaw ----
        # The raw pose frame rotates wrongly, but the DISTANCE between
        # consecutive samples is rotation-invariant and accurate.
        if self.prev_px is not None:
            step = math.hypot(msg.pose.pose.position.x - self.prev_px,
                              msg.pose.pose.position.y - self.prev_py)
            if step >= MIN_STEP_M:
                map_yaw = normalize_angle(yaw_rel + self.dr_yaw_off)
                self.dr_x_m += step * math.cos(map_yaw)
                self.dr_y_m += step * math.sin(map_yaw)
        self.prev_px = msg.pose.pose.position.x
        self.prev_py = msg.pose.pose.position.y

        new_x = self.dr_x_m / BLOCK_SIZE
        new_y = self.dr_y_m / BLOCK_SIZE
        block_name = get_block_name(new_x, new_y)

        # ---- Build output ----
        out_msg = deepcopy(msg)
        out_msg.pose.pose.position.x = new_x
        out_msg.pose.pose.position.y = new_y

        # Quaternion carries yaw_rel so path_executor's
        # RAW_YAW_TO_MAP_YAW_OFFSET = pi/2 produces correct map yaw.
        half = yaw_rel / 2.0
        out_msg.pose.pose.orientation.x = 0.0
        out_msg.pose.pose.orientation.y = 0.0
        out_msg.pose.pose.orientation.z = math.sin(half)
        out_msg.pose.pose.orientation.w = math.cos(half)

        self.odom_publisher.publish(out_msg)

        self.log_counter += 1
        if self.log_counter % 10 == 0:
            map_yaw_deg = math.degrees(
                normalize_angle(yaw_rel + self.dr_yaw_off))
            self.get_logger().info(
                f"pos: ({new_x:.2f}, {new_y:.2f}) | "
                f"block: {block_name} | "
                f"yaw: {map_yaw_deg:+.0f}deg [{yaw_src}] "
                f"[{NODE_VERSION}]"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
