# -*- coding: utf-8 -*-
"""
Path planning and waypoint navigation node.

Subscribes to:
- /odom/localization : current position (x, y in block units) + orientation
- /nav/waypoint : target waypoint command

Publishes to:
- /movebase_angle : velocity command for car control

Waypoint command format (std_msgs/msg/String):
  "M10,y"  -> go to block M10, facing Y axis positive
  "M7,x"   -> go to block M7, facing X axis positive
  "K7,y"   -> go to block K7, facing Y axis positive

Orientation:
  'x' = face X axis positive direction (row 11->1)
  'y' = face Y axis positive direction (col N->A)

Motion strategy:
  1. Turn to face the target direction first (in-place rotation)
  2. Drive straight to the target block center
  3. Stop when close enough
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu

# QoS matching dzactuator subscriptions
CMD_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
MODE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# ===== Map constants =====
COL_LETTERS = 'ABCDEFGHIJKLMN'
MAP_COLS = 14
MAP_ROWS = 11
BLOCK_SIZE = 0.35

# ===== Navigation params =====
ARRIVAL_DISTANCE = 0.15       # blocks (~5cm), arrival threshold
HEADING_TOLERANCE = 0.1      # radian (~6 deg), heading tolerance
DRIVE_SPEED = 0.35           # m/s, forward speed
TURN_SPEED = 0.8             # rad/s, turning speed
REVERSE_SPEED = 0.25         # m/s, reverse speed
MAX_YAW_RATE = 1.2           # rad/s, max turn rate


def block_to_coords(block_name):
    """Convert block name (e.g. 'M10') to block center coordinates."""
    letter = block_name[0]
    number = int(block_name[1:])
    col_idx = COL_LETTERS.index(letter)
    row_idx = number - 1
    x = 10.5 - row_idx
    y = 13.5 - col_idx
    return x, y


def heading_to_yaw(heading_dir):
    """Convert heading direction ('x' or 'y') to yaw angle (radians)."""
    if heading_dir == 'y':
        return math.pi / 2  # Y axis positive
    elif heading_dir == 'x':
        return 0.0          # X axis positive
    else:
        return 0.0


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


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # State machine states
        self.STATE_IDLE = 0
        self.STATE_TURN = 1
        self.STATE_DRIVE = 2
        self.STATE_ARRIVED = 3

        self.state = self.STATE_IDLE
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_heading = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False

        # Subscribe to localization
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom/localization',
            self.odom_callback,
            10
        )

        # Subscribe to IMU for better orientation
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Subscribe to waypoint commands
        self.waypoint_sub = self.create_subscription(
            String,
            '/nav/waypoint',
            self.waypoint_callback,
            10
        )

        # Publish velocity commands to dzactuator (pursuitAngle for Pursuit mode)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'pursuitAngle',
            CMD_QOS
        )

        # Publish mode control to switch dzactuator to Pursuit mode
        self.mode_ctrl_pub = self.create_publisher(
            Twist,
            'logicontrol',
            MODE_QOS
        )

        # Create timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        # Create timer for mode switching (send mode command periodically)
        self.mode_timer = self.create_timer(0.5, self.mode_switch_callback)  # 2Hz

        self.last_heading_update = self.get_clock().now()
        self.log_count = 0
        self.start_time = self.get_clock().now()

        self.get_logger().info("=" * 50)
        self.get_logger().info("Path Planner Node Initialized")
        self.get_logger().info("Command format: 'BLOCK_NAME,HEADING'")
        self.get_logger().info("  Example: 'M10,y' -> go to M10 facing Y+")
        self.get_logger().info("  Heading: 'x' = X+, 'y' = Y+")
        self.get_logger().info("=" * 50)

    def send_mode_command(self):
        """Send mode switch command to dzactuator to enable Pursuit mode."""
        mode_msg = Twist()
        mode_msg.linear.x = 0.0   # speed
        mode_msg.linear.y = 0.0   # Must be < 0.5 for Pursuit mode
        mode_msg.linear.z = 2.0   # z=2.0 -> Pursuit mode
        mode_msg.angular.x = 0.0
        mode_msg.angular.y = 0.0
        mode_msg.angular.z = 0.0  # angle
        self.mode_ctrl_pub.publish(mode_msg)

    def mode_switch_callback(self):
        """Periodically send mode switch command to keep dzactuator in Pursuit mode."""
        self.send_mode_command()

    def odom_callback(self, msg):
        """Update current position from localization odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.has_odom = True

        # Get yaw from odometry orientation
        q = msg.pose.pose.orientation
        self.current_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def imu_callback(self, msg):
        """Update orientation from IMU (higher rate than odom)."""
        q = msg.orientation
        if q.w != 0.0 or q.x != 0.0 or q.y != 0.0 or q.z != 0.0:
            imu_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            # Blend with odom yaw (favor IMU for stability)
            self.current_yaw = 0.3 * self.current_yaw + 0.7 * imu_yaw

    def waypoint_callback(self, msg):
        """Parse waypoint command and start navigation."""
        try:
            # Check if node has been running long enough for mode switch
            elapsed_ns = (self.get_clock().now() - self.start_time).nanoseconds
            elapsed = elapsed_ns / 1e9
            if elapsed < 3.0:
                self.get_logger().warning(
                    f"Please wait {3.0 - elapsed:.1f}s for mode switch before sending commands"
                )
                return

            parts = msg.data.strip().split(',')
            if len(parts) != 2:
                self.get_logger().error(
                    f"Invalid format: '{msg.data}'. Expected 'BLOCK,HEADING'"
                )
                return

            block_name = parts[0].strip().upper()
            heading_dir = parts[1].strip().lower()

            if heading_dir not in ['x', 'y']:
                self.get_logger().error(
                    f"Invalid heading: '{heading_dir}'. Use 'x' or 'y'"
                )
                return

            self.target_x, self.target_y = block_to_coords(block_name)
            self.target_heading = heading_to_yaw(heading_dir)
            self.state = self.STATE_TURN

            self.get_logger().info(
                f"Navigating to {block_name} ({self.target_x:.2f}, {self.target_y:.2f}) "
                f"heading={heading_dir}({self.target_heading:.2f}rad)"
            )
            self.get_logger().info(
                f"Current: ({self.current_x:.2f}, {self.current_y:.2f}) "
                f"yaw={self.current_yaw:.2f}rad"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to parse command: {e}")

    def control_loop(self):
        """Main control loop - runs at 20Hz."""
        if not self.has_odom:
            return

        cmd = Twist()

        if self.state == self.STATE_IDLE:
            # No target, but still send idle command to keep pursuit_active
            # This ensures dzactuator stays in Pursuit mode
            self.cmd_vel_pub.publish(cmd)
            return

        elif self.state == self.STATE_TURN:
            # Turn to face target heading
            yaw_error = normalize_angle(self.target_heading - self.current_yaw)

            if abs(yaw_error) < HEADING_TOLERANCE:
                # Heading achieved, start driving
                self.state = self.STATE_DRIVE
                self.get_logger().info("Heading achieved, starting drive")
                return

            # Turn in place - need small forward speed for turning
            turn_dir = 1.0 if yaw_error > 0 else -1.0
            turn_speed = min(TURN_SPEED, abs(yaw_error) * 3.0)

            cmd.angular.z = turn_speed * turn_dir
            cmd.linear.x = 0.15  # Small forward speed for turning
            self.cmd_vel_pub.publish(cmd)

        elif self.state == self.STATE_DRIVE:
            # Calculate distance to target
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < ARRIVAL_DISTANCE:
                # Arrived
                self.state = self.STATE_ARRIVED
                self.get_logger().info(
                    f"ARRIVED at ({self.current_x:.2f}, {self.current_y:.2f})"
                )
                return

            # Calculate desired heading (toward target)
            desired_yaw = math.atan2(dy, dx)
            yaw_error = normalize_angle(desired_yaw - self.current_yaw)

            # Also need to align with target heading while driving
            heading_error = normalize_angle(self.target_heading - self.current_yaw)

            # Control strategy:
            # 1. If significantly off-course, turn more
            # 2. If aligned with target heading, drive straight
            # 3. If aligned but not facing target heading, rotate toward it

            # Turn component
            if abs(heading_error) > HEADING_TOLERANCE:
                # Need to turn to target heading first
                turn_dir = 1.0 if heading_error > 0 else -1.0
                cmd.angular.z = min(MAX_YAW_RATE, abs(heading_error) * 4.0) * turn_dir
                cmd.linear.x = 0.0
            else:
                # Correct minor heading error while driving
                turn_correction = 0.0
                if abs(yaw_error) > 0.3:
                    turn_correction = 0.5 * yaw_error / abs(yaw_error)
                elif abs(yaw_error) > 0.1:
                    turn_correction = 0.2 * yaw_error / abs(yaw_error)

                cmd.angular.z = turn_correction
                cmd.linear.x = DRIVE_SPEED

            self.cmd_vel_pub.publish(cmd)

        elif self.state == self.STATE_ARRIVED:
            # Stop and wait for next command
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.state = self.STATE_IDLE

            self.get_logger().info(
                f"Final position: ({self.current_x:.2f}, {self.current_y:.2f})"
            )
            self.get_logger().info("Waiting for next waypoint...")

        # Periodic status log
        self.log_count += 1
        if self.log_count % 40 == 0:  # Every 2 seconds
            state_names = ['IDLE', 'TURN', 'DRIVE', 'ARRIVED']
            state_name = state_names[self.state] if self.state < len(state_names) else '???'

            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            dist = math.sqrt(dx * dx + dy * dy)

            heading_err = normalize_angle(self.target_heading - self.current_yaw)

            self.get_logger().info(
                f"[{state_name}] pos=({self.current_x:.2f},{self.current_y:.2f}) "
                f"target=({self.target_x:.2f},{self.target_y:.2f}) "
                f"dist={dist:.2f}blocks "
                f"yaw={self.current_yaw:.2f}rad "
                f"heading_err={heading_err:.2f}rad"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
        # Send stop command
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
