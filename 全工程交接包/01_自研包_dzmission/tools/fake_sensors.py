#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 fake_sensors.py —— 假传感器发布器（离线测试用）
================================================================================

作用
----
在没有真底盘、真相机的情况下，发布车端所需的三类传感器话题，
让 `dzmission` 的**定位状态变健康**、脱离 ESTOP，从而真正跑起：
    · 排雷规划（40 分）
    · 桥洞过门判定（20 分）
    · 云台识别与红蓝面联锁（打击）

发布的话题
----------
| 话题              | 类型                       | 频率   | 车端用途 |
|-------------------|----------------------------|--------|----------|
| `/imu_data`       | sensor_msgs/Imu            | 100 Hz | ESKF 预测 + **地图偏航**来源 |
| `/odom`           | nav_msgs/Odometry          |  50 Hz | **距离**（沿偏航积分）+ 堵转判定 |
| `/offset_center`  | std_msgs/Int32MultiArray   |  15 Hz | 近场避碰 + 云台瞄准 + 面/红点 |

为什么这三样就够
----------------
车端定位补偿的设计（见 doc 与 localization_compensator.cpp）：
    **位置 = odom 的相邻采样距离，沿 IMU 地图偏航角积分**
    （不用 dzactuator 积分出的位姿，实测其朝向与物理世界不一致）
所以：
    · `odom.twist.twist.linear.x` -> 前进距离增量（**必需**）
    · `imu.orientation`           -> 相对偏航（**必需**，标定零点后映射到地图系）
    · `imu.angular_velocity.z`    -> ZUPT 判静止 + 静止判定
    · `imu.linear_acceleration`   -> ESKF 初始化（需含重力）

★ 重要：这与真机的差异
----------------------
    · 真 IMU 有零偏、噪声、温漂；这里发的是**理想值**
    · 真 odom 有轮径误差、打滑；这里发的是**精确里程**
    · 真相机有识别误差；这里发的 bbox 是**按几何精确算出**的
    => 本工具验证**逻辑正确性**（状态机、规划、联锁、协议），
       **不能**验证精度、鲁棒性、抗噪能力。

用法
----
    # 终端 A：车端
    ros2 launch dzmission bringup_mission.launch.py

    # 终端 B：假传感器（先发传感器，让定位健康）
    python3 tools/fake_sensors.py

    # 终端 C：地图/协议模拟器（发侦察数据与任务指令）
    python3 tools/drone_sim.py --map tools/maps/example_standard.yaml

    # 可选：让"机器人"真的按指令动起来（否则 odom 速度恒为 0，
    #       堵转判定会一直触发）
    python3 tools/fake_sensors.py --drive-from-cmd

常用选项
--------
    --drive-from-cmd   订阅 /cmd_vel，按指令速度推进里程（闭合成"能跑"的仿真）
    --start-col/--start-row  起始格（默认 M11 = 12,10）
    --no-imu / --no-odom / --no-vision   只发其中一部分（排查问题用）
    --vision-none      视觉发"无目标"（class_id=0），用于测禁火/扫描
================================================================================
"""

from __future__ import annotations

import argparse
import math
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int32MultiArray
except ImportError as e:                                  # pragma: no cover
    sys.stderr.write(
        '★ 需要 rclpy（ROS 2 Humble 自带）。请先 source 环境：\n'
        '    source /opt/ros/humble/setup.bash\n'
        '    source ~/ws/install/setup.bash\n'
        '  原始错误: %s\n' % e)
    sys.exit(2)

# 与 C++ 侧一致的场地常量（FieldGrid::Spec 默认值）
CELL_M = 0.35
COLS, ROWS = 14, 11
GRAVITY = 9.80665


def yaw_to_quat(yaw: float):
    """偏航角(rad) -> 四元数 (x, y, z, w)（只有 z 分量与 w）"""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class FakeSensors(Node):
    def __init__(self, args):
        super().__init__('fake_sensors')
        self.args = args

        # ---- 状态：位置(米)与偏航(rad)，都在**地图系** ----
        # 起点 M11 = (col 12, row 10) 的格心
        self.x = (args.start_col + 0.5) * CELL_M
        self.y = (args.start_row + 0.5) * CELL_M
        self.x0, self.y0 = self.x, self.y      # 记原点（怠速运动软性拉回用）
        self.yaw = math.radians(args.heading_deg)   # -90° = 朝北（row 减小）
        self.v = 0.0            # 前进速度 m/s（车体系 x）
        self.w = 0.0            # 偏航角速度 rad/s
        self.last_t = self.now_s()

        # ★ 怠速运动时的"上报速度"（让安全监控的堵转判定看到一致的数据）
        if args.idle_motion and not args.drive_from_cmd:
            self.v = args.idle_v
            self.w = args.yaw_rate_deg_s * math.pi / 180.0

        # 视觉：无目标为默认（class_id=0）
        self.target_visible = not args.vision_none
        self.face = args.face          # 0未知 1红/友 2蓝/敌
        self.track_phase = 0.0

        # ---- QoS：车端订阅用 BEST_EFFORT（不匹配会**静默丢弃**）----
        qos = QoSProfile(depth=20,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        qos_rel = QoSProfile(depth=20,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)

        if not args.no_imu:
            self.pub_imu = self.create_publisher(Imu, args.imu_topic, qos)
        if not args.no_odom:
            self.pub_odom = self.create_publisher(Odometry, args.odom_topic, qos)
        if not args.no_vision:
            self.pub_vision = self.create_publisher(
                Int32MultiArray, args.vision_topic, qos)

        # 可选：订阅 /cmd_vel，按指令推进（闭合成可跑的仿真）
        if args.drive_from_cmd:
            self.sub_cmd = self.create_subscription(
                Twist, args.cmd_topic, self.on_cmd, qos_rel)

        # ---- 定时器 ----
        self.t_imu = self.create_timer(1.0 / args.imu_hz, self.tick_imu)
        self.t_odom = self.create_timer(1.0 / args.odom_hz, self.tick_odom)
        self.t_vis = self.create_timer(1.0 / args.vision_hz, self.tick_vision)
        self.t_int = self.create_timer(0.02, self.integrate)   # 50Hz 积分

        self.get_logger().info(
            '假传感器启动：起点格(%d,%d) 航向%.0f° 目标面=%d 可见=%s'
            % (args.start_col, args.start_row, args.heading_deg,
               self.face, self.target_visible))
        self.get_logger().info(
            'IMU=%s(%.0fHz) ODOM=%s(%.0fHz) VISION=%s(%.0fHz) drive_from_cmd=%s'
            % (args.imu_topic, args.imu_hz, args.odom_topic, args.odom_hz,
               args.vision_topic, args.vision_hz, args.drive_from_cmd))
        self.get_logger().info(
            '★ 若车端订阅用 BEST_EFFORT，本节点也用 BEST_EFFORT —— 否则静默丢包')
        if args.idle_motion and not args.drive_from_cmd:
            self.get_logger().info(
                '★ 怠速运动已开启：%.2f m/s 前进 + %.1f°/s 偏航 —— '
                '这是**故意的**：车端只在检测到有效位移(>0.5mm/tick)时才认为定位可信，'
                '完全静止会一直 ESTOP。要模拟静止加 --no-idle-motion'
                % (args.idle_v, args.yaw_rate_deg_s))
        elif not args.idle_motion:
            self.get_logger().warn(
                '怠速运动已关闭：里程不会产生位移 -> 车端 pose_valid_ 将一直为 false '
                '-> 停在 ESTOP（这是预期行为，不是 bug）')

    # ------------------------------------------------------------------ 工具
    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg: Twist):
        # 车体系：linear.x 前进，angular.z 偏航
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def integrate(self):
        """把速度积分成里程与航向

        ★★ 关键：为什么默认要让"车"动 ★★
          车端的定位补偿有这一步（localization_compensator.cpp odomCallback）：
              step = |odom 相邻采样位移|
              if (step >= dr_min_step_m(0.5mm) && step <= dr_max_step_m(0.5m)
                  && yaw_offset_valid_)
                  -> pose_valid_ = true      // 任务层据此认为"定位可信"
          也就是说：**位置只有检测到有效位移才会被置为可信**。
          如果假传感器让位置恒定（step 恒为 0），车端 `pose_valid_` 永远是 false
          -> `localization unhealthy` -> 一直 ESTOP -> 拒绝所有 NAV_TO/ENGAGE。
          （这正是第一版的表现：数据都收到了，但车端不动。）

          所以默认开一个很小的"怠速运动"：0.05 m/s 前进 + 2°/s 缓慢偏航，
          50Hz 下每 tick 位移约 1mm —— 稳稳高于 0.5mm 门槛，也远低于 0.5m 上限。
          要模拟"完全静止"用 --no-idle-motion（那时车端会停在 ESTOP，属预期）。
        """
        t = self.now_s()
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0 or dt > 0.5:
            return

        if self.args.drive_from_cmd:
            # 跟 /cmd_vel：车体系前进 + 偏航
            self.yaw += self.w * dt
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            return

        if self.args.idle_motion:
            # 怠速运动：恒速前进 + 缓慢偏航（只为产生有效位移）
            self.yaw += self.args.yaw_rate_deg_s * math.pi / 180.0 * dt
            self.x += self.args.idle_v * math.cos(self.yaw) * dt
            self.y += self.args.idle_v * math.sin(self.yaw) * dt
            # 位置别飘太远：软性拉回起点附近（超出 2m 就掉头）
            d0x = self.x - self.x0
            d0y = self.y - self.y0
            if math.hypot(d0x, d0y) > 2.0:
                self.yaw += math.pi          # 掉头
            return

        # 完全静止（不推荐：车端会 ESTOP）

    # ------------------------------------------------------------------ IMU
    def tick_imu(self):
        if self.args.no_imu:
            return
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'imu_link'
        q = yaw_to_quat(self.yaw)
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
        m.orientation_covariance[0] = 0.001

        # 角速度：只有偏航（平地）
        m.angular_velocity.x = 0.0
        m.angular_velocity.y = 0.0
        m.angular_velocity.z = self.w
        m.angular_velocity_covariance[0] = 0.0001

        # 线加速度：静止/匀速时 IMU 读到的是比力 f = a - g，
        #   即静止时 z 分量约为 +g（这是 ESKF 初始化所期望的）
        m.linear_acceleration.x = 0.0
        m.linear_acceleration.y = 0.0
        m.linear_acceleration.z = GRAVITY
        m.linear_acceleration_covariance[0] = 0.0001

        self.pub_imu.publish(m)

    # ------------------------------------------------------------------ ODOM
    def tick_odom(self):
        if self.args.no_odom:
            return
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.child_frame_id = 'base_link'

        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        m.pose.pose.position.z = 0.0
        q = yaw_to_quat(self.yaw)
        m.pose.pose.orientation.x, m.pose.pose.orientation.y = q[0], q[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = q[2], q[3]
        m.pose.covariance[0] = 0.01
        m.pose.covariance[7] = 0.01
        m.pose.covariance[35] = 0.01

        # ★ 车端只用 twist：linear.x 前进速度、angular.z 偏航角速度
        m.twist.twist.linear.x = self.v
        m.twist.twist.linear.y = 0.0
        m.twist.twist.linear.z = 0.0
        m.twist.twist.angular.x = 0.0
        m.twist.twist.angular.y = 0.0
        m.twist.twist.angular.z = self.w
        m.twist.covariance[0] = 0.001
        m.twist.covariance[35] = 0.001

        self.pub_odom.publish(m)

    # ------------------------------------------------------------------ VISION
    def tick_vision(self):
        """发布 /offset_center（10 个整数）

        布局（与 rknn_pt/src/det_node.cc 一致）：
            [0] offset_x     像素，正=目标在画面右侧
            [1] offset_y     像素，正=目标在画面下方
            [2] class_id     0 = 无目标
            [3] dist         距离（本工具不用，置 0）
            [4..7] bbox      x1 y1 x2 y2
            [8] face         0未知 1红/友 2蓝/敌
            [9] laser_dot    0/1 激光红点是否可见
        """
        if self.args.no_vision:
            return
        d = [0] * 10

        if not self.target_visible:
            d[2] = 0
            self.pub_vision.publish(self._arr(d))
            return

        # 模拟靶标在画面里横向匀速往复（周期由 --period 给定），
        # 幅度 ±amplitude 像素 —— 用于测 α 换向点 / β 提前量
        self.track_phase += (2.0 * math.pi /
                             max(self.args.period_s, 0.1)) / self.args.vision_hz
        off = int(self.args.amplitude_px * math.sin(self.track_phase))

        d[0] = off                       # offset_x
        d[1] = 0                         # offset_y（垂直居中）
        d[2] = 1                         # class_id（单类模型，非 0 即"有目标"）
        d[3] = 0                         # dist
        d[4] = 320 + off - 24            # bbox x1
        d[5] = 240 - 30                  # bbox y1
        d[6] = 320 + off + 24            # bbox x2
        d[7] = 240 + 30                  # bbox y2（高 60px -> 近场比值用）
        d[8] = self.face                 # 面：0/1/2
        d[9] = self.args.laser_dot       # 激光红点

        self.pub_vision.publish(self._arr(d))

    @staticmethod
    def _arr(vals):
        m = Int32MultiArray()
        m.data = [int(v) for v in vals]
        return m


def main():
    p = argparse.ArgumentParser(
        description='假传感器发布器（离线测试：让车端定位健康、脱离 ESTOP）')
    # 话题名（须与 params/dzmission.yaml 里 loc.* / safety.* 一致）
    p.add_argument('--imu-topic', default='imu_data')
    p.add_argument('--odom-topic', default='odom')
    p.add_argument('--vision-topic', default='offset_center')
    p.add_argument('--cmd-topic', default='cmd_vel')
    # 频率
    p.add_argument('--imu-hz', type=float, default=100.0)
    p.add_argument('--odom-hz', type=float, default=50.0)
    p.add_argument('--vision-hz', type=float, default=15.0)
    # 起始状态
    p.add_argument('--start-col', type=int, default=12, help='起点列（默认 12=M）')
    p.add_argument('--start-row', type=int, default=10, help='起点行（默认 10=M11）')
    p.add_argument('--heading-deg', type=float, default=-90.0,
                   help='起始航向（-90=朝北）')
    # 视觉
    p.add_argument('--face', type=int, default=2,
                   help='靶标可见面：0未知 1红/友 2蓝/敌（默认 2 蓝=可打）')
    p.add_argument('--laser-dot', type=int, default=1, help='激光红点 0/1')
    p.add_argument('--period-s', type=float, default=4.0,
                   help='靶标横向往复周期（秒），用于测提前量')
    p.add_argument('--amplitude-px', type=float, default=120.0,
                   help='靶标横向摆动幅度（像素）')
    p.add_argument('--vision-none', action='store_true', help='视觉发"无目标"')
    # 开关
    p.add_argument('--no-imu', action='store_true')
    p.add_argument('--no-odom', action='store_true')
    p.add_argument('--no-vision', action='store_true')
    p.add_argument('--drive-from-cmd', action='store_true',
                   help='订阅 /cmd_vel 推进里程（让"车"按指令动）')
    # ★ 怠速运动：默认开。原理见 integrate() 的注释 ——
    #   车端只在"检测到有效位移"时才把位置视为可信，完全静止会一直 ESTOP。
    p.add_argument('--no-idle-motion', dest='idle_motion',
                   action='store_false', default=True,
                   help='关掉怠速运动（模拟完全静止；车端会停在 ESTOP，属预期）')
    p.add_argument('--idle-v', type=float, default=0.05,
                   help='怠速前进速度 m/s（默认 0.05）')
    p.add_argument('--yaw-rate-deg-s', type=float, default=2.0,
                   help='怠速偏航角速度 °/s（默认 2.0）')
    args = p.parse_args()

    rclpy.init()
    node = FakeSensors(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
