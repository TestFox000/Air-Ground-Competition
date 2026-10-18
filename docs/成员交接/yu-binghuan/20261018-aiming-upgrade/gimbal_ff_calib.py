#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gyro_ff（车体转动前馈）**自动符号标定**节点 —— 车端一次跑完。

【为什么要它】
`dzrknn_dataprocess` 的车体转动前馈 `gyro_ff_gain_` 默认是 0（关闭），
而它的**符号** `gyro_ff_sign_` 必须现场标定：符号反了前馈会"帮倒忙"。
闭环仿真（`aim_ff_calib_sim.py`）量化过：gain=1 时符号正确把车体转动引起的
画面漂移 RMS 从 142 → 0；符号取反则**翻倍到 284**。

【怎么标】
本节点在你（前馈先保持关闭 `gyro_ff_gain: 0`）让**车体明显转动**、
同时云台锁着一个**静止目标**时，在线估计：
    rho = d(画面偏差 dx)/dt ÷ 车体角速度(deg/s)     [px/deg]
再按**已证规则**给出符号：
    gyro_ff_sign = -yaw_deg_sign * sign(rho)
（规则推导与 4 种 IMU/符号组合的闭环验证见 aim_ff_calib_sim.py，全部匹配。）

【用法（车上）】
    1) python3 gimbal_ff_calib.py            # 前馈保持关闭
    2) 云台锁一个静止目标；手动/遥控让车体左右各明显转几秒（幅度 >30° 最好）
    3) 读 [标定] 行的 rho 与"建议 gyro_ff_sign"，写入 robot.yaml：
           gyro_ff_gain: 1.0
           gyro_ff_sign: <建议值>
       重启 dzrknn_dataprocess
    4) 复核：前馈已开后再跑一次本节点，|rho| 应显著变小（趋近 0）＝标定成功

【回退】gyro_ff_gain: 0.0 即完全关闭，行为与标定前一致。
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray


class GimbalFfCalib(Node):
    def __init__(self) -> None:
        super().__init__("gimbal_ff_calib")

        self.declare_parameter("imu_topic", "/imu_data")
        self.declare_parameter("yaw_deg_sign", 1.0)        # 与 C++ 同一真源
        self.declare_parameter("min_omega_deg_s", 20.0)    # 有效激励阈值（deg/s）
        self.declare_parameter("min_samples", 30)          # 出结论所需样本数
        self.declare_parameter("min_span_deg", 30.0)       # 车体累计转角最小幅度（deg）
        self.declare_parameter("diag_period_s", 1.0)

        p = self.get_parameter
        self.imu_topic = str(p("imu_topic").value)
        self.yaw_deg_sign = float(p("yaw_deg_sign").value)
        self.min_omega = float(p("min_omega_deg_s").value)
        self.min_samples = int(p("min_samples").value)
        self.min_span = float(p("min_span_deg").value)
        self.diag_period = float(p("diag_period_s").value)

        qos1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Imu, self.imu_topic, self._cb_imu, qos1)
        self.create_subscription(Int32MultiArray, "offset_center", self._cb_vision, qos1)

        self.w_rad = 0.0
        self.w_t = None
        self.last_dx = None
        self.last_vis_t = None
        self.sw = 0.0          # Σ w*dx_dot
        self.sww = 0.0         # Σ w*w
        self.n = 0
        self.span = 0.0        # 累计 |ω|*dt（激励幅度）
        self.wmax = 0.0
        self.last_diag = None

        self.get_logger().info(
            f"标定启动: imu={self.imu_topic} yaw_deg_sign={self.yaw_deg_sign} "
            f"激励阈值={self.min_omega}deg/s 样本下限={self.min_samples}")
        self.get_logger().info(
            "步骤: ①锁定一个【静止】目标 ②让车体明显左右转动几秒 ③读 [标定] 行")

    def _cb_imu(self, msg: Imu) -> None:
        self.w_rad = float(msg.angular_velocity.z)
        self.w_t = self.get_clock().now().nanoseconds * 1e-9

    def _cb_vision(self, msg: Int32MultiArray) -> None:
        if len(msg.data) < 5:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        dx = float(msg.data[0])
        cls = int(msg.data[2])
        if cls not in (1, 2):           # 只统计靶标帧
            self.last_dx = None
            self.last_vis_t = now
            return
        if self.last_dx is not None and self.last_vis_t is not None:
            dt = now - self.last_vis_t
            if 0.005 < dt < 0.12 and self.w_t is not None and (now - self.w_t) < 0.2:
                w_deg = math.degrees(self.w_rad)
                if abs(w_deg) >= self.min_omega:
                    dx_dot = (dx - self.last_dx) / dt
                    self.sw += w_deg * dx_dot
                    self.sww += w_deg * w_deg
                    self.n += 1
                    self.span += abs(w_deg) * dt
                    self.wmax = max(self.wmax, abs(w_deg))
        self.last_dx = dx
        self.last_vis_t = now
        self._maybe_report(now)

    def _maybe_report(self, now: float) -> None:
        if self.last_diag is not None and now - self.last_diag < self.diag_period:
            return
        self.last_diag = now
        if self.sww < 1e-9:
            self.get_logger().info(f"[采样] n={self.n} 激励幅度={self.span:.0f}deg"
                                   f"（还没攒到有效转动，继续转）")
            return
        rho = self.sw / self.sww
        sign = -self.yaw_deg_sign if rho >= 0 else self.yaw_deg_sign
        ready = self.n >= self.min_samples and self.span >= self.min_span
        tag = "[标定]" if ready else "[采样]"
        self.get_logger().info(
            f"{tag} rho={rho:+.3f} px/deg  n={self.n} 激励={self.span:.0f}deg "
            f"max|w|={self.wmax:.0f}deg/s  ⇒ 建议 gyro_ff_sign={sign:+.0f}")
        if ready:
            self.get_logger().info(
                "结论: 写入 robot.yaml → gyro_ff_gain: 1.0, "
                f"gyro_ff_sign: {sign:+.0f}，重启节点；"
                "随后再跑一次本节点，|rho| 应显著变小即标定成功。")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GimbalFfCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
