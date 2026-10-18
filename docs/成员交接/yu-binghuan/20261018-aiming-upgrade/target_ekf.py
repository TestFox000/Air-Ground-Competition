# -*- coding: utf-8 -*-
"""目标跟踪 EKF（匀速 CV 模型）—— 瞄准链路状态估计器。

**吸收来源**：RoboMaster 自瞄生态的标准做法。`rm_gimbal_controllers`
（ROS，BSD，github.com/rm-controls/rm_controllers）与 `rm_auto_aim`（含 armor_tracker）
都用卡尔曼/EKF 做目标状态估计 + 预测，而不是长窗最小二乘。

**为什么换掉 target_motion_predictor 的 0.6s 长窗 LS**：
  · 长窗 0.6s 对匀速段的【等效滞后 ≈ 0.3s】，而链路延迟才 0.26s —— 估计器本身就吃掉
    掉全部延迟预算，超前量永远算不准（实测稳态滞后 22~44px，开火框才 3.5px）；
  · 丢靶 >0.15s 时 LS 整段作废、状态丢失；EKF 丢帧只"预测不更新"，状态连续不丢；
  · LS 对噪声的抑制与滞后是硬绑定的（窗口越长越平滑越滞后）；EKF 用过程/观测噪声
    协方差把两者解耦 —— 可以既平滑又低滞后。

状态 x = [θ(deg), ω(deg/s)]；观测 z = θ（由 云台角 + 画面偏差角 合成）。
"""
from __future__ import annotations


class TargetEkf:
    """匀速(CV)目标 EKF。角度以度为单位，角速度以度/秒为单位。"""

    def __init__(self, q_theta: float = 0.1, q_omega: float = 200.0,
                 r_theta: float = 0.30) -> None:
        self.q_theta = float(q_theta)   # 角度过程噪声
        self.q_omega = float(q_omega)   # 角速度过程噪声（决定能否快速跟上折返/起步）
        self.r_theta = float(r_theta)   # 观测噪声方差（度²）
        # 默认值由 aim_ekf_sim.py 离线扫掠定标：在合成实测运动 + 噪声 + 丢帧下，
        # 该组参数在【全程/匀速段/丢帧期】三个口径上都优于 0.6s 长窗 LS。
        self.x = None                   # [theta, omega]
        self.P = None
        self.inited = False

    def reset(self, theta: float) -> None:
        self.x = [float(theta), 0.0]
        # 初值不确定：角度按观测噪声，角速度给大方差（尽快靠观测收敛）
        self.P = [[self.r_theta, 0.0], [0.0, 100.0]]
        self.inited = True

    def predict(self, dt: float) -> None:
        if not self.inited or dt <= 0.0:
            return
        th, w = self.x
        self.x = [th + w * dt, w]                     # F = [[1, dt], [0, 1]]
        P = self.P
        p00 = P[0][0] + dt * (P[0][1] + P[1][0]) + dt * dt * P[1][1] + self.q_theta * dt
        p01 = P[0][1] + dt * P[1][1]
        p10 = P[1][0] + dt * P[1][1]
        p11 = P[1][1] + self.q_omega * dt
        self.P = [[p00, p01], [p10, p11]]

    def update(self, z: float) -> None:
        if not self.inited:
            self.reset(z)
            return
        P = self.P
        s = P[0][0] + self.r_theta
        k0 = P[0][0] / s                                # H = [1, 0]
        k1 = P[1][0] / s
        y = float(z) - self.x[0]
        self.x = [self.x[0] + k0 * y, self.x[1] + k1 * y]
        p00 = (1.0 - k0) * P[0][0]
        p01 = (1.0 - k0) * P[0][1]
        p10 = P[1][0] - k1 * P[0][0]
        p11 = P[1][1] - k1 * P[0][1]
        self.P = [[p00, p01], [p10, p11]]

    @property
    def theta(self):
        return self.x[0] if self.inited else None

    @property
    def omega(self):
        return self.x[1] if self.inited else None

    def predict_ahead(self, t: float):
        """外推 t 秒后的角度（超前瞄准量 = 该值 − 当前观测角）。"""
        if not self.inited:
            return None
        return self.x[0] + self.x[1] * t
