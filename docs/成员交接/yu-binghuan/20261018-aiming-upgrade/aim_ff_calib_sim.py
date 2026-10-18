# -*- coding: utf-8 -*-
"""陀螺前馈(gyro_ff)自动符号标定 —— 闭环仿真验证（纯 python，无 ROS）。

目的：证明标定脚本用的符号规则 `gyro_ff_sign = -yaw_deg_sign * sign(rho)` 是对的，
并展示 gain=1、符号正确时，车体转动引起的**画面漂移**被有效抵消。

约定（与 C++ dzrknn_dataprocess 一致）：
    w      = IMU 偏航角速度(C++ 变量 imu_omega_z_)，为简化直接用 deg/s
    K_imu  = w 与"车体真实偏航角速度"之比，∈{+1,-1}（IMU 安装方向未知，就是要标的东西）
    SY     = yaw_deg_sign（度 -> 刻度 符号，已知）
    ff_x(刻度) = -w * ticks_per_deg * dt * gain * sign
    -> 折算成云台相对角增量 Δg = ff_x / (ticks_per_deg * SY) = -w*dt*gain*sign/SY
    完全补偿所需 Δg = -ψ̇*dt = -w*dt/K_imu  ⇒ 正确 sign = SY / K_imu
    开环(ff off)观测：目标静止时 dx_dot ≈ -ψ̇*px_per_deg ⇒
       rho = dx_dot / w = -px_per_deg / K_imu ⇒ sign(rho) = -sign(K_imu)
    两式消去 K_imu ⇒ 正确 sign = -SY * sign(rho)   ← 待验证的规则
"""
from __future__ import annotations

import math

PX_PER_DEG = 10.7
TICKS_PER_DEG = 11.378
DT = 1.0 / 30.0
AMP_DEG = 30.0        # 车体偏航幅度
FREQ = 0.5            # 偏航频率 Hz
KP = 4.0              # 视觉伺服居中增益(deg/s per deg error)
N = 600


def simulate(K_imu: float, SY: float, gain: float, sign: float):
    """返回逐帧 (w, dx, dx_dot)。目标在世界角 0 处静止。"""
    psi, g = 0.0, 0.0
    out = []
    prev_dx = None
    for k in range(N):
        t = k * DT
        psi_dot = AMP_DEG * 2 * math.pi * FREQ * math.cos(2 * math.pi * FREQ * t)  # deg/s
        w = K_imu * psi_dot
        dx = (0.0 - (psi + g)) * PX_PER_DEG
        dx_dot = (dx - prev_dx) / DT if prev_dx is not None else 0.0
        prev_dx = dx
        # 伺服：向目标居中(把 psi+g 拉回 0)  + 陀螺前馈
        g += (KP * (-(psi + g)) * DT) + (-w * DT * gain * sign / SY)
        psi += psi_dot * DT
        out.append((w, dx, dx_dot))
    return out


def rho_of(data):
    """最小二乘斜率 rho = dx_dot / w（px per deg）。"""
    num = sum(w * d for w, _, d in data)
    den = sum(w * w for w, _, _ in data)
    return num / den if den > 1e-9 else 0.0


def rms_dx(data):
    return math.sqrt(sum(dx * dx for _, dx, _ in data) / len(data))


if __name__ == "__main__":
    print(f"参数: px/deg={PX_PER_DEG} AMP={AMP_DEG}° f={FREQ}Hz gain=1\n")
    print(f"{'K_imu':>5} {'SY':>4} {'rho(开环)':>10} {'sign(测)=−SY·sgn(rho)':>22} "
          f"{'sign(真值)=SY/K':>16} {'匹配':>5}  {'RMS_dx off':>10} {'RMS_dx on':>10}")
    for K_imu in (1.0, -1.0):
        for SY in (1.0, -1.0):
            open_loop = simulate(K_imu, SY, gain=0.0, sign=1.0)
            rho = rho_of(open_loop)
            sign_meas = -SY * (1.0 if rho >= 0 else -1.0)
            sign_true = SY / K_imu
            # 开环 vs 闭环(gain=1, 正确符号)
            on = simulate(K_imu, SY, gain=1.0, sign=sign_true)
            match = "是" if abs(sign_meas - sign_true) < 1e-9 else "否"
            print(f"{K_imu:5.0f} {SY:4.0f} {rho:10.4f} {sign_meas:22.0f} "
                  f"{sign_true:16.0f} {match:>5}  {rms_dx(open_loop):10.2f} {rms_dx(on):10.2f}")

    print("\n--- 反例：gain=1 但符号取反（错误符号）---")
    K, SY = 1.0, 1.0
    off = simulate(K, SY, 0.0, 1.0)
    right = simulate(K, SY, 1.0, SY / K)
    wrong = simulate(K, SY, 1.0, -SY / K)
    print(f"RMS_dx  前馈关={rms_dx(off):.2f}  符号对={rms_dx(right):.2f}  符号反={rms_dx(wrong):.2f}")
