# -*- coding: utf-8 -*-
"""横切误差(cross-track error)转向修正控制器 —— CTE-PID。

**吸收来源**：Gitee 开源项目 Line_Follower（长江大学极客班，MIT 许可）的自适应 PID。
原实现见 `config/pid.cfg` 与 `src/line_follower_node.cpp:522-620`，其 6 重
抗饱和/抗抖动机制我们原车载「纯追踪 + CTE 纯P」没有，此处逐条移植：

    1. 输入滤波(EMA, filter_coefficient=0.7)
    2. 死区(deadzone)
    3. 积分分离(error_threshold 内才积分)
    4. 积分限幅(integral_limit)
    5. 微分先行(先滤波后微分，抑制噪声)
    6. 软限幅(soft_limit)

**量纲与定标**：原实现以像素为单位；本车 cte 单位是米(m)、输出是舵角(度)。比例增益
`kp=115.0` **对齐改动前的 `CTE_GAIN=115.0`**。Ki/Kd **不能照搬像素尺度**，而是用
`cte_pid_sim.py`（同一实车参数 v=0.32/L=0.17/20Hz + 恒定左偏扰动）离线扫掠定标：
纯P 在该模型下是【无阻尼振荡】，加 I/D 后超调与 RMS 显著下降（见该脚本输出）。

**符号约定**：`error(cte) > 0` 表示车在航线**左侧** → 需要**向右**打舵 → 返回**负**的 steer 增量。
"""
from __future__ import annotations


class CtePidController:
    """CTE 转向修正：带 6 重抗饱和保护的 P-I-D。"""

    def __init__(
        self,
        kp: float = 115.0,               # deg/m     比例（对齐改动前 CTE_GAIN）
        ki: float = 50.0,                # deg/(m·s) 积分（扫掠定标：消恒定左偏的稳态误差）
        kd: float = 100.0,               # deg/(m/s) 微分先行（扫掠定标：给无阻尼环路加阻尼）
        deadzone: float = 0.005,         # m    死区（5mm）
        integral_limit: float = 0.05,    # m·s  积分限幅（≈最多积出 2.5° 抵消机械偏置）
        error_threshold: float = 0.050,  # m    积分分离阈值（超出则不积分，防大误差期间饱和）
        filter_coefficient: float = 0.7, #      输入滤波系数（同 Line_Follower 0.7）
        soft_limit: float = 30.0,        # deg  软限幅（= MAX_STEER_DEG）
    ) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.deadzone = float(deadzone)
        self.integral_limit = float(integral_limit)
        self.error_threshold = float(error_threshold)
        self.filter_coefficient = float(filter_coefficient)
        self.soft_limit = float(soft_limit)
        self.reset()

    def reset(self) -> None:
        """复位积分/微分/滤波状态。进入新航段时应调用，避免旧段偏置带入。"""
        self._filt = 0.0        # 滤波后的误差
        self._last_e = 0.0      # 上一拍（滤波后）误差，用于微分先行
        self._integral = 0.0

    def update(self, error: float, dt: float) -> float:
        """输入横切误差(m) 与控制周期(s)，返回转向角增量(度)。

        返回值为「需要叠加到纯追踪舵角上的修正量」，符号见模块 docstring。
        """
        # 1) 输入滤波（EMA）
        self._filt = (self.filter_coefficient * float(error)
                      + (1.0 - self.filter_coefficient) * self._filt)
        e = self._filt

        # 2) 死区（小于阈值视为 0，避免来回抖动）
        if abs(e) < self.deadzone:
            e = 0.0
        else:
            e = e - self.deadzone if e > 0 else e + self.deadzone

        # 3) 积分分离 + 4) 积分限幅
        if abs(e) < self.error_threshold:
            self._integral += e * dt
            self._integral = max(-self.integral_limit,
                                 min(self.integral_limit, self._integral))
        else:
            self._integral = 0.0

        # 5) 微分先行：对「滤波后」误差求导（抗噪）
        deriv = (e - self._last_e) / dt if dt > 0 else 0.0
        self._last_e = e

        pid = self.kp * e + self.ki * self._integral + self.kd * deriv

        # 6) 软限幅
        pid = max(-self.soft_limit, min(self.soft_limit, pid))

        # 符号：error>0（车在左侧）→ 向右打舵 → 负增量
        return -pid
