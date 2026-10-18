# -*- coding: utf-8 -*-
"""舵机-曲率模型：前馈映射与单侧极限。

为什么要建模"左右不对称"：发学员版 2026-09-04 实测（见
reference/car_src_fxxy_20261017/car_localization_rect/.../path_executor.py
TURN_ARC_SCALE 注释）：同一满舵指令下，左转弧 R≈0.15 m（κ≈6.6），
右转弧 R≈0.44 m（κ≈2.3）——舵机中位偏置 + 连杆不对称所致。
对称的 atan(L·κ) 前馈在右转时严重欠打角，这正是原版被迫用
"开环满舵弧 + yaw 停止"过弯的根因之一。

三种档案：
- ideal   ：对称自行车模型（默认，与 SIL ChassisSim 标定一致）；
- fxxy    ：发学员实测左右不对称分段线性档案（保守备选）；
- custom  ：tools/dznav_calib.py 现场标定产物（线性表）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class ServoModel:
    steer_limit_deg: float = 30.0     # 控制层使用的打角上限（≤物理 35°）
    wheelbase: float = 0.157          # m
    mode: str = "ideal"               # ideal | fxxy | custom
    center_deg: float = 0.0           # 零曲率打角（右偏为负）
    # fxxy 档案：满舵幅值曲率（1/m，恒正；右转 κ<0 由符号约定给出）
    kappa_left_full: float = 6.6
    kappa_right_full: float = 2.3
    # custom 档案：[(deg, kappa)] 单调序列，含 (0 附近) 零点
    table: list = field(default_factory=list)

    # ---------------------------------------------------------------- 查询
    def kappa_for_command(self, steer_deg: float) -> float:
        """打角(度) -> 曲率(1/m)。左转(正舵角) κ>0，右转 κ<0。"""
        s = max(-self.steer_limit_deg, min(self.steer_limit_deg, steer_deg))
        if self.mode == "ideal":
            return math.tan(math.radians(s)) / self.wheelbase
        if self.mode == "custom" and len(self.table) >= 2:
            xs = [p[0] for p in self.table]
            ks = [p[1] for p in self.table]
            return self._interp(xs, ks, s)
        # fxxy：零点在 center_deg 的分段线性（右转为负 κ）
        if s >= self.center_deg:
            span = max(1e-6, self.steer_limit_deg - self.center_deg)
            return (s - self.center_deg) / span * self.kappa_left_full
        span = max(1e-6, self.center_deg + self.steer_limit_deg)
        return (s - self.center_deg) / span * self.kappa_right_full

    def command_for_kappa(self, kappa: float) -> float:
        """曲率(1/m) -> 打角(度)。κ>0 左转。"""
        if self.mode == "ideal":
            return math.degrees(math.atan(self.wheelbase * kappa))
        if self.mode == "custom" and len(self.table) >= 2:
            xs = [p[1] for p in self.table]   # κ 升序
            ks = [p[0] for p in self.table]
            if xs[0] > xs[-1]:                # 表按 deg 给出时翻转
                xs, ks = xs[::-1], ks[::-1]
            return self._interp(xs, ks, kappa)
        # fxxy 分段线性逆映射
        if kappa >= 0.0:
            span = max(1e-6, self.steer_limit_deg - self.center_deg)
            return self.center_deg + kappa / self.kappa_left_full * span
        span = max(1e-6, self.center_deg + self.steer_limit_deg)
        return self.center_deg + kappa / self.kappa_right_full * span

    def kappa_max(self, turn_sign: float) -> float:
        """单侧可达曲率幅值。turn_sign: +1 左转(CCW) / -1 右转(CW)。"""
        if self.mode == "ideal":
            return math.tan(math.radians(self.steer_limit_deg)) / self.wheelbase
        if turn_sign >= 0:
            return max(0.05, self.kappa_left_full)
        return max(0.05, self.kappa_right_full)

    # ---------------------------------------------------------------- 工具
    @staticmethod
    def _interp(xs: list, ks: list, x: float) -> float:
        if x <= xs[0]:
            x0, x1, k0, k1 = xs[0], xs[1], ks[0], ks[1]
        elif x >= xs[-1]:
            n = len(xs)
            x0, x1, k0, k1 = xs[n - 2], xs[n - 1], ks[n - 2], ks[n - 1]
        else:
            i = 0
            while i < len(xs) - 2 and x > xs[i + 1]:
                i += 1
            x0, x1, k0, k1 = xs[i], xs[i + 1], ks[i], ks[i + 1]
        if x1 == x0:
            return k0
        return k0 + (k1 - k0) * (x - x0) / (x1 - x0)
