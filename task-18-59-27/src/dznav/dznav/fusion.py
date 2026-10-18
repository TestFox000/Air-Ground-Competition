# -*- coding: utf-8 -*-
"""定位融合：编码器距离 + IMU 航向的航位推算，锚点软校准 + 偏差自学习。

继承发学员 serial_reader 的核心洞察（2026-09-06 实测结论）：
dzactuator 的 /odom 位姿增量所在坐标系旋转错误（同一次右转后两趟跑
漂向相反侧、单调外旋），只有"相邻采样位移距离"是旋转不变量、可用。
因此位置积分只取步长，方向完全来自 IMU 航向 —— 这一层被完整保留。

拔高点（结构性根治画圆机理）：原版 reset 语义分裂在两个节点 ——
serial_reader 沿 yaw_rel+π/2 积分、path_executor 把信念航向 snap 到
断言值，两者不同步 ⇒ 信念里"车朝哪"与"车往哪挪"恒差一个偏置
（docs/sim_car.py 画圆机理注释、2026-10-01 事故）。dznav 融合与控制
同节点同状态：积分方向就是滤波器自己的航向态，reset/锚点同时校正
位置与航向 —— 双重误差在结构上不存在。

校准学习（锚点残差观测器）：
- 途经点/出弯锚点不再硬吸附（belief 说谎），而是按 blend 软拉回；
- 残差沿航向分量 → 编码器尺度因子 enc_scale（慢增益，±3% 钳位）；
- 残差横向分量 → 航向锚 yaw_anchor（慢增益）；
- 静止时航向保持观测：IMU 陀螺零漂导致的缓慢转头被自动抵消。
一趟路线跑下来系统偏差被估计掉，锚点修正量逐步缩小（自校准）。
"""

from __future__ import annotations

import math

from .contract import BLOCK, wrap_pi, RAW_YAW_TO_MAP_YAW_OFFSET

MIN_STEP_M = 0.0005      # 静止抖动滤除（对齐原版）
STATIONARY_S = 0.5       # s 无位移判定静止
K_HOLD = 0.04            # 静止航向保持增益（每 IMU 拍）
K_SCALE = 0.10           # 编码器尺度学习增益
K_YAW = 0.05             # 航向锚学习增益
SCALE_CLAMP = 0.03       # 尺度学习钳位 ±3%


class Localizer:
    def __init__(self, start_block: tuple = (1.5, 0.5)):
        # 信念位姿（米，地图系）
        self.x = start_block[0] * BLOCK
        self.y = start_block[1] * BLOCK
        # th = yaw_rel + yaw_anchor；开机默认朝 Y+（与原版一致）
        self.yaw_anchor = RAW_YAW_TO_MAP_YAW_OFFSET
        self.th = 0.0
        # IMU
        self.yaw_rel = 0.0
        self.imu_seen = False
        # 编码器（raw 系任意旋转，仅取距离）
        self._prev_raw = None
        self.raw_travel = 0.0        # 累计里程（ARC 行程保险复用）
        # 静止保持
        self._last_move_s = None
        self._th_hold_ref = None
        self._clock = 0.0
        # 锚点学习
        self._anchor_base_travel = 0.0
        self.enc_scale = 1.0

    # ---------------------------------------------------------------- 输入
    def on_imu(self, yaw_rel: float, t: float | None = None):
        prev_th = self.yaw_rel + self.yaw_anchor if self.imu_seen else None
        self.yaw_rel = yaw_rel
        self.imu_seen = True
        self.th = wrap_pi(yaw_rel + self.yaw_anchor)
        # 静止航向保持：IMU 慢漂被锚回参考
        if (prev_th is not None and self._last_move_s is not None
                and self._clock - self._last_move_s > STATIONARY_S):
            if self._th_hold_ref is None:
                self._th_hold_ref = prev_th
            self.yaw_anchor = wrap_pi(
                self.yaw_anchor
                + K_HOLD * wrap_pi(self._th_hold_ref - self.th))
            self.th = wrap_pi(self.yaw_rel + self.yaw_anchor)

    def on_odom_raw(self, x_m: float, y_m: float):
        if self._prev_raw is None:
            self._prev_raw = (x_m, y_m)
            return
        ds = math.hypot(x_m - self._prev_raw[0], y_m - self._prev_raw[1])
        self._prev_raw = (x_m, y_m)
        if ds < MIN_STEP_M:
            return
        self._last_move_s = self._clock
        self._th_hold_ref = None
        self.raw_travel += ds
        self.x += ds * self.enc_scale * math.cos(self.th)
        self.y += ds * self.enc_scale * math.sin(self.th)

    def tick(self, t: float):
        """外部时钟注入（executor 每控制拍调用）。"""
        self._clock = t

    # ---------------------------------------------------------------- 锚点
    def anchor(self, x: float, y: float, th: float, blend: float = 0.5,
               blend_th: float | None = None, learn: bool = True):
        """软锚点：位置按 blend、航向按 blend_th 拉回，残差喂学习器。

        blend_th 默认取 blend。航向增益通常应小于位置增益：锚定瞬间的
        航向残差混有弯道跟踪滞后（控制误差，非传感器偏差），增益过大
        会把滞后"学习"进信念航向、随后沿错误方向航位推算（SIL 实测
        blend_th=0.6 时 5 弯累积 0.4m 信念-真值偏差）。
        """
        if not self.imu_seen:
            return
        if blend_th is None:
            blend_th = blend
        ex, ey = x - self.x, y - self.y
        e_th = wrap_pi(th - self.th)
        # 学习：锚点间里程足够长才可信（抑制锚点自身噪声）
        traveled = self.raw_travel - self._anchor_base_travel
        if learn and traveled > 0.3:
            along = ex * math.cos(self.th) + ey * math.sin(self.th)
            cross = -ex * math.sin(self.th) + ey * math.cos(self.th)
            if abs(along) < 0.15 and abs(cross) < 0.15:
                self.enc_scale += K_SCALE * (along / traveled)
                self.enc_scale = max(1.0 - SCALE_CLAMP,
                                     min(1.0 + SCALE_CLAMP, self.enc_scale))
                self.yaw_anchor = wrap_pi(
                    self.yaw_anchor + K_YAW * cross / max(traveled, 0.1))
            self._anchor_base_travel = self.raw_travel
        # 拉回
        self.x += blend * ex
        self.y += blend * ey
        self.yaw_anchor = wrap_pi(self.yaw_anchor + blend_th * e_th)
        self.th = wrap_pi(self.yaw_rel + self.yaw_anchor)

    def reset(self, x: float, y: float, th: float):
        """硬重锚（reset:BLOCK,HEAD 语义：车就在这、就朝这）。"""
        self.x, self.y = x, y
        if self.imu_seen:
            self.yaw_anchor = wrap_pi(th - self.yaw_rel)
        else:
            self.yaw_anchor = th      # IMU 未上流：等首帧后 on_imu 校正
        self.th = wrap_pi(self.yaw_rel + self.yaw_anchor)
        self._anchor_base_travel = self.raw_travel
        self._th_hold_ref = None

    # ---------------------------------------------------------------- 输出
    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.th

    def yaw_out(self) -> float:
        """对外发布 yaw：使下游 yaw + π/2 == 内部地图航向（契约保持）。"""
        return wrap_pi(self.th - RAW_YAW_TO_MAP_YAW_OFFSET)

    def make_odom(self, odom_factory):
        """按 /odom/localization 契约产出消息（位置=格，四元数=yaw_out）。"""
        msg = odom_factory()
        msg.pose.pose.position.x = self.x / BLOCK
        msg.pose.pose.position.y = self.y / BLOCK
        yaw = self.yaw_out()
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return msg
