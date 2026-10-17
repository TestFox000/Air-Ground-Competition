# -*- coding: utf-8 -*-
"""④ 可动双面板靶射击模型（本包建模；dzbrain 仿真器不建模射击/驻留——局限如实标注）。

## 物理口径（队长 2026-10 最新参数）

- 靶 = 一块可动双面板（蓝/红各一面）+ 水平轨道两格（0.7 m），面板沿轨道 0.1 m/s 来回；
- 朝向（蓝面法线 φ）每场随机；**打红面 = 扣分事件**；
- 蓝面朝向需无人机判读（YOLO 口径），判读误差 ε 是受限臂旋钮；
- 射击停站点须在蓝面可见扇区（法线 ±45°；敏感度 ±30/±45/±60）；
- 移动靶提前量 0.35 s（``aim.lead_time_s`` 同源）→ 瞄准偏移 = 0.1 × 0.35 = 3.5 cm，
  面板半长 17.5 cm ⇒ 提前量在本几何下不绑定命中（结论进报告）；
- "贴墙不可打"：蓝面朝南（φ=180°）时场内任何停站点都落在红面侧/掠射 → 记为
  **已知不可解场景**（0 命中 → 驻留吃满 30 s 硬死线）。

## 命中判定（确定性）

- 面板中心 p(t) 沿轨道振荡（三角波，行程 = 轨道长 − 面板长 = 0.35 m，周期 7 s）。
- 停站点 s：视角 θ = atan2(面板→s)；|θ−φ| ≤ sector ⇒ 蓝面可见（命中有效性 ✓）；
  |θ−(φ+180)| ≤ sector ⇒ 红面侧（命中 = **红面命中扣分**）；否则掠射（不命中）。
- 跟踪正常 ⇒ 命中确定（理想感知）；本模型不建模弹道散布（局限标注）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .geo import block_to_meters
from .scenarios import TargetParams


def _angle_diff(a_deg: float, b_deg: float) -> float:
    d = (a_deg - b_deg) % 360.0
    return d if d <= 180.0 else d - 360.0


def panel_track_center_m(target: TargetParams) -> tuple[float, float]:
    """轨道中点（米）。"""
    cells = [block_to_meters(c) for c in target.track_cells]
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    return sum(xs) / len(xs), sum(ys) / len(ys)


def panel_center(t: float, target: TargetParams) -> tuple[float, float]:
    """t 时刻面板中心（沿轨道三角波振荡；方向 = 轨道轴向）。

    轨道两格总长 = 格心距 + 一格宽 = 0.7 m；面板 0.35 m ⇒ 行程 0.35 m（±0.175）。
    ★ t28：轨道轴向按两格连心线方向推（横向 B11-C11 = 列轴；纵向 = 行轴），
    不硬编码 y 轴 —— 支持 ③ 靶格位更改后的任意相邻两格。
    """
    cx, cy = panel_track_center_m(target)
    (x0, y0), (x1, y1) = (block_to_meters(c) for c in target.track_cells)
    dx, dy = x1 - x0, y1 - y0
    span = max(abs(dx), abs(dy))
    track_len = span + 0.35
    amp = max(0.0, (track_len - target.panel_len_m) / 2.0)   # 半行程：off ∈ [−amp, +amp]
    period = 4.0 * amp / target.speed_mps if target.speed_mps > 0 else float("inf")
    if period <= 0 or amp <= 0 or span <= 1e-12:
        return cx, cy
    phase = (t % period) / period          # [0,1)
    # 三角波：0→amp→0→−amp→0
    if phase < 0.25:
        off = 4.0 * phase * amp
    elif phase < 0.75:
        off = (2.0 - 4.0 * phase) * amp
    else:
        off = (4.0 * phase - 4.0) * amp
    # 轨道轴：横向对（同行相邻列）沿 +y，纵向对沿 +x —— 与 t11 原式（cy + off）逐位同源
    ux, uy = (0.0, 1.0) if abs(dy) >= abs(dx) else (1.0, 0.0)
    return cx + off * ux, cy + off * uy


@dataclass
class ShootOutcome:
    stop: str                  # 选定的停站点（无解时仍返回计划格 C10，标记 sector_fail）
    sector_fail: bool = False  # 判读扇区内无候选停站点（0 命中、驻留 30 s）
    blue_hits: int = 0
    red_hits: int = 0          # 红面命中 = 扣分事件数
    misses: int = 0
    dwell_s: float = 30.0      # 驻留：命中情景 settle+acquire+shots；0 命中 = 30 s 硬死线
    judged_phi: float = 0.0    # 无人机判读的蓝面法线（含误差）
    chosen_theta: float = 0.0  # 选定停站点的视角
    true_phi: float = 0.0

    @property
    def penalties(self) -> int:
        return self.red_hits

    @property
    def valid_hits(self) -> int:
        return self.blue_hits

    @property
    def score(self) -> int:
        """② 打蓝面得分、打红面扣分：净分 = 蓝 − 红。"""
        return self.blue_hits - self.red_hits


def pick_stop(target: TargetParams, judged_phi_deg: float,
              on_route: set[str] | None = None) -> tuple[str | None, float | None]:
    """按判读朝向在候选停站点里选 |θ−φ̂| ≤ sector 且角度最小的格；无候选 → (None, None)。"""
    best: tuple[str, float] | None = None
    for cell in target.stop_candidates:
        if on_route is not None and cell not in on_route:
            continue
        px, py = panel_track_center_m(target)
        sx, sy = block_to_meters(cell)
        theta = math.degrees(math.atan2(sy - py, sx - px))
        err = abs(_angle_diff(theta, judged_phi_deg))
        if err <= target.sector_deg + 1e-9:
            if best is None or err < best[1]:
                best = (cell, err)
    if best is None:
        return None, None
    return best[0], best[1]


def evaluate_shooting(target: TargetParams, planned_stop: str,
                      on_route: set[str] | None = None, judge_available: bool = True,
                      anchor_t: float = 0.0) -> ShootOutcome:
    """一次射击停站全程（确定性，② 命中时车→靶连线落蓝面法线扇区）。

    :param target: 靶模型（含真值朝向与判读误差）。
    :param planned_stop: 路线上的原计划停站点（无判读/无解时退回它——现车行为）。
    :param on_route: 路线经过的格集合（停站点候选须在路线上，否则视为不可选）。
    :param judge_available: 是否有无人机判读。False（无无人机）= 按计划格盲打；
        True 且扇区内无停站点 = 判读知道蓝面不可达 → **不开火**（0 发、驻留 30 s）。
    :param anchor_t: 射击时刻锚点（joint 时间）= 车到达停站点 + 等待。第 i 发在
        ``anchor_t + settle(0.4) + acquire(2.0) + i×cooldown`` 击发，逐发用**该时刻**
        面板位置算车→靶连线角（面板在动 → 命中有效性逐发不同——② 的"命中时"口径）。
    """
    judged = target.orientation_deg + target.judge_error_deg
    out = ShootOutcome(stop=planned_stop, judged_phi=judged, true_phi=target.orientation_deg)
    chosen = None
    if judge_available:
        chosen, _ = pick_stop(target, judged, on_route)
    if chosen is None:
        chosen = planned_stop
        if judge_available:
            out.sector_fail = True
            out.dwell_s = 30.0            # 蓝面贴墙/扇区无解：0 发，驻留吃满 30 s 硬死线
            return out
    out.stop = chosen
    px0, py0 = panel_track_center_m(target)
    sx, sy = block_to_meters(chosen)
    out.chosen_theta = math.degrees(math.atan2(sy - py0, sx - px0))
    sector = target.sector_deg

    for i in range(target.shots):
        t_shot = anchor_t + 0.4 + 2.0 + i * target.shot_cooldown_s
        px, py = panel_center(t_shot, target)
        theta = math.degrees(math.atan2(sy - py, sx - px))
        if abs(_angle_diff(theta, target.orientation_deg)) <= sector + 1e-9:
            out.blue_hits += 1
        elif abs(_angle_diff(theta, target.orientation_deg + 180.0)) <= sector + 1e-9:
            out.red_hits += 1
        else:
            out.misses += 1
    out.dwell_s = (30.0 if out.blue_hits + out.red_hits == 0
                   else 0.4 + 2.0 + target.shots * target.shot_cooldown_s)
    return out
