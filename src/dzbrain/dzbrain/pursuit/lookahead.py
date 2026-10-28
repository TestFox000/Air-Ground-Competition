# -*- coding: utf-8 -*-
"""前视点（lookahead point）—— 纯追踪的"瞄准目标"。

旧实现（`path_executor.find_lookahead_point` 1657–1732）的三条语义全部保留：
1. 从投影点沿路径按弧长 `ld` 前进，**可跨段**；路径不足前视距离时直接瞄准终点；
2. **转角顶点钳制**：本段与下一段夹角 > 阈值且距顶点 < `vertex_clamp_m` → 直接把前视点
   钳到顶点，防最后一刻切角（旧 20° / 0.04 m，硬编码；现在两个数都在 Config 里）；
3. 接近末点时钳到最后一点，**绝不越界**。

★ 在此基础上加一条**退化保护**（旧实现同样有这个洞）：车正好站在顶点上
  （`dist_to_vertex == 0`，Localization 把位姿吸附到格心时必然出现）时**不做顶点钳制**，
  而是把 `s_target` 抬到顶点之后 —— 否则前视点 = 车自己 ⇒ 弦长 0 ⇒ 舵角恒 0、车直行漂出
  走廊（现场 F10 实测，详见函数内注释）。

★ 前视点只按**弧长**推进（不是直线插值）—— 这是"贴着路线走"的关键：
  若按直线取点，过弯时会把目标点切到路线内侧，等于主动抄近道。
⚠️ 转角钳制只看**本段末端的顶点**（与旧实现一致）。若需要"提前看两个顶点"，
   属于策略升级（会影响通过性），不在本次移植范围。
"""

from __future__ import annotations

from dataclasses import dataclass

from ..core.config import Config
from .path import Path, Projection
from .trajectory import Trajectory
from ..core.config import lookahead_distance

__all__ = ["LookaheadPoint", "find_lookahead_point"]


def trajectory_lookahead_distance(path: Trajectory, s: float, cfg: Config,
                                  stop_s: float | None = None) -> float:
    """Cruise-derived arc preview, shortened by reference curvature only."""
    base = lookahead_distance(cfg.get("pursuit.cruise_speed_mps"), cfg)
    end = path.total_length if stop_s is None else min(path.total_length, stop_s)
    peak = max(abs(path.sample(min(end, s + d)).curvature)
               for d in (0.0, base * 0.5, base))
    return max(cfg.get("pursuit.lookahead_floor_m"), base / (1.0 + base * peak))

#: 已经站在顶点上时，把前视目标抬到**顶点之后**这幺多（米）。
#: 与 `path._VERTEX_STEP_M` 同值同理由：必须 > 浮点噪声，又必须 ≪ 段长 0.35 m
#: （否则会跳过下一段本身）。
_VERTEX_STEP_M = 1e-3
#: `dist_to_vertex` 低于它就认定"车正站在顶点上"（= 投影点与车重合）。
_DEGENERATE_M = 1e-6


@dataclass(frozen=True)
class LookaheadPoint:
    """前视点。`clamped_to_vertex=True` 表示它被**转角钳制**到了顶点（不是自由推进点）。"""

    x: float
    y: float
    seg_index: int
    s: float
    clamped_to_vertex: bool


def find_lookahead_point(path: Path, proj: Projection, ld: float, cfg: Config) -> LookaheadPoint:
    """求前视点：`proj` 沿路径前进 `ld`（米）处的点。

    ★ `ld` 由调用方决定（通常 `core.config.lookahead_distance(巡航速度, cfg)`），
      本函数不做速度相关计算 —— 保持"几何归几何"。
    ★ 转角钳制时返回的 `seg_index` 是**正在行驶的段**（`proj.seg_index`），
      此时 `s` 落在两段的公共顶点上（既等于本段末端，也等于下一段起点）。
    ★ 退化保护触发时返回的 `seg_index` 是**顶点之后那一段**（`path.point_at` 给谁算谁），
      `clamped_to_vertex=False`、`s` 严格大于 `proj.s` —— 保证前视点不落在车身上。
    """
    if isinstance(path, Trajectory):
        # No grid-vertex clamp or degenerate-vertex exception exists on the
        # continuous path. A cusp is a required stop, so preview never crosses
        # into its opposite travel direction before that event is released.
        end = min((s for s in path.cusp_s if s > proj.s + 1e-9),
                  default=path.total_length)
        target = path.sample(min(end, proj.s + max(0.0, ld)))
        return LookaheadPoint(target.x, target.y, target.seg_index, target.s, False)
    total = path.total_length
    seg_index = proj.seg_index

    # ---- ② 转角顶点钳制（优先于前进：宁可直接瞄顶点，也不切角） ----
    clamp_m = cfg.get("pursuit.vertex_clamp_m")
    corner_thr = cfg.get("pursuit.lookahead_corner_angle_deg")
    # ⚠️⚠️ 退化保护：车**正好站在顶点上**（`dist_to_vertex == 0`）时，顶点钳制会把前视点
    #    钳到顶点 = **车自己所在的位置** ⇒ 弦长 = 0 ⇒ `compute_command` 的方位角无意义、
    #    舵角退化为 +0.0°，车保持直行漂出走廊（实闭环 bench 现场 F10 实测：2.5 s 内
    #    投影 s 冻在 3.500 不动、dist 长到 0.300 m、舵角恒 +0.0°、v 恒 +0.120，
    #    12 s 后触发「拐角绕圈兜底」强制跳点）。
    #    此时的正确语义是：**顶点已到，从顶点之后开始前视**（沿下一段推进），
    #    而不是把目标放在车身上。`project()` 的顶点前瞻重投影已把 seg_index 推到下一段，
    #    这里是二道防线 —— 直接用手工构造的 `Projection` 调用本函数时同样不许塌缩。
    if (proj.dist_to_vertex <= _DEGENERATE_M
            and seg_index + 1 < path.segment_count and ld > 0.0):
        s_jump = path.s_at_waypoint(seg_index + 1) + _VERTEX_STEP_M
        s_target = proj.s + ld
        if s_target < s_jump:
            s_target = s_jump
        if s_target > total:
            s_target = total
        x, y, si = path.point_at(s_target)
        return LookaheadPoint(x=x, y=y, seg_index=si, s=s_target,
                              clamped_to_vertex=False)
    if clamp_m > 0.0 and path.corner_angle_deg(seg_index) > corner_thr:
        if proj.dist_to_vertex < clamp_m:
            s_vertex = path.s_at_waypoint(seg_index + 1)
            vx, vy, _ = path.point_at(s_vertex)
            return LookaheadPoint(x=vx, y=vy, seg_index=seg_index,
                                  s=s_vertex, clamped_to_vertex=True)

    # ---- ① 沿弧长前进；③ 越界则钳到最后一点 ----
    # ⚠️ `ld <= 0` 只表示"不前视"，必须退化成投影点本身，**不能**跳到终点。
    s_target = proj.s + (ld if ld > 0.0 else 0.0)
    if s_target >= total:      # 含"车已跑过末点"（proj.s 已达总长）
        x, y, si = path.point_at(total)
        return LookaheadPoint(x=x, y=y, seg_index=si, s=total, clamped_to_vertex=False)
    if s_target < 0.0:
        s_target = 0.0
    x, y, si = path.point_at(s_target)
    return LookaheadPoint(x=x, y=y, seg_index=si, s=s_target, clamped_to_vertex=False)
