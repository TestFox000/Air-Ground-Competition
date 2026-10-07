# -*- coding: utf-8 -*-
"""前视点（lookahead point）—— 纯追踪的"瞄准目标"。

旧实现（`path_executor.find_lookahead_point` 1657–1732）的三条语义全部保留：
1. 从投影点沿路径按弧长 `ld` 前进，**可跨段**；路径不足前视距离时直接瞄准终点；
2. **转角顶点钳制**：本段与下一段夹角 > 阈值且距顶点 < `vertex_clamp_m` → 直接把前视点
   钳到顶点，防最后一刻切角（旧 20° / 0.04 m，硬编码；现在两个数都在 Config 里）；
3. 接近末点时钳到最后一点，**绝不越界**。

★ 前视点只按**弧长**推进（不是直线插值）—— 这是"贴着路线走"的关键：
  若按直线取点，过弯时会把目标点切到路线内侧，等于主动抄近道。
⚠️ 转角钳制只看**本段末端的顶点**（与旧实现一致）。若需要"提前看两个顶点"，
   属于策略升级（会影响通过性），不在本次移植范围。
"""

from __future__ import annotations

from dataclasses import dataclass

from ..core.config import Config
from .path import Path, Projection

__all__ = ["LookaheadPoint", "find_lookahead_point"]


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
    """
    total = path.total_length
    seg_index = proj.seg_index

    # ---- ② 转角顶点钳制（优先于前进：宁可直接瞄顶点，也不切角） ----
    clamp_m = cfg.get("pursuit.vertex_clamp_m")
    corner_thr = cfg.get("pursuit.lookahead_corner_angle_deg")
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
