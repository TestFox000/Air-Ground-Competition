# -*- coding: utf-8 -*-
r"""用户手画侦察路线（对照臂）+ 覆盖率/帧数/时长对照。

## 路线来源与读图方法（可复核）

- 图：`_tmp_session/step_loop/scout_route_user.png`（1120x878，14x11 格，格 80 px，
  网格原点 (4,5)）；
- 读图：按路线主色 `RGB(76,76,76)` 做掩码 -> 逐格统计 -> 人工按格心连线顺序还原。
  手绘线有斜率/圆角，**斜线段按正交折线还原**（如 D4 斜到 E5 记为 D4->E4->E5）；
- 图上的标注：无人机起点 **M8**（主线描述常写作 N8，二者相邻；本对照以图上 M8 为准）、
  无人机终点 M2；先扫下半场（行 8~10 的西向条带），沿 B 列上穿隔板，再扫上半场
  （行 2~5 的东向条带），终点 M2；
- 该路线只扫了 6 条纵向条带（L/J/H/F/D/B）+ M 列，**未覆盖 N 列的 1~3/6/9~11**。

## 对照口径

两条路线都走 `frames.frames_from_cell_route()` / `frames.generate_snake_timeline()`
的**同一套视场模型**（4x2 格、格点帧位、格心严格在内）。用户路线按格心逐格给帧
（stride=1，与"手绘一格一帧"一致）；另给 stride=2（0.70 m，与推荐路线同帧距）
的第二组数字，避免"帧距不同"造成的口径偏差。
"""

from __future__ import annotations

from typing import Any

from .frames import (ScoutFrame, frames_from_cell_route, generate_snake_timeline,
                     timeline_stats)

#: 从用户手画图还原的格序列（行 8~10 下半场 -> B 列上穿 -> 行 2~5 上半场 -> M2 终点）。
USER_ROUTE_BLOCKS: tuple[str, ...] = (
    "M8", "L8", "L9", "L10",
    "K10", "J10", "J9", "J8", "I8", "H8", "H9", "H10",
    "G10", "F10", "F9", "F8", "E8", "D8", "D9", "D10",
    "C10", "B10", "B9", "B8", "B7", "B6", "B5", "B4", "B3", "B2",
    "C2", "D2", "D3", "D4", "E4", "E5", "F5", "F4", "F3", "F2",
    "G2", "H2", "H3", "H4", "H5", "I5", "J5", "J4", "J3", "J2",
    "K2", "L2", "L3", "L4", "L5", "M5", "M4", "M3", "M2",
)

#: 路线元数据（报告/JSON 用；后续若换图重画，只改这里）。
USER_ROUTE_SOURCE: dict[str, Any] = {
    "image": "_tmp_session/step_loop/scout_route_user.png",
    "method": "RGB(76,76,76) 掩码 + 逐格统计 + 按格心连线还原（斜线正交化）",
    "start": "M8",
    "end": "M2",
    "note": "图上起点 M8（主线描述写作 N8，二者相邻）；只扫 6 条纵向条带 + M 列，N 列有缺口",
}


def user_route_timeline(*, stride: int = 1, start_t: float = 0.0,
                        frame_hz: float = 2.0) -> list[ScoutFrame]:
    """用户手绘路线的帧位时间线（同一视场模型）。"""
    return frames_from_cell_route(list(USER_ROUTE_BLOCKS), stride=stride,
                                  start_t=start_t, frame_hz=frame_hz)


def compare_routes(generated: list[ScoutFrame] | None = None, *,
                   start_t: float = 0.0, frame_hz: float = 2.0) -> dict:
    """推荐路线 vs 用户手绘路线：覆盖率 / 帧数 / 时长（含同帧距口径）。

    返回 `{"generated": stats, "user_route": stats, "user_route_same_step": stats,
    "delta": {...}, "source": USER_ROUTE_SOURCE}`。
    """
    gen = generated if generated is not None else generate_snake_timeline(
        start_t=start_t, frame_hz=frame_hz)
    ref = user_route_timeline(stride=1, start_t=start_t, frame_hz=frame_hz)
    ref2 = user_route_timeline(stride=2, start_t=start_t, frame_hz=frame_hz)
    gen_stats = timeline_stats(gen)
    ref_stats = timeline_stats(ref)
    ref2_stats = timeline_stats(ref2)
    return {
        "generated": gen_stats,
        "user_route": ref_stats,
        "user_route_same_step": ref2_stats,
        "delta": {
            "coverage_cells": gen_stats["coverage_cells"] - ref_stats["coverage_cells"],
            "frames": gen_stats["frames"] - ref_stats["frames"],
            "duration_s": round(gen_stats["duration_s"] - ref_stats["duration_s"], 3),
            "coverage_cells_vs_same_step": (gen_stats["coverage_cells"]
                                            - ref2_stats["coverage_cells"]),
            "duration_s_vs_same_step": round(gen_stats["duration_s"]
                                             - ref2_stats["duration_s"], 3),
        },
        "source": dict(USER_ROUTE_SOURCE),
    }


__all__ = ["USER_ROUTE_BLOCKS", "USER_ROUTE_SOURCE", "user_route_timeline",
           "compare_routes"]
