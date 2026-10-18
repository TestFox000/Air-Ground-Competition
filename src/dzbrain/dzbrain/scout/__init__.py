# -*- coding: utf-8 -*-
r"""机端侦察仿真模型（步进循环协议 v2，T28）。

一站式入口：`simulate()` -> 帧位时间线 + 布局 v2 序列 + 心跳计划 + 对照指标。
各层也可单独使用（全部纯内存、无 ROS、无 DDS）：

    from dzbrain.scout import simulate, generate_snake_timeline, TruthMap
    res = simulate(start_t=-30.0)            # 读 config/map.yaml 真值图
    res.frames[0].t, res.frames[0].pose, res.frames[0].cells
    res.layouts[-1]["known"], res.layouts[-1]["target_cell"]
    res.heartbeat[:3]                        # [(t, layout), ...]

模块划分：
- `fov.py`：4x2 格视场（格点姿态下 2x4/4x2 恰好 8 格）+ 面积重叠；
- `frames.py`：蛇形条带帧位（下半场 -> 上半场）+ 任意格路线转帧位；
- `layout.py`：真值图 / known 可达 / 侦察前沿 / 目标格 / 布局 v2；
- `heartbeat.py`：变化即发 + 每秒重发（纯函数 + 无 I/O 状态机）；
- `reference.py`：用户手绘图路线与覆盖率对照。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .fov import (FOV_LENGTH_CELLS, FOV_LENGTH_M, FOV_WIDTH_CELLS, FOV_WIDTH_M,
                  SCOUT_ALTITUDE_M, Pose, all_blocks, block_key, cells_in_view,
                  cell_centers, footprint_corners, overlap_fraction,
                  sorted_blocks)
from .frames import (DEFAULT_FRAME_HZ, ScoutFrame, adjacent_overlaps, coverage,
                     frames_from_cell_route, frames_from_poses, generate_timeline,
                     generate_snake_timeline, timeline_duration_s, timeline_stats)
from .heartbeat import (DEFAULT_HEARTBEAT_S, LayoutFeed, heartbeat_messages,
                        heartbeat_schedule, to_json)
from .layout import (LayoutStream, TruthMap, build_layout, choose_target_cell,
                     frontier_cells, layouts_for_timeline, reachable_cells,
                     step_ok)
from .reference import (USER_ROUTE_BLOCKS, USER_ROUTE_SOURCE, compare_routes,
                        user_route_timeline)


@dataclass
class ScoutResult:
    """`simulate()` 的产物（可 `as_dict()` 直接落 JSON）。"""

    truth: TruthMap
    frames: list[ScoutFrame] = field(default_factory=list)
    layouts: list[dict] = field(default_factory=list)
    heartbeat: list[tuple[float, dict]] = field(default_factory=list)
    stats: dict = field(default_factory=dict)
    comparison: dict = field(default_factory=dict)
    car_block: str = ""

    @property
    def known_count(self) -> int:
        return len(self.layouts[-1]["known"]) if self.layouts else 0

    def as_dict(self, *, include_frames: bool = False) -> dict:
        out: dict[str, Any] = {
            "truth": {
                "name": self.truth.name, "start": self.truth.start,
                "end": self.truth.end,
                "mines": sorted_blocks(self.truth.mines),
                "obstacles": sorted_blocks(self.truth.obstacles),
                "targets": sorted_blocks(self.truth.targets),
                "bridges": sorted_blocks(self.truth.bridges),
            },
            "stats": dict(self.stats),
            "layouts": self.layouts,
            "heartbeat_count": len(self.heartbeat),
            "heartbeat_first": [[round(t, 3), int(l["version"])]
                                for t, l in self.heartbeat[:5]],
            "comparison": self.comparison,
            "known_count": self.known_count,
            "car_block": self.car_block,
        }
        if include_frames:
            out["frames"] = [f.as_dict() for f in self.frames]
        return out


def simulate(*, map_path: str | None = None, start_t: float = -30.0,
             frame_hz: float = DEFAULT_FRAME_HZ,
             car_block: str | None = None,
             heartbeat_interval_s: float = DEFAULT_HEARTBEAT_S,
             t_end: float | None = None) -> ScoutResult:
    """跑一遍"机端侦察"：真值图 -> 帧位 -> 布局 v2 -> 心跳计划 -> 对照。

    :param start_t: 机侧时间基准（T29 场景 t=-30 s 起飞；默认 -30）。
    :param car_block: 车当前格（决定 target_cell）；缺省用图上的 start。
    :param t_end: 心跳计划截止时刻；缺省=最后一帧。
    """
    truth = TruthMap.load(map_path)
    frames = generate_snake_timeline(start_t=start_t, frame_hz=frame_hz)
    stream = LayoutStream(truth, car_block=car_block)
    layouts = stream.layouts_for(frames)
    end = frames[-1].t if t_end is None else float(t_end)
    heartbeat = heartbeat_messages(layouts, t_end=end,
                                   interval_s=heartbeat_interval_s)
    return ScoutResult(
        truth=truth, frames=frames, layouts=layouts, heartbeat=heartbeat,
        stats=timeline_stats(frames),
        comparison=compare_routes(frames, start_t=start_t, frame_hz=frame_hz),
        car_block=stream.car_block,
    )


__all__ = [
    "FOV_LENGTH_CELLS", "FOV_WIDTH_CELLS", "FOV_LENGTH_M", "FOV_WIDTH_M",
    "SCOUT_ALTITUDE_M", "Pose", "all_blocks", "block_key", "cells_in_view",
    "cell_centers", "footprint_corners", "overlap_fraction", "sorted_blocks",
    "DEFAULT_FRAME_HZ", "ScoutFrame", "adjacent_overlaps", "coverage",
    "frames_from_cell_route", "frames_from_poses", "generate_timeline",
    "generate_snake_timeline",
    "timeline_duration_s", "timeline_stats",
    "DEFAULT_HEARTBEAT_S", "LayoutFeed", "heartbeat_messages",
    "heartbeat_schedule", "to_json",
    "LayoutStream", "TruthMap", "build_layout", "choose_target_cell",
    "frontier_cells", "layouts_for_timeline", "reachable_cells", "step_ok",
    "USER_ROUTE_BLOCKS", "USER_ROUTE_SOURCE", "compare_routes",
    "user_route_timeline", "ScoutResult", "simulate",
]
