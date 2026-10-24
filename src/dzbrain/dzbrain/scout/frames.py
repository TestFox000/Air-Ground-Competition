# -*- coding: utf-8 -*-
r"""侦察帧位生成（步进循环协议 v2 §4.1）。

本模块提供两类**纯函数**：

1. `generate_snake_timeline()`：**推荐路线**。下半场 7 条纵向条带（先 N/M 侧，
   再向西）-> 沿 A/B 列外侧上到上半场 -> 上半场 7 条纵向条带回扫到 M/N 侧。
   帧位全部落在**格点**（0.35 m 整数倍），因此 0°/90° 视场恰好 2x4 / 4x2 = 8 格；
   相邻帧按"每条线 0.70 m 取一帧 + 转向处补一帧"生成，物理足迹重叠 >= 50%。

2. `frames_from_cell_route()`：把"按格心画出来的侦察路线"转成同一套视场模型的
   帧位（格心 -> 该格右下角格点；可选每 N 格抽一帧），用于**用户手绘路线对照**
   （见 `reference.py`）。同一视场模型保证两条路线的覆盖率可比。

坐标/姿态口径见 `fov.py`。本模块**不依赖 ROS / 不读地图 / 不发报文**。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

from ..core import geometry as geo
from .fov import (FOV_LENGTH_CELLS, FOV_WIDTH_CELLS, Pose, cells_in_view,
                  cell_overlap_fraction, overlap_fraction)

#: 默认帧频（协议 §4.1：1~2 Hz；取上限让车端尽早拿到布局）
DEFAULT_FRAME_HZ = 2.0

#: 7 条纵向条带所在的 y（列边界格点）：0.35, 1.05, ... 4.55。
#: 每条线覆盖相邻 2 列（如 0.35 = N/M，1.05 = L/K，...，4.55 = B/A）。
SCAN_LINE_YS: tuple[float, ...] = tuple(
    round(geo.BLOCK_SIZE * (2 * m + 1), 6) for m in range(7))

#: 下半场（行 7~11）沿 x 的两个帧位格点：x=1.05 覆盖行 7~10，x=0.35 覆盖行 9~11。
LOWER_SCAN_XS: tuple[float, float] = (1.05, 0.35)
#: 上半场（行 1~6）沿 x 的四个帧位格点（0.70 m 步长，覆盖行 1~9 中的 1~6）。
UPPER_SCAN_XS: tuple[float, float, float, float] = (1.40, 2.10, 2.80, 3.50)


@dataclass(frozen=True)
class ScoutFrame:
    """一帧侦察结果：时刻 + 姿态 + 视场覆盖格。"""

    index: int
    t: float
    pose: Pose
    cells: tuple[str, ...]

    def as_dict(self) -> dict:
        return {
            "index": self.index,
            "t": round(float(self.t), 3),
            "x": round(self.pose.x, 4),
            "y": round(self.pose.y, 4),
            "yaw_deg": self.pose.yaw_deg,
            "block": self.pose.block,
            "cells": list(self.cells),
        }


def _dedupe_poses(poses: Iterable[Pose]) -> list[Pose]:
    out: list[Pose] = []
    seen_prev: tuple[float, float, float] | None = None
    for pose in poses:
        key = (round(pose.x, 6), round(pose.y, 6), round(pose.yaw, 6))
        if key != seen_prev:
            out.append(Pose(key[0], key[1], key[2]))
            seen_prev = key
    return out


def scan_poses() -> list[Pose]:
    """推荐蛇形路线的**帧位序列**（不含时间；转角处有同位置双帧）。

    路线（先下半场、后上半场）：
      ★ 起点：无人机在 N8 起飞后进入首条扫描线 y=0.35 的格点 (x=1.05, y=0.35)，
        随后按 x=1.05 -> 0.35 扫完 N/M 列对（"先扫右侧/下半场"）；
      线序 y = 0.35,1.05,...,4.55（即 N/M, L/K, ..., B/A 七对列）；
      下半场每线在 x=1.05 / 0.35 两帧，沿 x 往返；到最西侧后沿 y=4.55 横移到
      上半场起点 x=1.40；上半场每线在 x=1.40/2.10/2.80/3.50 四帧回扫。
    转角处：上一线末帧（旧航向）-> 同位置新航向帧 -> 平移 0.70 m 的同航向帧，
    保证相邻帧足迹重叠 >= 50%（协议要求 >= 30%）。
    """
    poses: list[Pose] = []
    prev_end: Pose | None = None
    half_pi = math.pi / 2.0
    for i, y in enumerate(SCAN_LINE_YS):
        xs = LOWER_SCAN_XS if i % 2 == 0 else LOWER_SCAN_XS[::-1]
        yaw = math.pi if i % 2 == 0 else 0.0
        if prev_end is not None:
            poses.append(Pose(prev_end.x, prev_end.y, half_pi))      # 转到 +y
            poses.append(Pose(prev_end.x, y, half_pi))               # 沿 +y 平移
            poses.append(Pose(prev_end.x, y, yaw))                   # 转到本线方向
        for x in xs:
            poses.append(Pose(x, y, yaw))
        prev_end = Pose(xs[-1], y, yaw)
    # 下半场最后一条线（i=6）末端 x=0.35,y=4.55：沿 +x 横移到上半场起点 x=1.40
    assert prev_end is not None
    poses.append(Pose(prev_end.x, prev_end.y, 0.0))
    for x in (1.05, 1.40):
        poses.append(Pose(x, prev_end.y, 0.0))
    # 上半场：y 从 4.55 回扫到 0.35
    prev_end = Pose(1.40, SCAN_LINE_YS[-1], 0.0)
    rev = list(reversed(SCAN_LINE_YS))
    for k, y in enumerate(rev):
        xs = UPPER_SCAN_XS if k % 2 == 0 else UPPER_SCAN_XS[::-1]
        yaw = 0.0 if k % 2 == 0 else math.pi
        if k > 0:
            poses.append(Pose(prev_end.x, prev_end.y, -half_pi))      # 转到 -y
            poses.append(Pose(prev_end.x, y, -half_pi))              # 沿 -y 平移
            poses.append(Pose(prev_end.x, y, yaw))                   # 转到本线方向
        for x in xs:
            poses.append(Pose(x, y, yaw))
        prev_end = Pose(xs[-1], y, yaw)
    return _dedupe_poses(poses)


def frames_from_poses(poses: Sequence[Pose], *, start_t: float = 0.0,
                      frame_hz: float = DEFAULT_FRAME_HZ) -> list[ScoutFrame]:
    """给姿态序列加时间戳与视场格集 -> `timeline`。"""
    if frame_hz <= 0:
        raise ValueError("frame_hz 必须 > 0")
    return [ScoutFrame(index=i, t=float(start_t) + i / float(frame_hz),
                       pose=pose, cells=cells_in_view(pose))
            for i, pose in enumerate(poses)]


def generate_snake_timeline(*, start_t: float = 0.0,
                            frame_hz: float = DEFAULT_FRAME_HZ) -> list[ScoutFrame]:
    """推荐蛇形路线的`timeline`：`[(t, pose, 视场格集)]`（协议 §4.2）。

    `start_t`：机侧时间基准。T29 场景"无人机 t=-30 s 起飞"传 `start_t=-30.0`。
    """
    return frames_from_poses(scan_poses(), start_t=start_t, frame_hz=frame_hz)


#: 别名（T29 常见叫法）：就是推荐蛇形路线的 timeline。
generate_timeline = generate_snake_timeline


def _yaw_between(a: tuple[float, float], b: tuple[float, float]) -> float:
    """格心/格点坐标 a->b 的 map yaw（x 正方向 = 0，y 正方向 = pi/2）。"""
    return math.atan2(b[1] - a[1], b[0] - a[0])


def frames_from_cell_route(blocks: Sequence[str], *,
                           start_t: float = 0.0,
                           frame_hz: float = DEFAULT_FRAME_HZ,
                           stride: int = 1) -> list[ScoutFrame]:
    """把"格心串起来的路线"转成同一视场模型的帧位（用于手绘路线对照）。

    * 位置：格心 -> 该格 **右下角格点**（x/y 各 +0.175 m），保证 0°/90° 恰好 8 格；
    * 每 `stride` 格取一帧（1=逐格；2=0.70 m 步长，与推荐路线同帧距）；
    * 方向变化处补"进/出"双帧（同位置换航向），使相邻帧重叠不塌陷。
    """
    if stride < 1:
        raise ValueError("stride 必须 >= 1")
    if len(blocks) < 2:
        raise ValueError("格路线至少需要 2 个格")
    cells = [(int(b[1:]) - 1, geo.COL_LETTERS.index(b[0].upper())) for b in blocks]
    idxs = list(range(0, len(cells), stride))
    if idxs[-1] != len(cells) - 1:
        idxs.append(len(cells) - 1)
    half = geo.BLOCK_SIZE / 2.0
    poses: list[Pose] = []
    for idx in idxs:
        x, y = geo.block_to_meters(blocks[idx])
        pos = (x + half, y + half)
        inc = _yaw_between(
            geo.block_to_meters(blocks[idx - 1]), geo.block_to_meters(blocks[idx])
        ) if idx > 0 else None
        out = _yaw_between(
            geo.block_to_meters(blocks[idx]), geo.block_to_meters(blocks[idx + 1])
        ) if idx + 1 < len(cells) else None
        if inc is None:
            poses.append(Pose(pos[0], pos[1], 0.0 if out is None else out))
        elif out is None:
            poses.append(Pose(pos[0], pos[1], inc))
        elif abs(((inc - out + math.pi) % (2 * math.pi)) - math.pi) < 1e-9:
            poses.append(Pose(pos[0], pos[1], inc))
        else:
            poses.append(Pose(pos[0], pos[1], inc))
            poses.append(Pose(pos[0], pos[1], out))
    poses = _dedupe_poses(poses)
    return frames_from_poses(poses, start_t=start_t, frame_hz=frame_hz)


def coverage(frames: Iterable[ScoutFrame]) -> set[str]:
    """全部帧覆盖到的格（并集）。"""
    out: set[str] = set()
    for frame in frames:
        out.update(frame.cells)
    return out


def adjacent_overlaps(frames: Sequence[ScoutFrame]) -> list[float]:
    """相邻帧**物理足迹**面积重叠率（0..1）。"""
    return [overlap_fraction(frames[i].pose, frames[i + 1].pose)
            for i in range(len(frames) - 1)]


def cell_overlaps(frames: Sequence[ScoutFrame]) -> list[float]:
    """相邻帧**格集合** Jaccard（诊断；协议判据用物理面积）。"""
    return [cell_overlap_fraction(frames[i].pose, frames[i + 1].pose)
            for i in range(len(frames) - 1)]


def timeline_duration_s(frames: Sequence[ScoutFrame]) -> float:
    """时间线时长（秒）：`(n-1)/frame_hz`（帧间等距）。"""
    if len(frames) < 2:
        return 0.0
    return float(frames[-1].t - frames[0].t)


def timeline_stats(frames: Sequence[ScoutFrame]) -> dict:
    """路线级指标（覆盖率/帧数/时长/重叠/格数分布）。"""
    total = geo.MAP_COLS * geo.MAP_ROWS
    cov = coverage(frames)
    area = adjacent_overlaps(frames)
    cell_ov = cell_overlaps(frames)
    counts = [len(f.cells) for f in frames]
    return {
        "frames": len(frames),
        "duration_s": round(timeline_duration_s(frames), 3),
        "start_t": round(frames[0].t, 3) if frames else 0.0,
        "end_t": round(frames[-1].t, 3) if frames else 0.0,
        "coverage_cells": len(cov),
        "cells_total": total,
        "coverage_ratio": round(len(cov) / total, 4),
        "missing": sorted((b for b in (f"{c}{r}" for r in range(1, geo.MAP_ROWS + 1)
                                       for c in geo.COL_LETTERS) if b not in cov),
                          key=lambda b: (int(b[1:]), b[0])),
        "overlap_area_min": round(min(area), 4) if area else 1.0,
        "overlap_area_mean": round(sum(area) / len(area), 4) if area else 1.0,
        "overlap_cell_min": round(min(cell_ov), 4) if cell_ov else 1.0,
        "cells_per_frame_min": min(counts) if counts else 0,
        "cells_per_frame_max": max(counts) if counts else 0,
        "fov_length_cells": FOV_LENGTH_CELLS,
        "fov_width_cells": FOV_WIDTH_CELLS,
    }


__all__ = [
    "DEFAULT_FRAME_HZ", "SCAN_LINE_YS", "LOWER_SCAN_XS", "UPPER_SCAN_XS",
    "ScoutFrame", "scan_poses", "frames_from_poses", "generate_snake_timeline",
    "frames_from_cell_route", "coverage", "adjacent_overlaps", "cell_overlaps",
    "timeline_duration_s", "timeline_stats",
]
