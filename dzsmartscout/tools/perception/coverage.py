# -*- coding: utf-8 -*-
r"""片（slice）与覆盖清单（coverage）—— 让「扫过且为空」可以被表达。

## 为什么需要这个模块（t4 报告 P0-3，决策层方案 §4）

旧报文 ``field_verify`` 只有 ``mines/obstacles``：**空表与"没扫过"长得一模一样**。
车侧只能"报差异"，区分不了「这格是空的」与「我根本没看那格」⇒ 黑图永远点不亮。

本模块把**几何先验**算成"这个片到底判读了哪些格"，写进报文的
``coverage{complete,cells,mode}``。语义边界（与决策层方案 §4.2 逐字对齐）：

* ``complete: true``  = 本片的 ``coverage.cells`` **已判读完毕**，
  片内未出现的要素视为 **ABSENT**；
* ``complete: false`` = 只报看得见的，**片内其余格仍是 UNKNOWN**（**不得**当 ABSENT）。
  ⇒ :func:`protocol.coverage_absent_cells` 对不完整的片直接返回 ``None``。

## 口径（几何先验，与检测结果无关）

::

    片 = 无人机航线上「一个 checkpoint 之间」的航点集合（决策层方案 §4.1）
    coverage.cells = 格心到该片航段折线的距离 ≤ corridor_m 的格

**必须强调这不是"看到什么就报扫过什么"**：coverage 只由航点几何决定，
与 ``detect_field_objects`` 的输出**完全无关**——否则"没检到"会被自动解释成"扫过了且为空"，
正是黑图探索最不能接受的循环论证。

三个已知的**宽松方向**（宁可少报，不可虚报的反面，必须在现场收紧）：

1. 判据用**格心**：格子边缘没看全也会被算成"已判读"；
2. 端点按**圆头**处理（真实相机视场接近矩形，圆头在航段两端多算半圆）；
3. ``corridor_m`` 默认 0.525 m（= 1.5 格）是照 t4 §6.3 的 4 条横带**反推**出来的
   （行 2/5/8/11 各覆盖 ±1 行才可能凑满 154 格），**不是**相机实测视场。

⇒ 现场必须用 :func:`cells_of_route` 的同一口径，把 ``corridor_m`` 调到**实测视场**再发车；
   本模块的默认值只保证"合成场景下 7 片拼满 154 格"这一结构验收。

## 7 片从哪来（与 t4 §6.3 的 S3' 对齐）

``S2 桥洞优先短程`` 的 3 片（U 形：西扫 → 竖穿 K 列 → 东扫降落）+ ``4 条横带``：

============  ==================================  ======
片            航点                                 点数
============  ==================================  ======
S1            N8 → M8 → L8 → K8                   4
S2            K7 → K6 → K5 → K4 → K3 → K2          6
S3            L2 → M2                             2
S4            B2 → D2 → F2 → H2 → J2 → L2 → N2     7
S5            N5 → L5 → J5 → H5 → F5 → D5 → B5     7（蛇形反向）
S6            B8 → D8 → F8 → H8 → J8 → L8 → N8     7
S7            N11 → L11 → J11 → H11 → F11 → D11 → B11  7（蛇形反向）
============  ==================================  ======

合计 40 个航点（= t4 §6.3 的 12+28）。本机实测（:func:`SlicePlan.report`）：
7 片并集 **154/154 格**，每片 15~54 格不等。
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Iterable, Mapping, Sequence

import numpy as np

from .grid import GridSpec, RegistrationError

__all__ = [
    "Slice",
    "SliceCoverage",
    "SlicePlan",
    "DEFAULT_SLICE_ROUTES",
    "T1_SLICE_ROUTES",
    "SLICE_PRESETS",
    "cells_of_route",
    "plan_for",
    "parse_slice_spec",
    "as_plan",
]

#: 默认 7 片（t4 §6.3 的 S3' = U 形 3 段 + 4 条横带）
DEFAULT_SLICE_ROUTES: tuple[tuple[str, tuple[str, ...]], ...] = (
    ("S1", ("N8", "M8", "L8", "K8")),
    ("S2", ("K7", "K6", "K5", "K4", "K3", "K2")),
    ("S3", ("L2", "M2")),
    ("S4", ("B2", "D2", "F2", "H2", "J2", "L2", "N2")),
    ("S5", ("N5", "L5", "J5", "H5", "F5", "D5", "B5")),
    ("S6", ("B8", "D8", "F8", "H8", "J8", "L8", "N8")),
    ("S7", ("N11", "L11", "J11", "H11", "F11", "D11", "B11")),
)

#: 现役 T1 航线（只求桥洞的 3 片；覆盖 39/154，**不含**雷/障/靶）
T1_SLICE_ROUTES: tuple[tuple[str, tuple[str, ...]], ...] = DEFAULT_SLICE_ROUTES[:3]

SLICE_PRESETS: dict[str, str] = {
    "s3": "7 片：U 形 3 段 + 4 条横带（t4 §6.3 推荐，154/154）",
    "t1": "3 片：现役 U 形航线（只覆盖 39/154，只求桥洞）",
    "full": "1 片：整场（单帧全场，mode=full，154/154）",
}


# --------------------------------------------------------------------------- #
#  几何：格 → 片覆盖
# --------------------------------------------------------------------------- #
def _cell_centers(spec: GridSpec) -> tuple[list[str], np.ndarray]:
    blocks = spec.all_blocks()          # 行 1..11 × 列 A..N，顺序固定 ⇒ 结果可复现
    pts = np.asarray([spec.block_to_meters(b) for b in blocks], dtype=np.float64)
    return blocks, pts


def _point_segment_distance(pts: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """点到线段的距离（批量；退化线段按点距处理）。"""
    ab = b - a
    denom = float(ab @ ab)
    if denom <= 1e-12:
        return np.linalg.norm(pts - a, axis=1)
    t = np.clip(((pts - a) @ ab) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return np.linalg.norm(pts - proj, axis=1)


def cells_of_route(route: Sequence[str], spec: GridSpec | None = None, *,
                   corridor_m: float = 0.525, mode: str = "strip") -> list[str]:
    """片内 **coverage 格清单**（格心到航段折线的距离 ≤ ``corridor_m``）。

    * ``mode="full"``  ：直接返回全部 154 格（单帧全场口径）；
    * ``mode="point"`` ：按"到最近航点的距离 ≤ corridor_m"（3×3 式视场）；
    * ``mode="strip"`` ：按航段折线的走廊（默认，逐片报告用它）。

    返回顺序恒为网格序（行 1..11、列 A..N），与路线方向无关 ⇒ 同一片可复现、可逐字节比对。
    """
    spec = spec or GridSpec()
    mode = str(mode).strip().lower()
    if mode not in ("strip", "point", "full"):
        raise ValueError(f"coverage mode 只能是 strip/point/full，收到 {mode!r}")
    blocks, pts = _cell_centers(spec)
    if mode == "full":
        return list(blocks)
    waypoints = [w.strip().upper() for w in route if str(w).strip()]
    if not waypoints:
        raise ValueError("路线不能为空")
    if len(waypoints) == 1 or mode == "point":
        centers = np.asarray([spec.block_to_meters(w) for w in waypoints],
                             dtype=np.float64)
        d = np.min(np.linalg.norm(pts[:, None, :] - centers[None, :, :], axis=2), axis=1)
    else:
        centers = np.asarray([spec.block_to_meters(w) for w in waypoints],
                             dtype=np.float64)
        d = np.full(len(blocks), np.inf)
        for a, b in zip(centers[:-1], centers[1:]):
            d = np.minimum(d, _point_segment_distance(pts, a, b))
    keep = d <= float(corridor_m) + 1e-9
    return [b for b, k in zip(blocks, keep) if k]


# --------------------------------------------------------------------------- #
#  片 / 片清单
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Slice:
    """一个片：``id``（S1..）、``index``/``of``（第几片/共几片）、``route`` 航点。"""

    id: str
    index: int
    of: int
    route: tuple[str, ...] = ()
    complete: bool = True

    def to_json(self) -> dict:
        out = {"id": self.id, "index": int(self.index), "of": int(self.of)}
        if self.route:
            out["route"] = ">".join(self.route)
        return out


@dataclass(frozen=True)
class SliceCoverage:
    """一片 + 它判读的格清单（``complete`` 决定"片内未报"能不能当 ABSENT）。"""

    slice: Slice
    cells: tuple[str, ...]
    complete: bool = True

    def to_json(self) -> dict:
        return {"slice": self.slice.to_json(), "cells": list(self.cells),
                "complete": bool(self.complete), "n_cells": len(self.cells)}


@dataclass(frozen=True)
class SlicePlan:
    """一次扫描的片清单（``scan_id`` 由调用方给，见 :func:`protocol.make_scan_id`）。"""

    spec: GridSpec
    slices: tuple[Slice, ...]
    mode: str = "strip"
    corridor_m: float = 0.525

    # ---------------- 覆盖 ----------------
    def report(self) -> list[SliceCoverage]:
        """逐片算 coverage（纯几何，不读任何检测结果）。"""
        out: list[SliceCoverage] = []
        for sl in self.slices:
            cells = cells_of_route(sl.route, self.spec,
                                   corridor_m=self.corridor_m, mode=self.mode)
            out.append(SliceCoverage(slice=sl, cells=tuple(cells),
                                     complete=sl.complete))
        return out

    def union_cells(self) -> list[str]:
        """全部片的并集（网格序）。"""
        cov = set()
        for sl in self.report():
            cov.update(sl.cells)
        blocks, _ = _cell_centers(self.spec)
        return [b for b in blocks if b in cov]

    # ---------------- 编辑（frozen ⇒ 返回新对象） ----------------
    def mark_incomplete(self, selectors: Iterable[str | int]) -> "SlicePlan":
        """把指定片标成 ``complete=False``（帧丢失/只扫了一半时用）。

        ``selectors``：片 id（``"S4"``，大小写不敏感）或 1-based 序号（``4``）。
        """
        sel = list(selectors)
        wanted = {str(s).strip().upper() for s in sel if not isinstance(s, int)}
        unknown = wanted - {s.id.upper() for s in self.slices}
        bad_index = {s for s in sel if isinstance(s, int)} - {
            s.index for s in self.slices}
        if unknown or bad_index:
            raise ValueError(f"没有这些片：{sorted(unknown | {str(i) for i in bad_index})}"
                             f"（可选 {[s.id for s in self.slices]}）")
        hit_ids = {s.id.upper() for s in self.slices if s.id.upper() in wanted}
        hit_idx = {s.index for s in self.slices if s.index in sel}
        new = tuple(replace(sl, complete=False)
                    if (sl.id.upper() in hit_ids or sl.index in hit_idx) else sl
                    for sl in self.slices)
        return replace(self, slices=new)

    # ---------------- 序列化 ----------------
    def to_json(self) -> dict:
        reps = self.report()
        return {
            "mode": self.mode,
            "corridor_m": float(self.corridor_m),
            "n_slices": len(reps),
            "cells_union": len(self.union_cells()),
            "cells_total": self.spec.rows * self.spec.cols,
            "slices": [
                {"id": r.slice.id, "index": r.slice.index, "of": r.slice.of,
                 "route": ">".join(r.slice.route), "complete": r.complete,
                 "n_cells": len(r.cells), "cells": list(r.cells)}
                for r in reps
            ],
        }


def _routes_to_slices(routes: Sequence[tuple[str, Sequence[str]]]) -> tuple[Slice, ...]:
    of = len(routes)
    return tuple(Slice(id=str(sid), index=i, of=of,
                       route=tuple(str(w).strip().upper() for w in rts))
                 for i, (sid, rts) in enumerate(routes, start=1))


def plan_for(kind: str = "s3", spec: GridSpec | None = None, *,
             corridor_m: float = 0.525, mode: str | None = None) -> SlicePlan:
    """预设：``"s3"``（7 片）/ ``"t1"``（3 片）/ ``"full"``（1 片整场）。"""
    spec = spec or GridSpec()
    key = str(kind).strip().lower()
    if key not in SLICE_PRESETS:
        raise ValueError(f"未知预设 {kind!r}（可选 {sorted(SLICE_PRESETS)}）")
    if key == "full":
        all_blocks = tuple(spec.all_blocks())
        sl = Slice(id="S1", index=1, of=1, route=all_blocks)
        return SlicePlan(spec=spec, slices=(sl,), mode="full", corridor_m=corridor_m)
    routes = DEFAULT_SLICE_ROUTES if key == "s3" else T1_SLICE_ROUTES
    return SlicePlan(spec=spec, slices=_routes_to_slices(routes),
                     mode=(mode or "strip"), corridor_m=corridor_m)


def parse_slice_spec(text: str, spec: GridSpec | None = None, *,
                     corridor_m: float = 0.525,
                     mode: str | None = None) -> SlicePlan:
    """CLI 用的片规格解析。

    * ``"s3"`` / ``"t1"`` / ``"full"``：预设；
    * ``"N8>M8>L8>K8;K7>K6>K5"``：分号分片、``>`` 分航点，自动命名 S1..Sn；
    * ``"P1=A8>E8;P2=N10>C10"``：显式片 id。

    航点必须是网格内合法格名（否则抛 :class:`~.grid.RegistrationError`）。
    """
    spec = spec or GridSpec()
    raw = str(text or "").strip()
    if not raw:
        raise ValueError("片规格不能为空（可选 s3/t1/full 或 'A8>E8;N8>K8'）")
    if raw.lower() in SLICE_PRESETS:
        return plan_for(raw.lower(), spec, corridor_m=corridor_m, mode=mode)
    routes: list[tuple[str, tuple[str, ...]]] = []
    for k, chunk in enumerate([c for c in raw.replace("|", ";").split(";") if c.strip()],
                              start=1):
        chunk = chunk.strip()
        sid = f"S{k}"
        if "=" in chunk:
            sid, chunk = chunk.split("=", 1)
            sid = sid.strip().upper() or f"S{k}"
        waypoints = tuple(w.strip().upper() for w in chunk.split(">") if w.strip())
        if not waypoints:
            raise ValueError(f"第 {k} 片没有航点：{chunk!r}")
        for wp in waypoints:
            try:
                spec.block_to_index(wp)
            except RegistrationError as exc:
                raise RegistrationError(f"片 {sid} 的航点 {wp!r} 非法：{exc}") from exc
        routes.append((sid, waypoints))
    if len({sid for sid, _ in routes}) != len(routes):
        raise ValueError("片 id 重复（'P1=...;P1=...'）")
    return SlicePlan(spec=spec, slices=_routes_to_slices(routes),
                     mode=(mode or "strip"), corridor_m=corridor_m)


def as_plan(obj: SlicePlan | str | Mapping | None, spec: GridSpec | None = None,
             **kw) -> SlicePlan | None:
    """内部：``process()`` 允许传预设名 / 自定义串 / 已构造的 plan。"""
    if obj is None or isinstance(obj, SlicePlan):
        return obj
    if isinstance(obj, Mapping):
        raise TypeError("只接受 SlicePlan / 预设名 / 'A8>E8;...' 字符串")
    return parse_slice_spec(str(obj), spec, **kw)
