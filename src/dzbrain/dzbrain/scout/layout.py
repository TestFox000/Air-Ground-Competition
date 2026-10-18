# -*- coding: utf-8 -*-
r"""布局 v2 生成与目标格决策（步进循环协议 v2 §2 / §4.2 / §4.3）。

职责（**纯函数 + 一个无 I/O 的增量类**）：
1. `TruthMap`：本局真值图（只读 `config/map.yaml`；机侧仿真用，车端不读文件）；
2. `reachable_cells()`：车从当前格出发、**只走 known 格**的可达集合
   （未知格=障碍；隔板只在 A/B/C 列可穿；K 列只在桥洞行进出 —— 与车端规划器同口径）；
3. `choose_target_cell()`：车在 known 图里可达的**最远侦察前沿点**；
   终点已知且可达 -> 终点（协议 §4.3）；
4. `build_layout()` / `LayoutStream`：每帧把视场格并进 known，特征只从 known 里取，
   version 单调递增，输出可直接下发的 v2 报文 dict（字段见 `docs/step_loop.md` §2）。

★ "目标格"用**格**而不是米制：车端规划器/回报都按格名，减少一层换算。
★ 未知格=障碍是 v2 的核心安全语义：路线绝不进未知格（协议 §3.5）。
"""

from __future__ import annotations

import pathlib
from dataclasses import dataclass
from typing import Any, Iterable

from ..core import geometry as geo
from ..route.validate import CROSS_COLUMN
from .fov import all_blocks, block_key, sorted_blocks

_EPS = 1e-9
#: K 列的 0-based 列索引（桥洞单列）
_K_COL = geo.COL_LETTERS.index(CROSS_COLUMN)


def _as_cells(value: Any) -> list[str]:
    if isinstance(value, str):
        value = [v for v in value.replace(";", ",").split(",") if v.strip()]
    if not isinstance(value, (list, tuple, set, frozenset)):
        return []
    return sorted_blocks(str(v) for v in value if str(v).strip())


def _repo_root() -> pathlib.Path:
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "config" / "map.yaml").is_file():
            return parent
    return here.parents[4]


@dataclass(frozen=True)
class TruthMap:
    """本局真值图（机侧仿真口径；字段与 `config/map.yaml` 的 `map:` 段一致）。"""

    name: str
    start: str
    end: str
    mines: frozenset[str]
    obstacles: frozenset[str]
    targets: frozenset[str]
    bridges: frozenset[str]
    barrier_col_from: str = geo.BARRIER_COL_FROM
    barrier_row_hi: int = geo.BARRIER_ROW_HI
    barrier_row_lo: int = geo.BARRIER_ROW_LO

    @classmethod
    def from_map(cls, map_cfg: Any) -> "TruthMap":
        """从 `{'map': {...}}`（原始 YAML）或已展平 dict 构造。"""
        section: dict = {}
        if isinstance(map_cfg, dict):
            inner = map_cfg.get("map")
            section = dict(inner) if isinstance(inner, dict) else dict(map_cfg)
        if not section:
            raise ValueError("空地图：无法构造 TruthMap")
        start = str(section.get("start") or "").strip().upper()
        end = str(section.get("end") or "").strip().upper()
        if not start or not end:
            raise ValueError("地图缺少 map.start / map.end")
        geo.block_to_index(start)
        geo.block_to_index(end)
        return cls(
            name=str(section.get("name") or ""),
            start=start, end=end,
            mines=frozenset(_as_cells(section.get("mines"))),
            obstacles=frozenset(_as_cells(section.get("obstacles"))),
            targets=frozenset(_as_cells(section.get("targets"))),
            bridges=frozenset(_as_cells(section.get("bridges"))),
            barrier_col_from=str(section.get("barrier_col_from")
                                 or geo.BARRIER_COL_FROM).strip().upper(),
            barrier_row_hi=int(section.get("barrier_row_hi")
                               or geo.BARRIER_ROW_HI),
            barrier_row_lo=int(section.get("barrier_row_lo")
                               or geo.BARRIER_ROW_LO),
        )

    @classmethod
    def load(cls, path: str | pathlib.Path | None = None) -> "TruthMap":
        """读 `config/map.yaml`（默认仓库真源）。机侧仿真入口，车上不调用。"""
        import yaml
        target = pathlib.Path(path) if path else (_repo_root() / "config" / "map.yaml")
        raw = yaml.safe_load(target.read_text(encoding="utf-8")) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"地图不是 YAML 映射: {target}")
        return cls.from_map(raw)

    # ------------------------------------------------------------------ #
    @property
    def all_cells(self) -> frozenset[str]:
        return frozenset(all_blocks())

    @property
    def bridge_rows(self) -> frozenset[int]:
        """桥洞所在行号（1-based；只认 K 列格，与 `route.planner` 同口径）。"""
        return frozenset(int(b[1:]) for b in self.bridges if b[0] == CROSS_COLUMN)

    def blocked_for_driving(self, block: str) -> bool:
        """车能不能"站"在这一格（障碍/靶板不可站；未知格由调用方另行判断）。"""
        return block in self.obstacles or block in self.targets


# --------------------------------------------------------------------------- #
#  可达 / 前沿 / 目标格
# --------------------------------------------------------------------------- #
def _neighbours(block: str) -> tuple[str, ...]:
    row_idx, col_idx = geo.block_to_index(block)
    out: list[str] = []
    for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        r, c = row_idx + dr, col_idx + dc
        if 0 <= r < geo.MAP_ROWS and 0 <= c < geo.MAP_COLS:
            out.append(geo.index_to_block(r, c))
    return tuple(out)


def step_ok(truth: TruthMap, known: set[str], cur: str, nxt: str) -> bool:
    """从 `cur` 走到相邻 `nxt` 是否合法（**车端规划器口径的马口**）。

    四类硬约束（与 `route.planner.Grid.move_ok`/协议 §3.5 对齐）：
      ① 目标格必须已知且可站（未知=障碍；障/靶不可站）；
      ② 隔板（第 hi/lo 行之间）只有列索引 < barrier_col_from 的列能穿；
      ③ K 列内不得上下挪格（桥洞单格宽，侧壁会撞）；
      ④ 只能在桥洞行进出 K 列。
    """
    if nxt not in known or truth.blocked_for_driving(nxt):
        return False
    r0, c0 = geo.block_to_index(cur)          # 0-based
    r1, c1 = geo.block_to_index(nxt)
    if abs(r0 - r1) + abs(c0 - c1) != 1:
        return False
    hi, lo = truth.barrier_row_hi - 1, truth.barrier_row_lo - 1
    col_from = geo.COL_LETTERS.index(truth.barrier_col_from)
    if {r0, r1} == {hi, lo} and c1 >= col_from:
        return False
    if c0 == _K_COL and c1 == _K_COL:
        return False
    if (c0 == _K_COL) != (c1 == _K_COL):
        k_row = r0 if c0 == _K_COL else r1
        if (k_row + 1) not in truth.bridge_rows:
            return False
    return True


def reachable_cells(known: Iterable[str], truth: TruthMap,
                    from_block: str) -> dict[str, int]:
    """known 图上的 BFS：返回 `{可达格: 步数}`（未知格/障/靶不可通行）。"""
    known_set = {str(b).strip().upper() for b in known}
    start = str(from_block).strip().upper()
    if start not in known_set or truth.blocked_for_driving(start):
        return {}
    dist: dict[str, int] = {start: 0}
    queue: list[str] = [start]
    head = 0
    while head < len(queue):
        cur = queue[head]
        head += 1
        for nxt in _neighbours(cur):
            if nxt in dist:
                continue
            if not step_ok(truth, known_set, cur, nxt):
                continue
            dist[nxt] = dist[cur] + 1
            queue.append(nxt)
    return dist


def frontier_cells(known: Iterable[str], truth: TruthMap) -> set[str]:
    """侦察前沿点 = 已知格中至少有一个**越界内未知**正交邻居的格。"""
    known_set = {str(b).strip().upper() for b in known}
    out: set[str] = set()
    for block in known_set:
        for nxt in _neighbours(block):
            if nxt not in known_set:
                out.add(block)
                break
    return out


def choose_target_cell(known: Iterable[str], truth: TruthMap,
                       from_block: str) -> str:
    """目标格决策（协议 §4.3）。

    1. 终点已知且从 `from_block` 可达 -> 返回终点；
    2. 否则取"可达的最远侦察前沿点"（BFS 步数最大；同距离按行优先取最大格名，确定性）；
    3. 没有任何可达前沿 -> 返回终点（协议允许"否则 end"；车端会 WAIT_INFO）。
    """
    known_list = [str(b).strip().upper() for b in known]
    dist = reachable_cells(known_list, truth, from_block)
    if truth.end in dist:
        return truth.end
    known_set = set(known_list)
    reachable_frontier = frontier_cells(known_set, truth) & set(dist)
    if not reachable_frontier:
        return truth.end
    best = max(reachable_frontier, key=lambda b: (dist[b], block_key(b)))
    return best


# --------------------------------------------------------------------------- #
#  布局 v2
# --------------------------------------------------------------------------- #
def build_layout(truth: TruthMap, known: Iterable[str], *,
                 version: int, ts: float, car_block: str,
                 segment_id: int | None = None) -> dict:
    """生成一份可直接下发的 `field_layout` v2 dict（字段顺序即协议顺序）。"""
    known_set = {str(b).strip().upper() for b in known}
    known_list = sorted(known_set, key=block_key)
    target = choose_target_cell(known_set, truth, car_block)
    return {
        "type": "field_layout",
        "version": int(version),
        "ts": round(float(ts), 3),
        "start": truth.start,
        "end": truth.end,
        "known": known_list,
        "mines": sorted_blocks(truth.mines & known_set),
        "obstacles": sorted_blocks(truth.obstacles & known_set),
        "targets": sorted_blocks(truth.targets & known_set),
        "bridges": sorted_blocks(truth.bridges),
        "barrier_col_from": truth.barrier_col_from,
        "barrier_row_hi": int(truth.barrier_row_hi),
        "barrier_row_lo": int(truth.barrier_row_lo),
        "target_cell": target,
        "segment_id": int(version if segment_id is None else segment_id),
    }


class LayoutStream:
    """按帧增量生成布局（known 单调增长；变化才产生新 version）。

    用法（T29）：```
        stream = LayoutStream(truth, car_block="N10")
        for frame in timeline:
            layout = stream.feed(frame)      # None = 本帧没有新格
            if layout: send(layout)
        stream.car_block = "K7"              # 车移动后再 retarget
        layout = stream.retarget(now=...)
    ```
    """

    def __init__(self, truth: TruthMap, *, car_block: str | None = None,
                 start_version: int = 0) -> None:
        self.truth = truth
        self.known: set[str] = set()
        self.car_block = str(car_block or truth.start).strip().upper()
        self.version = int(start_version)
        self.target_cell = ""
        self.layouts: list[dict] = []

    @property
    def known_count(self) -> int:
        return len(self.known)

    def _emit(self, ts: float) -> dict:
        self.version += 1
        layout = build_layout(self.truth, self.known, version=self.version,
                              ts=ts, car_block=self.car_block)
        self.target_cell = layout["target_cell"]
        self.layouts.append(layout)
        return layout

    def feed(self, frame: Any) -> dict | None:
        """喂一帧（`ScoutFrame` 或带 `cells`/`t` 的对象）；有新格才产生布局。"""
        cells = {str(b).strip().upper() for b in getattr(frame, "cells", ())}
        new = cells - self.known
        if not new:
            return None
        self.known |= new
        return self._emit(float(getattr(frame, "t", 0.0)))

    def retarget(self, *, now: float, car_block: str | None = None) -> dict | None:
        """车移动后重算目标格；目标变了才产生新版本（心跳不换版）。"""
        if car_block is not None:
            self.car_block = str(car_block).strip().upper()
        new_target = choose_target_cell(self.known, self.truth, self.car_block)
        if new_target == self.target_cell:
            return None
        return self._emit(float(now))

    def layouts_for(self, frames: Iterable[Any]) -> list[dict]:
        out: list[dict] = []
        for frame in frames:
            layout = self.feed(frame)
            if layout is not None:
                out.append(layout)
        return out


def layouts_for_timeline(frames: Iterable[Any], truth: TruthMap, *,
                         car_block: str | None = None,
                         start_version: int = 0) -> list[dict]:
    """便捷函数：整条时间线 -> 布局序列（每次 known 增长一份）。"""
    stream = LayoutStream(truth, car_block=car_block, start_version=start_version)
    return stream.layouts_for(frames)


__all__ = [
    "TruthMap", "step_ok", "reachable_cells", "frontier_cells",
    "choose_target_cell", "build_layout", "LayoutStream", "layouts_for_timeline",
]
