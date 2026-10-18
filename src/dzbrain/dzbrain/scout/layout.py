# -*- coding: utf-8 -*-
r"""布局 v2 生成与目标格决策（步进循环协议 v2 §2 / §4.2 / §4.3）。

职责（纯函数 + 一个无 I/O 的增量类）：
1. TruthMap：本局真值图（只读 config/map.yaml；机侧仿真用，车端不读文件）；
2. reachable_cells()：车从当前格出发、只走 known 格的可达集合
   （未知格=障碍；隔板只在 A/B/C 列可穿；K 列只在桥洞行进出 —— 与车端规划器同口径）；
3. 目标格决策（T30 修正）：**任务目标优先** —— 在 known 图上用 planner 规划一条
   "剩余雷 + 射击停站 + 终点" 的路线，取路线上的**第一个未完成检查点**作为 target_cell；
   规划失败/目标尚未进 known 时回退"朝任务目标方向的最远可达侦察前沿格"。
   全 known 时该检查点序列必须覆盖 8/8 雷 + 2/2 射击停站 + 到终点（test_scout 对账）。
4. build_layout() / LayoutStream：每帧把视场格并进 known，特征只从 known 里取，
   version 单调递增，输出可直接下发的 v2 报文 dict（字段见 docs/step_loop.md §2）。

★ 为什么 T30 要改：T29 旧口径取"最远可达前沿"（全 known 后=对角 M1），
  车被送到场地另一端，不经过雷/靶 —— 跑到 N5 也不是任务运行。任务目标优先后，
  无人机的 target 才是"下一个该压的雷 / 该打的停站 / 终点"。
★ "目标格"用格而不是米制：车端规划器/回报都按格名，减少一层换算。
★ 未知格=障碍是 v2 的核心安全语义：路线绝不进未知格（协议 §3.5）。
"""

from __future__ import annotations

import math
import pathlib
from dataclasses import dataclass
from functools import lru_cache
from typing import Any, Iterable

from ..core import geometry as geo
from ..route.parser import parse_token
from ..route.validate import CROSS_COLUMN
from .fov import all_blocks, block_key, sorted_blocks

_EPS = 1e-9
#: K 列的 0-based 列索引（桥洞单列）
_K_COL = geo.COL_LETTERS.index(CROSS_COLUMN)
#: 任务目标类型：雷 / 射击停站 / 终点 / 侦察前沿（回退）
MISSION_KINDS = ("mine", "stop", "end", "frontier")


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


@lru_cache(maxsize=1)
def default_config():
    """读取仓库 config/robot.yaml（仿真用；失败退回 SCHEMA 默认值）。"""
    from ..core.config import Config

    path = _repo_root() / "config" / "robot.yaml"
    try:
        return Config.load(str(path))
    except Exception:                                  # noqa: BLE001 仿真不因坏配置停摆
        return Config.from_dict({})


@dataclass(frozen=True)
class TruthMap:
    """本局真值图（机侧仿真口径；字段与 config/map.yaml 的 map: 段一致）。"""

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
        """从 {'map': {...}}（原始 YAML）或已展平 dict 构造。"""
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
        """读 config/map.yaml（默认仓库真源）。机侧仿真入口，车上不调用。"""
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
        """桥洞所在行号（1-based；只认 K 列格，与 route.planner 同口径）。"""
        return frozenset(int(b[1:]) for b in self.bridges if b[0] == CROSS_COLUMN)

    def blocked_for_driving(self, block: str) -> bool:
        """车能不能"站"在这一格（障碍/靶板不可站；未知格由调用方另行判断）。"""
        return block in self.obstacles or block in self.targets


# --------------------------------------------------------------------------- #
#  可达 / 前沿
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
    """从 cur 走到相邻 nxt 是否合法（车端规划器口径的马口）。

    四类硬约束（与 route.planner.Grid.move_ok / 协议 §3.5 对齐）：
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
    """known 图上的 BFS：返回 {可达格: 步数}（未知格/障/靶不可通行）。"""
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


# --------------------------------------------------------------------------- #
#  任务目标（T30 修正）：known 图 planner 检查点
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class MissionTarget:
    """一次目标格决策的结果（block + 类型 + 证据）。"""

    block: str
    kind: str                       # mine | stop | end | frontier
    engine: str = ""                # 任务路线用的引擎（frontier 为空）
    route: tuple[str, ...] = ()     # planner 产出的完整 token（证据/缓存）

    def as_dict(self) -> dict:
        return {"block": self.block, "kind": self.kind, "engine": self.engine,
                "route_len": len(self.route)}


def known_map_cfg(truth: TruthMap, known: Iterable[str], *,
                  mines: Iterable[str], from_block: str | None = None) -> dict:
    """known 图 -> planner 输入（未知格塞进 obstacles：不可通行；不是物理水桶）。

    ★ from_block（车当前格）**强制算已知**：车物理上在那里，否则起点被判未知会让
      任务规划永远无解（与 T27 plan_segment_from_layout 的起点破例同口径）。
    """
    known_set = {str(b).strip().upper() for b in known}
    if from_block:
        known_set.add(str(from_block).strip().upper())
    unknown = set(all_blocks()) - known_set
    return {"map": {
        "name": "scout:known", "start": truth.start, "end": truth.end,
        "mines": sorted_blocks(mines),
        "obstacles": sorted_blocks(set(truth.obstacles) | unknown),
        "targets": sorted_blocks(truth.targets),
        "bridges": sorted_blocks(truth.bridges),
        "barrier_col_from": truth.barrier_col_from,
        "barrier_row_hi": int(truth.barrier_row_hi),
        "barrier_row_lo": int(truth.barrier_row_lo),
    }}


def _ensure_second(tokens: list[str], map_cfg: dict, cfg: Any,
                   source: str) -> list[str]:
    from ..route import planner as PL

    try:
        out, _added, _note = PL.ensure_second_shoot_stop(tokens, map_cfg, cfg,
                                                         source=source)
        return list(out)
    except Exception:                                  # noqa: BLE001 补标失败不影响路线
        return list(tokens)


def plan_mission_route(known: Iterable[str], truth: TruthMap, *,
                       from_block: str,
                       covered_mines: Iterable[str] = (),
                       done_stops: Iterable[str] = (),
                       cfg: Any = None,
                       max_stops: int = 2,
                       max_seconds: float = 10.0) -> tuple[str, list[str]] | None:
    """在 known 图上规划"剩余雷 + 射击停站 + 终点"，返回 (engine, tokens) 或 None。

    只有**所有剩余雷 + 终点都已在 known 里**才尝试 planner（否则必定失败，
    直接回退前沿，避免仿真里每帧一次无谓的 1~2 s 规划）；planner 依次尝试
    tracker-safe（带 from_cell=车当前格）-> tracker-safe 整线 -> tour。
    """
    known_set = {str(b).strip().upper() for b in known}
    car = str(from_block).strip().upper()
    remaining = set(truth.mines) - {str(b).strip().upper() for b in covered_mines}
    done = {str(b).strip().upper() for b in done_stops}
    if not remaining and len(done) >= max_stops:
        return None                                     # 任务全完成 -> 直接 end
    if not (remaining | {truth.end}) <= (known_set | {car}):
        return None                                     # 目标还没进 known -> 前沿
    from ..route import planner as PL

    conf = cfg if cfg is not None else default_config()
    map_cfg = known_map_cfg(truth, known_set, mines=remaining, from_block=car)
    allow_rev = bool(conf.get("planner.allow_reverse", False))
    try:
        pivot = float(conf.get("planner.pivot_penalty", 0.0) or 0.0)
    except (TypeError, ValueError):
        pivot = 0.0
    attempts: list[tuple[str, Any]] = []
    # ① 增量：从车当前格续算（tracker-safe 支持 from_cell）
    def _ts_from() -> Any:
        return PL.plan_route_tracker_safe(map_cfg, conf, from_cell=car,
                                          max_seconds=max_seconds,
                                          source="scout:mission")
    attempts.append(("tracker-safe(from_cell)", _ts_from))
    # ② 整线：从图上的 start 重算（车不在 N10 时也给出全局检查点顺序）
    def _ts_full() -> Any:
        return PL.plan_route_tracker_safe(map_cfg, conf,
                                          max_seconds=max_seconds,
                                          source="scout:mission")
    attempts.append(("tracker-safe", _ts_full))
    # ③ 旧引擎兜底
    def _tour() -> Any:
        return PL.plan_route(map_cfg, conf, allow_reverse=allow_rev,
                             pivot_penalty=pivot, max_seconds=max_seconds,
                             source="scout:mission")
    attempts.append(("tour", _tour))
    for engine, fn in attempts:
        try:
            res = fn()
            tokens = _ensure_second([str(t) for t in (res.tokens or [])],
                                    map_cfg, conf, "scout:mission")
            if tokens:
                return engine, tokens
        except Exception:                              # noqa: BLE001 换下一个引擎
            continue
    return None


def mission_checkpoints(tokens: Iterable[str], truth: TruthMap, *,
                        covered_mines: Iterable[str] = (),
                        done_stops: Iterable[str] = (),
                        max_stops: int = 2) -> list[MissionTarget]:
    """把 planner token 串切成"检查点序列"：未压雷 / 未做射击停站 / 终点。

    顺序 = planner 路线的实际经过顺序（优先序交给现有 planner，不在机端重排）。
    """
    covered = {str(b).strip().upper() for b in covered_mines}
    done = {str(b).strip().upper() for b in done_stops}
    out: list[MissionTarget] = []
    for token in tokens:
        try:
            info = parse_token(str(token))
            block = str(info["block"]).strip().upper()
        except Exception:                              # noqa: BLE001 坏 token 跳过
            continue
        item: MissionTarget | None = None
        if block in truth.mines and block not in covered:
            item = MissionTarget(block, "mine")
        elif bool(info.get("shoot")) and block not in done and len(done) < max_stops:
            item = MissionTarget(block, "stop")
        elif block == truth.end:
            item = MissionTarget(block, "end")
        if item is not None and (not out or (out[-1].block, out[-1].kind)
                                 != (item.block, item.kind)):
            out.append(item)
    return out


def plan_mission_target(known: Iterable[str], truth: TruthMap, from_block: str, *,
                        covered_mines: Iterable[str] = (),
                        done_stops: Iterable[str] = (),
                        cfg: Any = None,
                        max_stops: int = 2,
                        max_seconds: float = 10.0) -> MissionTarget | None:
    """任务目标优先：返回下一个任务检查点；做不到（未知挡住）返回 None。"""
    known_set = {str(b).strip().upper() for b in known}
    car = str(from_block).strip().upper()
    remaining = set(truth.mines) - {str(b).strip().upper() for b in covered_mines}
    done = {str(b).strip().upper() for b in done_stops}
    if not remaining and len(done) >= max_stops:
        dist = reachable_cells(known_set, truth, car)
        if truth.end in dist:
            return MissionTarget(truth.end, "end")
        return None
    planned = plan_mission_route(known_set, truth, from_block=car,
                                 covered_mines=covered_mines,
                                 done_stops=done_stops, cfg=cfg,
                                 max_stops=max_stops,
                                 max_seconds=max_seconds)
    if planned is None:
        return None
    engine, tokens = planned
    checkpoints = mission_checkpoints(tokens, truth, covered_mines=covered_mines,
                                      done_stops=done_stops, max_stops=max_stops)
    if not checkpoints:
        return None
    dist = reachable_cells(known_set | {car}, truth, car)
    for cp in checkpoints:
        if cp.block in dist:                           # 车能到才算合法目标
            return MissionTarget(cp.block, cp.kind, engine=engine,
                                 route=tuple(tokens))
    return None


def _euclid(a: str, b: str) -> float:
    ax, ay = geo.block_to_meters(a)
    bx, by = geo.block_to_meters(b)
    return math.hypot(ax - bx, ay - by)


def frontier_target(known: Iterable[str], truth: TruthMap, from_block: str, *,
                    covered_mines: Iterable[str] = (),
                    done_stops: Iterable[str] = (),
                    max_stops: int = 2) -> MissionTarget | None:
    """回退：朝任务目标方向的**最远可达侦察前沿格**。

    方向 = 最近未压雷（无雷则最近未做停站，再否则终点）；同方向按 BFS 距离取最远，
    再按行优先格名打破平局（确定性）。
    """
    known_set = {str(b).strip().upper() for b in known}
    car = str(from_block).strip().upper()
    dist = reachable_cells(known_set | {car}, truth, car)
    if not dist:
        return None
    remaining = sorted(set(truth.mines) - {str(b).strip().upper()
                                          for b in covered_mines})
    done = {str(b).strip().upper() for b in done_stops}
    objective = ""
    if remaining:
        objective = min(remaining, key=lambda b: (_euclid(b, car), block_key(b)))
    elif len(done) < max_stops:
        objective = truth.end
    else:
        objective = truth.end
    frontier = frontier_cells(known_set, truth) & set(dist)
    if not frontier:
        return MissionTarget(truth.end, "end") if truth.end in dist else None
    best = min(frontier, key=lambda b: (_euclid(b, objective), -dist[b], block_key(b)))
    return MissionTarget(best, "frontier")


def choose_mission_target(known: Iterable[str], truth: TruthMap, from_block: str, *,
                          covered_mines: Iterable[str] = (),
                          done_stops: Iterable[str] = (),
                          cfg: Any = None,
                          max_stops: int = 2) -> MissionTarget:
    """目标格决策总入口：任务目标优先，失败回退前沿（永不返回空 block）。"""
    target = plan_mission_target(known, truth, from_block,
                                 covered_mines=covered_mines,
                                 done_stops=done_stops, cfg=cfg,
                                 max_stops=max_stops)
    if target is not None:
        return target
    fallback = frontier_target(known, truth, from_block,
                               covered_mines=covered_mines,
                               done_stops=done_stops, max_stops=max_stops)
    if fallback is not None:
        return fallback
    return MissionTarget(truth.end, "end")


def choose_target_cell(known: Iterable[str], truth: TruthMap, from_block: str, *,
                       covered_mines: Iterable[str] = (),
                       done_stops: Iterable[str] = (),
                       cfg: Any = None,
                       max_stops: int = 2) -> str:
    """兼容旧接口：只返回格名（新代码请用 choose_mission_target 拿 kind/证据）。"""
    return choose_mission_target(known, truth, from_block,
                                 covered_mines=covered_mines,
                                 done_stops=done_stops, cfg=cfg,
                                 max_stops=max_stops).block


# --------------------------------------------------------------------------- #
#  布局 v2
# --------------------------------------------------------------------------- #
def build_layout(truth: TruthMap, known: Iterable[str], *,
                 version: int, ts: float, car_block: str,
                 segment_id: int | None = None,
                 target_cell: str | None = None,
                 covered_mines: Iterable[str] = (),
                 done_stops: Iterable[str] = (),
                 cfg: Any = None,
                 max_stops: int = 2) -> dict:
    """生成一份可直接下发的 field_layout v2 dict（字段顺序即协议顺序）。

    target_cell 显式给出时直接使用（LayoutStream 缓存结果，避免每版重复规划）；
    省略时按任务目标优先决策（未知挡住 -> 前沿）。
    """
    known_set = {str(b).strip().upper() for b in known}
    known_list = sorted(known_set, key=block_key)
    target = (str(target_cell).strip().upper() if target_cell is not None
              else choose_target_cell(known_set, truth, car_block,
                                      covered_mines=covered_mines,
                                      done_stops=done_stops, cfg=cfg,
                                      max_stops=max_stops))
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

    目标格（T30）：任务目标优先。缓存一份 planner 路线，车每完成一段
    （note_segment_done / observe_car_block）就沿检查点前进；缓存耗尽或不可达
    才重新规划，规划不了回退前沿。用法（T29）::

        stream = LayoutStream(truth, car_block="N10")
        for frame in timeline:
            layout = stream.feed(frame)          # 有新格才产生布局
            if layout: send(layout)
        # 车回报段完成后：
        layout = stream.note_segment_done("I8", now=t)
        if layout: send(layout)
    """

    def __init__(self, truth: TruthMap, *, car_block: str | None = None,
                 start_version: int = 0, cfg: Any = None,
                 max_stops: int = 2) -> None:
        self.truth = truth
        self.cfg = cfg if cfg is not None else default_config()
        self.max_stops = int(max_stops)
        self.known: set[str] = set()
        self.car_block = str(car_block or truth.start).strip().upper()
        self.version = int(start_version)
        self.covered_mines: set[str] = set()
        self.done_stops: set[str] = set()
        self.known_stops: set[str] = set()       # 规划路线里出现过的停站候选
        self.layouts: list[dict] = []
        self._target = MissionTarget("", "")
        self._mission_tokens: list[str] | None = None
        self._mission_engine = ""

    # ------------------------------------------------------------------ #
    @property
    def known_count(self) -> int:
        return len(self.known)

    @property
    def target_cell(self) -> str:
        return self._target.block

    @property
    def target_kind(self) -> str:
        return self._target.kind

    def _reachable(self) -> dict[str, int]:
        return reachable_cells(self.known | {self.car_block}, self.truth,
                               self.car_block)

    def _target_valid(self) -> bool:
        t = self._target
        if not t.block:
            return False
        if t.block not in self.known and t.block != self.car_block:
            return False
        if t.kind == "mine" and t.block in self.covered_mines:
            return False
        if t.kind == "stop" and (t.block in self.done_stops
                                 or len(self.done_stops) >= self.max_stops):
            return False
        if t.kind == "frontier":
            return (t.block in frontier_cells(self.known, self.truth)
                    and t.block in self._reachable())
        if t.kind in ("mine", "stop", "end"):
            return t.block in self._reachable()
        return False

    def _recompute_target(self) -> None:
        """任务目标优先；缓存检查点 -> planner -> 前沿回退。"""
        # ① 缓存路线：按 covered/done 过滤，取第一个仍可达的检查点（避免每段重规划）
        if self._mission_tokens:
            cps = mission_checkpoints(self._mission_tokens, self.truth,
                                      covered_mines=self.covered_mines,
                                      done_stops=self.done_stops,
                                      max_stops=self.max_stops)
            reach = self._reachable()
            for cp in cps:
                if cp.block in self.known and cp.block in reach:
                    if cp.kind == "stop":
                        self.known_stops.add(cp.block)
                    self._target = MissionTarget(cp.block, cp.kind,
                                                 engine=self._mission_engine,
                                                 route=tuple(self._mission_tokens))
                    return
            self._mission_tokens = None
        # ② 任务规划（只有目标都进 known 才会真的调 planner）
        mt = plan_mission_target(self.known, self.truth, self.car_block,
                                 covered_mines=self.covered_mines,
                                 done_stops=self.done_stops, cfg=self.cfg,
                                 max_stops=self.max_stops)
        if mt is not None:
            if mt.route:
                self._mission_tokens = list(mt.route)
                self._mission_engine = mt.engine
            if mt.kind == "stop":
                self.known_stops.add(mt.block)
            self._target = mt
            return
        # ③ 前沿回退
        fb = frontier_target(self.known, self.truth, self.car_block,
                             covered_mines=self.covered_mines,
                             done_stops=self.done_stops, max_stops=self.max_stops)
        self._target = fb if fb is not None else MissionTarget(self.truth.end, "end")

    def _emit(self, ts: float) -> dict:
        self.version += 1
        layout = build_layout(
            self.truth, self.known, version=self.version, ts=ts,
            car_block=self.car_block, target_cell=self._target.block,
            covered_mines=self.covered_mines, done_stops=self.done_stops,
            cfg=self.cfg, max_stops=self.max_stops)
        self.layouts.append(layout)
        return layout

    # ------------------------------------------------------------------ #
    def feed(self, frame: Any) -> dict | None:
        """喂一帧（ScoutFrame 或带 cells/t 的对象）；有新格才产生布局。"""
        cells = {str(b).strip().upper() for b in getattr(frame, "cells", ())}
        new = cells - self.known
        if not new:
            return None
        self.known |= new
        if not self._target_valid():
            self._recompute_target()
        return self._emit(float(getattr(frame, "t", 0.0)))

    def retarget(self, *, now: float, car_block: str | None = None) -> dict | None:
        """车移动后重算目标格；目标变了才产生新版本（心跳不换版）。"""
        if car_block is not None:
            self.car_block = str(car_block).strip().upper()
        old = (self._target.block, self._target.kind)
        self._recompute_target()
        if (self._target.block, self._target.kind) == old:
            return None
        return self._emit(float(now))

    def note_segment_done(self, block: str, *,
                          now: float = 0.0) -> dict | None:
        """车到达 target_cell（段完成）：记已压雷/已做停站，并推进到下一检查点。"""
        block = str(block).strip().upper()
        self.car_block = block
        if block in self.truth.mines:
            self.covered_mines.add(block)
        if block in self.known_stops:
            self.done_stops.add(block)
        old = (self._target.block, self._target.kind)
        self._recompute_target()
        if (self._target.block, self._target.kind) == old:
            return None
        return self._emit(float(now))

    def observe_car_block(self, block: str, *,
                          now: float = 0.0, done: bool = False) -> dict | None:
        """车位置回报（car_state.block）：累积已压雷；done=True 当段完成处理。"""
        block = str(block).strip().upper()
        changed = block != self.car_block
        self.car_block = block
        if block in self.truth.mines:
            self.covered_mines.add(block)
        if done:
            return self.note_segment_done(block, now=now)
        if changed and not self._target_valid():
            return self.retarget(now=now)
        return None

    def mark_covered(self, blocks: Iterable[str]) -> None:
        """显式标记已压雷（T29 用真值轨迹/车回报喂）。"""
        for block in blocks:
            text = str(block).strip().upper()
            if text in self.truth.mines:
                self.covered_mines.add(text)

    def layouts_for(self, frames: Iterable[Any]) -> list[dict]:
        out: list[dict] = []
        for frame in frames:
            layout = self.feed(frame)
            if layout is not None:
                out.append(layout)
        return out


def layouts_for_timeline(frames: Iterable[Any], truth: TruthMap, *,
                         car_block: str | None = None,
                         start_version: int = 0,
                         cfg: Any = None) -> list[dict]:
    """便捷函数：整条时间线 -> 布局序列（每次 known 增长一份）。"""
    stream = LayoutStream(truth, car_block=car_block,
                          start_version=start_version, cfg=cfg)
    return stream.layouts_for(frames)


__all__ = [
    "TruthMap", "step_ok", "reachable_cells", "frontier_cells",
    "MissionTarget", "known_map_cfg", "plan_mission_route",
    "mission_checkpoints", "plan_mission_target", "frontier_target",
    "choose_mission_target", "choose_target_cell", "build_layout",
    "LayoutStream", "layouts_for_timeline", "default_config",
]
