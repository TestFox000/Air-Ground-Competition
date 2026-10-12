# -*- coding: utf-8 -*-
"""出发前路线校验（R001~R014）—— 纯函数、无 ROS 依赖、**只报问题不改写路线**。

血统：旧 `path_executor.validate_mission_blocks`（path_executor.py:342-478，**只报错不改写**）
+ 用户侧 `tools/route_check.py`。旧实现的教训（`docs/survey_path_executor.md` §5.3 / §7）：

- ⚠️ K13：障碍表真源唯一性 —— `obstacle_blocks` 曾与 `config/map.yaml` 的 `map.obstacles`
  不一致，导致射击点判据全错 + 误判路线踩障。本模块**只认 `map_cfg`**（来自 `map.yaml`），
  不再接受第二份障碍表。
- ⚠️ K11：`ticksPer2PI` 与轮距差 42% —— 直接影响"命令转 90° 实际约 38°"，
  过桥洞会撞壁。所以 R009 必须**硬拦斜切**，不能只告警。
- ⚠️ 旧代码的校验阈值大量硬编码；本模块全部走 `Config` 点号键。

每个检查一个**稳定编号**：`R001`~`R014`，编号被测试与文档引用，**不得改号**。
每条 `Issue.msg` 都带**证据数值**（距离/角度/格名/计数），方便直接贴进日志。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, Sequence

from ..core import geometry as geo
from ..core.config import Config
from .model import Route

__all__ = [
    "Issue", "validate_route", "summary", "crossing_entries",
    "CROSS_COLUMN", "CROSS_LOWER_ROWS", "CROSS_UPPER_ROWS",
    "BARRIER_SEGMENT_MAX_M", "MAX_SEGMENT_BLOCKS", "MIN_SEGMENT_M",
    "DEFAULT_SHOOT_DISTANCE_MIN_M", "DEFAULT_SHOOT_DISTANCE_MAX_M",
    "LOWER_HALF_ROWS", "UPPER_HALF_ROWS", "U_TURN_COLUMNS",
]

#: 穿洞列（现场是 K 列；`config/robot.yaml` 的 `bridge_blocks` 覆盖整列 K1~K11）
CROSS_COLUMN = "K"
#: 下半场穿洞行（向西穿 L* -> K* -> J*）
CROSS_LOWER_ROWS = (7, 11)
#: 上半场穿洞行（向东穿 J* -> K* -> L*）
CROSS_UPPER_ROWS = (1, 6)
#: 隔板：第 6/7 行之间、D~N 列封死，只有 A/B/C 列能上下穿行
BARRIER_ROW_HI = 6
BARRIER_ROW_LO = 7
BARRIER_COL_FROM = "D"
#: U 形绕行必须经过的列（否则没绕过 D6~N6/D7~N7 隔板）
U_TURN_COLUMNS = ("A", "B", "C")
#: 下半场 / 上半场的行号
LOWER_HALF_ROWS = (7, 11)
UPPER_HALF_ROWS = (1, 6)

#: R011：单段长度上限（格）与下限（米）
MAX_SEGMENT_BLOCKS = 2.5
MIN_SEGMENT_M = 0.02
#: R009：入洞段与穿洞方向的夹角上限（度）
MAX_CROSS_ANGLE_DEG = 10.0
#: R014：倒车段占比上限（仅提醒）
MAX_REVERSE_RATIO = 0.30
#: R006/R013：射击窗口（米）；缺省与 `docs/P0接口约定.md` §2 一致
DEFAULT_SHOOT_DISTANCE_MIN_M = 0.20
DEFAULT_SHOOT_DISTANCE_MAX_M = 0.45
#: 数值比较用的极小容差
EPS = 1e-9

# --------------------------------------------------------------------------- #
#  Config 键名（集中登记，避免字符串散落）
# --------------------------------------------------------------------------- #
_KEY_SHOOT_MIN = "path_executor.shoot_distance_min"
_KEY_SHOOT_MAX = "path_executor.shoot_distance_max"
_KEY_CROSS_LOWER = "route.cross_lower_candidates"
_KEY_CROSS_UPPER = "route.cross_upper_candidates"
_KEY_CROSS_LOWER_DEFAULT = "route.cross_lower_default"
_KEY_CROSS_UPPER_DEFAULT = "route.cross_upper_default"
_KEY_BRIDGE_BLOCKS = "pursuit.bridge_blocks"

_FALLBACK_CFG: Config | None = None


def _fallback_cfg() -> Config:
    """没有外部 Config 时，用 `config.SCHEMA` 的显式默认值兜底（绝不硬编码第二份默认值）。"""
    global _FALLBACK_CFG
    if _FALLBACK_CFG is None:
        _FALLBACK_CFG = Config.from_dict({}, source="<validate 默认值>")
    return _FALLBACK_CFG


def _cfg_get(cfg: Config | None, key: str, fallback: Any = None) -> Any:
    """安全取配置：缺失/类型不符/缺 PyYAML 都退回 `fallback`，绝不因为少一个键崩掉预检。"""
    source = cfg if cfg is not None else _fallback_cfg()
    try:
        value = source.get(key, default=None)
    except Exception:                                      # noqa: BLE001  配置缺失不该让预检崩
        return fallback
    return fallback if value is None else value


def _window(cfg: Config | None) -> tuple[float, float]:
    lo = float(_cfg_get(cfg, _KEY_SHOOT_MIN, DEFAULT_SHOOT_DISTANCE_MIN_M))
    hi = float(_cfg_get(cfg, _KEY_SHOOT_MAX, DEFAULT_SHOOT_DISTANCE_MAX_M))
    if lo > hi:
        lo, hi = hi, lo
    return lo, hi


def _as_list(value: Any) -> list[str]:
    if not value:
        return []
    if isinstance(value, str):
        return [p for p in value.replace(";", ",").split(",") if p.strip()]
    if isinstance(value, (list, tuple, set, frozenset)):
        return [str(v) for v in value]
    return []


def _cross_rows(cfg: Config | None,
                cross_candidates: Iterable[str] | None = None) -> tuple[list[int], list[int]]:
    """穿洞候选行：**优先从候选格名集合反推行号**（与现场 YAML 永远同步）。

    `docs/P0接口约定.md` §2 把 `cross_candidates` 定义为"穿洞候选集合"的覆盖项，
    行号与候选格名是同一件事的两种说法 —— 所以显式覆盖时**两边都要覆盖**：
    只覆盖集合不覆盖行号，就会出现"候选里有 K5、候选行里没有 5"的自相矛盾。
    """
    lower: list[int] = []
    upper: list[int] = []
    lower_names = _as_list(_cfg_get(cfg, _KEY_CROSS_LOWER, None))
    upper_names = _as_list(_cfg_get(cfg, _KEY_CROSS_UPPER, None))
    if cross_candidates:
        lower_names = list(lower_names) + [str(c) for c in cross_candidates]
        upper_names = list(upper_names) + [str(c) for c in cross_candidates]
    for raw in lower_names:
        try:
            row_idx, _ = geo.block_to_index(raw)
        except ValueError:
            continue
        lower.append(row_idx + 1)
    for raw in upper_names:
        try:
            row_idx, _ = geo.block_to_index(raw)
        except ValueError:
            continue
        upper.append(row_idx + 1)
    if not lower:
        lower = list(range(CROSS_LOWER_ROWS[0], CROSS_LOWER_ROWS[1] + 1))
    if not upper:
        upper = list(range(CROSS_UPPER_ROWS[0], CROSS_UPPER_ROWS[1] + 1))
    return lower, upper


def _candidates(cfg: Config | None, cross_candidates: Iterable[str] | None) -> set[str]:
    if cross_candidates:
        return {str(c).upper() for c in cross_candidates}
    pool = set(_as_list(_cfg_get(cfg, _KEY_CROSS_LOWER, None)))
    pool |= set(_as_list(_cfg_get(cfg, _KEY_CROSS_UPPER, None)))
    if pool:
        return pool
    lower, upper = _cross_rows(cfg)
    pool = {f"{CROSS_COLUMN}{r}" for r in lower}
    pool |= {f"{CROSS_COLUMN}{r}" for r in upper}
    # ★ t20（t14-F3）：这里**不再写第二份兜底穿洞格**（旧代码硬编码 "K9"/"K4"，属第四张图，
    #   与现役真值 K10/K5 不符 —— 过期值只会把人误导到失效的洞）。
    #   改为**读配置**：`_cfg_get(..., None)` 在键缺失时会走 `Config.get()` 的 SCHEMA 缺省
    #   （`core/config.py` 现值 K10/K5，与 `config/robot.yaml:520-521` 同源），
    #   所以"候选与兜底都拿不到"时补进池子的仍是配置里的兜底穿洞格。
    #   ⚠️ 本分支只在 `route.cross_{half}_candidates` 全空/非列表时才走到（现场两者都非空 ⇒ 不生效），
    #   且此处已按 K 列上下半场行段建池，故这是一次**只去旧值、不改判定**的收口。
    for key in (_KEY_CROSS_LOWER_DEFAULT, _KEY_CROSS_UPPER_DEFAULT):
        cell = str(_cfg_get(cfg, key, None) or "").strip().upper()
        if cell:
            pool.add(cell)
    return pool


# --------------------------------------------------------------------------- #
#  地图要素
# --------------------------------------------------------------------------- #
def _map_section(map_cfg: Any) -> dict:
    """`map_cfg` 既接受 `{'map': {...}}`（原始 YAML），也接受已展平的 dict / `Config`。"""
    if map_cfg is None:
        return {}
    raw = getattr(map_cfg, "raw", None)
    if isinstance(raw, dict):
        map_cfg = raw
    if not isinstance(map_cfg, dict):
        return {}
    section = map_cfg.get("map")
    if isinstance(section, dict):
        merged = dict(section)
    else:
        merged = {}
        for key, value in map_cfg.items():
            name = str(key)
            if name.startswith("map."):
                merged[name[len("map."):]] = value
            elif name in ("mines", "obstacles", "targets", "bridges", "start", "end"):
                merged[name] = value
    return merged


def _cells(merged: dict, key: str) -> list[str]:
    return [str(v).strip().upper() for v in _as_list(merged.get(key)) if str(v).strip()]


# --------------------------------------------------------------------------- #
#  几何小工具（一律走 geometry，不另写换算）
# --------------------------------------------------------------------------- #
def _rc(block: str) -> tuple[int, int]:
    """格名 -> (行号 1-based, 列索引 0-based)。"""
    row_idx, col_idx = geo.block_to_index(block)
    return row_idx + 1, col_idx


def _sign(value: int) -> int:
    return (value > 0) - (value < 0)


def _blocks(route: Any) -> list[str]:
    items = getattr(route, "waypoints", None)
    if items is None:
        items = route
    out: list[str] = []
    for wp in items or []:
        block = getattr(wp, "block", None)
        if block is None and isinstance(wp, str):
            block = wp
        out.append(str(block).strip().upper() if block is not None else "")
    return out


def _barrier_crossing(a: str, b: str) -> tuple[bool, float, float] | None:
    """线段 a→b 是否**真正跨越**第 6/7 行隔板，且交点落在禁行列。

    返回 ``(是否违规, 交点列索引, 交点行坐标)``；不跨越隔板返回 None。
    坐标系与 `geometry.block_to_meters` 一致：``x`` 随行号增大而减小、
    ``y`` 随列号增大而减小，所以"行"在几何上等价于 `x`。

    ★ 判据只有一条：**两个端点的行号必须分居第 6 行两侧**（``(6-r1)*(6-r2) < 0``）。
    只在第 6 行或第 7 行内横向移动（现场 `B6 -> D6 -> C6`：车在 A/B/C 通道口探出再退回）
    **不算跨越隔板** —— 否则会把合法路线误判成 R003（这是本模块第一版踩过的坑）。
    """
    r1, c1 = _rc(a)
    r2, c2 = _rc(b)
    d1 = BARRIER_ROW_HI - r1                             # 第 6 行：>0 在隔板下方，<0 在上方
    d2 = BARRIER_ROW_HI - r2
    if d1 == 0 or d2 == 0:
        return None                                      # 端点正压在隔板行上 → 不算跨越
    if d1 * d2 >= 0:
        return None                                      # 同侧
    t = d1 / (d1 - d2)
    rd = r1 + t * (r2 - r1)
    cd = c1 + t * (c2 - c1)
    blocked = cd + EPS >= geo.COL_LETTERS.index(BARRIER_COL_FROM)
    return blocked, cd, rd


def _crossing_row_of_cell(cell: str | None) -> float | None:
    """由穿洞格直接给出它所在的行号（与 `_crossing_cell` 永远一致）。"""
    if not cell:
        return None
    try:
        row, _ = _rc(cell)
    except ValueError:
        return None
    return float(row)


def _is_cross(a: str, b: str) -> bool:
    """线段 a→b 是否**朝该列推进并抵达/越过它**（即算一次进入 `CROSS_COLUMN`）。

    ★ 三个 helper（`_crossing_row` / `_crossing_cell` / `_crossing_at_column`）
      共用这一条判据，避免各写一套导致口径打架（本模块前几版反复踩的坑）。

    判据：**把 K 列夹在两端之间（含恰好落在某个端点上）**，只排除"起点就在 K 列上、
    而且往东/沿列走"这两种情形（那只是**从洞里出来**或**在洞里横着走**，不是一次新的进入）：

    - `L9 -> K9`（列 11→10）→ 真（**东侧进洞**，过洞前半）
    - `K9 -> J9`（列 10→9） → 真（过洞后半）
    - `L9 -> J9`（列 11→9） → 真（一次跨两格，先跨进 K 再从 K 出去）
    - `J9 -> K9`（列 9→10） → 真（从西侧推进到洞口）
    - `K9 -> L9`（列 10→11）→ **假**（只是从洞里出来，没有再进入）
    - `K9 -> K10`（同列挪格）→ **假**（在洞里横着走，由 R009 单独报）
    - 任何斜切（行号在变）→ 真，但角度 > 10° 会被 R009 拦下

    于是 `K9 -> J9` 与 `K9 -> L9` 被正确区分开：两者几何相同，**方向相反** ——
    前者是"穿过 K 继续走"，后者是"离开 K 往外走"。

    ★★ 2026-10-11 修正（t7；实证见 `_tmp_session/t3_verify/t3_报告_全量回归与复核.md` §5.1）：
      旧判据写成 `min(c1, c2) < k_col <= max(c1, c2)` —— 一端**恰好压在 K 列上**时
      `min == k_col` ⇒ `min < k_col` 恒假 ⇒ **东侧进洞（`L→K`）整类漏判**：
      不仅 `L8 -> K7` 这种 45° 斜切永远进不了 R009，连 docstring 自举的第一个例子
      `L9 -> K9`（标"真"）实测都是 `False` —— 代码与文档互相矛盾，而且**方向不对称**：
      西侧进洞 `J→K` 为真、东侧进洞 `L→K` 为假。现在两端点对称：`L→K` 与 `J→K` 同等对待，
      只把"从 K 往东离开"`K→L` 与"K 列内挪格"`K→K` 排除在外。
    """
    _, c1 = _rc(a)
    _, c2 = _rc(b)
    k_col = geo.COL_LETTERS.index(CROSS_COLUMN)
    if not (min(c1, c2) <= k_col <= max(c1, c2)):
        return False
    # 起点就在 K 列上、且没有往西走（`K→L` 出洞 / `K9→K10` 列内挪格）⇒ 不算一次新的进入
    return not (c1 == k_col and c2 >= k_col)


def _crossing_row(a: str, b: str) -> float | None:
    """线段 a→b **在 `CROSS_COLUMN` 列那一刻**所处的行号；不算进入该列返回 None。

    ★ 不能拿两端点的行号来判断"穿的是哪一行"：现场 `L9 -> J9` 这种长段直线，
    两端行号都是 9 看着没问题，但 `L5 -> J5` 只有跨完列才看得出穿的是第 5 行。
    判据必须落在**与 K 列相交的那一点**上。
    """
    if not _is_cross(a, b):
        return None
    r1, c1 = _rc(a)
    r2, c2 = _rc(b)
    k_col = geo.COL_LETTERS.index(CROSS_COLUMN)
    if c2 == k_col:
        return float(r2)
    if c1 == k_col:
        return float(r1)
    t = (k_col - c1) / (c2 - c1)
    if t < -EPS or t > 1.0 + EPS:
        return None
    return r1 + t * (r2 - r1)


def _crossing_cell(a: str, b: str) -> str | None:
    """线段 a→b 进入 `CROSS_COLUMN` 时落在该列上的那一格，否则 None。

    ★ 必须把**车实际穿过的那一格**记下来：`L9 -> J9` 这种跨两格的长段，两端都不是 K 格，
    但车确实从 K9 里穿了过去 —— 只看端点会让 R008/R007 漏判（本模块调试中踩过的坑）。
    """
    row = _crossing_row(a, b)
    if row is None:
        return None
    k_col = geo.COL_LETTERS.index(CROSS_COLUMN)
    idx = int(round(row))
    idx = min(max(idx, 1), geo.MAP_ROWS)
    return geo.index_to_block(idx - 1, k_col)


def _crossing_at_column(a: str, b: str) -> tuple[float, float, float, float] | None:
    """线段 a→b 与列 `CROSS_COLUMN` 的交点。

    返回 ``(row, angle_in_deg, angle_out_deg, col_diff)``；**只对真正的穿越**返回非 None
    （口径与 `_is_cross` 完全一致：`K9->L9` 这类"只是从洞里出来"的段返回 None）。

    ★ 角度语义（现场硬约束，见 `config/robot.yaml` 的 `bridge_order` 注释）：
    "穿洞方向" = **只换行号、不换列**（沿用 `geometry` 的 x = 行轴）。
    所以：

    - 水平直穿（列差非 0、行号不变）→ 进入角 0°、离开角 0°（**唯一合法形态**）；
    - 斜切（行号与列同时变）→ 角度 = `atan2(|Δ列|, |Δ行|)`，超过 10° 即 R009。
    """
    if not _is_cross(a, b):
        return None
    r1, c1 = _rc(a)
    r2, c2 = _rc(b)
    k_col = geo.COL_LETTERS.index(CROSS_COLUMN)
    if c2 == k_col:
        row = float(r2)
    elif c1 == k_col:
        row = float(r1)
    else:
        t = (k_col - c1) / (c2 - c1)
        if t < -EPS or t > 1.0 + EPS:
            return None
        row = r1 + t * (r2 - r1)
    straight = abs(r2 - r1) < EPS                        # 严格水平直穿
    # 直线段与"穿洞方向"（行轴）的夹角：水平 = 0°，斜切 = atan2(|Δ列|, |Δ行|)
    angle = 0.0 if straight else math.degrees(math.atan2(abs(c2 - c1), abs(r2 - r1)))
    return row, angle, angle, float(c2 - c1)


def _crossing_row_of(blocks: Sequence[str]) -> float | None:
    """一串格名里，位于 `CROSS_COLUMN` 列上的那一格的行号（诊断/日志用）。

    注意：R007 的"穿的是哪一行"**不要**用它 —— 长段直线（`L5 -> J5`）两端都不是 K 格，
    必须用 `_crossing_row` 算交点所在的行。
    """
    for block in blocks:
        try:
            row, col = _rc(block)
        except ValueError:
            continue
        if col == geo.COL_LETTERS.index(CROSS_COLUMN):
            return float(row)
    return None


def crossing_entries(blocks: Sequence[str]
                     ) -> list[tuple[str, str, tuple[float, float, float, float]]]:
    """逐段给出**一次过洞各一条**的命中记录：`(a, b, (行号, 入洞角, 出洞角, Δ列))`。

    ★ 为什么需要它（2026-10-11 t7 新增）：`_crossing_at_column` 对**一次过洞的两段**
      都返回非 None —— 进洞那一半（`L7 -> K7`）与出洞那一半（`K7 -> J7`）；
      `validate_route` 靠 run 合并（同格、同行、`prev["dir"]` 同向）把它们拼回**一次**（R007）。
      需要"**一次过洞 = 一条记录**"的调用方（规划器自检 `planner.check_route_structure`、
      测试 `test_planner.audit`）请用本函数，**不要**各自重新发明"只数进洞那一半"的规则 ——
      两套口径必然打架（本模块前几版反复踩的坑）。
      ⚠️ 背景：t7 把"东侧进洞 `L→K`"从整类漏判改成算数之后，两半都可见了；
      那些"按段数计过洞次数"的调用方若不换用本函数，现场路线会被数成 **2 次**。

    口径：只收**起点不在 `CROSS_COLUMN` 列上**的那一段 —— 它正是"朝 K 列推进 / 跨过 K"
    的进洞段；另一半（`K -> *`）是"从洞里出来"，不是新的一次进入。
    规划器起点固定为 `map.start`（现场 N10，不在 K 列），故本函数与 R007 的 run 计数
    在上述调用方的输入上**逐例等价**。
    """
    k_col = geo.COL_LETTERS.index(CROSS_COLUMN)
    out: list[tuple[str, str, tuple[float, float, float, float]]] = []
    for a, b in zip(blocks, blocks[1:]):
        hit = _crossing_at_column(a, b)
        if hit is None:
            continue
        if _rc(a)[1] == k_col:                              # 出洞那一半：不算新的一次进入
            continue
        out.append((a, b, hit))
    return out


# --------------------------------------------------------------------------- #
#  校验主入口
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Issue:
    """一条校验发现。`code` 是稳定编号（R001~R014），测试与文档引用它。"""

    level: str      # "error" | "warn"
    code: str
    msg: str

    def __str__(self) -> str:
        mark = "❌" if self.level == "error" else "⚠️"
        return f"{mark} [{self.code}] {self.msg}"


def validate_route(route: Any, *, map_cfg: Any = None, cfg: Config | None = None,
                   cross_candidates: Iterable[str] | None = None) -> list[Issue]:
    """出发前校验。返回 `list[Issue]`，**空列表 = 可以发车**。

    :param route: `Route`（也接受任何有 `.waypoints` 的对象，或格名序列）
    :param map_cfg: `config/map.yaml` 的原始 dict（`{'map': {...}}`）—— 雷/障/靶/桥的唯一真源
    :param cfg: `core.config.Config`（点号键）；射击窗口、穿洞候选集等全部从这里读
    :param cross_candidates: 显式覆盖穿洞候选格集合（默认取 `cfg` 的
        `route.cross_lower_candidates/route.cross_upper_candidates`）
    """
    issues: list[Issue] = []
    blocks = _blocks(route)
    n = len(blocks)

    # ---------------------------------------------------------------- R001
    if n < 2:
        issues.append(Issue("error", "R001",
                            f"路线为空或少于 2 点（实际 {n} 点），无法循迹"))
        return issues                                    # 后面所有检查都依赖几何，直接收工

    merged = _map_section(map_cfg)
    mines = _cells(merged, "mines")
    obstacles = _cells(merged, "obstacles")
    targets = _cells(merged, "targets")
    lower_rows, upper_rows = _cross_rows(cfg, cross_candidates)
    cross_pool = _candidates(cfg, cross_candidates)
    lo_m, hi_m = _window(cfg)
    #: 两个半场的行号并集 —— R007 之外的检查里只用来判断"某行是否属于某个半场"
    cross_rows = set(lower_rows) | set(upper_rows)
    k_col = geo.COL_LETTERS.index(CROSS_COLUMN)

    # ---------------------------------------------------------------- R002
    valid: list[tuple[int, str]] = []
    for idx, block in enumerate(blocks):
        wp = route[idx] if hasattr(route, "__getitem__") else None
        token = str(getattr(wp, "token", "") or "") if wp is not None else ""
        raw = token or block
        if not block:
            issues.append(Issue("error", "R002", f"第 {idx} 个航点没有格名"))
            continue
        try:
            geo.block_to_index(block)
        except ValueError as exc:
            issues.append(Issue("error", "R002",
                                f"第 {idx} 个航点格名非法（token={raw!r}）：{exc}"))
            continue
        if token:
            try:
                from .parser import parse_token                # 局部导入：避免顶层循环依赖
                info = parse_token(token)
            except ValueError as exc:
                issues.append(Issue("error", "R002",
                                    f"第 {idx} 个航点 token 非法（{token!r}）：{exc}"))
                continue
            if info["shoot_at"] or info["block"] != block:
                issues.append(Issue("error", "R002",
                                    f"第 {idx} 个航点 token {token!r} 与格名 {block!r} 不一致"))
                continue
        valid.append((idx, block))

    # 起点检查（现场必须从 N10 出发，否则里程计硬锚点与实际不符）
    start_block = str(merged.get("start") or "").strip().upper()
    if start_block and valid and valid[0][1] != start_block:
        issues.append(Issue("warn", "R002",
                            f"路线起点 {valid[0][1]} 与地图起点 {start_block} 不一致"
                            f"（里程计开机硬锚在 {start_block}，起点不符会全程错位）"))

    # ---------------------------------------------------------------- R010（相邻重复）
    for i in range(len(valid) - 1):
        if valid[i][1] == valid[i + 1][1]:
            issues.append(Issue("error", "R010",
                                f"第 {valid[i][0]}/{valid[i + 1][0]} 个航点重复（{valid[i][1]}）"
                                f"—— parser 应当已经合并，检查是否手工构造了路线"))

    # ---------------------------------------------------------------- R003（隔板硬约束）
    for i in range(len(valid) - 1):
        a, b = valid[i][1], valid[i + 1][1]
        hit = _barrier_crossing(a, b)
        if hit is None:
            continue
        bad, cd, rd = hit
        if bad:
            col_name = geo.COL_LETTERS[min(geo.MAP_COLS - 1, max(0, int(round(cd))))]
            issues.append(Issue("error", "R003",
                                f"第 {i} 段 {a}->{b} 跨越第 {BARRIER_ROW_HI}/{BARRIER_ROW_LO} 行隔板"
                                f"（交点行 {rd:.2f}、列 {col_name}，禁行列 {BARRIER_COL_FROM}~N）"
                                f"—— 只有 {'/'.join(U_TURN_COLUMNS)} 列能上下穿行"))

    # ---------------------------------------------------------------- R004（踩障碍）
    obstacle_set = set(obstacles)
    if obstacle_set:
        for idx, block in valid:
            if block in obstacle_set:
                issues.append(Issue("error", "R004",
                                    f"第 {idx} 个航点 {block} 踩到障碍格"
                                    f"（map.obstacles 共 {len(obstacles)} 个）"))

    # ---------------------------------------------------------------- R005（雷覆盖）
    if mines:
        mine_set = set(mines)
        route_set = {block for _, block in valid}
        covered = sorted(mine_set & route_set)
        missing = sorted(mine_set - route_set)
        if len(covered) != len(mine_set):
            issues.append(Issue("error", "R005",
                                f"雷覆盖 {len(covered)}/{len(mine_set)}，漏 "
                                f"{' '.join(missing)}（路线必须压过每一颗雷）"))

    # ---------------------------------------------------------------- R007 / R008 / R009（穿洞）
    # ① 把**连续相接**的穿越段并成一次"过洞"（L9->K9 与 K9->J9 是同一次穿越的两段）
    runs: list[dict] = []
    prev: dict | None = None
    for i in range(len(valid) - 1):
        a, b = valid[i][1], valid[i + 1][1]
        # ★ 半步也算：`L9->K9` 是过洞的**前半**（跨到 K 格），`K9->J9` 是**后半**（从 K 格跨出去），
        #   两者靠下面的合并拼回**一次**过洞；`J9->K9` 只是"停在洞里"、`K9->L9` 只是"从洞里出来"，
        #   都不是过洞（`_crossing_cell`/`_crossing_row` 对它们返回 None）。
        ca, cb = _rc(a)[1], _rc(b)[1]
        ra, rb = _rc(a)[0], _rc(b)[0]
        along_k = ca == cb == k_col and ra != rb       # 在洞里沿列挪格（必撞侧壁）
        if ca == cb == k_col:
            moves = ra != rb
        else:                                          # 跨列：一端在 K 列上、另一端在两侧之一
            moves = ca != cb
        cell = _crossing_cell(a, b)
        cross_row = _crossing_row_of_cell(cell)        # ★ 行号一律由穿洞格反推，两者永远一致
        if not moves or cell is None or cross_row is None:
            prev = None
            continue
        if along_k:
            issues.append(Issue("error", "R009",
                                f"第 {i} 段 {a}->{b} 沿 {CROSS_COLUMN} 列**列内**行驶"
                                f"（第 {ra}->{rb} 行）—— 桥洞只有单格宽度，必须严格直穿"))
            prev = None
            continue
        # 与前一次过洞相接（从刚穿到的那一格继续往外走，还是同一格、同一行、**同一方向**）→ 合并
        #   ★ 方向必须一致：`J6->K6->J9` 是在 K6 里掉了个头（两次反向穿越），
        #     若只按"格 + 行"合并，会被拼成一次方向混乱的假穿越。
        direction = (_sign(cb - ca), _sign(rb - ra))
        if (prev is not None and prev["segs"][-1][1] == a
                and prev["cell"] == cell
                and int(round(prev["row"])) == int(round(cross_row))
                and prev["dir"] == direction):
            prev["segs"].append((a, b))
            prev["b"] = b
            hit2 = _crossing_at_column(a, b)
            if hit2 is not None:
                prev["col_diff"] += hit2[3]
            continue
        # ★ 这里**不按行号过滤**：行号不在候选范围内恰恰是最该报 R008 的情形，
        #   先滤掉就再也看不见了（第三版踩过的坑）。
        hit = _crossing_at_column(a, b)
        if hit is None:
            prev = None
            continue
        prev = {"segs": [(a, b)], "a": a, "b": b, "index": i, "col_diff": hit[3],
                "row": cross_row, "cell": cell, "dir": direction}
        runs.append(prev)

    for run in runs:
        # ★ 进来的角度与出去的角度都要看：`J9->K8->H9` 这种"折线穿洞"两段都斜，
        #   只取其中一段会漏判（本模块调试中踩过的坑）。
        angles = []
        for seg_a, seg_b in run["segs"]:
            hit = _crossing_at_column(seg_a, seg_b)
            if hit is None:
                continue
            _, a_in, a_out, _ = hit
            angles.append(a_in)
            if a_out > 0.0:
                angles.append(a_out)
        run["angle"] = max(angles) if angles else 0.0
        run["cells"] = {blk for seg in run["segs"] for blk in seg}
        crossed = [c for c in (_crossing_cell(sa, sb) for sa, sb in run["segs"]) if c]
        run["cells"] |= set(crossed)
        run["k_cells"] = sorted(set(crossed))
        row = int(round(run["row"]))
        # ★ t31 权威门放宽（按 t11 §10 建议①、t23 复核）：穿洞行的硬门 = **本半场行段**
        #   （下 7~11 / 上 1~6），不再要求"落在候选行"上 —— 候选集合 / `map.bridges`
        #   只用于挑**缺省洞**，可以过期，由无人机的新鲜情报纠正（见 R008 报文与
        #   `tasks/mission.py::_intel_cell_ok`、`route/planner.py` 的同口径硬门）。
        if CROSS_LOWER_ROWS[0] <= row <= CROSS_LOWER_ROWS[1]:
            run["half"] = "lower"
            run["band_ok"] = True
        elif CROSS_UPPER_ROWS[0] <= row <= CROSS_UPPER_ROWS[1]:
            run["half"] = "upper"
            run["band_ok"] = True
        else:                                              # 不该发生：行号只有 1~11
            run["half"] = "other"
            run["band_ok"] = False

    lower_runs = [r for r in runs if r["half"] == "lower"]
    upper_runs = [r for r in runs if r["half"] == "upper"]

    # ★ R008（t31 后语义）：**只**看"穿洞行是否在 K 列本半场行段内"（下半场 7~11 /
    #   上半场 1~6），候选行/候选集合不再当硬门；报文里仍打印候选行供现场对账。
    #   "不在候选集合"因此**不再是 error** —— 那正是本卡要放开的（无人机情报可纠正过期地图）。

    # ★ R009：**每一次**过洞都要看夹角，不能只查 R007 那个"唯一一次"——
    #   现场一旦多了一次斜切（例如某段折线斜着蹭过 K 列），只查唯一一次就会漏掉。
    for run in runs:
        if run["angle"] > MAX_CROSS_ANGLE_DEG + EPS:
            issues.append(Issue("error", "R009",
                                f"第 {run['index']} 段 {run['a']}->{run['b']} 斜切入洞：与穿洞方向"
                                f"夹角 {run['angle']:.1f}°（上限 {MAX_CROSS_ANGLE_DEG:.0f}°），"
                                f"在 {CROSS_COLUMN} 列第 {run['row']:.2f} 行穿行 —— 会撞侧壁"))

    def _fmt(run_list: list[dict]) -> str:
        return " ".join(f"{r['a']}->{r['b']}@{int(round(r['row']))}行" for r in run_list)

    if len(lower_runs) != 1:
        issues.append(Issue("error", "R007",
                            f"下半场（第 {CROSS_LOWER_ROWS[0]}~{CROSS_LOWER_ROWS[1]} 行）"
                            f"应恰好 1 次穿越 {CROSS_COLUMN} 列，实际 {len(lower_runs)} 次"
                            f"（{_fmt(lower_runs) or '无'}）"))
    elif lower_runs[0]["col_diff"] >= 0:
        run = lower_runs[0]
        issues.append(Issue("error", "R007",
                            f"下半场穿越方向错：{run['a']}->{run['b']} 必须**向西**"
                            f"（列号减小，现场 bridge_order: L* -> K* -> J*）"))
    if len(upper_runs) != 1:
        issues.append(Issue("error", "R007",
                            f"上半场（第 {CROSS_UPPER_ROWS[0]}~{CROSS_UPPER_ROWS[1]} 行）"
                            f"应恰好 1 次穿越 {CROSS_COLUMN} 列，实际 {len(upper_runs)} 次"
                            f"（{_fmt(upper_runs) or '无'}）"))
    else:
        run = upper_runs[0]
        if run["col_diff"] <= 0:
            issues.append(Issue("error", "R007",
                                f"上半场穿越方向错：{run['a']}->{run['b']} 必须**向东**"
                                f"（列号增大，现场 bridge_order: J* -> K* -> L*）"))

    if not runs:
        issues.append(Issue("error", "R007",
                            f"路线没有任何一次穿越 {CROSS_COLUMN} 列（两个半场各需一次）"))

    for run in runs:
        if not run["band_ok"]:
            # ★ t31：报文写"行段"（硬门），候选行只作参考 —— 旧文"而候选行只有 …"会让现场
            #   以为是候选集合把关（已放开：无人机新鲜情报可纠正过期地图）。
            issues.append(Issue("error", "R008",
                                f"第 {run['index']} 段 {run['a']}->{run['b']} 在 {CROSS_COLUMN} 列第 "
                                f"{run['row']:.0f} 行穿行，不在本半场行段"
                                f"（下 {CROSS_LOWER_ROWS[0]}~{CROSS_LOWER_ROWS[1]} / "
                                f"上 {CROSS_UPPER_ROWS[0]}~{CROSS_UPPER_ROWS[1]} 行）里"
                                f"（候选行 下半场 {'/'.join(str(r) for r in lower_rows)}、"
                                f"上半场 {'/'.join(str(r) for r in upper_rows)} 仅供参考）"))

    # ② 整段都在 K 列里、还沿列挪格（在洞里横着走）—— 已在上面的 run 构建里逐段报过，
    #    这里只在"该段没被 run 收录"时兜底（防御性：逻辑改动时不会静默漏掉）
    reported_k = {tuple(seg) for run in runs for seg in run["segs"]}
    for i in range(len(valid) - 1):
        a, b = valid[i][1], valid[i + 1][1]
        ra, ca = _rc(a)
        rb, cb = _rc(b)
        if ca != k_col or cb != k_col or ra == rb or (a, b) in reported_k:
            continue
        issues.append(Issue("error", "R009",
                            f"第 {i} 段 {a}->{b} 沿 {CROSS_COLUMN} 列**列内**行驶"
                            f"（第 {ra}->{rb} 行）—— 桥洞只有单格宽度，必须严格直穿"))

    # ---------------------------------------------------------------- R012（U 形绕行）
    u_cols = {geo.COL_LETTERS.index(c) for c in U_TURN_COLUMNS}
    if valid and not any(_rc(block)[1] in u_cols for _, block in valid):
        issues.append(Issue("error", "R012",
                            f"路线完全没有经过 {'/'.join(U_TURN_COLUMNS)} 列"
                            f"—— 没有绕过第 {BARRIER_ROW_HI}/{BARRIER_ROW_LO} 行隔板"
                            f"（{BARRIER_COL_FROM}~N 列封死）"))

    # ---------------------------------------------------------------- R006 / R013（靶）
    shoot_points = [(idx, wp) for idx, wp in enumerate(getattr(route, "waypoints", []) or [])
                    if getattr(wp, "shoot", False)]
    if targets:
        if not shoot_points:
            issues.append(Issue("error", "R006",
                                f"路线没有任何射击停站点，靶 {' '.join(targets)} 无法被打到"))
        else:
            covered = []
            for _, wp in shoot_points:
                for tgt in targets:
                    if _stop_ok(wp, tgt, lo_m, hi_m):
                        if tgt not in covered:
                            covered.append(tgt)
            if not covered:
                issues.append(Issue("error", "R006",
                                    f"靶 {' '.join(targets)} 未被任何射击停站点覆盖"
                                    f"（窗口 {lo_m:.2f}~{hi_m:.2f} m）"))
            for idx, wp in shoot_points:
                if not any(_stop_ok(wp, tgt, lo_m, hi_m) for tgt in targets):
                    dists = " ".join(f"{tgt}={_euc(wp, tgt):.3f}m" for tgt in targets)
                    issues.append(Issue("error", "R013",
                                        f"射击停站点[{idx}] {getattr(wp, 'block', '?')} 距靶不在窗口内"
                                        f"（{dists}；窗口 {lo_m:.2f}~{hi_m:.2f} m）"))
                elif _arrives_reverse(route, idx) and not any(
                        _stop_ok(wp, tgt, lo_m, math.inf) for tgt in targets):
                    # 倒车到位时车尾朝靶，"车心到靶心"必须仍在窗口上限内
                    dists = " ".join(f"{tgt}={_euc(wp, tgt):.3f}m" for tgt in targets)
                    issues.append(Issue("error", "R013",
                                        f"射击停站点[{idx}] {getattr(wp, 'block', '?')} 倒车到位，"
                                        f"车尾朝靶使有效距离超出窗口上限（{dists}；上限 {hi_m:.2f} m）"))

    # ---------------------------------------------------------------- R011（段长）
    # ★ 长段**汇总成一条** warn：现场路线天然含 3 处长段（F8->B8 / C2->G2 / J1->J4），
    #   逐段刷屏会把真正的问题淹掉 —— `P0接口约定.md` §2 要求"R011 至多 1 条 warn"。
    long_segments: list[tuple[float, int, str, str]] = []
    for i in range(len(valid) - 1):
        a, b = valid[i][1], valid[i + 1][1]
        ax, ay = geo.block_to_meters(a)
        bx, by = geo.block_to_meters(b)
        length = math.hypot(bx - ax, by - ay)
        if length < MIN_SEGMENT_M:
            issues.append(Issue("error", "R011",
                                f"第 {i} 段 {a}->{b} 长度 {length:.4f} m < {MIN_SEGMENT_M} m"
                                f"（重复点？）"))
        elif length / geo.BLOCK_SIZE > MAX_SEGMENT_BLOCKS:
            long_segments.append((length / geo.BLOCK_SIZE, i, a, b))
    if long_segments:
        longest = max(long_segments)
        detail = "、".join(f"{a}->{b}({cells:.2f}格)" for cells, _, a, b in long_segments)
        issues.append(Issue("warn", "R011",
                            f"有 {len(long_segments)} 段超过 {MAX_SEGMENT_BLOCKS} 格，疑似漏点"
                            f"（最长：第 {longest[1]} 段 {longest[2]}->{longest[3]} "
                            f"{longest[0]:.2f} 格 / {longest[0] * geo.BLOCK_SIZE:.3f} m）"
                            f"：{detail}"))

    # ---------------------------------------------------------------- R014（倒车占比）
    reverse_m = 0.0
    for i, wp in enumerate(getattr(route, "waypoints", []) or []):
        if i + 1 >= n or not getattr(wp, "reverse", False):
            continue
        ax, ay = geo.block_to_meters(blocks[i])
        bx, by = geo.block_to_meters(blocks[i + 1])
        reverse_m += math.hypot(bx - ax, by - ay)
    total_m = _total_length(blocks)
    if total_m > EPS and reverse_m / total_m > MAX_REVERSE_RATIO:
        issues.append(Issue("warn", "R014",
                            f"倒车段占比 {reverse_m / total_m * 100:.1f}%（{reverse_m:.2f}/"
                            f"{total_m:.2f} m）> {MAX_REVERSE_RATIO * 100:.0f}%"
                            f"—— 倒车慢且舵向无实车依据，核对路线"))

    return issues


def _euc(wp: Any, block: str) -> float:
    tx, ty = geo.block_to_meters(block)
    return math.hypot(float(getattr(wp, "x", 0.0)) - tx, float(getattr(wp, "y", 0.0)) - ty)


def _stop_ok(wp: Any, target: str, lo: float, hi: float) -> bool:
    """射击停站点到靶格的距离是否落在 ``[lo, hi]``。"""
    dist = _euc(wp, target)
    return lo - 1e-9 <= dist <= hi + 1e-9


def _arrives_reverse(route: Any, idx: int) -> bool:
    """第 idx 个航点是否是**倒车到位**的（段 idx-1 → idx 带 R:）。"""
    wps = getattr(route, "waypoints", None) or []
    return 0 < idx < len(wps) and bool(getattr(wps[idx], "reverse", False))


def _total_length(blocks: Sequence[str]) -> float:
    total = 0.0
    for a, b in zip(blocks, blocks[1:]):
        try:
            ax, ay = geo.block_to_meters(a)
            bx, by = geo.block_to_meters(b)
        except ValueError:
            continue
        total += math.hypot(bx - ax, by - ay)
    return total


def summary(issues: Iterable[Issue]) -> str:
    """人类可读汇总（含 error/warn 计数）；可直接打进启动横幅。"""
    items = list(issues)
    errors = [i for i in items if i.level == "error"]
    warns = [i for i in items if i.level == "warn"]
    if not items:
        return "✅ 路线校验通过：0 error / 0 warn"
    head = (f"{'❌' if errors else '⚠️'} 路线校验：{len(errors)} error / {len(warns)} warn"
            + ("（**不可发车**）" if errors else "（可发车，但有提醒）"))
    lines = [head]
    for issue in errors + warns:
        lines.append("  " + str(issue))
    return "\n".join(lines)
