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
    "Issue", "validate_route", "summary",
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


def _cross_rows(cfg: Config | None) -> tuple[list[int], list[int]]:
    """穿洞候选行：**优先从候选格名集合反推行号**（与现场 YAML 永远同步）。"""
    lower: list[int] = []
    upper: list[int] = []
    for raw in _as_list(_cfg_get(cfg, _KEY_CROSS_LOWER, None)):
        try:
            row_idx, _ = geo.block_to_index(raw)
        except ValueError:
            continue
        lower.append(row_idx + 1)
    for raw in _as_list(_cfg_get(cfg, _KEY_CROSS_UPPER, None)):
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
    pool |= {str(_cfg_get(cfg, _KEY_CROSS_LOWER_DEFAULT, "K9")).upper(),
             str(_cfg_get(cfg, _KEY_CROSS_UPPER_DEFAULT, "K4")).upper()}
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


def _crossing_at_column(a: str, b: str) -> tuple[float, float, float, float] | None:
    """线段 a→b 与列 `CROSS_COLUMN` 的交点。

    返回 ``(row, angle_in_deg, angle_out_deg, col_diff)``；不穿越返回 None。

    ★ 角度语义（现场硬约束，见 `config/robot.yaml` 的 `bridge_order` 注释）：
    "穿洞方向" = **只换行号、不换列**（沿用 `geometry` 的 x = 行轴）。
    所以：

    - 水平直穿（列差非 0、行号不变）→ 进入角 0°、离开角 0°（**唯一合法形态**）；
    - 沿列行驶（行号在变）→ 进入角 = 离开角 = 90°（在洞里横着走，必撞侧壁）；
    - 斜切（行号与列同时变）→ 进入角 = `atan2(|Δ列|, |Δ行|)`，超过 10° 即 R009。
    """
    r1, c1 = _rc(a)
    r2, c2 = _rc(b)
    k_col = geo.COL_LETTERS.index(CROSS_COLUMN)
    if c1 == c2:
        if c1 != k_col:
            return None
        # 整段都在 K 列内：沿列方向行驶 = 90°（在洞里横着走）
        angle = 0.0 if r1 == r2 else 90.0
        return float(max(r1, r2)), angle, angle, 0.0
    if (c1 - k_col) * (c2 - k_col) > 0:                  # 同侧，不穿越
        return None
    t = (k_col - c1) / (c2 - c1)
    if t < -EPS or t > 1.0 + EPS:
        return None
    row = r1 + t * (r2 - r1)
    # 行号不变（严格水平）→ 0°；行号随列号一起变（斜切）→ atan2(|Δ列|, |Δ行|)
    angle_in = 0.0 if abs(r2 - r1) < EPS else \
        math.degrees(math.atan2(abs(c2 - c1), abs(r2 - r1)))
    angle_out = 0.0 if c2 == k_col else angle_in
    return row, angle_in, angle_out, float(c2 - c1)


def _crossing_row_of(blocks: Sequence[str]) -> float | None:
    """一段"穿越"里位于 `CROSS_COLUMN` 列上的那个格子的行号（可能是"洞前/洞后"两侧）。"""
    for block in blocks:
        try:
            row, col = _rc(block)
        except ValueError:
            continue
        if col == geo.COL_LETTERS.index(CROSS_COLUMN):
            return float(row)
    return None


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
    lower_rows, upper_rows = _cross_rows(cfg)
    cross_pool = _candidates(cfg, cross_candidates)
    lo_m, hi_m = _window(cfg)

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
    cross_rows = set(lower_rows) | set(upper_rows)
    k_col = geo.COL_LETTERS.index(CROSS_COLUMN)

    # ① 把**连续相接**的穿越段并成一次"过洞"（L9->K9 与 K9->J9 是同一次穿越的两段）
    runs: list[dict] = []
    prev: dict | None = None
    for i in range(len(valid) - 1):
        a, b = valid[i][1], valid[i + 1][1]
        hit = _crossing_at_column(a, b)
        if hit is None or (cross_rows and int(round(hit[0])) not in cross_rows):
            prev = None
            continue
        if prev is not None and prev["segs"][-1][1] == a:      # 与上一段首尾相接
            prev["segs"].append((a, b))
            prev["b"] = b
            prev["col_diff"] += hit[3]
        else:
            prev = {"segs": [(a, b)], "a": a, "b": b, "index": i, "col_diff": hit[3]}
            runs.append(prev)

    for run in runs:
        # ★ 只统计**跨越 K 列那一刻**的角度：进来一段、出去一段。
        #   整段落在 K 列内、只是沿列挪格的那一段（K9->K10）不算"斜切"，另由 R007 报方向错。
        angles = []
        for seg_a, seg_b in run["segs"]:
            hit = _crossing_at_column(seg_a, seg_b)
            if hit is None:
                continue
            _, a_in, a_out, _ = hit
            angles.append(a_in)
            if a_out > 0.0:
                angles.append(a_out)
        row = _crossing_row_of([blk for seg in run["segs"] for blk in seg])
        run["row"] = row if row is not None else float("nan")
        run["angle"] = max(angles) if angles else 0.0
        run["cells"] = {blk for seg in run["segs"] for blk in seg}

    lower_runs = [r for r in runs if int(round(r["row"])) in set(lower_rows)]
    upper_runs = [r for r in runs if int(round(r["row"])) in set(upper_rows)]

    def _fmt(run_list: list[dict]) -> str:
        return " ".join(f"{r['a']}->{r['b']}@{int(round(r['row']))}行" for r in run_list)

    if len(lower_runs) != 1:
        issues.append(Issue("error", "R007",
                            f"下半场（第 {CROSS_LOWER_ROWS[0]}~{CROSS_LOWER_ROWS[1]} 行）"
                            f"应恰好 1 次穿越 {CROSS_COLUMN} 列，实际 {len(lower_runs)} 次"
                            f"（{_fmt(lower_runs) or '无'}）"))
    else:
        run = lower_runs[0]
        if run["col_diff"] >= 0:
            issues.append(Issue("error", "R007",
                                f"下半场穿越方向错：{run['a']}->{run['b']} 必须**向西**"
                                f"（列号减小，现场 bridge_order: L* -> K* -> J*）"))
        if run["angle"] > MAX_CROSS_ANGLE_DEG + EPS:
            issues.append(Issue("error", "R009",
                                f"下半场入洞段 {run['a']}->{run['b']} 斜切：与穿洞方向夹角 "
                                f"{run['angle']:.1f}°（上限 {MAX_CROSS_ANGLE_DEG:.0f}°），"
                                f"在 {CROSS_COLUMN} 列第 {run['row']:.2f} 行穿行 —— 会撞侧壁"))
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
        if run["angle"] > MAX_CROSS_ANGLE_DEG + EPS:
            issues.append(Issue("error", "R009",
                                f"上半场入洞段 {run['a']}->{run['b']} 斜切：与穿洞方向夹角 "
                                f"{run['angle']:.1f}°（上限 {MAX_CROSS_ANGLE_DEG:.0f}°），"
                                f"在 {CROSS_COLUMN} 列第 {run['row']:.2f} 行穿行 —— 会撞侧壁"))

    if not runs:
        issues.append(Issue("error", "R007",
                            f"路线没有任何一次穿越 {CROSS_COLUMN} 列（两个半场各需一次）"))

    for run in runs:
        if not (run["cells"] & cross_pool):
            issues.append(Issue("error", "R008",
                                f"第 {run['index']} 段 {run['a']}->{run['b']}（{CROSS_COLUMN} 列第 "
                                f"{run['row']:.0f} 行）的穿洞格不在候选集合里"
                                f"（候选 {' '.join(sorted(cross_pool))}）"))

    # ② 不与任何穿越相接、却整段沿 K 列行驶的段：同样算"在洞里横着走"
    for i in range(len(valid) - 1):
        a, b = valid[i][1], valid[i + 1][1]
        ra, ca = _rc(a)
        rb, cb = _rc(b)
        if ca != k_col or cb != k_col or ra == rb:
            continue
        if cross_rows and ra not in cross_rows:
            continue
        if any(seg[0] == a and seg[1] == b for run in runs for seg in run["segs"]):
            continue                                          # 已由上面的 R009 报过
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
    for i in range(len(valid) - 1):
        a, b = valid[i][1], valid[i + 1][1]
        ax, ay = geo.block_to_meters(a)
        bx, by = geo.block_to_meters(b)
        length = math.hypot(bx - ax, by - ay)
        cells = length / geo.BLOCK_SIZE
        if length < MIN_SEGMENT_M:
            issues.append(Issue("error", "R011",
                                f"第 {i} 段 {a}->{b} 长度 {length:.4f} m < {MIN_SEGMENT_M} m"
                                f"（重复点？）"))
        elif cells > MAX_SEGMENT_BLOCKS:
            issues.append(Issue("warn", "R011",
                                f"第 {i} 段 {a}->{b} 长 {cells:.2f} 格（{length:.3f} m）"
                                f"> {MAX_SEGMENT_BLOCKS} 格，疑似漏点（中途无航点约束）"))

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
