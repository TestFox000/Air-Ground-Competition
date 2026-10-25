# -*- coding: utf-8 -*-
"""运动原语判定器 v2 —— **转弯内侧侧向投影** + 动作建议（ok/tight/pivot/forbid）。

移植自 V1019 路径规划交付包（原作者 yubinghuan，2026-10-20 入仓）；
方案与实测证据：docs/路径规划优化-转角物理分级-20261020.md。

硬件前提（2026-10-16 现场确认）：
    阿克曼前轮转向，δ_max=28°，R_min=260mm，车 278×140mm，格 350mm，
    距障碍**边缘** ≥50mm；倒车仅用于死胡同；雷心 ≤50mm。

v2 相对 v1 的关键修正：
    v1 的 avail 用"邻格有桶就算 225mm"的粗估 → **不区分转弯内侧/外侧** ✗，
    这正是"守卫按 A 优化、体检按 B 打分 → 越优化越差"的根因。
    v2 沿**转弯内侧**做侧向投影：
        avail = 175（半格）+ 350×(k−1) + 50（桶边余量）   k = 内侧第 k 格被挡
    并同时检查内切对角方向，取较小者。

物理基线（由参数直接算出）：
    45°  need ≈ 146mm  → 桶在邻格时 avail ≈ 225mm ✓ 永远可行
    90°  need ≈ 330mm  → 一格只给 225mm ✗ 临界/多半不可行 → 需减速/. 改 45° 折线
    135° need ≈ 514mm  → 单段绝无可能 ✗ → 只能多点掉头（需 ≥411mm 自由径）

API（与 v1 兼容）：turn_space(...) · check_route(cells, blocked) · summarize(...) · verify(...)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

CELL_MM = 350.0
LENGTH_MM = 278.0
WIDTH_MM = 140.0
DELTA_MAX_DEG = 28.0
R_MIN_MM = 260.0
MARGIN_MM = 50.0
MINE_CENTER_TOL_MM = 50.0
BUCKET_EDGE_MM = 50.0
HALF_CELL_MM = CELL_MM / 2.0
OPEN_MM = 575.0

OUTER_DIA_MM = math.hypot(LENGTH_MM, WIDTH_MM)
PIVOT_FREE_DIA_MM = OUTER_DIA_MM + 2 * MARGIN_MM

PRIM_STRAIGHT = "straight"
PRIM_ARC45 = "arc45"
PRIM_ARC90 = "arc90"
PRIM_FORBID = "forbid135"

ACT_OK, ACT_TIGHT, ACT_PIVOT, ACT_FORBID = "ok", "tight", "pivot", "forbid"

# 8 向位移 -> 朝向索引（与 mission_router._DIRS 同约定：0=东，顺时针）。
# solid 探测表按此索引取值。
_DIR8 = {
    (1, 0): 0, (1, 1): 1, (0, 1): 2, (-1, 1): 3,
    (-1, 0): 4, (-1, -1): 5, (0, -1): 6, (1, -1): 7,
}


def need_mm(delta_deg: float) -> float:
    """该转角所需的最小横向空间：R(1−cosΔθ) + 半车宽。"""
    d = math.radians(min(abs(delta_deg), 180.0))
    return R_MIN_MM * (1.0 - math.cos(d)) + WIDTH_MM / 2.0


def lateral_mm(delta_deg: float) -> float:
    return need_mm(delta_deg)


def classify(delta_deg: float) -> str:
    d = abs(delta_deg)
    if d < 1e-6:
        return PRIM_STRAIGHT
    if d <= 50.0:
        return PRIM_ARC45
    if d < 135.0:            # ★ 恰好 135° 归 forbid（单段绝无可能，只能多点掉头）
        return PRIM_ARC90
    return PRIM_FORBID


def _xy(cell: str) -> tuple[int, int]:
    return ord(cell[0]) - ord("A"), int(cell[1:]) - 1


def _name(c: int, r: int) -> str:
    return f"{chr(ord('A') + c)}{r + 1}"


def _in_field(c: int, r: int) -> bool:
    return 0 <= c < 14 and 0 <= r < 11


def _probe(cell: str, d: tuple[int, int], blocked: set[str], solid=None) -> float:
    """沿方向 d 从格心探测，返回遇到障碍**边缘**的距离（mm）；场外无围挡 → 视为开阔。

    solid（2026-10-25）：{格名: 8 向可用 mm}——中场挡板实体侵入边界两侧格
    87.5mm 的探测表（mission_router._build_solid_barriers 产）。挡板在格内，
    比邻格的桶更近，必须先于桶判。与搜索内守卫（_dijkstra）同一口径。
    """
    if solid is not None:
        row = solid.get(cell)
        if row is not None:
            dd = _DIR8.get(d)
            if dd is not None and row[dd] < OPEN_MM:
                return row[dd]
    c, r = _xy(cell)
    for k in (1, 2):
        nc, nr = c + d[0] * k, r + d[1] * k
        if not _in_field(nc, nr):
            return OPEN_MM
        if _name(nc, nr) in blocked:
            return HALF_CELL_MM + (k - 1) * CELL_MM + BUCKET_EDGE_MM
    return OPEN_MM


def _inside_dir(v_in: tuple[int, int], v_out: tuple[int, int]) -> tuple[int, int] | None:
    """转弯内侧方向（左转→v_in 逆时针 90°；右转→顺时针 90°）；共线返回 None。"""
    cross = v_in[0] * v_out[1] - v_in[1] * v_out[0]
    if cross > 0:
        return (-v_in[1], v_in[0])
    if cross < 0:
        return (v_in[1], -v_in[0])
    return None


def _clearance_inside(cell: str, v_in: tuple[int, int], v_out: tuple[int, int],
                      blocked: set[str], solid=None) -> tuple[float, str]:
    side = _inside_dir(v_in, v_out)
    if side is None:
        return OPEN_MM, "直行"
    a1 = _probe(cell, side, blocked, solid)
    # 内切对角方向（弧线从内侧前方擦过）：取 v_out 与 side 的合成方向
    dx = v_out[0] + side[0]
    dy = v_out[1] + side[1]
    dg = (1 if dx > 0 else -1, 1 if dy > 0 else -1)
    a2 = _probe(cell, dg, blocked, solid)
    return min(a1, a2), ("左转内侧" if side == (-v_in[1], v_in[0]) else "右转内侧")


def turn_space(cell: str, v_in: tuple[int, int], v_out: tuple[int, int],
               blocked=(), solid=None) -> dict:
    """★ 单一权威判据：一个转角的空间需求/可用/余量/动作。

    solid（2026-10-25）：实体挡板探测表（{格名: 8 向 mm}）——给出时挡板
    实体先于桶判（87.5mm < 桶 225mm），与 mission_router 搜索内守卫同口径。
    """
    bl = set(blocked)
    a1 = math.atan2(v_in[1], v_in[0])
    a2 = math.atan2(v_out[1], v_out[0])
    deg = math.degrees(abs((a2 - a1 + math.pi) % (2 * math.pi) - math.pi))
    if deg < 1e-6:
        return {"delta_deg": 0.0, "primitive": PRIM_STRAIGHT, "need_mm": 0.0,
                "avail_mm": OPEN_MM, "margin_mm": OPEN_MM, "action": ACT_OK, "side": "直行"}
    avail, side = _clearance_inside(cell, v_in, v_out, bl, solid)
    nd = need_mm(deg)
    margin = avail - nd
    prim = classify(deg)
    if prim == PRIM_FORBID:
        act = ACT_FORBID
    elif margin < 0:
        act = ACT_PIVOT
    elif margin < MARGIN_MM:
        act = ACT_TIGHT
    else:
        act = ACT_OK
    return {"delta_deg": deg, "primitive": prim, "need_mm": nd, "avail_mm": avail,
            "margin_mm": margin, "action": act, "side": side}


@dataclass(frozen=True)
class TurnCheck:
    idx: int
    cell: str
    delta_deg: float
    primitive: str
    feasible: bool
    reason: str
    need_mm: float
    avail_mm: float
    margin_mm: float
    action: str

    # v1 兼容字段
    @property
    def lateral_mm(self) -> float:
        return self.need_mm

    @property
    def clearance_mm(self) -> float:
        return self.avail_mm

    @property
    def level(self) -> str:
        return self.action

    @property
    def r_need_m(self) -> float:
        return R_MIN_MM / 1000.0


def check_route(cells, blocked=(), solid=None) -> list[TurnCheck]:
    bl = set(blocked)
    out: list[TurnCheck] = []
    cs = list(cells)
    for i in range(1, len(cs) - 1):
        a, b, c2 = _xy(cs[i - 1]), _xy(cs[i]), _xy(cs[i + 1])
        v_in = (b[0] - a[0], b[1] - a[1])
        v_out = (c2[0] - b[0], c2[1] - b[1])
        if v_in == (0, 0) or v_out == (0, 0):
            continue
        sp = turn_space(cs[i], v_in, v_out, bl, solid)
        if sp["primitive"] == PRIM_STRAIGHT:
            continue
        feasible = sp["action"] in (ACT_OK, ACT_TIGHT)
        reason = (f"{sp['side']}｜need {sp['need_mm']:.0f} > avail {sp['avail_mm']:.0f}"
                  f"（{sp['action']}）" if not feasible else
                  f"{sp['side']}｜need {sp['need_mm']:.0f} ≤ avail {sp['avail_mm']:.0f}"
                  f"（{sp['action']}）")
        out.append(TurnCheck(i, cs[i], sp["delta_deg"], sp["primitive"], feasible,
                             reason, sp["need_mm"], sp["avail_mm"],
                             sp["margin_mm"], sp["action"]))
    return out


def summarize(checks) -> dict:
    chk = list(checks)
    by: dict[str, int] = {}
    act: dict[str, int] = {ACT_OK: 0, ACT_TIGHT: 0, ACT_PIVOT: 0, ACT_FORBID: 0}
    for c in chk:
        by[c.primitive] = by.get(c.primitive, 0) + 1
        act[c.action] = act.get(c.action, 0) + 1
    return {
        "turns": len(chk), "by_primitive": by, "by_action": act,
        "infeasible": sum(1 for c in chk if not c.feasible),
        "forbid135": by.get(PRIM_FORBID, 0),
        "pivot_needed": act.get(ACT_PIVOT, 0) + act.get(ACT_FORBID, 0),
        "max_turn_deg": max((c.delta_deg for c in chk), default=0.0),
        "min_margin_mm": min((c.margin_mm for c in chk), default=float("nan")),
        "hardware": {"delta_max_deg": DELTA_MAX_DEG, "r_min_mm": R_MIN_MM,
                     "need_45_mm": round(need_mm(45), 1),
                     "need_90_mm": round(need_mm(90), 1),
                     "need_135_mm": round(need_mm(135), 1),
                     "pivot_free_dia_mm": round(PIVOT_FREE_DIA_MM, 1)},
    }


def verify(cells, blocked=(), solid=None) -> dict:
    chk = check_route(cells, blocked, solid)
    s = summarize(chk)
    bad = [c for c in cells if c in set(blocked)]
    return {"ok": s["forbid135"] == 0 and s["pivot_needed"] == 0 and not bad,
            **s, "blocked_cells": bad,
            "violations": [f"{c.idx}:{c.cell}:{c.reason}" for c in chk if not c.feasible]}


def report_lines(checks, limit: int = 12) -> list[str]:
    """可执行报告：每个转角一行（含动作建议）。"""
    out = []
    for c in sorted(checks, key=lambda x: x.margin_mm)[:limit]:
        tip = {"ok": "弧线可过", "tight": "减速贴行",
               "pivot": "弧线不可行→多点掉头/改规划",
               "forbid": "135°+→必须多点掉头"}[c.action]
        out.append(f"#{c.idx:>2} {c.cell:<4} Δθ={c.delta_deg:>5.1f}° {c.primitive:<8}"
                   f" need={c.need_mm:>4.0f} avail={c.avail_mm:>4.0f} "
                   f"margin={c.margin_mm:>+6.0f}mm  [{c.action}] {tip}")
    return out
