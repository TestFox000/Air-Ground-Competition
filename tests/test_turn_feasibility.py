# -*- coding: utf-8 -*-
"""转角可行性判据 + 转角分级代价的回归测试（V1019 移植，2026-10-20 入仓）。

覆盖三件事：
    1) 物理基线（need_mm：45°≈146 / 90°≈330 / 135°≈514）
    2) turn_feasibility_v2 的判据：45° 永远 ok；90° 内侧紧邻桶 → pivot；135° → forbid
    3) mission_router._TURN_GRADE 默认值（放宽版）与其对 135° 的抑制效果
"""
from __future__ import annotations

from ag_integrate import mission_router as MR
from ag_integrate import turn_feasibility_v2 as TF
from ag_integrate import world_model as WM
from ag_integrate.field_frame import CellRef


def _c(name: str) -> CellRef:
    return CellRef(ord(name[0]) - ord("A"), int(name[1:]) - 1)


def _make(cls, **kw):
    import dataclasses
    names = {f.name for f in dataclasses.fields(cls)}
    return cls(**{k: v for k, v in kw.items() if k in names})


def _plan(mines, obstacles, grade):
    """跑一次分区规划，返回 (路线格数, 135° 个数)。"""
    snap = _make(WM.WorldSnapshot, timestamp_ms=0,
                 mines=tuple(_c(m) for m in mines),
                 passable=(), obstacles=tuple(_c(o) for o in obstacles), targets=())
    old = MR._TURN_GRADE
    MR._TURN_GRADE = grade
    try:
        r = MR.plan_route_zoned(snap, _c("N10"), _c("N5"), None, mid_boundary=6,
                                start_heading="y", final_heading=None)
        cells = list(getattr(r, "cells", r))
    finally:
        MR._TURN_GRADE = old
    s = TF.summarize(TF.check_route(cells, obstacles))
    return len(cells), s["forbid135"]


# ---------- 1) 物理基线 ----------
def test_need_mm_physics_baseline():
    assert 140.0 < TF.need_mm(45) < 155.0, "45° 需求应 ≈146mm"
    assert 320.0 < TF.need_mm(90) < 340.0, "90° 需求应 ≈330mm"
    assert TF.need_mm(135) > 500.0, "135° 需求应 >500mm"
    assert TF.PIVOT_FREE_DIA_MM > 350.0, "原地转/掉头所需自由径应大于一格"


# ---------- 2) 判据 ----------
def test_turn_45_ok_even_with_adjacent_obstacle():
    """45° 弧线在格内可完成 —— 即使相邻格有桶也应可过。"""
    sp = TF.turn_space("C5", (1, 0), (1, 1), {"D5", "D6"})
    assert sp["action"] in ("ok", "tight"), sp
    assert sp["need_mm"] < sp["avail_mm"]


def test_turn_90_with_blocked_inside_needs_reverse():
    """90° 弯且**转弯内侧**紧邻桶（B11 那类）→ 必须倒车/多点掉头。"""
    # (1,0) 东 → (0,1) 北：左转，内侧是北侧（C6）
    sp = TF.turn_space("C5", (1, 0), (0, 1), {"C6"})
    assert sp["action"] in ("pivot", "forbid"), sp
    assert sp["margin_mm"] < 0, "内侧被桶占用时应出现负余量"


def test_turn_90_with_open_inside_is_ok():
    """同样的 90°，内侧开阔 → 可过（证明判据区分内侧/外侧）。"""
    sp = TF.turn_space("C5", (1, 0), (0, 1), set())
    assert sp["action"] == "ok", sp
    assert sp["margin_mm"] > 0


def test_turn_135_is_forbidden():
    """135° 单段绝无可能（需多点掉头）。"""
    sp = TF.turn_space("C5", (1, 0), (-1, 1), set())
    assert sp["primitive"] == "forbid135", sp
    assert sp["action"] == "forbid"


# ---------- 3) 分级代价 ----------
def test_default_turn_grade_is_relaxed():
    """默认应为 (1,1,2,10,1)：135° 重罚、**180° 按倒车计与 45° 同价**（2026-10-21）。

    180° 从 10.0 改 1.0 的依据：阿克曼车可倒车，回头不必原地掉头（用户确认 +
    同态势实测：41 格/28 转向单元 → 33 格/22 单元）。若车端实测"画圈而非倒车"，
    把末位改回 10.0，本断言同步改回。
    """
    assert MR._TURN_GRADE == (1.0, 1.0, 2.0, 10.0, 1.0)


def test_relaxed_grade_does_not_increase_135_turns():
    """放宽（2,10,10）相对均匀（1,1,1,1,1）不应增加 135° 转角个数。"""
    mines = ["E3", "I6", "A3"]
    obst = ["A1", "D8", "E2", "H4", "I8", "J9"]
    _, n_flat = _plan(mines, obst, (1.0, 1.0, 1.0, 1.0, 1.0))
    _, n_relax = _plan(mines, obst, (1.0, 1.0, 2.0, 10.0, 10.0))
    assert n_relax <= n_flat, f"放宽后 135° 数应不增（flat={n_flat}, relax={n_relax}）"
