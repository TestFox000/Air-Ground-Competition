# -*- coding: utf-8 -*-
"""靶标身份多帧确认行为锁定（2026-10-25，flight_20261025 诊断③）。

蓝面判定一直靠单帧类别（d13 类别横跳 + 红蓝面随视角翻转），两层修复：
    1) WorldModel 蓝靶多帧确认（blue_confirm_frames）：单帧幽灵蓝不进
       态势（不骗规划安排停靠射击位）；红帧证据清零蓝候选（横跳格
       不放行开火位，fail-safe）；N=1 退化为旧行为。
    2) EvidentialFusion 同报格红蓝仲裁：红蓝 logit 各自独立过确认门时
       只上报证据更强一侧（prob=有效 logit 的单调 sigmoid），平手判红。
"""

from __future__ import annotations

from ag_integrate.evidential_fusion import EvidentialFusion
from ag_integrate.field_frame import CellRef, cell_to_world
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType
from ag_integrate.world_model import WorldModel


def _frame(ts: int, cells, t: ObjectType, recv_mono: float | None = None):
    objs = []
    for c in cells:
        x, y = cell_to_world(c)
        objs.append(IntelObject(t, x, y, 0.35, 0.35))
    return IntelFrame(ts, tuple(objs)), recv_mono


def _fusion_frame(ts: int, cell: CellRef, t: ObjectType,
                  conf: float, recv_mono: float) -> IntelFrame:
    x, y = cell_to_world(cell)
    return IntelFrame(ts, (IntelObject(t, x, y, 0.12, 0.12, conf),))


# ---- 1) WorldModel 蓝靶多帧确认 ----

def test_blue_single_frame_not_confirmed():
    """单帧幽灵蓝检不进态势（默认 N=2，与红靶对称）。"""
    wm = WorldModel()
    snap = wm.update(_frame(1, [CellRef(5, 6)], ObjectType.TARGET_BLUE)[0])
    assert not any(t.is_blue for t in snap.targets)


def test_blue_two_frames_confirm():
    """连续 2 帧确认进入态势（5fps 下只晚 0.2s）。"""
    wm = WorldModel()
    wm.update(*_frame(1, [CellRef(5, 6)], ObjectType.TARGET_BLUE, recv_mono=1.0))
    snap = wm.update(*_frame(2, [CellRef(5, 6)], ObjectType.TARGET_BLUE, recv_mono=1.2))
    blues = [t for t in snap.targets if t.is_blue]
    assert len(blues) == 1 and blues[0].cell.name == "F7"


def test_blue_n1_opt_out_preserves_old_behavior():
    """N=1 显式退化为旧行为（单帧蓝检立即进态势）。"""
    wm = WorldModel(blue_confirm_frames=1)
    snap = wm.update(_frame(1, [CellRef(5, 6)], ObjectType.TARGET_BLUE)[0])
    assert any(t.is_blue for t in snap.targets)


def test_red_frame_resets_blue_candidate():
    """红帧证据清零蓝候选：红蓝对半横跳的格不累积到确认（fail-safe）。"""
    wm = WorldModel()
    wm.update(*_frame(1, [CellRef(5, 6)], ObjectType.TARGET_BLUE, recv_mono=1.0))
    # 红帧：蓝候选被清零（红自身单帧也不确认、不进态势）
    snap_r = wm.update(*_frame(2, [CellRef(5, 6)], ObjectType.TARGET_RED, recv_mono=1.2))
    assert not snap_r.targets
    # 蓝帧重新从 1 计（不是 2）：仍不确认
    snap_b1 = wm.update(*_frame(3, [CellRef(5, 6)], ObjectType.TARGET_BLUE, recv_mono=1.4))
    assert not any(t.is_blue for t in snap_b1.targets)
    # 再一帧蓝才确认
    snap_b2 = wm.update(*_frame(4, [CellRef(5, 6)], ObjectType.TARGET_BLUE, recv_mono=1.6))
    assert any(t.is_blue for t in snap_b2.targets)


def test_blue_candidate_ttl_expiry():
    """候选窗口过期（> red_veto_ttl_s 无新证据）后计数作废重来。"""
    wm = WorldModel()          # red_veto_ttl_s 默认 5s
    wm.update(*_frame(1, [CellRef(5, 6)], ObjectType.TARGET_BLUE, recv_mono=1.0))
    snap = wm.update(*_frame(2, [CellRef(5, 6)], ObjectType.TARGET_BLUE, recv_mono=9.0))
    assert not any(t.is_blue for t in snap.targets)   # 8s 间隔：计数重置为 1


# ---- 2) EvidentialFusion 同报格红蓝仲裁 ----

def test_fusion_same_cell_tie_prefers_red():
    """红蓝同格、证据等强（各一次 0.95 观测）→ 平手判红，只报一个。"""
    f = EvidentialFusion()
    f.add_frame(_fusion_frame(1, CellRef(5, 6), ObjectType.TARGET_RED, 0.95, 1.0),
                recv_mono=1.0)
    f.add_frame(_fusion_frame(2, CellRef(5, 6), ObjectType.TARGET_BLUE, 0.95, 1.1),
                recv_mono=1.1)
    snap = f.snapshot
    tgts = [t for t in snap.targets if t.cell.name == "F7"]
    assert len(tgts) == 1, "同报格必须只出一个 TargetRef（诊断③敌我同格）"
    assert tgts[0].is_blue is False, "平手判红（fail-safe 不放行开火位）"


def test_fusion_same_cell_stronger_blue_wins():
    """蓝侧证据更强（两次 0.95 vs 红一次）→ 上报蓝，可安排停靠射击。"""
    f = EvidentialFusion()
    f.add_frame(_fusion_frame(1, CellRef(5, 6), ObjectType.TARGET_RED, 0.95, 1.0),
                recv_mono=1.0)
    f.add_frame(_fusion_frame(2, CellRef(5, 6), ObjectType.TARGET_BLUE, 0.95, 1.1),
                recv_mono=1.1)
    f.add_frame(_fusion_frame(3, CellRef(5, 6), ObjectType.TARGET_BLUE, 0.95, 1.2),
                recv_mono=1.2)
    snap = f.snapshot
    tgts = [t for t in snap.targets if t.cell.name == "F7"]
    assert len(tgts) == 1 and tgts[0].is_blue is True
