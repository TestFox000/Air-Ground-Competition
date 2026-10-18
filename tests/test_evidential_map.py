"""evidential_map 证据格地图测试。

锁定的可靠性承诺：
    - 两次高置信检测确认，单帧不进门（与 field_matrix 同门槛语义）；
    - 负证据清除低置信 FP；但雷语义减半权重 → 已确认的雷不被轻易清除；
    - 粘性类（红靶/红桶）确认后，负证据与时间衰减永不击穿（禁火铁律）；
    - 画质不足的覆盖不产生负证据，只计覆盖率（负证据合法性来源）；
    - 靶类走轨道级 TrackSpot，不落死格；快照带滞后无闪断。
"""

import math

import pytest

from ag_integrate.evidential_map import EvidentialMap, EvidenceParams
from ag_integrate.field_frame import CellRef, LETTERS


def _cell(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


def test_two_hits_confirm_single_does_not():
    m = EvidentialMap()
    m.add_positive(_cell("F5"), 0, 0.9, t=1.0)
    assert not any(s.confirmed for s in m.snapshot(t=1.0))
    m.add_positive(_cell("F5"), 0, 0.9, t=1.1)
    spot = [s for s in m.snapshot(t=1.1) if s.confirmed]
    assert len(spot) == 1 and spot[0].kind == 0 and spot[0].cell == _cell("F5")


def test_negative_evidence_clears_low_confidence_fp():
    """单帧低置信 FP（conf 0.6）被 3 次清晰负证据清除。"""
    m = EvidentialMap()
    m.add_positive(_cell("D8"), 0, 0.6, t=1.0)       # logit ≈ 0.405
    assert len(m.snapshot(t=1.0)) == 1               # 未确认但在快照内
    for k in range(3):
        m.observe_cell(_cell("D8"), t=2.0 + k, quality=0.8)
    # PAPER 负证据减半：0.405 - 3×0.35 = -0.645 < drop(-0.5)
    assert m.snapshot(t=5.0) == []
    assert m.belief(_cell("D8"), 0, t=5.0) < 0.4


def test_confirmed_mine_survives_moderate_negatives():
    """已确认的雷（2×0.9）在 5 次负证据后仍确认（负证据不能轻易清真雷）。"""
    m = EvidentialMap()
    m.add_positive(_cell("K4"), 0, 0.9, t=1.0)
    m.add_positive(_cell("K4"), 0, 0.9, t=1.1)       # logit ≈ 4.39
    for k in range(5):
        m.observe_cell(_cell("K4"), t=2.0 + k, quality=0.9)
    spots = [s for s in m.snapshot(t=7.0) if s.confirmed]
    assert len(spots) == 1 and spots[0].kind == 0


def test_obstacle_negatives_full_weight():
    """障碍负证据全权重：6 次即可压下确认（与雷语义形成对比）。"""
    m = EvidentialMap()
    from ag_integrate.intel_protocol import ObjectType
    m.add_positive(_cell("B2"), ObjectType.OBSTACLE, 0.9, t=1.0)
    m.add_positive(_cell("B2"), ObjectType.OBSTACLE, 0.9, t=1.1)
    for k in range(6):
        m.observe_cell(_cell("B2"), t=2.0 + k, quality=0.9)
    assert not any(s.confirmed for s in m.snapshot(t=8.0))


def test_sticky_red_target_never_cleared():
    """红靶一次确认后：无限负证据 + 长期衰减都保持确认（禁火铁律）。"""
    from ag_integrate.intel_protocol import ObjectType
    m = EvidentialMap()
    m.add_positive(_cell("C7"), ObjectType.TARGET_RED, 0.9, t=1.0)
    for k in range(20):
        m.observe_cell(_cell("C7"), t=2.0 + k, quality=1.0)
    t_far = 1.0 + 5 * m.p.half_life_s                # 衰减 5 个半衰期
    spots = [s for s in m.snapshot(t=t_far)
             if s.kind == ObjectType.TARGET_RED and s.confirmed]
    assert len(spots) == 1


def test_quality_gate_blocks_negative_evidence():
    """画质 0.3 的覆盖：不产生负证据（信念不变），覆盖率不计清。"""
    m = EvidentialMap()
    m.add_positive(_cell("F5"), 0, 0.9, t=1.0)
    before = m.belief(_cell("F5"), 0, t=1.0)
    m.observe_cell(_cell("F5"), t=2.0, quality=0.3)
    assert m.belief(_cell("F5"), 0, t=2.0) == pytest.approx(before)
    assert not m.cell_is_clear(_cell("F5"))
    assert m.coverage_frac() == 0.0


def test_coverage_fraction():
    m = EvidentialMap()
    cells = [CellRef(c, r) for c in range(7) for r in range(11)]   # 77 格
    m.observe_cells(cells, t=1.0, quality=0.8)
    assert m.coverage_frac() == pytest.approx(77 / 154)


def test_decay_half_life():
    m = EvidentialMap()
    m.add_positive(_cell("F5"), 0, 0.9, t=10.0)
    m.add_positive(_cell("F5"), 0, 0.9, t=10.1)     # logit = 2·ln9
    l0 = math.log(m.belief(_cell("F5"), 0, t=10.1) /
                  (1 - m.belief(_cell("F5"), 0, t=10.1)))
    l1 = math.log(m.belief(_cell("F5"), 0, t=10.1 + m.p.half_life_s) /
                  (1 - m.belief(_cell("F5"), 0, t=10.1 + m.p.half_life_s)))
    assert l1 == pytest.approx(l0 / 2, rel=1e-6)


def test_snapshot_no_flap_on_single_negative():
    """已确认目标夹一次负证据不闪断（confirm/drop 双门限滞后）。"""
    m = EvidentialMap()
    m.add_positive(_cell("G6"), 0, 0.9, t=1.0)
    m.add_positive(_cell("G6"), 0, 0.9, t=1.1)
    m.observe_cell(_cell("G6"), t=2.0, quality=0.9)
    spots = [s for s in m.snapshot(t=2.0) if s.confirmed]
    assert len(spots) == 1


def test_target_uses_track_semantics():
    """set_track 后靶证据写轨道级：快照出 TrackSpot（双格），无单格 MapSpot。"""
    from ag_integrate.intel_protocol import ObjectType
    m = EvidentialMap()
    m.set_track([_cell("D8"), _cell("E8")])
    m.add_positive(_cell("D8"), ObjectType.TARGET_BLUE, 0.9, t=1.0)
    m.add_positive(_cell("E8"), ObjectType.TARGET_BLUE, 0.9, t=3.0)  # 相位变化无妨
    snap = m.snapshot(t=3.0)
    tracks = [s for s in snap if s.__class__.__name__ == "TrackSpot"]
    assert len(tracks) == 1 and tracks[0].confirmed
    assert set(tracks[0].cells) == {_cell("D8"), _cell("E8")}
    assert not [s for s in snap
                if s.__class__.__name__ == "MapSpot"
                and s.kind == ObjectType.TARGET_BLUE]


def test_untracked_kind_rejected():
    from ag_integrate.intel_protocol import ObjectType
    m = EvidentialMap()
    with pytest.raises(ValueError, match="not tracked"):
        m.add_positive(_cell("F5"), ObjectType.CAR, 0.9, t=1.0)


def test_track_sticky_red_target():
    """轨道级红靶同样受粘性保护（轨道上确认一次，永久确认）。"""
    from ag_integrate.intel_protocol import ObjectType
    m = EvidentialMap()
    m.set_track([_cell("D8"), _cell("E8")])
    m.add_positive(_cell("D8"), ObjectType.TARGET_RED, 0.9, t=1.0)
    for k in range(20):
        m.observe_cell(_cell("E8"), t=2.0 + k, quality=1.0)
    t_far = 1.0 + 3 * m.p.half_life_s
    tracks = [s for s in m.snapshot(t=t_far)
              if s.__class__.__name__ == "TrackSpot" and s.confirmed]
    assert len(tracks) == 1 and tracks[0].kind == ObjectType.TARGET_RED
