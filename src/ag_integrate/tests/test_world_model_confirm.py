# -*- coding: utf-8 -*-
"""WorldModel 障碍多帧确认（N=2 默认，2026-10-17）行为锁定。

动机：实拍复盘确认单帧幻影障碍触发无效重规划的代价远大于 0.2s 确认
延迟——运行时默认 N=2（车载版三帧共识），单帧幽灵不进态势。
"""

from __future__ import annotations

from ag_integrate.field_frame import CellRef, cell_to_world
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType
from ag_integrate.world_model import WorldModel


def _frame(ts, cells, t=ObjectType.OBSTACLE):
    objs = []
    for c in cells:
        x, y = cell_to_world(c)
        objs.append(IntelObject(t, x, y, 0.35, 0.35))
    return IntelFrame(ts, tuple(objs))


def test_default_single_frame_obstacle_not_confirmed():
    """默认 N=2：单帧障碍不进态势（防幻影）。"""
    wm = WorldModel()
    snap = wm.update(_frame(1, [CellRef(3, 3)]))          # D4 单帧
    assert all(c.name != "D4" for c in snap.obstacles)


def test_default_two_frames_confirm():
    wm = WorldModel()
    wm.update(_frame(1, [CellRef(3, 3)]))
    snap = wm.update(_frame(2, [CellRef(3, 3)]))          # 连续第二帧
    assert any(c.name == "D4" for c in snap.obstacles)


def test_default_hysteresis_exit():
    """进入需 2 帧，移出同样需连续 2 帧消失（对称滞回）。"""
    wm = WorldModel()
    wm.update(_frame(1, [CellRef(3, 3)]))
    wm.update(_frame(2, [CellRef(3, 3)]))
    snap1 = wm.update(_frame(3, []))                       # 漏检 1 帧：保持
    assert any(c.name == "D4" for c in snap1.obstacles)
    snap2 = wm.update(_frame(4, []))                       # 连续 2 帧：移出
    assert all(c.name != "D4" for c in snap2.obstacles)


def test_n1_opt_out_preserves_old_behavior():
    """N=1 显式退化为旧行为（路由器测试依赖此语义）。"""
    wm = WorldModel(obstacle_confirm_frames=1)
    snap = wm.update(_frame(1, [CellRef(3, 3)]))
    assert any(c.name == "D4" for c in snap.obstacles)


# ---- 红靶否决多帧确认（N=2 默认，2026-10-25） ----

def test_single_frame_red_not_vetoed():
    """单帧幻影红靶不进态势、不武装否决（hbb_d13 幻影红检实测动机）。"""
    wm = WorldModel()
    wm.update(_frame(1, [CellRef(5, 6)], ObjectType.TARGET_RED))
    snap = wm.update(_frame(2, []))                        # 幻影消失
    assert not any(t.is_blue is False for t in snap.targets)
    assert not wm.veto_active(CellRef(5, 6))


def test_two_frames_red_confirmed_and_sticky():
    """连续 2 帧确认进入态势；漏检 1 帧靠 TTL 粘性保持（只增不减）。"""
    wm = WorldModel()
    wm.update(_frame(1, [CellRef(5, 6)], ObjectType.TARGET_RED))
    snap = wm.update(_frame(2, [CellRef(5, 6)], ObjectType.TARGET_RED))
    assert any(t.is_blue is False for t in snap.targets)
    snap2 = wm.update(_frame(3, []))
    assert any(t.is_blue is False for t in snap2.targets)


def test_red_n1_opt_out_preserves_old_behavior():
    """N=1 显式退化为旧行为（单帧红检立即武装）。"""
    wm = WorldModel(red_confirm_frames=1)
    snap = wm.update(_frame(1, [CellRef(5, 6)], ObjectType.TARGET_RED))
    assert any(t.is_blue is False for t in snap.targets)
