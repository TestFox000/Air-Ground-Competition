"""ag_integrate field_frame / world_model 测试（13 类协议）。

13 类协议：雷 = 非 K 列 PAPER，桥洞 = K 列 PAPER（落格判语义）；
CAR / CELL_LABEL / BUCKET_* / 起止点类不进态势。红靶粘性否决的 TTL
时基是**本地 monotonic 收包时刻**（update 的 recv_mono，2026-10-17
修订：机载 system_clock 无 NTP 会回跳），故粘性测试显式传 recv_mono。
"""

import pytest

from ag_integrate.field_frame import CellRef, cell_to_world, world_to_cell
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType, build, parse
from ag_integrate.world_model import WorldModel


@pytest.mark.parametrize(
    "cell",
    [CellRef(0, 0), CellRef(13, 10), CellRef(10, 5), CellRef(3, 0)],
)
def test_world_cell_roundtrip(cell):
    x, y = cell_to_world(cell)
    assert world_to_cell(x, y) == cell


def test_world_to_cell_rejects_outliers():
    # 收紧吸附容差：偏离格心 0.4 个格距应被拒
    x = 0.065 + 0.35 * 2.4
    with pytest.raises(ValueError):
        world_to_cell(x, 0.295, max_residual=0.3)
    with pytest.raises(ValueError):
        world_to_cell(10.0, 10.0)


def obj_at(cell: CellRef, obj_type: ObjectType) -> IntelObject:
    x, y = cell_to_world(cell)
    return IntelObject(obj_type, x, y, 0.35, 0.35)


def test_world_model_classifies_and_dedups():
    m1, m2 = CellRef(3, 10), CellRef(1, 6)     # D11, B7
    a1 = CellRef(10, 8)                         # K9
    o1 = CellRef(5, 5)                          # F6
    blue, red = CellRef(3, 1), CellRef(4, 1)    # D2, E2

    frame = IntelFrame(
        1234,
        (
            obj_at(m1, ObjectType.PAPER),   # 非 K 列 -> 雷
            obj_at(m1, ObjectType.PAPER),   # 重复，去重
            obj_at(m2, ObjectType.PAPER),
            obj_at(a1, ObjectType.PAPER),   # K 列 -> 桥洞通行
            obj_at(o1, ObjectType.OBSTACLE),
            obj_at(blue, ObjectType.TARGET_BLUE),
            obj_at(red, ObjectType.TARGET_RED),
            # 13 类新增的噪声类：定位/展示/起止点，一律不进态势
            obj_at(CellRef(7, 7), ObjectType.CAR),
            obj_at(CellRef(7, 8), ObjectType.CELL_LABEL),
            obj_at(CellRef(6, 3), ObjectType.BUCKET_GREEN),
            obj_at(CellRef(2, 9), ObjectType.UAV_START),
        ),
    )

    # 单帧直出（N=1）：分类/去重语义本测试焦点，默认 N=2 的多帧确认
    # 行为由 test_world_model_confirm.py 锁定。
    model = WorldModel(obstacle_confirm_frames=1)
    assert not model.ready
    snap = model.update(frame)
    assert model.ready
    assert snap.timestamp_ms == 1234
    assert {c.name for c in snap.mines} == {"D11", "B7"}
    assert {c.name for c in snap.passable} == {"K9"}
    assert {c.name for c in snap.obstacles} == {"F6"}
    assert {t.cell.name for t in snap.targets} == {"D2", "E2"}
    assert snap.has_enemy
    assert {(t.cell.name, t.is_blue) for t in snap.targets} == {("D2", True), ("E2", False)}


def test_pipeline_from_bytes():
    """协议字节 -> 解析 -> 世界模型 端到端。"""
    cell = CellRef(0, 0)
    raw = build(IntelFrame(99, (obj_at(cell, ObjectType.PAPER),)))
    model = WorldModel(obstacle_confirm_frames=1)
    snap = model.update(parse(raw))
    assert snap.mines[0].name == "A1"


def test_red_target_sticky_veto_ttl():
    """红靶粘性否决（禁火铁律）：红靶出现后消失，TTL 内仍留在态势参与
    否决（扛单帧漏检/遮挡/落格残差丢弃），TTL 后恢复。"""
    model = WorldModel(red_veto_ttl_s=5.0)
    red = obj_at(CellRef(4, 1), ObjectType.TARGET_RED)   # E2
    s1 = model.update(IntelFrame(1000, (red,)), recv_mono=100.0)
    assert {(t.cell.name, t.is_blue) for t in s1.targets} == {("E2", False)}

    # 2s 时红靶消失：距最后见到 2s ≤ TTL，粘性补回，仍否决
    s2 = model.update(IntelFrame(3000, ()), recv_mono=102.0)
    assert {(t.cell.name, t.is_blue) for t in s2.targets} == {("E2", False)}

    # 恰好 TTL 边界（距 5s）：仍否决
    s3 = model.update(IntelFrame(6000, ()), recv_mono=105.0)
    assert {(t.cell.name, t.is_blue) for t in s3.targets} == {("E2", False)}

    # 6s（距 5s 的边界后 1s > TTL 5s）：恢复，不再否决
    s4 = model.update(IntelFrame(9500, ()), recv_mono=106.0)
    assert s4.targets == ()


def test_sticky_red_fail_safe_same_cell_as_blue():
    """fail-safe 方向只增不减：粘性红与本帧蓝同格时，红靶保留否决，
    不因本帧报蓝而删除。"""
    model = WorldModel(red_veto_ttl_s=5.0)
    model.update(IntelFrame(1000, (obj_at(CellRef(3, 1), ObjectType.TARGET_RED),)),
                 recv_mono=100.0)
    snap = model.update(
        IntelFrame(2000, (obj_at(CellRef(3, 1), ObjectType.TARGET_BLUE),)),
        recv_mono=101.0,
    )
    assert {(t.cell.name, t.is_blue) for t in snap.targets} == {
        ("D2", True), ("D2", False)
    }


def test_sticky_red_tracks_latest_sighting():
    """红靶持续可见时计时不断刷新，消失后从最后一次见到起算 TTL。"""
    model = WorldModel(red_veto_ttl_s=5.0)
    red = obj_at(CellRef(4, 1), ObjectType.TARGET_RED)   # E2
    model.update(IntelFrame(1000, (red,)), recv_mono=100.0)
    model.update(IntelFrame(5000, (red,)), recv_mono=104.0)   # 仍可见，刷新计时
    # 距首次 8s 但距末次 4s：仍否决
    assert any(not t.is_blue
               for t in model.update(IntelFrame(9000, ()), recv_mono=108.0).targets)
    # 距末次 6s > TTL：恢复
    assert model.update(IntelFrame(11000, ()), recv_mono=110.0).targets == ()
