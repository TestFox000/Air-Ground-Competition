"""field_matrix.MatrixFusion 滑窗时序融合测试（13 类协议）。

融合器的可靠性承诺（也是这里锁的语义）：
    - 滑窗收敛：窗口内出现 ≥confirm 帧才上矩阵（单帧误检进不来、偶发漏检不闪断）；
    - 过期淘汰：移动类（靶/车）超过 window 帧未复现即清除，静态类（雷）永久保留；
    - 优先级裁决：一格至多一个目标，票数高者优先，平票障碍优先（安全）。

13 类协议：雷 = 非 K 列 PAPER（落格判语义，fixture 里 C3/D4/E6 都是非 K
列，故 PAPER 即雷）；矩阵层 K 列 PAPER 渲染为箭头 A（见 test_paper_symbol）。
"""

import pytest

from ag_integrate.field_matrix import MatrixFusion, build_grid
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType

_LETTERS = "ABCDEFGHIJKLMN"


def _xy(cell: str) -> tuple:
    """格号 -> 格心世界坐标（与 field_frame.world_to_cell 的吸附映射一致）。"""
    col = _LETTERS.index(cell[0])
    row = int(cell[1:]) - 1
    return 0.065 + col * 0.35, 0.295 + (10 - row) * 0.35


def _obj(kind: ObjectType, cell: str) -> IntelObject:
    x, y = _xy(cell)
    return IntelObject(kind, x, y, 0.35, 0.35)


def _frame(ts: int, *objs: IntelObject) -> IntelFrame:
    return IntelFrame(ts, tuple(objs))


def _cells(spots) -> set:
    return {s.cell for s in spots}


def test_bad_window_confirm_rejected():
    with pytest.raises(ValueError):
        MatrixFusion(window=2, confirm=3)
    with pytest.raises(ValueError):
        MatrixFusion(window=3, confirm=0)


def test_confirm_after_enough_votes():
    """滑窗收敛：第 1 帧是"嫌疑人"，第 confirm 帧才上矩阵。"""
    fusion = MatrixFusion(window=5, confirm=2)
    spots = fusion.add_frame(_frame(1, _obj(ObjectType.PAPER, "C3")))
    assert _cells(spots) == set()                  # 单帧误检不进门
    spots = fusion.add_frame(_frame(2, _obj(ObjectType.PAPER, "C3")))
    assert _cells(spots) == {"C3"}                 # 第 2 票确认

    # 偶发漏检不闪断：第 3 帧没拍到，确认目标仍在
    spots = fusion.add_frame(_frame(3))
    assert _cells(spots) == {"C3"}


def test_median_coordinates_suppress_jitter():
    """坐标抖动取窗口中值：输出比单帧稳。"""
    fusion = MatrixFusion(window=5, confirm=2)
    x0, y0 = _xy("D4")
    jit = [0.0, 0.10, -0.10]     # 三次观测的 x 偏移（仍在格心残差容忍内）
    for i, dx in enumerate(jit):
        fusion.add_frame(_frame(i, IntelObject(ObjectType.PAPER, x0 + dx, y0, 0.35, 0.35)))
    (spot,) = fusion.fused_spots()
    assert spot.cell == "D4"
    assert spot.x == pytest.approx(x0)             # 中值是 0 偏移那一帧
    assert spot.y == pytest.approx(y0)


def test_moving_target_ages_out_but_mine_stays():
    """过期淘汰：移动靶超过 window 帧未复现即清除（防拖影）；静态雷永久保留。"""
    fusion = MatrixFusion(window=3, confirm=2)
    fusion.add_frame(_frame(1, _obj(ObjectType.TARGET_BLUE, "F5"),
                            _obj(ObjectType.PAPER, "C3")))
    spots = fusion.add_frame(_frame(2, _obj(ObjectType.TARGET_BLUE, "F5"),
                                    _obj(ObjectType.PAPER, "C3")))
    assert _cells(spots) == {"F5", "C3"}

    # 之后目标消失：空帧喂到超过 window
    for ts in range(3, 6):
        spots = fusion.add_frame(_frame(ts))
    assert "F5" not in _cells(spots)               # 移动类被清除
    assert "C3" in _cells(spots)                   # 静态类仍在


def test_moving_target_relocates_after_reconfirm():
    """移动靶换格：新格重新计数确认，旧格超过 window 帧未复现被老化清除。"""
    fusion = MatrixFusion(window=3, confirm=2)
    fusion.add_frame(_frame(1, _obj(ObjectType.TARGET_BLUE, "F5")))
    fusion.add_frame(_frame(2, _obj(ObjectType.TARGET_BLUE, "F5")))
    assert _cells(fusion.fused_spots()) == {"F5"}

    # 靶移动到 F6：第 2 票时新格确认（旧格票数还在，两格短暂共存属预期）
    fusion.add_frame(_frame(3, _obj(ObjectType.TARGET_BLUE, "F6")))
    spots = fusion.add_frame(_frame(4, _obj(ObjectType.TARGET_BLUE, "F6")))
    assert "F6" in _cells(spots)
    # 再一拍：旧格 F5 连续 window 帧未复现，老化清除
    spots = fusion.add_frame(_frame(5, _obj(ObjectType.TARGET_BLUE, "F6")))
    assert _cells(spots) == {"F6"}


def test_same_cell_priority_arbitration():
    """优先级裁决：一格至多一个目标；票数高者优先，平票障碍优先。"""
    fusion = MatrixFusion(window=5, confirm=2)
    # 同格 D4：雷 3 票 vs 障碍 2 票 -> 票数高的雷赢
    for ts in range(3):
        objs = [_obj(ObjectType.PAPER, "D4")]
        if ts < 2:
            objs.append(_obj(ObjectType.OBSTACLE, "D4"))
        fusion.add_frame(_frame(ts, *objs))
    (spot,) = [s for s in fusion.fused_spots() if s.cell == "D4"]
    assert spot.kind is ObjectType.PAPER

    # 同格 E6：雷与障碍同票 -> 平票障碍优先（规划宁可绕开，别撞桶）
    fusion2 = MatrixFusion(window=5, confirm=2)
    for ts in range(2):
        fusion2.add_frame(_frame(ts, _obj(ObjectType.PAPER, "E6"),
                                 _obj(ObjectType.OBSTACLE, "E6")))
    (spot,) = [s for s in fusion2.fused_spots() if s.cell == "E6"]
    assert spot.kind is ObjectType.OBSTACLE


def test_paper_symbol_split_by_k_column():
    """13 类落格判语义在矩阵层的分流：K 列 PAPER=箭头(A)，非 K 列=雷(M)。"""
    from ag_integrate.field_matrix import spots_from_frame
    frame = _frame(
        1,
        _obj(ObjectType.PAPER, "K9"),   # K 列桥洞
        _obj(ObjectType.PAPER, "D4"),   # 非 K 列雷
        _obj(ObjectType.OBSTACLE, "F6"),
    )
    grid = build_grid(spots_from_frame(frame))
    # 行/列 0 基：K9 -> grid[8][10]，D4 -> grid[3][3]，F6 -> grid[5][5]
    assert grid[8][10] == "A"
    assert grid[3][3] == "M"
    assert grid[5][5] == "O"


def test_build_grid_obstacle_beats_paper_same_cell():
    """单帧同格冲突裁决与融合路径统一：障碍优先（安全方向）。

    2026-10-18 前单帧 build_grid 按 _PRIORITY 是 PAPER(0) 压 OBSTACLE(1)，
    与融合路径的平票障碍优先相反——同格既有雷又有障碍时会把障碍盖成雷，
    规划可能直接压过去。两种到达顺序都必须留 O。
    """
    from ag_integrate.field_matrix import spots_from_frame
    for first, second in ((ObjectType.PAPER, ObjectType.OBSTACLE),
                          (ObjectType.OBSTACLE, ObjectType.PAPER)):
        frame = _frame(1, _obj(first, "D4"), _obj(second, "D4"))
        grid = build_grid(spots_from_frame(frame))
        assert grid[3][3] == "O"


def test_build_grid_obstacle_beats_k_column_arrow():
    """K 列桥洞格：箭头(A)同样不许覆盖障碍（A 是可通行语义，覆盖更危险）。"""
    from ag_integrate.field_matrix import spots_from_frame
    frame = _frame(1, _obj(ObjectType.PAPER, "K5"),
                   _obj(ObjectType.OBSTACLE, "K5"))
    grid = build_grid(spots_from_frame(frame))
    assert grid[4][10] == "O"


def test_out_of_bounds_spot_not_fused_but_reported():
    """出界目标不落格、不参与融合，但保留最近坐标供明细展示。"""
    fusion = MatrixFusion(window=5, confirm=2)
    fusion.add_frame(_frame(1, IntelObject(ObjectType.CAR, 99.0, 99.0, 0.5, 0.5)))
    fusion.add_frame(_frame(2, IntelObject(ObjectType.CAR, 99.0, 99.0, 0.5, 0.5)))
    assert fusion.fused_spots() == []
    outside = fusion.outside_spots()
    assert len(outside) == 1 and outside[0].kind is ObjectType.CAR
    assert outside[0].cell is None
