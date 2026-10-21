# -*- coding: utf-8 -*-
"""video_recon 机械规则回归测试：全部用合成数据，不依赖模型与视频。

比赛时没有 agent 介入——每一条裁决规则（共识拟合/锚点门控/残差门控/
幻影消除/K列语义/射击停靠保护）必须由测试保证其机械正确性。
用例对应 2026-10-17 实拍视频取证中的真实场景：
- 共识拟合：6 标签含 1 个 v4d 误读仍可自愈（毒化帧恢复）
- 幻影 C7→I2：误读锚点把真雷拽到远格（剔除）
- 共现保真 E2/E3：同帧检出两颗雷（保留，不得误杀）
- 票期独立 F5：拍摄空窗段的真雷（保留，不得误杀）
"""

from __future__ import annotations

from collections import Counter

import numpy as np
import pytest

from ag_integrate.field_frame import CellRef
from ag_integrate.video_recon import (
    ANCHOR_GATE,
    MAX_VOTE_RESIDUAL,
    VOTE_MIN,
    anchor_within,
    circular_mode,
    confirmed_cells,
    fit_consensus_affine,
    parse_cell,
    snap_cell,
    split_mines_passable,
    eliminate_phantoms,
)


# ---------------------------------------------------------------- 共识仿射
def _synth_frame(n_labels: int = 6, misread: dict[int, tuple[int, int]] | None = None,
                 seed: int = 7):
    """合成一帧：真实仿射 grid->px，指定索引的标签被误读到错误格。"""
    rng = np.random.default_rng(seed)
    A = np.array([[42.0, -6.0, 55.0],      # px = A @ [c, r, 1]
                  [5.0, 38.0, 30.0]])
    grid = np.stack([rng.integers(3, 9, n_labels),
                     rng.integers(1, 6, n_labels)]).T.astype(float)
    px = np.stack([A @ np.stack([grid[:, 0], grid[:, 1], np.ones(n_labels)]), ], 0)[0].T
    px += rng.normal(0, 0.8, px.shape)     # 亚像素级噪声
    grid_used = grid.copy()
    for i, (bc, br) in (misread or {}).items():
        grid_used[i] = [bc, br]
    return px, grid, grid_used


def test_consensus_affine_recovers_from_one_misread():
    """6 标签 1 误读：共识投票必须剔掉坏点并恢复真实几何（毒化帧自愈）。"""
    px, grid_true, grid_used = _synth_frame(6, misread={2: (0, 9)})   # 索引2被读飞
    A, mask, mae = fit_consensus_affine(px, grid_used)
    assert A is not None, "1 个误读不应毁掉整帧拟合"
    assert mask[2] is False or mae < 0.2
    assert mae < 0.2
    # 拟合结果应接近真实格坐标（把误读格纠正回真值后残差小）
    x = np.hstack([px, np.ones((len(px), 1))])
    resid_on_true = np.linalg.norm(x @ A - grid_true, axis=1)
    assert resid_on_true[mask].max() < 0.45


def test_consensus_affine_all_good():
    px, grid, grid = _synth_frame(5, seed=11)
    A, mask, mae = fit_consensus_affine(px, grid)
    assert A is not None and mask.all() and mae < 0.2


def test_consensus_affine_majority_bad_three_point_limitation():
    """已知局限（设计取舍）：3 点无冗余，即使含误读也必然精确拟合——
    这正是实拍中 C7 幻影的成因。防线在下游幻影消除规则
    （见 test_phantom_c7_pattern_eliminated），此处固化该行为契约。"""
    px, grid, grid_used = _synth_frame(3, misread={0: (13, 10), 1: (0, 0)}, seed=3)
    A, mask, mae = fit_consensus_affine(px, grid_used)
    assert A is not None and mae < 0.05    # 3 点精确解：拟合器无法自检


def test_consensus_affine_collinear_rejected():
    """共线 3 点无唯一仿射解，必须拒绝。"""
    px = np.array([[10.0, 10.0], [30.0, 20.0], [50.0, 30.0]])
    grid = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    A, _, mae = fit_consensus_affine(px, grid)
    assert A is None


def test_consensus_affine_needs_three():
    px = np.array([[10.0, 10.0], [50.0, 12.0]])
    A, _, mae = fit_consensus_affine(px, np.array([[0.0, 0.0], [1.0, 0.0]]))
    assert A is None and mae > 5


# ---------------------------------------------------------------- 门控
def test_anchor_gate():
    anchors = [(8, 2), (7, 1)]                      # I3 / H2
    assert anchor_within(8.0, 0.5, anchors)         # 距 H2 切比雪夫 1.0 -> 通过
    assert anchor_within(8.4, 2.4, anchors)         # I3 格心 -> 通过
    assert not anchor_within(10.2, 0.1, anchors)    # 距最近锚点 2.2 -> 外推拒绝


def test_snap_cell_residual():
    fc = fr = 0.38
    assert snap_cell(4.86 + fc, 3.14 + fr, fc, fr) == (5, 3)       # F4 格心附近
    assert snap_cell(5.44 + fc, 4.42 + fr, fc, fr) is None          # 残差 0.44 -> 拒绝
    assert snap_cell(4.0 + fc, 2.0 + fr, fc, fr) == (4, 2)          # E3 格心
    assert snap_cell(13.9, 5.0, 0.0, 0.0) is None                    # 越界拒绝


def test_circular_mode_wraps():
    vals = [0.95, 0.96, 0.97, 0.02, 0.03, 0.55, 0.58]
    m = circular_mode(vals)
    assert m > 0.88 or m < 0.10            # 众数跨 0 点聚集（环形）
    assert abs(circular_mode([0.3, 0.36, 0.39, 0.41, 0.44]) - 0.38) < 0.06


# ---------------------------------------------------------------- 幻影消除
def _votes_setup(spec: dict[str, list[int]]) -> tuple[Counter, dict]:
    votes = Counter({c: len(fs) for c, fs in spec.items()})
    vframes = {c: set(fs) for c, fs in spec.items()}
    return votes, vframes


def test_phantom_c7_pattern_eliminated():
    """实拍场景复现：I2 标签被误读成 C7，真雷被拽到 C7。
    C7 两票夹在 I2 票期内且从未同帧共现 -> 幻影剔除。"""
    spec = {
        "I2": [2385, 2430, 2460, 2610, 2700, 3075, 3120, 3165, 3225, 3300],  # 10 票
        "C7": [3210, 3255],                                                  # 2 票
    }
    votes, vframes = _votes_setup(spec)
    kept, phantoms = eliminate_phantoms(list(spec), votes, vframes)
    assert phantoms == {"C7": "I2"}
    assert "C7" not in kept and "I2" in kept


def test_phantom_cooccur_keeps_real_adjacent():
    """实拍场景复现：E2 与 E3 同帧共现（一帧检出两颗雷）-> 独立物体，不得误杀。"""
    spec = {
        "E3": [100, 105, 110, 112, 115, 120, 200],   # 7 票
        "E2": [110, 112],                             # 2 票，110/112 与 E3 共现
    }
    votes, vframes = _votes_setup(spec)
    kept, phantoms = eliminate_phantoms(list(spec), votes, vframes)
    assert "E2" in kept and "E3" in kept and not phantoms


def test_phantom_temporal_gap_keeps_real():
    """实拍场景复现：F5 票期落在 E3 与 I2 拍摄段之间的空窗 -> 不构成夹芯，保留。"""
    spec = {
        "E3": [10, 20, 30, 40, 50, 60],               # 6 票强格
        "F5": [2280, 2295],                            # 空窗段 2 票
        "I2": [3000, 3100, 3200, 3300, 3400, 3500],   # 6 票强格
    }
    votes, vframes = _votes_setup(spec)
    kept, phantoms = eliminate_phantoms(list(spec), votes, vframes)
    assert "F5" in kept and not phantoms


def test_phantom_strong_cell_immune():
    """>3 票的格不参与弱格判定，永不被剔除。"""
    spec = {"E3": list(range(100, 140, 5)), "I2": list(range(200, 240, 5))}
    votes, vframes = _votes_setup(spec)
    kept, phantoms = eliminate_phantoms(list(spec), votes, vframes)
    assert kept == sorted(spec) and not phantoms


# ---------------------------------------------------------------- 聚合
def test_confirmed_cells_mean_conf_gate():
    """低置信双票（人体误检）被均值置信度门控剔除。"""
    votes = {"paper": Counter({"C3": 2, "E2": 2})}
    vconf = {"paper": {"C3": 0.23 + 0.28, "E2": 0.62 + 0.44}}   # 均值 0.255 / 0.53
    vframes = {"paper": {"C3": {1, 2}, "E2": {3, 4}}}
    out = confirmed_cells(["paper"], votes, vconf, vframes,
                          min_votes=VOTE_MIN, min_mean_conf=0.35)
    assert out == ["E2"]


def test_confirmed_cells_phantom_kill_integrated():
    votes = {"paper": Counter({"I2": 11, "C7": 2})}
    vconf = {"paper": {"I2": 6.6, "C7": 1.1}}
    vframes = {"paper": {"I2": {1, 2, 3, 5, 6, 7, 8, 9, 12, 13, 14}, "C7": {4, 10}}}
    out = confirmed_cells(["paper"], votes, vconf, vframes,
                          min_votes=VOTE_MIN, min_mean_conf=0.35, phantom_kill=True)
    assert out == ["I2"]


def test_split_mines_passable_k_column():
    """K 列 paper = 桥洞箭头（可通行），其余列 = 雷。"""
    mines, passable = split_mines_passable(["E2", "E3", "K4", "K9", "F5"])
    assert mines == ["E2", "E3", "F5"]
    assert passable == ["K4", "K9"]


def test_static_target_fire_cell_survives_compaction():
    """静态蓝靶的开火格不能被共线压缩删掉（2026-10-21 实测缺陷）。

    事故形态：video_recon 原先手拼 KEEP，只保"有轨道的靶"的停靠格。
    D11 静态蓝靶的正交邻格 D10 落在 E10→C10 的直线上，压缩当中间格删除，
    adapt 再看不到它 → 车命令一个 '*' 都没有，整局不开火。
    canonical compact_and_adapt 走 mission_keep（含"路线上所有可开火格"），
    能保住。
    """
    from ag_integrate.car_adapter import compact_and_adapt
    from ag_integrate.world_model import TargetRef, WorldSnapshot

    def cell(n):
        c, r = parse_cell(n)
        return CellRef(c, r)

    snapshot = WorldSnapshot(
        timestamp_ms=0, mines=(), passable=(cell("K4"), cell("K9")),
        obstacles=(), targets=(TargetRef(cell("D11"), True),))
    route = ("N10", "M10", "L10", "K10", "J10", "I10", "H10", "G10",
             "F10", "E10", "D10", "C10", "B10", "N6", "N5")
    compact, cmds = compact_and_adapt(snapshot, route, "N10", "y", "y")

    assert "D10" in compact, f"开火格 D10 被压缩删掉: {compact}"
    assert "D10*" in cmds.route.split(), f"车命令缺失开火标记: {cmds.route}"


def test_parse_cell_roundtrip():
    assert parse_cell("A1") == (0, 0)
    assert parse_cell("N11") == (13, 10)
    assert parse_cell("K4") == (10, 3)
    with pytest.raises(ValueError):
        parse_cell("Z1")
    with pytest.raises(ValueError):
        parse_cell("A0")


# ---------------------------------------------------------------- 标签重叠过滤
def test_filter_label_overlap_kills_label_text():
    """720p 大字号格号被 paper 误检（2026-10-17 实拍实锤）→ 与
    cell_label 框重叠即剔除。"""
    from ag_integrate.video_recon import filter_label_overlap
    dets = [
        {"cls": "cell_label", "conf": 0.9, "box": [100, 100, 180, 150]},   # 格号文字
        {"cls": "paper", "conf": 0.88, "box": [105, 102, 150, 148]},      # 误检：标签文字
        {"cls": "paper", "conf": 0.62, "box": [400, 400, 460, 460]},      # 真雷贴纸（格心）
    ]
    out = filter_label_overlap(dets)
    papers = [d for d in out if d["cls"] == "paper"]
    assert len(papers) == 1 and papers[0]["box"][0] == 400   # 标签误检被剔


def test_filter_label_overlap_no_labels_keeps_all():
    from ag_integrate.video_recon import filter_label_overlap
    dets = [{"cls": "paper", "conf": 0.5, "box": [0, 0, 50, 50]}]
    assert filter_label_overlap(dets) == dets


def test_iou_disjoint_and_partial():
    from ag_integrate.video_recon import _iou
    assert _iou([0, 0, 10, 10], [20, 20, 30, 30]) == 0.0
    assert abs(_iou([0, 0, 10, 10], [5, 5, 15, 15]) - 25 / 175) < 1e-9
