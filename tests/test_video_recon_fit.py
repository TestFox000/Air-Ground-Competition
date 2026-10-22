# -*- coding: utf-8 -*-
"""网格拟合 + 时间外推验收门回归（2026-10-20，docs/标靶打击优化方案-20261020.md §三）。

背景：真机视频网格拟合成功率仅 50~62%，主因是拟合失败/跳过的帧（只有 2 个
可读标签的帧直接跳过拟合）整帧零投票。本次新增 validate_affine_carry：
此类帧在帧距内用最近成功仿射补位，但必须通过 ≥2 个可信读数的投影残差
验收——相机没大动才放行，动了立即拒绝（宁丢帧不投错票）。

已知局限（本次不改，特征测试锁定现状）：恰 3 读数时任何组合都精确过定
（残差恒 0），单误读不可检测——拟合会静默成功且可能被毒化；共识冗余
从第 4 个读数才开始。真机 A/B 时若此类帧成灾，对策是 n=3 帧也过验收门。

本文件全部合成数据（真机视频 A/B 待做）；拟合行为与验收门语义在此锁定，
任一侧改动（FIT_INLIER/CARRY_* 阈值、拟合器、验收函数）都应先红这里。
"""
from __future__ import annotations

import numpy as np

from ag_integrate.video_recon import (
    CARRY_INLIER,
    GRID_MAE_MAX,
    fit_consensus_affine,
    validate_affine_carry,
)

# 已知良好仿射：grid = [px, 1] @ A（像素 (c*80+40, r*60+30) 读回格 (c, r)）
_A_GOOD = np.array([[1 / 80.0, 0.0], [0.0, 1 / 60.0], [-0.5, -0.5]])


def _reads(points: list[tuple[int, int, float, float]]) -> list[tuple]:
    """构造 reads：[(col, row, px_x, px_y)] -> [(li, ri, lc, rc, box)]。"""
    return [(c, r, 0.9, 0.9, [x - 14, y - 10, x + 14, y + 10])
            for c, r, x, y in points]


def _px_grid(points: list[tuple[int, int, float, float]]):
    px = np.array([[p[2], p[3]] for p in points], float)
    grid = np.array([[p[0], p[1]] for p in points], float)
    return px, grid


_PTS_3_CLEAN = [(2, 3, 2 * 80 + 40, 3 * 60 + 30),
                (9, 2, 9 * 80 + 40, 2 * 60 + 30),
                (5, 8, 5 * 80 + 40, 8 * 60 + 30)]


# ---------------- fit_consensus_affine（存量行为锁定） ----------------

def test_fit_exact_affine_all_inliers():
    pts = _PTS_3_CLEAN + [(12, 10, 12 * 80 + 40, 10 * 60 + 30),
                          (6, 6, 6 * 80 + 40, 6 * 60 + 30)]
    aff, inl, mae = fit_consensus_affine(*_px_grid(pts))
    assert aff is not None
    assert inl.sum() == 5
    assert mae < 1e-6


def test_fit_survives_one_misread_of_four():
    # 4 读数 1 远端误读（声称格偏离真格 8+ 格）：干净组合胜出，误读被剔出内点
    pts = _PTS_3_CLEAN + [(12, 1, 4 * 80 + 40, 10 * 60 + 30)]   # 误读：真格(4,10)
    aff, inl, mae = fit_consensus_affine(*_px_grid(pts))
    assert aff is not None
    assert inl.sum() == 3
    assert not inl[3]


def test_fit_near_misread_ambiguous_known_limit():
    # 特征测试（锁定现状，非断言正确）：4 读数 1 近距误读（偏移 (3,-3)）时
    # 所有 3 点组合都精确过定、内点数恒 3——共识无法定位坏点，误读是否
    # 入选取决于 1e-15 量级的浮点细节（实测被 (0,2,3) 毒化组合偷走）。
    # 单靠帧内几何无解；真机防线 = 下游投票门 + 幻影消除 + 靶先验裁决。
    pts = _PTS_3_CLEAN + [(7, 7, 4 * 80 + 40, 10 * 60 + 30)]   # 误读：真格(4,10)
    aff, inl, mae = fit_consensus_affine(*_px_grid(pts))
    assert aff is not None
    assert inl.sum() == 3          # 歧义存在：恰 3 内点，谁是坏点不可知


def test_fit_needs_three_reads():
    # 2 读数无法定仿射：帧级跳过拟合——时间外推要救的正是这种帧
    px, grid = _px_grid(_PTS_3_CLEAN[:2])
    aff, _, mae = fit_consensus_affine(px, grid)
    assert aff is None and mae > 1.0


def test_fit_three_reads_always_solves_known_limit():
    # 特征测试（锁定现状，非断言正确）：恰 3 读数时 3 点精确过定、残差恒 0，
    # 单误读不可检测——拟合静默成功且可能被毒化。共识冗余从第 4 读数开始。
    pts = _PTS_3_CLEAN[:2] + [(7, 7, 4 * 80 + 40, 10 * 60 + 30)]   # 1 误读
    aff, inl, mae = fit_consensus_affine(*_px_grid(pts))
    assert aff is not None          # 不失败——这就是"静默毒化"局限本身
    assert inl.sum() == 3 and mae < GRID_MAE_MAX


# ---------------- validate_affine_carry（新增验收门） ----------------

def test_carry_accepts_two_consistent_reads():
    reads = _reads([(2, 3, 2 * 80 + 40, 3 * 60 + 30),
                    (9, 2, 9 * 80 + 40, 2 * 60 + 30)])
    out = validate_affine_carry(_A_GOOD, reads)
    assert out is not None
    keep, mae = out
    assert list(keep) == [0, 1]
    assert mae <= GRID_MAE_MAX


def test_carry_accepts_with_misread_excluded():
    # 3 读数 1 误读：验收门用 2 个干净读数放行，误读读数被剔出（不参与投票）
    reads = _reads([(2, 3, 2 * 80 + 40, 3 * 60 + 30),
                    (9, 2, 9 * 80 + 40, 2 * 60 + 30),
                    (7, 7, 4 * 80 + 40, 10 * 60 + 30)])
    out = validate_affine_carry(_A_GOOD, reads)
    assert out is not None
    keep, _ = out
    assert list(keep) == [0, 1]


def test_carry_rejects_moved_camera():
    # 相机平移 1.5 格：所有读数残差同向爆门，验收拒绝（宁丢帧不投错票）
    reads = _reads([(2, 3, 2 * 80 + 40 + 120, 3 * 60 + 30 + 90),
                    (9, 2, 9 * 80 + 40 + 120, 2 * 60 + 30 + 90)])
    assert validate_affine_carry(_A_GOOD, reads) is None


def test_carry_rejects_single_read():
    reads = _reads([(2, 3, 2 * 80 + 40, 3 * 60 + 30)])
    assert validate_affine_carry(_A_GOOD, reads) is None


def test_carry_rejects_none_affine():
    reads = _reads([(2, 3, 2 * 80 + 40, 3 * 60 + 30),
                    (9, 2, 9 * 80 + 40, 2 * 60 + 30)])
    assert validate_affine_carry(None, reads) is None


def test_carry_tolerance_no_worse_than_fit():
    # 验收门与拟合内点同口径（0.45 格）：不给外推帧更松的几何容忍
    assert CARRY_INLIER <= 0.45


def test_gate3_arbitration_kills_silent_poison():
    # n=3 强制验收门（2026-10-22 落地）的语义锚：新鲜拟合"成功"但含误读
    # （静默毒化——3 点必精确过定），帧级处理不采信新鲜解，改用最近成功
    # 仿射当仲裁者：≥2 读数一致 → 放行并剔出误读；不一致 → 丢帧。
    pts = _PTS_3_CLEAN[:2] + [(7, 7, 4 * 80 + 40, 10 * 60 + 30)]   # 1 误读
    aff, inl, mae = fit_consensus_affine(*_px_grid(pts))
    assert aff is not None and inl.sum() == 3       # 拟合"成功"= 毒化解本身
    out = validate_affine_carry(_A_GOOD, _reads(pts))
    assert out is not None                          # 仲裁：2 干净读数放行
    assert list(out[0]) == [0, 1]                   # 误读读数被剔出（不投票）
