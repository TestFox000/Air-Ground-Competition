# -*- coding: utf-8 -*-
"""GridInferencer 两道防护门行为锁定（2026-10-26，真机 397 帧抽帧反例驱动）。

门① 防缓存毒化：帧距大时旧锚点 px 过期，混合 RANSAC 被旧多数派绑架
（实测 stride15 双源一致率 82.2%→6.4%）——两级拟合 + 新帧共识门 + 清缓存自愈。
门② 防锚域外推：低锚点帧（4~6 锚）远端外推把 B10 纠成 C8/C10 且跨帧
不自洽——纠正/落格仅限锚点支撑域内（切比雪夫 ≤1.5 格 + 锚点 ≥6）。
原始合成场景（边缘误读纠正、50% 误读压力）同时锁定，防门误伤。
"""

from __future__ import annotations

import numpy as np

from ag_integrate.grid_infer import (GridInferencer, LETTERS, cell_center_grid,
                                    infer_world_frame)

SX, SY = 45.0, 40.0          # 格距（px），合成像机


def _frame(cells, ox, oy, corrupt=None):
    """格名列表 -> [(px_x, px_y, 读数)]；px = 网格坐标 * 格距 + 相机平移。

    corrupt: {真格名: 误读}——标签物理位置不变、v4d 读错。
    """
    out = []
    for name in cells:
        gx, gy = cell_center_grid(name)
        out.append((ox + gx * SX, oy + gy * SY, (corrupt or {}).get(name, name)))
    return out


def _block(cols, rows):
    return [f"{c}{r}" for c in cols for r in rows]


# ---------------- 门①：防缓存毒化 ---------------- #

def test_gate1_fresh_fit_authoritative_over_stale_cache():
    """相机大幅移动后，本帧读数独立拟合优先——不被旧缓存绑架。

    旧实现在此场景把整帧新读数全部误判 inferred（毒化 6.4% 一致率根因）。
    """
    gi = GridInferencer()
    cells = _block("FGHIJ", range(5, 10))
    gi.infer(_frame(cells, 200, 300))                 # 第一帧锚定
    assert gi.last_h_src == "fit"
    lr2, _ = gi.infer(_frame(cells, 200 - 6 * SX, 300 - 2 * SY))   # 平移 6 格
    assert gi.last_h_src == "fit"                     # 新帧独立可解，权威
    assert all(r.source == "label+geo" for r in lr2)  # 无一被旧网格覆盖


def test_gate1_carry_bridges_few_reads_under_small_motion():
    """30fps 小运动下 3 读数帧：缓存辅助解过共识门，网格连续。"""
    gi = GridInferencer()
    gi.infer(_frame(_block("FGHIJ", range(5, 10)), 200, 300))
    lr, _ = gi.infer(_frame(["F6", "G6", "H6"], 200 + 8, 300 + 6))  # 0.2 格内
    assert gi.last_h_src == "carry" and gi.grid_ok
    assert all(r.source == "label+geo" for r in lr)


def test_gate1_carry_rejects_stale_grid_and_self_heals():
    """大运动 + 读数不足：共识门拒绝旧网格、清缓存；下帧满锚点自愈。"""
    gi = GridInferencer()
    cells = _block("FGHIJ", range(5, 10))
    gi.infer(_frame(cells, 200, 300))
    lr2, _ = gi.infer(_frame(["F6", "G6", "H6"], 200 - 6 * SX, 300))
    assert gi.last_h_src == "" and not gi.grid_ok     # 弃解
    assert all(r.source == "label" for r in lr2)      # 读数保留原值
    assert gi._cache_px == []                         # 证据反对 → 清缓存
    lr3, _ = gi.infer(_frame(cells, 200 - 6 * SX, 300))
    assert gi.last_h_src == "fit" and gi.grid_ok      # 自愈
    assert all(r.source == "label+geo" for r in lr3)


def test_gate1_single_read_is_not_evidence_keeps_cache():
    """单读数帧共识门必然不过，但证据不足 ≠ 证据反对——不动缓存。"""
    gi = GridInferencer()
    gi.infer(_frame(_block("FGHIJ", range(5, 10)), 200, 300))
    gi.infer(_frame(["F6"], 200, 300))
    assert gi._cache_px, "证据不足时不应清缓存"
    lr, _ = gi.infer(_frame(["F6", "G6", "H6"], 200 + 4, 300 + 3))
    assert gi.last_h_src == "carry"                   # 缓存仍可用


# ---------------- 门②：防锚域外推 ---------------- #

def test_gate2_far_read_outside_anchor_support_not_corrected():
    """锚域外的分歧读数保留原读（低锚帧 B10→C8/C10 跨帧不自洽的根因）。"""
    gi = GridInferencer()
    cells = _block("FGHIJ", range(5, 10)) + ["B11"]
    lr, _ = gi.infer(_frame(cells, 200, 300, corrupt={"B11": "B10", "G7": "J7"}))
    far = next(r for r in lr if r.px_x < 300)         # B11 位置
    assert far.cell == "B10" and far.source == "label" and far.gated
    near = next(r for r in lr if r.label_raw == "J7")  # 锚域内误读
    assert near.cell == "G7" and near.source == "inferred"


def test_gate2_target_outside_anchor_support_not_assigned():
    """目标落格同受锚域门：域外宁可不落格（错格雷=错路线，比缺格贵）。"""
    gi = GridInferencer()
    cells = _block("FGHIJ", range(5, 10))
    t_far = (200 + 1 * SX, 300 + 10 * SY)             # B11 世界位（锚域外）
    t_near = (200 + 6 * SX, 300 + 6 * SY)             # G7（锚域内）
    lr, tr = gi.infer(_frame(cells, 200, 300), [t_far, t_near])
    assert [t.cell for t in tr] == ["G7"]


def test_gate2_min_anchors_floor_blocks_correction():
    """锚点数不足（<6）时即便域内也不纠正——4~6 锚帧外推反例实测带。"""
    gi = GridInferencer()
    lr, _ = gi.infer(_frame(["F5", "F6", "G5", "H6", "H7", "G6"], 200, 300,
                            corrupt={"G6": "J7"}))
    assert gi.grid_ok and gi.last_n_inlier == 5        # 5 锚网格本身可解
    bad = next(r for r in lr if r.gated)
    assert bad.cell == "J7" and bad.gated              # 但纠正被门拦下


def test_gate2_infer_world_frame_same_gating():
    """世界坐标版同口径：域外保留原读、目标不落格。"""
    labels = [(gx * 0.35, gy * 0.35, "B10")
              for (gx, gy) in [cell_center_grid("B11")]]
    labels += [(gx * 0.35, gy * 0.35, "J7")
               for (gx, gy) in [cell_center_grid("G7")]]
    labels += [(*[v * 0.35 for v in cell_center_grid(f"{c}{r}")], f"{c}{r}")
               for c in "FGHIJ" for r in range(5, 10)
               if not (c == "G" and r == 7)]
    out, tr = infer_world_frame(labels, [(1 * 0.35, 10 * 0.35)])
    far = next(r for r in out if r.px_x < 2.0)
    assert far.cell == "B10" and far.gated
    assert any(r.cell == "G7" and r.source == "inferred" for r in out)
    assert tr == []                                    # B11 域外目标不落格


# ---------------- 原合成场景（防门误伤） ---------------- #

def test_edge_misread_within_support_still_corrected():
    """边缘格（A 列/1 行）误读在锚点支撑域内——必须仍被纠正（原价值主张）。"""
    gi = GridInferencer()
    cells = _block("ABCDE", range(1, 6)) + ["F3"]
    lr, _ = gi.infer(_frame(cells, 200, 300, corrupt={"A3": "J7", "C1": "N9"}))
    assert any(r.cell == "A3" and r.source == "inferred" for r in lr)
    assert any(r.cell == "C1" and r.source == "inferred" for r in lr)


def test_half_misread_stress_all_recovered():
    """50% 误读率压力（e976d523 原合成验证口径）：25 读 12 错仍全对。"""
    rng = np.random.default_rng(3)
    cells = _block("FGHIJ", range(5, 10))
    victims = list(rng.permutation(cells))[:12]
    # 误读=字母+3 列、行+2（恒异于原格，跨 3 格不与正确格阵共线撞解）
    corrupt = {v: f"{LETTERS[(LETTERS.index(v[0]) + 3) % 14]}"
                  f"{(int(v[1:]) + 2) % 11 + 1}" for v in victims}
    gi = GridInferencer()
    lr, _ = gi.infer(_frame(cells, 200, 300, corrupt=corrupt))
    assert {r.cell for r in lr} == set(cells)          # 全部恢复真格名
    assert sum(r.source == "inferred" for r in lr) == 12
