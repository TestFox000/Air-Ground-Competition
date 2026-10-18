"""anchored_registration 锚定配准测试。

锁定的可靠性承诺：
    - 无噪/带噪锚点恢复变换（2 点定解、Kabsch 闭式解正确）；
    - ~11% 格号误读（整格偏移）下 RANSAC 输出不受污染；
    - 共线锚（θ 病态）、锚点不足、互相矛盾的锚集 → 显式报错；
    - 互证规则：n≥4 时无 3 点一致即拒绝（2 点最小集对自身恒零残差）。
"""

import math

import pytest

from ag_integrate.anchored_registration import (
    Anchor,
    RegistrationError,
    solve_transform,
)
from ag_integrate.field_frame import cell_to_world

# 真值变换：p_field = R(θ)·p_slam + t
THETA, TX, TY = 0.6, 1.2, -0.7
COS, SIN = math.cos(THETA), math.sin(THETA)


def _slam_of(fx: float, fy: float) -> tuple[float, float]:
    """由场地坐标反推 SLAM 坐标（真值变换的逆）。"""
    dx, dy = fx - TX, fy - TY
    return (COS * dx + SIN * dy, -SIN * dx + COS * dy)


def _grid_anchors(cells, noise_sigma=0.0, rng=None):
    out = []
    for name in cells:
        fx, fy = cell_to_world(_cell(name))
        sx, sy = _slam_of(fx, fy)
        if noise_sigma:
            sx += rng.gauss(0, noise_sigma)
            sy += rng.gauss(0, noise_sigma)
        out.append(Anchor("grid", sx, sy, fx, fy))
    return out


def _cell(name: str):
    from ag_integrate.field_frame import LETTERS, CellRef
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


SPREAD = ["F5", "D8", "K2", "N11", "B3", "H7", "L9", "C6", "J4"]


def test_exact_recovery():
    anchors = _grid_anchors(SPREAD)
    tf = solve_transform(anchors, seed=7)
    assert tf.confident and tf.rms < 1e-6
    assert abs(tf.theta - THETA) < 1e-9
    # 检查点：远端格心映射误差毫米级
    fx, fy = cell_to_world(_cell("A1"))
    px, py = tf.apply(*_slam_of(fx, fy))
    assert math.hypot(px - fx, py - fy) < 1e-6


def test_noisy_recovery():
    import random
    rng = random.Random(3)
    anchors = _grid_anchors(SPREAD, noise_sigma=0.02, rng=rng)
    tf = solve_transform(anchors, seed=7)
    assert tf.confident
    assert tf.rms < 0.05
    fx, fy = cell_to_world(_cell("A1"))
    px, py = tf.apply(*_slam_of(fx, fy))
    assert math.hypot(px - fx, py - fy) < 0.05


def test_ransac_rejects_misread_anchors():
    """11% 量级格号误读（9 好 2 坏）+ 2 结构锚 → 输出不被污染。"""
    anchors = _grid_anchors(SPREAD)
    # 2 个误读锚：场地位偏移一整格（把 D8 认成了 D9）
    bad = []
    for name in ("D8", "J4"):
        fx, fy = cell_to_world(_cell(name))
        bad.append(Anchor("grid", *_slam_of(fx, fy), fx, fy + 0.35))
    anchors = anchors[:7] + bad
    # 2 个可靠结构锚
    for name in ("G6", "M10"):
        fx, fy = cell_to_world(_cell(name))
        anchors.append(Anchor("structure", *_slam_of(fx, fy), fx, fy))
    tf = solve_transform(anchors, seed=7)
    assert tf.confident
    assert tf.n_inliers >= 9 and tf.rms < 0.05
    fx, fy = cell_to_world(_cell("A1"))
    px, py = tf.apply(*_slam_of(fx, fy))
    assert math.hypot(px - fx, py - fy) < 0.05


def test_collinear_rejected():
    anchors = _grid_anchors(["D3", "D5", "D7", "D9"])   # 同列共线
    with pytest.raises(RegistrationError, match="collinear"):
        solve_transform(anchors, seed=7)


def test_too_few_anchors():
    with pytest.raises(RegistrationError, match=">=2"):
        solve_transform(_grid_anchors(["F5"]))


def test_two_anchor_minimal_solution():
    """2 锚可解但无冗余 → confident=False（可用但需谨慎）。"""
    anchors = _grid_anchors(["F5", "K2"])
    tf = solve_transform(anchors)
    assert not tf.confident
    fx, fy = cell_to_world(_cell("B7"))
    px, py = tf.apply(*_slam_of(fx, fy))
    assert math.hypot(px - fx, py - fy) < 1e-6


def test_mutually_inconsistent_rejected():
    """互相矛盾的锚集（模拟全离群）→ 拒绝，绝不输出随机变换。"""
    anchors = _grid_anchors(["F5", "K2", "B10", "N1", "H6", "C4"])
    # 场地侧全部打乱：与 SLAM 侧无任何一致对应
    shuffled = [Anchor(a.kind, a.slam_x, a.slam_y,
                       a.field_x + 2.1 * ((i % 3) - 1), a.field_y + 1.7 * (i % 2))
                for i, a in enumerate(anchors)]
    with pytest.raises(RegistrationError, match="inconsistent"):
        solve_transform(shuffled, seed=7)


def test_anchor_from_cell_helper():
    a = Anchor.from_cell("F5", 1.0, 2.0)
    fx, fy = cell_to_world(_cell("F5"))
    assert (a.field_x, a.field_y) == (fx, fy)
    assert a.kind == "grid"
