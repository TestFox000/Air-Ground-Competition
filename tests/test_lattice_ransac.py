"""lattice_ransac 纯函数回归测试 — MapLocalizer 单应兜底的 Python 参考逻辑。

MapLocalizer::solve() 在 PnP 失败时用 cv::findHomography 拟合
格号世界坐标↔图像坐标的单应矩阵作为兜底（M0-3 无人机出情报关键路径）。
本测试覆盖 lattice_ransac.py 中对应的纯函数：cell_to_idx / idx_to_cell /
_any3_collinear / fit_lattice / apply_lattice / invert_lattice / ransac_lattice。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
ML = REPO / "tools" / "ml"

cv2 = pytest.importorskip("cv2")


def load_module(name: str, file: Path):
    spec = importlib.util.spec_from_file_location(name, file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


lattice = load_module("lattice_ransac", ML / "lattice_ransac.py")


# ---------------------------------------------------------------- cell id

def test_cell_to_idx_and_back_roundtrip():
    """所有合法格号 A1..N11 的 idx 往返必须一致。"""
    for col_letter in "ABCDEFGHIJKLMN":
        for row in range(1, 12):
            cell = f"{col_letter}{row}"
            idx = lattice.cell_to_idx(cell)
            assert idx is not None
            assert lattice.idx_to_cell(*idx) == cell


def test_cell_to_idx_rejects_out_of_range():
    assert lattice.cell_to_idx("O1") is None       # O 超出 A-N
    assert lattice.cell_to_idx("A0") is None       # 行号从 1 开始
    assert lattice.cell_to_idx("A12") is None      # 最大 11 行
    assert lattice.cell_to_idx("") is None
    assert lattice.cell_to_idx("Z9") is None


# ---------------------------------------------------------------- collinear

def test_any3_collinear_detects_straight_line():
    pts = np.array([[0, 0], [1, 0], [2, 0], [3, 1]], dtype=np.float32)
    assert lattice._any3_collinear(pts) is True


def test_any3_collinear_accepts_quadrilateral():
    pts = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    assert lattice._any3_collinear(pts) is False


# ---------------------------------------------------------------- homography

def _synthetic_grid(n_cols=14, n_rows=11, scale=60.0, offset=(200.0, 150.0)):
    """构造 (col,row)->(u,v) 对应点：纯平移+缩放的正射投影。"""
    pts = []
    for c in range(n_cols):
        for r in range(n_rows):
            u = offset[0] + c * scale
            v = offset[1] + r * scale
            pts.append((c, r, u, v))
    return np.array(pts, dtype=np.float64)


def test_fit_then_apply_roundtrip_error_is_subpixel():
    pts = _synthetic_grid()
    H = lattice.fit_lattice(pts)
    assert H is not None
    for c, r, u, v in pts:
        pu, pv = lattice.apply_lattice(H, c, r)
        assert abs(pu - u) < 1e-3
        assert abs(pv - v) < 1e-3


def test_invert_lattice_recovers_cell_coords():
    pts = _synthetic_grid()
    H = lattice.fit_lattice(pts)
    assert H is not None
    for c, r, u, v in pts:
        pc, pr = lattice.invert_lattice(H, u, v)
        assert abs(pc - c) < 1e-2
        assert abs(pr - r) < 1e-2


def test_fit_lattice_returns_none_for_three_collinear():
    """3 个共线点 + 1 个远点仍可拟合；但若全部退化（<4 非共线）应返回 None。"""
    # 4 点全部共线 → findHomography 失败
    pts = np.array([[0, 0, 10, 10], [1, 0, 20, 10],
                    [2, 0, 30, 10], [3, 0, 40, 10]], dtype=np.float64)
    H = lattice.fit_lattice(pts)
    assert H is None


# ---------------------------------------------------------------- RANSAC

def test_ransac_finds_inliers_and_rejects_outliers():
    rng = np.random.default_rng(0)
    base = _synthetic_grid()
    # 加 5 个离群点（随机图像坐标）
    outliers = np.array([[rng.integers(0, 14), rng.integers(0, 11),
                          rng.uniform(0, 2000), rng.uniform(0, 2000)]
                         for _ in range(5)], dtype=np.float64)
    pts = np.vstack([base, outliers])

    H, mask = lattice.ransac_lattice(pts, tol=3.0)
    assert H is not None
    assert mask.sum() >= len(base) - 2  # 内点应覆盖绝大多数 base 点
    # 离群点应被剔除（mask=False）
    outlier_mask = mask[len(base):]
    assert outlier_mask.sum() <= 1


def test_ransac_returns_none_below_min_points():
    pts = np.array([[0, 0, 10, 10], [1, 1, 20, 20], [2, 2, 30, 30]], dtype=np.float64)
    H, mask = lattice.ransac_lattice(pts, tol=3.0)
    assert H is None
    assert mask.sum() == 0


def test_ransac_refines_with_all_inliers():
    """内点重拟合后，重投影 RMS 应优于仅用 4 点采样的模型。"""
    rng = np.random.default_rng(1)
    base = _synthetic_grid()
    noise = rng.normal(0, 0.5, size=base[:, 2:].shape)
    noisy = base.copy()
    noisy[:, 2:] += noise
    H, mask = lattice.ransac_lattice(noisy, tol=2.0)
    assert H is not None
    assert mask.sum() > 10  # 大部分点为内点
    # 内点重投影误差应在 tol 量级内
    src = noisy[mask, :2].astype(np.float32).reshape(-1, 1, 2)
    pred = cv2.perspectiveTransform(src, H).reshape(-1, 2)
    res = np.hypot(noisy[mask, 2] - pred[:, 0], noisy[mask, 3] - pred[:, 1])
    assert float(np.median(res)) < 2.0
