"""单应矩阵标定：无人机俯视像素 <-> 场地米制坐标。

为什么放在 ag_core：
    标定时需要的最小二乘求解是纯数学，与 ROS 无关，必须能在笔记本电脑上
    用四个角点的像素坐标离线算出 H，再去现场验证。

用法（现场标定流程）：
    1. 无人机升高到比赛高度，拍一张能看全四角的图
    2. 人工点出四角像素坐标，顺序 A1 -> N1 -> N11 -> A11（左上起顺时针）
    3. calib_homography.py 算出 H，写入 config/params.yaml 的 homography
    4. 用 reprojection_error() 检查残差，> 3 px 说明点位不准，重标

注意四角是 **A1 / N1 / N11 / A11**（14 列 A~N x 11 行 1~11），
不是早期的 A1/K1/K14/A14 —— 那个组合里 K14 根本不在场上。
"""

from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np

__all__ = [
    "field_corners",
    "homography_from_correspondences",
    "reprojection_error",
    "CORNER_ORDER",
]

#: 四角顺序约定：A1(0,0) -> N1(W,0) -> N11(W,L) -> A11(0,L)
CORNER_ORDER = ("A1", "N1", "N11", "A11")


def field_corners(cols: int, rows: int, cell_size: float) -> np.ndarray:
    """场地四角的米制坐标，顺序与 CORNER_ORDER 一致。"""
    w = cols * cell_size
    length = rows * cell_size
    return np.array([[0.0, 0.0], [w, 0.0], [w, length], [0.0, length]], dtype=np.float64)


def _normalize(pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """各向同性归一化，提升 DLT 数值稳定性。"""
    centroid = pts.mean(axis=0)
    d = float(np.sqrt(((pts - centroid) ** 2).sum(axis=1)).mean())
    s = np.sqrt(2.0) / max(d, 1e-12)
    T = np.array(
        [[s, 0.0, -s * centroid[0]], [0.0, s, -s * centroid[1]], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    pts_h = np.hstack([pts, np.ones((len(pts), 1))])
    return T, (T @ pts_h.T).T[:, :2]


def homography_from_correspondences(
    src: Sequence[Sequence[float]],
    dst: Sequence[Sequence[float]],
    normalize: bool = True,
) -> np.ndarray:
    """由点对求单应 H（满足 dst ~= H @ src）。

    src: 像素坐标 (u, v)，至少 4 点
    dst: 场地米制坐标 (x, y)
    """
    src_a = np.asarray(src, dtype=np.float64)
    dst_a = np.asarray(dst, dtype=np.float64)
    if src_a.shape[0] < 4 or dst_a.shape[0] != src_a.shape[0]:
        raise ValueError(f"至少需要 4 组对应点，当前 {src_a.shape[0]} 组")

    if normalize:
        Ts, src_n = _normalize(src_a)
        Td, dst_n = _normalize(dst_a)
    else:
        Ts = Td = np.eye(3)
        src_n, dst_n = src_a, dst_a

    rows = []
    for (x, y), (u, v) in zip(src_n, dst_n):
        rows.append([-x, -y, -1.0, 0.0, 0.0, 0.0, u * x, u * y, u])
        rows.append([0.0, 0.0, 0.0, -x, -y, -1.0, v * x, v * y, v])
    A = np.asarray(rows, dtype=np.float64)

    _, _, vt = np.linalg.svd(A)
    H_n = vt[-1].reshape(3, 3)
    if abs(H_n[2, 2]) > 1e-12:
        H_n = H_n / H_n[2, 2]
    H = np.linalg.inv(Td) @ H_n @ Ts
    if abs(H[2, 2]) > 1e-12:
        H = H / H[2, 2]
    return H


def reprojection_error(H: np.ndarray, src, dst) -> float:
    """重投影残差的 RMSE（像素->场地后与实测场地点位的距离，单位米）。"""
    H = np.asarray(H, dtype=np.float64)
    src_a = np.asarray(src, dtype=np.float64)
    dst_a = np.asarray(dst, dtype=np.float64)
    errs = []
    for (u, v), (x, y) in zip(src_a, dst_a):
        q = H @ np.array([u, v, 1.0])
        if abs(q[2]) < 1e-12:
            errs.append(float("inf"))
            continue
        errs.append(float(np.hypot(q[0] / q[2] - x, q[1] / q[2] - y)))
    if not errs:
        return float("inf")
    return float(np.sqrt(np.mean(np.square(errs))))
