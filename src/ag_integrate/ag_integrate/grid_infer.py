# -*- coding: utf-8 -*-
"""GridInferencer：邻格推理层——用可信格号锚定网格单应，几何推算全场格名。

问题（2026-10-24 实飞视频实证）：v4d 格号分类器对**边缘格**（行 1、行 11、
A 列）误读——速度快时边缘格在画面里小、透视拉伸、运动模糊，96x96 输入分辨率
不够。误读的 label 直接进 MapLocalizer 的 PnP 会把位姿带偏一格。

方案（"用旁边格的格号推理"的数学化）：
    场地是规则 14×11 网格 ⇒ **少量可信格号 + 单应 H = 全场格名几何唯一确定**。
    1) 每帧把所有 cell_label 的 (v4d 读数→理论格心, 像素中心) 配对；
    2) RANSAC 拟合单应（像面→场地平面）：正确读数是多数、错读是离群点——
       被自然剔除；H 的 inlier 集即"可信锚点"；
    3) 对任何目标（读错的 label、无 label 的雷/箭头/障碍）投影其像素位置
       → 网格坐标 → round 出格名——**不再依赖每个格子自己读对**；
    4) 交叉验证：inlier 上的 v4d 读数与几何格名一致 ⇒ 网格可信；错读被覆盖。

鲁棒性：
    - 最少 4 个 inlier 锚点（网格可解下限）；<4 时退化为纯 v4d 读数（旧行为）
    - 跨帧累积：H 的 inlier 锚点缓存复用（相机移动时按重投影误差淘汰旧锚点）
    - 网格置信 = inlier 数 + 重投影 RMS；输出带 origin 标记（label / inferred）
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

COLS = 14
ROWS = 11
LETTERS = "ABCDEFGHIJKLMN"


def cell_center_grid(name: str) -> tuple[float, float]:
    """格名 → 场地网格坐标（列 x，行 y；格距 1，原点在 A1 格心）。"""
    col = LETTERS.index(name[0].upper())
    row = int(name[1:]) - 1
    return float(col), float(row)


def grid_to_cell(x: float, y: float) -> tuple[str, float]:
    """网格坐标 → (格名, 距格心距离)；越界返回 ('', inf)。"""
    c, r = int(round(x)), int(round(y))
    if not (0 <= c < COLS and 0 <= r < ROWS):
        return "", math.inf
    dist = math.hypot(x - c, y - r)
    return f"{LETTERS[c]}{r + 1}", dist


# ---------------- 单应最小二乘 / RANSAC ---------------- #

def _fit_homography_dlt(pts_a: np.ndarray, pts_b: np.ndarray) -> np.ndarray | None:
    """DLT 解 3x3 单应 H（a→b，齐次）。最少 4 点。返回 None 若退化。"""
    n = len(pts_a)
    if n < 4:
        return None
    A = np.zeros((2 * n, 9))
    for i, ((ax, ay), (bx, by)) in enumerate(zip(pts_a, pts_b)):
        A[2 * i] = [ax, ay, 1, 0, 0, 0, -bx * ax, -bx * ay, -bx]
        A[2 * i + 1] = [0, 0, 0, ax, ay, 1, -by * ax, -by * ay, -by]
    try:
        _, _, Vt = np.linalg.svd(A)
    except np.linalg.LinAlgError:
        return None
    H = Vt[-1].reshape(3, 3)
    if abs(H[2, 2]) < 1e-12:
        return None
    return H / H[2, 2]


def _apply_h(H: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """H 应用到 N×2 点，返回 N×2（除以 w；w≈0 时给 inf 让误差爆炸）。"""
    pts_h = np.hstack([pts, np.ones((len(pts), 1))])
    out = pts_h @ H.T
    w = out[:, 2]
    w_safe = np.where(np.abs(w) < 1e-12, 1e-12, w)
    return out[:, :2] / w_safe[:, None]


def ransac_homography(px: np.ndarray, grid: np.ndarray, *, iters: int = 60,
                      tol: float = 0.35, seed: int = 7) -> tuple[np.ndarray | None,
                                                                 np.ndarray, float]:
    """RANSAC：像面点 px(N,2) → 网格点 grid(N,2) 的单应。

    tol：重投影容差（网格单位；0.35 = 恰一格内）。
    返回 (H 或 None, inlier 布尔, inlier 重投影 RMS)。
    """
    n = len(px)
    if n < 4:
        return None, np.zeros(n, bool), math.inf
    rng = np.random.default_rng(seed)
    best_inliers = np.zeros(n, bool)
    best_rms = math.inf
    best_H = None
    for _ in range(iters):
        idx = rng.choice(n, 4, replace=False)
        H = _fit_homography_dlt(px[idx], grid[idx])
        if H is None:
            continue
        proj = _apply_h(H, px)
        err = np.linalg.norm(proj - grid, axis=1)
        inliers = err < tol
        if inliers.sum() >= 4 and inliers.sum() > best_inliers.sum():
            # 用 inliers 重拟合提精度
            H2 = _fit_homography_dlt(px[inliers], grid[inliers])
            if H2 is not None:
                err2 = np.linalg.norm(_apply_h(H2, px) - grid, axis=1)
                in2 = err2 < tol
                if in2.sum() >= 4:
                    best_H, best_inliers = H2, in2
                    best_rms = float(np.sqrt((err2[in2] ** 2).mean()))
    if best_H is None and best_inliers.sum() >= 4:
        best_H = _fit_homography_dlt(px[best_inliers], grid[best_inliers])
    return best_H, best_inliers, best_rms


# ---------------- 推理器 ---------------- #

@dataclass
class CellRead:
    """一次格号读数（v4d 原始输出或推理覆盖）。"""
    cell: str
    px_x: float
    px_y: float
    source: str          # "label"=v4d 原读 / "inferred"=几何推理 / "label+geo"=双源一致
    label_raw: str = ""  # v4d 原始读数（被覆盖时留证）
    resid: float = 0.0   # 网格推理残差（格）


@dataclass
class GridInferencer:
    """邻格推理：帧级网格锚定 + 目标落格几何化。"""
    tol: float = 0.35                    # RANSAC 容差（格）
    min_anchors: int = 4                 # 解单应下限
    max_cache: int = 60                  # 跨帧锚点缓存
    # 缓存锚点（像素坐标在帧间会变——缓存的是"网格↔本帧像面"的最新 H 下
    # 仍一致的锚点；相机大幅移动时重投影误差淘汰旧点）
    _cache_px: list = field(default_factory=list)
    _cache_grid: list = field(default_factory=list)
    last_rms: float = math.inf
    last_n_inlier: int = 0

    def infer(self, labels_px: list[tuple[float, float, str]],
              targets_px: list[tuple[float, float]] | None = None
              ) -> tuple[list[CellRead], list[CellRead]]:
        """一帧推理。

        labels_px: [(px_x, px_y, 格名读数), ...]——v4d 的 cell_label 输出
        targets_px: 需要落格的目标像素中心（雷/箭头/障碍 bbox 中心）

        返回 (label_reads, target_reads)：
            label_reads —— cell_label 的修正读数（含 inferred 覆盖）
            target_reads —— 目标格名（全部来自几何投影）
        """
        px_all: list[tuple[float, float]] = []
        grid_all: list[tuple[float, float]] = []
        raw_names: list[str] = []
        for (x, y, name) in labels_px:
            if len(name) >= 2 and name[0].upper() in LETTERS and name[1:].isdigit():
                gx, gy = cell_center_grid(name)
                px_all.append((x, y)); grid_all.append((gx, gy)); raw_names.append(name)
        # 合并缓存锚点
        px_arr = np.array(px_all + self._cache_px, dtype=float)
        grid_arr = np.array(grid_all + self._cache_grid, dtype=float)
        n_frame = len(px_all)

        H, inliers, rms = (None, np.zeros(0, bool), math.inf)
        if len(px_arr) >= self.min_anchors:
            H, inliers, rms = ransac_homography(px_arr, grid_arr, tol=self.tol)

        label_reads: list[CellRead] = []
        target_reads: list[CellRead] = []

        if H is None:
            # 几何不可解：退化为纯 v4d（旧行为），目标落格无法给（空）
            for (x, y, name) in labels_px:
                label_reads.append(CellRead(name, x, y, "label"))
            self.last_rms, self.last_n_inlier = math.inf, 0
            return label_reads, target_reads

        # 缓存维护：inlier 的帧锚点入缓存（网格-像素对）；重投影误差筛旧点
        proj_all = _apply_h(H, px_arr)
        keep_px, keep_grid = [], []
        for i in range(len(px_arr)):
            if inliers[i]:
                keep_px.append(tuple(px_arr[i])); keep_grid.append(tuple(grid_arr[i]))
        if len(keep_px) > self.max_cache:
            keep_px, keep_grid = keep_px[-self.max_cache:], keep_grid[-self.max_cache:]
        self._cache_px, self._cache_grid = keep_px, keep_grid
        self.last_rms, self.last_n_inlier = rms, int(inliers.sum())

        # label 修正：帧内每读数投影回网格
        for i, (x, y, name) in enumerate(labels_px):
            gx, gy = proj_all[i]
            cell_geo, resid = grid_to_cell(gx, gy)
            if not cell_geo:
                # 落在网格外（误检/出界）——保留原读数但标记
                label_reads.append(CellRead(name, x, y, "label", label_raw=name, resid=resid))
                continue
            if cell_geo == name.upper():
                label_reads.append(CellRead(cell_geo, x, y, "label+geo", resid=resid))
            elif inliers[i]:
                # 是 inlier 却与几何不一致（tol 内 round 到邻格的极少数）
                label_reads.append(CellRead(cell_geo, x, y, "inferred",
                                            label_raw=name, resid=resid))
            else:
                # 离群读数 = 边缘误读 —— 几何覆盖（核心修复路径）
                label_reads.append(CellRead(cell_geo, x, y, "inferred",
                                            label_raw=name, resid=resid))
        # 目标落格（雷/箭头/障碍）：纯几何
        for (x, y) in (targets_px or []):
            gx, gy = _apply_h(H, np.array([[x, y]], dtype=float))[0]
            cell_geo, resid = grid_to_cell(gx, gy)
            if cell_geo:
                target_reads.append(CellRead(cell_geo, x, y, "inferred", resid=resid))
        return label_reads, target_reads

    @property
    def grid_ok(self) -> bool:
        return self.last_n_inlier >= self.min_anchors and self.last_rms < self.tol


# ---------------- 世界坐标版（消费 dzuav UDP 帧，2026-10-24） ---------------- #

def infer_world_frame(labels_world: list[tuple[float, float, str]],
                       targets_world: list[tuple[float, float]] | None = None,
                       *, tol: float = 0.35, min_anchors: int = 4
                       ) -> tuple[list[CellRead], list[CellRead]]:
    """世界坐标版邻格推理（无像素、已投影场景）。

    输入：cell_label 的 (世界x, 世界y, 读数)。多数 anchors 读对时 PnP 世界
    坐标准 ⇒ (世界↔网格) 近似仿射（PnP 用错读也会全局偏——RANSAC 剔错读
    后对剩余 anchors 拟合仿射即可恢复）。
    返回同 infer()：修正读数 + 目标几何落格。
    """
    px: list[tuple[float, float]] = []
    grid: list[tuple[float, float]] = []
    valid: list[int] = []
    for i, (x, y, name) in enumerate(labels_world):
        if len(name) >= 2 and name[0].upper() in LETTERS and name[1:].isdigit():
            gx, gy = cell_center_grid(name)
            px.append((x, y)); grid.append((gx, gy)); valid.append(i)
    if len(px) < min_anchors:
        out = [CellRead(n, x, y, "label") for (x, y, n) in labels_world]
        return out, []
    px_a = np.array(px); grid_a = np.array(grid)
    # 仿射（3 点定）：RANSAC iters 少些
    rng = np.random.default_rng(11)
    best_in = np.zeros(len(px_a), bool); best_M = None
    for _ in range(40):
        idx = rng.choice(len(px_a), 3, replace=False)
        M = _fit_affine(px_a[idx], grid_a[idx])
        if M is None: continue
        err = np.linalg.norm(px_a @ M[:2, :2].T + M[:2, 2], axis=1) - 0  # grid
        # 注意方向：M 是 px→grid
        proj = px_a @ M[:2, :2].T + M[:2, 2]
        err = np.linalg.norm(proj - grid_a, axis=1)
        inl = err < tol
        if inl.sum() >= min_anchors and inl.sum() > best_in.sum():
            M2 = _fit_affine(px_a[inl], grid_a[inl])
            if M2 is not None:
                proj2 = px_a @ M2[:2, :2].T + M2[:2, 2]
                err2 = np.linalg.norm(proj2 - grid_a, axis=1)
                in2 = err2 < tol
                if in2.sum() >= min_anchors:
                    best_M, best_in = M2, in2
    label_reads: list[CellRead] = []
    target_reads: list[CellRead] = []
    if best_M is None:
        return [CellRead(n, x, y, "label") for (x, y, n) in labels_world], []
    proj_all = px_a @ best_M[:2, :2].T + best_M[:2, 2]
    for k, i in enumerate(valid):
        x, y, name = labels_world[i]
        cell_geo, resid = grid_to_cell(*proj_all[k])
        if not cell_geo:
            label_reads.append(CellRead(name, x, y, "label", name, resid)); continue
        src = "label+geo" if cell_geo == name.upper() else "inferred"
        label_reads.append(CellRead(cell_geo, x, y, src, name if src == "inferred" else "", resid))
    for (x, y) in (targets_world or []):
        g = np.array([[x, y]]) @ best_M[:2, :2].T + best_M[:2, 2]
        cell_geo, resid = grid_to_cell(*g[0])
        if cell_geo:
            target_reads.append(CellRead(cell_geo, x, y, "inferred", resid=resid))
    return label_reads, target_reads


def _fit_affine(a: np.ndarray, b: np.ndarray) -> np.ndarray | None:
    """最小二乘仿射 a→b（3x3 齐次，末行 [0,0,1]）。"""
    if len(a) < 3: return None
    A = np.hstack([a, np.ones((len(a), 1))])
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    M = np.eye(3)
    M[:2, :] = sol.T
    return M
