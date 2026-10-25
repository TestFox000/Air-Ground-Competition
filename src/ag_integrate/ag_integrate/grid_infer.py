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
    - 门① 防缓存毒化（2026-10-26，397 帧真机抽帧反例驱动）：本帧读数独立
      拟合优先；缓存只在拟合不足时辅助，且辅助解必须过"新帧共识门"——
      本帧 ≥carry_min_fresh 个读数落在 tol 内，否则弃解并清空缓存。动机：
      帧距大时（stride 采样/掉帧）旧锚点 px 已随相机运动过期，混合 RANSAC
      会被旧多数派绑架（实测双源一致率 82.2%→6.4%）；缓存只收本帧内点。
    - 门② 防锚域外推（同上驱动）：分歧读数的几何纠正与目标落格仅限锚点
      支撑域内——最近锚点切比雪夫距离 ≤anchor_gate 格、且锚点数 ≥
      min_anchors_correct。动机：低锚点帧（4~6 锚）远端外推把 B10 纠成
      C8/C10 且跨帧不自洽；门内纠正经像素原型独立核验 86.5% 支持。
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
    接受门 min_in：n≤4 时 4（旧行为——4 点本身无验证力可用）；n≥5 时 5——
    任意 4 点对应都精确确定单应（零验证力），含 1 误读的 4 点子集照样给出
    完美假解，与正确子集打平后由采样顺序裁决（2026-10-26 测试实证：5 读
    1 错时 J7 误读反成 label+geo、真锚投出场）。≥5 内点才含首个过约束点，
    假解几乎必被淘汰。口径对齐 MapLocalizer 单应兜底的 inlier 下限取 5。
    返回 (H 或 None, inlier 布尔, inlier 重投影 RMS)。
    """
    n = len(px)
    if n < 4:
        return None, np.zeros(n, bool), math.inf
    min_in = 4 if n <= 4 else 5
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
        if inliers.sum() >= min_in and inliers.sum() > best_inliers.sum():
            # 用 inliers 重拟合提精度
            H2 = _fit_homography_dlt(px[inliers], grid[inliers])
            if H2 is not None:
                err2 = np.linalg.norm(_apply_h(H2, px) - grid, axis=1)
                in2 = err2 < tol
                if in2.sum() >= min_in:
                    best_H, best_inliers = H2, in2
                    best_rms = float(np.sqrt((err2[in2] ** 2).mean()))
    if best_H is None and best_inliers.sum() >= min_in:
        best_H = _fit_homography_dlt(px[best_inliers], grid[best_inliers])
    return best_H, best_inliers, best_rms


def within_anchor_gate(gx: float, gy: float, anchors: np.ndarray, gate: float) -> bool:
    """门②判据：投影点距最近锚点（切比雪夫距离，格单位）≤ gate。

    近似"锚点凸包 + gate 边距"——锚点是格点阵时两者几乎等价，口径对齐
    video_recon.ANCHOR_GATE（=1.5）。锚点为空返回 False（无从支撑）。
    """
    if anchors is None or len(anchors) == 0:
        return False
    d = np.abs(anchors - np.array([gx, gy])).max(axis=1)
    return bool(d.min() <= gate)


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
    gated: bool = False  # True=分歧读数因锚域门②保留原读（resid 仍是分歧幅度）


@dataclass
class GridInferencer:
    """邻格推理：帧级网格锚定 + 目标落格几何化。

    infer() 每帧两步：两级拟合（门①防缓存毒化）→ 锚域内纠正/落格（门②
    防外推）。两道门的反例与核验数据见模块 docstring。
    """
    tol: float = 0.35                    # RANSAC 容差（格）
    min_anchors: int = 4                 # 解单应下限
    max_cache: int = 60                  # 跨帧锚点缓存
    carry_min_fresh: int = 2             # 门①：缓存辅助解需 ≥N 个本帧读数为内点
    anchor_gate: float = 1.5             # 门②：纠正/落格点距最近锚点（格，切比雪夫）
    min_anchors_correct: int = 6         # 门②：纠正/落格的锚点数下限
    # 门②工作点（397 帧真机扫描 sweep_gates.py）：1.5/6 为最优——收紧到 1.0
    # 会把锚点间隙（格点阵空档）里的正当纠正一起拦掉（判对率 90.5%→72.6%，
    # F5 类核心纠正掉 1/3），0.8 几乎全拦；1.2/7 仅 +0.7pp 不值得偏离
    # video_recon.ANCHOR_GATE=1.5 的口径。残余 ~9% 反例是锚域内局部几何误差，
    # 距离/数量门不可解，留给跨帧投票。
    # 缓存锚点：只收上一帧的内点（px 只在相邻帧近似成立；跨多帧混存正是
    # 毒化根因——相机一动旧 px 全数过期，混合 RANSAC 反被旧多数派绑架）
    _cache_px: list = field(default_factory=list)
    _cache_grid: list = field(default_factory=list)
    last_rms: float = math.inf
    last_n_inlier: int = 0               # 只计本帧内点（诚实计数，不含缓存）
    last_h_src: str = ""                 # ""=未解 / "fit"=本帧独立拟合 / "carry"=缓存辅助

    def infer(self, labels_px: list[tuple[float, float, str]],
              targets_px: list[tuple[float, float]] | None = None
              ) -> tuple[list[CellRead], list[CellRead]]:
        """一帧推理。

        labels_px: [(px_x, px_y, 格名读数), ...]——v4d 的 cell_label 输出
        targets_px: 需要落格的目标像素中心（雷/箭头/障碍 bbox 中心）

        返回 (label_reads, target_reads)：
            label_reads —— cell_label 的修正读数（含 inferred 覆盖；
                          锚域外的分歧读数保留原读并标 gated=True）
            target_reads —— 目标格名（锚域内的纯几何投影；域外不落格）
        """
        px_all: list[tuple[float, float]] = []
        grid_all: list[tuple[float, float]] = []
        for (x, y, name) in labels_px:
            if len(name) >= 2 and name[0].upper() in LETTERS and name[1:].isdigit():
                gx, gy = cell_center_grid(name)
                px_all.append((x, y)); grid_all.append((gx, gy))
        n_frame = len(px_all)

        # ---- 两级拟合（门①防缓存毒化）----
        # 第一级：只用本帧读数——本帧读数是当前相机位姿的唯一新鲜证据，
        # 成功即权威。第二级（本帧 <min_anchors 或拟合失败且缓存非空）：
        # 借缓存补点拟合，但解必须过新帧共识门（本帧 ≥carry_min_fresh 个
        # 读数落在 tol 内）；不过门 = 相机已大动、旧锚点全数过期 → 弃解清缓存。
        H: np.ndarray | None = None
        h_src = ""
        if n_frame >= self.min_anchors:
            H, _, _ = ransac_homography(np.array(px_all), np.array(grid_all),
                                        tol=self.tol)
            if H is not None:
                h_src = "fit"
        if H is None and self._cache_px and n_frame >= 1:
            px_arr = np.array(px_all + self._cache_px, dtype=float)
            grid_arr = np.array(grid_all + self._cache_grid, dtype=float)
            H, _, _ = ransac_homography(px_arr, grid_arr, tol=self.tol)
            if H is not None:
                err0 = np.linalg.norm(_apply_h(H, np.array(px_all, dtype=float))
                                      - np.array(grid_all, dtype=float), axis=1)
                if int((err0 < self.tol).sum()) >= self.carry_min_fresh:
                    h_src = "carry"
                else:
                    H = None
                    if n_frame >= self.carry_min_fresh:
                        # 有足够新证据反对旧网格：相机已大动，旧锚点全数
                        # 过期 → 清缓存自愈（下帧起重新独立拟合）
                        self._cache_px, self._cache_grid = [], []
                    # 本帧读数太少=证据不足：不动缓存，留待下一帧裁决

        # 几何不可解：退化为纯 v4d（旧行为），目标落格无法给（空）
        if H is None:
            label_reads = [CellRead(name, x, y, "label")
                           for (x, y, name) in labels_px]
            self.last_rms, self.last_n_inlier, self.last_h_src = math.inf, 0, ""
            return label_reads, []

        # 本帧内点统一重算（fit/carry 两路同一口径）；缓存只收本帧内点
        px_fresh = np.array(px_all, dtype=float)
        grid_fresh = np.array(grid_all, dtype=float)
        proj = _apply_h(H, px_fresh)
        err = np.linalg.norm(proj - grid_fresh, axis=1)
        fresh_in = err < self.tol
        n_fresh_in = int(fresh_in.sum())
        rms = float(np.sqrt((err[fresh_in] ** 2).mean())) if n_fresh_in else math.inf
        self._cache_px = [tuple(px_fresh[i]) for i in range(n_frame) if fresh_in[i]]
        self._cache_grid = [tuple(grid_fresh[i]) for i in range(n_frame) if fresh_in[i]]
        if len(self._cache_px) > self.max_cache:
            self._cache_px = self._cache_px[-self.max_cache:]
            self._cache_grid = self._cache_grid[-self.max_cache:]
        self.last_rms, self.last_n_inlier, self.last_h_src = rms, n_fresh_in, h_src

        # ---- 锚域门②：纠正与落格仅限锚点支撑域内 ----
        anchors_grid = proj[fresh_in] if n_fresh_in else np.zeros((0, 2))
        correct_ok = n_fresh_in >= self.min_anchors_correct

        label_reads: list[CellRead] = []
        for i, (x, y, name) in enumerate(labels_px):
            gx, gy = proj[i]
            cell_geo, resid = grid_to_cell(gx, gy)
            if not cell_geo:
                # 落在网格外（误检/出界）——保留原读数但标记
                label_reads.append(CellRead(name, x, y, "label", label_raw=name, resid=resid))
                continue
            if cell_geo == name.upper():
                label_reads.append(CellRead(cell_geo, x, y, "label+geo", resid=resid))
                continue
            # 分歧读数：域内→几何覆盖（核心修复路径）；域外→外推不可信，
            # 保留 v4d 原读（gated 标记留给跨帧投票/人工复核看分歧）
            in_gate = correct_ok and within_anchor_gate(gx, gy, anchors_grid, self.anchor_gate)
            label_reads.append(CellRead(
                cell_geo if in_gate else name,
                x, y,
                "inferred" if in_gate else "label",
                label_raw=name if in_gate else "",
                resid=resid, gated=not in_gate))

        # 目标落格（雷/箭头/障碍）：纯几何，但同样只在锚域内给格名——
        # 锚域外的落格恰是低锚点帧外推反例（B10→C8/C10 跨帧不自洽）来源
        target_reads: list[CellRead] = []
        for (x, y) in (targets_px or []):
            gx, gy = _apply_h(H, np.array([[x, y]], dtype=float))[0]
            cell_geo, resid = grid_to_cell(gx, gy)
            if cell_geo and correct_ok and                     within_anchor_gate(gx, gy, anchors_grid, self.anchor_gate):
                target_reads.append(CellRead(cell_geo, x, y, "inferred", resid=resid))
        return label_reads, target_reads

    @property
    def grid_ok(self) -> bool:
        if self.last_h_src == "carry":
            return self.last_n_inlier >= self.carry_min_fresh and self.last_rms < self.tol
        return self.last_n_inlier >= self.min_anchors and self.last_rms < self.tol


# ---------------- 世界坐标版（消费 dzuav UDP 帧，2026-10-24） ---------------- #

def infer_world_frame(labels_world: list[tuple[float, float, str]],
                       targets_world: list[tuple[float, float]] | None = None,
                       *, tol: float = 0.35, min_anchors: int = 4,
                       anchor_gate: float = 1.5, min_anchors_correct: int = 6
                       ) -> tuple[list[CellRead], list[CellRead]]:
    """世界坐标版邻格推理（无像素、已投影场景）。

    输入：cell_label 的 (世界x, 世界y, 读数)。多数 anchors 读对时 PnP 世界
    坐标准 ⇒ (世界↔网格) 近似仿射（PnP 用错读也会全局偏——RANSAC 剔错读
    后对剩余 anchors 拟合仿射即可恢复）。
    返回同 infer()：修正读数 + 目标几何落格。无状态（无缓存），故无门①；
    门②锚域门与 infer() 同口径：分歧读数的纠正与目标落格仅限锚点支撑域内。
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
    # 门②：锚域门（同 infer() 口径）
    anchors_grid = proj_all[best_in]
    correct_ok = int(best_in.sum()) >= min_anchors_correct
    for k, i in enumerate(valid):
        x, y, name = labels_world[i]
        gx, gy = proj_all[k]
        cell_geo, resid = grid_to_cell(gx, gy)
        if not cell_geo:
            label_reads.append(CellRead(name, x, y, "label", name, resid)); continue
        if cell_geo == name.upper():
            label_reads.append(CellRead(cell_geo, x, y, "label+geo", "", resid))
            continue
        in_gate = correct_ok and within_anchor_gate(gx, gy, anchors_grid, anchor_gate)
        label_reads.append(CellRead(
            cell_geo if in_gate else name, x, y,
            "inferred" if in_gate else "label",
            name if in_gate else "", resid, gated=not in_gate))
    for (x, y) in (targets_world or []):
        g = (np.array([[x, y]]) @ best_M[:2, :2].T + best_M[:2, 2])[0]
        cell_geo, resid = grid_to_cell(*g)
        if cell_geo and correct_ok and within_anchor_gate(*g, anchors_grid, anchor_gate):
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
