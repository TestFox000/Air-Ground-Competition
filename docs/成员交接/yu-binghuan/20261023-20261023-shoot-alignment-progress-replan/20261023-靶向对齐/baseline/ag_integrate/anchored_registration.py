"""锚定配准：SLAM 局部系 → 场地系 的 2D 刚体变换求解。

设计依据（2026-10-17 方案）：
    - LiDAR SLAM 小场短会话内相对几何精确（cm 级），绝对位姿可漂移；
    - 用少量已知场地位的锚点解一个刚体变换 T(x, y, θ)，作用于全体语义物体，
      一个变换让所有检测同时获得绝对位置——不需要逐物体定位；
    - 格号锚（视觉）误读率 ~11% 是常态而非意外 → 必须 RANSAC 剔离群；
    - 结构锚（挡板/围墙，LiDAR 几何直出）可靠但稀疏，与格号锚互验。

可靠性承诺（测试锁定）：
    - 3 个非共线内点锚即可恢复变换（2 点定解、3 点起冗余、RANSAC 起效）；
    - 内点率 ≥50% 时 RANSAC 以 >99% 置信剔除非离群锚，输出不受污染；
    - 共线锚（θ 病态）显式拒绝，绝不静默输出坏变换；
    - 残差 RMS 超阈或锚点不足 → 报错 / 不可信标记，绝不给出错误落格。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from .field_frame import CellRef, cell_to_world


class RegistrationError(ValueError):
    """锚点不足 / 共线退化 / 全离群等不可解情形。"""


@dataclass(frozen=True)
class Anchor:
    """一个锚点对应：(SLAM 局部系坐标, 场地系已知坐标)。

    kind:
        "grid"     格号识别锚（视觉，可能误读，权重 1.0）
        "structure" 挡板/围墙等结构锚（LiDAR 几何，可靠，权重 2.0）
    """

    kind: str
    slam_x: float
    slam_y: float
    field_x: float
    field_y: float

    @classmethod
    def from_cell(cls, cell: CellRef | str, slam_x: float, slam_y: float,
                  kind: str = "grid") -> "Anchor":
        """按格号构造：场地侧坐标 = 格心世界坐标（field_frame 唯一定义）。"""
        if isinstance(cell, str):
            from .field_frame import LETTERS
            cell = CellRef(LETTERS.index(cell[0]), int(cell[1:]) - 1)
        fx, fy = cell_to_world(cell)
        return cls(kind, slam_x, slam_y, fx, fy)


@dataclass(frozen=True)
class Transform2D:
    """SLAM→场地 刚体变换；apply: p_field = R·p_slam + t。"""

    cos: float
    sin: float
    tx: float
    ty: float
    rms: float            # 内点集拟合残差 RMS（米）
    n_inliers: int
    n_anchors: int
    confident: bool       # False = 数学上可用但冗余不足，结果仅供参考

    def apply(self, x: float, y: float) -> tuple[float, float]:
        return (self.cos * x - self.sin * y + self.tx,
                self.sin * x + self.cos * y + self.ty)

    def apply_many(self, pts) -> list[tuple[float, float]]:
        return [self.apply(x, y) for x, y in pts]

    @property
    def theta(self) -> float:
        return math.atan2(self.sin, self.cos)


# ---------------------------------------------------------------- 拟合
def _weighted_fit(src, dst, weights) -> tuple[float, float, float, float, float]:
    """加权 Kabsch 2D 闭式解：返回 (cos, sin, tx, ty, rms)。"""
    w_sum = sum(weights)
    if w_sum <= 0:
        raise RegistrationError("anchor weights sum to zero")
    csx = sum(w * s[0] for w, s in zip(weights, src)) / w_sum
    csy = sum(w * s[1] for w, s in zip(weights, src)) / w_sum
    cfx = sum(w * d[0] for w, d in zip(weights, dst)) / w_sum
    cfy = sum(w * d[1] for w, d in zip(weights, dst)) / w_sum
    num = 0.0   # Σ (dx·dy' − dy·dx')
    den = 0.0   # Σ (dx·dx' + dy·dy')
    for w, s, d in zip(weights, src, dst):
        sx, sy = s[0] - csx, s[1] - csy
        fx, fy = d[0] - cfx, d[1] - cfy
        num += w * (sx * fy - sy * fx)
        den += w * (sx * fx + sy * fy)
    if abs(num) < 1e-12 and abs(den) < 1e-12:
        raise RegistrationError("degenerate anchors (zero spread)")
    theta = math.atan2(num, den)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    tx = cfx - (cos_t * csx - sin_t * csy)
    ty = cfy - (sin_t * csx + cos_t * csy)
    sq = 0.0
    for s, d in zip(src, dst):
        px = cos_t * s[0] - sin_t * s[1] + tx
        py = sin_t * s[0] + cos_t * s[1] + ty
        sq += (px - d[0]) ** 2 + (py - d[1]) ** 2
    rms = math.sqrt(sq / len(src)) if src else 1e9
    return cos_t, sin_t, tx, ty, rms


def _residual(cos_t, sin_t, tx, ty, a: Anchor) -> float:
    px = cos_t * a.slam_x - sin_t * a.slam_y + tx
    py = sin_t * a.slam_x + cos_t * a.slam_y + ty
    return math.hypot(px - a.field_x, py - a.field_y)


def _collinear(pts) -> bool:
    """2D 协方差小特征值占比判断共线（θ 病态）。"""
    n = len(pts)
    if n < 3:
        return False
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts) / n
    syy = sum((p[1] - my) ** 2 for p in pts) / n
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts) / n
    tr = sxx + syy
    if tr <= 1e-12:
        return True
    disc = math.sqrt(max((sxx - syy) ** 2 + 4 * sxy * sxy, 0.0))
    lam_min = (tr - disc) / 2
    lam_max = (tr + disc) / 2
    return lam_min < max(1e-6, 5e-3 * lam_max)


# ---------------------------------------------------------------- 求解
def solve_transform(anchors, *, inlier_tol: float = 0.15,
                    rms_max: float = 0.10, seed: int | None = None) -> Transform2D:
    """锚点集 → SLAM→场地变换。

    流程：≥3 锚走 RANSAC（2 点最小集采样 + 自适应迭代），
    内点集加权重拟合（结构锚权重 2.0）；共线/不足/全离群显式报错。

    inlier_tol: 内点残差容限（米），默认 0.15（格号落格 cm 级 + SLAM 相对误差）。
    rms_max:    confident=True 的 RMS 上限（米）。
    """
    anchors = list(anchors)
    if len(anchors) < 2:
        raise RegistrationError(f"need >=2 anchors, got {len(anchors)}")
    for a in anchors:
        if not all(map(math.isfinite, (a.slam_x, a.slam_y, a.field_x, a.field_y))):
            raise RegistrationError("anchor contains NaN/Inf")

    if len(anchors) == 2:
        a, b = anchors
        if math.hypot(a.slam_x - b.slam_x, a.slam_y - b.slam_y) < 0.15:
            raise RegistrationError("two anchors too close (theta unresolvable)")
        src = [(a.slam_x, a.slam_y), (b.slam_x, b.slam_y)]
        dst = [(a.field_x, a.field_y), (b.field_x, b.field_y)]
        w = [2.0 if x.kind == "structure" else 1.0 for x in anchors]
        cos_t, sin_t, tx, ty, rms = _weighted_fit(src, dst, w)
        return Transform2D(cos_t, sin_t, tx, ty, rms, 2, 2, confident=False)

    # ---------- RANSAC ----------
    rng = random.Random(seed)
    n = len(anchors)
    best_inliers: list[int] = []
    n_iter = min(60, math.ceil(math.log(1e-3) / math.log(1 - 0.55 ** 2)))
    done = 0
    while done < n_iter:
        i, j = rng.sample(range(n), 2)
        a, b = anchors[i], anchors[j]
        if math.hypot(a.slam_x - b.slam_x, a.slam_y - b.slam_y) < 0.3:
            done += 1
            continue
        try:
            cos_t, sin_t, tx, ty, _ = _weighted_fit(
                [(a.slam_x, a.slam_y), (b.slam_x, b.slam_y)],
                [(a.field_x, a.field_y), (b.field_x, b.field_y)], [1.0, 1.0])
        except RegistrationError:
            done += 1
            continue
        inliers = [k for k, c in enumerate(anchors)
                   if _residual(cos_t, sin_t, tx, ty, c) <= inlier_tol]
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            ratio = len(inliers) / n
            if ratio > 0.9:   # 已足够好，提前收敛
                n_iter = done + 1
        # 自适应：找到更高内点率则缩减剩余迭代
        if len(best_inliers) >= 3:
            ratio = len(best_inliers) / n
            need = math.ceil(math.log(1e-3) / math.log(1 - ratio ** 2)) if ratio < 1 else 1
            n_iter = min(n_iter, max(need, done + 1))
        done += 1

    if len(best_inliers) < 2 or (n >= 4 and len(best_inliers) < 3):
        # 2 点最小集对自身恒零残差，必须用第三点互证才能排除"随机对碰"
        raise RegistrationError(
            f"anchors mutually inconsistent (inliers {len(best_inliers)}/{n}); "
            "格号大面积误读或结构锚错误")
    inl = [anchors[k] for k in best_inliers]
    if _collinear([(a.slam_x, a.slam_y) for a in inl]):
        raise RegistrationError(
            f"inlier anchors collinear (n={len(inl)}): theta ill-conditioned; "
            "需要面内散开的锚点（扫描路线覆盖 2D）")

    src = [(a.slam_x, a.slam_y) for a in inl]
    dst = [(a.field_x, a.field_y) for a in inl]
    w = [2.0 if a.kind == "structure" else 1.0 for a in inl]
    cos_t, sin_t, tx, ty, rms = _weighted_fit(src, dst, w)
    confident = len(inl) >= 3 and rms <= rms_max
    return Transform2D(cos_t, sin_t, tx, ty, rms, len(inl), n, confident)
