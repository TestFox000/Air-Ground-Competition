# -*- coding: utf-8 -*-
r"""网格注册：四角点 → 单应矩阵 → 任意像素 → 格名 A1..N11（格边 0.35 m）。

## 坐标语义（与车侧 **逐字一致**，这是本模块唯一的坐标系）

真源：``dzrobot_v2/src/dzbrain/dzbrain/core/geometry.py:6-16,69-81``

```
x = (10.5 - row_idx) * 0.35      row 11 → 0.175, row 1 → 3.675
y = (13.5 - col_idx) * 0.35      col N  → 0.175, col A → 4.725
row_idx = 行号-1, col_idx = 列字母索引(A=0 … N=13)
```

所以本模块的"场地米制系"就是车的地图系：

* 原点 = **N11 格外角**（不是格心）；+x 指向**行号减小**方向（row 11 → row 1，即从下半场
  指向上半场）；+y 指向**列号减小**方向（N → A，与 ``config/map.yaml:18`` 的"车头朝 +y
  = 图左"一致）。
* 场地矩形：x ∈ [0, 3.85]（11 行 × 0.35）、y ∈ [0, 4.90]（14 列 × 0.35）。
* 四个"外角"标签与米制坐标：
  ``N11``=(0, 0)、``N1``=(3.85, 0)、``A1``=(3.85, 4.90)、``A11``=(0, 4.90)。
* 半场：``x > 6×0.35 = 2.10`` 为 **upper**（上半场，行 1~6），否则 **lower**
  （下半场，行 7~11）——分界即 ``map.yaml:46-47`` 的挡板行 6/7。

⚠️ ``geometry.py:39`` 的注释写 ``HEADING_X = 0 → 朝 X+（行号增大方向）``，与
``geometry.py:72`` 的公式（x 随 row_idx **减小**而增大）矛盾。**以公式为准**：
x=0 朝 +x 即朝行号减小方向。本模块对此只做记录，不改车侧任何文件。

## 角点顺序 / 手性（★ 容易踩，实测过）

场地米制系是**右手系（z 朝上）**：x̂ × ŷ 指向地面外（远处观者方向），依据是
``map.yaml:18`` 的"车头朝 +y = 图左"与 ``geometry.py:72`` 的 x 公式。而图像数组
``(u 向右, v 向下)`` 是**左手系**，所以任何**正常**的俯视照片都有

    signed_area(像素四角, 顺序 N11→N1→A1→A11)  <  0

（实测：``dzrobot_v2/_tmp_session/t28_newmap/map_src.png`` 上该值 ≈ −6.3×10⁵，
米制侧 = +18.87）。若两者**同号**，只可能是图像被镜像、或角点标签张冠李戴 ⇒
格名会系统性错位，``register()`` 默认直接报错（真要放行给 ``expect_flip=False``）。

## 单应矩阵

``meters → pixels`` 的 3×3 单应 ``H`` 由 DLT 最小二乘拟合（Hartley 归一化，
支持 4 点及以上：多给的对应点会一起参与最小二乘，用于把标定误差摊薄）。
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Iterable, Mapping, Sequence

import numpy as np

from .colors import as_rgb_array

__all__ = [
    "COL_LETTERS",
    "GridSpec",
    "RegistrationError",
    "Homography",
    "fit_homography",
    "ArenaRegistration",
    "register",
    "register_from_corners",
    "register_from_named_corners",
    "CellHit",
    "CORNER_ORDER",
    "arena_corner_meters",
]

COL_LETTERS = "ABCDEFGHIJKLMN"
#: 四角点默认顺序（**场地米制系**）——顺序错位会被 ``allow_mirror=False`` 拦下
CORNER_ORDER: tuple[str, str, str, str] = ("N11", "N1", "A1", "A11")


class RegistrationError(ValueError):
    """角点退化 / 顺序镜像 / 对应点不足。"""


# --------------------------------------------------------------------------- #
#  网格定义
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class GridSpec:
    """场地网格：14 列 A~N × 11 行 1~11，格边 0.35 m。"""

    cols: int = 14
    rows: int = 11
    cell_m: float = 0.35
    col_letters: str = COL_LETTERS
    barrier_row_hi: int = 6      # map.yaml:46
    barrier_row_lo: int = 7      # map.yaml:47

    # ---------------- 基本换算 ----------------
    @property
    def width_m(self) -> float:
        """列方向总长（y 轴），14 × 0.35 = 4.90 m。"""
        return self.cols * self.cell_m

    @property
    def height_m(self) -> float:
        """行方向总长（x 轴），11 × 0.35 = 3.85 m。"""
        return self.rows * self.cell_m

    @property
    def barrier_x_m(self) -> float:
        """上半场 / 下半场分界（米制 x）。``map.yaml`` 挡板在第 6/7 行之间。"""
        return self.barrier_row_hi * self.cell_m

    def check(self) -> None:
        if self.cols != len(self.col_letters):
            raise RegistrationError(
                f"cols={self.cols} 与列字母表长度 {len(self.col_letters)} 不符")
        if not (1 <= self.barrier_row_hi < self.barrier_row_lo <= self.rows):
            raise RegistrationError(
                f"挡板行区间非法：{self.barrier_row_hi}/{self.barrier_row_lo}（rows={self.rows}）")

    # ---------------- 格名 ↔ 索引 ↔ 米制 ----------------
    def index_to_block(self, row_idx: int, col_idx: int) -> str:
        """(row_idx, col_idx) 0-based → 'K9'（越界**夹紧**，与车侧 index_to_block 同）。"""
        r = max(0, min(self.rows - 1, int(row_idx)))
        c = max(0, min(self.cols - 1, int(col_idx)))
        return f"{self.col_letters[c]}{r + 1}"

    def block_to_index(self, block: str) -> tuple[int, int]:
        """'K9' → (row_idx, col_idx)；非法抛 :class:`RegistrationError`。"""
        if not isinstance(block, str) or len(block) < 2:
            raise RegistrationError(f"非法格名: {block!r}")
        letter = block[0].upper()
        if letter not in self.col_letters:
            raise RegistrationError(f"非法列字母: {block!r}（应为 A~{self.col_letters[-1]}）")
        try:
            number = int(block[1:])
        except ValueError as exc:
            raise RegistrationError(f"非法行号: {block!r}") from exc
        if not 1 <= number <= self.rows:
            raise RegistrationError(f"行号越界: {block!r}（应为 1~{self.rows}）")
        return number - 1, self.col_letters.index(letter)

    def block_to_meters(self, block: str) -> tuple[float, float]:
        """格名 → 格中心米制 (x, y)。与 ``geometry.block_to_meters`` 逐字同式。"""
        r, c = self.block_to_index(block)
        return (self.rows - 0.5 - r) * self.cell_m, (self.cols - 0.5 - c) * self.cell_m

    def meters_to_index(self, x: float, y: float) -> tuple[int, int]:
        """米制 → 最近格的 (row_idx, col_idx)（四舍五入，与车侧 meters_to_block 同）。"""
        r = int(round(self.rows - 0.5 - float(x) / self.cell_m))
        c = int(round(self.cols - 0.5 - float(y) / self.cell_m))
        return r, c

    def meters_to_block(self, x: float, y: float) -> str:
        return self.index_to_block(*self.meters_to_index(x, y))

    def in_arena(self, x: float, y: float, margin: float = 0.0) -> bool:
        """米制点是否落在场地矩形内（可外扩 margin，负值为内缩）。"""
        return (-margin <= x <= self.height_m + margin
                and -margin <= y <= self.width_m + margin)

    # ---------------- 半场 ----------------
    def half_of_meters(self, x: float, y: float) -> str:
        """米制 → 'upper'（上半场，行 1~6）/ 'lower'（行 7~11）。"""
        return "upper" if x > self.barrier_x_m else "lower"

    def half_of_block(self, block: str) -> str:
        x, _ = self.block_to_meters(block)
        return self.half_of_meters(x, 0.0)

    def blocks_of_half(self, half: str) -> list[str]:
        half = half.strip().lower()
        if half not in ("upper", "lower"):
            raise RegistrationError(f"half 只能是 upper/lower，收到 {half!r}")
        return [f"{c}{r}"
                for r in range(1, self.rows + 1)
                for c in self.col_letters
                if ("upper" if r <= self.barrier_row_hi else "lower") == half]

    def all_blocks(self) -> list[str]:
        return [f"{c}{r}" for r in range(1, self.rows + 1) for c in self.col_letters]


def arena_corner_meters(tag: str, spec: GridSpec | None = None) -> tuple[float, float]:
    """四角点标签 → 场地米制坐标（"该格的**外角**"）。

    ``N11``=(0,0)、``N1``=(3.85,0)、``A1``=(3.85,4.90)、``A11``=(0,4.90)。
    """
    spec = spec or GridSpec()
    table = {
        "N11": (0.0, 0.0),
        "N1": (spec.height_m, 0.0),
        "A1": (spec.height_m, spec.width_m),
        "A11": (0.0, spec.width_m),
    }
    key = str(tag).strip().upper()
    if key not in table:
        raise RegistrationError(f"角点标签只能是 {sorted(table)}，收到 {tag!r}")
    return table[key]


# --------------------------------------------------------------------------- #
#  单应矩阵
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Homography:
    """3×3 单应。``H`` 把 ``src`` 映到 ``dst``（本包一律 meters → pixels）。"""

    H: np.ndarray

    def apply(self, pts: np.ndarray) -> np.ndarray:
        """``(N,2)`` → ``(N,2)``（齐次除法，自动处理批量）。"""
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        ph = np.hstack([p, np.ones((p.shape[0], 1))])
        q = ph @ self.H.T
        w = q[:, 2:3]
        if np.any(np.abs(w) < 1e-12):
            raise RegistrationError("单应变换出现 w≈0（点落在无穷远处）")
        return q[:, :2] / w

    def apply_one(self, pt: Sequence[float]) -> tuple[float, float]:
        out = self.apply(np.asarray(pt, dtype=np.float64).reshape(1, 2))[0]
        return float(out[0]), float(out[1])

    def inverse(self) -> "Homography":
        return Homography(np.linalg.inv(self.H))

    def condition(self) -> float:
        """``‖H‖·‖H⁻¹‖``（列归一后），越接近 1 越稳。"""
        h = self.H / np.maximum(np.linalg.norm(self.H, axis=0, keepdims=True), 1e-12)
        return float(np.linalg.norm(h) * np.linalg.norm(np.linalg.inv(h)))


def _normalize(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Hartley 归一化：质心到原点 + 平均距离 √2。返回 (归一化点, 3×3 变换 T)。"""
    c = pts.mean(axis=0)
    shifted = pts - c
    d = np.sqrt((shifted ** 2).sum(axis=1)).mean()
    s = np.sqrt(2.0) / d if d > 1e-12 else 1.0
    T = np.array([[s, 0.0, -s * c[0]],
                  [0.0, s, -s * c[1]],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    return (shifted * s), T


def fit_homography(src_xy, dst_uv) -> tuple[Homography, float]:
    """DLT 拟合 ``src → dst``（≥4 点，最小二乘）。返回 ``(Homography, 残差_px)``。

    残差按 ``dst`` 单位（本包 dst=pixel，所以是像素）。
    """
    src = np.asarray(src_xy, dtype=np.float64).reshape(-1, 2)
    dst = np.asarray(dst_uv, dtype=np.float64).reshape(-1, 2)
    if src.shape != dst.shape:
        raise RegistrationError(f"对应点数量不一致：{src.shape} vs {dst.shape}")
    if src.shape[0] < 4:
        raise RegistrationError(f"至少需要 4 组对应点，收到 {src.shape[0]}")
    sn, T1 = _normalize(src)
    dn, T2 = _normalize(dst)
    A = np.zeros((2 * sn.shape[0], 9), dtype=np.float64)
    for i, ((x, y), (u, v)) in enumerate(zip(sn, dn)):
        A[2 * i] = [-x, -y, -1.0, 0.0, 0.0, 0.0, u * x, u * y, u]
        A[2 * i + 1] = [0.0, 0.0, 0.0, -x, -y, -1.0, v * x, v * y, v]
    _, _, vt = np.linalg.svd(A)
    h = vt[-1].reshape(3, 3)
    if abs(h[2, 2]) < 1e-12:
        raise RegistrationError("DLT 退化（h22≈0）")
    Hn = h / h[2, 2]
    H = np.linalg.inv(T2) @ Hn @ T1
    H = H / H[2, 2]
    if not np.all(np.isfinite(H)):
        raise RegistrationError("单应矩阵含非有限值（对应点退化）")
    hom = Homography(H)
    proj = hom.apply(src)
    res = float(np.sqrt(((proj - dst) ** 2).sum(axis=1)).max())
    return hom, res


def _signed_area(pts: np.ndarray) -> float:
    p = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    x, y = p[:, 0], p[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


# --------------------------------------------------------------------------- #
#  注册结果
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class CellHit:
    """一个点落格的结果。

    ``fx`` / ``fy`` 是**格内归一化坐标**：0 在靠近 ``+x``/``+y`` 大端（行号小/列号小）的
    格边，1 在另一端，格心 = 0.5。越界时允许落在 [0,1) 之外，便于判越界。
    """

    block: str
    row_idx: int
    col_idx: int
    x_m: float
    y_m: float
    fx: float
    fy: float
    in_arena: bool
    half: str
    cell_m: float = 0.35

    @property
    def offset_from_center_m(self) -> tuple[float, float]:
        """相对格心的偏置（米），方向同场地 +x/+y。"""
        return (self.fx - 0.5) * self.cell_m, (self.fy - 0.5) * self.cell_m


@dataclass(frozen=True)
class ArenaRegistration:
    """一次标定：``spec`` + ``H: meters → pixels`` + 质量指标 + 参与拟合的点。"""

    spec: GridSpec
    H_meters_to_px: Homography
    residual_px: float
    pairs: tuple[tuple[tuple[float, float], tuple[float, float]], ...] = ()
    notes: tuple[str, ...] = ()

    # ---------------- 变换 ----------------
    def meters_to_px(self, pts) -> np.ndarray:
        return self.H_meters_to_px.apply(np.asarray(pts, dtype=np.float64))

    def px_to_meters(self, pts) -> np.ndarray:
        return self.H_meters_to_px.inverse().apply(np.asarray(pts, dtype=np.float64))

    def point_to_meters(self, u: float, v: float) -> tuple[float, float]:
        out = self.px_to_meters([[u, v]])[0]
        return float(out[0]), float(out[1])

    # ---------------- 落格 ----------------
    def meters_to_hit(self, x: float, y: float, margin_m: float = 0.0) -> CellHit:
        """米制点 → 落格。``margin_m>0`` 表示要求点离场地边界至少这么远才算 in_arena。"""
        spec = self.spec
        r, c = spec.meters_to_index(x, y)
        block = spec.index_to_block(r, c)
        # 连续"格坐标"：格心处等于浮点索引，沿 +x / +y 递减
        rr = spec.rows - 0.5 - float(x) / spec.cell_m
        cc = spec.cols - 0.5 - float(y) / spec.cell_m
        return CellHit(
            block=block, row_idx=r, col_idx=c, x_m=float(x), y_m=float(y),
            fx=float(0.5 - (rr - round(rr))), fy=float(0.5 - (cc - round(cc))),
            in_arena=spec.in_arena(x, y, margin=-margin_m),
            half=spec.half_of_meters(x, y), cell_m=spec.cell_m,
        )

    def px_to_hit(self, u: float, v: float, margin_m: float = 0.0) -> CellHit:
        x, y = self.point_to_meters(u, v)
        return self.meters_to_hit(x, y, margin_m=margin_m)

    def px_to_block(self, u: float, v: float) -> str:
        return self.px_to_hit(u, v).block

    # ---------------- 格几何 ----------------
    def cell_center_px(self, block: str) -> tuple[float, float]:
        return self.H_meters_to_px.apply_one(self.spec.block_to_meters(block))

    def cell_polygon_px(self, block: str) -> np.ndarray:
        """格四角像素坐标，**按环序**给出（不自交，可直接喂 ``ImageDraw.polygon``）。"""
        spec = self.spec
        r, c = spec.block_to_index(block)
        pts = []
        for dr, dc in ((-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)):
            x = (spec.rows - 0.5 - (r + dr)) * spec.cell_m
            y = (spec.cols - 0.5 - (c + dc)) * spec.cell_m
            pts.append((x, y))
        return self.meters_to_px(pts)

    def cell_area_px(self, block: str) -> float:
        """该格在图像中的面积（px²）—— 透视会改变它，用于面积类判据的归一化。"""
        return abs(_signed_area(self.cell_polygon_px(block)))

    def cell_size_px(self, block: str) -> tuple[float, float]:
        """该格在图像中的等效边长 ``(沿 x, 沿 y)``（px），用于比例类判据。"""
        spec = self.spec
        r, c = spec.block_to_index(block)
        a = self.meters_to_px([((spec.rows - 0.5 - r + 0.5) * spec.cell_m,
                                (spec.cols - 0.5 - c) * spec.cell_m),
                               ((spec.rows - 0.5 - r - 0.5) * spec.cell_m,
                                (spec.cols - 0.5 - c) * spec.cell_m)])
        b = self.meters_to_px([((spec.rows - 0.5 - r) * spec.cell_m,
                                (spec.cols - 0.5 - c + 0.5) * spec.cell_m),
                               ((spec.rows - 0.5 - r) * spec.cell_m,
                                (spec.cols - 0.5 - c - 0.5) * spec.cell_m)])
        return float(np.linalg.norm(a[0] - a[1])), float(np.linalg.norm(b[0] - b[1]))

    # ---------------- 自检 ----------------
    def self_check(self, tol_px: float = 1.0) -> dict:
        """整数格往返自检：格心 → 像素 → 米 → 格名 是否闭合。"""
        bad = []
        for block in self.spec.all_blocks():
            u, v = self.cell_center_px(block)
            back = self.px_to_block(u, v)
            if back != block:
                bad.append((block, back, round(u, 2), round(v, 2)))
        return {"n_cells": self.spec.rows * self.spec.cols, "mismatch": bad,
                "residual_px": self.residual_px, "ok": not bad and self.residual_px <= tol_px}

    def diagnostics(self) -> dict:
        return {
            "residual_px": round(self.residual_px, 4),
            "condition": round(self.H_meters_to_px.condition(), 3),
            "pairs": len(self.pairs),
            "notes": list(self.notes),
            "self_check": self.self_check()["ok"],
        }


# --------------------------------------------------------------------------- #
#  构造入口
# --------------------------------------------------------------------------- #
def register(pairs: Iterable[tuple[Sequence[float], Sequence[float]]],
             spec: GridSpec | None = None, *,
             expect_flip: bool = True,
             notes: Sequence[str] = ()) -> ArenaRegistration:
    """按任意组 ``(米制点, 像素点)`` 拟合注册；角点/格心都可以混着给。

    ``expect_flip``：像素坐标是**图像数组**约定（v 向下）时为 True —— 此时像素与
    米制两侧的有向面积**必须反号**（见模块 docstring「手性」）。若你的像素坐标是
    v 向上的绘图约定，置 False。
    """
    spec = spec or GridSpec()
    spec.check()
    pl = [(tuple(float(a) for a in p), tuple(float(b) for b in q)) for p, q in pairs]
    if len(pl) < 4:
        raise RegistrationError(f"至少 4 组对应点，收到 {len(pl)}")
    src = np.asarray([p for p, _ in pl], dtype=np.float64)
    dst = np.asarray([q for _, q in pl], dtype=np.float64)
    hom, res = fit_homography(src, dst)
    note_list = list(notes)
    # 手性检查：v 向下的图像约定下，两侧有向面积必须反号
    s_src, s_dst = _signed_area(src), _signed_area(dst)
    same_sign = (s_src * s_dst > 0)
    if same_sign == expect_flip:
        msg = (f"角点/对应点手性异常（米制有向面积 {s_src:.2f} / 像素 {s_dst:.2f}，"
               f"expect_flip={expect_flip}）⇒ 图像被镜像或角点标签错位，格名会系统性错位")
        raise RegistrationError(msg + "；确认无误请显式 expect_flip=取反")
    if s_dst == 0.0 or s_src == 0.0:
        raise RegistrationError("四角点共线（有向面积 0）")
    if res > 2.0:
        note_list.append(f"⚠️ 拟合残差 {res:.2f} px 偏大（角点定位不准？）")
    return ArenaRegistration(spec=spec, H_meters_to_px=hom, residual_px=res,
                             pairs=tuple(pl), notes=tuple(note_list))


#: 兼容别名（强调返回类型）
ArenaRegistration_fit = register


def register_from_corners(corners_px: Sequence[Sequence[float]],
                          spec: GridSpec | None = None, *,
                          order: Sequence[str] = CORNER_ORDER,
                          expect_flip: bool = True) -> ArenaRegistration:
    """四角点注册。``corners_px[i]`` 对应 ``order[i]`` 的外角（图像像素）。

    默认顺序 ``N11, N1, A1, A11`` —— 米制坐标 (0,0)、(3.85,0)、(3.85,4.90)、(0,4.90)。
    """
    spec = spec or GridSpec()
    if len(order) != len(corners_px):
        raise RegistrationError(f"order({len(order)}) 与角点数({len(corners_px)}) 不符")
    pairs = [(arena_corner_meters(tag, spec), tuple(map(float, px)))
             for tag, px in zip(order, corners_px)]
    return register(pairs, spec, expect_flip=expect_flip,
                    notes=(f"角点顺序 {','.join(order)}",))


def register_from_named_corners(px_of: Mapping[str, Sequence[float]],
                                spec: GridSpec | None = None, *,
                                order: Sequence[str] = CORNER_ORDER,
                                expect_flip: bool = True) -> ArenaRegistration:
    """``{"N11": (u, v), "N1": ..., "A1": ..., "A11": ...}`` —— 标签化，免记顺序。"""
    spec = spec or GridSpec()
    corners = []
    used = []
    for tag in order:
        key = tag if tag in px_of else tag.upper()
        if key not in px_of:
            raise RegistrationError(f"缺少角点 {tag!r}；已给 {sorted(px_of)}")
        corners.append(px_of[key])
        used.append(tag)
    return register_from_corners(corners, spec, order=used, expect_flip=expect_flip)


# --------------------------------------------------------------------------- #
#  自动标定：从"官方图纸"自己找格网（t59）
# --------------------------------------------------------------------------- #
def _runs_1d(mask: np.ndarray) -> list[tuple[int, int]]:
    out: list[tuple[int, int]] = []
    cur: int | None = None
    for i, v in enumerate(mask):
        if v and cur is None:
            cur = i
        elif not v and cur is not None:
            out.append((cur, i - 1))
            cur = None
    if cur is not None:
        out.append((cur, len(mask) - 1))
    return out


def register_from_arena_frame(rgb, spec: GridSpec | None = None, *,
                              frame_rgb: tuple[int, int, int] = (48, 80, 144),
                              frame_tol: int = 70,
                              grid_gray: int = 208, gray_tol: int = 16,
                              search_px: float = 2.0) -> ArenaRegistration:
    """**无需手点角点**的自动标定：官方图纸/截图类输入（外框 + 灰格线）。

    做法（与 t39 官方图识读同源，两步都留证据）：

    1. **外框定 arena 外接矩形**：按颜色找"外框带"（默认深蓝 ``(48,80,144)``），
       取左/右两条竖直带的内边与上/下两条水平带的内边 ⇒ 场地矩形
       ``[x0,y0,x1,y1]``（同时给出 ``frame_inner`` 证据）。
    2. **灰格线最小二乘精修**：在候选格距/相位上搜索，令 ``x0 + i·s``、
       ``y0 + j·s`` 正好落在灰格线（默认灰度 208±16）上，覆盖率最高的
       ``(x0, sx)``/``(y0, sy)`` 胜出 —— 单靠外框只能到 1~2 px，精修后到亚像素。
    3. 用步 2 的结果算 14×11 的四个外角像素 → :func:`register_from_corners`。

    ★ 适用面：**规则图纸/示意图**（有外框与格线）。实拍照片没有外框，请用
      ``register_from_corners`` 手点或 ``register(pairs)`` 多点最小二乘。
    """
    arr = as_rgb_array(rgb)
    r = arr[..., 0].astype(np.int16)
    g = arr[..., 1].astype(np.int16)
    b = arr[..., 2].astype(np.int16)
    fr, fg, fb = frame_rgb
    frame = ((np.abs(r - fr) < frame_tol) & (np.abs(g - fg) < frame_tol)
             & (np.abs(b - fb) < frame_tol))
    if frame.sum() < 200:
        raise RegistrationError("找不到外框带（frame_rgb/ frame_tol 需要按图调整）")
    H, W = frame.shape
    left_in, right_in, top_in, bot_in = [], [], [], []
    for y in range(H):
        xs = np.flatnonzero(frame[y])
        if xs.size == 0:
            continue
        k = 0
        while k + 1 < xs.size and xs[k + 1] - xs[k] <= 2:
            k += 1
        left_in.append(int(xs[k]))
        k2 = xs.size - 1
        while k2 - 1 >= 0 and xs[k2] - xs[k2 - 1] <= 2:
            k2 -= 1
        right_in.append(int(xs[k2]))
    for x in range(W):
        ys = np.flatnonzero(frame[:, x])
        if ys.size == 0:
            continue
        k = 0
        while k + 1 < ys.size and ys[k + 1] - ys[k] <= 2:
            k += 1
        top_in.append(int(ys[k]))
        k2 = ys.size - 1
        while k2 - 1 >= 0 and ys[k2] - ys[k2 - 1] <= 2:
            k2 -= 1
        bot_in.append(int(ys[k2]))
    if not (left_in and right_in and top_in and bot_in):
        raise RegistrationError("外框带不完整（只有单边）⇒ 该图不是『外框+格线』型")
    x_left = int(np.median(left_in)) + 1
    x_right = int(np.median(right_in)) - 1
    y_top = int(np.median(top_in)) + 1
    y_bot = int(np.median(bot_in)) - 1
    spec = spec or GridSpec()
    spec.check()
    gray = ((np.abs(r - grid_gray) < gray_tol) & (np.abs(g - grid_gray) < gray_tol)
            & (np.abs(b - grid_gray) < gray_tol))
    col_proj = gray.sum(axis=0)
    row_proj = gray.sum(axis=1)

    def _refine(proj: np.ndarray, a0: float, s0: float, n: int) -> tuple[float, float, float]:
        best: tuple[float, float, float] | None = None
        for s in np.arange(s0 - search_px, s0 + search_px, 0.002):
            for a in np.arange(a0 - search_px, a0 + search_px, 0.05):
                pos = a + np.arange(0, n + 1) * s
                if pos[0] < 0 or pos[-1] > len(proj) - 1:
                    continue
                vals = [max(proj[max(0, int(round(p)) - 1)],
                            proj[int(round(p))],
                            proj[min(len(proj) - 1, int(round(p)) + 1)]) for p in pos]
                score = float(np.mean(vals))
                if best is None or score > best[0]:
                    best = (score, float(a), float(s))
        if best is None:
            raise RegistrationError("灰格线精修失败（图像太小或格线颜色不同）")
        return best[1], best[2], best[0]

    x0, sx, score_x = _refine(col_proj, float(x_left), (x_right - x_left + 1) / spec.cols,
                              spec.cols)
    y0, sy, score_y = _refine(row_proj, float(y_top), (y_bot - y_top + 1) / spec.rows,
                              spec.rows)
    corners = {"N11": (x0 + spec.cols * sx, y0 + spec.rows * sy),
               "N1": (x0 + spec.cols * sx, y0),
               "A1": (x0, y0),
               "A11": (x0, y0 + spec.rows * sy)}
    reg = register_from_named_corners(corners, spec)
    return replace(reg, notes=tuple(reg.notes) + (
        f"自动标定(外框+灰线)：frame_inner=[{x_left},{y_top},{x_right},{y_bot}] "
        f"格距 sx={sx:.4f} sy={sy:.4f} px（格线覆盖率 {score_x:.1f}/{score_y:.1f}）",))
