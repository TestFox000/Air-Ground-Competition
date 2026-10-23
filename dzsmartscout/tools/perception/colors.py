# -*- coding: utf-8 -*-
r"""颜色分类 + 形态学 + 连通域 —— **纯 numpy**（本机无 cv2 / scipy / skimage）。

## 为什么不是"HSV 阈值一把梭"

1. **调色板是数据，不是代码。** 场地（实物）颜色只能现场标定：规则文档只给了
   "彩色地雷图片 / 彩色带盖水桶 / 绿色箭头贴纸"，没有给出可用的 RGB。所以阈值全部
   放在 :class:`Palette` 里（可 ``to_json`` / ``from_json`` 往返），分类器本身不认识语义。
2. **provisional 规则默认关闭。** 没有实测证据的规则（例如"红=地雷"）默认不生效，
   必须显式 :meth:`Palette.enable_provisional` 或换成标定产物，避免"猜错的阈值
   静默给出错误结论"。实测证据见 ``README.md`` §颜色证据。
3. **形态学 / 连通域自研。** 环境里没有 cv2/scipy，所以自己实现：膨胀/腐蚀用移位
   累加；连通域是 run-based union-find（8 邻接），并顺带算出二阶矩，
   供箭头方向 / 车体位姿估计使用。

所有函数对入参只读，返回新数组；掩膜一律 ``dtype=bool``、形状 ``(H, W)``。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, replace

import numpy as np

__all__ = [
    "ImageShapeError",
    "as_rgb_array",
    "rgb_to_hsv",
    "hsv_to_rgb",
    "ColorRule",
    "Palette",
    "MAP_IMAGE",
    "FIELD_DEFAULT",
    "PALETTES",
    "blob_masks",
    "classify",
    "connected_components",
    "dilate",
    "erode",
    "open_",
    "close_",
    "Blob",
    "sample_patch_stats",
]


class ImageShapeError(ValueError):
    """入参不是 HxWx3 的 RGB 图像。"""


# --------------------------------------------------------------------------- #
#  基础：数组规整 / 色彩空间
# --------------------------------------------------------------------------- #
def as_rgb_array(image) -> np.ndarray:
    """把 Pillow Image / list / ndarray 规整成 ``(H, W, 3) uint8``。

    支持：PIL.Image（自动 ``convert("RGB")``）、``(H, W, 3)`` 的 uint8/float、
    ``(H, W, 4)`` 的 RGBA（丢掉 alpha）。**不接受**已经是 HSV 的数组（请自行转换）。
    """
    if hasattr(image, "convert") and hasattr(image, "size"):      # PIL
        image = np.asarray(image.convert("RGB"))
    arr = np.asarray(image)
    if arr.ndim == 3 and arr.shape[2] == 4:
        arr = arr[:, :, :3]
    if arr.ndim != 3 or arr.shape[2] != 3:
        raise ImageShapeError(f"需要 (H, W, 3) 的 RGB 图像，收到 shape={arr.shape}")
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(arr)


def rgb_to_hsv(rgb) -> np.ndarray:
    """``(H,W,3) uint8`` → ``(H,W,3) float64``，H∈[0,360)，S,V∈[0,1]。

    灰度像素（max==min）H=0、S=0 —— 与 OpenCV 的 ``COLOR_RGB2HSV`` 色相量纲不同
    （OpenCV 用 H∈[0,179]），本包统一用**角度制 H**，写阈值时不要混用。
    """
    arr = as_rgb_array(rgb).astype(np.float64) / 255.0
    r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
    mx = np.max(arr, axis=-1)
    mn = np.min(arr, axis=-1)
    d = mx - mn
    h = np.zeros_like(mx)
    safe = np.where(d > 1e-12, d, 1.0)
    m = (d > 1e-12) & (mx == r)
    h[m] = (60.0 * ((g - b)[m] / safe[m])) % 360.0
    m = (d > 1e-12) & (mx == g)
    h[m] = 60.0 * ((b - r)[m] / safe[m]) + 120.0
    m = (d > 1e-12) & (mx == b)
    h[m] = 60.0 * ((r - g)[m] / safe[m]) + 240.0
    s = np.where(mx > 1e-12, d / np.where(mx > 1e-12, mx, 1.0), 0.0)
    return np.stack([h, s, mx], axis=-1)


def hsv_to_rgb(hsv: np.ndarray) -> np.ndarray:
    """``(H,W,3)``（H 角度制）→ ``(H,W,3) uint8``。用于合成测试图。"""
    hsv = np.asarray(hsv, dtype=np.float64)
    h, s, v = np.moveaxis(hsv, -1, 0)
    c = v * s
    hp = (h % 360.0) / 60.0
    x = c * (1.0 - np.abs(hp % 2.0 - 1.0))
    z = np.zeros_like(c)
    r = np.select([hp < 1, hp < 2, hp < 3, hp < 4, hp < 5, True], [c, x, z, z, x, c])
    g = np.select([hp < 1, hp < 2, hp < 3, hp < 4, hp < 5, True], [x, c, c, x, z, z])
    b = np.select([hp < 1, hp < 2, hp < 3, hp < 4, hp < 5, True], [z, z, x, c, c, x])
    m = v - c
    out = np.stack([r + m, g + m, b + m], axis=-1)
    return np.clip(np.round(out * 255.0), 0, 255).astype(np.uint8)


# --------------------------------------------------------------------------- #
#  颜色规则 / 调色板
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class ColorRule:
    """一条阈值规则。HSV 与 RGB 二选一（同时给则取交集）。

    * ``hsv_lo`` / ``hsv_hi``：``(h_deg, s, v)``，H 角度制。``h_lo > h_hi`` 表示
      **跨 0° 的红色缠绕**（例如 ``(345, 0.35, 0.30) → (18, 1, 1)``）。
    * ``rgb_lo`` / ``rgb_hi``：``(r, g, b)`` 0~255，逐通道闭区间。
    * ``provisional``：True = **没有实测证据**的占位阈值（默认不参与分类）。
    """

    cls: str
    hsv_lo: tuple[float, float, float] | None = None
    hsv_hi: tuple[float, float, float] | None = None
    rgb_lo: tuple[int, int, int] | None = None
    rgb_hi: tuple[int, int, int] | None = None
    provisional: bool = False
    note: str = ""

    def mask(self, rgb, hsv=None) -> np.ndarray:
        out = np.ones(as_rgb_array(rgb).shape[:2], dtype=bool)
        if self.hsv_lo is not None and self.hsv_hi is not None:
            hh = rgb_to_hsv(rgb) if hsv is None else hsv
            h, s, v = hh[..., 0], hh[..., 1], hh[..., 2]
            h0, s0, v0 = self.hsv_lo
            h1, s1, v1 = self.hsv_hi
            if h0 <= h1:
                hm = (h >= h0) & (h <= h1)
            else:                                    # 跨 0° 缠绕
                hm = (h >= h0) | (h <= h1)
            out &= hm & (s >= s0) & (s <= s1) & (v >= v0) & (v <= v1)
        if self.rgb_lo is not None and self.rgb_hi is not None:
            a = as_rgb_array(rgb).astype(np.int16)
            lo = np.asarray(self.rgb_lo, dtype=np.int16)
            hi = np.asarray(self.rgb_hi, dtype=np.int16)
            out &= np.all((a >= lo) & (a <= hi), axis=-1)
        return out

    def to_json(self) -> dict:
        return {
            "cls": self.cls, "hsv_lo": self.hsv_lo, "hsv_hi": self.hsv_hi,
            "rgb_lo": self.rgb_lo, "rgb_hi": self.rgb_hi,
            "provisional": self.provisional, "note": self.note,
        }

    @staticmethod
    def from_json(d: dict) -> "ColorRule":
        return ColorRule(
            cls=d["cls"],
            hsv_lo=tuple(d["hsv_lo"]) if d.get("hsv_lo") else None,
            hsv_hi=tuple(d["hsv_hi"]) if d.get("hsv_hi") else None,
            rgb_lo=tuple(d["rgb_lo"]) if d.get("rgb_lo") else None,
            rgb_hi=tuple(d["rgb_hi"]) if d.get("rgb_hi") else None,
            provisional=bool(d.get("provisional", False)),
            note=d.get("note", ""),
        )


@dataclass(frozen=True)
class Palette:
    """一个具名调色板：``cls`` → 一组 :class:`ColorRule`。"""

    name: str
    rules: tuple[ColorRule, ...]
    provenance: str = ""
    allow_provisional: bool = False

    # ---------------- 构造 / 派生 ----------------
    def enable_provisional(self) -> "Palette":
        """打开 provisional 规则（在现场标定之前**不要**用它下结论）。"""
        return replace(self, allow_provisional=True)

    def disable_provisional(self) -> "Palette":
        return replace(self, allow_provisional=False)

    def active_rules(self) -> tuple[ColorRule, ...]:
        if self.allow_provisional:
            return self.rules
        return tuple(r for r in self.rules if not r.provisional)

    def classes(self) -> tuple[str, ...]:
        seen: list[str] = []
        for r in self.active_rules():
            if r.cls not in seen:
                seen.append(r.cls)
        return tuple(seen)

    def with_rule(self, rule: ColorRule) -> "Palette":
        """替换同类名的全部规则（标定覆盖用）。"""
        kept = tuple(r for r in self.rules if r.cls != rule.cls)
        return replace(self, rules=kept + (rule,))

    def merged(self, other: "Palette") -> "Palette":
        out = self
        for r in other.rules:
            out = out.with_rule(r)
        return out

    # ---------------- 分类 ----------------
    def masks(self, rgb) -> dict[str, np.ndarray]:
        """``cls → bool 掩膜``（同类多规则取并集）。"""
        arr = as_rgb_array(rgb)
        hsv = rgb_to_hsv(arr)
        out: dict[str, np.ndarray] = {}
        for rule in self.active_rules():
            m = rule.mask(arr, hsv=hsv)
            if rule.cls in out:
                out[rule.cls] |= m
            else:
                out[rule.cls] = m
        return out

    # ---------------- 序列化 ----------------
    def to_json(self) -> dict:
        return {
            "name": self.name, "provenance": self.provenance,
            "allow_provisional": self.allow_provisional,
            "rules": [r.to_json() for r in self.rules],
        }

    @staticmethod
    def from_json(d: dict) -> "Palette":
        return Palette(
            name=d.get("name", "custom"),
            provenance=d.get("provenance", ""),
            allow_provisional=bool(d.get("allow_provisional", False)),
            rules=tuple(ColorRule.from_json(r) for r in d.get("rules", [])),
        )

    def save(self, path) -> None:
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(self.to_json(), fh, ensure_ascii=False, indent=2)

    @staticmethod
    def load(path) -> "Palette":
        with open(path, "r", encoding="utf-8") as fh:
            return Palette.from_json(json.load(fh))


# --------------------------------------------------------------------------- #
#  两个内置调色板
# --------------------------------------------------------------------------- #
#  (A) 图纸调色板：阈值**逐字**抄自已验证过的识图脚本
#      dzrobot_v2/_tmp_session/t28_newmap/identify_t28_v2.py:58-63
#      （该脚本产出的 map_t28.yaml 已被 t29 接入为唯一真源 config/map.yaml）
MAP_IMAGE = Palette(
    name="map_image",
    provenance=(
        "RGB 阈值抄自 dzrobot_v2/_tmp_session/t28_newmap/identify_t28_v2.py:58-63；"
        "语义（雷=黄块/障=红块/靶=品红块/通=绿块）见 config/map.yaml:48-55 注释与 "
        "t28_newmap/identify_report.txt"
    ),
    rules=(
        ColorRule("obstacle", rgb_lo=(171, 0, 0), rgb_hi=(255, 109, 109),
                  note="红：满格色块 = 障碍（水桶预留格）"),
        ColorRule("mine", rgb_lo=(191, 161, 0), rgb_hi=(255, 255, 129),
                  note="黄：0.9 格近方纯色块 = 雷"),
        ColorRule("target", rgb_lo=(171, 0, 151), rgb_hi=(255, 129, 255),
                  note="品红：靶区格"),
        ColorRule("bridge", rgb_lo=(0, 121, 0), rgb_hi=(139, 255, 149),
                  note="绿：桥洞可通行格（图纸标注）"),
    ),
)

#  (B) 现场调色板：**只有两条有实测证据**，其余是关闭的占位
#
#  证据 1（黄=桥面/挡板）：场地布置说明书 p4「地图中间用黄色挡板隔离」、
#      p8「35 厘米挡板架空结构」；照片 fc-audit/arena_pdf_img/p09_1.png 中桥面是黄色板。
#  证据 2（绿=箭头）：同照片，桥面上贴有**白色纸 + 绿色箭头**的通行标识；
#      规则文档 p11 表也写「桥洞顶部通行箭头 = 贴在桥面顶部的绿色箭头贴纸」。
#  ⚠️ 未取证：地雷（p5「直径 12cm 彩色地雷图片」，颜色未知）、水桶（p7「彩色带盖水桶」，
#      p09_1.png 同框可见**红/青/绿三种桶**）、靶标（p6「低矮黑色装置」）、
#      车顶标记（规则未规定，需己方加装）。这些规则一律 provisional=True。
FIELD_DEFAULT = Palette(
    name="field",
    provenance=(
        "仅 deck/arrow 有实测证据（arena_pdf_img/p09_1.png + 场地布置说明书 p4/p8/p11）；"
        "其余为占位，默认关闭，需 --palette field --provisional 或现场标定"
    ),
    rules=(
        ColorRule("deck", hsv_lo=(30.0, 0.30, 0.40), hsv_hi=(70.0, 1.0, 1.0),
                  note="黄：挡板/桥面（证据：说明书 p4/p8 + p09_1.png）"),
        ColorRule("arrow", hsv_lo=(75.0, 0.25, 0.25), hsv_hi=(165.0, 1.0, 1.0),
                  note="绿：桥顶通行箭头贴纸（证据：p09_1.png 白纸绿箭头）"),
        ColorRule("mine", hsv_lo=(345.0, 0.35, 0.30), hsv_hi=(20.0, 1.0, 1.0),
                  provisional=True,
                  note="占位：假设地雷为红。图纸口径是**黄**（map.yaml:48），实物未知"),
        ColorRule("obstacle", hsv_lo=(345.0, 0.35, 0.30), hsv_hi=(20.0, 1.0, 1.0),
                  provisional=True,
                  note="占位：红桶。但 p09_1.png 同框有红/青/绿三种桶 ⇒ 单色不可靠"),
        ColorRule("obstacle", hsv_lo=(170.0, 0.30, 0.30), hsv_hi=(215.0, 1.0, 1.0),
                  provisional=True, note="占位：青/蓝桶（p09_1.png 实测有）"),
        ColorRule("target", hsv_lo=(290.0, 0.25, 0.25), hsv_hi=(340.0, 1.0, 1.0),
                  provisional=True, note="占位：品红靶区。图纸口径（map.yaml:52）"),
        ColorRule("car", hsv_lo=(20.0, 0.55, 0.55), hsv_hi=(45.0, 1.0, 1.0),
                  provisional=True,
                  note="占位：橙黄车顶板。规则未规定车顶标记，需己方加装并现场标定"),
        ColorRule("car_nose", hsv_lo=(220.0, 0.40, 0.30), hsv_hi=(260.0, 1.0, 1.0),
                  provisional=True, note="占位：车头小标记（用于解 180° 歧义）"),
    ),
)

PALETTES: dict[str, Palette] = {
    "map_image": MAP_IMAGE,
    "field": FIELD_DEFAULT,
}


def classify(rgb, palette: Palette) -> dict[str, np.ndarray]:
    """``palette.masks`` 的别名（模块级入口，便于 ``from .colors import classify``）。"""
    return palette.masks(rgb)


# --------------------------------------------------------------------------- #
#  形态学（二值）
# --------------------------------------------------------------------------- #
def _offsets(k: int) -> list[tuple[int, int]]:
    """k×k 方形结构元的偏移（含自身）。k 必须为正奇数。"""
    if k < 1 or k % 2 == 0:
        raise ValueError(f"结构元边长必须是正奇数，收到 {k}")
    r = k // 2
    return [(dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1)]


def dilate(mask: np.ndarray, k: int = 3) -> np.ndarray:
    """二值膨胀（k×k 方形，**图像外当 False** —— 与 OpenCV 的默认 border 一致）。"""
    m = np.asarray(mask, dtype=bool)
    out = np.zeros_like(m)
    for dy, dx in _offsets(k):
        src = np.zeros_like(m)
        ys = slice(max(0, dy), m.shape[0] + min(0, dy))
        xs = slice(max(0, dx), m.shape[1] + min(0, dx))
        yd = slice(max(0, -dy), m.shape[0] + min(0, -dy))
        xd = slice(max(0, -dx), m.shape[1] + min(0, -dx))
        src[yd, xd] = m[ys, xs]
        out |= src
    return out


def erode(mask: np.ndarray, k: int = 3) -> np.ndarray:
    """二值腐蚀（k×k 方形）。

    **边界语义与 OpenCV 一致**：图像外一律当 ``True``（``erode`` 的默认 border 是
    +∞），所以贴着图像边缘的目标**不会**被凭空削掉一圈；:func:`dilate` 相反。
    """
    m = np.asarray(mask, dtype=bool)
    out = np.ones_like(m)
    for dy, dx in _offsets(k):
        src = np.ones_like(m)
        ys = slice(max(0, dy), m.shape[0] + min(0, dy))
        xs = slice(max(0, dx), m.shape[1] + min(0, dx))
        yd = slice(max(0, -dy), m.shape[0] + min(0, -dy))
        xd = slice(max(0, -dx), m.shape[1] + min(0, -dx))
        src[yd, xd] = m[ys, xs]
        out &= src
    return out


def open_(mask: np.ndarray, k: int = 3) -> np.ndarray:
    """先腐蚀后膨胀：去小噪点（椒盐 / 孤立像素）。"""
    return dilate(erode(mask, k), k)


def close_(mask: np.ndarray, k: int = 3) -> np.ndarray:
    """先膨胀后腐蚀：补小孔洞 / 断裂。"""
    return erode(dilate(mask, k), k)


# --------------------------------------------------------------------------- #
#  连通域（run-based union-find，8 邻接）
# --------------------------------------------------------------------------- #
@dataclass
class Blob:
    """一个连通域。坐标一律 ``(x, y)``（x=列，y=行），与 numpy 索引相反。"""

    label: int
    area: int
    x0: int
    x1: int                      # 闭区间
    y0: int
    y1: int
    cx: float
    cy: float
    mean_rgb: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mxx: float = 0.0             # 关于质心的二阶中心矩（像素单位）
    myy: float = 0.0
    mxy: float = 0.0
    pixels: np.ndarray | None = None   # (N,2) int32，列为 (y, x)

    # ---------------- 派生 ----------------
    @property
    def w(self) -> int:
        return self.x1 - self.x0 + 1

    @property
    def h(self) -> int:
        return self.y1 - self.y0 + 1

    @property
    def bbox_area(self) -> int:
        return self.w * self.h

    @property
    def fill(self) -> float:
        """填充率 = area / bbox 面积。圆盘≈0.785，正方形≈1.0，细长条≪1。"""
        return self.area / float(self.bbox_area)

    @property
    def aspect(self) -> float:
        """bbox 长短边比（≥1）。"""
        return max(self.w, self.h) / float(min(self.w, self.h))

    @property
    def elongation(self) -> float:
        """惯量张量各向异性 ``1 - λ_min/λ_max`` ∈ [0,1]。0=各向同性（yaw 无意义）。"""
        tr = self.mxx + self.myy
        det = self.mxx * self.myy - self.mxy * self.mxy
        disc = max(0.0, tr * tr / 4.0 - det)
        lam_max = tr / 2.0 + np.sqrt(disc)
        lam_min = tr / 2.0 - np.sqrt(disc)
        if lam_max <= 1e-12:
            return 0.0
        return float(max(0.0, 1.0 - max(0.0, lam_min) / lam_max))

    @property
    def axis_deg(self) -> float:
        """主轴方向（图像坐标，x 向右、y 向下），∈[0,180)。各向同性时无意义。"""
        theta = 0.5 * np.arctan2(2.0 * self.mxy, (self.mxx - self.myy))
        return float(np.degrees(theta) % 180.0)

    def to_json(self) -> dict:
        return {"area": self.area, "bbox": [self.x0, self.y0, self.x1, self.y1],
                "c": [round(self.cx, 2), round(self.cy, 2)],
                "fill": round(self.fill, 3), "aspect": round(self.aspect, 3),
                "elongation": round(self.elongation, 3),
                "axis_deg": round(self.axis_deg, 2),
                "mean_rgb": [round(v, 1) for v in self.mean_rgb]}


def connected_components(mask: np.ndarray, min_pixels: int = 1,
                         connectivity: int = 8, rgb=None,
                         want_pixels: bool = True) -> list[Blob]:
    """连通域标记。返回按面积降序的 :class:`Blob` 列表。

    * ``connectivity``：4 或 8（默认 8）。
    * ``min_pixels``：面积下限，小于它的连通域直接丢弃。
    * ``rgb``：给了就顺便算每个 blob 的平均 RGB。
    * ``want_pixels``：是否保留像素索引（方向/位姿估计需要，False 可省内存）。
    """
    m = np.asarray(mask, dtype=bool)
    if m.ndim != 2:
        raise ImageShapeError(f"掩膜必须是 (H, W)，收到 shape={m.shape}")
    if connectivity not in (4, 8):
        raise ValueError(f"connectivity 只能是 4 或 8，收到 {connectivity}")
    H, W = m.shape
    if H == 0 or W == 0 or not m.any():
        return []

    parent: list[int] = []

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra == rb:
            return
        if ra < rb:
            parent[rb] = ra
        else:
            parent[ra] = rb

    runs: list[tuple[int, int, int, int]] = []          # (y, x0, x1, run_id)，x1 开区间
    prev: list[tuple[int, int, int]] = []                # (x0, x1, run_id)
    mi = m.astype(np.int8)
    for y in range(H):
        row = mi[y]
        d = np.diff(np.concatenate((np.zeros(1, np.int8), row, np.zeros(1, np.int8))))
        starts = np.flatnonzero(d == 1)
        ends = np.flatnonzero(d == -1)
        cur: list[tuple[int, int, int]] = []
        p = 0
        for x0, x1 in zip(starts, ends):
            rid = len(parent)
            parent.append(rid)
            lo = x0 - (1 if connectivity == 8 else 0)
            hi = x1 + (1 if connectivity == 8 else 0)     # 与 [lo, hi) 相交即邻接
            while p < len(prev) and prev[p][1] <= lo:
                p += 1
            q = p
            while q < len(prev) and prev[q][0] < hi:
                union(rid, prev[q][2])
                q += 1
            cur.append((x0, x1, rid))
            runs.append((y, x0, x1, rid))
        prev = cur

    groups: dict[int, list[tuple[int, int, int, int]]] = {}
    for run in runs:
        groups.setdefault(find(run[3]), []).append(run)

    arr = as_rgb_array(rgb) if rgb is not None else None
    blobs: list[Blob] = []
    for label, rs in groups.items():
        area = sum(r[2] - r[1] for r in rs)
        if area < min_pixels:
            continue
        x0 = min(r[1] for r in rs)
        x1 = max(r[2] for r in rs) - 1
        y0 = min(r[0] for r in rs)
        y1 = max(r[0] for r in rs)
        ys: list[np.ndarray] = []
        xs: list[np.ndarray] = []
        for y, a, b, _ in rs:
            xs.append(np.arange(a, b, dtype=np.int32))
            ys.append(np.full(b - a, y, dtype=np.int32))
        px = np.stack([np.concatenate(ys), np.concatenate(xs)], axis=1)
        cx = float(px[:, 1].mean())
        cy = float(px[:, 0].mean())
        dx = px[:, 1].astype(np.float64) - cx
        dy = px[:, 0].astype(np.float64) - cy
        mean_rgb = (0.0, 0.0, 0.0)
        if arr is not None:
            vals = arr[px[:, 0], px[:, 1]].astype(np.float64)
            mean_rgb = tuple(float(v) for v in vals.mean(axis=0))
        blobs.append(Blob(
            label=len(blobs), area=int(area), x0=int(x0), x1=int(x1),
            y0=int(y0), y1=int(y1), cx=cx, cy=cy, mean_rgb=mean_rgb,
            mxx=float((dx * dx).mean()), myy=float((dy * dy).mean()),
            mxy=float((dx * dy).mean()),
            pixels=px if want_pixels else None,
        ))
    blobs.sort(key=lambda b: -b.area)
    for i, b in enumerate(blobs):
        b.label = i
    return blobs


def blob_masks(mask: np.ndarray, min_pixels: int = 1, k_open: int = 0,
               k_close: int = 0, rgb=None, want_pixels: bool = True,
               connectivity: int = 8) -> list[Blob]:
    """形态学（可选）→ 连通域 的一步封装。"""
    m = np.asarray(mask, dtype=bool)
    if k_open:
        m = open_(m, k_open)
    if k_close:
        m = close_(m, k_close)
    return connected_components(m, min_pixels=min_pixels, connectivity=connectivity,
                                rgb=rgb, want_pixels=want_pixels)


# --------------------------------------------------------------------------- #
#  现场标定小工具
# --------------------------------------------------------------------------- #
def sample_patch_stats(rgb, x: int, y: int, radius: int = 3) -> dict:
    """取 ``(x, y)`` 附近 ``radius`` 邻域的 RGB/HSV 统计 —— 现场标定第一步。

    返回 ``median_rgb`` / ``median_hsv`` / ``p10_hsv`` / ``p90_hsv`` / ``n``，
    可直接照抄成 :class:`ColorRule` 的阈值。
    """
    arr = as_rgb_array(rgb)
    hsv = rgb_to_hsv(arr)
    H, W = arr.shape[:2]
    x0, x1 = max(0, x - radius), min(W, x + radius + 1)
    y0, y1 = max(0, y - radius), min(H, y + radius + 1)
    if x0 >= x1 or y0 >= y1:
        raise ValueError(f"取样窗口为空：({x},{y}) radius={radius} 图像 {W}x{H}")
    sub = arr[y0:y1, x0:x1].reshape(-1, 3).astype(np.float64)
    subh = hsv[y0:y1, x0:x1].reshape(-1, 3)
    return {
        "n": int(sub.shape[0]),
        "window": [x0, y0, x1 - 1, y1 - 1],
        "median_rgb": [float(v) for v in np.median(sub, axis=0)],
        "median_hsv": [float(v) for v in np.median(subh, axis=0)],
        "p10_hsv": [float(v) for v in np.percentile(subh, 10, axis=0)],
        "p90_hsv": [float(v) for v in np.percentile(subh, 90, axis=0)],
        "std_rgb": [float(v) for v in sub.std(axis=0)],
    }
