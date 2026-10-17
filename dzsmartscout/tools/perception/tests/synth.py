# -*- coding: utf-8 -*-
r"""确定性合成场地渲染器 —— 单测的"真值发生器"。

**为什么需要它**：真机照片里的场地此刻不存在（``cam_probe/`` 全是糊的室内图），
而"格名对不对"这种错误一旦上线就是系统性错位。所以先用一个**解析可算**的相机
把场地投影成图像：相机就是 3×3 单应 ``H``，所有图元都在**场地米制系**里画，再用
``H`` 投到像素。这样：

* 每个图元的真值（格名 / 朝向）都是渲染时给进去的，测试断言的是"管线能不能把它读回来"；
* 单测只把 **4 个角点的像素坐标**喂给注册器，``H`` 本身不参与拟合 ⇒ 不构成作弊；
* 支持旋转 / 透视倾斜 / 固定种子噪声，用来复现"无人机不是正对场地"的真实情况。

**合成色板与真实场地无关**：这里用的是"各图元互相不撞色"的人造色（见
:data:`SYNTH_FIELD_PALETTE` / ``colors.MAP_IMAGE``），**不能**用来推断现场颜色。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Sequence

import numpy as np
from PIL import Image, ImageDraw

from ..colors import ColorRule, Palette
from ..grid import GridSpec, Homography, arena_corner_meters

__all__ = [
    "SynthArena",
    "make_camera",
    "render_arena",
    "arrow_polygon_meters",
    "board_polygon_meters",
    "SYNTH_FIELD_PALETTE",
    "MAT",
    "GRID",
]

MAT = (246, 246, 242)        # 刀刮布白（略暖，避免与纯白高光混淆）
GRID = (170, 175, 180)

#: 两套渲染风格：``field``（实物俯视：雷=蓝圆片、桶=红圆、桥面=黄板、箭头=绿）
#: 与 ``drawing``（图纸：黄雷块/红障块/品红靶块/绿通块）。
#: 颜色取值都经过核对，能与对应调色板**且只与对应调色板**匹配（见 test_colors/test_objects）。
STYLE_TABLE: dict[str, dict] = {
    "field": {
        "mine": ((40, 90, 220), "disc", 0.060),       # 12 cm 直径 ⇒ r=0.06 m
        "obstacle": ((214, 40, 40), "disc", 0.150),   # 30 cm 桶俯视 ⇒ r=0.15 m
        "target": ((226, 80, 190), "cell", 0.75),
        "bridge": ((10, 190, 190), "cell", 0.86),
        "deck": (240, 205, 40),
        "arrow": (30, 170, 70),
        "car": (240, 130, 30),
        "car_nose": (150, 90, 230),
    },
    "drawing": {
        "mine": ((235, 205, 60), "cell", 0.86),       # 图纸：黄雷块
        "obstacle": ((214, 40, 40), "cell", 0.86),    # 图纸：红满格
        "target": ((226, 80, 190), "cell", 0.86),     # 图纸：品红靶区
        "bridge": ((60, 180, 90), "cell", 0.86),      # 图纸：绿"通"
        "deck": (240, 205, 40),
        "arrow": (30, 170, 70),
        "car": (240, 130, 30),
        "car_nose": (150, 90, 230),
    },
}


# --------------------------------------------------------------------------- #
#  合成色板（各图元不撞色；与现场颜色无关）
# --------------------------------------------------------------------------- #
def _rule(cls: str, lo: tuple, hi: tuple, note: str = "") -> ColorRule:
    return ColorRule(cls=cls, hsv_lo=lo, hsv_hi=hi, note=note)


#: 合成测试用：mine=蓝、obstacle=红、target=品红、bridge=青、deck=黄、arrow=绿、
#: car=橙、car_nose=紫。**故意让 mine≠黄**，这样"雷=黄"这种假设不会被测试放行。
SYNTH_FIELD_PALETTE = Palette(
    name="synth_field",
    provenance="合成单测专用（各图元不撞色），不代表现场颜色",
    rules=(
        _rule("mine", (200.0, 0.55, 0.35), (250.0, 1.0, 1.0), "合成：蓝"),
        _rule("obstacle", (345.0, 0.60, 0.35), (12.0, 1.0, 1.0), "合成：红"),
        _rule("target", (285.0, 0.45, 0.35), (330.0, 1.0, 1.0), "合成：品红"),
        _rule("bridge", (170.0, 0.45, 0.35), (195.0, 1.0, 1.0), "合成：青"),
        _rule("deck", (40.0, 0.45, 0.55), (62.0, 1.0, 1.0), "合成：黄桥面"),
        _rule("arrow", (100.0, 0.45, 0.25), (145.0, 1.0, 1.0), "合成：绿箭头"),
        _rule("car", (18.0, 0.65, 0.55), (38.0, 1.0, 1.0), "合成：橙车顶板"),
        _rule("car_nose", (255.0, 0.35, 0.25), (285.0, 1.0, 1.0), "合成：紫车头标记"),
    ),
)


# --------------------------------------------------------------------------- #
#  虚拟相机
# --------------------------------------------------------------------------- #
def make_camera(spec: GridSpec | None = None, *, px_per_m: float = 64.0,
                rot_deg: float = 0.0, tilt: float = 0.0,
                pad: int = 12) -> tuple[Homography, tuple[int, int]]:
    """构造 ``meters → pixels`` 的虚拟相机，返回 ``(H, (W, H))``。

    基座是"正俯视、行 1 在上、列 A 在左"的相似变换（**有向面积为负**，见
    ``grid.py`` 的手性说明）；``rot_deg`` 绕图像中心旋转；``tilt`` 给一个
    ``k`` 的透视项 ``v' = v / (1 + k·v_norm)``，模拟相机不垂直向下。
    """
    spec = spec or GridSpec()
    s = float(px_per_m)
    Wm, Hm = spec.width_m, spec.height_m
    # meters -> 未加 pad 的像素
    base = np.array([[0.0, -s, s * Wm],
                     [-s, 0.0, s * Hm],
                     [0.0, 0.0, 1.0]])
    corners_m = np.array([arena_corner_meters(t, spec)
                          for t in ("N11", "N1", "A1", "A11")], dtype=np.float64)
    corners = Homography(base).apply(corners_m)
    w = float(corners[:, 0].max() - corners[:, 0].min() + 2 * pad)
    h = float(corners[:, 1].max() - corners[:, 1].min() + 2 * pad)
    # pad：把 bbox 挪到 (pad, pad) 起
    shift = np.array([[1.0, 0.0, pad - corners[:, 0].min()],
                      [0.0, 1.0, pad - corners[:, 1].min()],
                      [0.0, 0.0, 1.0]])
    H = shift @ base
    cx, cy = w / 2.0, h / 2.0
    if rot_deg:
        a = np.radians(rot_deg)
        ca, sa = np.cos(a), np.sin(a)
        R = np.array([[ca, -sa, cx - ca * cx + sa * cy],
                      [sa, ca, cy - sa * cx - ca * cy],
                      [0.0, 0.0, 1.0]])
        H = R @ H
        corners = Homography(H).apply(corners_m)
        w = float(corners[:, 0].max() - corners[:, 0].min() + 2 * pad)
        h = float(corners[:, 1].max() - corners[:, 1].min() + 2 * pad)
        shift2 = np.array([[1.0, 0.0, pad - corners[:, 0].min()],
                           [0.0, 1.0, pad - corners[:, 1].min()],
                           [0.0, 0.0, 1.0]])
        H = shift2 @ H
    if tilt:
        # 在图像下半部分施加透视（模拟镜头俯仰）
        corners = Homography(H).apply(corners_m)
        H = np.array([[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0],
                      [0.0, tilt / max(h, 1.0), 1.0]]) @ H
    size = (int(round(w)), int(round(h)))
    return Homography(H), size


# --------------------------------------------------------------------------- #
#  图元（全部在米制系里定义）
# --------------------------------------------------------------------------- #
def arrow_polygon_meters(center_xy: tuple[float, float], dir_deg: float,
                         length: float, width: float = None) -> np.ndarray:
    """在场地米制系里造一个箭头多边形（尖头指向 ``dir_deg``）。

    ``dir_deg``：0 = +x（行号减小，即朝上半场），90 = +y（列号减小，即朝图左）。
    """
    L = float(length)
    Wd = float(width) if width is not None else L * 0.72
    head = L * 0.45
    shaft = Wd * 0.34
    local = np.array([
        [L / 2.0, 0.0],                    # 尖
        [L / 2.0 - head, -Wd / 2.0],       # 头左翼
        [L / 2.0 - head, -shaft / 2.0],
        [-L / 2.0, -shaft / 2.0],          # 尾左
        [-L / 2.0, shaft / 2.0],
        [L / 2.0 - head, shaft / 2.0],
        [L / 2.0 - head, Wd / 2.0],        # 头右翼
    ])
    a = np.radians(float(dir_deg))
    ca, sa = np.cos(a), np.sin(a)
    rot = np.array([[ca, -sa], [sa, ca]])
    return local @ rot.T + np.asarray(center_xy, dtype=np.float64)


def board_polygon_meters(center_xy: tuple[float, float], yaw_deg: float,
                         length: float = 0.22, width: float = 0.10) -> np.ndarray:
    """矩形"车顶板"：长边沿 ``yaw_deg``（场地米制系角度），中心在 ``center_xy``。"""
    local = np.array([[-length / 2, -width / 2], [length / 2, -width / 2],
                      [length / 2, width / 2], [-length / 2, width / 2]])
    a = np.radians(float(yaw_deg))
    ca, sa = np.cos(a), np.sin(a)
    rot = np.array([[ca, -sa], [sa, ca]])
    return local @ rot.T + np.asarray(center_xy, dtype=np.float64)


def _disc_meters(center_xy: tuple[float, float], radius: float, n: int = 24) -> np.ndarray:
    t = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    return np.stack([center_xy[0] + radius * np.cos(t),
                     center_xy[1] + radius * np.sin(t)], axis=1)


# --------------------------------------------------------------------------- #
#  渲染
# --------------------------------------------------------------------------- #
@dataclass
class SynthArena:
    image: np.ndarray
    H: Homography
    spec: GridSpec
    corners_px: dict          # {"N11": (u,v), ...}
    truth: dict = field(default_factory=dict)

    def save(self, path) -> None:
        Image.fromarray(self.image).save(path)


def render_arena(spec: GridSpec | None = None, *, px_per_m: float = 64.0,
                 rot_deg: float = 0.0, tilt: float = 0.0,
                 mines: Iterable[str] = (), obstacles: Iterable[str] = (),
                 targets: Iterable[str] = (), bridges: Iterable[str] = (),
                 deck_col: str | None = None,
                 arrows: Sequence[tuple[str, float]] = (),
                 car: tuple[str, float] | None = None,
                 car_nose_offset: float = 0.0,
                 style: str = "field",
                 arrow_len_m: float = 0.20,
                 noise_sigma: float = 0.0, seed: int = 0,
                 draw_grid: bool = True, draw_border: bool = True) -> SynthArena:
    """渲染一张俯视合成图并返回真值。

    * ``mines`` / ``obstacles`` / ``targets`` / ``bridges``：格名 → 按 ``style``
      画圆盘或色块（``field``：雷 12 cm 圆片、桶 30 cm 圆；``drawing``：近满格方块）。
    * ``deck_col``：桥面（黄板）覆盖的**列**，例如 ``"K"``；``arrows`` 是
      ``(桥面格名, 指向角度)``，箭头画在该格中心的白纸上（桥面之上）。
    * ``car``：``(格名, yaw_deg)`` 画车顶板；``car_nose_offset>0`` 时在板前方
      再画一个小方块（用于解 180° 歧义）。
    """
    spec = spec or GridSpec()
    if style not in STYLE_TABLE:
        raise ValueError(f"style 只能是 {sorted(STYLE_TABLE)}，收到 {style!r}")
    tbl = STYLE_TABLE[style]
    H, size = make_camera(spec, px_per_m=px_per_m, rot_deg=rot_deg, tilt=tilt)
    img = Image.new("RGB", size, MAT)
    draw = ImageDraw.Draw(img)

    def poly(pts_m) -> list[tuple[float, float]]:
        return [tuple(p) for p in H.apply(np.asarray(pts_m, dtype=np.float64))]

    def block_poly(block: str, frac: float = 1.0) -> np.ndarray:
        r, c = spec.block_to_index(block)
        cx, cy = spec.block_to_meters(block)
        half = spec.cell_m * frac / 2.0
        return np.array([[cx - half, cy - half], [cx + half, cy - half],
                         [cx + half, cy + half], [cx - half, cy + half]])

    def draw_one(block: str, cls: str) -> None:
        color, shape, size_ = tbl[cls]
        cx, cy = spec.block_to_meters(block)
        if shape == "disc":
            draw.polygon(poly(_disc_meters((cx, cy), size_)), fill=color)
        else:
            draw.polygon(poly(block_poly(block, size_)), fill=color)

    # 网格线
    if draw_grid:
        for block in spec.all_blocks():
            draw.polygon(poly(block_poly(block, 1.0)), outline=GRID)
    if draw_border:
        border = np.array([arena_corner_meters(t, spec)
                           for t in ("N11", "N1", "A1", "A11")], dtype=np.float64)
        draw.polygon(poly(border), outline=(60, 60, 60))

    # 色块类图元
    for block in obstacles:
        draw_one(block, "obstacle")
    for block in targets:
        draw_one(block, "target")
    for block in bridges:
        draw_one(block, "bridge")
    for block in mines:
        draw_one(block, "mine")

    # 桥面（黄板）+ 箭头
    if deck_col:
        ci = spec.col_letters.index(deck_col.upper())
        for r in range(spec.rows):
            block = spec.index_to_block(r, ci)
            draw.polygon(poly(block_poly(block, 0.98)), fill=tbl["deck"])
    for block, deg in arrows:
        cx, cy = spec.block_to_meters(block)
        # 白纸底 + 箭头（与 p09_1.png 的实物形态一致）
        draw.polygon(poly(block_poly(block, 0.62)), fill=(250, 250, 250))
        draw.polygon(poly(arrow_polygon_meters((cx, cy), deg, arrow_len_m)),
                     fill=tbl["arrow"])

    # 车
    if car is not None:
        block, yaw = car
        cx, cy = spec.block_to_meters(block)
        draw.polygon(poly(board_polygon_meters((cx, cy), yaw)), fill=tbl["car"])
        if car_nose_offset:
            a = np.radians(yaw)
            nx = cx + car_nose_offset * np.cos(a)
            ny = cy + car_nose_offset * np.sin(a)
            draw.polygon(poly(board_polygon_meters((nx, ny), yaw, 0.06, 0.06)),
                         fill=tbl["car_nose"])

    arr = np.asarray(img).astype(np.float64)
    if noise_sigma > 0:
        rng = np.random.default_rng(seed)
        arr = arr + rng.normal(0.0, float(noise_sigma), arr.shape)
    arr = np.clip(arr, 0, 255).astype(np.uint8)

    corners_px = {t: tuple(float(v) for v in
                           H.apply(np.asarray([arena_corner_meters(t, spec)]))[0])
                  for t in ("N11", "N1", "A1", "A11")}
    truth = {
        "mines": sorted(mines), "obstacles": sorted(obstacles),
        "targets": sorted(targets), "bridges": sorted(bridges),
        "arrows": [(b, float(d)) for b, d in arrows],
        "deck_col": deck_col, "car": car,
    }
    return SynthArena(image=arr, H=H, spec=spec, corners_px=corners_px, truth=truth)
