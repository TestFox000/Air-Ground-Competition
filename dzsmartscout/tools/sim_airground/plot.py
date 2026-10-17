# -*- coding: utf-8 -*-
"""路线图渲染（Pillow → PNG）：网格 + 场地真值 + 车路线/轨迹 + 车位姿 + 无人机航线。

t28 改版（四项改进之 ①②）：
- **一张图两条路线**（同一坐标系 ``_px``）：小车路线（蓝·参考 token 折线）+ 小车真值轨迹
  （红·run_sim truth 轨迹）+ 无人机航线（青·虚线）+ K 列侦察腿（紫·加粗），图例区分；
- **车位姿标注**：真值轨迹上均匀抽 **60 个位姿点**（`pose_every` 由调用方按
  ``ceil(steps/60)`` 定死），画车体矩形（0.28×0.135 m）+ 车头朝向箭头；
  ⛔ 位姿是**真值口径**（``truth_x/truth_y/truth_yaw``），图例文字明示"车位姿(真值)"，
  不与信念位姿混用（t21 教训：信念↔真值 RMS 误当横向 RMS）。

判据纪律：每次仿真运行必须留 route_map.png（交接包）；本模块不依赖 matplotlib。
"""

from __future__ import annotations

import math
import os

from .drone_model import DroneTimeline
from .geo import BLOCK_M, COL_LETTERS, MAP_COLS, MAP_ROWS, block_to_meters, index_to_block

# 画布参数
_W, _H = 1400, 1180
_X0, _Y0 = 90, 70          # 左上角
_SCALE = _W / (MAP_COLS * BLOCK_M * 1.22)   # 每米像素（留边）

# 线/标记配色（自检按这些**精确色值**数像素，改色必须同步改自检）
_C_ROUTE = (30, 80, 220)        # 车路线（参考 token 折线）
_C_TRAJ = (200, 40, 40)         # 车真值轨迹
_C_POSE = (240, 120, 30)        # 车位姿（车体矩形 + 朝向箭头）
_C_DRONE = (60, 160, 200)       # 无人机航线
_C_RECON = (150, 60, 200)       # K 列侦察腿
_C_MINE = (235, 180, 40)
_C_OBS = (200, 60, 60)
_C_TGT = (200, 80, 200)
_C_BRIDGE = (70, 190, 90)
_C_BARRIER = (40, 40, 40)

#: 车体物理（与 route_check/config 同源：长 0.28 / 宽 0.135）
_VEH_LEN_M = 0.28
_VEH_WID_M = 0.135

#: 图例两行（t28 ①：双路线 + 位姿口径各自成条；自检按这些文案断言图例完整）
LEGEND_ROW_FIELD = ("雷", _C_MINE), ("障", _C_OBS), ("靶", _C_TGT), ("通口", _C_BRIDGE)
LEGEND_ROW_ROUTE = (("车路线(参考)", _C_ROUTE), ("车轨迹(真值)", _C_TRAJ),
                    ("车位姿(真值)", _C_POSE), ("无人机航线", _C_DRONE),
                    ("K列侦察腿", _C_RECON))


def _px(x: float, y: float) -> tuple[float, float]:
    return _X0 + y * _SCALE, _Y0 + (MAP_ROWS * BLOCK_M - x) * _SCALE


def draw_map(png_path: str, *, title: str, truth, car_tokens: list[str],
             car_cells: list[str] | None = None,
             car_traj: list[tuple[float, float]] | None = None,
             car_poses: list[tuple[float, float, float]] | None = None,
             pose_every: int = 1, drone: DroneTimeline | None = None,
             reveal_joint: dict[str, float] | None = None, key_cells: list[str] | None = None,
             car_abort_t: float | None = None) -> str:
    """渲染**一张**路线图（车+机同图同坐标系）。``truth`` 为 FieldSpec。返回 png 路径。

    :param car_tokens: 参考路线 token（蓝线）。
    :param car_traj: 真值轨迹 (x, y) 序列（红线）。
    :param car_poses: 真值位姿 (x, y, yaw) 序列（橙：车体矩形 + 朝向箭头，每 ``pose_every``
        步画一个）。
    """
    from PIL import Image, ImageDraw, ImageFont
    img = Image.new("RGB", (_W, _H), (252, 252, 250))
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("arial.ttf", 14)
        font_s = ImageFont.truetype("arial.ttf", 11)
        font_b = ImageFont.truetype("arialbd.ttf", 20)
    except Exception:                                # noqa: BLE001  无字体环境用默认
        font = font_s = font_b = ImageFont.load_default()

    # ---- 网格与格名 ----
    for r in range(MAP_ROWS):
        for c in range(MAP_COLS):
            x, y = block_to_meters(index_to_block(r, c))
            px, py = _px(x - BLOCK_M / 2, y - BLOCK_M / 2)
            d.rectangle([px, py, px + BLOCK_M * _SCALE, py + BLOCK_M * _SCALE],
                        outline=(220, 220, 216), width=1)
            if r == 0 or c == 0:
                d.text((px + 2, py + 2), index_to_block(r, c), fill=(160, 160, 156), font=font_s)

    # ---- 场地真值 ----
    def cell_rect(block):
        x, y = block_to_meters(block)
        px, py = _px(x - BLOCK_M / 2, y + BLOCK_M / 2)
        return [px, py, px + BLOCK_M * _SCALE, py + BLOCK_M * _SCALE]

    for b in truth.obstacles:
        x, y = block_to_meters(b)
        px, py = _px(x, y)
        r_px = 0.1275 * _SCALE
        d.ellipse([px - r_px, py - r_px, px + r_px, py + r_px], fill=_C_OBS, outline=(120, 0, 0))
    for b in truth.mines:
        x, y = block_to_meters(b)
        px, py = _px(x, y)
        h = 0.12 * _SCALE
        d.rectangle([px - h / 2, py - h / 2, px + h / 2, py + h / 2], fill=_C_MINE)
    for b in truth.targets:
        x, y = block_to_meters(b)
        px, py = _px(x, y)
        h = 0.175 * _SCALE
        d.rectangle([px - h / 2, py - h / 2, px + h / 2, py + h / 2], fill=_C_TGT)
    for b in truth.bridges:
        x, y = block_to_meters(b)
        px, py = _px(x, y)
        h = 0.16 * _SCALE
        d.rectangle([px - h / 2, py - h / 2, px + h / 2, py + h / 2], fill=_C_BRIDGE)
    # 挡板（第 6/7 行之间，D..N）
    bar_x0, bar_y0, bar_y1 = truth.barrier_col_from, truth.barrier_row_hi, truth.barrier_row_lo
    if bar_x0:
        c0 = COL_LETTERS.index(bar_x0)
        yb = (10.5 - (bar_y0 - 0.5)) * BLOCK_M
        x_from = (10.5 - bar_y0) * BLOCK_M
        px1, py1 = _px(x_from, (13.5 - c0) * BLOCK_M)
        px2, py2 = _px((10.5 - bar_y1) * BLOCK_M, 0.0)
        d.line([px1, (py1 + py2) / 2, px2, (py1 + py2) / 2], fill=_C_BARRIER, width=3)

    # ---- 无人机航线 + 侦察腿（同一坐标系，青虚线 + 紫加粗） ----
    if drone is not None:
        pts = [block_to_meters(b) for b in drone.waypoints]
        k_pts = [p for p in pts if abs(p[1] - block_to_meters("K1")[1]) < 1e-9]
        for a, b in zip(k_pts, k_pts[1:]):
            pa, pb = _px(*a), _px(*b)
            d.line([*pa, *pb], fill=_C_RECON, width=4)
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            if i % 2 == 0:
                pa, pb = _px(*a), _px(*b)
                d.line([*pa, *pb], fill=_C_DRONE, width=2)
        for label, blk, color in (("N8 起", "N8", (0, 0, 220)), ("M2 降", "M2", (0, 120, 0))):
            px, py = _px(*block_to_meters(blk))
            d.ellipse([px - 6, py - 6, px + 6, py + 6], fill=color)
            d.text((px + 8, py - 8), label, fill=color, font=font_s)

    # ---- 车路线（蓝·参考 token 折线） ----
    route_pts: list[tuple[float, float]] = []
    if car_tokens:
        for tok in car_tokens:
            text = str(tok).strip().upper()
            base = text[2:] if text.startswith("R:") else text
            base = base.strip("*").rstrip(",y").strip(",")
            if not base:
                continue
            try:
                route_pts.append(block_to_meters(base))
            except ValueError:
                continue
        for i in range(len(route_pts) - 1):
            pa, pb = _px(*route_pts[i]), _px(*route_pts[i + 1])
            d.line([*pa, *pb], fill=_C_ROUTE, width=2)
        for x, y in route_pts:
            px, py = _px(x, y)
            d.ellipse([px - 3, py - 3, px + 3, py + 3], fill=_C_ROUTE)
        # 起/终点标注
        for label, pt, color in (("起", route_pts[0], (0, 60, 200)),
                                 ("终", route_pts[-1], (0, 140, 60))):
            px, py = _px(*pt)
            d.text((px - 14, py - 20), label, fill=color, font=font_b)

    # ---- 车真值轨迹（红·truth 轨迹） ----
    if car_traj:
        for i in range(len(car_traj) - 1):
            pa, pb = _px(*car_traj[i]), _px(*car_traj[i + 1])
            d.line([*pa, *pb], fill=_C_TRAJ, width=1)
    if car_cells:
        first = True
        for cell in car_cells:
            x, y = block_to_meters(cell)
            px, py = _px(x, y)
            if first:
                first = False
                continue
            d.ellipse([px - 1, py - 1, px + 1, py + 1], fill=(20, 40, 120))
    if car_abort_t is not None and car_cells:
        x, y = block_to_meters(car_cells[-1])
        px, py = _px(x, y)
        d.ellipse([px - 8, py - 8, px + 8, py + 8], outline=(220, 0, 0), width=3)

    # ---- 车位姿（t28 ②：真值口径车体矩形 + 朝向箭头，每 pose_every 步一个） ----
    if car_poses:
        hl, hw = _VEH_LEN_M / 2.0, _VEH_WID_M / 2.0
        for i in range(0, len(car_poses), max(1, pose_every)):
            x, y, yaw = car_poses[i]
            c, s = math.cos(yaw), math.sin(yaw)
            corners = []
            for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)):
                corners.append(_px(x + dx * c - dy * s, y + dx * s + dy * c))
            d.polygon(corners, outline=_C_POSE, fill=(250, 220, 190))
            px0, py0 = _px(x, y)
            px1, py1 = _px(x + 0.16 * c, y + 0.16 * s)     # 朝向箭头 = 车头方向 0.16 m
            d.line([px0, py0, px1, py1], fill=_C_POSE, width=3)

    # ---- 图例（两行：场地图例 + 路线/位姿图例） ----
    lx = 40
    ly_field = _H - 62
    ly_route = _H - 36
    for i, (txt, col) in enumerate(LEGEND_ROW_FIELD):
        x = lx + i * 150
        d.rectangle([x, ly_field - 8, x + 14, ly_field + 6], fill=col)
        d.text((x + 18, ly_field - 12), txt, fill=(40, 40, 40), font=font_s)
    for i, (txt, col) in enumerate(LEGEND_ROW_ROUTE):
        x = lx + i * 150
        d.rectangle([x, ly_route - 8, x + 14, ly_route + 6], fill=col)
        d.text((x + 18, ly_route - 12), txt, fill=(40, 40, 40), font=font_s)
    d.text((_X0, 24), title, fill=(20, 20, 20), font=font_b)

    img.save(png_path)
    return os.path.abspath(png_path)
