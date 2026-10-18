# -*- coding: utf-8 -*-
"""裁判系统随机地图（2026-10-15 截图）→ 新路径规划 → 渲染。

地图元素（从 referee_map.png 人工判读）：
  雷: B6 B10 D8 E1 F5 H3 I6 I8（8 颗，符合规则）
  障碍: B2 B9 C1 D11 E4 F9 H11 I7（8 个，符合规则）
  桥洞通行格: K2（下段 K1~K6 的隧道口）、K8（上段 K7~K11 的隧道口）
  移动靶轨道: E2-E3（蓝面朝向图上读不出，演示按 W 朝西；实赛由裁判系统地图给）
  智能车起点 N10 / 终点 N5（图右缘标注），朝向按比赛惯例 y/y
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parent / "air_ground"
sys.path.insert(0, str(REPO / "src" / "ag_integrate"))

from ag_integrate.field_frame import CellRef, COLS, LETTERS, ROWS  # noqa: E402
from ag_integrate.mission_router import plan_route  # noqa: E402
from ag_integrate.world_model import TargetRef, WorldSnapshot  # noqa: E402


def cell(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0].upper()), int(name[1:]) - 1)


MINES = ["B6", "B10", "D8", "E1", "F5", "H3", "I6", "I8"]
OBSTACLES = ["B2", "B9", "C1", "D11", "E4", "F9", "H11", "I7"]
PASSABLE = ["K2", "K8"]
TRACK = ("E2", "E3")
BLUE_FACE = "W"   # ⚠️ 图上读不出蓝面朝向，演示假设朝西
START, GOAL = "N10", "N5"

# 桥体（现场确认）：下段 K1~K6、上段 K7~K11
BRIDGE_DECK = [f"K{r}" for r in range(1, 7)] + [f"K{r}" for r in range(7, 12)]
BARRIER_ROW_BOUNDARY = 6  # 第6/7行之间（mb=6，已现场确认）


def build_snapshot() -> WorldSnapshot:
    targets = [TargetRef(cell("E2"), is_blue=True,
                         track=(cell(TRACK[0]), cell(TRACK[1])), blue_face=BLUE_FACE)]
    return WorldSnapshot(
        0,
        tuple(cell(n) for n in MINES),
        tuple(cell(n) for n in PASSABLE),
        tuple(cell(n) for n in OBSTACLES),
        tuple(targets),
    )


# ---------------- 渲染 ----------------
CELL = 56
ML, MT, MR, MB = 64, 64, 300, 96   # 左右下外边距（右侧图例）
W = ML + COLS * CELL + MR
H = MT + ROWS * CELL + MB

FONT_PATH = r"C:\Windows\Fonts\msyh.ttc"
FONT = ImageFont.truetype(FONT_PATH, 18)
FONT_S = ImageFont.truetype(FONT_PATH, 14)
FONT_B = ImageFont.truetype(FONT_PATH, 22)


def center(c: CellRef) -> tuple[int, int]:
    return (ML + c.col * CELL + CELL // 2, MT + c.row * CELL + CELL // 2)


def render(snap: WorldSnapshot, route: tuple[str, ...], out: Path,
           note_lines: list[str]) -> None:
    img = Image.new("RGB", (W, H), (250, 250, 250))
    dr = ImageDraw.Draw(img, "RGBA")

    # 区域底色：西区走廊淡蓝 / 东南淡黄 / 东北淡绿
    def rect_zone(c0, r0, c1, r1, color):
        dr.rectangle([ML + c0 * CELL, MT + r0 * CELL,
                      ML + (c1 + 1) * CELL, MT + (r1 + 1) * CELL], fill=color)
    rect_zone(0, 0, 2, ROWS - 1, (219, 236, 252))          # 西区 A~C 通长
    rect_zone(3, 0, COLS - 1, BARRIER_ROW_BOUNDARY - 1, (255, 247, 214))  # 东南 D~N 1~6
    rect_zone(3, BARRIER_ROW_BOUNDARY, COLS - 1, ROWS - 1, (222, 245, 222))  # 东北 D~N 7~11

    # 网格线
    for r in range(ROWS + 1):
        y = MT + r * CELL
        dr.line([ML, y, ML + COLS * CELL, y], fill=(190, 190, 190), width=1)
    for c in range(COLS + 1):
        x = ML + c * CELL
        dr.line([x, MT, x, MT + ROWS * CELL], fill=(190, 190, 190), width=1)

    # 挡板（第6/7行边界，D~N列，黄色条带）
    by = MT + BARRIER_ROW_BOUNDARY * CELL
    dr.rectangle([ML + 3 * CELL, by - 7, ML + COLS * CELL, by + 7],
                 fill=(255, 193, 7, 200))
    dr.text((ML + 3 * CELL + 6, by - 24), "挡板 D7~N7", font=FONT_S, fill=(150, 100, 0))

    # K 列桥体（灰），隧道格（绿，稍后再画强调）
    for name in BRIDGE_DECK:
        c = cell(name)
        if name in PASSABLE:
            continue
        x, y = center(c)
        dr.rectangle([x - CELL // 2 + 3, y - CELL // 2 + 3,
                      x + CELL // 2 - 3, y + CELL // 2 - 3],
                     fill=(120, 120, 120), outline=(60, 60, 60))
        dr.text((x - 9, y - 12), "桥", font=FONT_S, fill=(255, 255, 255))

    # 元素
    for c in snap.obstacles:
        x, y = center(c)
        dr.rectangle([x - CELL // 2 + 3, y - CELL // 2 + 3, x + CELL // 2 - 3, y + CELL // 2 - 3],
                     fill=(220, 60, 60), outline=(140, 0, 0), width=2)
        dr.text((x - 10, y - 13), "障", font=FONT, fill=(255, 255, 255))
    for c in snap.passable:
        x, y = center(c)
        dr.rectangle([x - CELL // 2 + 3, y - CELL // 2 + 3, x + CELL // 2 - 3, y + CELL // 2 - 3],
                     fill=(40, 180, 60), outline=(0, 110, 0), width=2)
        dr.text((x - 10, y - 13), "通", font=FONT, fill=(255, 255, 255))
    for c in snap.mines:
        x, y = center(c)
        dr.ellipse([x - CELL // 3, y - CELL // 3, x + CELL // 3, y + CELL // 3],
                   fill=(255, 165, 0), outline=(180, 90, 0), width=2)
        dr.text((x - 10, y - 13), "雷", font=FONT, fill=(255, 255, 255))
    for t in snap.targets:
        for tc in (t.track or (t.cell,)):
            x, y = center(tc)
            dr.rectangle([x - CELL // 2 + 3, y - CELL // 2 + 3, x + CELL // 2 - 3, y + CELL // 2 - 3],
                         fill=(255, 105, 180), outline=(160, 30, 90), width=2)
            dr.text((x - 10, y - 13), "靶", font=FONT, fill=(255, 255, 255))

    # 起点/终点
    sx, sy = center(cell(START))
    gx, gy = center(cell(GOAL))
    dr.ellipse([sx - 18, sy - 18, sx + 18, sy + 18], fill=(30, 120, 255))
    dr.text((sx - 7, sy - 12), "起", font=FONT, fill=(255, 255, 255))
    dr.ellipse([gx - 18, gy - 18, gx + 18, gy + 18], fill=(200, 30, 30))
    dr.text((gx - 7, gy - 12), "终", font=FONT, fill=(255, 255, 255))

    # 路线（折线 + 方向箭头 + 序号）
    pts = [center(cell(n.rstrip("*,xy"))) for n in route]
    for i in range(len(pts) - 1):
        dr.line([pts[i], pts[i + 1]], fill=(30, 100, 255), width=4)
        # 箭头（在线段 70% 处画小三角指示方向）
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        ax, ay = x0 + (x1 - x0) * 0.7, y0 + (y1 - y0) * 0.7
        dx, dy = x1 - x0, y1 - y0
        L = (dx * dx + dy * dy) ** 0.5 or 1
        ux, uy = dx / L, dy / L
        px, py = -uy, ux
        dr.polygon([(ax + 8 * ux, ay + 8 * uy),
                    (ax - 4 * ux + 5 * px, ay - 4 * uy + 5 * py),
                    (ax - 4 * ux - 5 * px, ay - 4 * uy - 5 * py)],
                   fill=(30, 100, 255))
    for i, p in enumerate(pts):
        dr.ellipse([p[0] - 4, p[1] - 4, p[0] + 4, p[1] + 4], fill=(200, 30, 30))
    # 射击格星标
    for n in route:
        if n.endswith("*"):
            x, y = center(cell(n.rstrip("*")))
            dr.text((x - 14, y - 38), "★射", font=FONT_S, fill=(200, 30, 30))

    # 列字母 / 行号
    for ci in range(COLS):
        dr.text((ML + ci * CELL + CELL // 2 - 6, MT - 26), LETTERS[ci],
                font=FONT_S, fill=(60, 60, 60))
    for ri in range(ROWS):
        dr.text((ML - 30, MT + ri * CELL + CELL // 2 - 8), str(ri + 1),
                font=FONT_S, fill=(60, 60, 60))

    # 图例 + 说明（右侧）
    lx = ML + COLS * CELL + 18
    ly = MT
    legend = [
        ((255, 165, 0), "雷（8）"), ((220, 60, 60), "障碍物（8）"),
        ((40, 180, 60), "桥洞隧道口"), ((120, 120, 120), "桥体（不可踩）"),
        ((255, 105, 180), "移动靶轨道"), ((255, 193, 7), "挡板 D7~N7"),
        ((30, 120, 255), "起点 N10"), ((200, 30, 30), "终点 N5"),
    ]
    dr.text((lx, ly - 24), "图例", font=FONT_B, fill=(0, 0, 0))
    for color, name in legend:
        dr.rectangle([lx, ly, lx + 18, ly + 18], fill=color)
        dr.text((lx + 24, ly - 2), name, font=FONT_S, fill=(20, 20, 20))
        ly += 26
    ly += 8
    for line in note_lines:
        dr.text((lx, ly), line, font=FONT_S, fill=(60, 60, 60))
        ly += 20

    # 底部标题（路线长，两行折行防溢出）
    dr.text((ML, H - MB + 8), f"路线（{len(route)} 格）:", font=FONT, fill=(0, 0, 0))
    route_txt = " → ".join(route)
    mid = len(route_txt) // 2
    cut = route_txt.find(" → ", mid - 30, mid + 30)
    line1, line2 = route_txt[:cut], route_txt[cut + 4:]
    dr.text((ML, H - MB + 34), line1, font=FONT_S, fill=(40, 40, 40))
    dr.text((ML, H - MB + 54), line2, font=FONT_S, fill=(40, 40, 40))

    img.save(out)


def main() -> int:
    snap = build_snapshot()
    route = plan_route(snap, cell(START), cell(GOAL),
                       start_heading="y", final_heading="y")
    note = [
        "地图：裁判系统随机图（2026-10-15 截图）",
        f"雷 {len(MINES)} / 障 {len(OBSTACLES)} / 隧道口 {','.join(PASSABLE)}",
        "规划器：合并后 mb=6 版（上游加固+重构汇流）",
        "穿越走西区走廊；桥体格不可踩",
    ]
    out = Path(__file__).resolve().parent / "referee_route.png"
    render(snap, route, out, note)
    print("路径:", " → ".join(route))
    print(f"共 {len(route)} 格；渲染: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
