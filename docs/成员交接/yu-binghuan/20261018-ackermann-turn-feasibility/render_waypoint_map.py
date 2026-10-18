# -*- coding: utf-8 -*-
"""根据 route_plan.json 渲染航点图（压缩航点带序号 + 车端命令）。

用法：
  python render_waypoint_map.py 视频侦察_20261018/cam_20261017_205906 [--out 视频侦察_20261018/cam_xxx/waypoint_map.png]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

BASE = Path(__file__).resolve().parent
LETTERS = "ABCDEFGHIJKLMN"
COLS, ROWS = 14, 11
BRIDGE_COL = "K"


def cell(name: str) -> tuple[int, int]:
    """格号 -> (col, row)。"""
    return LETTERS.index(name[0]), int(name[1:]) - 1


def xy(c: tuple[int, int], cell_px: int, pad: int) -> tuple[int, int]:
    return pad + c[0] * cell_px, pad + c[1] * cell_px


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("plan_json", type=str, help="route_plan.json 路径或所在目录")
    ap.add_argument("--out", type=str, default="", help="输出 png 路径（默认与 plan_json 同目录 waypoint_map.png）")
    args = ap.parse_args()

    p = Path(args.plan_json)
    plan_file = p if p.suffix == ".json" else p / "route_plan.json"
    root = plan_file.parent
    plan = json.loads(plan_file.read_text(encoding="utf-8"))

    snap = plan["snapshot"]
    src = plan["source"]
    route = plan["route"]
    compact = plan["compact"]
    cmds = plan["car_cmds"]
    mines: list[str] = snap.get("mines", [])
    obstacles: list[str] = snap.get("obstacles", [])
    passable: list[str] = snap.get("passable", [])
    targets: list[str] = snap.get("targets", [])   # e.g. "蓝@D8"
    baseline_only: list[str] = plan.get("evidence", {}).get("mines_baseline_only", [])
    start = src.get("start", "N11")
    goal = src.get("goal", "N6")

    # ---------- 画布 ----------
    CELL_PX, PAD = 60, 50
    W = PAD * 2 + COLS * CELL_PX
    H = PAD * 2 + ROWS * CELL_PX
    img = Image.new("RGB", (W, H + 96), (30, 30, 30))
    dr = ImageDraw.Draw(img)
    try:
        f16 = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 16)
        f12 = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 12)
        f20 = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 20)
    except Exception:
        f16 = f12 = f20 = ImageFont.load_default()

    # 网格
    for r in range(ROWS + 1):
        dr.line([(PAD, PAD + r * CELL_PX), (PAD + COLS * CELL_PX, PAD + r * CELL_PX)], (80, 80, 80))
    for c in range(COLS + 1):
        dr.line([(PAD + c * CELL_PX, PAD), (PAD + c * CELL_PX, PAD + ROWS * CELL_PX)], (80, 80, 80))
    for c in range(COLS):
        dr.text((PAD + c * CELL_PX + CELL_PX // 2, PAD - 20), LETTERS[c], (200, 200, 200), f12, "mm")
    for r in range(ROWS):
        dr.text((PAD - 20, PAD + r * CELL_PX + CELL_PX // 2), str(r + 1), (200, 200, 200), f12, "mm")

    # 桥列 K：洞(可通行) / 墙
    for r in range(ROWS):
        name = f"{BRIDGE_COL}{r + 1}"
        x, y = xy(cell(name), CELL_PX, PAD)
        if name in passable:
            dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (80, 60, 120))
            dr.text((x + CELL_PX // 2, y + CELL_PX // 2), "洞", (255, 255, 255), f16, "mm")
        else:
            dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (60, 60, 60))
            dr.text((x + CELL_PX // 2, y + CELL_PX // 2), "墙", (150, 150, 150), f12, "mm")

    # 障碍 / 雷
    for name in obstacles:
        x, y = xy(cell(name), CELL_PX, PAD)
        dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (180, 40, 40))
        dr.text((x + CELL_PX // 2, y + CELL_PX // 2), "障", (255, 255, 255), f16, "mm")
    for name in mines:
        x, y = xy(cell(name), CELL_PX, PAD)
        dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (220, 200, 30))
        dr.text((x + CELL_PX // 2, y + CELL_PX // 2 - 6), "雷", (0, 0, 0), f16, "mm")
        dr.text((x + CELL_PX // 2, y + CELL_PX - 10),
                "基线" if name in baseline_only else "✓拍",
                (90, 60, 0) if name in baseline_only else (20, 90, 20), f12, "mm")

    # 靶（蓝/红/未辨）
    for t in targets:
        tag, _, name = t.partition("@")
        x, y = xy(cell(name), CELL_PX, PAD)
        tag = tag or t
        bg = (95, 75, 140) if tag == "蓝" else ((190, 90, 90) if tag == "红" else (110, 110, 120))
        dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], bg)
        dr.text((x + CELL_PX // 2, y + CELL_PX // 2), tag, (255, 255, 255), f16, "mm")

    # 起终点
    sx, sy = xy(cell(start), CELL_PX, PAD)
    gx, gy = xy(cell(goal), CELL_PX, PAD)
    dr.rectangle([sx + 4, sy + 4, sx + CELL_PX - 4, sy + CELL_PX - 4], outline=(0, 255, 0), width=3)
    dr.text((sx + CELL_PX // 2, sy + CELL_PX - 8), "起点", (0, 255, 0), f12, "mm")
    dr.rectangle([gx + 4, gy + 4, gx + CELL_PX - 4, gy + CELL_PX - 4], outline=(255, 100, 0), width=3)
    dr.text((gx + CELL_PX // 2, gy + CELL_PX - 8), "终点", (255, 100, 0), f12, "mm")

    # 航点折线 + 序号
    pts = [xy(cell(n), CELL_PX, PAD) for n in compact]
    for i in range(len(pts) - 1):
        x1, y1 = pts[i]
        x2, y2 = pts[i + 1]
        t = i / max(len(pts) - 2, 1)
        col = (int(120 * t * 2 + 40), 220, 80) if t < 0.5 else (255, int(220 * (1 - (t - 0.5) * 2)), 80)
        dr.line([(x1 + CELL_PX // 2, y1 + CELL_PX // 2), (x2 + CELL_PX // 2, y2 + CELL_PX // 2)], col, 4)
    for i, n in enumerate(compact):
        x, y = xy(cell(n), CELL_PX, PAD)
        cx, cy = x + CELL_PX // 2, y + CELL_PX // 2
        t = i / max(len(compact) - 1, 1)
        col = (int(120 * t * 2 + 40), 220, 80) if t < 0.5 else (255, int(220 * (1 - (t - 0.5) * 2)), 80)
        dr.ellipse([cx - 13, cy - 13, cx + 13, cy + 13], fill=col, outline=(255, 255, 255))
        dr.text((cx, cy), str(i + 1), (0, 0, 0), f12, "mm")

    # 射击格 ⊕
    for name in cmds.get("shoot_cells", []):
        x, y = xy(cell(name), CELL_PX, PAD)
        cx, cy = x + CELL_PX // 2, y + CELL_PX // 2
        dr.ellipse([cx - 19, cy - 19, cx + 19, cy + 19], outline=(255, 200, 0), width=2)
        dr.line([(cx - 9, cy - 9), (cx + 9, cy + 9)], (255, 200, 0), 2)
        dr.line([(cx - 9, cy + 9), (cx + 9, cy - 9)], (255, 200, 0), 2)
        dr.text((cx, cy - 8), "射", (255, 200, 0), f12, "mm")

    # 底部信息
    y0 = H + 6
    dr.text((PAD, y0), f"航点图 · {plan_file.parent.name}  （{len(compact)} 航点 / 路线 {len(route)} 格）", (235, 235, 235), f20)
    dr.text((PAD, y0 + 30), f"reset: {cmds.get('reset', '')}", (200, 255, 200), f12)
    route_txt = f"route: {cmds.get('route', '')}"
    dr.text((PAD, y0 + 50), route_txt, (200, 200, 200), f12)
    dr.text((PAD, y0 + 70), "①…⑳=航点顺序  雷/障/洞/靶=场地态势  ⊕=停靠射击格", (255, 230, 130), f12)

    out = Path(args.out) if args.out else root / "waypoint_map.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out)
    print(f"[waypoint] {out}")
    print(f"[waypoint] 航点: {' → '.join(compact)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
