# -*- coding: utf-8 -*-
"""根据上传的场地图片态势，生成路径规划并渲染为带路径的场地图。"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np

from src.ag_integrate.ag_integrate.field_frame import CellRef, COLS, ROWS
from src.ag_integrate.ag_integrate.world_model import WorldSnapshot, TargetRef
from src.ag_integrate.ag_integrate.mission_router import plan_route


def cell(name: str) -> CellRef:
    """格号字符串 -> CellRef。例如 'A2' -> CellRef(0, 1)。"""
    col = ord(name[0].upper()) - ord('A')
    row = int(name[1:]) - 1
    return CellRef(col, row)


def build_snapshot() -> WorldSnapshot:
    """从图片解析的场地态势。"""
    mines = [cell(n) for n in [
        "A2", "I2", "E5", "E6", "F7", "H7", "B9", "H9",
    ]]
    passable = [cell(n) for n in ["K3", "K7"]]
    obstacles = [cell(n) for n in [
        "A3", "A4", "G4", "J6", "I8", "C10", "E10", "D11",
    ]]
    # 靶 B1 C1（蓝面=敌方）
    targets = [
        TargetRef(cell("B1"), is_blue=True),
        TargetRef(cell("C1"), is_blue=True),
    ]
    return WorldSnapshot(0, tuple(mines), tuple(passable), tuple(obstacles), tuple(targets))


def cell_xy(cell: CellRef, cell_px: int = 48, margin: int = 40) -> tuple[int, int]:
    """格子中心点像素坐标。"""
    x = margin + cell.col * cell_px + cell_px // 2
    y = margin + cell.row * cell_px + cell_px // 2
    return x, y


def render(snapshot: WorldSnapshot, route: tuple[str, ...], out_path: str) -> None:
    cell_px = 48
    margin = 40
    W = margin * 2 + COLS * cell_px
    H = margin * 2 + ROWS * cell_px
    img = np.full((H, W, 3), 250, np.uint8)

    # 网格
    for r in range(ROWS + 1):
        y = margin + r * cell_px
        cv2.line(img, (margin, y), (margin + COLS * cell_px, y), (200, 200, 200), 1)
    for c in range(COLS + 1):
        x = margin + c * cell_px
        cv2.line(img, (x, margin), (x, margin + ROWS * cell_px), (200, 200, 200), 1)

    # 列字母 / 行号
    for c in range(COLS):
        x = margin + c * cell_px + cell_px // 2
        cv2.putText(img, chr(ord('A') + c), (x - 6, margin - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)
    for r in range(ROWS):
        y = margin + r * cell_px + cell_px // 2 + 4
        cv2.putText(img, str(r + 1), (margin - 24, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)

    # 中场横向挡板（第6/7行边界，D~N列，黄色半透明条带）
    boundary_y = margin + 6 * cell_px
    x_start = margin + 3 * cell_px   # D列开始
    x_end = margin + COLS * cell_px  # N列结束
    cv2.rectangle(img, (x_start, boundary_y - cell_px // 4),
                  (x_end, boundary_y + cell_px // 4),
                  (0, 200, 255), -1)

    # 障碍物（红）
    for c in snapshot.obstacles:
        x, y = cell_xy(c, cell_px, margin)
        cv2.rectangle(img, (x - cell_px // 2 + 2, y - cell_px // 2 + 2),
                      (x + cell_px // 2 - 2, y + cell_px // 2 - 2), (0, 0, 200), -1)
        cv2.putText(img, "障", (x - 10, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # 桥洞可通行（绿）
    for c in snapshot.passable:
        x, y = cell_xy(c, cell_px, margin)
        cv2.rectangle(img, (x - cell_px // 2 + 2, y - cell_px // 2 + 2),
                      (x + cell_px // 2 - 2, y + cell_px // 2 - 2), (0, 180, 0), -1)
        cv2.putText(img, "通", (x - 10, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # 雷（橙）
    for c in snapshot.mines:
        x, y = cell_xy(c, cell_px, margin)
        cv2.circle(img, (x, y), cell_px // 3, (0, 140, 255), -1)
        cv2.putText(img, "雷", (x - 10, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    # 靶（粉）
    for t in snapshot.targets:
        x, y = cell_xy(t.cell, cell_px, margin)
        cv2.rectangle(img, (x - cell_px // 2 + 2, y - cell_px // 2 + 2),
                      (x + cell_px // 2 - 2, y + cell_px // 2 - 2), (180, 105, 255), -1)
        cv2.putText(img, "靶", (x - 10, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # 起点 / 终点
    start = cell("N10")
    goal = cell("N5")
    sx, sy = cell_xy(start, cell_px, margin)
    gx, gy = cell_xy(goal, cell_px, margin)
    cv2.circle(img, (sx, sy), cell_px // 3, (255, 0, 0), -1)
    cv2.putText(img, "S", (sx - 6, sy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.circle(img, (gx, gy), cell_px // 3, (0, 0, 255), -1)
    cv2.putText(img, "G", (gx - 6, gy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # 路径线
    route_cells = [cell(n) for n in route]
    pts = [cell_xy(c, cell_px, margin) for c in route_cells]
    for i in range(len(pts) - 1):
        cv2.line(img, pts[i], pts[i + 1], (255, 0, 0), 3)
    for x, y in pts:
        cv2.circle(img, (x, y), 3, (0, 0, 255), -1)

    # 标题
    cv2.putText(img, f"Route: {' -> '.join(route)}",
                (margin, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

    cv2.imwrite(out_path, img)
    print(f"渲染完成: {out_path}")
    print(f"路径长度: {len(route)} 格")
    print(f"路径: {' -> '.join(route)}")


def main() -> int:
    snapshot = build_snapshot()
    start = cell("N10")
    goal = cell("N5")
    route = plan_route(snapshot, start, goal)
    out = "runs/capture_dataset/route_plan.png"
    render(snapshot, route, out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
