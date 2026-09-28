#!/usr/bin/env python3
"""单应矩阵标定：四角像素 -> 场地米制。

现场流程：
    1. 无人机升到比赛高度，拍一张能看全 A1/K1/K14/A14 四角的图
    2. 从图里读出四角像素坐标（顺序必须是 A1 -> K1 -> K14 -> A14）
    3. 运行本脚本，得到 9 个数，填进 config/params.yaml 的 homography
    4. 残差 > 5 cm 说明点位不准，重标

用法：
    python tools/calib_homography.py --corners "120,80 980,95 965,700 105,690"
    python tools/calib_homography.py --corners "..." --check "500,400"
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
# 每个包目录都要入 path（src/<pkg>/<pkg>/ 两层布局），与 tests/conftest.py 一致
for _p in [SRC] + sorted(p for p in SRC.iterdir() if p.is_dir()):
    sys.path.insert(0, str(_p))

from ag_core.grid_map import FieldGrid  # noqa: E402
from ag_core.homography import (  # noqa: E402
    CORNER_ORDER,
    field_corners,
    homography_from_correspondences,
    reprojection_error,
)


def parse_points(text: str) -> np.ndarray:
    pts = []
    for chunk in text.replace(";", " ").split():
        u, v = chunk.split(",")
        pts.append([float(u), float(v)])
    return np.array(pts, dtype=np.float64)


def main() -> int:
    ap = argparse.ArgumentParser(description="四角像素 -> 单应矩阵标定")
    ap.add_argument("--corners", required=True,
                    help=f'四角像素坐标 "u,v u,v u,v u,v"，顺序 {CORNER_ORDER}')
    ap.add_argument("--cols", type=int, default=11)
    ap.add_argument("--rows", type=int, default=14)
    ap.add_argument("--cell-size", type=float, default=0.35)
    ap.add_argument("--check", default="", help="可选：验证某个像素点落在哪个格号")
    args = ap.parse_args()

    src = parse_points(args.corners)
    if src.shape != (4, 2):
        raise SystemExit(f"需要 4 组坐标，当前 {src.shape[0]} 组")

    dst = field_corners(args.cols, args.rows, args.cell_size)
    H = homography_from_correspondences(src, dst)
    err = reprojection_error(H, src, dst)

    flat = ", ".join(f"{v:.8f}" for v in H.flatten())
    print("四角顺序: " + " -> ".join(CORNER_ORDER))
    print(f"场地尺寸: {args.cols} 列 x {args.rows} 行 x {args.cell_size} m "
          f"= {args.cols * args.cell_size:.2f} x {args.rows * args.cell_size:.2f} m")
    print(f"重投影残差: {err * 100:.2f} cm")
    if err > 0.05:
        print("  残差偏大，检查四角点位是否点错或顺序是否反了")

    print("\nhomography (params.yaml):")
    print(f"    homography: [{flat}]")

    if args.check:
        u, v = parse_points(args.check)[0]
        grid = FieldGrid(cols=args.cols, rows=args.rows, cell_size=args.cell_size)
        grid.set_homography(H)
        cid = grid.pixel_to_cell(float(u), float(v))
        x, y = grid.pixel_to_field(float(u), float(v))
        print(f"\n检查点 ({u:.0f}, {v:.0f}) -> 场地 ({x:.2f}, {y:.2f}) m -> 格号 {cid.name if cid else '界外'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
