#!/usr/bin/env python3
"""官方爆炸物图集：X-AnyLabeling(labelme) JSON -> YOLO txt 转换 + 校验。

用法：
  python tools/ml/convert_mine_official.py --check    # 只检查不写入
  python tools/ml/convert_mine_official.py            # 检查 + 转换写入

输出 YOLO 格式（class 0 = mine）：
  <class> <x_center/w> <y_center/h> <box_w/w> <box_h/h>

注意历史教训：x/w 用 imageWidth，y/h 用 imageHeight，严禁混用。
空标注图生成空 txt（保留为背景负样本）。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "capture" / "mine_official" / "images"
DST = ROOT / "datasets" / "mine_official"


def check_and_convert(write: bool) -> None:
    jsons = sorted(SRC.glob("*.json"))
    if not jsons:
        raise SystemExit(f"no json found in {SRC}")

    total_boxes = 0
    empty = 0
    problems: list[str] = []
    sizes: dict[tuple[int, int], int] = {}

    if write:
        (DST / "labels").mkdir(parents=True, exist_ok=True)

    for jp in jsons:
        data = json.loads(jp.read_text(encoding="utf-8"))
        img_path = SRC / data.get("imagePath", jp.stem + ".jpg")
        if not img_path.exists():
            problems.append(f"{jp.name}: imagePath 缺失 {img_path.name}")
            continue

        # json 声明尺寸 vs 实际图片尺寸
        with Image.open(img_path) as im:
            real_w, real_h = im.size
        jw, jh = int(data["imageWidth"]), int(data["imageHeight"])
        sizes[(jw, jh)] = sizes.get((jw, jh), 0) + 1
        if (jw, jh) != (real_w, real_h):
            problems.append(f"{jp.name}: json 尺寸 {jw}x{jh} != 实际 {real_w}x{real_h}")

        lines: list[str] = []
        for i, s in enumerate(data.get("shapes", [])):
            if s.get("label") != "mine":
                problems.append(f"{jp.name}: shape{i} 标签 {s.get('label')!r} != 'mine'")
                continue
            if s.get("shape_type") != "rectangle":
                problems.append(f"{jp.name}: shape{i} 类型 {s.get('shape_type')!r} != rectangle")
                continue
            pts = s["points"]
            if len(pts) < 2:
                problems.append(f"{jp.name}: shape{i} 点数 {len(pts)} < 2")
                continue
            # rectangle: 取所有点的 min/max（兼容 2 点 / 4 点表示）
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
            w = x1 - x0
            h = y1 - y0
            if w <= 0 or h <= 0:
                problems.append(f"{jp.name}: shape{i} 零尺寸框")
                continue
            if x0 < 0 or y0 < 0 or x1 > jw or y1 > jh:
                problems.append(f"{jp.name}: shape{i} 框越界 ({x0:.0f},{y0:.0f})-({x1:.0f},{y1:.0f})")
            # YOLO 归一化：x/w 按 imageWidth，y/h 按 imageHeight
            cx = (x0 + x1) / 2 / jw
            cy = (y0 + y1) / 2 / jh
            bw = w / jw
            bh = h / jh
            if not (0 < cx < 1 and 0 < cy < 1 and 0 < bw <= 1 and 0 < bh <= 1):
                problems.append(f"{jp.name}: shape{i} 归一化异常 ({cx:.3f},{cy:.3f},{bw:.3f},{bh:.3f})")
                continue
            lines.append(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")

        if not lines:
            empty += 1
        total_boxes += len(lines)

        if write:
            (DST / "labels" / (jp.stem + ".txt")).write_text(
                "\n".join(lines) + ("\n" if lines else ""), encoding="utf-8"
            )

    print(f"json 总数: {len(jsons)}")
    print(f"标注框总数: {total_boxes}（平均 {total_boxes / len(jsons):.2f} 框/图）")
    print(f"空标注图: {empty}（将作为背景负样本保留）")
    print(f"json 声明尺寸分布: {sizes}")
    if problems:
        print(f"\n发现 {len(problems)} 个问题:")
        for p in problems:
            print(f"  - {p}")
    else:
        print("校验通过，无问题。")
    if write:
        print(f"\n已写入 {DST / 'labels'}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="只检查，不写入")
    args = parser.parse_args()
    check_and_convert(write=not args.check)
