"""半自动补标：HSV 颜色分割 + 格号凸包空间约束，补出桥洞与橙格草稿框。

官方 hbb 模型不含「黄色桥洞」「起点/终点橙格」，人工从零标很贵。本脚本利用
两个场地事实自动生成草稿：
    1. 桥洞（亮黄）和橙格（橙色）颜色独特，HSV 可分割；
    2. 它们都位于场地地垫范围内。已检测的格号（类 1 cell_label）框中心的
       凸包 ≈ 地垫范围；地砖虽被灯光染黄，但在凸包外 → 重叠率过滤即可排除。
输出追加到 convert_draft_labels.py 生成的 labels/：
    3 bridge       桥洞整体一个框（识别"桥在这里"，不逐节）
    12 orange_raw  橙格，人工在标注工具里细分为 4 car_start / 5 car_end /
                   6 uav_start / 7 uav_end

用法：
    python tools/ml/auto_supplement.py --images capture/aerial_field/images \\
        --labels capture/aerial_field/labels
退出码：0 成功；2 参数/依赖。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("需要 opencv-python / numpy", file=sys.stderr)
    raise SystemExit(2)


BRIDGE_CLS = 3
ORANGE_TMP_CLS = 12


# ---------------------------------------------------------------- 纯函数（可测）

def hull_mask(points: list[tuple[int, int]], h: int, w: int,
              expand_px: int = 0):
    """格号中心点 → 地垫凸包 mask；点数不足返回 None。

    expand_px>0 时向外膨胀（格号文字印在格子左缘，裸凸包够不到格子另一侧，
    会把贴边的橙格误删）。
    """
    if len(points) < 3:
        return None
    hull = cv2.convexHull(np.array(points, dtype=np.int32))
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillConvexPoly(mask, hull, 1)
    if expand_px > 0:
        k = np.ones((expand_px * 2 + 1,) * 2, np.uint8)
        mask = cv2.dilate(mask, k)
    return mask


def median_spacing(points: list[tuple[int, int]]) -> float:
    """格号中心两两距离（>30px）的中位数，作为典型格距。"""
    distances = []
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            d = float(np.linalg.norm(np.array(points[i]) - np.array(points[j])))
            if d > 30:
                distances.append(d)
    return float(np.median(distances)) if distances else 200.0


def contour_overlap(contour, region_mask, h: int, w: int) -> float:
    """轮廓与区域 mask 的像素重叠比例（占轮廓自身面积）。"""
    cm = np.zeros((h, w), dtype=np.uint8)
    cv2.fillConvexPoly(cm, contour, 1)
    area = max(int(cm.sum()), 1)
    return float((cm & region_mask).sum()) / area


def colored_masks(hsv):
    """返回 (orange_mask, yellow_mask)；黄色中已扣除橙色区域。"""
    orange = cv2.inRange(hsv, np.array((18, 100, 95)),
                         np.array((30, 255, 200)))
    orange = cv2.morphologyEx(orange, cv2.MORPH_CLOSE,
                              np.ones((40, 40), np.uint8), iterations=1)
    orange_big = cv2.dilate(orange, np.ones((30, 30), np.uint8))

    yellow = cv2.inRange(hsv, np.array((30, 90, 65)),
                         np.array((42, 255, 220)))
    yellow = cv2.bitwise_and(yellow, cv2.bitwise_not(orange_big))
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE,
                              np.ones((40, 40), np.uint8), iterations=2)
    yellow = cv2.dilate(yellow, np.ones((20, 20), np.uint8))
    return orange, yellow


def _normalized(cx, cy, bw, bh, h, w) -> str:
    return (f"{cx / w:.6f} {cy / h:.6f} "
            f"{min(bw / w, 1):.6f} {min(bh / h, 1):.6f}")


def entrance_from_arrow(arrow: list[float], expand: float = 3.0) -> str:
    """桥洞入口框 = 绿色箭头框中心、尺寸外扩 expand 倍（用户确认只框入口段）。

    arrow：[cx, cy, w, h] 归一化。入口单节桥段约为箭头贴标的 3 倍。
    箭头贴着画面边缘时框会越界 → 中心平移夹回画面内（尺寸不变）。
    """
    cx, cy, aw, ah = arrow
    bw = min(aw * expand, 1)
    bh = min(ah * expand, 1)
    cx = min(max(cx, bw / 2), 1 - bw / 2)
    cy = min(max(cy, bh / 2), 1 - bh / 2)
    return f"{BRIDGE_CLS} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}"


def supplement(image, existing_lines: list[str], add_orange: bool = True):
    """对一帧生成补标行。返回 (新标签行列表, info)。

    existing_lines：该帧已转换的标签（类 1 cell_label 用于凸包、类 2 arrow
    用于定位桥洞入口）。add_orange=False（repair-bridge）：橙格已处理，跳过。
    """
    h, w = image.shape[:2]
    points, arrows = [], []
    for line in existing_lines:
        v = list(map(float, line.split()))
        if int(v[0]) == 1:
            points.append((int(v[1] * w), int(v[2] * h)))
        elif int(v[0]) == 2:
            arrows.append(v[1:5])
    spacing = median_spacing(points) if len(points) >= 2 else 200.0
    # 橙格在边列，需大膨胀凸包才够到格子外侧。
    region = hull_mask(points, h, w, expand_px=int(spacing * 0.9))

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    orange_m, _yellow_m = colored_masks(hsv)

    new_lines = []
    info = {"orange": 0, "bridge": 0, "no_hull": region is None}

    # 橙格：本身定位准；有凸包时要求轻度重叠，无凸包时放宽
    if add_orange:
        for contour in cv2.findContours(orange_m, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)[0]:
            x, y, bw, bh_ = cv2.boundingRect(contour)
            if bw * bh_ < w * h * 0.0025:
                continue
            if not (0.35 < bw / max(bh_, 1) < 2.8):
                continue
            if region is not None and \
                    contour_overlap(contour, region, h, w) < 0.25:
                continue
            new_lines.append(f"{ORANGE_TMP_CLS} " + _normalized(
                x + bw / 2, y + bh_ / 2, bw, bh_, h, w))
            info["orange"] += 1

    # 桥洞入口：由箭头标定位（一个箭头 → 一个入口段），不再分割整条桥
    for arrow in arrows:
        new_lines.append(entrance_from_arrow(arrow))
        info["bridge"] += 1

    return new_lines, info


# ---------------------------------------------------------------- 主流程

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", required=True)
    parser.add_argument("--labels", required=True)
    parser.add_argument("--draw-dir")
    parser.add_argument("--repair-bridge", action="store_true",
                        help="原地重建：先删除已有桥洞(类3)行再重算，橙格不动")
    args = parser.parse_args()

    images_dir, labels_dir = Path(args.images), Path(args.labels)
    draw_dir = Path(args.draw_dir) if args.draw_dir else None
    if draw_dir:
        draw_dir.mkdir(parents=True, exist_ok=True)

    totals = {"orange": 0, "bridge": 0, "no_hull": 0}
    for image_path in sorted(images_dir.glob("*.jpg")):
        image = cv2.imread(str(image_path))
        label_path = labels_dir / (image_path.stem + ".txt")
        existing = label_path.read_text(
            encoding="utf-8").splitlines() if label_path.exists() else []
        if args.repair_bridge:
            filtered = [line for line in existing
                        if int(line.split()[0]) != BRIDGE_CLS]
            removed = len(existing) - len(filtered)
            existing = filtered
        else:
            removed = 0
        new_lines, info = supplement(
            image, existing, add_orange=not args.repair_bridge)
        for key in totals:
            totals[key] += int(info[key])
        # repair 模式下即使重算为空也要写回（旧行已删）；否则残留旧桥洞框
        if new_lines or removed:
            label_path.write_text(
                "\n".join(existing + new_lines) + "\n", encoding="utf-8")
        if draw_dir:
            vis = image.copy()
            for line in existing + new_lines:
                v = list(map(float, line.split()))
                cls = int(v[0])
                if cls not in (BRIDGE_CLS, ORANGE_TMP_CLS):
                    continue
                h, w = image.shape[:2]
                cx, cy, bw, bh_ = v[1] * w, v[2] * h, v[3] * w, v[4] * h
                color = (0, 255, 255) if cls == BRIDGE_CLS else (0, 128, 255)
                cv2.rectangle(vis, (int(cx - bw / 2), int(cy - bh_ / 2)),
                              (int(cx + bw / 2), int(cy + bh_ / 2)), color, 6)
            cv2.imwrite(str(draw_dir / image_path.name), vis)

    print(f"补桥洞 {totals['bridge']} 个，补橙格 {totals['orange']} 个；"
          f"{totals['no_hull']} 帧因格号点不足未补桥洞（需人工）")
    if draw_dir:
        print(f"可视化 → {draw_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
