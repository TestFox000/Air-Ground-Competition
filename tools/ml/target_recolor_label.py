# -*- coding: utf-8 -*-
"""用HSV颜色分割自动标注标靶（红桶=red, 蓝桶=blue）

best(1).onnx在手机俯拍视频上召回太低，改用颜色分割：
  red:  红色桶   → cls=1
  blue: 蓝色桶   → cls=0
  (绿桶是障碍物，不标注)

靶心关键点 = 检测框中心点。
"""
import argparse
from pathlib import Path

import cv2
import numpy as np


def detect_color_buckets(img_bgr):
    """返回 [(cls, x1,y1,x2,y2, cx,cy), ...]，坐标为像素值。"""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    H, W = img_bgr.shape[:2]
    dets = []

    # 红色：HSV中红跨0°和180°，需两段
    red_mask1 = cv2.inRange(hsv, (0, 100, 80), (10, 255, 255))
    red_mask2 = cv2.inRange(hsv, (160, 100, 80), (180, 255, 255))
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    # 蓝色
    blue_mask = cv2.inRange(hsv, (90, 100, 80), (135, 255, 255))

    for cls, mask in [(1, red_mask), (0, blue_mask)]:
        # 形态学清理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # 连通域
        n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        for i in range(1, n_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < 800:  # 过滤小噪点
                continue
            x, y, w, h = stats[i, :4]
            # 过滤太扁或太细长的（桶应该近似正方形）
            ratio = w / max(h, 1)
            if ratio < 0.3 or ratio > 3.0:
                continue
            cx, cy = centroids[i]
            dets.append((cls, int(x), int(y), int(x + w), int(y + h), float(cx), float(cy)))
    return dets


def to_yolo_pose(W, H, det):
    cls, x1, y1, x2, y2, kx, ky = det
    cx, cy = (x1 + x2) / 2 / W, (y1 + y2) / 2 / H
    w, h = (x2 - x1) / W, (y2 - y1) / H
    return f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} {kx/W:.6f} {ky/H:.6f} 2"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", default=r"E:\ml_datasets\target_raw\images")
    ap.add_argument("--labels", default=r"E:\ml_datasets\target_raw\labels")
    ap.add_argument("--preview", default=r"E:\ml_datasets\target_raw\preview")
    args = ap.parse_args()

    img_dir = Path(args.images)
    lbl_dir = Path(args.labels)
    prev_dir = Path(args.preview)

    n_total = n_red = n_blue = 0
    for img_path in sorted(img_dir.glob("*.jpg")):
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        H, W = img.shape[:2]
        dets = detect_color_buckets(img)
        n_total += len(dets)
        n_red += sum(1 for d in dets if d[0] == 1)
        n_blue += sum(1 for d in dets if d[0] == 0)
        # 写标签
        with open(lbl_dir / (img_path.stem + ".txt"), "w") as f:
            for d in dets:
                f.write(to_yolo_pose(W, H, d) + "\n")
        # 重画预览
        prev = img.copy()
        for d in dets:
            cls, x1, y1, x2, y2, kx, ky = d
            c = (0, 0, 255) if cls == 1 else (255, 0, 0)
            cv2.rectangle(prev, (x1, y1), (x2, y2), c, 3)
            cv2.circle(prev, (int(kx), int(ky)), 6, (0, 255, 0), -1)
        cv2.imwrite(str(prev_dir / img_path.name), prev)

    print(f"标注完成: 总框={n_total} red={n_red} blue={n_blue}")
    print(f"预览已更新到 {prev_dir}")


if __name__ == "__main__":
    main()
