#!/usr/bin/env python3
"""数据集加工：原始图片 + 标注 -> 可直接训练的 YOLO(-pose) 数据集。

    python tools/build_dataset.py --images 原始/images --labels 原始/labels \
        --out dataset --classes "blue_glowing_target,red_glowing_target" \
        --pose --val-ratio 0.2

做的事：
    1. 读取标注（YOLO detect 5 列 / YOLO-pose 8 列）
    2. 若为 pose 且标签没有关键点：自动用「发光中心」算法补（大津法 -> 最大轮廓 -> 外接圆圆心，
       与你桌上 auto_center.py 同一套思路）；没有 cv2 时退回框中心并告警
    3. 清洗：空标签、越界框、非法类别、图片与标签不匹配
    4. 划分 train/val，生成目录与 data.yaml
    5. 打印统计报告（每类实例数、负样本占比、可疑标注）

只做文件整理，不依赖 torch；生成关键点需要 cv2（可选）。
"""

from __future__ import annotations

import argparse
import random
import shutil
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Tuple

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".jpg"}
DEFAULT_CLASSES = ["blue_glowing_target", "red_glowing_target"]


# ------------------------------------------------------------------ 读取
def read_label(path: Path) -> List[List[float]]:
    rows = []
    if not path.is_file():
        return rows
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        try:
            rows.append([float(x) for x in parts])
        except ValueError:
            continue
    return rows


def width_of(row: List[float]) -> int:
    """5 列 = detect，>=6 列 = pose（每多一个关键点多 3 列）。"""
    return 5 if len(row) == 5 else 4 + 3 * ((len(row) - 4) // 3)


def is_pose(row: List[float]) -> bool:
    return len(row) >= 8


# ------------------------------------------------------------------ 关键点
def make_kpt_auto(img_path: Path, box: Tuple[float, float, float, float]) -> Tuple[float, float]:
    """在框内找发光中心，返回归一化 (kx, ky)。失败返回框中心。"""
    cx, cy, bw, bh = box
    try:
        import cv2
        import numpy as np
    except Exception:
        return cx, cy

    img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        return cx, cy
    h, w = img.shape[:2]
    x1 = max(0, int((cx - bw / 2) * w))
    y1 = max(0, int((cy - bh / 2) * h))
    x2 = min(w, int((cx + bw / 2) * w))
    y2 = min(h, int((cy + bh / 2) * h))
    roi = img[y1:y2, x1:x2]
    if roi.size == 0:
        return cx, cy

    _, th = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return cx, cy
    cnt = max(contours, key=cv2.contourArea)
    (mx, my), radius = cv2.minEnclosingCircle(cnt)
    if radius < 3:
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            return cx, cy
        mx, my = M["m10"] / M["m00"], M["m01"] / M["m00"]
    return (x1 + mx) / w, (y1 + my) / h


# ------------------------------------------------------------------ 校验
def validate(row: List[float], ncls: int) -> Optional[str]:
    if len(row) < 5:
        return f"列数不足: {len(row)}"
    cls = int(row[0])
    if cls < 0 or cls >= ncls:
        return f"类别越界: {cls}"
    cx, cy, bw, bh = row[1:5]
    for v in (cx, cy, bw, bh):
        if not (0.0 <= v <= 1.5):
            return f"坐标越界: {row[1:5]}"
    if bw <= 0 or bh <= 0:
        return "宽高非正"
    return None


# ------------------------------------------------------------------ 主流程
def build(args) -> int:
    images_dir = Path(args.images)
    labels_dir = Path(args.labels)
    out = Path(args.out)
    classes = [c.strip() for c in args.classes.split(",") if c.strip()] or DEFAULT_CLASSES

    imgs = sorted(p for p in images_dir.rglob("*") if p.suffix.lower() in IMG_EXTS)
    if not imgs:
        print(f"没找到图片: {images_dir}")
        return 1

    kept: List[Tuple[Path, List[List[float]]]] = []
    dropped: List[Tuple[str, str]] = []
    cls_counter: Counter = Counter()
    empty_count = 0

    for img in imgs:
        rel = img.relative_to(images_dir)
        lab = labels_dir / rel.with_suffix(".txt")
        if not lab.is_file():
            lab = labels_dir / (img.stem + ".txt")
        rows = read_label(lab)

        clean = []
        for r in rows:
            err = validate(r, len(classes))
            if err:
                dropped.append((str(lab), err))
                continue
            cls_counter[int(r[0])] += 1
            clean.append(r)
        if not clean:
            empty_count += 1
            if not args.keep_empty:
                continue
        kept.append((img, clean))

    if not kept:
        print("清洗后没有可用样本，检查标注路径与格式")
        return 1

    # pose：补关键点
    if args.pose:
        filled = 0
        for img, rows in kept:
            for r in rows:
                if is_pose(r):
                    continue
                kx, ky = make_kpt_auto(img, (r[1], r[2], r[3], r[4]))
                r.extend([kx, ky, 2.0])
                filled += 1
        if filled:
            print(f"自动补关键点: {filled} 个目标")

    # 划分
    random.Random(args.seed).shuffle(kept)
    n_val = max(1, int(len(kept) * args.val_ratio)) if len(kept) > 1 else 0
    val_set, train_set = kept[:n_val], kept[n_val:]

    def dump(split: str, items):
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)
        for img, rows in items:
            dst_img = out / "images" / split / img.name
            if args.copy:
                shutil.copy2(img, dst_img)
            else:
                try:
                    dst_img.symlink_to(img.resolve())
                except (OSError, NotImplementedError):
                    shutil.copy2(img, dst_img)
            txt = out / "labels" / split / (img.stem + ".txt")
            txt.write_text(
                "\n".join(" ".join(f"{v:.6f}" if isinstance(v, float) else str(int(v))
                                   for v in r) for r in rows),
                encoding="utf-8",
            )

    for d in (out / "images" / "train", out / "images" / "val",
              out / "labels" / "train", out / "labels" / "val"):
        d.mkdir(parents=True, exist_ok=True)
    dump("train", train_set)
    dump("val", val_set)

    yaml_text = [
        f"path: {out.resolve().as_posix()}",
        "train: images/train",
        "val: images/val",
    ]
    if args.pose:
        yaml_text.append("kpt_shape: [1, 3]")
    yaml_text.append("names:")
    yaml_text += [f"  {i}: {c}" for i, c in enumerate(classes)]
    (out / "data.yaml").write_text("\n".join(yaml_text) + "\n", encoding="utf-8")

    # 报告
    print("=" * 56)
    print(f"图片总数 {len(imgs)}，清洗后 {len(kept)}（丢弃 {len(imgs) - len(kept)}）")
    print(f"train {len(train_set)} / val {len(val_set)}")
    print(f"无目标图（负样本）{empty_count}"
          + ("" if args.keep_empty else "，已剔除"))
    print("每类实例:")
    for i, c in enumerate(classes):
        print(f"  {i} {c}: {cls_counter.get(i, 0)}")
    if dropped:
        print(f"可疑标注 {len(dropped)} 条，示例:")
        for p, err in dropped[:5]:
            print(f"  {p} -> {err}")
    print(f"输出: {out.resolve()}")
    print("=" * 56)

    if min(cls_counter.get(i, 0) for i in range(len(classes))) < args.min_per_class:
        print(f"[警告] 有类别实例数少于 {args.min_per_class}，训练效果会明显下降")
        return 1
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="构建 YOLO(-pose) 训练数据集")
    ap.add_argument("--images", required=True)
    ap.add_argument("--labels", required=True)
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--classes", default=",".join(DEFAULT_CLASSES))
    ap.add_argument("--pose", action="store_true", help="输出 pose 标签（自动补发光中心关键点）")
    ap.add_argument("--val-ratio", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--copy", action="store_true", help="复制图片而不是建符号链接")
    ap.add_argument("--keep-empty", action="store_true", help="保留无目标的负样本图")
    ap.add_argument("--min-per-class", type=int, default=100, help="每类实例数下限，低于则告警")
    return build(ap.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
