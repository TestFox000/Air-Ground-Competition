# -*- coding: utf-8 -*-
"""ft_v3b 数据集（二轮）：ft_v3 集基础上对官方近景 Set A 过采样。

一轮教训（gates_ft_v3.json）：方向已学对（留出漏检 18/22 落 conf 0.02-0.1），
但 paper 大目标训练信号占比仅 ~3% 被稀释，带框训练图自身仅 30/66 检出。
二轮 = Set A 每图 4 份副本 + 2 张随机尺度裁切（近景信号当量 66→396），
其余（hue 集/Set B/val 原封）不变。
"""
from __future__ import annotations

import json
import os
import random
import shutil

import cv2
import numpy as np

SEED = 20261026
WORK = r"C:\Users\29212\Documents\Trae-CN\default\ft_v3_official_20261026"
SRC = os.path.join(WORK, "data_14_ft_v3")
DST = os.path.join(WORK, "data_14_ft_v3b")
N_DUPS = 4          # 全图副本数（含原图本身）
N_CROPS = 2         # 每图随机裁切数
MAX_SIDE = 1280


def imread_cn(p):
    return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)


def imwrite_cn(p, img):
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 92])
    buf.tofile(p)


def main():
    random.seed(SEED)
    if os.path.isdir(DST):
        raise SystemExit(f"{DST} 已存在，先删再跑")
    print("复制 ft_v3 数据集 ...", flush=True)
    shutil.copytree(SRC, DST, ignore=shutil.ignore_patterns("*.cache"))
    tr_img = os.path.join(DST, "images", "train")
    tr_lbl = os.path.join(DST, "labels", "train")

    man = json.load(open(os.path.join(WORK, "manifest.json"), encoding="utf-8"))
    off_dir = man["official_dir"]
    for i, s in enumerate(man["set_a"]):
        stem = f"off_{os.path.splitext(s['file'])[0]}"
        # 副本（原图已在集内，补 N_DUPS-1 份）
        src_img = os.path.join(tr_img, stem + ".jpg")
        src_lbl = os.path.join(tr_lbl, stem + ".txt")
        for d in range(1, N_DUPS):
            shutil.copyfile(src_img, os.path.join(tr_img, f"{stem}_d{d}.jpg"))
            shutil.copyfile(src_lbl, os.path.join(tr_lbl, f"{stem}_d{d}.txt"))
        # 随机尺度裁切：框外扩 1.6-2.8 倍为窗，含完整框
        img = imread_cn(os.path.join(off_dir, s["file"]))
        H, W = img.shape[:2]
        xc, yc, bw, bh = s["box"]
        for c in range(N_CROPS):
            k = random.uniform(1.6, 2.8)
            cw = min(bw * k * W, 0.98 * W)   # 框可占幅 60%+, 外扩后须钳回画幅
            ch = min(bh * k * H, 0.98 * H)
            # 窗中心带抖动，再整体钳进 [0, W-cw]——贴边框直接算区间会倒挂出负起点
            x0 = min(max(xc * W - cw / 2 + random.uniform(-0.15, 0.15) * cw, 0.0),
                     max(0.0, W - cw))
            y0 = min(max(yc * H - ch / 2 + random.uniform(-0.15, 0.15) * ch, 0.0),
                     max(0.0, H - ch))
            x0, y0 = int(x0), int(y0)
            x1, y1 = min(W, int(x0 + cw)), min(H, int(y0 + ch))
            if x1 - x0 < 200 or y1 - y0 < 200:
                continue
            crop = img[y0:y1, x0:x1]
            if crop.size == 0:
                print(f"  空裁切跳过 {s['file']} c{c}: box={s['box']} "
                      f"win=({x0},{y0},{x1},{y1}) img={W}x{H}")
                continue
            sc = MAX_SIDE / max(crop.shape[:2])
            if sc < 1:
                crop = cv2.resize(crop, (int(crop.shape[1] * sc), int(crop.shape[0] * sc)),
                                  interpolation=cv2.INTER_AREA)
            # resize 不改变归一化坐标，直接按裁切窗重归一
            nx = (xc * W - x0) / (x1 - x0)
            ny = (yc * H - y0) / (y1 - y0)
            imwrite_cn(os.path.join(tr_img, f"{stem}_c{c}.jpg"), crop)
            with open(os.path.join(tr_lbl, f"{stem}_c{c}.txt"), "w", encoding="utf-8") as f:
                f.write(f"0 {nx:.6f} {ny:.6f} {bw * W / (x1 - x0):.6f} "
                        f"{bh * H / (y1 - y0):.6f}\n")
        if (i + 1) % 20 == 0:
            print(f"  过采样 {i + 1}/{len(man['set_a'])}", flush=True)

    yaml = os.path.join(DST, "data_14_ft_v3b.yaml")
    with open(yaml, "w", encoding="utf-8") as f:
        f.write(f"path: {DST}\ntrain: images/train\nval: images/val\nnames:\n"
                + "".join(f"  {i}: {n}\n" for i, n in enumerate(
                    ["paper", "cell_label", "car", "target_red", "target_blue",
                     "obstacle", "bucket_red", "bucket_green", "bucket_cyan",
                     "car_start", "car_end", "uav_start", "uav_end", "target"])))
    print(f"train 总量: {len(os.listdir(tr_img))}")


if __name__ == "__main__":
    main()
