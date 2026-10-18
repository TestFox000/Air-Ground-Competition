#!/usr/bin/env python3
"""构建混合数据集：航拍场地图 + 官方爆炸物图集。

结构 capture/datasets/aerial_field_mixed/：
  images/train = 200 航拍 + 131 官方 + 1 官方负样本(053)
  images/val   =  32 航拍 + 25 官方
  images/test  =  25 航拍（保持纯航拍，用于对比航拍域指标）
  labels/*     = 对应 YOLO txt

官方图集还保留 157 张全量推理评估能力（对比旧模型 58% 漏检率）。
"""

from __future__ import annotations

import random
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
AERIAL = ROOT / "capture" / "datasets" / "aerial_field"
OFFICIAL = ROOT / "datasets" / "mine_official"
OFFICIAL_SRC = ROOT / "capture" / "mine_official" / "images"
DST = ROOT / "capture" / "datasets" / "aerial_field_mixed"

VAL_OFFICIAL = 25
SEED = 123


def main() -> None:
    if DST.exists():
        shutil.rmtree(DST)
    for split in ("train", "val", "test"):
        (DST / "images" / split).mkdir(parents=True)
        (DST / "labels" / split).mkdir(parents=True)

    # 1. 航拍数据原样复制
    for split in ("train", "val", "test"):
        for img in (AERIAL / "images" / split).glob("*"):
            shutil.copy2(img, DST / "images" / split / img.name)
        for lab in (AERIAL / "labels" / split).glob("*"):
            shutil.copy2(lab, DST / "labels" / split / lab.name)

    # 2. 官方图划分：053（空标注）进 train 作负样本，其余随机 131 train / 25 val
    names = sorted(p.stem for p in OFFICIAL_SRC.glob("explosive_*.jpg"))
    names.remove("explosive_053")
    rng = random.Random(SEED)
    rng.shuffle(names)
    val_set = set(names[:VAL_OFFICIAL])

    for stem in names:
        split = "val" if stem in val_set else "train"
        shutil.copy2(OFFICIAL_SRC / f"{stem}.jpg", DST / "images" / split / f"{stem}.jpg")
        shutil.copy2(OFFICIAL / "labels" / f"{stem}.txt", DST / "labels" / split / f"{stem}.txt")
    # 负样本 053 -> train（空 txt）
    shutil.copy2(OFFICIAL_SRC / "explosive_053.jpg", DST / "images" / "train" / "explosive_053.jpg")
    shutil.copy2(OFFICIAL / "labels" / "explosive_053.txt", DST / "labels" / "train" / "explosive_053.txt")

    # 3. data.yaml
    yaml_path = DST / "data.yaml"
    yaml_path.write_text(
        f"path: {DST.as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test\n\n"
        "nc: 11\n"
        "names:\n"
        "  0: mine\n"
        "  1: cell_label\n"
        "  2: arrow\n"
        "  3: bridge\n"
        "  4: car_start\n"
        "  5: car_end\n"
        "  6: uav_start\n"
        "  7: uav_end\n"
        "  8: bucket_red\n"
        "  9: bucket_green\n"
        "  10: bucket_cyan\n",
        encoding="utf-8",
    )

    # 4. 校验：每张图有对应 txt
    for split in ("train", "val", "test"):
        imgs = {p.stem for p in (DST / "images" / split).glob("*.jpg")}
        labs = {p.stem for p in (DST / "labels" / split).glob("*.txt")}
        missing = imgs - labs
        extra = labs - imgs
        n = len(imgs)
        official_n = sum(1 for s in imgs if s.startswith("explosive"))
        print(f"{split}: {n} 张（官方 {official_n}），标签缺失 {len(missing)} 多余 {len(extra)}")
        if missing:
            print("  缺失:", sorted(missing)[:5])


if __name__ == "__main__":
    main()
