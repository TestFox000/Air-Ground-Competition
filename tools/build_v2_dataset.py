# -*- coding: utf-8 -*-
"""构建 aerial_field_v2 数据集：去 bridge 类（11→10 类）+ 并入地面采集新批次。

类别映射（旧 11 类 -> 新 10 类）：
  0 mine / 1 cell_label / 2 arrow 不变；3 bridge 删除；4-10 顺移为 3-9。

来源：
  - 旧标签: capture/datasets/aerial_field_mixed/labels/{train,val,test}（仓库内，权威）
  - 航拍图: CodeBuddy 副本 capture/aerial_field/images（257 张原始帧，按标签文件名匹配）
  - 官方雷图: capture/mine_official/images/explosive_*.jpg（本机缺失 -> 记入 MISSING 清单）
  - 新批次: runs/capture_20261014_1918（重命名 gcap_ 前缀避免与航拍 frame_ 冲突；
            block 划分：帧号>=245 进 val，其余进 train；test 保持纯航拍）

输出 capture/datasets/aerial_field_v2/{images,labels}/{train,val,test} + data.yaml（path: . 相对路径，AutoDL 可用）
"""
from __future__ import annotations

import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OLD_LBL = ROOT / "capture" / "datasets" / "aerial_field_mixed" / "labels"
AERIAL_IMG = Path(r"C:\Users\29212\CodeBuddy\20260927120106\air_ground\capture\aerial_field\images")
OFFICIAL_IMG = ROOT / "capture" / "mine_official" / "images"
NEW = ROOT / "runs" / "capture_20261014_1918"
DST = ROOT / "capture" / "datasets" / "aerial_field_v2"

NEW_NAMES = ["mine", "cell_label", "arrow", "car_start", "car_end",
             "uav_start", "uav_end", "bucket_red", "bucket_green", "bucket_cyan"]
VAL_FROM_FRAME = 245  # 新批次 block 划分边界


def remap_line(line: str) -> str | None:
    p = line.split()
    if len(p) != 5:
        return None
    cid = int(p[0])
    if cid == 3:  # bridge 删除
        return None
    if cid > 3:
        cid -= 1
    return f"{cid} {p[1]} {p[2]} {p[3]} {p[4]}"


def main() -> None:
    if DST.exists():
        shutil.rmtree(DST)
    for s in ("train", "val", "test"):
        (DST / "images" / s).mkdir(parents=True)
        (DST / "labels" / s).mkdir(parents=True)

    stats = {"bridge_dropped": 0, "old_boxes": 0}
    missing_imgs = []

    # 1) 旧标签重映射 + 配图
    for split in ("train", "val", "test"):
        for txt in sorted((OLD_LBL / split).glob("*.txt")):
            out_lines = []
            for line in txt.read_text(encoding="utf-8").strip().splitlines():
                if not line.strip():
                    continue
                old_cid = int(line.split()[0])
                r = remap_line(line)
                if r is None:
                    if old_cid == 3:
                        stats["bridge_dropped"] += 1
                    continue
                out_lines.append(r)
                stats["old_boxes"] += 1
            (DST / "labels" / split / txt.name).write_text(
                "\n".join(out_lines), encoding="utf-8")
            # 配图
            if txt.stem.startswith("explosive_"):
                src = OFFICIAL_IMG / f"{txt.stem}.jpg"
            else:
                src = AERIAL_IMG / f"{txt.stem}.jpg"
            if src.exists():
                shutil.copy2(src, DST / "images" / split / src.name)
            else:
                missing_imgs.append(f"{split}/{txt.stem}.jpg")

    # 2) 新批次：重命名 gcap_ 前缀，block 划分
    new_img = NEW / "images"
    new_lbl = NEW / "labels_final"
    n_new = {"train": 0, "val": 0}
    for img in sorted(new_img.glob("*.jpg")):
        frame_idx = int(img.stem.split("_")[1])
        split = "val" if frame_idx >= VAL_FROM_FRAME else "train"
        name = f"gcap_{img.stem}.jpg"
        shutil.copy2(img, DST / "images" / split / name)
        lbl = new_lbl / f"{img.stem}.txt"
        (DST / "labels" / split / f"gcap_{img.stem}.txt").write_text(
            lbl.read_text(encoding="utf-8"), encoding="utf-8")
        n_new[split] += 1

    # 3) data.yaml（相对路径，上传 AutoDL 后直接可用）
    (DST / "data.yaml").write_text(
        "path: .\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test\n\n"
        f"nc: {len(NEW_NAMES)}\n"
        "names:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(NEW_NAMES)),
        encoding="utf-8")
    (DST / "classes.txt").write_text("\n".join(NEW_NAMES), encoding="utf-8")

    # 4) 校验 + 清单
    print(f"旧标签: 保留框 {stats['old_boxes']}, 删除 bridge {stats['bridge_dropped']}")
    print(f"新批次: train +{n_new['train']}, val +{n_new['val']}")
    for split in ("train", "val", "test"):
        imgs = {p.stem for p in (DST / "images" / split).glob("*.jpg")}
        labs = {p.stem for p in (DST / "labels" / split).glob("*.txt")}
        print(f"{split}: 图 {len(imgs)}, 标签 {len(labs)}, "
              f"缺图标签 {len(labs - imgs)}, 缺标签图 {len(imgs - labs)}")
    if missing_imgs:
        (DST / "MISSING_IMAGES.txt").write_text(
            "# 以下图片本机缺失（官方雷图），需从旧机器/AutoDL 补齐后放入对应 split：\n"
            + "\n".join(missing_imgs), encoding="utf-8")
        print(f"缺失图片 {len(missing_imgs)} 张 -> MISSING_IMAGES.txt")


if __name__ == "__main__":
    main()
