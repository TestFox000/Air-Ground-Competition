"""构建 cell_classifier v4d 训练集 = v4c全部成分 + 自动标注新帧(几何裁决)。

新增: aerial_auto/ (v4c_v6初读 + lattice几何裁决自动标注) + aerial_auto_aug/
val 与 v4c 完全一致 (177张, 可与v4a/v4b/v4c严格对比)。
本地用法: python build_cell_v4d.py [--aug-per 20]
服务器用法: python build_cell_v4d.py --root /root/autodl-tmp/cellv4d --aug-per 20 --aug-workers 12
输出: cell_labels/aerial_auto_aug/ + cell_labels/merged_v4d.csv
"""
from __future__ import annotations

import argparse
import csv
import json
import random
import sys
from multiprocessing import Pool
from pathlib import Path

from PIL import Image

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")

AUG_PER = 20
AUTO_AUG = None  # set in main


def _aug(task):
    src, stem, cid = task
    with Image.open(src) as im:
        base = im.convert("RGB").copy()
    rows = []
    for v2 in range(AUG_PER):
        out = AUTO_AUG / f"aug_{stem}_{v2:02d}.png"
        _apply_dr(base, 96).save(out)
        rows.append({"crop_path": f"aerial_auto_aug/{out.name}", "split": "train",
                     "cell_id": cid, "column": cid[0], "row": cid[1:],
                     "status": "labeled"})
    return rows


def main():
    global AUG_PER, AUTO_AUG, _apply_dr
    ap = argparse.ArgumentParser()
    ap.add_argument("--aug-per", type=int, default=20, help="每张auto图增广倍数")
    ap.add_argument("--root", default="", help="服务器根目录(含cell_labels/与autolabel_v4d.json)")
    ap.add_argument("--aug-workers", type=int, default=1, help="增广并行进程数")
    args = ap.parse_args()
    AUG_PER = args.aug_per

    if args.root:
        root = Path(args.root)
        sys.path.insert(0, str(root))
        from gen_cell_synth import apply_domain_randomization
        dataset = root / "cell_labels"
        stats_path = root / "autolabel_v4d.json"
    else:
        sys.path.insert(0, str(ROOT / "tools" / "ml"))
        from gen_cell_synth import apply_domain_randomization
        dataset = ROOT / "datasets" / "cell_labels"
        stats_path = ROOT / "runs/cell_recog_render/autolabel_v4d.json"
    _apply_dr = apply_domain_randomization
    AUTO_AUG = dataset / "aerial_auto_aug"
    random.seed(42)
    AUTO_AUG.mkdir(parents=True, exist_ok=True)

    stats = json.loads(stats_path.read_text(encoding="utf-8"))
    auto_rows, aug_specs = [], []
    for fr in stats["per_frame"]:
        for k, v in fr["labels"].items():
            name = f"{fr['frame']}_f{int(k):02d}_{v['cell_id']}.png"
            rel = f"aerial_auto/{name}"
            assert (dataset / rel).is_file(), rel
            cid = v["cell_id"]
            auto_rows.append({"crop_path": rel, "split": "train", "cell_id": cid,
                              "column": cid[0], "row": cid[1:], "status": "labeled"})
            aug_specs.append((str(dataset / rel), Path(rel).stem, cid))
    print(f"自动标注原图: {len(auto_rows)} 张, 增广 x{AUG_PER} = {len(aug_specs) * AUG_PER} 张")

    if args.aug_workers > 1:
        with Pool(args.aug_workers) as pool:
            groups = pool.map(_aug, aug_specs, chunksize=16)
    else:
        groups = [_aug(t) for t in aug_specs]
    aug_rows = [r for g in groups for r in g]

    with (dataset / "merged_v4c.csv").open(encoding="utf-8") as f:
        rows = [r for r in csv.DictReader(f)]
    n_train_v4c = sum(1 for r in rows if r["split"] == "train")
    n_val = sum(1 for r in rows if r["split"] == "val")
    out_rows = ([r for r in rows if r["split"] == "val"]
                + [r for r in rows if r["split"] == "train"]
                + auto_rows + aug_rows)
    with (dataset / "merged_v4d.csv").open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["crop_path", "split", "cell_id",
                                          "column", "row", "status"])
        w.writeheader()
        w.writerows(out_rows)
    print(f"merged_v4d.csv: train={n_train_v4c + len(auto_rows) + len(aug_rows)} "
          f"(v4c {n_train_v4c} + auto {len(auto_rows)} + aug {len(aug_rows)}) "
          f"val={n_val} (与v4c一致)")


if __name__ == "__main__":
    main()
