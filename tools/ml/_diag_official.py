"""诊断微调模型在官方157张上的mine置信度分布。

判定漏检80%的根因: conf阈值过高(置信度整体低但排序对) vs 真检不出(高分框都错)。
输出: GT有雷图的最高mine置信度分布 + 各阈值下的检出率。
"""
from __future__ import annotations

import csv
from pathlib import Path

from ultralytics import YOLO

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
IMG = ROOT / "capture/mine_official/images"
LBL = ROOT / "capture/mine_official/labels"

gt_imgs = set()
for lp in LBL.glob("*.txt"):
    if any(line.strip() for line in lp.open(encoding="utf-8")):
        gt_imgs.add(lp.stem)

model = YOLO(str(ROOT / "runs/detect/runs/detect/aerial_field_finetuned_b2/weights/best.pt"))

gt_conf, nongt_conf = [], []
for img_path in sorted(IMG.glob("*.jpg")):
    r = model.predict(str(img_path), conf=0.01, imgsz=1280, verbose=False, device=0)[0]
    best = 0.0
    for box in r.boxes:
        if int(box.cls) == 0:
            best = max(best, float(box.conf))
    (gt_conf if img_path.stem in gt_imgs else nongt_conf).append((img_path.stem, best))

gt_conf.sort(key=lambda x: -x[1])
n = len(gt_conf)
print(f"GT有雷图 {n} 张, 各阈值检出率:")
for th in (0.5, 0.35, 0.25, 0.15, 0.1, 0.05, 0.02, 0.01):
    hit = sum(1 for _, c in gt_conf if c >= th)
    print(f"  conf>={th}: {hit}/{n} = {hit/n:.1%} (漏检 {100-hit/n*100:.1f}%)")

print("\nGT图最高mine置信度 (top10):")
for s, c in gt_conf[:10]:
    print(f"  {s}: {c:.3f}")
print("GT图置信度 (30-45名):")
for s, c in gt_conf[29:45]:
    print(f"  {s}: {c:.3f}")

fp = [c for _, c in nongt_conf if c >= 0.25]
print(f"\n无GT图 {len(nongt_conf)} 张中 conf>=0.25 的: {len(fp)} 张")
print("各阈值下: GT检出 / 无GT误报")
for th in (0.25, 0.15, 0.1, 0.05):
    hit = sum(1 for _, c in gt_conf if c >= th)
    fpn = sum(1 for _, c in nongt_conf if c >= th)
    print(f"  conf>={th}: 检出{hit}/74 ({hit/74:.0%})  误报{fpn}/83")
fp_sorted = sorted(nongt_conf, key=lambda x: -x[1])[:8]
print("无GT图最高置信度 (top8):")
for s, c in fp_sorted:
    print(f"  {s}: {c:.3f}")
