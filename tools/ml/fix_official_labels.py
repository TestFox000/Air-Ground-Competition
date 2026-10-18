"""修复 mixed 数据集官方图标签: 大框伪标 -> X-AnyLabeling 精确标签。

问题: mixed 中官方图标签是场景级大框(h>0.5), 模型被教成输出大框,
与逐雷精确GT不匹配, 表现为"80%漏检"。
修复: 有精确GT(74张)的替换; 无GT的删除(有雷当背景会教坏模型)。
"""
from __future__ import annotations

import shutil
from pathlib import Path

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
MIX = ROOT / "capture/datasets/aerial_field_mixed"
OFF_LBL = ROOT / "capture/mine_official/labels"

replaced = deleted = kept = 0
for split in ("train", "val", "test"):
    img_dir = MIX / "images" / split
    lbl_dir = MIX / "labels" / split
    for lp in sorted(lbl_dir.glob("explosive_*.txt")):
        gt = OFF_LBL / lp.name
        gt_lines = [l for l in gt.open(encoding="utf-8").read().splitlines() if l.strip()]
        if gt_lines:
            lp.write_text("\n".join(gt_lines) + "\n", encoding="utf-8")
            replaced += 1
        else:
            # 无精确GT: 删除图+标签
            img = img_dir / (lp.stem + ".jpg")
            img.unlink(missing_ok=True)
            lp.unlink()
            deleted += 1
    # 清理缓存
    for cache in lbl_dir.glob("*.cache"):
        cache.unlink()
        print(f"已删缓存 {cache}")

print(f"替换为精确标签: {replaced} 张")
print(f"删除(无GT): {deleted} 张")

# 汇总各split官方图剩余
for split in ("train", "val", "test"):
    n = len(list((MIX / "labels" / split).glob("explosive_*.txt")))
    total = len(list((MIX / "images" / split).glob("*.jpg")))
    print(f"{split}: 官方图 {n} 张 / 总 {total} 张")
