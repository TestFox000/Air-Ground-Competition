"""诊断 v4a 对训练帧 frame_00124 自身 82 张裁剪的预测质量。

判断: 过拟合单帧(裁剪全对但别的帧不行) vs 航拍样本被淹没(连训练裁剪都错)。
输出: 逐张预测 vs 文件名真值, conf, 汇总准确率(全量 / conf>=0.45 / conf>=0.8)。
"""
from __future__ import annotations

import sys
from pathlib import Path

from PIL import Image

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
sys.path.insert(0, str(ROOT / "reference" / "onboard" / "dzuav" / "src"))
from cell_classifier.predictor import CellIdRecognizer  # noqa: E402


def main() -> None:
    rec = CellIdRecognizer(
        str(ROOT / "runs/cell_classifier_v4/best.pt"),
        device="cuda:0", half=True, rotation_tta=True, confidence_threshold=0.0,
    )
    d = ROOT / "datasets/cell_labels/aerial_124"
    files = sorted(d.glob("a124_*.png"))
    crops, gts = [], []
    for p in files:
        gts.append(p.stem.split("_")[-1])
        with Image.open(p) as im:
            crops.append(im.convert("RGB").copy())

    preds = rec.predict(crops)
    n = len(files)
    correct_all = correct_045 = correct_08 = n_045 = n_08 = n_none = 0
    confs = []
    wrong = []
    for pr, gt in zip(preds, gts):
        if pr is None:
            n_none += 1
            wrong.append((gt, "None", 0.0))
            continue
        confs.append(pr.joint_confidence)
        hit = pr.cell_id == gt
        correct_all += hit
        if pr.joint_confidence >= 0.45:
            n_045 += 1
            correct_045 += hit
        if pr.joint_confidence >= 0.80:
            n_08 += 1
            correct_08 += hit
        if not hit:
            wrong.append((gt, pr.cell_id, round(pr.joint_confidence, 2)))

    print(f"共 {n} 张 (frame_00124 裁剪)")
    print(f"预测为None: {n_none}")
    if confs:
        confs.sort()
        print(f"conf中位: {confs[len(confs)//2]:.2f}  最大: {confs[-1]:.2f}")
    print(f"全量准确: {correct_all}/{n} = {correct_all/n:.1%}")
    if n_045:
        print(f"conf>=0.45: {n_045} 张, 准确 {correct_045} = {correct_045/n_045:.1%}")
    if n_08:
        print(f"conf>=0.80: {n_08} 张, 准确 {correct_08} = {correct_08/n_08:.1%}")
    print("\n错误明细 (真值 -> 预测 @conf):")
    for gt, pr, c in wrong:
        print(f"  {gt} -> {pr} @ {c}")


if __name__ == "__main__":
    main()
