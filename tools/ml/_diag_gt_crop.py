"""用真值标注框裁剪格号并识别，排除 YOLO 检测框不准的影响。"""
from __future__ import annotations

import sys
from pathlib import Path

from PIL import Image
from ultralytics import YOLO  # noqa: F401  (keep import order stable)

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
CELL_WEIGHTS = ROOT / r"runs\cell_classifier_v3\best.pt"
VAL_IMG = ROOT / r"capture\datasets\aerial_field_mixed\images\val"
VAL_LBL = ROOT / r"capture\datasets\aerial_field_mixed\labels\val"
OUT = ROOT / r"runs\cell_recog_render"

sys.path.insert(0, str(ROOT / "reference" / "onboard" / "dzuav" / "src"))
from cell_classifier.predictor import CellIdRecognizer  # noqa: E402

CELL_CLS = 1


def gt_crops(img_path: Path, lbl_path: Path):
    """从真值标注读取 cell_label 框并裁剪。"""
    img = Image.open(img_path).convert("RGB")
    W, H = img.size
    crops = []
    if not lbl_path.is_file():
        return img, crops
    with lbl_path.open("r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if not parts or int(parts[0]) != CELL_CLS:
                continue
            cx, cy, w, h = map(float, parts[1:5])
            x1 = (cx - w / 2) * W
            y1 = (cy - h / 2) * H
            x2 = (cx + w / 2) * W
            y2 = (cy + h / 2) * H
            side = max(x2 - x1, y2 - y1) * 1.3  # 真值框留更多边距
            ccx = (x1 + x2) / 2
            ccy = (y1 + y2) / 2
            sx1 = max(0, int(ccx - side / 2))
            sy1 = max(0, int(ccy - side / 2))
            sx2 = min(W, int(ccx + side / 2))
            sy2 = min(H, int(ccy + side / 2))
            crops.append(img.crop((sx1, sy1, sx2, sy2)))
    return img, crops


rec = CellIdRecognizer(str(CELL_WEIGHTS), device="cuda:0", half=True,
                       rotation_tta=True, confidence_threshold=0.0)

frames = sorted(VAL_IMG.glob("frame_*.jpg"))
total, ok = 0, 0
for fp in frames:
    lp = VAL_LBL / (fp.stem + ".txt")
    _, crops = gt_crops(fp, lp)
    if not crops:
        continue
    preds = rec.predict(crops)
    n_ok = sum(1 for p in preds if p is not None and p.joint_confidence >= 0.45)
    total += len(preds)
    ok += n_ok
    # 打印前几张的置信度分布
    if fp.name in ("frame_00016.jpg", "frame_00103.jpg"):
        print(f"\n{fp.name} ({len(crops)} 格号):")
        for i, p in enumerate(preds[:10]):
            if p:
                print(f"  [{i}] {p.cell_id}  L={p.column_confidence:.2f} R={p.row_confidence:.2f} J={p.joint_confidence:.2f}")

print(f"\n=== 真值框裁剪结果 ===")
print(f"总格号数: {total}")
print(f"识别成功(conf>=0.45): {ok}")
print(f"识别率: {ok/total*100:.1f}%")
