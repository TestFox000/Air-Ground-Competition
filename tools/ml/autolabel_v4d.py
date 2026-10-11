"""格号裁剪自动标注（几何一致性裁决，替代人工精标）。

流程/帧:
  1. GT cell_label 框裁剪(pad 1.3, 与训练一致)
  2. CellIdRecognizer(v4c_v6, TTA) 初读
  3. 高置信锚点(>=0.80) 拟合多项式 lattice (迭代剔除残差>25px)
  4. 几何分配全部框(贪心互斥, dist<=80px) -> 格号由几何位置唯一确定
  5. 分档: 预测==几何且conf>=0.6 -> hi; 几何修正/低conf -> mid; dist>40 -> 丢弃
排除已用帧(00124/00170/00099/00250/00020)与val帧, 保证评估独立性。
输出: datasets/cell_labels/aerial_auto/ + runs/cell_recog_render/autolabel_v4d.json
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
sys.path.insert(0, str(ROOT / "reference" / "onboard" / "dzuav" / "src"))
from cell_classifier.predictor import CellIdRecognizer  # noqa: E402

LETTERS = "ABCDEFGHIJKLMN"
WEIGHTS = Path(r"E:\ml_datasets\downloaded\cell_classifier_v4c_v6_last.pt")
IMG_DIR = ROOT / "capture/datasets/aerial_field_mixed/images/train"
LBL_DIR = ROOT / "capture/datasets/aerial_field_mixed/labels/train"
OUT_DIR = ROOT / "datasets/cell_labels/aerial_auto"
USED = {"frame_00124", "frame_00170", "frame_00099", "frame_00250", "frame_00020"}
ANCHOR_CONF = 0.80
HI_CONF = 0.60
MAX_ASSIGN_DIST = 40.0


def to_grid(cid):
    return float(LETTERS.index(cid[0])), float(int(cid[1:]) - 1)


def to_cid(col, row):
    return f"{LETTERS[int(col)]}{int(row) + 1}"


def design(g, order=2):
    c, r = g[:, 0], g[:, 1]
    if order == 2:
        return np.stack([np.ones_like(c), c, r, c * c, c * r, r * r], 1)
    return np.stack([np.ones_like(c), c, r, c * c, c * r, r * r,
                     c ** 3, c * c * r, c * r * r, r ** 3], 1)


def fit_lattice(centers, cids, max_iter=8):
    pts = np.array(centers, float)
    grid = np.array([to_grid(c) for c in cids], float)
    order = 3 if len(pts) >= 14 else 2
    if len(pts) < 8:
        return None
    for _ in range(max_iter):
        X = design(grid, order)
        if len(pts) < max(8, X.shape[1] // 2 + 2):
            return None
        cx, *_ = np.linalg.lstsq(X, pts[:, 0], rcond=None)
        cy, *_ = np.linalg.lstsq(X, pts[:, 1], rcond=None)
        pred = np.stack([X @ cx, X @ cy], 1)
        resid = np.linalg.norm(pred - pts, axis=1)
        ok = resid < 25
        if ok.all() or ok.sum() < 8:
            break
        pts, grid = pts[ok], grid[ok]
        if len(pts) < 8:
            return None
    return cx, cy, order


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rec = CellIdRecognizer(str(WEIGHTS), device="cuda:0", half=True,
                           rotation_tta=True, confidence_threshold=0.0)
    frames = sorted(p for p in IMG_DIR.glob("frame_*.jpg") if p.stem not in USED)
    print(f"候选帧: {len(frames)} (排除已用 {len(USED)})")

    stats = {"frames_total": len(frames), "frames_labeled": 0,
             "frames_skipped_lowboxes": 0, "frames_skipped_lattice": 0,
             "crops_hi": 0, "crops_mid": 0, "crops_corrected": 0,
             "crops_dropped_far": 0, "per_frame": []}
    for fp in frames:
        lp = LBL_DIR / (fp.stem + ".txt")
        boxes = []
        if lp.is_file():
            for line in lp.open(encoding="utf-8"):
                p = line.split()
                if p and int(p[0]) == 1:
                    boxes.append(list(map(float, p[1:5])))
        if len(boxes) < 10:
            stats["frames_skipped_lowboxes"] += 1
            continue

        img = Image.open(fp).convert("RGB")
        W, H = img.size
        centers, crops = [], []
        for cx_, cy_, w_, h_ in boxes:
            x1, y1 = (cx_ - w_ / 2) * W, (cy_ - h_ / 2) * H
            x2, y2 = (cx_ + w_ / 2) * W, (cy_ + h_ / 2) * H
            ccx, ccy = (x1 + x2) / 2, (y1 + y2) / 2
            side = max(x2 - x1, y2 - y1) * 1.3
            sx1, sy1 = max(0, int(ccx - side / 2)), max(0, int(ccy - side / 2))
            sx2, sy2 = min(W, int(ccx + side / 2)), min(H, int(ccy + side / 2))
            centers.append([ccx, ccy])
            crops.append(img.crop((sx1, sy1, sx2, sy2)))

        preds = rec.predict(crops)
        results = [None if pr is None else (pr.cell_id, pr.joint_confidence)
                   for pr in preds]

        anchors = [(centers[i], cid) for i, (cid, c) in enumerate(results)
                   if c is not None and c >= ANCHOR_CONF]
        if len(anchors) < 10:
            stats["frames_skipped_lattice"] += 1
            continue
        fitres = fit_lattice([a[0] for a in anchors], [a[1] for a in anchors])
        if not fitres:
            stats["frames_skipped_lattice"] += 1
            continue
        cx, cy, order = fitres
        grid_all = np.array([[c, r] for r in range(11) for c in range(14)], float)
        Xa = design(grid_all, order)
        pos = np.stack([Xa @ cx, Xa @ cy], 1)
        cen = np.array(centers)
        cand = []
        for i in range(len(cen)):
            d = np.linalg.norm(pos - cen[i], axis=1)
            for g in np.argsort(d)[:2]:
                cand.append((float(d[g]), i, int(g)))
        cand.sort()
        used_g, amap, dmap = set(), {}, {}
        for dist, i, g in cand:
            if i in amap or g in used_g or dist > MAX_ASSIGN_DIST * 2:
                continue
            used_g.add(g)
            amap[i] = to_cid(*grid_all[g])
            dmap[i] = dist

        n_hi = n_mid = n_corr = n_far = 0
        saved = {}
        for i in sorted(amap):
            cid = amap[i]
            dist = dmap[i]
            pred = results[i]
            if dist > MAX_ASSIGN_DIST:
                n_far += 1
                continue
            if pred is not None and pred[0] == cid and pred[1] >= HI_CONF:
                q = "hi"
                n_hi += 1
            else:
                q = "mid"
                if pred is not None and pred[0] != cid:
                    n_corr += 1
                n_mid += 1
            out = OUT_DIR / f"{fp.stem}_f{i:02d}_{cid}.png"
            crops[i].save(out)
            saved[i] = {"cell_id": cid, "dist": round(dist, 1), "quality": q,
                        "pred": pred[0] if pred else None,
                        "conf": round(pred[1], 3) if pred else None}
        if len(saved) < 10:
            stats["frames_skipped_lattice"] += 1
            continue
        stats["frames_labeled"] += 1
        stats["crops_hi"] += n_hi
        stats["crops_mid"] += n_mid
        stats["crops_corrected"] += n_corr
        stats["crops_dropped_far"] += n_far
        stats["per_frame"].append({
            "frame": fp.stem, "boxes": len(boxes), "anchors": len(anchors),
            "saved": len(saved), "hi": n_hi, "mid": n_mid, "corrected": n_corr,
            "far": n_far,
            "labels": {str(k): v for k, v in saved.items()},
        })
        print(f"{fp.stem}: 框={len(boxes)} 锚点={len(anchors)} "
              f"保存={len(saved)} (hi={n_hi} mid={n_mid} 修正={n_corr} 远弃={n_far})",
              flush=True)

    (ROOT / "runs/cell_recog_render/autolabel_v4d.json").write_text(
        json.dumps(stats, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"\n=== 汇总 ===")
    print(f"标注帧: {stats['frames_labeled']}/{stats['frames_total']} "
          f"(跳过: 框少={stats['frames_skipped_lowboxes']} "
          f"lattice失败={stats['frames_skipped_lattice']})")
    print(f"裁剪: hi={stats['crops_hi']} mid={stats['crops_mid']} "
          f"(几何修正={stats['crops_corrected']}) 远弃={stats['crops_dropped_far']}")


if __name__ == "__main__":
    main()
