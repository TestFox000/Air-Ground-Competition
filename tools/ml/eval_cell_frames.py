"""用 lattice 几何交叉验证评估 cell_classifier 在航拍帧上的识别率与准确率。

对每帧:
  1. GT cell_label 框裁剪(pad 1.3, 与训练一致)
  2. CellIdRecognizer 预测(TTA, conf>=0.45 视为识别成功)
  3. 用高置信预测(>anchor_conf)作锚点拟合多项式 lattice
  4. 几何分配全部框 -> 与预测比对 => 准确率(预测与几何一致的比例)
用法: python eval_cell_frames.py --weights runs/cell_classifier_v4/best.pt --frames val
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
sys.path.insert(0, str(ROOT / "reference" / "onboard" / "dzuav" / "src"))
from cell_classifier.predictor import CellIdRecognizer  # noqa: E402

LETTERS = "ABCDEFGHIJKLMN"


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
    """用锚点拟合 lattice, 迭代剔除残差>25px的锚点。返回 (cx, cy, order) 或 None。"""
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
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", type=Path, required=True)
    ap.add_argument("--frames", default="val", choices=["val", "train"])
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--conf", type=float, default=0.45)
    ap.add_argument("--anchor-conf", type=float, default=0.80)
    ap.add_argument("--prefix", default="", help="帧名过滤子串")
    args = ap.parse_args()

    img_dir = ROOT / f"capture/datasets/aerial_field_mixed/images/{args.frames}"
    lbl_dir = ROOT / f"capture/datasets/aerial_field_mixed/labels/{args.frames}"

    rec = CellIdRecognizer(str(args.weights), device="cuda:0", half=True,
                           rotation_tta=True, confidence_threshold=0.0)  # 全部返回, 自行按conf过滤

    frames = sorted(img_dir.glob("frame_*.jpg"))
    if args.prefix:
        frames = [f for f in frames if args.prefix in f.name]
    if args.max_frames:
        frames = frames[:args.max_frames]

    tot_boxes = tot_rec = tot_acc_n = tot_acc_ok = tot_lattice = 0
    per_frame = []
    for fp in frames:
        lp = lbl_dir / (fp.stem + ".txt")
        boxes = []
        if lp.is_file():
            for line in lp.open(encoding="utf-8"):
                p = line.split()
                if p and int(p[0]) == 1:
                    boxes.append(list(map(float, p[1:5])))
        if len(boxes) < 8:
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
        results = []
        for pr in preds:
            if pr is None:
                results.append(None)
            else:
                results.append((pr.cell_id, pr.joint_confidence))

        # 识别成功: (原框索引, cell_id, conf) —— 必须保留原索引, 过滤后序号会与centers错位
        rec_ok = [(i, r[0], r[1]) for i, r in enumerate(results)
                  if r is not None and r[1] >= args.conf]
        n_rec = len(rec_ok)

        # lattice 几何校验
        grid_all = np.array([[c, r] for r in range(11) for c in range(14)], float)
        acc_n = acc_ok = 0
        assigned = {}
        anchors = [(centers[i], cid) for i, cid, c in rec_ok if c >= args.anchor_conf]
        if len(anchors) >= 8:
            fitres = fit_lattice([a[0] for a in anchors], [a[1] for a in anchors])
            if fitres:
                cx, cy, order = fitres
                Xa = design(grid_all, order)
                pos = np.stack([Xa @ cx, Xa @ cy], 1)
                cen = np.array(centers)
                cand = []
                for i in range(len(cen)):
                    d = np.linalg.norm(pos - cen[i], axis=1)
                    for g in np.argsort(d)[:2]:
                        cand.append((float(d[g]), i, int(g)))
                cand.sort()
                used, amap = set(), {}
                for dist, i, g in cand:
                    if i in amap or g in used or dist > 80:
                        continue
                    used.add(g)
                    amap[i] = to_cid(*grid_all[g])
                for i, cid, c in rec_ok:
                    if i in amap:
                        acc_n += 1
                        if amap[i] == cid:
                            acc_ok += 1
                assigned = amap

        tot_boxes += len(boxes)
        tot_rec += n_rec
        tot_acc_n += acc_n
        tot_acc_ok += acc_ok
        tot_lattice += len(assigned)
        per_frame.append({
            "frame": fp.name, "boxes": len(boxes), "recognized": n_rec,
            "lattice_checked": acc_n, "correct": acc_ok,
            "rec_rate": round(n_rec / len(boxes), 3),
            "acc": round(acc_ok / acc_n, 3) if acc_n else None,
        })
        print(f"{fp.name}: 框={len(boxes)} 识别={n_rec}({n_rec/len(boxes)*100:.0f}%) "
              f"lattice校验={acc_n} 正确={acc_ok}"
              f"{' acc={:.1%}'.format(acc_ok/acc_n) if acc_n else ' [lattice失败]'}")

    print(f"\n=== 汇总 ({len(per_frame)} 帧) ===")
    print(f"总框数: {tot_boxes}")
    print(f"识别成功(conf>={args.conf}): {tot_rec} ({tot_rec/tot_boxes*100:.1f}%)")
    if tot_acc_n:
        print(f"lattice几何校验: {tot_acc_n} 框, 预测与几何一致: {tot_acc_ok} "
              f"({tot_acc_ok/tot_acc_n*100:.1f}%)")
    out_json = ROOT / "runs/cell_recog_render/eval_frames.json"
    out_json.parent.mkdir(parents=True, exist_ok=True)  # 目录可能尚未存在，写前先建
    out_json.write_text(
        json.dumps(per_frame, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
