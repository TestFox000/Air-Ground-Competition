"""自举标注: 用 v4a 预测作锚点 + lattice 几何扩散, 自动生成航拍帧格号伪标签。

先自测: 对 frame_00124 跑自举, 与人工真值 labels_124_final.json 比对,
一致率>90% 才可信, 然后推广到 val/train 其他批次帧。

用法:
  python bootstrap_cells.py --selftest                  # 自测124帧
  python bootstrap_cells.py --frames val --prefix frame_0017   # 自举171批次
  python bootstrap_cells.py --frames train --prefix frame_0012  # 自举120批次
输出: runs/cell_recog_render/bootstrap/{frame}.json
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
OUTDIR = ROOT / "runs/cell_recog_render/bootstrap"


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


def fit_lattice(centers, cids, n_iter=150, resid_th=20):
    """RANSAC拟合: 随机采8点拟合order=2, 选内点最多的假设, 内点重拟合升阶。

    解决锚点系统性错位(如行号2/3/5混淆把格号整体拉偏一格)。
    返回 (cx, cy, order, n_inliers) 或 None。
    """
    pts = np.array(centers, float)
    grid = np.array([to_grid(c) for c in cids], float)
    n = len(pts)
    if n < 8:
        return None
    rng = np.random.default_rng(42)
    best_inl = None
    for _ in range(n_iter):
        sel = rng.choice(n, size=8, replace=False)
        X = design(grid[sel], 2)
        try:
            cx, *_ = np.linalg.lstsq(X, pts[sel, 0], rcond=None)
            cy, *_ = np.linalg.lstsq(X, pts[sel, 1], rcond=None)
        except np.linalg.LinAlgError:
            continue
        pred = np.stack([design(grid, 2) @ cx, design(grid, 2) @ cy], 1)
        resid = np.linalg.norm(pred - pts, axis=1)
        inl = resid < resid_th
        if best_inl is None or inl.sum() > best_inl.sum():
            best_inl = inl
            if inl.sum() >= n - 2:  # 已近全内点, 提前停
                break
    if best_inl is None or best_inl.sum() < 8:
        return None
    pi, gi = pts[best_inl], grid[best_inl]
    order = 3 if len(pi) >= 16 else 2
    for _ in range(3):  # 内点迭代收紧
        X = design(gi, order)
        cx, *_ = np.linalg.lstsq(X, pi[:, 0], rcond=None)
        cy, *_ = np.linalg.lstsq(X, pi[:, 1], rcond=None)
        pred = np.stack([X @ cx, X @ cy], 1)
        resid = np.linalg.norm(pred - pi, axis=1)
        ok = resid < resid_th
        if ok.all() or ok.sum() < (12 if order == 3 else 8):
            break
        pi, gi = pi[ok], gi[ok]
    return cx, cy, order, len(pi)


def bootstrap_frame(rec, fp, lp, anchor_conf, conf_pass):
    boxes = []
    for line in lp.open(encoding="utf-8"):
        p = line.split()
        if p and int(p[0]) == 1:
            boxes.append(list(map(float, p[1:5])))
    if len(boxes) < 10:
        return None, "框太少"
    # 与 _make_label_sheets 相同的排序, 保证索引与真值标签一致
    boxes.sort(key=lambda b: (b[1], b[0]))
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
    results = [(pr.cell_id, pr.joint_confidence) if pr else None for pr in preds]

    anchors = [(centers[i], cid) for i, (cid, c) in enumerate(results)
               if cid is not None and c >= anchor_conf]
    if len(anchors) < 8:
        return None, f"锚点不足({len(anchors)})"

    fitres = fit_lattice([a[0] for a in anchors], [a[1] for a in anchors])
    if not fitres:
        return None, "拟合失败"
    cx, cy, order, n_anchor = fitres

    grid_all = np.array([[c, r] for r in range(11) for c in range(14)], float)
    pos = np.stack([design(grid_all, order) @ cx, design(grid_all, order) @ cy], 1)
    cen = np.array(centers)
    cand = []
    for i in range(len(cen)):
        d = np.linalg.norm(pos - cen[i], axis=1)
        for g in np.argsort(d)[:2]:
            cand.append((float(d[g]), i, int(g)))
    cand.sort()
    used, assign = set(), {}
    for dist, i, g in cand:
        if i in assign or g in used or dist > 70:
            continue
        used.add(g)
        assign[i] = (to_cid(*grid_all[g]), dist)

    # 标记: 预测一致 / 仅几何 / 预测冲突
    out = []
    for i in range(len(boxes)):
        if i not in assign:
            continue
        cid, dist = assign[i]
        pred = results[i]
        agree = bool(pred and pred[0] == cid)
        pred_conf = pred[1] if pred else 0.0
        quality = "hi" if (dist < 40 and agree) else ("mid" if dist < 55 else "low")
        if not agree and pred and pred[1] >= conf_pass:
            quality = "conflict"  # 高置信预测与几何冲突 -> 剔除
        out.append({"idx": i, "cell_id": cid, "dist": round(dist, 1),
                    "quality": quality, "agree": agree, "pred_conf": round(pred_conf, 2)})
    stats = {"anchors": n_anchor, "order": order,
             "hi": sum(1 for o in out if o["quality"] == "hi"),
             "mid": sum(1 for o in out if o["quality"] == "mid"),
             "low": sum(1 for o in out if o["quality"] == "low"),
             "conflict": sum(1 for o in out if o["quality"] == "conflict")}
    return {"frame": fp.stem, "boxes": len(boxes), "stats": stats, "labels": out}, None


def main():
    OUTDIR.mkdir(parents=True, exist_ok=True)  # 只在真正运行时建目录，import 期无副作用
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", type=Path,
                   default=ROOT / "runs/cell_classifier_v4/best.pt")
    ap.add_argument("--frames", default="val", choices=["val", "train"])
    ap.add_argument("--prefix", default="")
    ap.add_argument("--anchor-conf", type=float, default=0.30)
    ap.add_argument("--conf-pass", type=float, default=0.45)
    ap.add_argument("--selftest", action="store_true",
                    help="对frame_00124自测: 与人工真值比对")
    args = ap.parse_args()

    rec = CellIdRecognizer(str(args.weights), device="cuda:0", half=True,
                           rotation_tta=True, confidence_threshold=0.0)

    if args.selftest:
        fp = ROOT / "capture/datasets/aerial_field_mixed/images/train/frame_00124.jpg"
        lp = ROOT / "capture/datasets/aerial_field_mixed/labels/train/frame_00124.txt"
        res, err = bootstrap_frame(rec, fp, lp, args.anchor_conf, args.conf_pass)
        if err:
            print(f"自测失败: {err}")
            return
        gt = json.loads((ROOT / "runs/cell_recog_render/label/labels_124_final.json")
                        .read_text(encoding="utf-8"))
        n = ok = 0
        for o in res["labels"]:
            g = gt.get(str(o["idx"]))
            if not g or g["quality"] == "bad":
                continue
            n += 1
            if g["cell_id"] == o["cell_id"]:
                ok += 1
        print(f"自测 frame_00124: 自举分配 {n} 框, 与真值一致 {ok} = {ok/n:.1%}")
        print(f"质量分档: {res['stats']}")
        for o in res["labels"]:
            g = gt.get(str(o["idx"]), {})
            mark = "OK" if g.get("cell_id") == o["cell_id"] else "XX"
            if mark == "XX" and g.get("cell_id"):
                print(f"  #{o['idx']}: 自举={o['cell_id']} 真值={g['cell_id']} "
                      f"dist={o['dist']} pred={o['pred_conf']}")
        return

    img_dir = ROOT / f"capture/datasets/aerial_field_mixed/images/{args.frames}"
    lbl_dir = ROOT / f"capture/datasets/aerial_field_mixed/labels/{args.frames}"
    frames = sorted(img_dir.glob("frame_*.jpg"))
    if args.prefix:
        frames = [f for f in frames if args.prefix in f.name]
    tot = {"hi": 0, "mid": 0, "low": 0, "conflict": 0, "fail": 0}
    for fp in frames:
        lp = lbl_dir / (fp.stem + ".txt")
        if not lp.is_file():
            continue
        res, err = bootstrap_frame(rec, fp, lp, args.anchor_conf, args.conf_pass)
        if err:
            tot["fail"] += 1
            print(f"{fp.stem}: 失败({err})")
            continue
        (OUTDIR / f"{fp.stem}.json").write_text(
            json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
        s = res["stats"]
        for k in ("hi", "mid", "low", "conflict"):
            tot[k] += s[k]
        print(f"{fp.stem}: 框={res['boxes']} 锚点={s['anchors']} "
              f"hi={s['hi']} mid={s['mid']} low={s['low']} conflict={s['conflict']}")
    print(f"\n汇总: {tot}")
    print(f"可用(hi+mid): {tot['hi'] + tot['mid']}")


if __name__ == "__main__":
    main()
