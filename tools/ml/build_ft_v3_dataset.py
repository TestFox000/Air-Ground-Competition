# -*- coding: utf-8 -*-
"""ft_v3 数据集构建：官方爆炸物域增广叠加 hue 扩增集（data_14_ft_v2）。

产出 data_14_ft_v3/：
  train = data_14_ft_v2/train(含 hue 扩增) + Set A(官方近景伪框) + Set B(copypaste 合成)
  val   = data_14_ft_v2/val 原样复制（与 data_14 val 一致，保跨版本可比）

伪标签纪律（标注主权）：Set A 框为算法生成（掩膜/d13），非用户标注，
manifest.json 逐图记录来源，待用户 X-AnyLabeling 复核后才可视为金标。

留出纪律：官方 157 张 seed 固定切 100 训/57 留出，留出集永不进训练，
门禁只在留出集上算，ft_v1/d13 同子集对照。
"""
from __future__ import annotations

import json
import os
import random
import shutil

import cv2
import numpy as np

SEED = 20261026
OFFICIAL_DIR = (r"D:\人工智能算法精英大赛-空地协同侦排挑战赛"
                r"\手雷图片+通行通道图片\1-爆炸物\B-爆炸物")
HUE_BASE = r"C:\Users\29212\Documents\Trae-CN\default\ft_v2_20261026\data_14_ft_v2"
SRC14 = r"C:\Users\29212\Documents\Trae-CN\default\data_14_20261025"
D13_PT = r"C:\Users\29212\Documents\Trae-CN\default\dk_train_pack_20261016\models\hbb_d13_best.pt"
WORK = r"C:\Users\29212\Documents\Trae-CN\default\ft_v3_official_20261026"
DST = os.path.join(WORK, "data_14_ft_v3")
N_TRAIN_OFFICIAL = 100
N_COMPOSITES = 240          # 60 画布 × 4
D13_CONF = 0.35
MAX_SIDE = 1280

# paper 真实框高分布（data_14 train 33180 框实测分位）
PAPER_H_PCT = {5: 0.0163, 25: 0.024, 50: 0.0349, 75: 0.0508, 95: 0.0783}


def imread_cn(path):
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)


def imwrite_cn(path, img):
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 92])
    if not ok:
        raise RuntimeError(f"encode fail {path}")
    buf.tofile(path)


def border_uniform(img):
    """估计边框环背景色；返回 (bg_bgr, 是否均匀, 环内通道标准差均值)."""
    h, w = img.shape[:2]
    t = max(2, int(min(h, w) * 0.02))
    ring = np.concatenate([
        img[:t].reshape(-1, 3), img[-t:].reshape(-1, 3),
        img[:, :t].reshape(-1, 3), img[:, -t:].reshape(-1, 3)])
    bg = np.median(ring, axis=0)
    std = float(np.mean(ring.std(axis=0)))
    return bg, std < 15.0, std


def mask_bbox(img, bg):
    """背景色距离前景掩膜 → 联通域过滤 → 紧框。失败返回 None。"""
    h, w = img.shape[:2]
    dist = np.linalg.norm(img.astype(np.int16) - bg.astype(np.int16), axis=2)
    m = (dist > 45).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,
                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,
                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    frac = m.sum() / m.size
    if not (0.003 <= frac <= 0.5):
        return None
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    keep = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= 0.15 * m.sum()]
    if not keep:
        return None
    x0 = min(stats[i, cv2.CC_STAT_LEFT] for i in keep)
    y0 = min(stats[i, cv2.CC_STAT_TOP] for i in keep)
    x1 = max(stats[i, cv2.CC_STAT_LEFT] + stats[i, cv2.CC_STAT_WIDTH] for i in keep)
    y1 = max(stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT] for i in keep)
    bw, bh = x1 - x0, y1 - y0
    if bw < 8 or bh < 8 or not (0.3 <= bw / bh <= 3.5):
        return None
    cx, cy = (x0 + x1) / 2 / w, (y0 + y1) / 2 / h
    if not (0.12 <= cx <= 0.88 and 0.12 <= cy <= 0.88):
        return None
    return int(x0), int(y0), int(x1), int(y1)


def d13_boxes(model, img):
    """d13 .pt 推理回退框（仅 cls0 paper）。返回 xyxy 像素框或 None."""
    r = model.predict(img, conf=D13_CONF, classes=[0], imgsz=1280,
                      device=0, verbose=False)[0]
    if len(r.boxes) == 0:
        return None
    i = int(r.boxes.conf.argmax())
    return r.boxes.xyxy[i].cpu().numpy().tolist()


def expand(bx, sw, sh, m=0.02):
    x0, y0, x1, y1 = bx
    dx, dy = (x1 - x0) * m, (y1 - y0) * m
    return (max(0, int(x0 - dx)), max(0, int(y0 - dy)),
            min(sw, int(x1 + dx)), min(sh, int(y1 + dy)))


def load_labels(img_dir, lbl_dir):
    out = {}
    for f in os.listdir(img_dir):
        if not f.lower().endswith((".jpg", ".jpeg", ".png")):
            continue
        stem = os.path.splitext(f)[0]
        lp = os.path.join(lbl_dir, stem + ".txt")
        rows = []
        if os.path.exists(lp):
            for ln in open(lp, encoding="utf-8"):
                p = ln.split()
                if len(p) == 5:
                    rows.append([float(v) for v in p])
        out[f] = rows
    return out


def sample_h():
    """按 paper 高分位线性插值采样贴图高度占比。"""
    ks = sorted(PAPER_H_PCT)
    u = random.uniform(ks[0], ks[-1])
    for a, b in zip(ks, ks[1:]):
        if a <= u <= b:
            t = (u - a) / (b - a)
            return PAPER_H_PCT[a] + t * (PAPER_H_PCT[b] - PAPER_H_PCT[a])
    return PAPER_H_PCT[50]


def overlaps(cx, cy, cw, ch, rows, thr=0.05):
    for r in rows:
        _, x, y, w, h = r
        ix = max(0, min(cx + cw, x + w) - max(cx, x))
        iy = max(0, min(cy + ch, y + h) - max(cy, y))
        inter = ix * iy
        if inter > thr * min(cw * ch, w * h):
            return True
    return False


def main():
    random.seed(SEED)
    np.random.seed(SEED)
    os.makedirs(DST, exist_ok=True)

    # ---------- 1. 切分官方集 ----------
    files = sorted(f for f in os.listdir(OFFICIAL_DIR)
                   if f.lower().endswith((".jpg", ".jpeg", ".png")))
    assert len(files) == 157, f"官方集应为 157 张，实际 {len(files)}"
    rng = random.Random(SEED)
    train_files = set(rng.sample(files, N_TRAIN_OFFICIAL))
    holdout = [f for f in files if f not in train_files]
    print(f"切分: 训练 {len(train_files)} / 留出 {len(holdout)}")

    # ---------- 2. 伪标注 Set A + 掩膜抠图 ----------
    from ultralytics import YOLO
    d13 = YOLO(D13_PT)
    assert d13.names[0] == "paper", f"d13 cls0 应为 paper，实际 {d13.names[0]}"

    cutouts = []          # (cutout_bgr, mask_u8) 供 Set B
    seta = []             # (src_img, norm_box, source)
    skipped = []
    d13_dropped = []
    for i, fn in enumerate(sorted(train_files)):
        img = imread_cn(os.path.join(OFFICIAL_DIR, fn))
        h, w = img.shape[:2]
        bg, uniform, std = border_uniform(img)
        box, src = None, None
        if uniform:
            mb = mask_bbox(img, bg)
            if mb is not None:
                box, src = expand(mb, w, h), "mask"
        if box is None:
            db = d13_boxes(d13, img)
            if db is not None:
                box, src = expand(db, w, h, m=0.0), "d13"
        if box is None:
            skipped.append(fn)
            continue
        if src == "d13":
            # 目检(aug_check 20261026): d13 框偏小/偏移/漏扣，定位噪声大，不入训练
            d13_dropped.append(fn)
            continue
        x0, y0, x1, y1 = box
        seta.append((fn, (x0 / w, y0 / h, (x1 - x0) / w, (y1 - y0) / h), src))
        if src == "mask":
            m = ((np.linalg.norm(img.astype(np.int16) - bg.astype(np.int16),
                                 axis=2) > 45).astype(np.uint8) * 255)
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,
                                 cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
            m = cv2.erode(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
            cutouts.append((img[y0:y1, x0:x1].copy(), m[y0:y1, x0:x1].copy()))
        if (i + 1) % 20 == 0:
            print(f"  伪标注 {i + 1}/{N_TRAIN_OFFICIAL}", flush=True)
    print(f"Set A: {len(seta)} 图入集（全部掩膜伪框）；d13 框质量弃 {len(d13_dropped)}，"
          f"无框弃 {len(skipped)}: {skipped[:6]}")

    # ---------- 3. 数据集骨架 = hue 集整体复制 ----------
    if not os.path.isdir(os.path.join(DST, "images", "train")):
        print("复制 hue 数据集骨架 ...", flush=True)
        shutil.copytree(HUE_BASE, DST, dirs_exist_ok=True)
    tr_img, tr_lbl = os.path.join(DST, "images", "train"), os.path.join(DST, "labels", "train")

    for fn, (xc, yc, bw, bh), _src in seta:
        img = imread_cn(os.path.join(OFFICIAL_DIR, fn))
        h, w = img.shape[:2]
        sc = MAX_SIDE / max(h, w)
        if sc < 1:
            img = cv2.resize(img, (int(w * sc), int(h * sc)), interpolation=cv2.INTER_AREA)
        imwrite_cn(os.path.join(tr_img, "off_" + fn), img)
        with open(os.path.join(tr_lbl, "off_" + os.path.splitext(fn)[0] + ".txt"),
                  "w", encoding="utf-8") as f:
            f.write(f"0 {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}\n")

    # ---------- 4. Set B copypaste 合成 ----------
    canvases = load_labels(os.path.join(SRC14, "images", "train"),
                           os.path.join(SRC14, "labels", "train"))
    canvas_names = sorted(canvases)
    assert cutouts, "无掩膜抠图可用，Set B 无法生成"
    n_cp = 0
    for k in range(N_COMPOSITES):
        cname = random.choice(canvas_names)
        base = imread_cn(os.path.join(SRC14, "images", "train", cname))
        H, W = base.shape[:2]
        rows = [list(r) for r in canvases[cname]]
        added = []
        for _ in range(random.randint(1, 3)):
            cut, msk = random.choice(cutouts)
            ch_px = int(sample_h() * H)
            ch0, cw0 = cut.shape[:2]
            cw_px = int(ch_px * cw0 / ch0)
            if cw_px < 12 or ch_px < 12:
                continue
            cut = cv2.resize(cut, (cw_px, ch_px), interpolation=cv2.INTER_AREA)
            msk = cv2.resize(msk, (cw_px, ch_px), interpolation=cv2.INTER_NEAREST)
            ok_pos = False
            for _try in range(30):
                px = random.randint(int(0.05 * W), int(0.95 * W) - cw_px)
                py = random.randint(int(0.05 * H), int(0.95 * H) - ch_px)
                if not overlaps(px / W, py / H, cw_px / W, ch_px / H, rows):
                    ok_pos = True
                    break
            if not ok_pos:
                continue
            # 泊松融合融入光照 + 轻模糊匹配 aerial 清晰度；异常退回羽化 alpha
            cut_b = cv2.GaussianBlur(cut, (5, 5), 0)
            pasted = False
            if msk.sum() > 0:
                try:
                    base = cv2.seamlessClone(cut_b, base, msk,
                                             (px + cw_px // 2, py + ch_px // 2),
                                             cv2.NORMAL_CLONE)
                    pasted = True
                except cv2.error:
                    pasted = False
            if not pasted:
                alpha = cv2.GaussianBlur(msk.astype(np.float32) / 255.0,
                                         (7, 7), 0)[..., None]
                roi = base[py:py + ch_px, px:px + cw_px].astype(np.float32)
                base[py:py + ch_px, px:px + cw_px] = np.clip(
                    roi * (1 - alpha) + cut_b.astype(np.float32) * alpha, 0, 255
                ).astype(np.uint8)
            rows.append([0.0, (px + cw_px / 2) / W, (py + ch_px / 2) / H,
                         cw_px / W, ch_px / H])
            added.append(rows[-1])
        if not added:
            continue
        stem = f"offcp_{os.path.splitext(cname)[0]}_{k:04d}"
        imwrite_cn(os.path.join(tr_img, stem + ".jpg"), base)
        with open(os.path.join(tr_lbl, stem + ".txt"), "w", encoding="utf-8") as f:
            for r in rows:
                f.write(" ".join(f"{v:.6f}" for v in r) + "\n")
        n_cp += 1
    print(f"Set B: {n_cp} 张合成")

    # ---------- 5. yaml + manifest ----------
    yaml_path = os.path.join(DST, "data_14_ft_v3.yaml")
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(f"path: {DST}\ntrain: images/train\nval: images/val\nnames:\n"
                + "".join(f"  {i}: {n}\n" for i, n in enumerate(
                    ["paper", "cell_label", "car", "target_red", "target_blue",
                     "obstacle", "bucket_red", "bucket_green", "bucket_cyan",
                     "car_start", "car_end", "uav_start", "uav_end", "target"])))
    manifest = {
        "seed": SEED, "official_dir": OFFICIAL_DIR,
        "train": sorted(train_files), "holdout": holdout,
        "set_a": [{"file": f, "box": b, "source": s} for f, b, s in seta],
        "set_a_d13_dropped": d13_dropped,
        "set_a_d13_drop_reason": "aug_check 目检 20261026: d13 框偏小/偏移, 定位噪声大",
        "set_a_skipped": skipped, "pseudo_label_notice":
            "Set A 框为掩膜/d13 伪标签，未经用户复核，不得视为金标",
        "n_composites": n_cp,
    }
    with open(os.path.join(WORK, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=1)
    print(f"yaml: {yaml_path}")
    print(f"train 总量: {len(os.listdir(tr_img))}")


if __name__ == "__main__":
    main()
