# -*- coding: utf-8 -*-
"""双 hbb 检测器同集头对头（2026-10-26，[H2H-MODEL-20261026]）。

对象：`models/release/hbb_detector_d13_20261016.onnx`（他们现役 13 类）
vs `models/release/hbb_detector_ft_v1_20261026.onnx`（我方 14 类）。
唯一变量=模型，预处理/阈值/NMS/匹配完全同参。

两把尺子：
- 集 A（官方爆炸物留出集，近景域纯正例）：**图级雷召回** = ≥1 个
  cls0(paper) 框的图占比——与 `tools/ml/eval_generalization.py` 门禁
  同口径（MINE_CLS_ID=0，conf≥0.25）。该集无逐框 GT。
- 集 B（data_14 val，222 图，YOLO 逐框标注）：共同类空间 IoU0.5 的
  TP/FP/FN → P/R/F1，幻影=FP 总数。

类空间映射（登记表口径）：
- D13 的 target_red/blue(3,4) 是**整体板**口径 → 映射到共同类
  whole_target；ft_v1 的 target(13) 同为整体板 → whole_target。
- ft_v1 的 target_red/blue(3,4) 是**可见靶面**（D13 无对应），单列
  ft_v1-only，不计入共同类对比。

协议参数：letterbox 1280、conf 0.25、类内 NMS IoU 0.45、匹配 IoU 0.5。
用法（xany 环境含 onnxruntime+GPU）：
    C:/Users/29212/miniconda/envs/xany/python.exe tools/ml/h2h_models.py
    （加 --sanity 只跑 5 张健全性检查）
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# 冻结副本：2026-10-26 22:23 并行会话重写了 models/release（删 D13、换 ft_v1
# 导出）。对比必须用**入库版**（md5 与登记表一致），从 git HEAD 提取到
# _h2h_20261026/models/，不与并行摊子互踩。
MODELS = {
    "d13": os.path.join(REPO, "_h2h_20261026", "models", "d13.onnx"),
    "ft_v1": os.path.join(REPO, "_h2h_20261026", "models", "ft_v1.onnx"),
}
VAL_IMG = r"C:\Users\29212\Documents\Trae-CN\default\data_14_20261025\images\val"
VAL_LBL = r"C:\Users\29212\Documents\Trae-CN\default\data_14_20261025\labels\val"
OFFICIAL_DIR = (r"D:\人工智能算法精英大赛-空地协同侦排挑战赛"
                r"\手雷图片+通行通道图片\1-爆炸物\B-爆炸物")
OUT_JSON = os.path.join(REPO, "_h2h_20261026", "model_h2h.json")

NAMES14 = ["paper", "cell_label", "car", "target_red", "target_blue",
           "obstacle", "bucket_red", "bucket_green", "bucket_cyan",
           "car_start", "car_end", "uav_start", "uav_end", "target"]
CONF, NMS_IOU, MATCH_IOU, IMGSZ = 0.25, 0.45, 0.5, 1280


def load_session(path: str):
    import onnxruntime as ort
    # 显式 CPU：本机 xany 环境的 CUDA EP 缺 cublasLt64_13.dll（2026-10-26
    # 实测），加载报错后仍回退 CPU；锁死 CPU 消除噪音并保证两次运行可比。
    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    so.log_severity_level = 3
    return ort.InferenceSession(path, so, providers=["CPUExecutionProvider"]), \
        ["CPUExecutionProvider"]


def letterbox(img):
    h, w = img.shape[:2]
    s = min(IMGSZ / h, IMGSZ / w)
    nw, nh = round(w * s), round(h * s)
    px, py = (IMGSZ - nw) // 2, (IMGSZ - nh) // 2
    canvas = np.full((IMGSZ, IMGSZ, 3), 114, np.uint8)
    canvas[py:py + nh, px:px + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return blob, s, px, py


def nms(boxes, scores, iou_thr):
    order = np.argsort(-scores)
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        a1 = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        a2 = (boxes[rest, 2] - boxes[rest, 0]) * (boxes[rest, 3] - boxes[rest, 1])
        ov = inter / np.maximum(a1 + a2 - inter, 1e-9)
        order = rest[ov <= iou_thr]
    return keep


def detect(sess, img):
    """返回 [(cls_id, score, xyxy 原图坐标)]。"""
    blob, s, px, py = letterbox(img)
    raw = sess.run(None, {"images": blob})[0][0]          # [4+C, N]
    n_cls = raw.shape[0] - 4
    cls_blk = raw[4:]
    ids = cls_blk.argmax(0)
    scores = cls_blk.max(0)
    keep = scores >= CONF
    cx, cy, bw, bh = raw[0, keep], raw[1, keep], raw[2, keep], raw[3, keep]
    ids, scores = ids[keep], scores[keep]
    x1 = (cx - bw / 2 - px) / s
    y1 = (cy - bh / 2 - py) / s
    x2 = (cx + bw / 2 - px) / s
    y2 = (cy + bh / 2 - py) / s
    boxes = np.stack([x1, y1, x2, y2], 1)
    out = []
    for c in np.unique(ids):
        m = ids == c
        k = nms(boxes[m], scores[m], NMS_IOU)
        for i in k:
            out.append((int(c), float(scores[m][i]), boxes[m][i].tolist()))
    return out


def iou(a, b):
    xx1, yy1 = max(a[0], b[0]), max(a[1], b[1])
    xx2, yy2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, xx2 - xx1) * max(0, yy2 - yy1)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def load_yolo_gt(lbl_path, w, h):
    boxes = []
    if os.path.exists(lbl_path):
        for ln in open(lbl_path, encoding="utf-8"):
            p = ln.split()
            if len(p) != 5:
                continue
            c, xc, yc, bw, bh = (float(x) for x in p)
            boxes.append((int(c), [(xc - bw / 2) * w, (yc - bh / 2) * h,
                                   (xc + bw / 2) * w, (yc + bh / 2) * h]))
    return boxes


def _pred_name(model: str, pid: int) -> str | None:
    """模型预测类 id → 共同类名（data_14 类名空间）。

    d13：0..12 同名，但 3/4 是整体板口径 → 归入 "target"；
    ft_v1：0..13 同名（13=target 整体板；3/4=可见面）。
    """
    if model == "d13":
        return "target" if pid in (3, 4) else NAMES14[pid]
    return NAMES14[pid] if 0 <= pid < 14 else None


FAMILY = "family:桶障族"
_FAM_IDS = {5, 6, 7, 8}       # obstacle + 三色桶：D13 的 obstacle=桶统称（与桶
                               # GT IoU 0.90-0.99 实证），跨口径公平对比须族合并


def eval_common(model: str, sess):
    """data_14 val 逐框评测：GT 与预测都折到 data_14 类名空间后 IoU0.5 匹配。

    另按 _FAM_IDS 维护"桶障族"合并行：族内 GT/预测重新按族名匹配——
    逐类直加会把"同一只桶"既记成 obstacle-FP 又记成 bucket-FN（双重
    计数），族行必须独立匹配才算数。
    """
    stats: dict[str, dict] = {}
    files = sorted(f for f in os.listdir(VAL_IMG) if f.lower().endswith(".jpg"))
    for fn in files:
        img = cv2.imdecode(np.fromfile(os.path.join(VAL_IMG, fn), dtype=np.uint8),
                           cv2.IMREAD_COLOR)          # 中文路径安全
        h, w = img.shape[:2]
        gt = load_yolo_gt(os.path.join(VAL_LBL, os.path.splitext(fn)[0] + ".txt"),
                          w, h)
        pred = detect(sess, img)
        for gid, _gb in gt:
            if gid < 14:
                stats.setdefault(NAMES14[gid], dict(tp=0, fp=0, fn=0))["fn"] += 1
                if gid in _FAM_IDS:
                    stats.setdefault(FAMILY, dict(tp=0, fp=0, fn=0))["fn"] += 1
        matched = set()
        for pid, ps, pb in sorted(pred, key=lambda x: -x[1]):
            cn = _pred_name(model, pid)
            if not cn:
                continue
            for name in (cn, FAMILY if pid in _FAM_IDS else None):
                if name is None:
                    continue
                st = stats.setdefault(name, dict(tp=0, fp=0, fn=0))
                best, bi = 0.0, None
                for gi, (gid, gb) in enumerate(gt):
                    gname = FAMILY if gid in _FAM_IDS else NAMES14[gid]
                    if gi in matched or gname != name:
                        continue
                    v = iou(pb, gb)
                    if v > best:
                        best, bi = v, gi
                if bi is not None and best >= MATCH_IOU:
                    matched.add(bi)
                    st["tp"] += 1
                    st["fn"] -= 1
                else:
                    st["fp"] += 1
    return stats



def eval_official(sess):
    """官方爆炸物留出集：图级雷(cls0 paper)召回，同 eval_generalization 口径。"""
    imgs = sorted(f for f in os.listdir(OFFICIAL_DIR)
                  if f.lower().endswith(".jpg"))
    detected = n_boxes = 0
    for i, fn in enumerate(imgs):
        img = cv2.imdecode(np.fromfile(os.path.join(OFFICIAL_DIR, fn), dtype=np.uint8),
                           cv2.IMREAD_COLOR)
        d = detect(sess, img)
        n_boxes += sum(1 for c, _, _ in d if c == 0)
        if any(c == 0 for c, _, _ in d):
            detected += 1
        if (i + 1) % 40 == 0:
            print(f"  official {i + 1}/{len(imgs)} 检出 {detected}", flush=True)
    return dict(n_images=len(imgs), detected_images=detected,
                recall=round(detected / len(imgs), 4), n_boxes=n_boxes)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sanity", action="store_true", help="只跑 5 张健全性检查")
    a = ap.parse_args()
    results = {}
    for name, path in MODELS.items():
        if not os.path.exists(path):
            sys.exit(f"缺模型: {path}")
        sess, use = load_session(path)
        print(f"== {name} == providers={use}", flush=True)
        if a.sanity:
            files = sorted(f for f in os.listdir(VAL_IMG)
                           if f.lower().endswith(".jpg"))[:5]
            for fn in files:
                img = cv2.imdecode(np.fromfile(os.path.join(VAL_IMG, fn),
                                               dtype=np.uint8), cv2.IMREAD_COLOR)
                d = detect(sess, img)
                gt = load_yolo_gt(os.path.join(
                    VAL_LBL, os.path.splitext(fn)[0] + ".txt"), *img.shape[:2][::-1])
                print(f"  {fn}: pred={[(c, round(s, 2)) for c, s, _ in d][:6]}"
                      f" gt={[g[0] for g in gt][:8]}")
            continue
        results[name] = dict(
            val_common=eval_common(name, sess),
            official=eval_official(sess),
        )
    if not a.sanity:
        os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
        json.dump(results, open(OUT_JSON, "w", encoding="utf-8"),
                  ensure_ascii=False, indent=1)
        for name, r in results.items():
            print(f"\n==== {name} · data_14 val（共同类空间）====")
            print(f"{'类':<14}{'TP':>5}{'FP':>5}{'FN':>5}{'P':>8}{'R':>8}")
            for cn, v in sorted(r["val_common"].items()):
                p = v["tp"] / (v["tp"] + v["fp"]) if v["tp"] + v["fp"] else 0.0
                rc = v["tp"] / (v["tp"] + v["fn"]) if v["tp"] + v["fn"] else 0.0
                print(f"{cn:<14}{v['tp']:>5}{v['fp']:>5}{v['fn']:>5}"
                      f"{p:>8.3f}{rc:>8.3f}")
            o = r["official"]
            print(f"官方留出集: 图级雷召回 {o['detected_images']}/{o['n_images']}"
                  f" = {o['recall']:.1%}（框数 {o['n_boxes']}）")


if __name__ == "__main__":
    main()
