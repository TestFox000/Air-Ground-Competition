# -*- coding: utf-8 -*-
"""视频场地侦察管线：D13 检测 + v4d 格号读取 → 场地态势 → 路线规划 + 渲染。

流程：
  1. 抽帧（默认每 15 帧一采）
  2. hbb_d13.onnx（OpenCV DNN, 1280 输入, 17 通道）逐帧检测
  3. cell_label 框裁剪 → cell_classifier_v4d.onnx（96x96 → letter14/row11）读格号
  4. 每帧用「标签像素 ↔ 整数格坐标」做抗粗差仿射拟合，建立帧内网格
  5. paper/靶标/障碍中心 → 连续格坐标；全局分数校准（雷贴格心的众数）→ 落格投票
  6. 聚合出场地态势（雷/障碍/靶/起终点），存 field_state.json
  7. WorldSnapshot → plan_route → 压缩 → 车端命令 + 渲染路线图

用法：
  python video_field_recon.py recording_20261017_143551.mp4 [--stride 15] [--max-frames N]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

BASE = Path(__file__).resolve().parent
D13_ONNX = BASE / "dk_train_pack_20261016" / "models" / "hbb_d13.onnx"
V4D_ONNX = BASE / "cloud_train_20261015" / "deploy_pkg" / "cell_classifier_v4d_20261015.onnx"
HANDOVER = BASE / "路径规划交接_20261017"
sys.path.insert(0, str(HANDOVER / "src" / "ag_integrate"))

LETTERS = "ABCDEFGHIJKLMN"
COLS, ROWS = 14, 11
BRIDGE_COL = "K"
BRIDGE_COL_IDX = LETTERS.index(BRIDGE_COL)

NAMES = ["paper", "cell_label", "car", "target_red", "target_blue", "obstacle",
         "bucket_red", "bucket_green", "bucket_cyan",
         "car_start", "car_end", "uav_start", "uav_end"]
CLS2ID = {n: i for i, n in enumerate(NAMES)}
CONF = {  # 部署参数对齐：paper=0.25 靶标车起止=0.25 障碍桶=0.4；cell_label 放宽保网格拟合样本
    "paper": 0.22, "cell_label": 0.35, "car": 0.25,
    "target_red": 0.25, "target_blue": 0.25, "obstacle": 0.40,
    "bucket_red": 0.40, "bucket_green": 0.40, "bucket_cyan": 0.40,
    "car_start": 0.25, "car_end": 0.25, "uav_start": 0.25, "uav_end": 0.25,
}
OBJ_CLASSES = [n for n in NAMES if n != "cell_label"]  # 需要落格投票的类别
NMS_IOU = 0.45
IN_SIZE = 1280
V4D_IN = 96
V4D_MIN_CONF = 0.65          # letter/row 双头 softmax 下限（TTA 最优旋转）
V4D_PAD = 1.3                # 与 autolabel_v4d 训练裁剪一致：正方形 side = max(w,h)*1.3
VOTE_MIN = 2                 # 落格最少票数
MAX_VOTE_RESIDUAL = 0.35     # 物体连续坐标吸附到格心的最大残差（格单位）


# ---------------------------------------------------------------- 模型封装
class D13:
    def __init__(self, path: Path):
        self.net = cv2.dnn.readNetFromONNX(str(path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)

    def detect(self, img_bgr: np.ndarray) -> tuple[list[dict], tuple[float, float, float]]:
        """返回检测列表 + (scale, pad_x, pad_y)。坐标已映射回原图。"""
        h, w = img_bgr.shape[:2]
        scale = IN_SIZE / max(h, w)
        nw, nh = round(w * scale), round(h * scale)
        resized = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((IN_SIZE, IN_SIZE, 3), 114, np.uint8)
        pad_x, pad_y = (IN_SIZE - nw) // 2, (IN_SIZE - nh) // 2
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255.0, (IN_SIZE, IN_SIZE), swapRB=True)
        self.net.setInput(blob)
        out = self.net.forward()[0]          # (17, 33600)
        boxes, scores = out[:4, :].T, out[4:, :]   # cx,cy,w,h @1280 | (13,33600)
        dets: list[dict] = []
        for c, name in enumerate(NAMES):
            th = CONF[name]
            s = scores[c]
            m = s > th
            if not m.any():
                continue
            b, sc = boxes[m], s[m]
            xywh = np.stack([(b[:, 0] - b[:, 2] / 2 - pad_x) / scale,
                             (b[:, 1] - b[:, 3] / 2 - pad_y) / scale,
                             b[:, 2] / scale, b[:, 3] / scale], axis=1)
            keep = cv2.dnn.NMSBoxes(xywh.tolist(), sc.tolist(), th, NMS_IOU)
            for i in np.array(keep).flatten():
                x, y, w_, h_ = xywh[i]
                dets.append({"cls": name, "conf": float(sc[i]),
                             "box": [float(x), float(y), float(x + w_), float(y + h_)]})
        return dets, (scale, pad_x, pad_y)


class V4D:
    def __init__(self, path: Path):
        self.net = cv2.dnn.readNetFromONNX(str(path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.out_names = self.net.getUnconnectedOutLayersNames()

    @staticmethod
    def _softmax(x: np.ndarray) -> np.ndarray:
        e = np.exp(x - x.max())
        return e / e.sum()

    def read(self, crop_bgr: np.ndarray) -> tuple[int, int, float, float]:
        """crop -> (letter_idx 0..13, row_idx 0..10, letter_conf, row_conf)"""
        best = None
        for k in range(4):                       # 旋转 TTA：手持视角标签可能旋转 90/180/270
            c = np.rot90(crop_bgr, k) if k else crop_bgr
            blob = cv2.dnn.blobFromImage(cv2.resize(c, (V4D_IN, V4D_IN)),
                                         1 / 255.0, (V4D_IN, V4D_IN), swapRB=True)
            self.net.setInput(blob)
            lg, rg = self.net.forward(self.out_names)
            lp, rp = self._softmax(lg[0]), self._softmax(rg[0])
            li, ri = int(lp.argmax()), int(rp.argmax())
            cand = (min(float(lp[li]), float(rp[ri])), li, ri, float(lp[li]), float(rp[ri]))
            if best is None or cand[0] > best[0]:
                best = cand
        return best[1], best[2], best[3], best[4]


# ---------------------------------------------------------------- 几何
def _affine_lstsq(px: np.ndarray, grid: np.ndarray) -> np.ndarray | None:
    x = np.hstack([px, np.ones((len(px), 1))])
    try:
        a, *_ = np.linalg.lstsq(x, grid, rcond=None)
    except np.linalg.LinAlgError:
        return None
    return a


def fit_affine(px: np.ndarray, grid: np.ndarray
               ) -> tuple[np.ndarray | None, np.ndarray, float]:
    """穷举一致性仿射：C(n,3) 基础子集 + 共识投票，抗 v4d 误读毒化。
    小样本（3-7 标签）下迭代剔除无法从 1 个坏点自救，穷举是唯一可靠解。"""
    from itertools import combinations
    n = len(px)
    if n < 3:
        return None, np.zeros(n, bool), 9.9
    best = None                    # (inlier_count, -mae, a, mask)
    for combo in combinations(range(n), 3):
        idx = list(combo)
        if np.linalg.matrix_rank(np.hstack([px[idx], np.ones((3, 1))])) < 3:
            continue               # 共线三点无解
        a = _affine_lstsq(px[idx], grid[idx])
        if a is None:
            continue
        x = np.hstack([px, np.ones((n, 1))])
        r = np.linalg.norm(x @ a - grid, axis=1)
        inl = r < 0.45
        if inl.sum() < 3:
            continue
        a2 = _affine_lstsq(px[inl], grid[inl])   # 共识内点精解
        if a2 is None:
            continue
        r2 = np.linalg.norm(x @ a2 - grid, axis=1)
        inl2 = r2 < 0.45
        if inl2.sum() < 3:
            continue
        mae = float(r2[inl2].mean())
        cand = (int(inl2.sum()), -mae, a2, inl2)
        if best is None or (cand[0], cand[1]) > (best[0], best[1]):
            best = cand
    if best is None or best[0] < 3 or -best[1] > 0.40:
        return None, np.zeros(n, bool), 9.9
    return best[2], best[3], -best[1]


def fit_homography(px: np.ndarray, grid: np.ndarray
                   ) -> tuple[np.ndarray | None, np.ndarray, float]:
    """抗粗差单应性：平面场地+针孔相机的精确模型。px(N,2)->grid(N,2)。
    返回 (H 3x3, inlier_mask, mae)。"""
    n = len(px)
    if n < 4:
        return None, np.zeros(n, bool), 9.9
    grid_f = np.asarray(grid, np.float32)
    pts_f = np.asarray(px, np.float32)
    m = np.ones(n, bool)
    r = np.full(n, 9.9)
    for _ in range(4):
        if m.sum() < 4:
            return None, m, 9.9
        H, _ = cv2.findHomography(pts_f[m], grid_f[m], cv2.USAC_MAGSAC, 0.45)
        if H is None:
            return None, m, 9.9
        proj = cv2.perspectiveTransform(pts_f.reshape(-1, 1, 2), H).reshape(-1, 2)
        r = np.linalg.norm(proj - grid_f, axis=1)
        new = r < 0.45
        if (new == m).all() or new.sum() < 4:
            break
        m = new
    if m.sum() < 4:
        return None, m, 9.9
    mae = float(r[m].mean())
    # 用最终内点做纯最小二乘精解（消除 RANSAC 抖动）
    H, _ = cv2.findHomography(pts_f[m], grid_f[m], 0)
    if H is None:
        return None, m, mae
    proj = cv2.perspectiveTransform(pts_f.reshape(-1, 1, 2), H).reshape(-1, 2)
    r = np.linalg.norm(proj - grid_f, axis=1)
    m = r < 0.45
    if m.sum() < 4:
        return None, m, 9.9
    return H, m, float(r[m].mean())


def circular_mode(vals: list[float], bins: int = 20) -> float:
    """1D 环形众数（分数部分聚类），返回众数中心。"""
    if not vals:
        return 0.5
    hist, edges = np.histogram(np.array(vals) % 1.0, bins=bins, range=(0, 1))
    ext = np.concatenate([hist[-1:], hist, hist[:1]])   # 环形平滑
    k = int(np.argmax(ext[1:-1]))
    return (edges[k] + edges[k + 1]) / 2


def cell_name(col: int, row: int) -> str:
    return f"{LETTERS[col]}{row + 1}"


def imwrite_u(path: Path, img: np.ndarray) -> None:
    """cv2.imwrite 在 Windows 中文路径下静默失败，用 imencode 规避。"""
    ok, buf = cv2.imencode(path.suffix, img)
    if ok:
        path.write_bytes(buf.tobytes())


# ---------------------------------------------------------------- 主流程
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("video", type=str)
    ap.add_argument("--stride", type=int, default=15)
    ap.add_argument("--max-frames", type=int, default=0, help="调试：只处理前 N 个采样帧")
    ap.add_argument("--out", type=str, default="视频侦察_20261017")
    args = ap.parse_args()

    out_dir = BASE / args.out
    (out_dir / "annotated").mkdir(parents=True, exist_ok=True)
    (out_dir / "label_crops").mkdir(parents=True, exist_ok=True)

    d13, v4d = D13(D13_ONNX), V4D(V4D_ONNX)

    cap = cv2.VideoCapture(args.video)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    idxs = list(range(0, total, args.stride))
    if args.max_frames:
        idxs = idxs[:args.max_frames]
    print(f"[recon] 视频总帧 {total}，采样 {len(idxs)} 帧 (stride={args.stride})")

    frames_meta = []          # 每帧：标签读数/拟合质量/物体连续坐标
    cont_pos: dict[str, list[tuple[float, float, float]]] = defaultdict(list)  # cls -> (qcol,qrow,conf)
    crop_saved = 0
    t0 = time.time()

    for fi, fidx in enumerate(idxs):
        cap.set(cv2.CAP_PROP_POS_FRAMES, fidx)
        ok, frame = cap.read()
        if not ok:
            continue
        dets, _ = d13.detect(frame)
        labels, objs = [], []
        for d in dets:
            (labels if d["cls"] == "cell_label" else objs).append(d)

        # --- v4d 读格号（正方形裁剪，与训练数据一致）
        reads = []            # (letter, row, lconf, rconf, box)
        for d in labels:
            x1, y1, x2, y2 = d["box"]
            ccx, ccy = (x1 + x2) / 2, (y1 + y2) / 2
            side = max(x2 - x1, y2 - y1) * V4D_PAD
            sx1 = max(0, int(ccx - side / 2)); sy1 = max(0, int(ccy - side / 2))
            sx2 = min(frame.shape[1], int(ccx + side / 2))
            sy2 = min(frame.shape[0], int(ccy + side / 2))
            crop = frame[sy1:sy2, sx1:sx2]
            if crop.size == 0:
                continue
            li, ri, lc, rc = v4d.read(crop)
            if lc >= V4D_MIN_CONF and rc >= V4D_MIN_CONF:
                reads.append((li, ri, lc, rc, d["box"], d["conf"]))
                if crop_saved < 40:      # 存裁剪图供人工核验
                    imwrite_u(out_dir / "label_crops" /
                               f"f{fidx:05d}_{cell_name(li, ri)}_L{lc:.2f}_R{rc:.2f}.jpg", crop)
                    crop_saved += 1

        # --- 帧内网格拟合：仿射打底保覆盖，内点充足时升级单应性保精度
        hom, mae = None, 9.9
        if len(reads) >= 3:
            px = np.array([[(b[0] + b[2]) / 2, (b[1] + b[3]) / 2] for *_, b, _ in reads], np.float64)
            grid = np.array([[li, ri] for li, ri, *_ in reads], np.float64)
            aff, inl, mae = fit_affine(px, grid)
            if aff is not None:
                # 纯仿射：小样本/共线锚点下单应性外推病态（实测毒化落格），
                # 仿射 + 锚点距离门控（投票阶段）是覆盖与精度的最优平衡
                hom = np.array([
                    [aff[0, 0], aff[1, 0], aff[2, 0]],
                    [aff[0, 1], aff[1, 1], aff[2, 1]],
                    [0.0, 0.0, 1.0]])
            reads = [r for r, k in zip(reads, inl) if k] if aff is not None else []
        grid_ok = hom is not None and mae < 0.45

        # --- 物体连续格坐标
        frame_objs = []
        if grid_ok:
            for d in objs:
                x1, y1, x2, y2 = d["box"]
                p = cv2.perspectiveTransform(
                    np.array([[[ (x1 + x2) / 2, (y1 + y2) / 2 ]]], np.float32), hom)[0, 0]
                qc, qr = float(p[0]), float(p[1])
                if -0.5 <= qc < COLS + 0.5 and -0.5 <= qr < ROWS + 0.5:
                    cont_pos[d["cls"]].append((qc, qr, d["conf"]))
                    frame_objs.append({"cls": d["cls"], "conf": d["conf"],
                                       "q": [round(qc, 3), round(qr, 3)],
                                       "box": [round(v, 1) for v in d["box"]]})

        frames_meta.append({"frame": fidx, "n_dets": len(dets), "n_labels_read": len(reads),
                            "grid_mae": round(mae, 3), "grid_ok": grid_ok,
                            "labels": [cell_name(li, ri) for li, ri, *_ in reads],
                            "objs": frame_objs})

        # --- 标注帧（每 8 个采样帧存一张）
        if fi % 8 == 0:
            vis = frame.copy()
            for li, ri, lc, rc, b, _ in reads:
                cv2.rectangle(vis, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (255, 180, 0), 1)
                cv2.putText(vis, cell_name(li, ri), (int(b[0]), int(b[1]) - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 180, 0), 1)
            for d in objs:
                x1, y1, x2, y2 = map(int, d["box"])
                color = (0, 0, 255) if d["cls"] == "paper" else (0, 200, 255)
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 1)
                cv2.putText(vis, f"{d['cls'][:6]}{d['conf']:.2f}", (x1, y2 + 11),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
            imwrite_u(out_dir / "annotated" / f"f{fidx:05d}.jpg", vis)

        if (fi + 1) % 20 == 0:
            print(f"[recon] {fi + 1}/{len(idxs)} 帧  网格OK {sum(1 for m in frames_meta if m['grid_ok'])}"
                  f"  {time.time() - t0:.0f}s")
    cap.release()

    grid_ok = [m for m in frames_meta if m["grid_ok"]]
    print(f"[recon] 检测完成：{len(frames_meta)} 帧中 {len(grid_ok)} 帧网格拟合成功，耗时 {time.time() - t0:.0f}s")

    # --- 全局分数校准：雷贴格心 → 分数众数即标签相对格心偏移
    frac_c = [q[0] for q in cont_pos.get("paper", [])]
    frac_r = [q[1] for q in cont_pos.get("paper", [])]
    if len(frac_c) < 8:      # paper 太少时用全部物体
        frac_c = [q[0] for cls in OBJ_CLASSES for q in cont_pos.get(cls, [])]
        frac_r = [q[1] for cls in OBJ_CLASSES for q in cont_pos.get(cls, [])]
    fc, fr = circular_mode(frac_c), circular_mode(frac_r)
    print(f"[recon] 分数校准 offset = ({fc:.2f}, {fr:.2f})  paper连续点 {len(cont_pos.get('paper', []))} 个")

    # --- 落格投票（每帧每类每格至多 1 票）
    # 门控1：物体连续坐标必须落在帧内标签锚点 1.5 格内（锚点区外仿射外推不可信）
    # 门控2：吸附残差 <= MAX_VOTE_RESIDUAL
    votes: dict[str, Counter] = defaultdict(Counter)
    vframes: dict[str, dict[str, set]] = defaultdict(dict)     # cls -> cell -> 投票帧集合
    vconf: dict[str, dict[str, float]] = defaultdict(dict)     # cls -> cell -> 置信度总和
    seen: dict[int, set[tuple[str, str]]] = defaultdict(set)   # frame -> {(cls, cell)}
    for m in frames_meta:
        if not m["grid_ok"]:
            continue
        anchors = [(LETTERS.index(n[0]), int(n[1:]) - 1) for n in m["labels"]]
        for o in m["objs"]:
            qc, qr = o["q"]
            if not any(max(abs(qc - a0), abs(qr - a1)) <= 1.5 for a0, a1 in anchors):
                continue
            col, row = int(round(qc - fc)), int(round(qr - fr))
            resid = max(abs(qc - fc - col), abs(qr - fr - row))
            if resid > MAX_VOTE_RESIDUAL:      # 吸附残差过大：落点不可信，弃票
                continue
            if 0 <= col < COLS and 0 <= row < ROWS:
                key = (o["cls"], cell_name(col, row))
                if key not in seen[m["frame"]]:
                    seen[m["frame"]].add(key)
                    votes[o["cls"]][key[1]] += 1
                    vframes[o["cls"]].setdefault(key[1], set()).add(m["frame"])
                    vconf[o["cls"]][key[1]] = vconf[o["cls"]].get(key[1], 0.0) + o["conf"]

    # --- 可见覆盖（该格被有效网格帧覆盖的次数）
    coverage: Counter = Counter()
    for m in frames_meta:
        if not m["grid_ok"]:
            continue
        cells = {n for n in m["labels"]}
        cols = [LETTERS.index(n[0]) for n in cells]
        rows = [int(n[1:]) - 1 for n in cells]
        for c in range(min(cols) - 1, max(cols) + 2):
            for r in range(min(rows) - 1, max(rows) + 2):
                if 0 <= c < COLS and 0 <= r < ROWS:
                    coverage[cell_name(c, r)] += 1

    # --- 场地态势
    def mean_conf(cls: str, cell: str) -> float:
        n = votes[cls].get(cell, 0)
        return (vconf[cls].get(cell, 0.0) / n) if n else 0.0

    def eliminate_phantoms(cells: list[str], cls: str = "paper") -> tuple[list[str], dict]:
        """幻影消除：弱格(≤3票)若与某强格(≥3×票)从未同帧共现、且票期被强格
        夹芯包裹（强格在弱格前后都有票），判定为同一物理物体的误归属（透视
        偏移/v4d 误读锚点拽偏），剔除。同帧共现 = 两个独立物体，保留。"""
        phantoms: dict[str, str] = {}          # 弱格 -> 强格
        strong = [c for c in cells if votes[cls][c] >= 6]
        for c in cells:
            n = votes[cls][c]
            if n > 3:
                continue
            for s in strong:
                if votes[cls][s] < 3 * n:
                    continue
                fs, fc_ = vframes[cls][c], vframes[cls][s]
                if fs & fc_:                   # 同帧共现：独立物体
                    continue
                if min(fs) > min(fc_) and max(fs) < max(fc_):   # 票期夹芯包裹
                    phantoms[c] = s
                    break
        return [c for c in cells if c not in phantoms], phantoms

    def confirmed(cls_list: list[str], min_votes: int = VOTE_MIN,
                  min_mean_conf: float = 0.0, phantom_kill: bool = False) -> list[str]:
        merged = Counter()
        for cls in cls_list:
            merged.update(votes.get(cls, Counter()))
        cells = []
        for c, v in merged.items():
            if v < min_votes:
                continue
            if min_mean_conf:
                mconf = sum(vconf[k].get(c, 0.0) for k in cls_list) / max(v, 1)
                if mconf < min_mean_conf:
                    continue
            cells.append(c)
        if phantom_kill and cells:
            cells, _ph = eliminate_phantoms(cells, cls_list[0])
        return sorted(cells)

    paper_cells = confirmed(["paper"], VOTE_MIN, 0.35, phantom_kill=True)
    _, phantom_log = eliminate_phantoms(
        [c for c, v in votes["paper"].items() if v >= VOTE_MIN], "paper")
    mines = [c for c in paper_cells if LETTERS.index(c[0]) != BRIDGE_COL_IDX]
    passable = [c for c in paper_cells if LETTERS.index(c[0]) == BRIDGE_COL_IDX]   # K列 paper=箭头
    obstacles = confirmed(["obstacle", "bucket_red", "bucket_green", "bucket_cyan"])
    tgt_red = confirmed(["target_red"], 2, 0.40)   # 单票/低置信多为人体误检
    tgt_blue = confirmed(["target_blue"], 2, 0.40)
    car_start = confirmed(["car_start"], 2)
    car_end = confirmed(["car_end"], 2)
    uav_marks = {cls: confirmed([cls], 2) for cls in ("uav_start", "uav_end")}

    field_state = {
        "video": args.video, "stride": args.stride,
        "frames_sampled": len(frames_meta), "frames_grid_ok": len(grid_ok),
        "frac_offset": [round(fc, 3), round(fr, 3)],
        "mines_video": mines, "passable_video": passable,
        "phantoms_eliminated": phantom_log,
        "obstacles_video": obstacles,
        "targets_video": {"red": tgt_red, "blue": tgt_blue},
        "car_start_video": car_start, "car_end_video": car_end,
        "uav_marks_video": uav_marks,
        "votes_detail": {cls: dict(cnt.most_common()) for cls, cnt in votes.items()},
        "coverage": dict(coverage.most_common()),
    }
    (out_dir / "field_state.json").write_text(
        json.dumps(field_state, ensure_ascii=False, indent=2), encoding="utf-8")
    (out_dir / "frames_meta.json").write_text(
        json.dumps(frames_meta, ensure_ascii=False), encoding="utf-8")
    print(f"[recon] 态势: 雷 {mines} | K列箭头 {passable} | 障碍 {obstacles}"
          f" | 靶 红{tgt_red} 蓝{tgt_blue} | 车起{car_start} 车终{car_end}")

    # ---------------------------------------------------------------- 规划
    from ag_integrate.field_frame import CellRef
    from ag_integrate.world_model import WorldSnapshot, TargetRef
    from ag_integrate.mission_router import (plan_route, compress_route, turn_violations,
                                             _turn_model_of, R_MIN_M, HALF_WIDTH_M,
                                             BUCKET_RADIUS_M, TARGET_RADIUS_M)
    from ag_integrate.car_adapter import adapt

    def cell(n: str) -> CellRef:
        return CellRef(LETTERS.index(n[0]), int(n[1:]) - 1)

    # 场地裁判基线（场地认知图_20261017）：规则固定 8 雷 + 8 障碍，桥门 K4/K10
    OFFICIAL_MINES = ["B1", "A2", "I2", "C3", "E3", "F5", "H8", "G9"]
    OFFICIAL_PASSABLE = [f"{BRIDGE_COL}4", f"{BRIDGE_COL}10"]
    OFFICIAL_OBSTACLES = ["D5", "I6", "B7", "F8", "C9", "E10", "G10", "A11"]
    TRACK_DFT = ("D7", "E7")
    START_DFT, GOAL_DFT = "N10", "N5"

    use_start = (car_start[0] if len(car_start) == 1 else START_DFT)
    use_goal = (car_end[0] if len(car_end) == 1 else GOAL_DFT)

    # 雷按规则固定 8 颗：以官方基线为权威集（视频检不全 ≠ 场上无雷），
    # 视频检出只作确认/异常标注，绝不因漏检而缩减排雷集合。
    v_mines, v_obstacles, v_passable = list(mines), list(obstacles), list(passable)  # 原始视频检出
    mines_baseline_only = [c for c in OFFICIAL_MINES if c not in v_mines]
    mines_video_unmatched = [c for c in v_mines if c not in OFFICIAL_MINES]
    use_mines = list(OFFICIAL_MINES)
    # 障碍取并集（避障宁可保守：漏避真实障碍代价高于多绕一格）
    use_obstacles = sorted(set(OFFICIAL_OBSTACLES) | set(v_obstacles))
    # 视频箭头并入桥门；靶若落在障碍格则剔除（禁止朝障碍/位置冲突目标开火）
    use_passable = sorted(set(OFFICIAL_PASSABLE) | set(v_passable))
    tgt_dropped = [c for c in (tgt_blue + tgt_red) if c in use_obstacles]

    def build_targets(blue_list: list[str], red_list: list[str]) -> list[TargetRef]:
        ts = ([TargetRef(cell(c), True) for c in blue_list]
              + [TargetRef(cell(c), False) for c in red_list])
        if not ts:
            # 移动靶默认：蓝面朝向每局不定、视频无法判定 → 发现态（未辨）。
            # 路由插确认点，车到点近距复判红蓝，再由 replan 守卫定射击位；
            # fire_guard 对未辨靶一律禁火（fail-safe），绝不赌一侧误击友方。
            ts = [TargetRef(cell(TRACK_DFT[0]), None,
                            track=(cell(TRACK_DFT[0]), cell(TRACK_DFT[1])),
                            blue_face=None)]
        return ts

    use_targets = build_targets([c for c in tgt_blue if c not in use_obstacles],
                                [c for c in tgt_red if c not in use_obstacles])

    def finalize_state(m_list, p_list, o_list, tgts, tc):
        """建快照 → 规划 → 压缩 → 车端命令。返回 (snapshot, route, compact, cmds)。

        tc = (最小转弯半径, 车身半宽) 格单位；非 None 时启用阿克曼转弯扫掠硬约束。
        """
        snap = WorldSnapshot(0, tuple(cell(c) for c in m_list), tuple(cell(c) for c in p_list),
                             tuple(cell(c) for c in o_list), tuple(tgts))
        rt = plan_route(snap, cell(use_start), cell(use_goal),
                        start_heading="y", final_heading="y", turn_clearance=tc)
        tnames = {TRACK_DFT[0], TRACK_DFT[1]} if not tgt_blue and not tgt_red else {
            c.name for t in tgts if t.track for c in t.track}
        keep = set(m_list) | set(p_list) | tnames | set(tgt_blue) | set(tgt_red)
        for t in tgts:                    # 射击停靠格必须保留（蓝面侧轨道端点邻格）
            if t.track is None or not t.blue_face:
                continue
            a, b = t.track
            if a.row == b.row:            # 水平轨道：S=row+1 / N=row-1
                d = 1 if t.blue_face == "S" else -1
                keep |= {CellRef(a.col, a.row + d).name, CellRef(b.col, b.row + d).name}
            else:                         # 垂直轨道：E=col+1 / W=col-1
                d = 1 if t.blue_face == "E" else -1
                keep |= {CellRef(a.col + d, a.row).name, CellRef(b.col + d, b.row).name}
        cp = compress_route(rt, keep)
        cm = adapt(snap, cp, use_start, "y", "y")
        return snap, rt, cp, cm

    # 可达性逐级放宽（不安全时才降级，全程记录来源）
    # L1/L2 启用阿克曼转弯扫掠硬约束（三类实体建模）；L3 才放宽该约束
    ladders = [
        (use_mines, use_passable, use_obstacles, use_targets, "官方8雷+保守避障（转弯约束开）"),
        (use_mines, OFFICIAL_PASSABLE, OFFICIAL_OBSTACLES, use_targets, "官方8雷+官方避障（转弯约束开）"),
        (use_mines, OFFICIAL_PASSABLE, OFFICIAL_OBSTACLES, use_targets, "官方8雷+官方避障（转弯约束放宽）"),
    ]
    snapshot = route = compact = cmds = None
    mines, obstacles = use_mines, use_obstacles
    fallback_note, last_err = ladders[0][4], ""
    for m_l, p_l, o_l, t_l, note in ladders:
        tc = None
        if "放宽" not in note:
            _snap = WorldSnapshot(0, tuple(cell(c) for c in m_l),
                                  tuple(cell(c) for c in p_l),
                                  tuple(cell(c) for c in o_l), tuple(t_l))
            tc = _turn_model_of(_snap)
        try:
            snapshot, route, compact, cmds = finalize_state(m_l, p_l, o_l, t_l, tc)
            mines, obstacles, use_passable, targets = m_l, o_l, p_l, t_l
            fallback_note = note
            break
        except ValueError as e:
            last_err = str(e)[:120]
            continue
    if route is None:
        (out_dir / "plan_failed.json").write_text(
            json.dumps({"video": args.video, "error": last_err,
                        "mines_video": mines, "obstacles_video": obstacles,
                        "passable_video": passable}, ensure_ascii=False, indent=2),
            encoding="utf-8")
        print(f"[plan] 不可达：{last_err}")
        print(f"[plan] 详情: {out_dir / 'plan_failed.json'}")
        return 2

    cells = [cell(n) for n in route]
    length, turns = 0.0, 0
    prev = None
    for a, b in zip(cells, cells[1:]):
        d = (b.col - a.col, b.row - a.row)
        length += (d[0] ** 2 + d[1] ** 2) ** 0.5
        if prev is not None and (abs(d[0] - prev[0]) + abs(d[1] - prev[1])) in (1, 2):
            turns += 1
        prev = d
    route_set = set(route)
    missing = [c for c in mines if c not in route_set]
    viol = [c for c in route if c in obstacles]

    # ---- 转弯可行性复核：满舵圆弧扫掠撞实体 = 车物理跑不了 ----
    # 与规划层 turn_clearance 同一套三类实体模型（桥墙整格 / 桶φ0.25 / 靶0.025薄板），
    # 由 mission_router.turn_violations 统一提供，作为独立第二意见。
    turn_bad = turn_violations(route, snapshot)
    turn_ok = not turn_bad

    route_plan = {
        "source": {"mines": "裁判规则固定8雷（官方基线，视频漏检不缩减）",
                   "obstacles": "官方基线∪视频检出（保守并集）",
                   "targets": "视频识别（剔除与障碍冲突者）；移动靶蓝面不定→发现态两段式",
                   "passable": use_passable, "start": use_start, "goal": use_goal,
                   "ladder": fallback_note},
        "snapshot": {"mines": mines, "passable": use_passable, "obstacles": obstacles,
                     "targets": [f"{'蓝' if t.is_blue else ('红' if t.is_blue is False else '未辨')}"
                                 f"@{t.cell.name}" for t in targets]},
        "evidence": {"mines_video_detected": v_mines, "mines_baseline_only": mines_baseline_only,
                     "mines_video_unmatched": mines_video_unmatched,
                     "obstacles_video": v_obstacles, "targets_dropped_conflict": tgt_dropped,
                     "passable_video": v_passable},
        "route": list(route), "route_len_cells": len(route),
        "route_len_diag": round(length, 2), "turns": turns,
        "compact": list(compact),
        "car_cmds": {"reset": cmds.reset, "route": cmds.route, "shoot_cells": list(cmds.shoot_cells)},
        "check": {"排雷覆盖": "全部覆盖" if not missing else f"缺失{missing}",
                  "障碍避让": "通过" if not viol else f"穿障{viol}",
                  "转弯可行性": ("通过（满舵圆弧扫掠无碰撞）" if turn_ok
                            else f"不可行{len(turn_bad)}处：" + "；".join(
                                f"{b['拐点']}→{b['扫掠撞']}" for b in turn_bad)),
                  "转弯模型": (f"R_min={R_MIN_M:.3f}m ({R_MIN_M / 0.35:.3f}格) 半宽{HALF_WIDTH_M:.2f}m"
                            f" | 桥墙=整格 / 桶φ{BUCKET_RADIUS_M * 2:.2f}m / 靶薄板厚{TARGET_RADIUS_M * 2:.3f}m"),
                  "雷集合来源": f"官方8雷基线，视频确认{8 - len(mines_baseline_only)}/8"
                              + (f"，视频独有异常{mines_video_unmatched}" if mines_video_unmatched else "")},
    }
    (out_dir / "route_plan.json").write_text(
        json.dumps(route_plan, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[plan] 路线 {len(route)} 格 ({fallback_note}): {' → '.join(route)}")
    print(f"[plan] 排雷覆盖 {'✓ 8/8' if not missing else missing}"
          f" | 避障 {'✓' if not viol else viol}"
          f" | 视频确认雷 {8 - len(mines_baseline_only)}/8，仅基线 {mines_baseline_only}")

    # ---------------------------------------------------------------- 渲染
    from PIL import Image, ImageDraw, ImageFont
    CELL_PX, PAD = 60, 50
    W, H = PAD * 2 + COLS * CELL_PX, PAD * 2 + ROWS * CELL_PX
    img = Image.new("RGB", (W, H + 34), (30, 30, 30))
    dr = ImageDraw.Draw(img)
    try:
        f14 = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 14)
        f10 = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 10)
    except Exception:
        f14 = f10 = ImageFont.load_default()

    def xy(c: CellRef) -> tuple[int, int]:
        return PAD + c.col * CELL_PX, PAD + c.row * CELL_PX

    for r in range(ROWS + 1):
        dr.line([(PAD, PAD + r * CELL_PX), (PAD + COLS * CELL_PX, PAD + r * CELL_PX)], (80, 80, 80))
    for c in range(COLS + 1):
        dr.line([(PAD + c * CELL_PX, PAD), (PAD + c * CELL_PX, PAD + ROWS * CELL_PX)], (80, 80, 80))
    for c in range(COLS):
        dr.text((PAD + c * CELL_PX + CELL_PX // 2, PAD - 20), LETTERS[c], "white", f10, "mm")
    for r in range(ROWS):
        dr.text((PAD - 20, PAD + r * CELL_PX + CELL_PX // 2), str(r + 1), "white", f10, "mm")

    for name in obstacles:
        x, y = xy(cell(name))
        dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (180, 40, 40))
        dr.text((x + CELL_PX // 2, y + CELL_PX // 2), "障", "white", f14, "mm")
    for r in range(ROWS):
        name = f"{BRIDGE_COL}{r + 1}"
        x, y = xy(cell(name))
        if name in use_passable:
            dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (80, 60, 120))
            dr.text((x + CELL_PX // 2, y + CELL_PX // 2), "洞", "white", f14, "mm")
        else:
            dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (60, 60, 60))
            dr.text((x + CELL_PX // 2, y + CELL_PX // 2), "墙", (150, 150, 150), f10, "mm")
    for name in mines:
        x, y = xy(cell(name))
        dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], (220, 200, 30))
        dr.text((x + CELL_PX // 2, y + CELL_PX // 2 - 6), "雷", "black", f14, "mm")
        # 标注证据来源：视频实拍确认 vs 仅规则基线（视检优先级不同）
        if name in (mines_baseline_only or []):
            dr.text((x + CELL_PX // 2, y + CELL_PX - 10), "基线", (90, 60, 0), f10, "mm")
        else:
            dr.text((x + CELL_PX // 2, y + CELL_PX - 10), "✓拍", (20, 90, 20), f10, "mm")
    for t in targets:
        x, y = xy(t.cell)
        bg = (95, 75, 140) if t.is_blue else ((190, 90, 90) if t.is_blue is False else (110, 110, 120))
        dr.rectangle([x + 2, y + 2, x + CELL_PX - 2, y + CELL_PX - 2], bg)
        dr.text((x + CELL_PX // 2, y + CELL_PX // 2 - 6),
                "蓝" if t.is_blue else ("红" if t.is_blue is False else "未辨"),
                "white", f14, "mm")

    def grad(t: float) -> tuple[int, int, int]:
        if t < 0.5:
            return (int(255 * t * 2), 255, 0)
        return (255, int(255 * (1 - (t - 0.5) * 2)), 0)

    rc = [cell(n) for n in route]
    for i in range(len(rc) - 1):
        x1, y1 = xy(rc[i]); x2, y2 = xy(rc[i + 1])
        dr.line([(x1 + CELL_PX // 2, y1 + CELL_PX // 2), (x2 + CELL_PX // 2, y2 + CELL_PX // 2)],
                grad(i / max(len(rc) - 2, 1)), 4)
    cset = set(compact)
    for i, n in enumerate(route):
        x, y = xy(cell(n))
        cx, cy = x + CELL_PX // 2, y + CELL_PX // 2
        r = 7 if n in cset else 3
        col = grad(i / max(len(route) - 1, 1)) if n in cset else (150, 150, 150)
        dr.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col, outline="white")
    for name in cmds.shoot_cells:
        x, y = xy(cell(name))
        cx, cy = x + CELL_PX // 2, y + CELL_PX // 2
        dr.ellipse([cx - 11, cy - 11, cx + 11, cy + 11], outline=(255, 200, 0), width=2)
        dr.line([(cx - 6, cy - 6), (cx + 6, cy + 6)], (255, 200, 0), 2)
        dr.line([(cx - 6, cy + 6), (cx + 6, cy - 6)], (255, 200, 0), 2)
    # 不可行拐点（满舵圆弧扫掠撞实体）：标红叉 + 撞到的格号
    for b in turn_bad:
        x, y = xy(cell(b["拐点"]))
        cx, cy = x + CELL_PX // 2, y + CELL_PX // 2
        dr.ellipse([cx - 14, cy - 14, cx + 14, cy + 14], outline=(255, 60, 60), width=3)
        dr.text((x + 3, y + 2), "✗弯", (255, 90, 90), f10, "mm")
    sx, sy = xy(cell(use_start)); gx, gy = xy(cell(use_goal))
    dr.rectangle([sx + 4, sy + 4, sx + CELL_PX - 4, sy + CELL_PX - 4], outline=(0, 255, 0), width=3)
    dr.text((sx + CELL_PX // 2, sy + CELL_PX - 8), "起点", (0, 255, 0), f10, "mm")
    dr.rectangle([gx + 4, gy + 4, gx + CELL_PX - 4, gy + CELL_PX - 4], outline=(255, 100, 0), width=3)
    dr.text((gx + CELL_PX // 2, gy + CELL_PX - 8), "终点", (255, 100, 0), f10, "mm")

    dr.text((PAD, H - 18), f"绿→黄→红=路径先后  雷(✓拍=视频确认/基线=规则8雷)  障/墙/洞/靶=态势  ⊕=停靠射击  ✗弯=满舵扫掠撞实体(车跑不了)  规划:{fallback_note}", "white", f10)
    dr.text((PAD, H + 2), f"路线 {len(route)} 格 / 压缩 {len(compact)} 点 / 转向 {turns} 次 / 直线长 {length:.1f} 格"
            f"  起点{use_start}→终点{use_goal}"
            f"  转弯可行性:{'通过' if turn_ok else '✗' + str(len(turn_bad)) + '处'}", (255, 230, 130), f10)
    img.save(out_dir / "route_render.png")
    print(f"[render] {out_dir / 'route_render.png'}")
    print(f"[done] 全部产物在 {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
