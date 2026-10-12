"""lattice RANSAC 全场几何扩散 — 三重冗余管线第三环。

原理：场地是 14 列 (A-N) × 11 行 (1-11) 的规则网格；航拍近似正射下，
格号 (col,row) → 图像坐标 (u,v) 近似仿射模型（无人机任意航向、轻微透视均可表达）。

流程：YOLO v2 检测 → v4c 单点识别 → RANSAC 拟合仿射 lattice（高置信识别点主导）
    → 用 lattice 模型：
      1. 补全：识别失败的框用位置反推最近合法格号
      2. 校验：识别成功的框与 lattice 预测交叉比对（一致=确认 / 不一致=冲突上报）
      3. 去重：同格号多框只留检测置信度最高者
      4. 剔除：离格过远的框标记为疑似误检

评价：32 val 帧——v4c 单点覆盖率 vs lattice 扩散后覆盖率、冲突清单、
     拟合残差 RMS（v4c 识别内在几何一致性）。

用法：
    python tools/ml/lattice_ransac.py
"""
from __future__ import annotations

import json
import random
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
YOLO_WEIGHTS = ROOT / r"runs\detect\runs\detect\aerial_field_finetuned_v2\weights\best.pt"
CELL_WEIGHTS = ROOT / r"runs\cell_classifier_v4c\best.pt"
VAL_DIR = ROOT / r"capture\datasets\aerial_field_mixed\images\val"
OUT_DIR = ROOT / "runs\lattice_ransac"
OUT_DIR.mkdir(parents=True, exist_ok=True)

CELL_LABEL_CLS = 1
CONF_DET = 0.25
IOU_DET = 0.45
DEVICE = "cpu"
LETTERS = "ABCDEFGHIJKLMN"
MIN_FIT_POINTS = 4          # RANSAC 最少识别点数
RANSAC_ITERS = 300
HIGH_CONF = 0.70            # RANSAC 采样池：高置信识别点
CELL_ID_RE = __import__("re").compile(r"^([A-N])(\d{1,2})$")


def cell_to_idx(cell_id: str) -> tuple[int, int] | None:
    m = CELL_ID_RE.match(cell_id or "")
    if not m:
        return None
    col = LETTERS.index(m.group(1))
    row = int(m.group(2)) - 1
    if not (0 <= col <= 13 and 0 <= row <= 10):
        return None
    return col, row


def idx_to_cell(col: int, row: int) -> str:
    return f"{LETTERS[int(np.clip(col, 0, 13))]}{int(np.clip(row, 0, 10)) + 1}"


def fit_lattice(pts: np.ndarray):
    """pts: Nx4 (col,row,u,v)。投影变换 homography（正确建模航拍透视）。
    返回 3x3 H 或 None。"""
    src = pts[:, :2].astype(np.float32).reshape(-1, 1, 2)
    dst = pts[:, 2:4].astype(np.float32).reshape(-1, 1, 2)
    H, _ = cv2.findHomography(src, dst)
    return H if H is not None else None


def apply_lattice(H, col: float, row: float) -> tuple[float, float]:
    p = np.array([[[col, row]]], dtype=np.float32)
    q = cv2.perspectiveTransform(p, H)[0][0]
    return float(q[0]), float(q[1])


def invert_lattice(H, u: float, v: float) -> tuple[float, float]:
    Hinv = np.linalg.inv(H)
    p = np.array([[[u, v]]], dtype=np.float32)
    q = cv2.perspectiveTransform(p, Hinv)[0][0]
    return float(q[0]), float(q[1])


def _any3_collinear(pts4: np.ndarray) -> bool:
    """4 个 (col,row) 点中任意 3 点共线则拒绝采样。"""
    from itertools import combinations
    for a, b, c in combinations(range(4), 3):
        p, q, r = pts4[a], pts4[b], pts4[c]
        cross = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
        if abs(cross) < 0.5:
            return True
    return False


def ransac_lattice(points: np.ndarray, tol: float) -> tuple[np.ndarray | None, np.ndarray]:
    """RANSAC 拟合 (col,row)->(u,v) 投影变换。points: Nx4。
    返回 (H, inlier_mask)。"""
    n = len(points)
    best_mask = np.zeros(n, dtype=bool)
    best_model = None
    rng = random.Random(42)
    if n < 4:
        return None, best_mask
    for _ in range(RANSAC_ITERS):
        idx = rng.sample(range(n), 4)
        sample = points[idx]
        if _any3_collinear(sample[:, :2]):
            continue
        H = fit_lattice(sample)
        if H is None:
            continue
        # 残差
        src = points[:, :2].astype(np.float32).reshape(-1, 1, 2)
        pred = cv2.perspectiveTransform(src, H).reshape(-1, 2)
        res = np.hypot(points[:, 2] - pred[:, 0], points[:, 3] - pred[:, 1])
        mask = res < tol
        if mask.sum() > best_mask.sum():
            best_mask = mask
            best_model = H
    if best_model is None:
        return None, best_mask
    # 用全部内点最小二乘重拟合
    if best_mask.sum() >= 4:
        refined = fit_lattice(points[best_mask])
        if refined is not None:
            best_model = refined
    return best_model, best_mask


def process_frame(img_path: Path, yolo: "YOLO", recognizer: "CellIdRecognizer") -> dict:
    from PIL import Image
    results = yolo.predict(source=str(img_path), conf=CONF_DET, iou=IOU_DET,
                           classes=[CELL_LABEL_CLS], verbose=False, device=DEVICE)
    boxes = results[0].boxes
    img = Image.open(img_path).convert("RGB")
    dets: list[dict] = []
    if boxes is not None and len(boxes) > 0:
        crops = []
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            w = x2 - x1
            h = y2 - y1
            side = max(w, h) * 1.15
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            crop = img.crop((max(0, cx - side / 2), max(0, cy - side / 2),
                             min(img.width, cx + side / 2), min(img.height, cy + side / 2)))
            canvas = Image.new("RGB", (int(side), int(side)), (232, 232, 232))
            canvas.paste(crop, ((int(side) - crop.width) // 2, (int(side) - crop.height) // 2))
            crops.append(canvas)
            dets.append({"u": cx, "v": cy, "det_conf": float(box.conf[0]),
                         "box_w": w, "box_h": h})
        preds = recognizer.predict(crops)
        for d, p in zip(dets, preds):
            if p is not None:
                d["v4c_id"] = p.cell_id
                d["v4c_conf"] = p.joint_confidence
            else:
                d["v4c_id"] = None
                d["v4c_conf"] = 0.0

    # ---- RANSAC lattice 拟合 ----
    recs = []
    for i, d in enumerate(dets):
        idx = cell_to_idx(d.get("v4c_id"))
        if idx is not None:
            recs.append((idx[0], idx[1], d["u"], d["v"], i, d["v4c_conf"]))
    frame_stats = {
        "name": img_path.name, "detected": len(dets),
        "v4c_ok": len(recs), "model": None, "rms": None,
        "inliers": 0, "coverage_v4c": 0.0, "coverage_lattice": 0.0,
        "conflicts": [], "dup_groups": 0, "far_from_lattice": 0,
    }
    if len(recs) < MIN_FIT_POINTS:
        frame_stats["error"] = "insufficient_points"
        for d in dets:
            d["final_id"] = d.get("v4c_id")
            d["source"] = "v4c" if d.get("v4c_id") else "none"
        frame_stats["dets"] = dets
        return frame_stats

    pts = np.array([(r[0], r[1], r[2], r[3]) for r in recs])
    med_w = float(np.median([d["box_w"] for d in dets]))
    # 两遍拟合：宽 tol 找结构 → 残差收紧重拟合
    model, mask = ransac_lattice(pts, tol=med_w * 1.0)
    if model is None:
        frame_stats["error"] = "ransac_failed"
        for d in dets:
            d["final_id"] = d.get("v4c_id")
            d["source"] = "v4c" if d.get("v4c_id") else "none"
        frame_stats["dets"] = dets
        return frame_stats

    def _rms_of(mk):
        src = pts[mk, :2].astype(np.float32).reshape(-1, 1, 2)
        pred = cv2.perspectiveTransform(src, model).reshape(-1, 2)
        res = pts[mk, 2:4] - pred
        return float(np.sqrt((res ** 2).sum() / (2 * mk.sum())))

    if mask.sum() >= 4:
        rms = _rms_of(mask)
        tol2 = max(med_w * 0.25, 2.0 * rms)
        model2, mask2 = ransac_lattice(pts, tol=tol2)
        if model2 is not None and mask2.sum() >= mask.sum():
            model, mask = model2, mask2
            rms = _rms_of(mask)
    else:
        rms = None

    frame_stats["model"] = model.tolist()
    frame_stats["inliers"] = int(mask.sum())
    frame_stats["rms"] = round(rms, 2) if rms else None

    # ---- 应用 lattice：补全 / 校验 / 剔除 ----
    tol_final = max(med_w * 0.35, (rms or med_w * 0.5) * 2.5)
    inlier_rec_idx = {recs[i][4] for i in np.nonzero(mask)[0]}
    assigned: dict[str, list[int]] = {}
    for i, d in enumerate(dets):
        try:
            col_f, row_f = invert_lattice(model, d["u"], d["v"])
        except np.linalg.LinAlgError:
            col_f, row_f = -1, -1
        if col_f < 0:  # 退化
            d["final_id"] = d.get("v4c_id")
            d["source"] = "v4c" if d.get("v4c_id") else "none"
            continue
        pred_cell = idx_to_cell(int(round(col_f)), int(round(row_f)))
        pred_u, pred_v = apply_lattice(model, *cell_to_idx(pred_cell))
        dist = float(np.hypot(d["u"] - pred_u, d["v"] - pred_v))
        d["lattice_pred"] = pred_cell
        d["lattice_dist"] = round(dist, 1)
        if dist > tol_final:
            # 离 lattice 过远：疑似误检（YOLO 假框）或 lattice 不覆盖该区域
            d["final_id"] = d.get("v4c_id")
            d["source"] = "v4c" if d.get("v4c_id") else "rejected"
            if d.get("v4c_id"):
                frame_stats["far_from_lattice"] += 1
            continue
        if d.get("v4c_id"):
            if d["v4c_id"] == pred_cell:
                d["final_id"] = pred_cell
                d["source"] = "v4c+confirmed"
            else:
                # 冲突：内点识别 vs lattice 几何
                if i in inlier_rec_idx:
                    d["final_id"] = d["v4c_id"]
                    d["source"] = "v4c_conflict(lattice_inlier)"
                elif d["v4c_conf"] >= HIGH_CONF:
                    d["final_id"] = d["v4c_id"]
                    d["source"] = "v4c_conflict(high_conf)"
                else:
                    d["final_id"] = pred_cell
                    d["source"] = "lattice_corrected"
                frame_stats["conflicts"].append(
                    {"det_conf": round(d["det_conf"], 2), "v4c": d["v4c_id"],
                     "v4c_conf": round(d["v4c_conf"], 2), "lattice": pred_cell,
                     "resolved": d["final_id"]})
        else:
            # v4c 失败 → lattice 补全
            d["final_id"] = pred_cell
            d["source"] = "lattice_filled"
        if d.get("final_id"):
            assigned.setdefault(d["final_id"], []).append(i)

    # 去重：同格号多框 → 留 det_conf 最高
    for cell_id, idxs in assigned.items():
        if len(idxs) > 1:
            frame_stats["dup_groups"] += 1
            best = max(idxs, key=lambda j: dets[j]["det_conf"])
            for j in idxs:
                if j != best:
                    dets[j]["final_id"] = None
                    dets[j]["source"] = "dup_dropped"

    n_final = sum(1 for d in dets if d.get("final_id"))
    frame_stats["coverage_v4c"] = round(len(recs) / len(dets) * 100, 1)
    frame_stats["coverage_lattice"] = round(n_final / len(dets) * 100, 1)
    frame_stats["dets"] = dets
    return frame_stats


def main() -> None:
    from ultralytics import YOLO
    sys.path.insert(0, str(ROOT / "reference" / "onboard" / "dzuav" / "src"))
    from cell_classifier.predictor import CellIdRecognizer
    print("[1/3] 加载模型...")
    yolo = YOLO(str(YOLO_WEIGHTS))
    recognizer = CellIdRecognizer(str(CELL_WEIGHTS), device=DEVICE, half=False,
                                  rotation_tta=True, confidence_threshold=0.45)
    frame_files = sorted(VAL_DIR.glob("frame_*.jpg"))
    print(f"[2/3] 处理 {len(frame_files)} 帧...")

    all_frames: list[dict] = []
    totals = {"detected": 0, "v4c_ok": 0, "final_ok": 0,
              "conflicts": 0, "dup_groups": 0, "far": 0, "no_model": 0}
    for i, fp in enumerate(frame_files, 1):
        fs = process_frame(fp, yolo, recognizer)
        all_frames.append(fs)
        totals["detected"] += fs["detected"]
        totals["v4c_ok"] += fs["v4c_ok"]
        totals["final_ok"] += sum(1 for d in fs.get("dets", []) if d.get("final_id"))
        totals["conflicts"] += len(fs.get("conflicts", []))
        totals["dup_groups"] += fs.get("dup_groups", 0)
        totals["far"] += fs.get("far_from_lattice", 0)
        if "error" in fs:
            totals["no_model"] += 1
        print(f"  [{i}/{len(frame_files)}] {fs['name']}: 检出={fs['detected']} "
              f"v4c={fs['v4c_ok']} 最终={sum(1 for d in fs.get('dets', []) if d.get('final_id'))} "
              f"覆盖 {fs['coverage_v4c']}%→{fs['coverage_lattice']}% "
              f"冲突={len(fs.get('conflicts', []))} rms={fs.get('rms')}"
              + (f" [{fs['error']}]" if "error" in fs else ""))

    print("[3/3] 汇总...")
    summary = {
        "帧数": len(frame_files),
        "无法拟合帧数": totals["no_model"],
        "总检出": totals["detected"],
        "v4c单点识别": totals["v4c_ok"],
        "v4c覆盖率%": round(totals["v4c_ok"] / totals["detected"] * 100, 1),
        "lattice后赋格号": totals["final_ok"],
        "lattice覆盖率%": round(totals["final_ok"] / totals["detected"] * 100, 1),
        "v4c-vs-lattice冲突数": totals["conflicts"],
        "同格号重复组": totals["dup_groups"],
        "离格过远(疑似误检且v4c有输出)": totals["far"],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))

    out = OUT_DIR / "lattice_ransac_results.json"
    slim = []
    for fs in all_frames:
        slim.append({k: v for k, v in fs.items()})
    out.write_text(json.dumps({"summary": summary, "frames": slim},
                              ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"完成：{out}")

    # 冲突明细（前 20）
    print("\n--- 冲突明细（v4c vs lattice，前 20）---")
    n = 0
    for fs in all_frames:
        for c in fs.get("conflicts", []):
            n += 1
            if n <= 20:
                print(f"  {fs['name']}: {c}")


if __name__ == "__main__":
    main()
