"""参数化 lattice 终裁: 读数 -> RANSAC拟合 -> 留一裁决分歧 -> 几何扩散全帧 -> 质量分档。

用法: python fit_lattice_frame.py frame_00170
输出: runs/cell_recog_render/label/{frame}/labels_final.json
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
LETTERS = "ABCDEFGHIJKLMN"

# frame_00170 初读 (73框, 存疑项将由几何裁决)
READINGS_170 = {
    0: None, 1: "H7", 2: "G7", 3: "C8", 4: "E7", 5: "B8", 6: "E8", 7: "A8",
    8: "D7", 9: "K9", 10: "J6", 11: "I6", 12: "H6", 13: "C7", 14: "G6",
    15: "B6", 16: "F6", 17: "A7", 18: "E6", 19: "D6", 20: "C6", 21: "K5",
    22: "I5", 23: "H5", 24: "B5", 25: "G5", 26: "A5", 27: "F5", 28: "E5",
    29: "D5", 30: "C5", 31: "B5", 32: "K4", 33: "J4", 34: "A4", 35: "I4",
    36: "H4", 37: "G4", 38: "F4", 39: "E4", 40: "D4", 41: "C4", 42: "B4",
    43: "A4", 44: "K3", 45: "J3", 46: "I3", 47: "H3", 48: "G3", 49: "F3",
    50: "E3", 51: "D3", 52: "C3", 53: "B3", 54: "A3", 55: "K2", 56: "J2",
    57: "I2", 58: "H2", 59: "B2", 60: "H2", 61: "C2", 62: "G2", 63: "D2",
    64: "E2", 65: "F2", 66: "K1", 67: "B1", 68: "C1", 69: "J1", 70: "D1",
    71: "H1", 72: "G1",
}

# frame_00099 初读 (33框): #12 G9-vs-G10、#19 H8-vs-H9、#30-32 低置信, 由几何裁决
READINGS_99 = {
    0: "E9", 1: "E8", 2: "E7", 3: "E6",
    4: "F11", 5: "F10", 6: "F9", 7: "F8", 8: "F7", 9: "F6", 10: "F5",
    11: "G11", 12: "G9", 13: "G8", 14: "G7", 15: "G6", 16: "G5",
    17: "H11", 18: "H10", 19: "H8", 20: "H7", 21: "H6",
    22: "I11", 23: "I10", 24: "I9", 25: "I8", 26: "I7", 27: "I6",
    28: "J11", 29: "J10", 30: "J9", 31: "J8", 32: "J7",
}

# frame_00250 初读 (22框): #8 J1/#18 I3/#19 H3 模糊, #6 H1 vs G1 存疑, 由几何裁决
READINGS_250 = {
    0: "A1", 1: "B1", 2: "C1", 3: "D1", 4: "E1", 5: "F1",
    6: "H1", 7: "I1", 8: "J1", 9: "A2",
    10: "C2", 11: "D2", 12: "E2", 13: "F2", 14: "G2", 15: "H2",
    16: "A3", 17: "B3", 18: "I3", 19: "H3",
    20: "D3", 21: "E3",
}

# frame_00020 初读 (18框): 东侧 I-N 列, J/K 列 GT 漏检, #2 G1/#17 M5 存疑
READINGS_20 = {
    0: "I1", 1: "H1", 2: "G1", 3: "L1", 4: "M1", 5: "N1",
    6: "I2", 7: "L2", 8: "N2",
    9: "H3", 10: "I3", 11: "L3", 12: "M3", 13: "N3",
    14: "L4", 15: "M4", 16: "N4", 17: "M5",
}

READINGS_MAP = {"frame_00170": READINGS_170, "frame_00099": READINGS_99,
                "frame_00250": READINGS_250, "frame_00020": READINGS_20}


def to_grid(cid):
    return float(LETTERS.index(cid[0])), float(int(cid[1:]) - 1)


def to_cid(col, row):
    return f"{LETTERS[int(col)]}{int(row) + 1}"


def design(g, order=3):
    c, r = g[:, 0], g[:, 1]
    return np.stack([np.ones_like(c), c, r, c * c, c * r, r * r,
                     c ** 3, c * c * r, c * r * r, r ** 3], 1)[:, :(
        10 if order == 3 else 6)]


def fit(fit_pts, fit_grid, order=3):
    X = design(fit_grid, order)
    cx, *_ = np.linalg.lstsq(X, fit_pts[:, 0], rcond=None)
    cy, *_ = np.linalg.lstsq(X, fit_pts[:, 1], rcond=None)
    return cx, cy


def main():
    frame = sys.argv[1] if len(sys.argv) > 1 else "frame_00170"
    readings = READINGS_MAP.get(frame)
    if readings is None:
        raise SystemExit(f"未定义 {frame} 的 READINGS")

    out_dir = ROOT / "runs/cell_recog_render/label" / frame
    data = json.loads((out_dir / "boxes.json").read_text(encoding="utf-8"))
    boxes = data["boxes"]
    centers = np.array([[b["cx"], b["cy"]] for b in boxes])
    n = len(boxes)
    print(f"{frame}: {n} 框")

    # RANSAC: 拟合锚点集 (读数非None的)
    idx = [i for i, v in readings.items() if v is not None]
    pts = centers[idx]
    grid = np.array([to_grid(readings[i]) for i in idx])
    rng = np.random.default_rng(7)
    best_inl = None
    for _ in range(200):
        sel = rng.choice(len(idx), size=10, replace=False)
        try:
            cx, cy = fit(pts[sel], grid[sel], 2)
        except Exception:
            continue
        pred = np.stack([design(grid, 2) @ cx, design(grid, 2) @ cy], 1)
        inl = np.linalg.norm(pred - pts, axis=1) < 30
        if best_inl is None or inl.sum() > best_inl.sum():
            best_inl = inl
    print(f"RANSAC 内点: {best_inl.sum()}/{len(idx)}")
    keep_idx = [idx[k] for k in range(len(idx)) if best_inl[k]]
    dropped = [(idx[k], readings[idx[k]]) for k in range(len(idx)) if not best_inl[k]]
    print(f"剔除读数: {dropped}")

    # 内点重拟合 order=3
    pi = centers[keep_idx]
    gi = np.array([to_grid(readings[i]) for i in keep_idx])
    for _ in range(3):
        cx, cy = fit(pi, gi, 3)
        pred = np.stack([design(gi, 3) @ cx, design(gi, 3) @ cy], 1)
        resid = np.linalg.norm(pred - pi, axis=1)
        ok = resid < 30
        if ok.all() or ok.sum() < 14:
            break
        pi, gi, keep_idx = pi[ok], gi[ok], [keep_idx[k] for k in range(len(keep_idx)) if ok[k]]
    print(f"最终拟合锚点: {len(keep_idx)}, med残差 {np.median(resid):.1f}px")

    # 几何分配全部框
    grid_all = np.array([[c, r] for r in range(11) for c in range(14)], float)
    img_pos = np.stack([design(grid_all, 3) @ cx, design(grid_all, 3) @ cy], 1)
    cand = []
    for i in range(n):
        d = np.linalg.norm(img_pos - centers[i], axis=1)
        for g in np.argsort(d)[:3]:
            cand.append((float(d[g]), i, int(g)))
    cand.sort()
    used, final = set(), {}
    for dist, i, g in cand:
        if i in final or g in used or dist > 70:
            continue
        used.add(g)
        final[i] = (to_cid(*grid_all[g]), round(dist, 1))

    # 读数 vs 几何
    mismatch = [(i, readings[i], final[i]) for i in sorted(final)
                if readings.get(i) and readings[i] != final[i][0]]
    print(f"\n读数与几何不一致: {len(mismatch)}")
    for i, rd, (asg, d) in mismatch:
        print(f"  #{i}: 读数={rd} 几何={asg} dist={d}")

    # 输出
    out = {}
    for i in sorted(final):
        cid, d = final[i]
        rd = readings.get(i)
        quality = "hi" if d < 40 else ("mid" if d < 55 else "low")
        if rd and rd != cid and d > 40:
            quality = "bad"  # 读数几何冲突且远 -> 剔除
        out[str(i)] = {"cell_id": cid, "dist": d, "quality": quality,
                       "reading": rd, "agree": rd == cid if rd else None}
    hi = sum(1 for v in out.values() if v["quality"] == "hi")
    mid = sum(1 for v in out.values() if v["quality"] == "mid")
    bad = sum(1 for v in out.values() if v["quality"] == "bad")
    print(f"\n分档: hi={hi} mid={mid} low={n - hi - mid - bad} bad={bad} 未分配={n - len(final)}")
    (out_dir / "labels_final.json").write_text(
        json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"已写 {out_dir / 'labels_final.json'}")


if __name__ == "__main__":
    main()
