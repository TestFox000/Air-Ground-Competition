"""lattice终裁：留一法验证分歧项 + 列线一致性检查 + 输出最终标签。

关键点:
  - #30(读N5/几何M5)、#55(读M7/几何N7)是仅有的分歧，且二者都在锚点集中
  - 留一法: 从锚点中剔除该框读数后重拟合，看其几何落点是否稳定
  - 列线检查: 每列的框按行号排布应平滑单调
输出: labels_124_final.json (含质量标记: dist<40 高置信, 40-70 中, >70 剔除)
"""
from __future__ import annotations

import json
from pathlib import Path
import numpy as np

OUT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\cell_recog_render\label")

READINGS = {
    0: "I2", 1: "N2", 2: "G2", 3: "F2", 4: "E2", 5: "K3", 6: "J3", 7: "D2",
    8: "M3", 9: "N3", 10: "H3", 11: None, 12: None, 13: None, 14: "F3",
    15: "E3", 16: "D3", 17: "K4", 18: "M4", 19: "J4", 20: "N4", 21: "I4",
    22: None, 23: "H4", 24: "G4", 25: None, 26: "F4", 27: "E4", 28: "D4",
    29: "K5", 30: "N5", 31: "C4", 32: "J5", 33: "I5", 34: "H5", 35: "B4",
    36: "G5", 37: "F5", 38: "E5", 39: "D5", 40: "C5", 41: "M6", 42: "N6",
    43: "K6", 44: "B5", 45: "J6", 46: "I6", 47: "A5", 48: "H6", 49: "G6",
    50: "F6", 51: "E6", 52: "D6", 53: "C6", 54: "B6", 55: "M7", 56: "K7",
    57: "J7", 58: "I7", 59: "H7", 60: "G7", 61: "F7", 62: "E7", 63: "D7",
    64: "C7", 65: "M8", 66: "K8", 67: "J8", 68: "I8", 69: "H8", 70: "G8",
    71: "F8", 72: "E8", 73: "D8", 74: None, 75: "N9", 76: "M9", 77: "K9",
    78: "D9", 79: "J9", 80: "E9", 81: "F9", 82: "I9", 83: "G9", 84: "M10",
    85: "E10", 86: "J10", 87: "H10",
}
LETTERS = "ABCDEFGHIJKLMN"
DISPUTED = [30, 55]  # 读数与几何分歧的框


def to_grid(cid):
    return float(LETTERS.index(cid[0])), float(int(cid[1:]) - 1)


def to_cid(col, row):
    return f"{LETTERS[int(col)]}{int(row) + 1}"


def design(g, order=3):
    c, r = g[:, 0], g[:, 1]
    return np.stack([np.ones_like(c), c, r, c * c, c * r, r * r,
                     c ** 3, c * c * r, c * r * r, r ** 3], 1)


def fit(fit_pts, fit_grid):
    X = design(fit_grid)
    cx, *_ = np.linalg.lstsq(X, fit_pts[:, 0], rcond=None)
    cy, *_ = np.linalg.lstsq(X, fit_pts[:, 1], rcond=None)
    return cx, cy


def project(cx, cy, grid_pts):
    X = design(grid_pts)
    return np.stack([X @ cx, X @ cy], 1)


def main():
    data = json.loads((OUT / "boxes_124.json").read_text(encoding="utf-8"))
    boxes = data["boxes"]
    centers = np.array([[b["cx"], b["cy"]] for b in boxes])
    n = len(boxes)

    idx = [i for i, v in READINGS.items() if v is not None and i not in DISPUTED]
    labs = [READINGS[i] for i in idx]
    grid = np.array([to_grid(l) for l in labs])
    pts = centers[idx]
    print(f"干净锚点数(剔除分歧项): {len(idx)}")

    cx, cy = fit(pts, grid)
    pred = project(cx, cy, grid)
    resid = np.linalg.norm(pred - pts, axis=1)
    print(f"拟合: med={np.median(resid):.2f}px p90={np.percentile(resid,90):.2f}px max={resid.max():.1f}px")

    # 留一验证分歧框
    grid_all = np.array([[c, r] for r in range(11) for c in range(14)], float)
    img_pos = project(cx, cy, grid_all)
    print("\n=== 分歧框留一裁决 ===")
    for i in DISPUTED:
        d = np.linalg.norm(img_pos - centers[i], axis=1)
        best3 = np.argsort(d)[:3]
        print(f"#{i} (读数={READINGS[i]}) center=({centers[i][0]:.0f},{centers[i][1]:.0f})")
        for g in best3:
            print(f"    -> {to_cid(*grid_all[g])}  {d[g]:.1f}px")

    # 全部88框分配（#13 与 #12 重复，跳过 #13）
    skip = {13}
    cand = []
    for i in range(n):
        if i in skip:
            continue
        d = np.linalg.norm(img_pos - centers[i], axis=1)
        for g in np.argsort(d)[:3]:
            cand.append((float(d[g]), i, int(g)))
    cand.sort()
    used_g, final = set(), {}
    for dist, i, g in cand:
        if i in final or g in used_g:
            continue
        if dist > 80:
            continue
        used_g.add(g)
        final[i] = (to_cid(*grid_all[g]), round(dist, 1))

    # 列线检查
    print("\n=== 列线一致性 ===")
    for li, letter in enumerate(LETTERS):
        cells = sorted((int(v[0][1:]) - 1, k, v[1]) for k, v in final.items()
                       if v[0][0] == letter)
        if cells:
            print(f"{letter}: " + " ".join(f"r{r}(#{k},{d}px)" for r, k, d in cells))

    # 与读数比对
    mismatch = [(i, READINGS[i], final[i]) for i in sorted(final)
                if READINGS.get(i) and READINGS[i] != final[i][0]]
    print(f"\n读数 vs 几何 不一致: {len(mismatch)}")
    for i, r, (cid, d) in mismatch:
        print(f"  #{i}: 读数={r} 几何={cid} ({d}px)  -> 采信几何")

    unsure = [(i, final[i]) for i in sorted(final) if READINGS.get(i) is None]
    print(f"存疑框几何分配: {len(unsure)}")
    for i, (cid, d) in unsure:
        print(f"  #{i}: -> {cid} ({d}px)")

    # 质量分档
    print("\n=== 质量分档 ===")
    q_hi = [i for i, (c, d) in final.items() if d < 40]
    q_mid = [i for i, (c, d) in final.items() if 40 <= d <= 70]
    q_bad = [i for i, (c, d) in final.items() if d > 70]
    print(f"高置信(<40px): {len(q_hi)}  中(40-70): {len(q_mid)}  剔除(>70): {len(q_bad)} -> {q_bad}")

    out = {str(i): {"cell_id": final[i][0], "dist": final[i][1],
                    "quality": "hi" if final[i][1] < 40 else ("mid" if final[i][1] <= 70 else "bad")}
           for i in sorted(final)}
    (OUT / "labels_124_final.json").write_text(json.dumps(out, ensure_ascii=False, indent=1),
                                               encoding="utf-8")
    print(f"\n最终标签 {len(final)} 框已写 labels_124_final.json (skip #13 重复框)")


if __name__ == "__main__":
    main()
