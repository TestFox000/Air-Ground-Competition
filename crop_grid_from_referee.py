"""从裁判系统截图用霍夫变换检测网格并切出格号小图。"""
from pathlib import Path
import cv2
import numpy as np
from PIL import Image

def imwrite_utf8(path, img):
    ext = Path(path).suffix
    ok, buf = cv2.imencode(ext, img)
    if ok:
        buf.tofile(str(path))
    return ok

SRC = Path(r"c:\Users\a\AppData\Roaming\Trae CN\User\workspaceStorage\fd6bd231d0069eab23d9ffa705670886\paste-files\e3429002-342a-4280-beef-4d0b338f9e4c_image.png")
OUT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\datasets\cell_labels")
CROPS = OUT / "crops"
CROPS.mkdir(parents=True, exist_ok=True)

COLS, ROWS = 14, 11
LETTERS = "ABCDEFGHIJKLMN"

img = cv2.imread(str(SRC))
H, W = img.shape[:2]
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# 边缘检测
edges = cv2.Canny(gray, 50, 150, apertureSize=3)

# 霍夫直线检测
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100,
                        minLineLength=W // 4, maxLineGap=20)

hy = set()
vx = set()
if lines is not None:
    for line in lines:
        coords = line[0] if line.ndim > 1 else line
        x1, y1, x2, y2 = int(coords[0]), int(coords[1]), int(coords[2]), int(coords[3])
        angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
        # 水平线
        if abs(angle) < 5 or abs(angle) > 175:
            hy.add((y1 + y2) // 2)
        # 垂直线
        elif 85 < abs(angle) < 95:
            vx.add((x1 + x2) // 2)

hy = sorted(hy)
vx = sorted(vx)
print(f"霍夫检测水平线: {len(hy)} 条")
print(f"霍夫检测垂直线: {len(vx)} 条")

# 聚类合并相近的线（间距 < 20 像素的合并）
def cluster(lines, tol=20):
    if not lines:
        return []
    clusters = [[lines[0]]]
    for v in lines[1:]:
        if v - clusters[-1][-1] < tol:
            clusters[-1].append(v)
        else:
            clusters.append([v])
    return [int(np.mean(c)) for c in clusters]

hy = cluster(hy)
vx = cluster(vx)
print(f"聚类后水平线: {len(hy)} 条, {hy}")
print(f"聚类后垂直线: {len(vx)} 条, {vx}")

# 取网格区域：水平线应该有 ROWS+1=12 条，垂直线 COLS+1=15 条
# 如果多了，取最密集的连续段
def select_grid_lines(lines, expected):
    if len(lines) < expected:
        return None
    # 找连续 expected 条线，间距最均匀的
    best = None
    best_score = float("inf")
    for i in range(len(lines) - expected + 1):
        seg = lines[i:i + expected]
        diffs = np.diff(seg)
        score = np.std(diffs)
        if score < best_score:
            best_score = score
            best = seg
    return best

hy_sel = select_grid_lines(hy, ROWS + 1)
vx_sel = select_grid_lines(vx, COLS + 1)

if hy_sel is None or vx_sel is None:
    print("网格线检测不足，用兜底均匀分布")
    # 用检测到的最外边界
    y0, y1 = (hy[0], hy[-1]) if len(hy) >= 2 else (int(H * 0.05), int(H * 0.95))
    x0, x1 = (vx[0], vx[-1]) if len(vx) >= 2 else (int(W * 0.05), int(W * 0.95))
    hy_sel = [int(y0 + i * (y1 - y0) / ROWS) for i in range(ROWS + 1)]
    vx_sel = [int(x0 + i * (x1 - x0) / COLS) for i in range(COLS + 1)]

print(f"最终水平线: {hy_sel}")
print(f"最终垂直线: {vx_sel}")

# 可视化
vis = img.copy()
for y in hy_sel:
    cv2.line(vis, (0, y), (W, y), (0, 255, 0), 2)
for x in vx_sel:
    cv2.line(vis, (x, 0), (x, H), (0, 0, 255), 2)
imwrite_utf8(str(OUT / "grid_detection.png"), vis)

# 切割
records = []
for row in range(ROWS):
    for col in range(COLS):
        x1, x2 = vx_sel[col], vx_sel[col + 1]
        y1, y2 = hy_sel[row], hy_sel[row + 1]
        cell_id = f"{LETTERS[col]}{row + 1}"
        crop = img[y1:y2, x1:x2]
        crop = cv2.resize(crop, (96, 96), interpolation=cv2.INTER_AREA)
        crop_path = f"crops/{cell_id}.png"
        imwrite_utf8(str(OUT / crop_path), crop)
        records.append({
            "crop_path": crop_path,
            "split": "train",
            "cell_id": cell_id,
            "column": LETTERS[col],
            "row": str(row + 1),
            "status": "labeled",
        })

import csv
csv_path = OUT / "annotations.csv"
with csv_path.open("w", newline="", encoding="utf-8-sig") as f:
    w = csv.DictWriter(f, fieldnames=["crop_path", "split", "cell_id", "column", "row", "status"])
    w.writeheader()
    w.writerows(records)

print(f"\n切割完成: {len(records)} 张")
print(f"可视化: {OUT / 'grid_detection.png'}")
