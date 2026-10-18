"""把标签里的临时橙格（类 12 orange_raw）自动细分为 4 类，免去人工改类。

依据：4 个橙格固定印刷在场地上，每个格子四周印有自己的「列字母 + 行号」
小标签（类 1 cell_label，框已准确）。
    智能车终点 N5  → 类 5 car_end
    无人机起点 N8  → 类 6 uav_start
    智能车起点 N10 → 类 4 car_start
    无人机终点 M2  → 类 7 uav_end（H 标志格）
做法：对每个橙框，用官方 cell_classifier 批量读取邻近格号，做距离加权投票
得到所在格坐标，再按固定表映射。投票需要的邻域读到票数不足、或结果不在
4 个已知格时保留类 12（输出报告，人工兜底）。

用法：
    python tools/ml/classify_orange.py --images capture/aerial_field/images \\
        --labels capture/aerial_field/labels --cell-model models/release/cell_classifier.onnx
    python tools/ml/classify_orange.py ... --limit 1   # 试跑不写文件
退出码：0 成功；2 依赖/参数。
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("需要 opencv-python / numpy", file=sys.stderr)
    raise SystemExit(2)


# 固定格（letter 0-based, row 0-based）→ 最终类号
ORANGE_LOOKUP = {
    (13, 4): 5,   # N5 智能车终点
    (13, 7): 6,   # N8 无人机起点
    (13, 9): 4,   # N10 智能车起点
    (12, 1): 7,   # M2 无人机终点
}
LETTERS = "ABCDEFGHIJKLMN"
ORANGE_CLS = 12
CELL_CLS = 1


# ---------------------------------------------------------------- 纯函数（可测）

def median_pairwise_spacing(points: np.ndarray) -> float:
    """点集两两距离中 > 30px 的中位数，作为典型格距（纯 numpy）。"""
    distances = []
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            d = float(np.linalg.norm(points[i] - points[j]))
            if d > 30:
                distances.append(d)
    if not distances:
        return 200.0
    return float(np.median(distances))


def weighted_vote(orange_center, cell_points, reads, radius: float) -> Counter:
    """邻域格号读取做距离加权投票；返回 (letter,row) → 权重 的 Counter。"""
    votes: Counter = Counter()
    for point, read in zip(cell_points, reads):
        dist = float(np.linalg.norm(point - orange_center))
        if dist > radius:
            continue
        weight = max(0.05, 1 - dist / radius)
        votes[read] += weight
    return votes


def map_orange(votes: Counter, min_weight: float = 0.5):
    """投票最高项映射类号。返回 (类号或 None, 格描述, 最高票)。"""
    if not votes:
        return None, "-", 0.0
    (letter, row), weight = votes.most_common(1)[0]
    if weight < min_weight or (letter, row) not in ORANGE_LOOKUP:
        return None, f"{LETTERS[letter]}{row + 1}", float(weight)
    return ORANGE_LOOKUP[(letter, row)], f"{LETTERS[letter]}{row + 1}", float(weight)


def classify_cell_crop(image, cx, cy, w, h, size: int = 96) -> np.ndarray:
    """单个格号框 → 96×96 padded blob（RGB CHW，0~1）。"""
    x1, y1 = max(0, int(cx - w / 2)), max(0, int(cy - h / 2))
    x2, y2 = int(cx + w / 2), int(cy + h / 2)
    crop = image[y1:y2, x1:x2]
    side = max(crop.shape[:2])
    canvas = np.full((side, side, 3), 114, dtype=np.uint8)
    dy, dx = (side - crop.shape[0]) // 2, (side - crop.shape[1]) // 2
    canvas[dy:dy + crop.shape[0], dx:dx + crop.shape[1]] = crop
    resized = cv2.resize(canvas, (size, size))
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    return rgb.transpose(2, 0, 1).astype(np.float32) / 255


# ---------------------------------------------------------------- 主流程

def classify_frame(image, label_lines, session, input_name):
    """返回 (新标签行列表, 橙格报告列表)。"""
    h, w = image.shape[:2]
    cells, orange, kept = [], [], []
    for line in label_lines:
        values = list(map(float, line.split()))
        cls = int(values[0])
        if cls == CELL_CLS:
            cells.append(values[1:5])
        elif cls == ORANGE_CLS:
            orange.append(values[1:5])
        if cls != ORANGE_CLS:
            kept.append(line)

    if not cells:
        return label_lines, [("no_cell", "-", 0.0) for _ in orange]

    blobs, cell_points = [], []
    for c in cells:
        cx, cy, bw, bh_ = c[0] * w, c[1] * h, c[2] * w, c[3] * h
        blobs.append(classify_cell_crop(image, cx, cy, bw, bh_))
        cell_points.append(np.array([cx, cy]))
    cell_points = np.array(cell_points)

    output = session.run(None, {input_name: np.array(blobs)})
    reads = [(int(output[0][i].argmax()), int(output[1][i].argmax()))
             for i in range(len(cells))]

    spacing = median_pairwise_spacing(cell_points)
    radius = spacing * 1.6

    reports = []
    for o in orange:
        center = np.array([o[0] * w, o[1] * h])
        votes = weighted_vote(center, cell_points, reads, radius)
        cls_id, desc, weight = map_orange(votes)
        coords = " ".join(f"{v:.6f}" for v in o)
        if cls_id is None:
            kept.append(f"{ORANGE_CLS} {coords}")
        else:
            kept.append(f"{cls_id} {coords}")
        reports.append((cls_id, desc, weight))
    return kept, reports


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", required=True)
    parser.add_argument("--labels", required=True)
    parser.add_argument("--cell-model", default="models/release/cell_classifier.onnx")
    parser.add_argument("--limit", type=int, help="只处理前 N 帧且不写文件（试跑）")
    args = parser.parse_args()

    import onnxruntime as ort

    session = ort.InferenceSession(args.cell_model, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    image_paths = sorted(Path(args.images).glob("*.jpg"))
    if args.limit:
        image_paths = image_paths[: args.limit]

    totals = Counter()
    unresolved = []
    for path in image_paths:
        image = cv2.imread(str(path))
        label_path = Path(args.labels) / (path.stem + ".txt")
        lines = label_path.read_text(
            encoding="utf-8").splitlines() if label_path.exists() else []
        new_lines, reports = classify_frame(image, lines, session, input_name)
        for cls_id, desc, weight in reports:
            if cls_id is None:
                totals["unresolved"] += 1
                unresolved.append(f"{path.name}:{desc}({weight:.2f})")
            else:
                totals[cls_id] += 1
        if not args.limit:
            label_path.write_text("\n".join(new_lines) + "\n", encoding="utf-8")

    print("分类结果："
          f"car_start={totals[4]} car_end={totals[5]} "
          f"uav_start={totals[6]} uav_end={totals[7]} "
          f"未决={totals['unresolved']}")
    if unresolved:
        print("未决（保留类12，需人工）:")
        for item in unresolved[:20]:
            print("  " + item)
    if args.limit:
        print("[试跑] 未写文件")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
