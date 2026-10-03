"""跨帧单应性传播：自动消解未决橙格与断续误检，替代人工校正。

场地是平面、无人机航拍 → 相邻帧间存在单应性。用图像特征(ORB+RANSAC)
建立帧链，而不是依赖检测框重叠（2fps 下无人机移动快，裸 IoU 关联率低）。

三个动作：
    1. identity：未决橙格(类12)沿帧链映射到已分类帧(类4-7)，IoU 命中
       投票定类；完全无票 → 删除该框（读不出且无法关联，宁缺毋错）。
    2. clean：mine/arrow/bucket 检出经单应性映射后 ±2 帧均无同类支持
       → 判误检删除（灯光反光、图案误检在时序上不连续）。
    3. clamp：bridge 越界框平移回画面内（由 auto_supplement 的
       entrance_from_arrow clamp 保证，本工具只统计不重算）。

用法（默认 dry-run 只统计，--apply 才写回）：
    python tools/ml/propagate_labels.py --images capture/aerial_field/images \\
        --labels capture/aerial_field/labels [--apply]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("需要 opencv-python / numpy", file=sys.stderr)
    raise SystemExit(2)


ORANGE_TMP_CLS = 12
ORANGE_FINAL = {4, 5, 6, 7}
CLEAN_CLS = {0, 2, 8, 9, 10}  # mine arrow bucket_*
MAX_HOP = 30          # 橙格传播最多跨 30 帧；有中心距离验证兜底，链漂移只会漏配不会错配
CENTER_DIST = 0.25    # 橙格中心距离命中阈值（4 个物理橙格彼此 >0.3，0.25 内必是同格）
SUPPORT_IOU = 0.25    # 断续检出支持阈值
FEATURE_WIDTH = 960   # ORB 在 1/4 宽度上做，速度优先


# ---------------------------------------------------------------- 纯函数（可测）

def box_iou(a, b) -> float:
    ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2
    ax2, ay2 = a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2
    bx2, by2 = b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = ix * iy
    return inter / (a[2] * a[3] + b[2] * b[3] - inter + 1e-9)


def map_box(H, box):
    """归一化框四角透视变换 → 轴对齐归一化框；退化/出画返回 None。

    中心点变换后落在画面外 → 认为链已不可用。
    """
    x1, y1 = box[0] - box[2] / 2, box[1] - box[3] / 2
    x2, y2 = box[0] + box[2] / 2, box[1] + box[3] / 2
    corners = np.array([[[x1, y1], [x2, y1], [x2, y2], [x1, y2]]],
                       dtype=np.float64)
    out = cv2.perspectiveTransform(corners, H)[0]
    if not np.all(np.isfinite(out)):
        return None
    cx = float(out[:, 0].mean())
    cy = float(out[:, 1].mean())
    if not (0 < cx < 1 and 0 < cy < 1):
        return None
    bw = float(out[:, 0].max() - out[:, 0].min())
    bh = float(out[:, 1].max() - out[:, 1].min())
    if bw <= 0 or bh <= 0:
        return None
    return (cx, cy, bw, bh)


def tally_votes(votes: dict[int, int]):
    """票决：最高票唯一且 >=2 票 → 该类；1 票也返回但标记弱。

    返回 (cls|None, strong:bool)。
    """
    if not votes:
        return None, False
    best = max(votes.values())
    winners = [c for c, n in votes.items() if n == best]
    if len(winners) != 1:
        return None, False
    return winners[0], best >= 2


def homography_between(prev_gray, next_gray, orb, matcher):
    """相邻帧单应性（归一化坐标系，帧坐标 → 目标帧坐标）；不可靠返回 None。"""
    h_px, w_px = prev_gray.shape[:2]
    scale = FEATURE_WIDTH / w_px
    small_p = cv2.resize(prev_gray, None, fx=scale, fy=scale)
    small_n = cv2.resize(next_gray, None, fx=scale, fy=scale)
    kp1, des1 = orb.detectAndCompute(small_p, None)
    kp2, des2 = orb.detectAndCompute(small_n, None)
    if des1 is None or des2 is None or len(kp1) < 30 or len(kp2) < 30:
        return None
    matches = matcher.knnMatch(des1, des2, k=2)
    good = [m for m, n in (p for p in matches if len(p) == 2)
            if m.distance < 0.75 * n.distance]
    if len(good) < 30:
        return None
    src = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
    if H is None or int(mask.sum()) < 20:
        return None
    # 半分辨率 → 原像素 → 归一化坐标（map_box 直接吃归一化框）
    S = np.diag([1 / scale, 1 / scale, 1.0])
    N = np.diag([1 / w_px, 1 / h_px, 1.0])
    return N @ S @ H @ np.linalg.inv(S) @ np.linalg.inv(N)


def load_labels(path: Path):
    rows = []
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            parts = line.split()
            if len(parts) >= 5:
                try:
                    v = list(map(float, parts))
                except ValueError:
                    continue
                rows.append((int(v[0]), tuple(v[1:5])))
    return rows


# ---------------------------------------------------------------- 主流程

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", required=True)
    parser.add_argument("--labels", required=True)
    parser.add_argument("--apply", action="store_true",
                        help="写回标签文件（默认 dry-run 只统计）")
    args = parser.parse_args()

    images_dir, labels_dir = Path(args.images), Path(args.labels)
    frames = sorted(images_dir.glob("*.jpg"))
    if not frames:
        print("没有找到帧", file=sys.stderr)
        return 2

    all_rows = {p.stem: load_labels(labels_dir / f"{p.stem}.txt") for p in frames}
    stems = [p.stem for p in frames]

    # ---- 1. 相邻帧单应性链
    orb = cv2.ORB_create(3000)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    H_fwd = [None] * (len(stems) - 1)   # H_fwd[i]: 帧 i → 帧 i+1
    for i in range(len(stems) - 1):
        g1 = cv2.imread(str(frames[i]), cv2.IMREAD_GRAYSCALE)
        g2 = cv2.imread(str(frames[i + 1]), cv2.IMREAD_GRAYSCALE)
        H_fwd[i] = homography_between(g1, g2, orb, matcher)
    ok = sum(h is not None for h in H_fwd)
    print(f"单应性链：{ok}/{len(H_fwd)} 对相邻帧可关联")

    def chain_H(i, j):
        """帧 i → 帧 j 的累积单应性（|j-i|<=MAX_HOP，链断返回 None）。

        前向 p_j = H[j-1]…H[i]·p_i（逐级左乘）；后向逐级左乘逆矩阵。
        """
        if abs(j - i) > MAX_HOP or i == j:
            return np.eye(3)
        H = np.eye(3)
        if j > i:
            for k in range(i, j):
                if H_fwd[k] is None:
                    return None
                H = H_fwd[k] @ H
        else:
            for k in range(i - 1, j - 1, -1):
                if H_fwd[k] is None:
                    return None
                H = np.linalg.inv(H_fwd[k]) @ H
        return H

    # ---- 2. 橙格身份传播（中心距离票决）
    # 场地 4 个物理橙格彼此相距 >0.3（归一化）：映射位置 0.25 内的最近已分
    # 橙格必是同一物理格，类别由多帧票决；错配需链漂移恰好跨过格间距，极难发生。
    resolved = {"strong": 0, "weak": 0, "dropped": 0}
    changes = {}  # stem -> list[(idx, new_cls|None)]
    for i, stem in enumerate(stems):
        undecided = [(k, b) for k, (c, b) in enumerate(all_rows[stem])
                     if c == ORANGE_TMP_CLS]
        if not undecided:
            continue
        for idx, box in undecided:
            votes: dict[int, int] = {}
            for j in range(max(0, i - MAX_HOP), min(len(stems), i + MAX_HOP + 1)):
                if j == i:
                    continue
                H = chain_H(i, j)
                if H is None:
                    continue
                mapped = map_box(H, box)
                if mapped is None:
                    continue
                best_d, best_c = CENTER_DIST, None
                for c2, b2 in all_rows[stems[j]]:
                    if c2 in ORANGE_FINAL:
                        d = float(np.hypot(mapped[0] - b2[0], mapped[1] - b2[1]))
                        if d < best_d:
                            best_d, best_c = d, c2
                if best_c is not None:
                    votes[best_c] = votes.get(best_c, 0) + 1
            cls, strong = tally_votes(votes)
            if cls is None:
                resolved["dropped"] += 1
            else:
                resolved["strong" if strong else "weak"] += 1
            changes.setdefault(stem, []).append((idx, cls))

    # ---- 3. 断续检出清理
    lone_removed = 0
    for i, stem in enumerate(stems):
        for idx, (c, b) in enumerate(all_rows[stem]):
            if c not in CLEAN_CLS:
                continue
            supported = False
            for j in (i - 2, i - 1, i + 1, i + 2):
                if not 0 <= j < len(stems) or j == i:
                    continue
                H = chain_H(i, j)
                if H is None:
                    continue
                mapped = map_box(H, b)
                if mapped is None:
                    continue
                if any(box_iou(mapped, b2) > SUPPORT_IOU
                       for c2, b2 in all_rows[stems[j]] if c2 == c):
                    supported = True
                    break
            if not supported:
                lone_removed += 1
                changes.setdefault(stem, []).append((idx, None))

    print(f"橙格传播：强证据 {resolved['strong']}，弱证据 {resolved['weak']}，"
          f"无票删除 {resolved['dropped']}")
    print(f"断续检出清理：删除 {lone_removed} 个孤立框")

    if not args.apply:
        print("dry-run 完成，未写回。加 --apply 生效。")
        return 0

    # ---- 4. 写回（同帧内先删后改，索引倒序安全）
    written = 0
    for stem, ops in changes.items():
        rows = all_rows[stem]
        for idx, new_cls in sorted(ops, key=lambda t: -t[0]):
            if new_cls is None:
                del rows[idx]
            else:
                rows[idx] = (new_cls, rows[idx][1])
        text = "\n".join(f"{c} " + " ".join(f"{v:.6f}" for v in b)
                         for c, b in rows) + ("\n" if rows else "")
        (labels_dir / f"{stem}.txt").write_text(text, encoding="utf-8")
        written += 1
    print(f"已写回 {written} 个标签文件")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
