# -*- coding: utf-8 -*-
"""侦察缓存 → YOLO 13 类数据集（d13 微调用，在 Orin 上跑）。

背景（2026-10-23 尸检结论）：d13 在比赛航拍域单帧召回只有 10~20%，门控再
聪明也是薄证据裁决。治本 = 用实拍域数据微调。本工具把侦察缓存变成训练集：

  1. 伪标签：frames_meta.json 里已落盘的像素框（识别时检测出的目标），
     conf >= --conf 才收——高置信子集，精度优先。
  2. 几何弱标注（关键，防自训练负证据陷阱）：纯伪标签会把"没检出的真雷"
     变成负样本，反而教模型漏检。对策：field_state 的多帧投票共识雷
     （对官方基线 8/8 验证过）在模型漏检的帧上，用该帧自己的检测对应关系
     （像素中心↔格坐标）恢复仿射，把雷格中心逆投影回像素生成正框。
     ——共识说那里有雷，几何说雷在哪个像素，模型漏检的那几帧正是最该学的。
  3. train/val 按时间物理分开（后 --val-frac 的帧进 val）：相邻帧近乎重复，
     随机划分会让 val 泄漏（exp3 教训：train=val 指标虚高无意义）。

用法（机上）：
  python3 tools/export_recon_dataset.py \
      --cache /tmp/ag_recon/cam_20261018_214302-*-s5 \
      [--cache /tmp/ag_recon/另一个缓存] \
      --out /tmp/ag_ds/recon_20261023

产物：标准 YOLO 布局（images/labels × train/val + data.yaml，13 类名与
video_recon.NAMES 严格同序）。
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "ag_integrate"))
from ag_integrate.video_recon import (  # noqa: E402
    COLS, LETTERS, NAMES, ROWS, parse_cell)

# 弱标注的几何安全门
WEAK_MIN_CORR = 3        # 恢复仿射最少需要 3 个像素↔格坐标对应
WEAK_FIT_RESID = 0.30    # 对应点拟合残差上限（格单位）
WEAK_ANCHOR_GATE = 1.5   # 雷格距帧内锚点标签的最大切比雪夫距离（同投票口径）
WEAK_SIZE_FALLBACK = (0.40, 0.40)   # 无检测样本时的雷框尺寸（格单位）


def parse_cell_safe(name: str):
    try:
        return parse_cell(name)
    except (ValueError, IndexError):
        return None


def fit_affine_px2grid(pairs: list[tuple[float, float, float, float]]):
    """最小二乘仿射 px(x,y) -> grid(col,row)：[(px_x, px_y, gc, gr), ...]。

    frames_meta 不存仿射，但每帧 objs 的 (像素框中心, 连续格坐标) 就是现成
    对应关系——≥3 个且残差小即可信。返回 2x3 矩阵或 None。"""
    if len(pairs) < WEAK_MIN_CORR:
        return None
    A = np.array([[x, y, 1.0] for x, y, _, _ in pairs])
    b = np.array([[gc, gr] for _, _, gc, gr in pairs])
    try:
        M, *_ = np.linalg.lstsq(A, b, rcond=None)     # (3,2)
    except np.linalg.LinAlgError:
        return None
    resid = float(np.abs(A @ M - b).max())
    return M if resid <= WEAK_FIT_RESID else None


def _grid_per_px(M) -> float:
    """仿射 M(px齐次→grid) 的每像素格数（线性部分行向量模长均值）。"""
    return float(np.mean([np.hypot(M[0, 0], M[1, 0]), np.hypot(M[0, 1], M[1, 1])]))


def _px_from_grid(M, gc: float, gr: float) -> tuple[float, float]:
    """格坐标 → 像素（M 的逆）。U 为 px→grid 的 3x3 齐次嵌入，px = g_row @ U⁻¹。"""
    U = np.hstack([M, np.array([[0.0], [0.0], [1.0]])])
    p = np.array([gc, gr, 1.0]) @ np.linalg.inv(U)
    return float(p[0]), float(p[1])


def grid_size_of_boxes(frames: list[dict]) -> tuple[float, float]:
    """检测出的 paper 框在格单位下的中位尺寸（弱标注框尺寸基准）。"""
    ws, hs = [], []
    for m in frames:
        pairs = [((o["box"][0]+o["box"][2])/2, (o["box"][1]+o["box"][3])/2,
                  o["q"][0], o["q"][1])
                 for o in m["objs"]]
        M = fit_affine_px2grid(pairs)
        if M is None or _grid_per_px(M) <= 0:
            continue
        for o in m["objs"]:
            if o["cls"] != "paper":
                continue
            w = (o["box"][2] - o["box"][0]) * _grid_per_px(M)
            h = (o["box"][3] - o["box"][1]) * _grid_per_px(M)
            if 0.05 < w < 1.5 and 0.05 < h < 1.5:
                ws.append(w); hs.append(h)
    if not ws:
        return WEAK_SIZE_FALLBACK
    return float(np.median(ws)), float(np.median(hs))


def export_cache(cache: Path, out: Path, conf: float, val_frac: float,
                 weak: bool, stats: Counter) -> bool:
    state_path = cache / "field_state.json"
    meta_path = cache / "frames_meta.json"
    for p in (state_path, meta_path):
        if not p.is_file():
            print(f"[跳过] {cache}：缺 {p.name}")
            return False
    state = json.loads(state_path.read_text(encoding="utf-8"))
    frames = json.loads(meta_path.read_text(encoding="utf-8"))
    video = Path(state.get("video", ""))
    if not video.is_file():
        print(f"[跳过] {cache}：视频不存在 {video}")
        return False
    mines = [c for c in state.get("mines_video", []) if parse_cell_safe(c)]
    frac = state.get("frac_offset") or [0.5, 0.5]
    mine_w, mine_h = grid_size_of_boxes([m for m in frames if m.get("grid_ok")]) \
        if weak else WEAK_SIZE_FALLBACK

    ok_frames = sorted((m for m in frames if m.get("grid_ok")),
                       key=lambda m: m["frame"])
    n_val = int(len(ok_frames) * val_frac)
    split_of = {m["frame"]: ("val" if i >= len(ok_frames) - n_val else "train")
                for i, m in enumerate(ok_frames)}
    tag = cache.name
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        print(f"[跳过] {cache}：打不开视频")
        return False
    wrote = 0
    for m in ok_frames:
        cap.set(cv2.CAP_PROP_POS_FRAMES, m["frame"])
        ret, img = cap.read()
        if not ret:
            continue
        hpx, wpx = img.shape[:2]
        lines: list[str] = []
        # ① 伪标签
        for o in m["objs"]:
            if o["conf"] < conf:
                continue
            ci = NAMES.index(o["cls"]) if o["cls"] in NAMES else None
            if ci is None:
                continue
            x1, y1, x2, y2 = o["box"]
            lines.append(f"{ci} {(x1+x2)/2/wpx:.6f} {(y1+y2)/2/hpx:.6f} "
                         f"{(x2-x1)/wpx:.6f} {(y2-y1)/hpx:.6f}")
            stats[f"pseudo_{o['cls']}"] += 1
        # ② 几何弱标注：共识雷在该帧可见但模型没检出 → 逆投影补正框
        if weak and mines:
            # 已检出 paper 的格（用投票同款 snap 语义：q 减校准偏移再取整）
            covered_cells = set()
            for o in m["objs"]:
                if o["cls"] != "paper":
                    continue
                cc = int(round(o["q"][0] - frac[0]))
                rr = int(round(o["q"][1] - frac[1]))
                if 0 <= cc < COLS and 0 <= rr < ROWS:
                    covered_cells.add(f"{LETTERS[cc]}{rr+1}")
            anchors = [parse_cell_safe(n) for n in m["labels"]]
            anchors = [a for a in anchors if a is not None]
            pairs = [((o["box"][0]+o["box"][2])/2, (o["box"][1]+o["box"][3])/2,
                      o["q"][0], o["q"][1]) for o in m["objs"]]
            M = fit_affine_px2grid(pairs)          # px -> grid
            if M is not None:
                gpp = _grid_per_px(M)              # 格/像素
                for cell in mines:
                    if cell in covered_cells:
                        continue
                    col, row = parse_cell(cell)
                    if not any(max(abs(col-ac), abs(row-ar)) <= WEAK_ANCHOR_GATE
                               for ac, ar in anchors):
                        continue                    # 该帧根本没拍到这个格
                    cx, cy = _px_from_grid(M, col + frac[0], row + frac[1])
                    # 格单位尺寸 -> 像素：px = 格尺寸 / (格/像素)
                    bw, bh = mine_w / gpp, mine_h / gpp
                    x1, y1, x2, y2 = cx-bw/2, cy-bh/2, cx+bw/2, cy+bh/2
                    if x2 < 0 or y2 < 0 or x1 > wpx or y1 > hpx:
                        continue                    # 投影出画（透视边缘不可信）
                    x1, y1 = max(x1, 0), max(y1, 0)
                    x2, y2 = min(x2, wpx), min(y2, hpx)
                    lines.append(f"{NAMES.index('paper')} {cx/wpx:.6f} {cy/hpx:.6f} "
                                 f"{(x2-x1)/wpx:.6f} {(y2-y1)/hpx:.6f}")
                    stats["weak_paper"] += 1
        sp = split_of[m["frame"]]
        (out / "images" / sp).mkdir(parents=True, exist_ok=True)
        (out / "labels" / sp).mkdir(parents=True, exist_ok=True)
        stem = f"{tag}_f{m['frame']:06d}"
        cv2.imwrite(str(out / "images" / sp / f"{stem}.jpg"), img,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        (out / "labels" / sp / f"{stem}.txt").write_text(
            "\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        stats[f"img_{sp}"] += 1
        wrote += 1
    cap.release()
    print(f"[OK] {tag}: {wrote} 帧（雷共识 {len(mines)}，弱标注框尺寸 "
          f"{mine_w:.2f}x{mine_h:.2f} 格）")
    return True


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="侦察缓存 → YOLO 13 类数据集")
    ap.add_argument("--cache", action="append", required=True,
                    help="侦察缓存目录（可重复；含 field_state.json + frames_meta.json）")
    ap.add_argument("--out", required=True, help="输出数据集目录")
    ap.add_argument("--conf", type=float, default=0.60, help="伪标签置信度下限")
    ap.add_argument("--val-frac", type=float, default=0.20,
                    help="按时间切 val 的比例（物理分开，防相邻帧泄漏）")
    ap.add_argument("--no-weak", action="store_true", help="关闭几何弱标注")
    args = ap.parse_args(argv)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    stats: Counter = Counter()
    n_ok = 0
    for c in args.cache:
        if export_cache(Path(c), out, args.conf, args.val_frac,
                        weak=not args.no_weak, stats=stats):
            n_ok += 1
    if n_ok == 0:
        print("[失败] 没有任何缓存导出成功", file=sys.stderr)
        return 2

    yaml = (f"path: {out.resolve()}\ntrain: images/train\nval: images/val\n"
            f"nc: {len(NAMES)}\nnames:\n"
            + "\n".join(f"  {i}: {n}" for i, n in enumerate(NAMES)) + "\n")
    (out / "data.yaml").write_text(yaml, encoding="utf-8")
    print(f"\n[done] 数据集 {out}")
    print(f"  帧: train {stats['img_train']} / val {stats['img_val']}")
    print("  标签: " + " ".join(f"{k}={v}" for k, v in sorted(stats.items())
                                if k.startswith(("pseudo_", "weak_"))))
    print(f"  data.yaml: nc={len(NAMES)}（与 video_recon.NAMES 同序）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
