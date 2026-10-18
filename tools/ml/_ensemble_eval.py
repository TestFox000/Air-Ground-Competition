# -*- coding: utf-8 -*-
"""集成检测评估：combined(obstacle+arrow) + final_v2(mine)。

策略：
- mine(cls=0) 取自 final_v2 预测（含 1% 边缘过滤）
- obstacle(cls=6) + arrow(cls=2) 取自 combined 预测
- 其他类别(cell_label/target_red/target_blue) 取自 combined
- car 排除

用法:
    python tools/ml/_ensemble_eval.py
"""
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PYTHON = r"C:\Users\a\venvs\air_ground\Scripts\python.exe"

COMBINED_LABELS = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls_combined" / "pred_val" / "labels"
FINALV2_LABELS = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls_combined" / "pred_val_final_v2" / "labels"
MERGED_DIR = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls_combined" / "pred_val_ensemble" / "labels"

VAL_IMG = Path(r"E:/ml_datasets/aerial_field_7cls/images/val")
VAL_GT = Path(r"E:/ml_datasets/aerial_field_7cls/labels/val")
NAMES = "mine,car,arrow,target_red,cell_label,target_blue,obstacle"
EXCLUDE = "car"

# mine 边缘过滤：中心点距边缘 < 1% 则移除
EDGE_THR = 0.01
MINE_CLS = 0


def apply_mine_edge_filter(lines, edge_thr=0.01, mine_cls=0):
    """过滤中心点距图像边缘 < edge_thr 的 mine 检测框。"""
    kept = []
    removed = 0
    for line in lines:
        parts = line.strip().split()
        if not parts:
            continue
        cls = int(parts[0])
        cx, cy = float(parts[1]), float(parts[2])
        if cls == mine_cls and (cx < edge_thr or cx > 1 - edge_thr or cy < edge_thr or cy > 1 - edge_thr):
            removed += 1
            continue
        kept.append(line.strip())
    return kept, removed


def main():
    if not COMBINED_LABELS.exists():
        print(f"combined 预测不存在: {COMBINED_LABELS}", file=sys.stderr)
        return 1
    if not FINALV2_LABELS.exists():
        print(f"final_v2 预测不存在: {FINALV2_LABELS}", file=sys.stderr)
        return 1

    MERGED_DIR.mkdir(parents=True, exist_ok=True)

    total_removed = 0
    frames = sorted(COMBINED_LABELS.glob("*.txt"))
    for f in frames:
        # combined 预测：取 obstacle(6) + arrow(2) + cell_label(4) + target_red(3) + target_blue(5)
        combined_lines = f.read_text().strip().split("\n") if f.exists() else []
        merged = []
        for line in combined_lines:
            parts = line.strip().split()
            if not parts:
                continue
            cls = int(parts[0])
            if cls in (2, 3, 4, 5, 6):  # arrow, target_red, cell_label, target_blue, obstacle
                merged.append(line.strip())

        # final_v2 预测：取 mine(0)，应用边缘过滤
        fv2_path = FINALV2_LABELS / f.name
        if fv2_path.exists():
            fv2_lines = fv2_path.read_text().strip().split("\n")
            mine_lines = [l for l in fv2_lines if l.strip() and int(l.strip().split()[0]) == MINE_CLS]
            mine_kept, removed = apply_mine_edge_filter(mine_lines, EDGE_THR, MINE_CLS)
            merged.extend(mine_kept)
            total_removed += removed

        out = MERGED_DIR / f.name
        out.write_text("\n".join(merged) + ("\n" if merged else ""), encoding="utf-8")

    print(f"集成完成: {len(frames)} 帧，mine 边缘过滤移除 {total_removed} 个框")
    print(f"合并标签 → {MERGED_DIR}")

    # 运行 eval_comprehensive.py
    eval_script = REPO / "tools" / "ml" / "eval_comprehensive.py"
    cmd = [
        PYTHON, str(eval_script),
        "--pred-dir", str(MERGED_DIR),
        "--gt-dir", str(VAL_GT),
        "--img-dir", str(VAL_IMG),
        "--names", NAMES,
        "--exclude", EXCLUDE,
    ]
    print("\n运行 eval_comprehensive.py ...")
    r = subprocess.run(cmd, capture_output=True, text=True)
    print(r.stdout)
    if r.stderr:
        print(r.stderr, file=sys.stderr)
    return r.returncode


if __name__ == "__main__":
    sys.exit(main())
