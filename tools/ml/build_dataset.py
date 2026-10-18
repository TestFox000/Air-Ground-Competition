"""从「分帧产物 + 标注结果」构建 YOLO-pose 数据集（train/val/test）。

关键设计：按**拍摄段落分组切分**，不按帧随机切分。
    同一段连续画面里的帧高度相似，若随机切分，同一辆车/同一批格号会同时
    出现在 train 和 val → 数据泄漏、mAP 虚高，模型看似很好、现场即穿帮。
    本工具用 manifest.json 的时间戳，把「相邻帧时间间隔 > gap」视为段落
    边界，整段整段地分配到 train/val/test。

输入目录（extract_frames 的输出 + 人工标注）：
    <frame_dir>/images/*.jpg
    <frame_dir>/labels/*.txt        YOLO-pose 标注（人工/预标注校正后）
    <frame_dir>/manifest.json
输出：
    <out>/images/{train,val,test}/*.jpg
    <out>/labels/{train,val,test}/*.txt
    <out>/data.yaml                 Ultralytics 直接可用

用法：
    python tools/ml/build_dataset.py --frames capture/field --out datasets/field
    python tools/ml/build_dataset.py --frames capture/field --out datasets/field \
        --nkpt 1 --classes blue_target red_target --ratios 0.8 0.1 0.1 --gap 3.0
"""

from __future__ import annotations

import argparse
import json
import random
import shutil
import sys
from pathlib import Path


# ---------------------------------------------------------------- 纯函数（可测）

def assign_groups(frame_entries: list[dict], gap_seconds: float) -> list[int]:
    """按 src_time_s 的间隔给每帧打「段落 id」。帧需按时间顺序排列。"""
    groups = []
    current = -1
    previous_time = None
    for entry in frame_entries:
        t = entry["src_time_s"]
        if previous_time is None or t - previous_time > gap_seconds:
            current += 1
        groups.append(current)
        previous_time = t
    return groups


def split_groups(group_ids: list[int], ratios: tuple[float, float, float],
                 seed: int = 42) -> dict[int, str]:
    """把段落整段分配到 train/val/test；返回 group_id -> 子集名。

    大段落优先装箱，尽量让各子集的帧数接近目标比例（贪心装箱，简单可复现）。
    """
    total = len(group_ids)
    unique = sorted(set(group_ids))
    counts = {g: group_ids.count(g) for g in unique}
    targets = {"train": ratios[0] * total, "val": ratios[1] * total,
               "test": ratios[2] * total}
    buckets: dict[str, list[int]] = {"train": [], "val": [], "test": []}
    loads = {"train": 0, "val": 0, "test": 0}

    rng = random.Random(seed)
    ordered = sorted(unique, key=lambda g: -counts[g])
    for g in ordered:
        # 放进「缺口比例」最大的桶；同分时按固定顺序 + 随机抖动保证可复现
        def deficit(name: str) -> float:
            return (targets[name] - loads[name]) / max(1.0, targets[name])
        best = max(("train", "val", "test"), key=deficit)
        tie = [name for name in ("train", "val", "test")
               if abs(deficit(name) - deficit(best)) < 1e-9]
        if len(tie) > 1:
            best = rng.choice(tie)
        buckets[best].append(g)
        loads[best] += counts[g]

    assignment: dict[int, str] = {}
    for name, members in buckets.items():
        for g in members:
            assignment[g] = name
    return assignment


def data_yaml_text(abs_path: Path, class_names: list[str], nkpt: int,
                   dim: int = 2) -> str:
    names_block = "\n".join(f"  {i}: {name}" for i, name in enumerate(class_names))
    # flip_idx：水平翻转时关键点对调索引。单点（靶心）无需对调 → [0]；
    # 若标了首尾对称的 2 点设 [1, 0]；不确定时保持顺序，避免错误增强。
    if nkpt == 2:
        flip = "[1, 0]"
    else:
        flip = str(list(range(nkpt)))
    return (
        f"path: {abs_path.as_posix()}\n"
        "train: images/train\nval: images/val\ntest: images/test\n\n"
        f"kpt_shape: [{nkpt}, {dim}]\n"
        f"flip_idx: {flip}\n\n"
        f"names:\n{names_block}\n"
    )


# ---------------------------------------------------------------- 构建

def build(frames_dir: str, out_dir: str, class_names: list[str], nkpt: int,
          ratios: tuple[float, float, float], gap: float, seed: int) -> dict:
    frames_dir = Path(frames_dir)
    manifest = json.loads((frames_dir / "manifest.json").read_text(encoding="utf-8"))
    entries = manifest["frames"]

    labels_dir = frames_dir / "labels"
    if not labels_dir.exists():
        raise SystemExit(
            f"缺少标注目录 {labels_dir}：先做预标注+人工校正（生成 YOLO-pose txt）"
        )

    group_ids = assign_groups(entries, gap)
    assignment = split_groups(group_ids, ratios, seed)

    out = Path(out_dir)
    stats = {"train": 0, "val": 0, "test": 0}
    skipped_missing_label = 0
    for entry, gid in zip(entries, group_ids):
        subset = assignment[gid]
        image_src = frames_dir / "images" / entry["file"]
        label_src = labels_dir / (Path(entry["file"]).stem + ".txt")
        if not image_src.exists() or not label_src.exists():
            skipped_missing_label += 1
            continue
        img_dst = out / "images" / subset / entry["file"]
        lbl_dst = out / "labels" / subset / (Path(entry["file"]).stem + ".txt")
        img_dst.parent.mkdir(parents=True, exist_ok=True)
        lbl_dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(image_src, img_dst)
        shutil.copy2(label_src, lbl_dst)
        stats[subset] += 1

    (out / "data.yaml").write_text(
        data_yaml_text(out.resolve(), class_names, nkpt), encoding="utf-8"
    )
    result = {"stats": stats, "groups": len(set(group_ids)),
              "skipped": skipped_missing_label}
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--frames", required=True, help="extract_frames 的输出目录")
    parser.add_argument("--out", required=True, help="数据集输出目录")
    parser.add_argument("--classes", nargs="+", default=["blue_target", "red_target"],
                        help="类别名顺序（默认蓝面敌方 blue_target、红面友方 red_target，"
                             "必须与标注/射击决策一致）")
    parser.add_argument("--nkpt", type=int, default=1,
                        help="每目标关键点数（默认 1：靶心）")
    parser.add_argument("--ratios", type=float, nargs=3, default=[0.8, 0.1, 0.1],
                        metavar=("TRAIN", "VAL", "TEST"))
    parser.add_argument("--gap", type=float, default=3.0,
                        help="相邻帧间隔超过此秒数视为新段落（默认 3）")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    ratios = tuple(args.ratios)
    if abs(sum(ratios) - 1.0) > 1e-6:
        print("ratios 之和必须为 1", file=sys.stderr)
        return 2

    result = build(args.frames, args.out, args.classes, args.nkpt,
                   ratios, args.gap, args.seed)
    print(f"段落数 {result['groups']} → train/val/test = "
          f"{result['stats']['train']}/{result['stats']['val']}/{result['stats']['test']}")
    if result["skipped"]:
        print(f"跳过缺标注/图片的帧: {result['skipped']}")
    print(f"data.yaml: {(Path(args.out) / 'data.yaml').resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
