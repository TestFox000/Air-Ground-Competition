#!/usr/bin/env python3
"""YOLO-pose 数据集体检（纯 Python，不依赖 ultralytics / torch）。

为什么必须有这一步：车端 `postprocess.h` 写死了
`OBJ_CLASS_NUM=2`（**0=blue_target 1=red_target**）、`KPT_NUM=1`、`kpt_shape [1,3]`。
数据集一旦标错（少一个关键点、类别顺序反了、坐标没归一化），
训练照样能跑完、导出照样能成功，**上车才会发现打错人/打不中**。
所以在训练之前先把数据集查一遍。

标签格式（YOLO-pose，每行一个目标）：
    <cls> <cx> <cy> <w> <h> <kx> <ky> <kv>
      cls  0=blue_target(敌方) 1=red_target(友方)
      cx cy w h  归一化到 [0,1]
      kx ky      关键点（靶心/发光中心）归一化到 [0,1]
      kv         关键点可见性 0=不可见 1=遮挡 2=可见

用法：
    python tools/ml/dataset_check.py --root /path/to/dataset
    python tools/ml/dataset_check.py --root ... --json report.json
    python tools/ml/dataset_check.py --root ... --names blue_target,red_target
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

IMAGE_EXT = (".jpg", ".jpeg", ".png", ".bmp", ".webp")

#: 与车端 postprocess.h 一致的契约
DEFAULT_NAMES = ("blue_target", "red_target")
DEFAULT_KPT_SHAPE = (1, 3)


@dataclass
class Report:
    root: str = ""
    images: int = 0
    labels: int = 0
    boxes: int = 0
    per_class: Dict[int, int] = field(default_factory=dict)
    per_split: Dict[str, int] = field(default_factory=dict)
    kpt_vis: Dict[int, int] = field(default_factory=dict)
    missing_labels: List[str] = field(default_factory=list)
    orphan_labels: List[str] = field(default_factory=list)
    empty_labels: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors

    def summary(self) -> str:
        lines = [
            f"数据集: {self.root}",
            f"  图像 {self.images} 张 / 标签 {self.labels} 个 / 目标框 {self.boxes} 个",
        ]
        if self.per_split:
            lines.append("  各 split: " + ", ".join(
                f"{k}={v}" for k, v in sorted(self.per_split.items())))
        if self.per_class:
            lines.append("  各类别: " + ", ".join(
                f"{k}={v}" for k, v in sorted(self.per_class.items())))
        if self.kpt_vis:
            lines.append("  关键点可见性: " + ", ".join(
                f"{k}={v}" for k, v in sorted(self.kpt_vis.items())))
        for label, items in (("错误", self.errors), ("告警", self.warnings)):
            if items:
                lines.append(f"  {label} {len(items)} 条：")
                lines += [f"    - {i}" for i in items[:20]]
                if len(items) > 20:
                    lines.append(f"    … 另有 {len(items) - 20} 条")
        return "\n".join(lines)


def discover(root: Path, splits: Sequence[str]) -> Dict[str, List[Path]]:
    """找各 split 的图像；兼容两种目录布局：

        root/images/train/*.jpg + root/labels/train/*.txt   （ultralytics 标准）
        root/train/images/*.jpg + root/train/labels/*.txt   （自建常见）
    """
    out: Dict[str, List[Path]] = {}
    for split in splits:
        cands = [root / "images" / split, root / split / "images", root / split]
        found: List[Path] = []
        for d in cands:
            if d.is_dir():
                found = sorted(p for p in d.iterdir()
                               if p.suffix.lower() in IMAGE_EXT)
                if found:
                    break
        out[split] = found
    return out


def label_path_for(img: Path, root: Path) -> Path:
    """由图像路径推出标签路径（同上两种布局），后缀一定换成 .txt。"""
    parts = list(img.parts)
    if "images" in parts:
        i = len(parts) - 1 - parts[::-1].index("images")
        parts[i] = "labels"
        return Path(*parts).with_suffix(".txt")   # 注意：要把 .jpg 换成 .txt
    return img.with_suffix(".txt")                # 图像与标签同目录


def parse_label(text: str, n_fields: int, n_classes: int) -> Tuple[List[Tuple], List[str]]:
    """解析一个标签文件，返回 (框列表, 错误列表)。"""
    boxes, errs = [], []
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != n_fields:
            errs.append(f"第 {lineno} 行字段数 {len(parts)}≠{n_fields}（1 框 + "
                        f"{n_fields - 5} 关键点值）")
            continue
        try:
            vals = [float(v) for v in parts]
        except ValueError:
            errs.append(f"第 {lineno} 行有非数字字段")
            continue
        cls = int(vals[0])
        if cls < 0 or cls >= n_classes:
            errs.append(f"第 {lineno} 行类别 {cls} 超出 0~{n_classes - 1}")
            continue
        nums = vals[1:]
        bad = [f"{v:.3f}" for v in nums[:4] if not (0.0 <= v <= 1.0)]
        if bad:
            errs.append(f"第 {lineno} 行框坐标未归一化到 [0,1]: {bad}")
            continue
        if not (nums[2] > 0 and nums[3] > 0):
            errs.append(f"第 {lineno} 行宽高必须 > 0（w={nums[2]}, h={nums[3]}）")
            continue
        kx, ky, kv = nums[4], nums[5], int(round(nums[6]))
        if not (0.0 <= kx <= 1.0 and 0.0 <= ky <= 1.0):
            errs.append(f"第 {lineno} 行关键点未归一化: ({kx:.3f}, {ky:.3f})")
            continue
        if kv not in (0, 1, 2):
            errs.append(f"第 {lineno} 行关键点可见性 {kv} 不在 0/1/2")
            continue
        boxes.append((cls, nums[0], nums[1], nums[2], nums[3], kx, ky, kv))
    return boxes, errs


def check_dataset(
    root: Path,
    names: Sequence[str] = DEFAULT_NAMES,
    kpt_shape: Tuple[int, int] = DEFAULT_KPT_SHAPE,
    splits: Sequence[str] = ("train", "val"),
) -> Report:
    r = Report(root=str(root))
    n_fields = 4 + 1 + kpt_shape[0] * kpt_shape[1]
    n_classes = len(names)

    if not root.is_dir():
        r.errors.append(f"数据集根目录不存在: {root}")
        return r

    split_images = discover(root, splits)
    total = sum(len(v) for v in split_images.values())
    if total == 0:
        r.errors.append(
            f"没找到任何图像；找过这些位置: "
            + ", ".join(str(root / "images" / s) for s in splits))
        return r

    for split, imgs in split_images.items():
        r.per_split[split] = len(imgs)
        for img in imgs:
            r.images += 1
            lp = label_path_for(img, root)
            rel = lp.relative_to(root).as_posix()   # 统一成 / 分隔，报告跨平台可比
            if not lp.is_file():
                r.missing_labels.append(rel)
                continue
            r.labels += 1
            boxes, errs = parse_label(lp.read_text(encoding="utf-8", errors="replace"),
                                      n_fields, n_classes)
            for e in errs:
                r.errors.append(f"{rel}: {e}")
            if not boxes and not errs:
                r.empty_labels.append(rel)
            for b in boxes:
                r.boxes += 1
                r.per_class[b[0]] = r.per_class.get(b[0], 0) + 1
                r.kpt_vis[b[7]] = r.kpt_vis.get(b[7], 0) + 1

    # 孤立标签（有标签没图像）
    for lp in root.rglob("*.txt"):
        if "labels" not in lp.parts and lp.parent == root:
            continue
        if "labels" in lp.parts:
            stem = lp.stem
            has = any(p.stem == stem for p in root.rglob(f"{stem}.*"))
            if not has:
                r.orphan_labels.append(lp.relative_to(root).as_posix())

    # 数据集层面的告警
    if len(split_images) > 1 and any(len(v) == 0 for v in split_images.values()):
        r.warnings.append("有 split 是空的：" + ", ".join(
            s for s, v in split_images.items() if not v))
    if r.per_class and set(r.per_class) != set(range(n_classes)):
        missing = sorted(set(range(n_classes)) - set(r.per_class))
        r.warnings.append(
            f"类别 {missing} 一个样本都没有（{names}）；"
            "缺红的会把友方误当敌方，缺蓝的会打不到分")
    if r.missing_labels:
        r.errors.append(f"{len(r.missing_labels)} 张图没有对应标签（见列表）")
    if r.orphan_labels:
        r.warnings.append(f"{len(r.orphan_labels)} 个标签没有对应图像")
    if r.kpt_vis and r.kpt_vis.get(0, 0) > 0.5 * max(1, r.boxes):
        r.warnings.append("超过一半的关键点标记为不可见（kv=0），靶心学不出来")
    if r.boxes and r.per_class and min(r.per_class.values()) < 0.2 * max(r.per_class.values()):
        r.warnings.append("类别数量严重不均衡，训练容易偏向多数类")
    return r


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="YOLO-pose 数据集体检（对照车端契约）")
    ap.add_argument("--root", required=True, help="数据集根目录")
    ap.add_argument("--names", default=",".join(DEFAULT_NAMES),
                    help="类别名，逗号分隔，顺序必须与训练一致（默认 blue_target,red_target）")
    ap.add_argument("--kpt-shape", default="1,3", help="关键点形状，默认 1,3")
    ap.add_argument("--splits", default="train,val", help="要检查的 split")
    ap.add_argument("--json", default=None, help="把报告写成 JSON")
    args = ap.parse_args(argv)

    names = tuple(n.strip() for n in args.names.split(",") if n.strip())
    kx, ky = (int(v) for v in args.kpt_shape.split(","))
    splits = tuple(s.strip() for s in args.splits.split(",") if s.strip())

    r = check_dataset(Path(args.root).expanduser(), names, (kx, ky), splits)
    print(r.summary())
    if args.json:
        Path(args.json).write_text(json.dumps({
            "ok": r.ok, "images": r.images, "labels": r.labels, "boxes": r.boxes,
            "per_class": r.per_class, "per_split": r.per_split, "kpt_vis": r.kpt_vis,
            "errors": r.errors, "warnings": r.warnings,
            "missing_labels": r.missing_labels[:50],
        }, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"\n报告已写入 {args.json}")
    print("\n" + ("[PASS] 数据集可用，可以开始训练" if r.ok
                  else f"[FAIL] 有 {len(r.errors)} 条错误，先修再训练"))
    return 0 if r.ok else 1


if __name__ == "__main__":
    sys.exit(main())
