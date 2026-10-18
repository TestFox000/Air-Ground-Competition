"""把「人工校正后的 capture 帧」切成 YOLO 检测数据集（train/val/test）。

航拍是一段 148s 连续录制，没有天然的拍摄段落；直接按帧随机会让相邻画面
同时进 train/val（数据泄漏，指标虚高）。做法：按时间顺序把连续帧划成
固定大小的「时间块」，整块分配到 train/val/test（复用 build_dataset 的
贪心装箱），保证每块画面只在一个子集。

输入（X-AnyLabeling 校正后的产物）：
    <capture>/images/*.jpg
    <capture>/labels/*.txt      11 类（含临时类 12 会被拒绝，必须先改完）
输出：
    <out>/images/{train,val,test}/*.jpg
    <out>/labels/{train,val,test}/*.txt
    <out>/data.yaml             Ultralytics 直接可用（11 类）

用法：
    python tools/ml/split_capture.py --capture capture/aerial_field \\
        --out datasets/aerial_field --block 8
退出码：0 成功；2 参数；3 标签未校正完。
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_dataset import split_groups  # noqa: E402


CLASS_NAMES = [
    "mine", "cell_label", "arrow", "bridge",
    "car_start", "car_end", "uav_start", "uav_end",
    "bucket_red", "bucket_green", "bucket_cyan",
]
MAX_VALID_CLS = 10


# ---------------------------------------------------------------- 纯函数（可测）

def block_ids_of(frame_count: int, block: int) -> list[int]:
    """帧按时间顺序编号，每 block 帧共用一个块 id（不足一块的尾帧独立成块）。"""
    if block < 1:
        raise ValueError("block 必须 >= 1")
    return [i // block for i in range(frame_count)]


def data_yaml_text(abs_path: Path) -> str:
    names_block = "\n".join(f"  {i}: {name}"
                           for i, name in enumerate(CLASS_NAMES))
    return (
        f"path: {abs_path.as_posix()}\n"
        "train: images/train\nval: images/val\ntest: images/test\n\n"
        f"nc: {len(CLASS_NAMES)}\nnames:\n{names_block}\n"
    )


def label_is_final(path: Path) -> bool:
    """标签内所有类号必须在 0~10；临时类 12 / 空文件异常均判未完成。"""
    if not path.exists():
        return False
    for line in path.read_text(encoding="utf-8").splitlines():
        values = line.split()
        if not values:
            continue
        try:
            cls = int(values[0])
        except ValueError:
            return False
        if cls < 0 or cls > MAX_VALID_CLS:
            return False
    return True


# ---------------------------------------------------------------- 切分

def split(capture: str, out: str, block: int,
          ratios: tuple[float, float, float], seed: int) -> dict:
    capture = Path(capture)
    images_dir = capture / "images"
    labels_dir = capture / "labels"
    image_paths = sorted(images_dir.glob("*.jpg"))
    if not image_paths:
        raise SystemExit(f"{images_dir} 无图片")

    # 全部标签必须已校正（无临时类）
    unfinished = [p.stem for p in image_paths
                  if not label_is_final(labels_dir / (p.stem + ".txt"))]
    if unfinished:
        raise SystemExit(
            f"{len(unfinished)} 帧标签未校正完（仍含临时类 12 或缺标签），"
            f"例：{unfinished[:5]}；改完再切分"
        )

    blocks = block_ids_of(len(image_paths), block)
    assignment = split_groups(blocks, ratios, seed)

    out_dir = Path(out)
    stats = {"train": 0, "val": 0, "test": 0}
    for path, bid in zip(image_paths, blocks):
        subset = assignment[bid]
        img_dst = out_dir / "images" / subset / path.name
        lbl_dst = out_dir / "labels" / subset / (path.stem + ".txt")
        img_dst.parent.mkdir(parents=True, exist_ok=True)
        lbl_dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, img_dst)
        shutil.copy2(labels_dir / (path.stem + ".txt"), lbl_dst)
        stats[subset] += 1

    (out_dir / "data.yaml").write_text(
        data_yaml_text(out_dir.resolve()), encoding="utf-8")
    return stats


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--capture", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--block", type=int, default=8,
                        help="时间块大小（连续帧数；默认 8 ≈ 4 秒）")
    parser.add_argument("--ratios", type=float, nargs=3, default=[0.8, 0.1, 0.1],
                        metavar=("TRAIN", "VAL", "TEST"))
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    ratios = tuple(args.ratios)
    if abs(sum(ratios) - 1.0) > 1e-6:
        print("ratios 之和必须为 1", file=sys.stderr)
        return 2

    stats = split(args.capture, args.out, args.block, ratios, args.seed)
    print(f"时间块切分 → train/val/test = "
          f"{stats['train']}/{stats['val']}/{stats['test']}")
    print(f"data.yaml: {(Path(args.out) / 'data.yaml').resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
