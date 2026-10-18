"""跨域留出泛化评估门禁：hbb 主检测器发布前的强制关卡（官方明牌考点）。

官方规则明示：现场可能使用训练集中没有的同类地雷图片。只看 val mAP 的
发布流程对这个考点是盲的——本门禁用"留出域"补盲：
  - 默认留出集 = 官方爆炸物图集（近景域，与航拍训练集不同域）；
  - 也可指向任何"确认不进训练集"的目录（--images）；
  - 判定 = 绝对底线（--min-recall）+ 相对现役发布模型的回归容忍
    （--max-regression-pp，基线文件 models/release/generalization_baseline.json）。

与 release_models.py 的关系：发布 hbb 检测器时自动调用（缺图片目录即拒绝），
紧急绕过用 --skip-generalization（会打印醒目警告，且登记表留标记）。

用法（训练机上，虚拟环境 C:\\Users\\a\\venvs\\air_ground）：
  python tools/ml/eval_generalization.py \
      --weights runs/detect/xxx/weights/best.pt \
      --images "E:\\...\\1-爆炸物\\B-爆炸物"
  # 人工复核后接受为新的现役基线：
  python tools/ml/eval_generalization.py --weights ... --images ... --update-baseline
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
BASELINE_PATH = REPO / "models" / "release" / "generalization_baseline.json"
README_PATH = REPO / "models" / "release" / "README.md"

MINE_CLS_ID = 0  # 与 intel_protocol.ObjectType.PAPER 一致（D13：雷=非 K 列 paper；本工具不引包，保持可独立运行）
_IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


@dataclass(frozen=True)
class HoldoutMetrics:
    """留出域评估结果。"""
    weights: str
    images_dir: str
    n_images: int
    detected_images: int     # 至少有 1 个雷框的图数
    recall: float            # detected_images / n_images
    n_boxes: int             # 雷框总数（一张图多个雷会重复计）
    conf_mean: float
    conf_min: float
    missed: tuple[str, ...]  # 漏检图文件名
    conf: float              # 推理置信度阈值（记录口径）
    imgsz: int
    date: str


def evaluate(weights: Path, images_dir: Path, conf: float = 0.25,
             imgsz: int = 1280, device: str = "cpu") -> HoldoutMetrics:
    """在留出图集上逐图推理，统计"每张图至少检出 1 个雷"的召回。

    指标口径与 10-13 巡检的域差异分析一致（explosive_* 近景域 mine 召回
    曾是 37.5% 的重灾区）；imgsz 默认 1280 与机载部署一致。
    """
    from ultralytics import YOLO  # 重依赖隔离在函数内：纯逻辑测试不需要它

    if not weights.is_file():
        raise SystemExit(f"权重文件不存在: {weights}")
    if not images_dir.is_dir():
        raise SystemExit(f"留出图集目录不存在: {images_dir}\n"
                         f"  训练机上用 --images 指向官方爆炸物图集，"
                         f"或设环境变量 AIC_MINE_HOLDOUT")
    imgs = sorted(p for p in images_dir.iterdir() if p.suffix.lower() in _IMAGE_EXTS)
    if not imgs:
        raise SystemExit(f"留出图集为空或目录不存在: {images_dir}")
    model = YOLO(str(weights))

    detected = 0
    n_boxes = 0
    confs: list[float] = []
    missed: list[str] = []
    for i, p in enumerate(imgs, 1):
        result = model.predict(source=str(p), imgsz=imgsz, conf=conf,
                               device=device, verbose=False)[0]
        boxes = [b for b in result.boxes if int(b.cls) == MINE_CLS_ID]
        if boxes:
            detected += 1
            n_boxes += len(boxes)
            confs.extend(float(b.conf) for b in boxes)
        else:
            missed.append(p.name)
        if i % 50 == 0 or i == len(imgs):
            print(f"  {i}/{len(imgs)} 检出图 {detected}", flush=True)

    return HoldoutMetrics(
        weights=str(weights), images_dir=str(images_dir),
        n_images=len(imgs), detected_images=detected,
        recall=detected / len(imgs), n_boxes=n_boxes,
        conf_mean=(sum(confs) / len(confs)) if confs else 0.0,
        conf_min=min(confs) if confs else 0.0,
        missed=tuple(missed), conf=conf, imgsz=imgsz,
        date=datetime.now().strftime("%Y-%m-%d %H:%M"),
    )


# ---------------- 判定逻辑（纯函数，pytest 直测） ----------------

def decide(metrics: HoldoutMetrics, baseline: dict | None,
           min_recall: float, max_regression_pp: float) -> tuple[bool, str]:
    """返回 (是否放行, 理由)。双闸：绝对底线 + 相对基线回归容忍。"""
    reasons: list[str] = []
    ok = True
    if metrics.recall < min_recall:
        ok = False
        reasons.append(
            f"留出召回 {metrics.recall:.1%} 低于绝对底线 {min_recall:.0%}")
    if baseline is not None:
        base_recall = float(baseline.get("recall", 0.0))
        delta_pp = (metrics.recall - base_recall) * 100
        if delta_pp < -max_regression_pp:
            ok = False
            reasons.append(
                f"留出召回 {metrics.recall:.1%} 较现役基线 "
                f"({baseline.get('weights', '?')} @ {base_recall:.1%}) "
                f"退化 {-delta_pp:.1f}pp，超过容忍 {max_regression_pp:.1f}pp")
        else:
            reasons.append(
                f"较现役基线 {base_recall:.1%} 变化 {delta_pp:+.1f}pp"
                f"（容忍 ±{max_regression_pp:.1f}pp 内）")
    else:
        reasons.append("无基线文件：本次为首次评估，仅按绝对底线判定")
    if ok:
        reasons.insert(0, "放行")
    else:
        reasons.insert(0, "拒绝")
    return ok, "；".join(reasons)


def load_baseline(path: Path = BASELINE_PATH) -> dict | None:
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def save_baseline(metrics: HoldoutMetrics, path: Path = BASELINE_PATH) -> None:
    path.write_text(json.dumps(asdict(metrics), ensure_ascii=False, indent=1),
                    encoding="utf-8")
    print(f"基线已更新: {path}（recall={metrics.recall:.1%}）")


_RECORD_HEADER = "\n## 泛化评估记录（release 门禁强制项）\n"
_RECORD_TABLE_HEAD = (
    "| 日期 | 模型 | 留出集 | 图数 | 召回 | 漏检 | 均置信度 | 判定 |\n"
    "|---|---|---|---|---|---|---|---|\n"
)


def append_record(metrics: HoldoutMetrics, verdict: str,
                  readme: Path = README_PATH) -> None:
    """把评估指标追加进 models/release/README.md 的泛化评估记录表。"""
    text = readme.read_text(encoding="utf-8")
    row = (f"| {metrics.date} | `{Path(metrics.weights).name}` "
           f"| `{Path(metrics.images_dir).name}` | {metrics.n_images} "
           f"| {metrics.recall:.1%} | {len(metrics.missed)} "
           f"| {metrics.conf_mean:.3f} | {verdict} |")
    if _RECORD_HEADER.strip() not in text:
        text = text.rstrip("\n") + "\n" + _RECORD_HEADER + _RECORD_TABLE_HEAD
    text = text.rstrip("\n") + "\n" + row + "\n"
    readme.write_text(text, encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weights", type=Path, required=True, help="待评估检测器 .pt")
    ap.add_argument("--images", type=Path, default=None,
                    help="留出图集目录（确认不进训练集；官方爆炸物图集即可；"
                         "缺省读环境变量 AIC_MINE_HOLDOUT）")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=1280, help="与机载部署一致，默认 1280")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--min-recall", type=float, default=0.50,
                    help="绝对底线召回（默认 0.50）")
    ap.add_argument("--max-regression-pp", type=float, default=2.0,
                    help="相对现役基线允许的最大退化百分点（默认 2.0）")
    ap.add_argument("--update-baseline", action="store_true",
                    help="人工复核后把本次结果写为新的现役基线")
    ap.add_argument("--record", action="store_true",
                    help="把本次评估追加进 models/release/README.md 泛化评估记录")
    args = ap.parse_args()

    import os
    images = args.images
    if images is None and os.environ.get("AIC_MINE_HOLDOUT"):
        images = Path(os.environ["AIC_MINE_HOLDOUT"])
    if images is None:
        ap.error("未给出留出图集：--images <目录> 或环境变量 AIC_MINE_HOLDOUT")

    m = evaluate(args.weights, images, args.conf, args.imgsz, args.device)

    print("\n" + "=" * 56)
    print(f"留出图集: {m.images_dir}（{m.n_images} 张）")
    print(f"检出图片: {m.detected_images}（召回 {m.recall:.1%}）")
    print(f"雷框总数: {m.n_boxes}；置信度 均 {m.conf_mean:.3f} / 最低 {m.conf_min:.3f}")
    print(f"漏检 {len(m.missed)} 张（前 20）:")
    for n in m.missed[:20]:
        print(f"  {n}")

    baseline = load_baseline()
    ok, reason = decide(m, baseline, args.min_recall, args.max_regression_pp)
    print(f"\n门禁判定: {reason}")

    if args.record:
        append_record(m, reason.split("；")[0])
        print(f"已记入 {README_PATH}")
    if args.update_baseline:
        save_baseline(m)
    if not ok:
        print("\n拒绝放行。若确认数据口径变化属预期，人工复核后用 "
              "--update-baseline 重设基线。", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
