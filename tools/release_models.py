"""一键模型发布：训练产物 .pt -> ONNX 契约校验 -> models/release 入库登记。

发布链最后一环。训练权重不直接上车/上机，先过三关：
  1. 导出 ONNX（张量名/形状逐字符合 dzuav C++ 后处理契约）
  2. verify_contract.py 契约校验（防"换了模型但代码解不出来"）
  3. 带版本文件名入库 models/release/ + 登记表 + current 指针

版本命名遵守 models/release/README.md 规则：版本变了换名字，不同名覆盖。
orin_convert_engines.sh 读 current 指针取最新版，机上 git pull 后重转引擎即可。

用法（虚拟环境 C:\\Users\\a\\venvs\\air_ground）:
  python tools/release_models.py --cell runs/cell_classifier_v4c/best.pt --tag v4c
  python tools/release_models.py --hbb "runs/detect/runs/detect/aerial_field_finetuned_v2/weights/best.pt" --tag v2
  python tools/release_models.py --cell ... --hbb ... --tag v4c   # 两个都发
"""
from __future__ import annotations

import argparse
import hashlib
import re
import shutil
import sys
import tempfile
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "tools" / "ml"))
from verify_contract import verify  # noqa: E402

RELEASE = REPO / "models" / "release"


def md5_of(path: Path) -> str:
    h = hashlib.md5()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest().upper()


def export_cell(pt: Path, out_dir: Path) -> Path:
    """格号分类器 .pt -> ONNX（契约: images[N,3,96,96] -> letter/row_logits）。"""
    sys.path.insert(0, str(REPO / "reference" / "onboard" / "dzuav" / "src"))
    import torch
    from cell_classifier.model import load_checkpoint_model

    model, ckpt = load_checkpoint_model(pt, "cpu")
    size = int(ckpt.get("config", {}).get("image_size", 96))
    out = out_dir / "cell.onnx"
    torch.onnx.export(
        model.eval(), torch.randn(1, 3, size, size), out,
        input_names=["images"],
        output_names=["letter_logits", "row_logits"],
        dynamic_axes={"images": {0: "N"}, "letter_logits": {0: "N"},
                      "row_logits": {0: "N"}},
        opset_version=17, do_constant_folding=True,
    )
    return out


def export_hbb(pt: Path, out_dir: Path) -> Path:
    """YOLO 检测器 .pt -> ONNX（契约: images[N,3,1280,1280] -> output0[N,11,33600]）。"""
    from ultralytics import YOLO

    YOLO(str(pt)).export(format="onnx", imgsz=1280, dynamic=True, opset=17)
    onnx = pt.with_suffix(".onnx")
    if not onnx.is_file():  # ultralytics 有时存到别的名字
        cand = sorted(pt.parent.glob("*.onnx"), key=lambda p: p.stat().st_mtime)
        if not cand:
            raise SystemExit(f"导出后找不到 ONNX（期望 {onnx}）")
        onnx = cand[-1]
    dst = out_dir / "hbb.onnx"
    shutil.move(str(onnx), dst)
    return dst


def register(kind: str, src: Path, tag: str) -> Path:
    """契约校验 -> 带版本名入库 -> 登记表追加 -> current 指针。返回入库路径。"""
    ok, msg = verify(src)
    print(msg)
    if not ok:
        raise SystemExit(f"[{kind}] 契约校验失败，拒绝入库: {src}")

    base = "cell_classifier" if kind == "cell" else "hbb_detector"
    date = datetime.now().strftime("%Y%m%d")
    dst = RELEASE / f"{base}_{tag}_{date}.onnx"
    i = 1
    while dst.exists():  # 同日多次发布加序号
        dst = RELEASE / f"{base}_{tag}_{date}_{i}.onnx"
        i += 1
    shutil.copy2(src, dst)

    # current 指针（orin_convert_engines.sh 读它取最新版）
    (RELEASE / f"current_{base}").write_text(dst.name, encoding="utf-8")

    # 登记表追加一行（models/release/README.md）
    readme = RELEASE / "README.md"
    text = readme.read_text(encoding="utf-8")
    row = (f"| `{dst.name}` | `{md5_of(dst)}` | {dst.stat().st_size / 1e6:.2f} MB "
           f"| 本次训练产物（tag={tag}） | {('格号分类器' if kind == 'cell' else '无人机主检测器')}"
           f"，见 current_{base} 指针 | {datetime.now().strftime('%Y-%m-%d')} |")
    lines = text.splitlines()
    last_bar = max(i for i, ln in enumerate(lines) if ln.startswith("| `"))
    lines.insert(last_bar + 1, row)
    readme.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"已入库: {dst} ({dst.stat().st_size / 1e6:.2f} MB)")
    print(f"已更新指针: current_{base} -> {dst.name}")
    return dst


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cell", type=Path, help="格号分类器 .pt（cell_classifier 训练产物）")
    ap.add_argument("--hbb", type=Path, help="YOLO 检测器 .pt（aerial_field 训练产物）")
    ap.add_argument("--tag", required=True, help="版本标签（如 v4c / v2）")
    args = ap.parse_args()
    if not args.cell and not args.hbb:
        ap.error("至少指定 --cell 或 --hbb 之一")

    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        if args.cell:
            print(f"==> 导出格号分类器: {args.cell}")
            register("cell", export_cell(args.cell, tmp), args.tag)
        if args.hbb:
            print(f"==> 导出主检测器: {args.hbb}")
            register("hbb", export_hbb(args.hbb, tmp), args.tag)

    print("\n机上换模型三步:")
    print("  git pull && tools/orin_convert_engines.sh   # 重转 FP16 引擎")
    print("  bash tools/orin_build_verify.sh --run ~/dzuav/engines <测试图>")
    print("  bash tools/orin_run_dzuav.sh camera          # 上场")
    return 0


if __name__ == "__main__":
    sys.exit(main())
