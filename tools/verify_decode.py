#!/usr/bin/env python3
"""模型输出体检：在真机/PC 上校准 cls_activation 与 kpt_unit 两个参数。

为什么必须校准：
    Ultralytics 导出到 RKNN/ONNX 时，box 通常已解码为「输入分辨率绝对像素」、
    cls 通常已过 sigmoid；但 pose 分支的 kpt 在部分导出路径下仍是 **网格单位**。
    猜错其中一个，表现为「框很准但瞄准点乱飘」或「置信度全是 0.5 附近」。

用法：
    # 有模型有图（板端 RKNN）
    python tools/verify_decode.py --model models/best_new.rknn --image samples/frame.jpg \
        --layout anchor_free --nc 2 --nkpt 1 --img-size 640

    # 只想看输出结构（无模型，DummyBackend 全零，仅验证脚本跑通）
    python tools/verify_decode.py --layout anchor_free --nc 2 --nkpt 1

输出：各类字段的数值范围 + 推断出的建议参数，直接填进 config/params.yaml。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
# 每个包目录都要入 path（src/<pkg>/<pkg>/ 两层布局），与 tests/conftest.py 一致
for _p in [SRC] + sorted(p for p in SRC.iterdir() if p.is_dir()):
    sys.path.insert(0, str(_p))

from ag_core.backend import DummyBackend, load_backend  # noqa: E402
from ag_core.postprocess import letterbox_params  # noqa: E402


def _load_image(path: str, img_size: int) -> np.ndarray:
    import cv2

    img = cv2.imread(path)
    if img is None:
        raise SystemExit(f"无法读取图片: {path}")
    h, w = img.shape[:2]
    scale, pad_x, pad_y, nw, nh = letterbox_params(h, w, img_size)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((img_size, img_size, 3), 114, dtype=np.uint8)
    canvas[int(round(pad_y)) : int(round(pad_y)) + nh, int(round(pad_x)) : int(round(pad_x)) + nw] = resized
    return canvas


def _to_blob(canvas: np.ndarray, layout: str) -> np.ndarray:
    x = canvas.astype(np.float32) / 255.0
    if layout == "nhwc":
        return np.ascontiguousarray(x[None, ...])
    return np.ascontiguousarray(x.transpose(2, 0, 1)[None, ...])


def _fmt(arr: np.ndarray) -> str:
    a = np.asarray(arr, dtype=np.float32).reshape(-1)
    if a.size == 0:
        return "(空)"
    return f"min={a.min():.4f} max={a.max():.4f} mean={a.mean():.4f}"


def _is_all_zero(arr: np.ndarray) -> bool:
    a = np.asarray(arr, dtype=np.float32)
    return a.size > 0 and float(np.abs(a).max()) == 0.0


def inspect_anchor_free(out: np.ndarray, nc: int, nkpt: int, img_size: int) -> None:
    print(f"\n[anchor_free] 输出形状 = {out.shape}")
    if out.ndim != 3:
        print("  形状不是 (1,C,N)，请确认模型导出格式")
        return
    c, n = out.shape[1], out.shape[2]
    print(f"  通道数 C={c}，锚点数 N={n}（640 输入下 8400 为常规）")

    box = out[0, :4]
    print(f"  box[0:4]  {_fmt(box)}")
    if _is_all_zero(box):
        print("  输出全零（DummyBackend 或推理未成功），无法据此判断，请带 --model/--image 重跑")
        return

    box_max = float(np.abs(box).max())
    if box_max <= 2.0:
        print("  -> box 疑似「归一化 / 网格单位」，本项目解码器按像素处理，需先换算")
    else:
        print(f"  -> box 量级 {box_max:.1f}，符合「输入分辨率像素」假设")

    cls = out[0, 4 : 4 + nc]
    print(f"  cls[4:{4+nc}] {_fmt(cls)}")
    cmin, cmax = float(cls.min()), float(cls.max())
    if 0.0 <= cmin and cmax <= 1.0:
        print("  -> cls 落在 [0,1]，建议 cls_activation=sigmoid")
    else:
        print("  -> cls 超出 [0,1]，建议 cls_activation=sigmoid（需过激活）")

    if nkpt > 0:
        kpt = out[0, 4 + nc : 4 + nc + 3 * nkpt]
        print(f"  kpt[{4+nc}:{4+nc+3*nkpt}] {_fmt(kpt)}")
        kx = float(np.abs(out[0, 4 + nc]).max())
        if kx <= 1.5:
            print("  -> kpt 落在 0~1，是「归一化」坐标，建议 kpt_unit=norm")
        elif kx <= img_size / 8.0:
            print("  -> kpt 量级偏小，疑似「网格单位」，建议 kpt_unit=grid")
        else:
            print("  -> kpt 量级符合像素，建议 kpt_unit=pixel")


def inspect_anchored(out, nc: int, img_size: int) -> None:
    outs = out if isinstance(out, (list, tuple)) else [out]
    print(f"\n[anchored] 输出层数 = {len(outs)}")
    for i, o in enumerate(outs):
        print(f"  第 {i} 层 形状={o.shape} {_fmt(o)}")
    if all(_is_all_zero(o) for o in outs):
        print("  输出全零（DummyBackend 或推理未成功），无法据此判断，请带 --model/--image 重跑")
        return
    print("  -> YOLOv5 风格：每层 C = 3*(5+nc)，light_det 对应 nc=1")
    print(f"  -> 若实际 C 与 3*(5+{nc}) 不符，请修正 --nc")


def main() -> int:
    ap = argparse.ArgumentParser(description="模型输出体检 / 解码参数校准")
    ap.add_argument("--model", default="", help="模型路径 .rknn / .onnx；留空用 DummyBackend")
    ap.add_argument("--image", default="", help="测试图片；留空用纯色画布")
    ap.add_argument("--layout", default="anchor_free", choices=["anchor_free", "anchored"])
    ap.add_argument("--nc", type=int, default=2)
    ap.add_argument("--nkpt", type=int, default=1)
    ap.add_argument("--img-size", type=int, default=640)
    ap.add_argument("--input-layout", default="nchw", choices=["nchw", "nhwc"])
    args = ap.parse_args()

    if args.image:
        canvas = _load_image(args.image, args.img_size)
    else:
        canvas = np.full((args.img_size, args.img_size, 3), 114, dtype=np.uint8)
        print("未指定 --image，使用纯色画布（只能看输出结构，不能看真实分布）")

    blob = _to_blob(canvas, args.input_layout)
    print(f"输入 blob: shape={blob.shape} layout={args.input_layout}")

    if args.model:
        backend = load_backend(args.model)
    else:
        print("未指定 --model，使用 DummyBackend（输出全零，仅验证脚本可跑）")
        backend = DummyBackend(channels=4 + args.nc + 3 * args.nkpt)

    try:
        out = backend.infer(blob)
    finally:
        backend.close()

    if args.layout == "anchored":
        inspect_anchored(out, args.nc, args.img_size)
    else:
        inspect_anchor_free(np.asarray(out), args.nc, args.nkpt, args.img_size)

    print("\n把上面「建议」行的取值填进 src/ag_bringup/config/params.yaml 对应字段。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
