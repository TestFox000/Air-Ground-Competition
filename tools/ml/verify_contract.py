#!/usr/bin/env python3
"""模型契约校验：模型的输出形状能不能被车端 `det_node` 的 postprocess 吃下。

车端契约（`rknn_pt/include/det/postprocess.h` + `det_node.cc`）：
    输入    [1, 3, 640, 640]                     （letterbox 到 640）
    输出    [1, 4 + nc + KPT_NUM*KPT_DIM, 8400]  （YOLOv8-pose 单输出、DFL 已在模型内解码）
             nc = OBJ_CLASS_NUM = 2，KPT_NUM = 1，KPT_DIM = 3  →  通道数必须是 9
             channel 0..3 = cx, cy, w, h（单位：模型输入尺度像素）
             channel 4..5 = 蓝/红两类分数（已激活）
             channel 6..8 = kx, ky, kconf

任何一条不符，上车就是"检测框乱飞/一个目标都出不来"。
本脚本在**导出后、上车前**把这件事查出来（纯形状检查，不需要 NPU）。

    python tools/ml/verify_contract.py --model best.onnx
    python tools/ml/verify_contract.py --model light_det.rknn
    python tools/ml/verify_contract.py --model best.onnx --nc 2 --nkpt 1 --kpt-dim 3
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import List, Optional, Tuple

IMG_SIZE = 640
NUM_PREDS = 8400          # 80*80 + 40*40 + 20*20


def onnx_io(path: Path) -> Tuple[List[tuple], List[tuple]]:
    try:
        import onnx
    except ImportError:
        raise SystemExit("检查 .onnx 需要 onnx：pip install onnx（或改用 rknn 文件）")
    m = onnx.load(str(path))
    ins = [tuple(d.dim_value or d.dim_param for d in i.type.tensor_type.shape.dim)
           for i in m.graph.input]
    outs = [tuple(d.dim_value or d.dim_param for d in o.type.tensor_type.shape.dim)
            for o in m.graph.output]
    return ins, outs


def rknn_io(path: Path) -> Tuple[List[tuple], List[tuple]]:
    """从 .rknn 里读出内嵌的模型描述（rknn-toolkit2 会把 norm_tensor 写进文件）。"""
    blob = path.read_bytes().decode("latin-1")
    best = None
    # 真机文件里是紧凑 JSON（无空格），但对空格宽容一点更稳
    for m in re.finditer(r'\{"connection".*?"virtual_tensor"\s*:\s*\[\s*\]\s*\}', blob, re.S):
        try:
            d = json.loads(m.group(0))
        except json.JSONDecodeError:
            continue
        if "norm_tensor" in d:
            best = d
    if best is None:
        raise SystemExit(f"没能从 {path} 里解析出模型描述（文件损坏或格式变了？）")
    tensors = best["norm_tensor"]
    ins = [tuple(t["size"]) for t in tensors[:1]]
    outs = [tuple(t["size"]) for t in tensors[1:]]
    return ins, outs


def check(ins, outs, nc: int, nkpt: int, kpt_dim: int) -> int:
    want_ch = 4 + nc + nkpt * kpt_dim
    fails = 0

    print(f"输入: {ins}")
    print(f"输出: {outs}")
    print(f"期望输出通道数 = 4 + {nc} 类 + {nkpt}x{kpt_dim} 关键点 = {want_ch}")

    if not ins:
        print("  [FAIL] 没有输入信息")
        return 1
    shape_in = ins[0]
    if len(shape_in) != 4 or shape_in[1] != 3:
        print(f"  [FAIL] 输入应为 NCHW 的 [1,3,{IMG_SIZE},{IMG_SIZE}]，实为 {shape_in}")
        fails += 1
    else:
        h, w = shape_in[2], shape_in[3]
        if isinstance(h, str) or isinstance(w, str):
            print(f"  [WARN] 输入尺寸是动态的（{h}x{w}）；车端按固定 {IMG_SIZE} 喂，"
                  "建议导出时固定尺寸")
        elif (int(h), int(w)) != (IMG_SIZE, IMG_SIZE):
            # 固定成别的尺寸就是硬伤：det_node 的预处理固定 letterbox 到 640，
            # 尺寸不符会在推理那一刻才报错
            print(f"  [FAIL] 输入固定为 {h}x{w}，车端预处理是 {IMG_SIZE}x{IMG_SIZE}，必崩")
            fails += 1
        else:
            print(f"  [OK]   输入 1x3x{IMG_SIZE}x{IMG_SIZE}")

    if len(outs) != 1:
        print(f"  [FAIL] 应是单输出（YOLOv8-pose 导出后 DFL 已解码），实为 {len(outs)} 个输出。"
              "     多输出通常是 YOLOv5 风格，车端 postprocess 不支持")
        fails += 1
        return fails

    shape_out = tuple(int(v) if isinstance(v, int) else v for v in outs[0])
    if len(shape_out) != 3:
        print(f"  [FAIL] 输出应为 [1, C, N]，实为 {shape_out}")
        return fails + 1
    if shape_out[1] != want_ch:
        print(f"  [FAIL] 输出通道 {shape_out[1]} ≠ {want_ch}；"
              "     说明类别数或关键点数与车端不一致（改了 data.yaml 就必须同步改车端宏）")
        fails += 1
    else:
        print(f"  [OK]   输出通道 {want_ch}（4 框 + {nc} 类 + {nkpt}x{kpt_dim} 关键点）")
    if shape_out[2] != NUM_PREDS:
        print(f"  [WARN] 候选数 {shape_out[2]} ≠ {NUM_PREDS}（非 640 输入时会变，确认一下）")
    else:
        print(f"  [OK]   候选数 {NUM_PREDS}")
    return fails


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="校验模型是否符合车端 det_node 的输出契约")
    ap.add_argument("--model", required=True, help=".onnx 或 .rknn")
    ap.add_argument("--nc", type=int, default=2, help="类别数（车端 OBJ_CLASS_NUM）")
    ap.add_argument("--nkpt", type=int, default=1, help="关键点数（车端 KPT_NUM）")
    ap.add_argument("--kpt-dim", type=int, default=3, help="每个关键点的值个数（车端 KPT_DIM）")
    args = ap.parse_args(argv)

    p = Path(args.model).expanduser()
    if not p.is_file():
        raise SystemExit(f"找不到 {p}")
    print(f"=== 契约校验 {p.name} ===")

    if p.suffix == ".onnx":
        ins, outs = onnx_io(p)
    elif p.suffix == ".rknn":
        ins, outs = rknn_io(p)
    elif p.suffix == ".pt":
        raise SystemExit(".pt 不是部署格式，先 export 成 onnx/rknn")
    else:
        raise SystemExit(f"不支持的格式 {p.suffix}")

    fails = check(ins, outs, args.nc, args.nkpt, args.kpt_dim)
    print()
    if fails:
        print(f"[FAIL] {fails} 项不符合车端契约，别上车")
    else:
        print("[PASS] 形状契约符合车端；接下来在**板端**验证数值语义：")
        print("       python tools/verify_decode.py --model <模型> --image <真图>")
        print("       （cls_activation / kpt_unit 必须实测，不能照搬假设）")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
