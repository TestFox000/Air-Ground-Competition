"""模型契约校验：新模型入库/上车前，核对 ONNX 的张量名与形状是否与 dzuav
C++ 后处理代码的硬假设一致。防止「换了模型但代码解不出来」的经典事故。

契约（逐字对应 onboard/dzuav/src/ 下的 C++ 代码，不可单方面改动）：

  hbb_detector（7 类，11 = 4 bbox + 7 类）
      输入  images        [N,3,1280,1280]  FLOAT   (HbbDetector 构造)
      输出  output0       [N,11,33600]     FLOAT   (33600 候选)

  hbb_detector_d13（13 类，17 = 4 bbox + 13 类；fe1d9df 起机载现役）
      输入  images        [N,3,1280,1280]  FLOAT
      输出  output0       [N,17,33600]     FLOAT   (单帧批固定 1 也合规)

  cell_classifier
      输入  images        [N,3,96,96]      FLOAT   (CellClassifier 构造)
      输出  letter_logits [N,14]           FLOAT   (列 A~N)
      输出  row_logits    [N,11]           FLOAT   (行 1~11)

  target_pose（车端靶标 pose，契约来自 docs/probe/car_models_contract.txt
              的 det_node 期望，而非 dzuav C++）
      输入  images        [N,3,640,640]    FLOAT   (yolo11n-pose 640)
      输出  output0       [N,9,8400]       FLOAT   (9 = 4 bbox + 2 类 + 1x3 关键点)

注意：hbb 与 target_pose 的输出同名 output0，靠形状分流（11/17 通道 vs 9 通道）。

用法：
    python tools/ml/verify_contract.py --model models/release/hbb_detector.onnx
    python tools/ml/verify_contract.py --all models/release
退出码：0=全部符合；1=契约不符或无法识别；2=用法/依赖问题。
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    import onnx
except ImportError:
    print("缺少 onnx 库，请先执行: python -m pip install onnx", file=sys.stderr)
    raise SystemExit(2)


@dataclass(frozen=True)
class TensorSpec:
    name: str
    dims: tuple  # int=固定维；str=符号维（动态批等）
    dtype: str = "FLOAT"  # ONNX elem_type 名；契约所有张量均为 FLOAT（见模块 docstring）


@dataclass(frozen=True)
class Contract:
    kind: str
    inputs: tuple[TensorSpec, ...]
    outputs: tuple[TensorSpec, ...]


HBB_CONTRACT = Contract(
    "hbb_detector",
    (TensorSpec("images", ("N", 3, 1280, 1280)),),
    (TensorSpec("output0", ("N", 11, 33600)),),
)

HBB_D13_CONTRACT = Contract(
    "hbb_detector_d13",
    (TensorSpec("images", ("N", 3, 1280, 1280)),),
    (TensorSpec("output0", ("N", 17, 33600)),),
)

CELL_CONTRACT = Contract(
    "cell_classifier",
    (TensorSpec("images", ("N", 3, 96, 96)),),
    (TensorSpec("letter_logits", ("N", 14)),
     TensorSpec("row_logits", ("N", 11))),
)

TARGET_POSE_CONTRACT = Contract(
    "target_pose",
    (TensorSpec("images", ("N", 3, 640, 640)),),
    (TensorSpec("output0", ("N", 9, 8400)),),
)

# 按输出张量名找候选契约（比按文件名稳，文件名可能被改过）。
# hbb / hbb_d13 / target_pose 输出同名 output0，候选会有多个，verify() 里再按形状分流。
_CONTRACTS = (HBB_CONTRACT, HBB_D13_CONTRACT, CELL_CONTRACT, TARGET_POSE_CONTRACT)


def _dims(shape) -> tuple:
    dims = []
    for dim in shape.dim:
        which = dim.WhichOneof("value")
        dims.append(dim.dim_value if which == "dim_value" else (dim.dim_param or "?"))
    return tuple(dims)


def _onnx_tensors(model) -> dict[str, tuple[tuple, str]]:
    """张量名 -> (形状, dtype 名)，输出在前、输入在后（同名时输入覆盖输出）。"""

    def _info(value) -> tuple[tuple, str]:
        tt = value.type.tensor_type
        dtype = onnx.TensorProto.DataType.Name(tt.elem_type)
        return _dims(tt.shape), dtype

    return {value.name: _info(value) for value in model.graph.output} | {
        value.name: _info(value) for value in model.graph.input
    }


def _match(spec: TensorSpec, actual: tuple) -> bool:
    """逐维匹配：契约里的 'N' 接受任意符号/值（动态批），int 维必须严格相等。"""
    if len(actual) != len(spec.dims):
        return False
    for expected, got in zip(spec.dims, actual):
        if expected == "N":
            continue
        if expected != got:
            return False
    return True


def verify(path: str | Path) -> tuple[bool, str]:
    """校验一个 ONNX 模型；返回 (是否通过, 说明)。"""
    path = Path(path)
    if not path.exists():
        return False, f"文件不存在: {path}"
    if path.suffix.lower() != ".onnx":
        return False, f"只支持 .onnx（{path.name} 不是；.rknn 需在车端另行校验）"

    model = onnx.load(str(path), load_external_data=False)
    tensors = _onnx_tensors(model)

    contract = None
    candidates = [c for c in _CONTRACTS
                  if all(spec.name in tensors for spec in c.outputs)]
    if candidates:
        # 名字命中多个契约（hbb/target_pose 共用 output0）时按形状精确分流；
        # 全都不匹配则保留第一个候选，让下方逐张量对比给出具体差异。
        contract = next(
            (c for c in candidates
             if all(_match(spec, tensors[spec.name][0])
                    for spec in (*c.inputs, *c.outputs))),
            candidates[0])
    if contract is None:
        return False, (
            f"无法识别模型类型：输出张量 {sorted(t for t in tensors)} 均不在已知契约内 "
            f"(hbb: output0 / cell: letter_logits,row_logits / target_pose: output0)"
        )

    problems = []
    for spec in (*contract.inputs, *contract.outputs):
        if spec.name not in tensors:
            problems.append(f"缺少张量 {spec.name!r}")
            continue
        dims, dtype = tensors[spec.name]
        if not _match(spec, dims):
            problems.append(
                f"张量 {spec.name!r} 形状 {dims} != 契约 {spec.dims}"
            )
        # dtype 也是契约的一部分：C++ 按 float 读张量，INT8/FP16 模型会解出乱码
        if dtype != spec.dtype:
            problems.append(
                f"张量 {spec.name!r} dtype {dtype} != 契约 {spec.dtype}"
            )

    if problems:
        return False, f"[{contract.kind}] 契约不符: " + "; ".join(problems)
    return True, f"[{contract.kind}] 契约符合: {sorted(tensors)}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--model", help="校验单个 ONNX 文件")
    group.add_argument("--all", metavar="DIR", help="校验目录下全部 .onnx")
    args = parser.parse_args()

    if args.model:
        ok, message = verify(args.model)
        print(message)
        return 0 if ok else 1

    targets = sorted(Path(args.all).glob("*.onnx"))
    if not targets:
        print(f"目录下没有 .onnx 文件: {args.all}")
        return 2
    all_ok = True
    for target in targets:
        ok, message = verify(target)
        print(message)
        all_ok = all_ok and ok
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
