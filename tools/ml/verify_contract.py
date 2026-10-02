"""模型契约校验：新模型入库/上车前，核对 ONNX 的张量名与形状是否与 dzuav
C++ 后处理代码的硬假设一致。防止「换了模型但代码解不出来」的经典事故。

契约（逐字对应 onboard/dzuav/src/ 下的 C++ 代码，不可单方面改动）：

  hbb_detector
      输入  images        [N,3,1280,1280]  FLOAT   (HbbDetector 构造)
      输出  output0       [N,11,33600]     FLOAT   (11 = 4 bbox + 7 类；33600 候选)

  cell_classifier
      输入  images        [N,3,96,96]      FLOAT   (CellClassifier 构造)
      输出  letter_logits [N,14]           FLOAT   (列 A~N)
      输出  row_logits    [N,11]           FLOAT   (行 1~11)

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

CELL_CONTRACT = Contract(
    "cell_classifier",
    (TensorSpec("images", ("N", 3, 96, 96)),),
    (TensorSpec("letter_logits", ("N", 14)),
     TensorSpec("row_logits", ("N", 11))),
)

# 按输出张量名识别模型类型（比按文件名稳，文件名可能被改过）。
_CONTRACT_BY_OUTPUT = {
    spec.name: HBB_CONTRACT for spec in HBB_CONTRACT.outputs
} | {
    spec.name: CELL_CONTRACT for spec in CELL_CONTRACT.outputs
}


def _dims(shape) -> tuple:
    dims = []
    for dim in shape.dim:
        which = dim.WhichOneof("value")
        dims.append(dim.dim_value if which == "dim_value" else (dim.dim_param or "?"))
    return tuple(dims)


def _onnx_tensors(model) -> dict[str, tuple]:
    return {value.name: _dims(value.type.tensor_type.shape) for value in model.graph.output} | {
        value.name: _dims(value.type.tensor_type.shape) for value in model.graph.input
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
    for name, candidate in _CONTRACT_BY_OUTPUT.items():
        if name in tensors:
            contract = candidate
            break
    if contract is None:
        return False, (
            f"无法识别模型类型：输出张量 {sorted(t for t in tensors)} 均不在已知契约内 "
            f"(hbb: output0 / cell: letter_logits,row_logits)"
        )

    problems = []
    for spec in (*contract.inputs, *contract.outputs):
        if spec.name not in tensors:
            problems.append(f"缺少张量 {spec.name!r}")
            continue
        if not _match(spec, tensors[spec.name]):
            problems.append(
                f"张量 {spec.name!r} 形状 {tensors[spec.name]} != 契约 {spec.dims}"
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
