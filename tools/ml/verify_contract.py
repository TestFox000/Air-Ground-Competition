"""模型契约校验：新模型入库/上车前，核对 ONNX 的张量名与形状是否与 dzuav
C++ 后处理代码的硬假设一致。防止「换了模型但代码解不出来」的经典事故。

契约（逐字对应 onboard/dzuav/src/ 下的 C++ 代码，不可单方面改动）：

  hbb_detector
      输入  images        [N,3,1280,1280]  FLOAT   (HbbDetector::allocate)
      输出  output0       [N,4+K,33600]    FLOAT   (K = types.hpp kObjectClassCount)

  cell_classifier
      输入  images        [N,3,96,96]      FLOAT   (CellClassifier 构造)
      输出  letter_logits [N,14]           FLOAT   (列 A~N)
      输出  row_logits    [N,11]           FLOAT   (行 1~11)

⚠️ 检测通道数**不在本文件写死**，而是从
`onboard/dzuav/include/dzuav/types.hpp` 的 `kObjectClassCount` 现场解析。
2026-10-20 事故复盘：本文件曾硬编码 11（= 4 bbox + 7 类），而机上
`HbbDetector.cpp` 硬校验 `[1,17,33600]`。两处各自"正确"，于是发布门禁
放行的 7 类 ONNX 一到机上就在构造期抛错；反过来真正的 13 类模型会被
门禁判为"契约不符"而拒绝入库。单点派生后，改类目只需改 types.hpp。

派生失败（头文件缺失/解析不到）时一律判 FAILED，不静默回退到任何默认值。

用法：
    python tools/ml/verify_contract.py --model models/release/hbb_detector.onnx
    python tools/ml/verify_contract.py --all models/release
退出码：0=全部符合；1=契约不符或无法识别；2=用法/依赖问题。
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    import onnx
except ImportError:
    print("缺少 onnx 库，请先执行: python -m pip install onnx", file=sys.stderr)
    raise SystemExit(2)

REPO = Path(__file__).resolve().parents[2]
TYPES_HPP = REPO / "onboard" / "dzuav" / "include" / "dzuav" / "types.hpp"

_BBOX_CHANNELS = 4          # HbbDetector.cpp: cx, cy, w, h
_ANCHORS = 33600            # HbbDetector.cpp kAnchors: 160²+80²+40² @ imgsz=1280
_LETTERS = 14               # CellClassifier: 列 A~N
_ROWS = 11                  # CellClassifier: 行 1~11


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


def object_class_count() -> int:
    """从 types.hpp 解析检测类目数，并与 enum ObjectClass 的成员数交叉校验。

    两者不一致即抛 ValueError：说明有人改了枚举却忘了改常量（或反之），
    这正是本模块要守住的那类分叉。
    """
    if not TYPES_HPP.is_file():
        raise ValueError(f"找不到 {TYPES_HPP}（无法派生检测通道数）")
    text = TYPES_HPP.read_text(encoding="utf-8", errors="replace")

    m = re.search(r"constexpr\s+int\s+kObjectClassCount\s*=\s*(\d+)\s*;", text)
    if not m:
        raise ValueError(f"{TYPES_HPP.name} 里解析不到 kObjectClassCount")
    declared = int(m.group(1))

    e = re.search(r"enum\s+class\s+ObjectClass\b[^{]*\{(.*?)\}", text, re.S)
    if not e:
        raise ValueError(f"{TYPES_HPP.name} 里解析不到 enum class ObjectClass")
    members = re.findall(r"^\s*([A-Za-z_]\w*)\s*=", e.group(1), re.M)
    if len(members) != declared:
        raise ValueError(
            f"{TYPES_HPP.name} 自相矛盾：kObjectClassCount={declared}，"
            f"但 enum ObjectClass 有 {len(members)} 个成员 {members}"
        )
    return declared


def hbb_contract(class_count: int) -> Contract:
    return Contract(
        "hbb_detector",
        (TensorSpec("images", ("N", 3, 1280, 1280)),),
        (TensorSpec("output0", ("N", _BBOX_CHANNELS + class_count, _ANCHORS)),),
    )


CELL_CONTRACT = Contract(
    "cell_classifier",
    (TensorSpec("images", ("N", 3, 96, 96)),),
    (TensorSpec("letter_logits", ("N", _LETTERS)),
     TensorSpec("row_logits", ("N", _ROWS))),
)

# 按输出张量名识别模型类型（比按文件名稳，文件名可能被改过）。
# cell 侧是静态的；hbb 侧通道数依赖 types.hpp，识别时只看张量名，故用固定键。
_CONTRACT_KIND_BY_OUTPUT = {
    "output0": "hbb_detector",
    "letter_logits": "cell_classifier",
    "row_logits": "cell_classifier",
}


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

    try:
        class_count = object_class_count()
    except ValueError as exc:
        # fail-closed：拿不到真源就不给绿灯，而不是退回一个可能过时的数字。
        return False, f"契约真源不可用: {exc}"

    model = onnx.load(str(path), load_external_data=False)
    tensors = _onnx_tensors(model)

    kind = next((v for k, v in _CONTRACT_KIND_BY_OUTPUT.items() if k in tensors), None)
    if kind is None:
        return False, (
            f"无法识别模型类型：输出张量 {sorted(t for t in tensors)} 均不在已知契约内 "
            f"(hbb: output0 / cell: letter_logits,row_logits)"
        )
    contract = hbb_contract(class_count) if kind == "hbb_detector" else CELL_CONTRACT

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

    try:
        print(f"检测类目数（types.hpp kObjectClassCount）= {object_class_count()}"
              f" -> output0 通道数 = {_BBOX_CHANNELS + object_class_count()}")
    except ValueError as exc:
        print(f"[失败] {exc}", file=sys.stderr)
        return 1

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
