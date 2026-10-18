"""模型契约校验纳入测试守护：models/release 下的每个 ONNX 都必须与
dzuav C++ 后处理的硬假设一致；并对校验工具本身做负向自检。

CI/无 onnx 库时 graceful skip，不阻塞基础测试。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
VERIFIER_PATH = REPO / "tools" / "ml" / "verify_contract.py"

onnx = pytest.importorskip("onnx", reason="未安装 onnx 库，跳过模型契约测试")

spec = importlib.util.spec_from_file_location("verify_contract", VERIFIER_PATH)
verifier = importlib.util.module_from_spec(spec)
# dataclass 处理期要经 sys.modules 查模块命名空间：必须先注册再 exec。
sys.modules["verify_contract"] = verifier
spec.loader.exec_module(verifier)


RELEASE_DIR = REPO / "models" / "release"


def test_all_release_onnx_pass_contract():
    targets = sorted(RELEASE_DIR.glob("*.onnx"))
    assert targets, "models/release 下没有 .onnx 文件"
    for target in targets:
        ok, message = verifier.verify(target)
        assert ok, message


def test_verifier_rejects_wrong_shape(tmp_path):
    """负向自检：输出形状故意写错时必须判定不符（校验工具不是摆设）。"""
    from onnx import TensorProto, helper

    # 假 cell_classifier：letter_logits 给成 13（契约要求 14）
    inputs = [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, 96, 96])]
    outputs = [
        helper.make_tensor_value_info("letter_logits", TensorProto.FLOAT, [1, 13]),
        helper.make_tensor_value_info("row_logits", TensorProto.FLOAT, [1, 11]),
    ]
    graph = helper.make_graph(
        [helper.make_node("Identity", ["images"], ["_img"])], "g", inputs, outputs
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    bad = tmp_path / "bad.onnx"
    onnx.save(model, str(bad))

    ok, message = verifier.verify(bad)
    assert not ok
    assert "契约不符" in message


def test_verifier_rejects_unknown_model(tmp_path):
    from onnx import TensorProto, helper

    inputs = [helper.make_tensor_value_info("x", TensorProto.FLOAT, [1, 5])]
    outputs = [helper.make_tensor_value_info("y", TensorProto.FLOAT, [1, 5])]
    graph = helper.make_graph(
        [helper.make_node("Identity", ["x"], ["y"])], "g", inputs, outputs
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    unknown = tmp_path / "unknown.onnx"
    onnx.save(model, str(unknown))

    ok, message = verifier.verify(unknown)
    assert not ok
    assert "无法识别" in message
