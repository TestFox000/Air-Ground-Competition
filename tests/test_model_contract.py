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

# 2026-10-16 切 13 类（D 方案）之前的历史留档。仓库策略是模型只放
# models/release/ 一处（见 .gitignore），评估失败品/旧契约品也照规矩留档，
# 所以本目录**允许**存在不符合当前契约的文件——但必须在这里显式登记。
# 下面 test_archived_models_still_fail_contract 会反向断言它们确实不符，
# 一旦有人把它们"修好"或改名，登记就会失效并报错，防止清单变成遮羞布。
ARCHIVED_PRE_13CLASS = {
    "hbb_detector.onnx",                        # 出厂 7 类（11 通道）
    "hbb_detector_7cls_distill_20261011.onnx",  # 7 类 val-B 90.30%
    "hbb_detector_7cls_local_20261013.onnx",    # 7 类，README 标注"评估失败品，勿部署"
}


def test_all_release_onnx_pass_contract():
    targets = sorted(RELEASE_DIR.glob("*.onnx"))
    assert targets, "models/release 下没有 .onnx 文件"
    failures = [f"  {t.name}: {m}"
                for t in targets if t.name not in ARCHIVED_PRE_13CLASS
                for ok, m in [verifier.verify(t)] if not ok]
    assert not failures, (
        "models/release 下有**非留档**模型不符合当前契约：\n" + "\n".join(failures) +
        "\n\n若原因是 output0 通道数 11 != 17：入库的是 2026-10-16 前的 7 类模型，"
        "而机上 HbbDetector.cpp 硬校验 [1,17,33600]，7 类引擎一上机就在构造期 throw。"
        "修法：用 tools/release_models.py --hbb 发布 13 类 hbb_d13"
        "（imgsz=1280，输出 [N,17,33600]），并更新 current_hbb_detector 指针；"
        "确属历史留档则加进本文件的 ARCHIVED_PRE_13CLASS。"
    )


def test_archived_models_still_fail_contract():
    """留档清单不许说谎：登记为"旧契约"的文件必须真的不符合当前契约。"""
    for name in sorted(ARCHIVED_PRE_13CLASS):
        path = RELEASE_DIR / name
        assert path.is_file(), f"ARCHIVED_PRE_13CLASS 里的 {name} 已不在 models/release/，请同步清单"
        ok, _ = verifier.verify(path)
        assert not ok, f"{name} 现在符合契约了——它不该再被当作旧契约留档，请移出清单"


def test_current_pointers_are_deployable():
    """current_* 指针指向的模型必须真能上车（在位 + 过契约 + 非留档）。

    orin_convert_engines.sh 只认这两个指针；指针指着 7 类模型时，换机/重刷
    JetPack 后按仓库流程重转引擎必得 11 通道产物，上机即炸——而本地自检全绿。
    """
    for pointer in ("current_hbb_detector", "current_cell_classifier"):
        p = RELEASE_DIR / pointer
        assert p.is_file(), f"缺少指针文件 {pointer}"
        name = p.read_text(encoding="utf-8").strip()
        assert name not in ARCHIVED_PRE_13CLASS, (
            f"{pointer} 指着历史留档 {name}（7 类旧契约），不可部署"
        )
        target = RELEASE_DIR / name
        assert target.is_file(), f"{pointer} -> {name} 不存在"
        ok, message = verifier.verify(target)
        assert ok, f"{pointer} -> {name} 不可部署：{message}"


def test_channels_derived_from_types_hpp():
    """通道数必须现场从 types.hpp 派生，不得在本仓任何位置写死。

    2026-10-20 事故复盘：verify_contract 硬编码 11（= 4 bbox + 7 类），
    HbbDetector.cpp 硬校验 [1,17,33600]。两处各自"正确"，于是发布门禁放行的
    7 类 ONNX 一到机上就 throw，反过来真正的 13 类模型被门禁判为契约不符。
    """
    n = verifier.object_class_count()
    assert n == 13, f"types.hpp kObjectClassCount={n}，与 13 类协议不符"
    spec = verifier.hbb_contract(n).outputs[0]
    assert spec.dims == ("N", 4 + n, 33600)
    source = VERIFIER_PATH.read_text(encoding="utf-8")
    assert '("N", 11, 33600)' not in source, "verify_contract.py 里又出现了写死的 11 通道"


def test_verifier_fails_closed_when_types_hpp_unusable(tmp_path, monkeypatch):
    """负向自检：真源不可用/自相矛盾时必须报错并判不符，不静默回退默认值。"""
    import re

    src = verifier.TYPES_HPP.read_text(encoding="utf-8")

    # 改了 enum 却忘了改常量（或反之）——正是本模块要守住的那类分叉
    bad = re.sub(r"(constexpr\s+int\s+kObjectClassCount\s*=\s*)\d+", r"\g<1>12", src)
    assert bad != src, "types.hpp 里的 kObjectClassCount 声明形式变了，本测试需同步"
    broken = tmp_path / "types.hpp"
    broken.write_text(bad, encoding="utf-8")
    monkeypatch.setattr(verifier, "TYPES_HPP", broken)
    with pytest.raises(ValueError):
        verifier.object_class_count()

    monkeypatch.setattr(verifier, "TYPES_HPP", tmp_path / "missing.hpp")
    with pytest.raises(ValueError):
        verifier.object_class_count()
    ok, message = verifier.verify(RELEASE_DIR / "cell_classifier.onnx")
    assert not ok and "契约真源不可用" in message


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
