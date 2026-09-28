"""契约校验器测试（用合成的 rknn 描述块，不需要真模型/NPU）。

真实模型上的验证结果见 `docs/probe/car_models_contract.txt`：
车端 5 个 .rknn 里只有 `light_det.rknn`（= light_det3）符合 [1,9,8400] 契约。
"""

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "ml"))

from verify_contract import IMG_SIZE, NUM_PREDS, check, main, rknn_io  # noqa: E402


def _rknn_blob(tensors) -> bytes:
    """按真机文件的结构造一段可被 rknn_io 解析的内容。"""
    desc = {
        "connection": [],
        "norm_tensor": [{"size": list(s), "url": f"t{i}"} for i, s in enumerate(tensors)],
        "norm_tensor_num": len(tensors),
        "virtual_tensor": [],
    }
    return b"\x00\x01junk" + json.dumps(desc).encode() + b"\xff\xfe"


def _write(tmp_path: Path, tensors, name="m.rknn") -> Path:
    p = tmp_path / name
    p.write_bytes(_rknn_blob(tensors))
    return p


# ---------------------------------------------------------------- 解析
def test_rknn_io_reads_first_as_input_rest_as_outputs(tmp_path):
    p = _write(tmp_path, [(1, 3, IMG_SIZE, IMG_SIZE), (1, 9, NUM_PREDS)])
    ins, outs = rknn_io(p)
    assert ins == [(1, 3, IMG_SIZE, IMG_SIZE)]
    assert outs == [(1, 9, NUM_PREDS)]


def test_rknn_io_rejects_garbage(tmp_path):
    p = tmp_path / "bad.rknn"
    p.write_bytes(b"not a model")
    with pytest.raises(SystemExit):
        rknn_io(p)


# ---------------------------------------------------------------- 契约
def test_good_contract_passes(capsys):
    fails = check([(1, 3, IMG_SIZE, IMG_SIZE)], [(1, 9, NUM_PREDS)], nc=2, nkpt=1, kpt_dim=3)
    out = capsys.readouterr().out
    assert fails == 0
    assert "输出通道 9" in out


def test_multi_output_is_rejected(capsys):
    """YOLOv5 风格的三尺度输出（车端 postprocess 吃不下）。"""
    outs = [(1, 18, 80, 80), (1, 18, 40, 40), (1, 18, 20, 20)]
    fails = check([(1, 3, IMG_SIZE, IMG_SIZE)], outs, 2, 1, 3)
    assert fails == 1
    assert "单输出" in capsys.readouterr().out


def test_wrong_channel_count_is_rejected(capsys):
    fails = check([(1, 3, IMG_SIZE, IMG_SIZE)], [(1, 18, NUM_PREDS)], 2, 1, 3)
    assert fails == 1
    assert "≠" in capsys.readouterr().out


def test_wrong_input_shape_is_rejected(capsys):
    """固定成非 640 的输入是硬伤：det_node 的预处理固定 letterbox 到 640。"""
    fails = check([(1, 3, 512, 512)], [(1, 9, NUM_PREDS)], 2, 1, 3)
    assert fails == 1
    out = capsys.readouterr().out
    assert "512x512" in out and "640x640" in out


def test_dynamic_input_shape_only_warns(capsys):
    fails = check([(1, 3, "height", "width")], [(1, 9, NUM_PREDS)], 2, 1, 3)
    assert fails == 0
    assert "动态" in capsys.readouterr().out


def test_non_640_candidates_only_warn(capsys):
    fails = check([(1, 3, IMG_SIZE, IMG_SIZE)], [(1, 9, 2100)], 2, 1, 3)
    assert fails == 0
    assert "候选数" in capsys.readouterr().out


def test_channel_formula_matches_car_header():
    """4 + nc + nkpt*kpt_dim == 9 必须成立（车端 OBJ_CLASS_NUM=2 / KPT_NUM=1 / KPT_DIM=3）。"""
    assert 4 + 2 + 1 * 3 == 9


# ---------------------------------------------------------------- CLI
def test_cli_on_good_model(tmp_path, capsys):
    p = _write(tmp_path, [(1, 3, IMG_SIZE, IMG_SIZE), (1, 9, NUM_PREDS)])
    assert main(["--model", str(p)]) == 0
    assert "[PASS]" in capsys.readouterr().out


def test_cli_on_bad_model(tmp_path, capsys):
    p = _write(tmp_path, [(1, 3, IMG_SIZE, IMG_SIZE), (1, 9, 80, 80), (1, 9, 40, 40)])
    assert main(["--model", str(p)]) == 1
    assert "[FAIL]" in capsys.readouterr().out


def test_cli_rejects_pt(tmp_path):
    p = tmp_path / "best.pt"
    p.write_bytes(b"x")
    with pytest.raises(SystemExit):
        main(["--model", str(p)])
