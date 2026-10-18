"""HbbDetector 后处理 Python 等价实现测试（tools/ml/hbb_postprocess.py）。

守护点（2026-10-16 起 D13 13 类统一模型，17 通道契约）：
  - 解码 + 按类别置信度过滤（13 类）
  - 按类别 NMS
  - paper 边缘过滤（cls=0，与 HbbDetector.cpp:152-155 同语义）
  - 17 通道接受 / 11 通道（旧 7cls）拒绝
  - 默认阈值 = 旧版单一阈值行为（向后兼容）
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
ML = REPO / "tools" / "ml"


def _load(name, file):
    spec = importlib.util.spec_from_file_location(name, file)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


hbbpp = _load("hbb_postprocess", ML / "hbb_postprocess.py")
cv2 = pytest.importorskip("cv2")

# D13 类别号（与 types.hpp ObjectClass / main.cpp className 一致）
PAPER, CELL_LABEL, OBSTACLE, BUCKET_GREEN = 0, 1, 5, 7


def _raw_one(cls_score_pairs):
    """构造 [1,17,N]：每个 anchor 给 (cx,cy,w,h) + 13 类分数。

    cls_score_pairs: list of (cx, cy, w, h, cls, score)
    """
    n = len(cls_score_pairs)
    arr = np.zeros((17, n), dtype=np.float32)
    for i, (cx, cy, w, h, cls, score) in enumerate(cls_score_pairs):
        arr[0, i] = cx
        arr[1, i] = cy
        arr[2, i] = w
        arr[3, i] = h
        arr[4 + cls, i] = score
    return arr[None]


def test_class_table_is_d13():
    """类别表 = D13 十三类（顺序与 C++ className 逐字一致）。"""
    assert hbbpp.NUM_CLASSES == 13
    assert hbbpp.CLASS_NAMES == [
        "paper", "cell_label", "car", "target_red", "target_blue",
        "obstacle", "bucket_red", "bucket_green", "bucket_cyan",
        "car_start", "car_end", "uav_start", "uav_end"]


def test_decode_and_conf_filter():
    # anchor 0: paper cls=0 score=0.9 → 保留
    # anchor 1: paper cls=0 score=0.1 → 过滤（< 0.25）
    # anchor 2: obstacle cls=5 score=0.8 → 保留
    raw = _raw_one([
        (100, 100, 20, 20, PAPER, 0.9),
        (200, 200, 20, 20, PAPER, 0.1),
        (300, 300, 30, 30, OBSTACLE, 0.8),
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    classes = sorted(d.cls for d in dets)
    assert classes == [PAPER, OBSTACLE]
    assert len(dets) == 2


def test_per_class_conf_threshold():
    """paper 阈值 0.5、obstacle 阈值 0.2：paper 0.4 被滤，obstacle 0.3 保留。"""
    raw = _raw_one([
        (100, 100, 20, 20, PAPER, 0.4),     # paper 0.4 < 0.5 → 过滤
        (300, 300, 30, 30, OBSTACLE, 0.3),  # obstacle 0.3 > 0.2 → 保留
    ])
    conf = [0.25] * 13
    conf[PAPER] = 0.5
    conf[OBSTACLE] = 0.2
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280, conf_per_class=conf)
    assert len(dets) == 1
    assert dets[0].cls == OBSTACLE


def test_nms_same_class_suppresses():
    """同类两个重叠框，NMS 只留高分。"""
    raw = _raw_one([
        (100, 100, 40, 40, PAPER, 0.9),
        (102, 102, 40, 40, PAPER, 0.5),   # 重叠 → 被抑制
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    assert len(dets) == 1
    assert dets[0].score == pytest.approx(0.9)


def test_nms_different_class_both_kept():
    """不同类重叠框都保留（C++ 按类别分别 NMS）。"""
    raw = _raw_one([
        (100, 100, 40, 40, PAPER, 0.9),
        (100, 100, 40, 40, OBSTACLE, 0.8),
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    classes = sorted(d.cls for d in dets)
    assert classes == [PAPER, OBSTACLE]


def test_paper_edge_filter_removes_edge():
    """paper 中心距左边缘 0.4% < 0.5% → 剔除（C++ kEdgePaperThr=0.005）。"""
    raw = _raw_one([
        (5, 640, 10, 10, PAPER, 0.9),    # cx=5px on 1280 wide ≈ 0.39% → 边缘
        (640, 640, 20, 20, PAPER, 0.9),  # 中心 → 保留
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280, edge_paper_thr=0.005)
    assert len(dets) == 1
    assert dets[0].cx == pytest.approx(640)


def test_paper_edge_filter_disabled():
    """edge_paper_thr=0 → 不过滤。"""
    raw = _raw_one([
        (5, 640, 10, 10, PAPER, 0.9),
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280, edge_paper_thr=0.0)
    assert len(dets) == 1


def test_edge_filter_only_affects_paper():
    """obstacle 在边缘不受 paper 边缘过滤影响。"""
    raw = _raw_one([
        (5, 640, 30, 30, OBSTACLE, 0.9),    # obstacle 边缘 → 保留
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280, edge_paper_thr=0.005)
    assert len(dets) == 1
    assert dets[0].cls == OBSTACLE


def test_default_thresholds_equal_legacy():
    """默认 conf=0.25 nms=0.45 edge=0.005 与 C++ 部署默认值一致。"""
    raw = _raw_one([
        (100, 100, 20, 20, PAPER, 0.26),   # > 0.25 保留
        (200, 200, 20, 20, PAPER, 0.24),   # < 0.25 过滤
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    assert len(dets) == 1
    assert dets[0].score == pytest.approx(0.26)


def test_deploy_conf_matches_main_cpp():
    """DEPLOY_CONF 与 main.cpp kDeployConf 逐项一致（13 类）。"""
    assert hbbpp.DEPLOY_CONF == (0.25, 0.50, 0.25, 0.25, 0.25, 0.40, 0.40,
                                 0.40, 0.40, 0.25, 0.25, 0.25, 0.25)


def test_empty_output():
    raw = np.zeros((1, 17, 33600), dtype=np.float32)
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    assert dets == []


def test_rejects_legacy_7cls_channels():
    """11 通道（旧 7cls 模型输出）必须拒绝——通道数不符即 ValueError。"""
    raw = np.zeros((1, 11, 33600), dtype=np.float32)
    with pytest.raises(ValueError):
        hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)


def test_rejects_bad_channels():
    raw = np.zeros((1, 9, 100), dtype=np.float32)
    with pytest.raises(ValueError):
        hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
