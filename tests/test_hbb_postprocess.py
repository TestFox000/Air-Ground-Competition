"""HbbDetector 后处理 Python 等价实现测试（tools/ml/hbb_postprocess.py）。

守护点：
  - 解码 + 按类别置信度过滤
  - 按类别 NMS
  - mine 边缘过滤（与 prelabel_onnx.filter_edge_boxes 同口径）
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


def _raw_one(cls_score_pairs):
    """构造 [1,11,N]：每个 anchor 给 (cx,cy,w,h) + 7 类分数。

    cls_score_pairs: list of (cx, cy, w, h, cls, score)
    """
    n = len(cls_score_pairs)
    arr = np.zeros((11, n), dtype=np.float32)
    for i, (cx, cy, w, h, cls, score) in enumerate(cls_score_pairs):
        arr[0, i] = cx
        arr[1, i] = cy
        arr[2, i] = w
        arr[3, i] = h
        arr[4 + cls, i] = score
    return arr[None]


def test_decode_and_conf_filter():
    # anchor 0: mine cls=0 score=0.9 → 保留
    # anchor 1: mine cls=0 score=0.1 → 过滤（< 0.25）
    # anchor 2: obstacle cls=6 score=0.8 → 保留
    raw = _raw_one([
        (100, 100, 20, 20, 0, 0.9),
        (200, 200, 20, 20, 0, 0.1),
        (300, 300, 30, 30, 6, 0.8),
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    classes = sorted(d.cls for d in dets)
    assert classes == [0, 6]
    assert len(dets) == 2


def test_per_class_conf_threshold():
    """mine 阈值 0.5，obstacle 阈值 0.2：mine 0.4 被滤，obstacle 0.3 保留。"""
    raw = _raw_one([
        (100, 100, 20, 20, 0, 0.4),   # mine 0.4 < 0.5 → 过滤
        (300, 300, 30, 30, 6, 0.3),   # obstacle 0.3 > 0.2 → 保留
    ])
    conf = [0.5, 0.25, 0.25, 0.25, 0.25, 0.25, 0.2]
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280, conf_per_class=conf)
    assert len(dets) == 1
    assert dets[0].cls == 6


def test_nms_same_class_suppresses():
    """同类两个重叠框，NMS 只留高分。"""
    raw = _raw_one([
        (100, 100, 40, 40, 0, 0.9),
        (102, 102, 40, 40, 0, 0.5),   # 重叠 → 被抑制
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    assert len(dets) == 1
    assert dets[0].score == pytest.approx(0.9)


def test_nms_different_class_both_kept():
    """不同类重叠框都保留（C++ 按类别分别 NMS）。"""
    raw = _raw_one([
        (100, 100, 40, 40, 0, 0.9),
        (100, 100, 40, 40, 6, 0.8),
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    classes = sorted(d.cls for d in dets)
    assert classes == [0, 6]


def test_mine_edge_filter_removes_edge():
    """mine 中心距左边缘 0.5% < 1% → 剔除。"""
    raw = _raw_one([
        (6, 640, 12, 12, 0, 0.9),    # cx=6px on 1280 wide = 0.47% → 边缘
        (640, 640, 20, 20, 0, 0.9),  # 中心 → 保留
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280, edge_mine_thr=0.01)
    assert len(dets) == 1
    assert dets[0].cx == pytest.approx(640)


def test_mine_edge_filter_disabled():
    """edge_mine_thr=0 → 不过滤。"""
    raw = _raw_one([
        (6, 640, 12, 12, 0, 0.9),
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280, edge_mine_thr=0.0)
    assert len(dets) == 1


def test_edge_filter_only_affects_mine():
    """obstacle 在边缘不受 mine 边缘过滤影响。"""
    raw = _raw_one([
        (6, 640, 30, 30, 6, 0.9),    # obstacle 边缘 → 保留
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280, edge_mine_thr=0.01)
    assert len(dets) == 1
    assert dets[0].cls == 6


def test_default_thresholds_equal_legacy():
    """默认 conf=0.25 nms=0.45 edge=0.01 与旧版单一阈值行为一致（仅多了边缘过滤）。"""
    raw = _raw_one([
        (100, 100, 20, 20, 0, 0.26),   # > 0.25 保留
        (200, 200, 20, 20, 0, 0.24),   # < 0.25 过滤
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    assert len(dets) == 1
    assert dets[0].score == pytest.approx(0.26)


def test_empty_output():
    raw = np.zeros((1, 11, 33600), dtype=np.float32)
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    assert dets == []


def test_rejects_bad_channels():
    raw = np.zeros((1, 9, 100), dtype=np.float32)
    with pytest.raises(ValueError):
        hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
