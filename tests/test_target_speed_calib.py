"""target_speed_calib 环形色相均值回归测试（P1-8 红色档修复）。

OpenCV 色相 0~180 环绕，红色横跨 0°（≈0-10 与 ≈170-180 两段）。旧实现直接对
h 取 sin/cos 平均，{170°,10°}（真值 0°）输出 90°——红色档必坏。修复为倍角
公式 atan2(⟨sin2h⟩,⟨cos2h⟩)/2，EMA 混合同样倍角化。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]

cv2 = pytest.importorskip("cv2")

spec = importlib.util.spec_from_file_location(
    "target_speed_calib", REPO / "tools" / "target_speed_calib.py")
tsc = importlib.util.module_from_spec(spec)
sys.modules["target_speed_calib"] = tsc
spec.loader.exec_module(tsc)


def _hue_gap(a: float, b: float) -> float:
    """环形色相距离（0~90）。"""
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def test_circular_mean_red_wraparound():
    """红色档回归：{170°,10°} 真值 0°，旧实现输出 90°。"""
    m = tsc.circular_mean(np.array([170.0, 10.0]))
    assert _hue_gap(m, 0.0) < 1.0


def test_circular_mean_red_near_zero():
    m = tsc.circular_mean(np.array([175.0, 178.0, 2.0, 5.0]))
    assert _hue_gap(m, 0.0) < 2.0


def test_circular_mean_blue_unaffected():
    """非环绕色（蓝 ~115°）倍角化后均值不变。"""
    m = tsc.circular_mean(np.array([110.0, 115.0, 120.0]))
    assert _hue_gap(m, 115.0) < 1.0


def test_learn_hsv_red_does_not_drift_to_90():
    """_learn_hsv 的 EMA 混合同样倍角化：全红内容素学习后中心不漂向 90°。"""
    tr = tsc.Tracker("red", min_area=80, s_lo=80, v_lo=110)
    # 合成 100x100 红色 ROI：左半 hue=170 右半 hue=10（环绕 0° 的红）
    hsv_roi = np.zeros((100, 100, 3), np.uint8)
    hsv_roi[:, :50, 0] = 170
    hsv_roi[:, 50:, 0] = 10
    hsv_roi[:, :, 1] = 200
    hsv_roi[:, :, 2] = 200
    contour = np.array([[0, 0], [99, 0], [99, 99], [0, 99]], np.int32)
    tr._learn_hsv(hsv_roi, contour, t=10.0)
    # 旧实现：circular_mean=90° → EMA 把中心从 0° 拉向 90°；修复后应仍 ≈0°
    assert _hue_gap(tr.hsv_rng[0], 0.0) < 5.0
