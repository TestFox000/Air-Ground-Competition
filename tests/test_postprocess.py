import numpy as np
import pytest

from ag_core.postprocess import (
    Detection,
    decode_anchor_free,
    decode_anchored,
    letterbox_params,
    nms,
    scale_coords,
)


def _pose_output(cls_scores=(0.9, 0.1), box=(320.0, 320.0, 40.0, 40.0), kpt=(320.0, 320.0, 0.85), idx=3240):
    """构造一个 [1,9,8400] 的 pose 输出，在指定 anchor 上放一个目标。"""
    out = np.zeros((1, 9, 8400), dtype=np.float32)
    out[0, 0:4, idx] = box
    out[0, 4, idx] = cls_scores[0]
    out[0, 5, idx] = cls_scores[1]
    out[0, 6:9, idx] = kpt
    return out


def test_pose_decode_basic():
    dets = decode_anchor_free(_pose_output(), nc=2, nkpt=1, conf_thres=0.25)
    assert len(dets) == 1
    d = dets[0]
    assert d.cls == 0
    assert pytest.approx(d.score, abs=1e-5) == 0.9
    cx, cy = d.center()
    assert pytest.approx(cx, abs=1e-4) == 320.0
    assert pytest.approx(cy, abs=1e-4) == 320.0
    kx, ky, kc = d.kpt(0)
    assert (kx, ky) == (320.0, 320.0)
    assert pytest.approx(kc, abs=1e-5) == 0.85


def test_pose_decode_red_class():
    dets = decode_anchor_free(_pose_output(cls_scores=(0.1, 0.95)), nc=2, nkpt=1)
    assert dets[0].cls == 1
    assert pytest.approx(dets[0].score, abs=1e-5) == 0.95


def test_empty_when_below_threshold():
    dets = decode_anchor_free(_pose_output(cls_scores=(0.1, 0.05)), nc=2, nkpt=1, conf_thres=0.5)
    assert dets == []


def test_nms_suppresses_duplicate():
    out = _pose_output()
    out[0, :, 3241] = out[0, :, 3240]          # 紧邻 anchor 上的重复框
    out[0, 0, 3241] = 322.0
    dets = decode_anchor_free(out, nc=2, nkpt=1, iou_thres=0.5)
    assert len(dets) == 1


def test_channel_mismatch_raises():
    with pytest.raises(ValueError):
        decode_anchor_free(np.zeros((1, 5, 8400), np.float32), nc=2, nkpt=1)


def test_grid_unit_keypoint_scaling():
    """kpt_unit='grid' 时关键点应乘以 stride 还原为像素。"""
    out = _pose_output(kpt=(40.0, 40.0, 0.8), idx=3240)  # 80x80 段 stride=8
    dets = decode_anchor_free(out, nc=2, nkpt=1, kpt_unit="grid")
    kx, ky, _ = dets[0].kpt(0)
    assert pytest.approx(kx, abs=1e-3) == 320.0
    assert pytest.approx(ky, abs=1e-3) == 320.0


def test_raw_activation_applies_sigmoid():
    out = _pose_output()
    out[0, 4, 3240] = 2.0   # logit， sigmoid(2)=0.8808
    out[0, 5, 3240] = 0.0
    dets = decode_anchor_free(out, nc=2, nkpt=1, cls_activation="raw", conf_thres=0.5)
    assert pytest.approx(dets[0].score, abs=1e-3) == 0.8808


def test_letterbox_roundtrip():
    h, w = 480, 640
    scale, pad_x, pad_y, nw, nh = letterbox_params(h, w, 640)
    # 原图中心点 -> 输入图 -> 反算回原图
    x_in, y_in = 160 * scale + pad_x, 120 * scale + pad_y
    back = scale_coords(np.array([[x_in, y_in]]), scale, pad_x, pad_y)
    assert pytest.approx(back[0][0], abs=1e-6) == 160.0
    assert pytest.approx(back[0][1], abs=1e-6) == 120.0


def test_detect_only_model():
    """rest_clean.rknn 风格：[1,5,8400]，nc=1，无关键点。"""
    out = np.zeros((1, 5, 8400), dtype=np.float32)
    out[0, 0:4, 100] = (200.0, 150.0, 30.0, 30.0)
    out[0, 4, 100] = 0.8
    dets = decode_anchor_free(out, nc=1, nkpt=0, conf_thres=0.25)
    assert len(dets) == 1
    assert dets[0].kpts is None
    assert dets[0].cls == 0


def test_anchored_decode_official_model():
    """官方 light_det.rknn 风格：1x18x{80,40,20}^2，YOLOv5 anchor-based。"""
    outs = []
    for s in (80, 40, 20):
        outs.append(np.zeros((1, 18, s, s), dtype=np.float32))
    # 在 stride=8 层、anchor 0、grid (gy=10, gx=10) 处放一个目标
    o = outs[0].reshape(1, 3, 6, 80, 80)
    o[0, 0, 0, 10, 10] = 0.0     # tx -> sigmoid=0.5
    o[0, 0, 1, 10, 10] = 0.0     # ty
    o[0, 0, 2, 10, 10] = 0.0     # tw -> w = anchor_w = 10
    o[0, 0, 3, 10, 10] = 0.0     # th -> h = anchor_h = 13
    o[0, 0, 4, 10, 10] = 2.0     # obj
    o[0, 0, 5, 10, 10] = 2.0     # cls
    outs[0] = o.reshape(1, 18, 80, 80)

    dets = decode_anchored(outs, conf_thres=0.5)
    assert len(dets) == 1
    cx = (dets[0].x1 + dets[0].x2) / 2.0
    cy = (dets[0].y1 + dets[0].y2) / 2.0
    assert pytest.approx(cx, abs=0.6) == 84.0   # (0.5*2-0.5+10)*8
    assert pytest.approx(cy, abs=0.6) == 84.0


def test_nms_ordering():
    boxes = np.array([[0, 0, 10, 10], [0.5, 0.5, 10.5, 10.5], [100, 100, 110, 110]], dtype=np.float32)
    scores = np.array([0.9, 0.8, 0.7], dtype=np.float32)
    keep = nms(boxes, scores, 0.5)
    assert keep == [0, 2]
