"""ML 工具纯函数测试：分组切分（防泄漏）、IoU、AP。"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest
from collections import Counter

REPO = Path(__file__).resolve().parents[1]
ML = REPO / "tools" / "ml"


def load_module(name: str, file: Path):
    spec = importlib.util.spec_from_file_location(name, file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


build_dataset = load_module("build_dataset", ML / "build_dataset.py")


# ---------------------------------------------------------------- 分组

def test_assign_groups_at_gap_boundary():
    frames = [{"src_time_s": t} for t in (0.0, 0.5, 1.0, 5.0, 5.5, 10.0)]
    groups = build_dataset.assign_groups(frames, gap_seconds=3.0)
    assert groups == [0, 0, 0, 1, 1, 2]


def test_split_groups_keeps_group_intact_and_covers_all():
    """同一段落所有帧必须落在同一子集（防泄漏的核心），且全部有归属。"""
    groups = [0, 0, 0, 1, 1, 2, 3]
    assignment = build_dataset.split_groups(groups, (0.8, 0.1, 0.1), seed=1)
    values = [assignment[g] for g in groups]
    assert values[:3] == [values[0]] * 3
    assert values[3:5] == [values[3]] * 2
    assert set(values) <= {"train", "val", "test"}
    assert set(assignment.values())  # 非空


def test_data_yaml_has_pose_contract(tmp_path):
    text = build_dataset.data_yaml_text(
        tmp_path, ["blue_target", "red_target"], nkpt=1
    )
    assert "kpt_shape: [1, 2]" in text
    assert "flip_idx: [0]" in text
    assert "train: images/train" in text
    assert "0: blue_target" in text
    assert "1: red_target" in text


# ---------------------------------------------------------------- IoU / AP

cv2 = pytest.importorskip("cv2")
compare = load_module("compare_models", ML / "compare_models.py")
prelabel = load_module("prelabel_rknn", ML / "prelabel_rknn.py")

import numpy as np


def test_iou_known_values():
    # 完全重合 → 1；不相交 → 0；半重合矩形 → 1/3
    assert compare.iou((0, 0, 2, 2), (0, 0, 2, 2)) == pytest.approx(1.0)
    assert compare.iou((0, 0, 1, 1), (5, 5, 6, 6)) == pytest.approx(0.0)
    assert compare.iou((0, 0, 2, 2), (1, 0, 3, 2)) == pytest.approx(1 / 3, abs=1e-6)


def test_ap_perfect_and_missed():
    gts = {"a": [(0, 0, 1, 1)], "b": [(0, 0, 1, 1)]}
    perfect = [("a", 0.9, (0, 0, 1, 1)), ("b", 0.8, (0, 0, 1, 1))]
    assert compare.average_precision(perfect, gts, 0.5) == pytest.approx(1.0)

    # 全部预测错位 → AP 0
    wrong = [("a", 0.9, (5, 5, 6, 6)), ("b", 0.8, (5, 5, 6, 6))]
    assert compare.average_precision(wrong, gts, 0.5) == pytest.approx(0.0)


def test_ap_false_positive_lowers_precision():
    gts = {"a": [(0, 0, 1, 1)]}
    # 高分框错位（FP 在前）、低分框才正确 → PR 曲线被压低，0 < AP < 1
    preds = [("a", 0.9, (5, 5, 6, 6)), ("a", 0.8, (0, 0, 1, 1))]
    ap = compare.average_precision(preds, gts, 0.5)
    assert ap == pytest.approx(0.5)


# ---------------------------------------------------------------- 解码契约

def _synthetic_output():
    """构造官方契约 [1,9,8400]：4框 + 蓝/红2类 + 1靶心点。"""
    arr = np.zeros((9, 8400), dtype=np.float32)
    # anchor 0：蓝类，框心 100,100，靶心 101,102
    arr[:4, 0] = [100, 100, 20, 20]
    arr[4, 0] = 0.9
    arr[5, 0] = 0.1
    arr[6:9, 0] = [101, 102, 0.95]
    # anchor 1：红类，框心 300,300，靶心 301,302
    arr[:4, 1] = [300, 300, 30, 40]
    arr[4, 1] = 0.05
    arr[5, 1] = 0.8
    arr[6:9, 1] = [301, 302, 0.90]
    return arr[None]


def test_decode_official_two_class_one_kpt():
    boxes, scores, kpts, class_ids = prelabel.decode_output(
        _synthetic_output(), 0.25, 0.45, 2, 1
    )
    assert boxes.shape == (2, 4)
    assert list(class_ids) == [0, 1]
    assert kpts.shape == (2, 1, 3)
    # 靶心坐标必须从通道 6:8 取，不能误当成第二关键点
    assert kpts[0, 0, 0] == pytest.approx(101)
    assert kpts[0, 0, 1] == pytest.approx(102)
    assert kpts[1, 0, 0] == pytest.approx(301)


def _synthetic_output_no_visibility():
    """构造 8 通道 [1,8,8400]：4框 + 蓝/红2类 + 1靶心点(无visibility)。"""
    arr = np.zeros((8, 8400), dtype=np.float32)
    arr[:4, 0] = [100, 100, 20, 20]
    arr[4, 0] = 0.9
    arr[5, 0] = 0.1
    arr[6:8, 0] = [101, 102]
    arr[:4, 1] = [300, 300, 30, 40]
    arr[4, 1] = 0.05
    arr[5, 1] = 0.8
    arr[6:8, 1] = [301, 302]
    return arr[None]


def test_decode_eight_channel_no_visibility():
    """8 通道模型无 visibility 分，解码后 v 应自动补 1.0。"""
    boxes, scores, kpts, class_ids = prelabel.decode_output(
        _synthetic_output_no_visibility(), 0.25, 0.45, 2, 1
    )
    assert boxes.shape == (2, 4)
    assert list(class_ids) == [0, 1]
    assert kpts.shape == (2, 1, 3)
    assert kpts[0, 0, 0] == pytest.approx(101)
    assert kpts[0, 0, 1] == pytest.approx(102)
    assert kpts[0, 0, 2] == pytest.approx(1.0)
    assert kpts[1, 0, 2] == pytest.approx(1.0)


def test_decode_rejects_channel_mismatch():
    # nc=1,nkpt=1 → 含visibility=8 / 无visibility=7，9 通道两种布局都不匹配
    with pytest.raises(ValueError):
        prelabel.decode_output(_synthetic_output(), 0.25, 0.45, 1, 1)


def test_decode_empty_when_all_below_conf():
    boxes, scores, kpts, class_ids = prelabel.decode_output(
        _synthetic_output(), 0.95, 0.45, 2, 1
    )
    assert len(boxes) == 0
    assert kpts.shape[1:] == (1, 3)


def test_yolo_lines_carry_class_id():
    boxes = np.array([[100, 100, 20, 20]], dtype=np.float32)
    scores = np.array([0.9], dtype=np.float32)
    kpts = np.array([[[100, 100, 0.95]]], dtype=np.float32)
    class_ids = np.array([1], dtype=np.int64)
    lines = prelabel.to_yolo_lines(
        boxes, scores, kpts, class_ids, 1.0, (0, 0), 640, 640
    )
    assert lines[0].startswith("1 ")


def test_map_matches_only_same_class():
    gts = {"a": [(0, (0, 0, 1, 1))]}
    # 真实蓝类、预测成红类 → 蓝类漏检，红类无 GT 不计入 → mAP 0
    wrong = [("a", 0.9, 1, (0, 0, 1, 1))]
    assert compare.mean_average_precision(wrong, gts, 0.5, 2) == 0.0
    # 类别正确 → mAP 1
    right = [("a", 0.9, 0, (0,0,1,1))]
    assert compare.mean_average_precision(right, gts, 0.5, 2) == 1.0


# ---------------------------------------------------------------- 航拍流水线

prelabel_o = load_module("prelabel_onnx", ML / "prelabel_onnx.py")
convert_l = load_module("convert_draft_labels", ML / "convert_draft_labels.py")
supplement_m = load_module("auto_supplement", ML / "auto_supplement.py")
gallery_m = load_module("card_gallery", ML / "card_gallery.py")


def test_decode_detect_11_channels():
    # 构造 [1,11,N]：4框+7类；anchor 0 蓝(类0)、anchor 1 红(类6)
    arr = np.zeros((11, 5), dtype=np.float32)
    arr[:4, 0] = [100, 100, 20, 20]
    arr[4, 0] = 0.9
    arr[:4, 1] = [300, 300, 30, 30]
    arr[10, 1] = 0.8
    boxes, scores, class_ids = prelabel_o.decode_detect(
        arr[None], 0.25, 0.45, 7
    )
    assert boxes.shape == (2, 4)
    assert list(class_ids) == [0, 6]
    assert scores == pytest.approx([0.9, 0.8])


def test_decode_detect_rejects_bad_channels():
    with pytest.raises(ValueError):
        prelabel_o.decode_detect(np.zeros((1, 9, 100), np.float32), .25, .45, 7)


def test_classify_bucket_colors_and_reflection():
    assert convert_l.classify_bucket(15, 140, 90) == "green"
    assert convert_l.classify_bucket(30, 50, 120) == "red"
    assert convert_l.classify_bucket(95, 105, 80) == "cyan"
    # 高亮低色差 → 灯光反光，非桶
    assert convert_l.classify_bucket(150, 155, 150) is None


def test_convert_line_mapping():
    # mine 官方0 → 新0
    assert convert_l.convert_line([0, .5, .5, .2, .2], None).startswith("0 ")
    # cell_label 官方4 → 新1
    assert convert_l.convert_line([4, .5, .5, .1, .05], None).startswith("1 ")
    # car 官方1 航拍无 → 丢弃
    assert convert_l.convert_line([1, .5, .5, .2, .2], None) is None
    # obstacle 反光帧 → 丢弃
    assert convert_l.convert_line([6, .5, .5, .2, .2], None) is None
    # obstacle 红桶 → 新8
    assert convert_l.convert_line([6, .5, .5, .2, .2], "red").startswith("8 ")


def test_assign_cluster():
    # 空 reps → 新建
    assert gallery_m.assign_cluster(0b1010, [], 20) == -1
    # 距离内 → 该簇
    assert gallery_m.assign_cluster(0b1000, [0b0000], 20) == 0
    # 距离外 → 新建
    assert gallery_m.assign_cluster(0b1111, [0b0000], 1) == -1


def test_hull_mask_and_overlap():
    h, w = 100, 100
    mask = supplement_m.hull_mask([(10, 10), (90, 10), (90, 90), (10, 90)], h, w)
    assert mask is not None and mask.sum() > 0
    assert supplement_m.hull_mask([(1, 2)], h, w) is None
    contour = np.array([[[20, 20]], [[80, 20]], [[80, 80]], [[20, 80]]])
    overlap = supplement_m.contour_overlap(contour, mask, h, w)
    assert overlap == pytest.approx(1.0)


def test_normalized_line():
    line = supplement_m._normalized(50, 50, 20, 40, 100, 100)
    assert line == "0.500000 0.500000 0.200000 0.400000"


def test_labeling_yaml_has_temp_classes():
    text = convert_l.labeling_yaml_text(Path("capture/aerial_field/images"))
    assert "10: bucket_cyan" in text
    assert "12: orange_raw" in text


# ---------------------------------------------------------------- 检测训练切分

split_cap = load_module("split_capture", ML / "split_capture.py")


def test_block_ids_of():
    assert split_cap.block_ids_of(20, 8) == [0, 0, 0, 0, 0, 0, 0, 0,
                                              1, 1, 1, 1, 1, 1, 1, 1,
                                              2, 2, 2, 2]
    with pytest.raises(ValueError):
        split_cap.block_ids_of(5, 0)


def test_split_capture_yaml():
    text = split_cap.data_yaml_text(Path("datasets/aerial_field"))
    assert "nc: 11" in text
    assert "10: bucket_cyan" in text
    assert "train: images/train" in text


def test_label_is_final(tmp_path):
    good = tmp_path / "good.txt"
    good.write_text("0 0.5 0.5 0.2 0.2\n10 0.1 0.1 0.1 0.1\n", encoding="utf-8")
    assert split_cap.label_is_final(good)
    bad = tmp_path / "bad.txt"
    bad.write_text("12 0.5 0.5 0.2 0.2\n", encoding="utf-8")
    assert not split_cap.label_is_final(bad)
    assert not split_cap.label_is_final(tmp_path / "missing.txt")


# ---------------------------------------------------------------- 橙格细分

co = load_module("classify_orange", ML / "classify_orange.py")
co_ocr = load_module("classify_orange_ocr", ML / "classify_orange_ocr.py")


def test_orange_vote_mapping():
    spacing = 100
    points = np.array([[0, 0], [100, 0], [50, 80]], dtype=float)
    reads = [(13, 7), (13, 7), (12, 6)]
    votes = co.weighted_vote(np.array([50, 40]), points, reads, spacing * 1.6)
    cls_id, desc, weight = co.map_orange(votes)
    assert cls_id == 6 and desc == "N8" and weight > 0


def test_orange_map_rejects_unknown_cell():
    votes = Counter({(0, 0): 3.0})
    cls_id, desc, weight = co.map_orange(votes)
    assert cls_id is None and desc == "A1"


def test_median_pairwise_spacing():
    points = np.array([[0, 0], [100, 0], [0, 100], [100, 100]], dtype=float)
    assert co.median_pairwise_spacing(points) == pytest.approx(100.0)


def test_ocr_classify_text():
    assert co_ocr.classify_text("智能车10起点") == 4
    assert co_ocr.classify_text("无人机终") == 7
    assert co_ocr.classify_text("智能车终点") == 5
    assert co_ocr.classify_text("乱七八糟") is None


def test_upscale_if_small():
    small = np.zeros((40, 60, 3), np.uint8)
    out = co_ocr.upscale_if_small(small, min_dim=320)
    assert max(out.shape[:2]) == 320
    big = np.zeros((400, 400, 3), np.uint8)
    assert co_ocr.upscale_if_small(big, 320) is big


def test_circularity_on_synthetic_circle():
    canvas = np.zeros((200, 200, 3), np.uint8)
    cv2.circle(canvas, (100, 100), 80, (255, 255, 255), -1)
    ok, fraction, circ = co_ocr.circularity_of_bright(canvas)
    assert ok and circ > 0.8


# ---------------------------------------------------------------- 卡片分类数据集

bcd = load_module("build_card_dataset", ML / "build_card_dataset.py")


def test_read_subtype_mapping(tmp_path):
    csv_path = tmp_path / "mapping.csv"
    csv_path.write_text(
        "cluster,subtype\n0\n1\n2,grenade\n", encoding="utf-8")
    mapping = bcd.read_subtype_mapping(csv_path)
    assert mapping == {2: "grenade"}


def test_nearest_cluster():
    reps = [0b0000, 0b1111]
    assert bcd.nearest_cluster(0b0001, reps, 10) == 0
    assert bcd.nearest_cluster(0b1101, reps, 10) == 1
    assert bcd.nearest_cluster(0b0011_0000, reps, 1) is None
    assert bcd.nearest_cluster(0b0000, [], 10) is None


# ---------------------------------------------------------------- 补标函数更新

def test_median_spacing():
    pts = [(0, 0), (100, 0), (0, 100), (100, 100)]
    assert supplement_m.median_spacing(pts) == pytest.approx(100.0)
    assert supplement_m.median_spacing([(0, 0)]) == 200.0


def test_hull_mask_expand(tmp_path):
    pts = [(20, 20), (80, 20), (80, 80), (20, 80)]
    bare = supplement_m.hull_mask(pts, 100, 100)
    expanded = supplement_m.hull_mask(pts, 100, 100, expand_px=10)
    assert expanded.sum() > bare.sum()


def test_entrance_from_arrow_clamps_into_frame():
    # 中心在画面外缘 → 框整体夹回画面内，不越界
    line = supplement_m.entrance_from_arrow([0.02, 0.5, 0.06, 0.04])
    v = list(map(float, line.split()))
    assert int(v[0]) == 3
    assert v[1] - v[3] / 2 >= 0.0
    # 中心在画面中部 → 不动
    line2 = supplement_m.entrance_from_arrow([0.5, 0.5, 0.06, 0.04])
    v2 = list(map(float, line2.split()))
    assert v2[1] == pytest.approx(0.5)


# ---------------------------------------------------------------- 跨帧传播

propagate_m = load_module("propagate_labels", ML / "propagate_labels.py")


def test_map_box_identity_and_out_of_frame():
    box = (0.5, 0.5, 0.2, 0.2)
    out = propagate_m.map_box(np.eye(3), box)
    assert out is not None
    assert out[0] == pytest.approx(0.5)
    assert out[2] == pytest.approx(0.2)
    # 平移把中心推出画面 → None
    H = np.array([[1, 0, 1.5], [0, 1, 0], [0, 0, 1]], dtype=np.float64)
    assert propagate_m.map_box(H, box) is None


def test_tally_votes():
    assert propagate_m.tally_votes({4: 3, 5: 1}) == (4, True)
    assert propagate_m.tally_votes({4: 1}) == (4, False)
    # 平票 / 无票 → 无结论
    assert propagate_m.tally_votes({4: 2, 5: 2}) == (None, False)
    assert propagate_m.tally_votes({}) == (None, False)


def test_box_iou_basic():
    a = (0.5, 0.5, 0.2, 0.2)
    assert propagate_m.box_iou(a, (0.5, 0.5, 0.2, 0.2)) == pytest.approx(1.0)
    assert propagate_m.box_iou(a, (0.9, 0.9, 0.2, 0.2)) == pytest.approx(0.0)
