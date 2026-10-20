"""HbbDetector 后处理 Python 等价实现测试（tools/ml/hbb_postprocess.py）。

守护点：
  - 解码 + 按类别置信度过滤（13 类 / 17 通道）
  - **居中 letterbox 的 pad 反算**（2026-10-20 与 C++ 一并从 top-left 改过来）
  - 三处舍入口径：nw/nh 用 half-to-even、框坐标用 int 截断、阈值边界为严格大于
  - 按类别 NMS
  - paper/格号重叠过滤（印刷格号被误检成雷贴纸的对策）
  - paper 边缘过滤
  - 类别表与阈值必须与 C++ 真源（types.hpp / main.cpp / HbbDetector.cpp）
    以及已验证的 PC 参考实现（video_recon.py）逐项一致
  - C++ 侧的居中 letterbox 本身（源码级反查，离线期唯一的防线）

后两条是本文件存在的理由：hbb_postprocess.py 自称"与 C++ 逐行对应"，而在
2026-10-20 之前它其实是 7 类 + top-left + edge 0.01，三处都与实机不符。
一个没人锁的镜像一定会漂移，所以这里直接去解析 C++ 源码反查。

本文件全部断言均经突变测试验证（16/16 killed，含"C++ 与镜像同时漂移"这一
单文件突变看不见的失效模式，见 2026-10-20 记录）；新增断言请同样跑一遍
"改坏它是否变红"，不红的断言等于没写。
"""

from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
ML = REPO / "tools" / "ml"
MAIN_CPP = REPO / "onboard" / "dzuav" / "src" / "main.cpp"
HBB_CPP = REPO / "onboard" / "dzuav" / "src" / "HbbDetector.cpp"
TYPES_HPP = REPO / "onboard" / "dzuav" / "include" / "dzuav" / "types.hpp"


def _load(name, file):
    spec = importlib.util.spec_from_file_location(name, file)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


hbbpp = _load("hbb_postprocess", ML / "hbb_postprocess.py")
cv2 = pytest.importorskip("cv2")

NUM_CLASSES = hbbpp.NUM_CLASSES
CHANNELS = 4 + NUM_CLASSES


def _raw_one(cls_score_pairs, n_class=NUM_CLASSES):
    """构造 [1, 4+n_class, N]：每个 anchor 给 (cx,cy,w,h, cls, score)。"""
    n = len(cls_score_pairs)
    arr = np.zeros((4 + n_class, n), dtype=np.float32)
    for i, (cx, cy, w, h, cls, score) in enumerate(cls_score_pairs):
        arr[0, i] = cx
        arr[1, i] = cy
        arr[2, i] = w
        arr[3, i] = h
        arr[4 + cls, i] = score
    return arr[None]


# ------------------------------------------------------------------ 解码 / 阈值


def test_decode_and_conf_filter():
    # anchor 0: paper cls=0 score=0.9 → 保留
    # anchor 1: paper cls=0 score=0.1 → 过滤（< 0.25）
    # anchor 2: obstacle cls=5 score=0.8 → 保留
    raw = _raw_one([
        (100, 100, 20, 20, 0, 0.9),
        (200, 200, 20, 20, 0, 0.1),
        (300, 300, 30, 30, 5, 0.8),
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    assert sorted(d.cls for d in dets) == [0, 5]


def test_per_class_conf_threshold():
    """paper 阈值 0.5、obstacle 阈值 0.2：paper 0.4 被滤，obstacle 0.3 保留。"""
    raw = _raw_one([
        (100, 100, 20, 20, 0, 0.4),   # paper 0.4 < 0.5 → 过滤
        (300, 300, 30, 30, 5, 0.3),   # obstacle 0.3 > 0.2 → 保留
    ])
    conf = list(hbbpp.DEFAULT_CONF_PER_CLASS)
    conf[0], conf[5] = 0.5, 0.2
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280, conf_per_class=conf)
    assert len(dets) == 1
    assert dets[0].cls == 5


def test_conf_boundary_is_exclusive_end_to_end():
    """score **恰好等于**阈值时，两侧都丢。

    直觉会以为该保留——因为 HbbDetector.cpp 的前置过滤写的是
    `if (score < conf_per_class_[cls]) continue;`，等于阈值确实过得去。
    但它紧接着把同一个 conf 传给了 `cv::dnn::NMSBoxes(b, q, conf_per_class_[c], ...)`，
    而 OpenCV 的 NMSBoxes 内部判据是 `scores[i] > score_threshold`（严格大于）。
    净效果 = 严格大于，本镜像走同一条 cv2.dnn.NMSBoxes，所以天然同口径。

    别为了"修"这条去把前置过滤改成 `<=` 或给 NMSBoxes 传 0：那会让 PC 与机上
    在阈值附近分叉，而阈值附近正是调参时最常盯的区间。
    """
    assert hbbpp.postprocess(_raw_one([(640, 640, 20, 20, 0, 0.25)]),
                             orig_w=1280, orig_h=1280) == []
    assert len(hbbpp.postprocess(_raw_one([(640, 640, 20, 20, 0, 0.26)]),
                                 orig_w=1280, orig_h=1280)) == 1


def test_cpp_passes_conf_into_nms_not_zero():
    """锁住上一条成立的前提：C++ 把 per-class conf 传进 NMSBoxes 作分数阈值。

    这是 test_conf_boundary_is_exclusive_end_to_end 的机制断言。有人把第三个实参
    改成 0.F（"反正前面已经过滤过了"）时，机上净判据会变成 >=，而本镜像仍是 >，
    边界分数就此分叉——这条会先红。
    """
    text = HBB_CPP.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"cv::dnn::NMSBoxes\(([^;]*?)\);", text, re.S)
    assert m, "HbbDetector.cpp 里找不到 cv::dnn::NMSBoxes 调用"
    args = [a.strip() for a in m.group(1).split(",")]
    assert len(args) == 5, f"NMSBoxes 实参个数 {len(args)}，与镜像假设不符: {args}"
    assert args[2] == "conf_per_class_[c]", (
        f"NMSBoxes 的分数阈值实参变成了 {args[2]!r}；"
        f"镜像按 conf_per_class_[c] 建模（见 test_conf_boundary_is_exclusive_end_to_end）"
    )


# ------------------------------------------------------------------ letterbox


def test_letterbox_params_are_centered():
    """1920x1080 → s=2/3、缩放 1280x720、上下各补 280（不是只补下面 560）。"""
    s, nw, nh, pad_x, pad_y = hbbpp.letterbox_params(1920, 1080)
    assert s == pytest.approx(1280 / 1920)
    assert (nw, nh) == (1280, 720)
    assert (pad_x, pad_y) == (0, 280)


def test_canvas_center_decodes_to_image_center():
    """居中 letterbox 的核心不变量：画布中心必须解回原图中心。

    top-left 对齐时这一条会给出 y=960（=(640-0)/(2/3)）而不是 540，
    即所有框整体下偏 420 个原图像素——占 1080 高的 39%。
    """
    raw = _raw_one([(640, 640, 60, 60, 0, 0.9)])
    dets = hbbpp.postprocess(raw, orig_w=1920, orig_h=1080, edge_paper_thr=0.0)
    assert len(dets) == 1
    d = dets[0]
    assert d.cx == pytest.approx(960, abs=3), f"中心 x 偏了: {d.cx}"
    assert d.cy == pytest.approx(540, abs=3), (
        f"中心 y={d.cy}，居中 letterbox 应为 540；"
        f"若接近 960 说明解码没减 pad_y（退回 top-left 口径）")


def test_square_input_has_zero_pad_so_old_and_new_agree():
    """1280x1280 输入下 pad=0，居中与 top-left 结果相同。

    这正解释了为什么旧测试全用 1280x1280 却从来没抓到这个偏差。
    """
    s, nw, nh, pad_x, pad_y = hbbpp.letterbox_params(1280, 1280)
    assert (nw, nh, pad_x, pad_y) == (1280, 1280, 0, 0)
    assert s == pytest.approx(1.0)


def test_wide_and_tall_inputs_both_centered():
    """竖幅（1080x1920）左右补边，同样要居中。"""
    s, nw, nh, pad_x, pad_y = hbbpp.letterbox_params(1080, 1920)
    assert (nw, nh) == (720, 1280)
    assert (pad_x, pad_y) == (280, 0)
    assert s == pytest.approx(1280 / 1920)


def test_letterbox_rounding_is_half_to_even_like_reference():
    """nw/nh 的取整必须是 half-to-even，与 C++ `cvRound`、video_recon 的 `round()` 一致。

    1437x2560 → s=0.5 → 1437*0.5 = 718.5，正是两种舍入唯一可分辨的地方：
    half-to-even 给 718，half-away-from-zero（`std::lround` / `floor(v+.5)`）给 719。

    这条曾经写反过（2026-10-20）：C++ 里先用了 `std::lround`，再写测试把镜像锁成
    lround——突变测试全绿，因为它只查"镜像与 C++ 一致"，查不出"两个一起偏离参考"。
    参考实现以 `video_recon.py` 的 `round(w*scale)` 为准（已核源码，它是产出
    106/106 那组指标的实现）；OpenCV 的 `cvRound` 实测同口径（718.5→718、
    719.5→720）。Ultralytics letterbox 推断亦为 half-to-even，但离线机上没装
    ultralytics，未能直接核源码——所以下面锁的是"镜像 / C++ / video_recon"三方，
    不含无法核验的一方。
    """
    s, nw, nh, pad_x, _ = hbbpp.letterbox_params(1437, 2560)
    assert s == pytest.approx(0.5)
    assert nw == 718, f"nw={nw}，half-to-even 应为 718（719 = 用了 lround 口径）"
    assert nh == 1280
    assert pad_x == (1280 - 718) // 2

    recon = (REPO / "src" / "ag_integrate" / "ag_integrate" / "video_recon.py"
             ).read_text(encoding="utf-8", errors="replace")
    assert re.search(r"nw,\s*nh\s*=\s*round\(w\s*\*\s*scale\),\s*round\(h\s*\*\s*scale\)",
                     recon), (
        "video_recon.D13Detector.detect 的取整方式变了（原为 Python round，即 "
        "half-to-even）。它是产出 106/106 那组指标的参考实现，改口径要连带改本镜像与 "
        "HbbDetector.cpp，否则三方分叉。")


def test_boxes_truncate_toward_zero_like_cpp_int_cast():
    """C++ 用 `int(x)` 截断（向零取整），不是四舍五入；镜像必须同口径。

    s=1.0、pad=0 时 x = cx - bw/2 = 100.7 - 10 = 90.7 → C++ 给 90，
    若镜像改用 np.rint 会给 91。1px 之差在格号 ROI 裁剪与 IoU=0.30 边界上可放大。
    """
    raw = _raw_one([(100.7, 640, 20, 20, 0, 0.9)])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    assert len(dets) == 1
    assert dets[0].x == 90, f"x={dets[0].x}，应为截断后的 90（四舍五入会给 91）"


def test_cpp_letterbox_is_centered():
    """源码级锁住 C++ 的居中 letterbox——本批最吃重的改动，也是无硬件时唯一的防线。

    此前 C++ 与镜像都是 top-left 补边，而所有可信指标（mine R98.3% / arrow R88.9%
    / P100%、val-B +0.76pp）都出自 Ultralytics 的**居中** letterbox。谁把 pad 改回 0
    或改回 `canvas(cv::Rect(0, 0, ...))`，机上输入分布就与指标来源分叉：
    1920x1080 下画面整体上移 280px。C++ 改动在离线期只算"已写待验"，
    所以这一层只能靠解析源码来守。
    """
    text = HBB_CPP.read_text(encoding="utf-8", errors="replace")
    checks = {
        "pad_x 居中": r"pad_x\s*=\s*\(kInputSide\s*-\s*nw\)\s*/\s*2",
        "pad_y 居中": r"pad_y\s*=\s*\(kInputSide\s*-\s*nh\)\s*/\s*2",
        "缩放图贴到 (pad_x, pad_y)": r"canvas\(cv::Rect\(pad_x,\s*pad_y,\s*nw,\s*nh\)\)",
        "解码 x 减回 pad_x": r"-\s*pad_x\)\s*/\s*s",
        "解码 y 减回 pad_y": r"-\s*pad_y\)\s*/\s*s",
        # cols / rows 分列两条：合成一条 `(?:cols|rows)` 时，只改坏一侧仍能匹配，
        # 突变测试实测存活（见 2026-10-20 mutation 记录）。
        "nw 用 cvRound": r"cvRound\(image\.cols\s*\*\s*s\)",
        "nh 用 cvRound": r"cvRound\(image\.rows\s*\*\s*s\)",
        # s 必须是 double：cvRound(float 乘积) 与 cvRound(double 乘积) 在边界上会分叉，
        # 而参考实现（video_recon / Ultralytics）都在 float64 下算。
        "scale 用 double": r"const\s+double\s+s\s*=\s*std::min\(",
    }
    missing = [name for name, pat in checks.items() if not re.search(pat, text)]
    assert not missing, (
        f"HbbDetector.cpp 的居中 letterbox 被改动，缺失: {missing}。"
        f"若确要改回 top-left 或换取整口径，必须同时改 hbb_postprocess.py 与本文件——"
        f"否则 PC 评估口径与机上分叉，且既有指标全部失效。"
        f"注意取整口径的真源是 video_recon.py 的 round()（half-to-even），"
        f"不是随手挑一个看起来更『正确』的舍入。"
    )


# ------------------------------------------------------------------ NMS


def test_nms_same_class_suppresses():
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
        (100, 100, 40, 40, 5, 0.8),
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    assert sorted(d.cls for d in dets) == [0, 5]


# ---------------------------------------------------- paper/格号重叠过滤


def _paper_and_label(dx: int):
    """一个 cell_label 框 + 一个横向偏 dx 的 paper 框（都在画面中央，避开边缘过滤）。"""
    return _raw_one([
        (640, 640, 80, 40, 1, 0.9),                       # cell_label
        (640 + dx, 640, 80, 40, 0, 0.88),                 # paper，实测误检分 0.58/0.88
    ])


def test_label_overlap_drops_paper_printed_on_cell_label():
    """印刷格号被 paper 误检（分数 0.88）时必须靠重叠关系剔掉。

    0.88 远高于任何可用 paper 阈值，**调阈值拦不住**——这一层是唯一防线。
    漏掉的后果是车端把印在地上的字当成雷：浪费弹药，或把可通行格判成雷区。
    """
    dets = hbbpp.postprocess(_paper_and_label(0), orig_w=1280, orig_h=1280)
    assert [d.cls for d in dets] == [1], "与格号完全重叠的 paper 没被剔掉"


def test_label_overlap_keeps_distant_paper():
    """真雷贴纸贴在格心、印刷格号在格角，物理上不重叠 → 必须保留。"""
    dets = hbbpp.postprocess(_paper_and_label(400), orig_w=1280, orig_h=1280)
    assert sorted(d.cls for d in dets) == [0, 1]


def test_label_overlap_keeps_paper_when_no_label_detected():
    """画面里没有格号框时，这一层不许误伤 paper。"""
    raw = _raw_one([(640, 640, 80, 40, 0, 0.88)])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    assert [d.cls for d in dets] == [0]


def test_label_overlap_does_not_touch_other_classes():
    """只滤 paper：obstacle 压着格号也要保留（桶/障碍确实可能压在格号上）。"""
    raw = _raw_one([
        (640, 640, 80, 40, 1, 0.9),
        (640, 640, 80, 40, 5, 0.8),
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    assert sorted(d.cls for d in dets) == [1, 5]


def test_label_overlap_can_be_disabled():
    dets = hbbpp.postprocess(_paper_and_label(0), orig_w=1280, orig_h=1280,
                             label_overlap_iou=0.0)
    assert sorted(d.cls for d in dets) == [0, 1]


# ------------------------------------------------------------------ 边缘过滤


def test_paper_edge_filter_removes_edge():
    raw = _raw_one([
        (6, 640, 12, 12, 0, 0.9),    # cx=6px / 1280 = 0.47% < 1% → 边缘
        (640, 640, 20, 20, 0, 0.9),  # 中心 → 保留
    ])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280, edge_paper_thr=0.01)
    assert len(dets) == 1
    assert dets[0].cx == pytest.approx(640)


def test_paper_edge_filter_disabled():
    raw = _raw_one([(6, 640, 12, 12, 0, 0.9)])
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280, edge_paper_thr=0.0)
    assert len(dets) == 1


def test_edge_filter_only_affects_paper():
    raw = _raw_one([(6, 640, 30, 30, 5, 0.9)])    # obstacle 在边缘 → 保留
    dets = hbbpp.postprocess(raw, orig_w=1280, orig_h=1280, edge_paper_thr=0.01)
    assert len(dets) == 1
    assert dets[0].cls == 5


# ------------------------------------------------------------------ 契约 / 兜底


def test_empty_output():
    raw = np.zeros((1, CHANNELS, hbbpp.ANCHORS), dtype=np.float32)
    assert hbbpp.postprocess(raw, orig_w=1280, orig_h=1280) == []


def test_rejects_seven_class_model_with_actionable_message():
    """11 通道 = 7 类旧模型，报错必须直接说出来，不能只丢个维度不匹配。"""
    raw = np.zeros((1, 11, 33600), dtype=np.float32)
    with pytest.raises(ValueError) as exc:
        hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)
    assert "7 类旧模型" in str(exc.value)


def test_rejects_bad_channels():
    raw = np.zeros((1, 9, 100), dtype=np.float32)
    with pytest.raises(ValueError):
        hbbpp.postprocess(raw, orig_w=1280, orig_h=1280)


# ------------------------------------------- 与 C++ / video_recon 真源逐项对齐


def _cpp_const(path: Path, name: str) -> float:
    text = path.read_text(encoding="utf-8", errors="replace")
    m = re.search(rf"constexpr\s+float\s+{name}\s*=\s*([0-9.]+)F?\s*;", text)
    assert m, f"{path.name} 里找不到 constexpr float {name}"
    return float(m.group(1))


def _cpp_kDeployConf() -> list[tuple[float, str]]:
    """从 main.cpp 解析 kDeployConf，连每项后面的类别名注释一起取。"""
    text = MAIN_CPP.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"kDeployConf\s*=\s*\{(.*?)\};", text, re.S)
    assert m, "main.cpp 里找不到 kDeployConf 初始化列表"
    pairs = re.findall(r"([0-9.]+)F\s*,\s*//\s*(\w+)", m.group(1))
    assert pairs, "kDeployConf 的每一项都必须带 `// 类别名` 注释，否则无法核对顺序"
    return [(float(v), name) for v, name in pairs]


def test_class_names_match_types_hpp():
    """类别表必须与 types.hpp 的 ObjectClass 逐项同名同序。

    顺序错一位不会报错，只会让"障碍被当蓝靶"——这个事故在
    orin_run_dzuav.sh 的历史坑注释里有记录。
    """
    text = TYPES_HPP.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"enum\s+class\s+ObjectClass\b[^{]*\{(.*?)\}", text, re.S)
    assert m, "types.hpp 里找不到 enum class ObjectClass"
    members = re.findall(r"^\s*([A-Za-z_]\w*)\s*=\s*(\d+)", m.group(1), re.M)
    assert members, "ObjectClass 成员解析为空"
    # CamelCase → snake_case（CellLabel → cell_label，BucketCyan → bucket_cyan）
    snake = [re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower() for name, _ in members]
    indices = [int(i) for _, i in members]
    assert indices == list(range(len(indices))), f"ObjectClass 编号不连续: {members}"
    assert snake == hbbpp.CLASS_NAMES, (
        f"类别表与 types.hpp 不一致（顺序或命名）:\n"
        f"  types.hpp       : {snake}\n"
        f"  hbb_postprocess : {hbbpp.CLASS_NAMES}")


def test_class_names_match_video_recon_reference():
    """同一份类别表还要与已验证的 PC 参考实现 video_recon.NAMES 一致。"""
    text = (REPO / "src" / "ag_integrate" / "ag_integrate" / "video_recon.py"
            ).read_text(encoding="utf-8", errors="replace")
    m = re.search(r"^NAMES\s*=\s*\[(.*?)\]", text, re.S | re.M)
    assert m, "video_recon.py 里找不到 NAMES"
    names = re.findall(r'"([^"]+)"', m.group(1))
    assert names == hbbpp.CLASS_NAMES, (
        f"video_recon.NAMES 与 hbb_postprocess.CLASS_NAMES 不一致:\n"
        f"  video_recon     : {names}\n"
        f"  hbb_postprocess : {hbbpp.CLASS_NAMES}")


def test_default_conf_matches_main_cpp():
    """DEFAULT_CONF_PER_CLASS 必须逐项等于 main.cpp 的 kDeployConf（含顺序）。"""
    cpp = _cpp_kDeployConf()
    assert [name for _, name in cpp] == hbbpp.CLASS_NAMES, (
        f"kDeployConf 的类别注释顺序与 CLASS_NAMES 不符: {[n for _, n in cpp]}")
    assert [v for v, _ in cpp] == pytest.approx(hbbpp.DEFAULT_CONF_PER_CLASS), (
        f"阈值不同步:\n  main.cpp     : {[v for v, _ in cpp]}\n"
        f"  hbb_postproc : {hbbpp.DEFAULT_CONF_PER_CLASS}")


def test_nms_edge_and_overlap_defaults_match_cpp():
    assert hbbpp.DEFAULT_NMS == pytest.approx(_cpp_const(MAIN_CPP, "kDeployNms"))
    assert hbbpp.DEFAULT_EDGE_PAPER == pytest.approx(
        _cpp_const(MAIN_CPP, "kEdgePaperThr"))
    assert hbbpp.LABEL_OVERLAP_IOU == pytest.approx(
        _cpp_const(HBB_CPP, "kLabelOverlapIou"))


def test_cell_read_gate_matches_reference():
    """格号读数下限与 video_recon.V4D_MIN_CONF 同值——这一对是绑定调的。

    机载没有时序投票（参考实现每个落格要 VOTE_MIN=2 票），所以读数门槛不能
    比参考实现松：少一个地标只是样本变少，错一个地标会把位姿带偏一格以上。
    """
    gate = _cpp_const(MAIN_CPP, "kCellMinConf")
    text = (REPO / "src" / "ag_integrate" / "ag_integrate" / "video_recon.py"
            ).read_text(encoding="utf-8", errors="replace")
    m = re.search(r"^V4D_MIN_CONF\s*=\s*([0-9.]+)", text, re.M)
    assert m, "video_recon.py 里找不到 V4D_MIN_CONF"
    assert gate == pytest.approx(float(m.group(1))), (
        f"机载 kCellMinConf={gate} 与参考实现 V4D_MIN_CONF={m.group(1)} 不一致")


def test_cell_label_detect_threshold_matches_reference():
    """cell_label 检测阈值与参考实现同值：宽检测 + 严读数是同一套口径的两半。"""
    cpp = dict((name, v) for v, name in _cpp_kDeployConf())
    text = (REPO / "src" / "ag_integrate" / "ag_integrate" / "video_recon.py"
            ).read_text(encoding="utf-8", errors="replace")
    m = re.search(r'"cell_label"\s*:\s*([0-9.]+)', text)
    assert m, "video_recon.CONF 里找不到 cell_label"
    assert cpp["cell_label"] == pytest.approx(float(m.group(1))), (
        f"cell_label 检测阈值不同步: main.cpp={cpp['cell_label']} "
        f"video_recon={m.group(1)}")
