# -*- coding: utf-8 -*-
r"""雷 / 障 / 靶区检测单测：两套几何口径（图纸 vs 实物）+ 拒收与冲突消解。

合成色板故意让 **雷=蓝**（而不是图纸的黄），这样"雷是黄的"这类**未取证假设**
不可能靠测试蒙混过关；图纸口径则单独用 ``MAP_IMAGE`` 调色板 + ``drawing`` 风格验证。
"""

from __future__ import annotations

import pathlib
import re

import numpy as np
from PIL import Image, ImageDraw

from ..colors import MAP_IMAGE
from ..grid import CORNER_ORDER, GridSpec, register_from_corners
from ..objects import (FIELD_SPECS, MAP_IMAGE_SPECS, detect_field_objects,
                       group_blocks)
from ..pipeline import PerceptionPipeline, config_for
from .synth import SYNTH_FIELD_PALETTE, render_arena

#: 真实图纸与其识图报告（只读引用；缺失时跳过对应测试）
V1 = pathlib.Path(__file__).resolve().parents[4]
MAP_SRC = V1 / "dzrobot_v2" / "_tmp_session" / "t28_newmap" / "map_src.png"
IDENTIFY = V1 / "dzrobot_v2" / "_tmp_session" / "t28_newmap" / "identify_report.txt"
MAP_YAML = V1 / "dzrobot_v2" / "config" / "map.yaml"
#: ★t59：``map_src.png`` 是**第 4/5 张图**的图纸（当时 ``config/map.yaml`` 的真值）。
#:   2026-10-14 官方贴图重掷后 ``map.yaml`` 已换成**新图** ⇒ 图纸回归必须钉住
#:   "这张图纸自己画的是什么"，不能再拿它会变的 ``map.yaml`` 当真值。
#:   与"真值 == 现役 map.yaml"的端到端对账见 ``test_field_map.py``（用 map_official.png）。
DRAWING_TRUTH = {
    "mines": ["A4", "B1", "D8", "E11", "F9", "G2", "H3", "I10"],
    "obstacles": ["A5", "B7", "C9", "D1", "E10", "F4", "G11", "H5"],
    "targets": ["B11", "C11"],
    "bridges": ["K10", "K5"],
}


def _register(arena, spec: GridSpec | None = None):
    spec = spec or arena.spec
    return register_from_corners([arena.corners_px[t] for t in CORNER_ORDER], spec)


def test_field_objects_round_trip_exact():
    """实物口径：雷=12cm 圆片、桶=30cm 圆 —— 必须一格不差地读回来。"""
    spec = GridSpec()
    truth_mines = ["B1", "G2", "H3", "A4", "D8"]
    truth_obst = ["D1", "F4", "A5", "H5"]
    truth_targets = ["B11", "C11"]
    arena = render_arena(spec, mines=truth_mines, obstacles=truth_obst,
                         targets=truth_targets)
    reg = _register(arena)
    objs = detect_field_objects(arena.image, reg, SYNTH_FIELD_PALETTE, FIELD_SPECS)
    assert objs.mines == sorted(truth_mines), (objs.mines, objs.rejected)
    assert objs.obstacles == sorted(truth_obst)
    assert objs.targets == sorted(truth_targets)
    assert objs.target_groups == [["B11", "C11"]]      # 横向相邻 ⇒ 一块跨格靶板
    assert all(d.confidence > 0.3 for d in objs.detections)


def test_drawing_palette_and_geometry_round_trip():
    """图纸口径：黄雷块 / 红障块 / 品红靶块 / 绿通块 —— 用 MAP_IMAGE 阈值读回来。"""
    spec = GridSpec()
    truth = {"mine": ["B1", "G2", "H3", "A4", "D8", "F9", "I10", "E11"],
             "obstacle": ["D1", "F4", "A5", "H5", "B7", "C9", "E10", "G11"],
             "target": ["B11", "C11"],
             "bridge": ["K5", "K10"]}
    arena = render_arena(spec, style="drawing", mines=truth["mine"],
                         obstacles=truth["obstacle"], targets=truth["target"],
                         bridges=truth["bridge"], draw_border=False)
    reg = _register(arena)
    objs = detect_field_objects(arena.image, reg, MAP_IMAGE, MAP_IMAGE_SPECS)
    for cls, blocks in truth.items():
        assert objs.blocks_of(cls) == sorted(blocks), (cls, objs.blocks_of(cls),
                                                       objs.rejected[:4])


def test_rejection_reasons_are_recorded():
    """面积/填充率不达标的 blob 必须留档（而不是静默丢弃）。"""
    spec = GridSpec()
    arena = render_arena(spec, mines=[], obstacles=[])
    reg = _register(arena)
    objs = detect_field_objects(arena.image, reg, SYNTH_FIELD_PALETTE, FIELD_SPECS)
    assert objs.mines == [] and objs.obstacles == []
    # 造一个巨大的蓝色块（跨 3 格），应被"面积比超上限"拒收
    big = arena.image.copy()
    from PIL import ImageDraw
    from PIL import Image
    img = Image.fromarray(big)
    d = ImageDraw.Draw(img)
    p = reg.cell_polygon_px("F6")
    d.rectangle([p[:, 0].min() - 40, p[:, 1].min() - 40,
                 p[:, 0].max() + 40, p[:, 1].max() + 40], fill=(40, 90, 220))
    objs2 = detect_field_objects(np.asarray(img), reg, SYNTH_FIELD_PALETTE, FIELD_SPECS)
    assert any("面积比" in r["reason"] for r in objs2.rejected), objs2.rejected


def test_same_cell_conflict_resolved_by_priority():
    """同格同时命中障(红盘)与雷(蓝盘) ⇒ 按 priority 保留障，并记录冲突。"""
    spec = GridSpec()
    arena = render_arena(spec, mines=["D1"], obstacles=["D1"])   # 先画红盘、再叠蓝盘
    reg = _register(arena)
    objs = detect_field_objects(arena.image, reg, SYNTH_FIELD_PALETTE, FIELD_SPECS)
    assert objs.obstacles == ["D1"]
    assert objs.mines == []
    assert any("D1" in c for c in objs.conflicts), objs.conflicts


def test_group_blocks_merges_adjacent_cells():
    spec = GridSpec()
    assert group_blocks(["B11", "C11"], spec) == [["B11", "C11"]]      # 横向相邻
    assert group_blocks(["E4", "E5"], spec) == [["E4", "E5"]]          # 纵向相邻
    assert group_blocks(["B11", "D11"], spec) == [["B11"], ["D11"]]    # 中间隔一格
    assert group_blocks([], spec) == []
    # 规则 p6：单局只有一个"相连的两个单元" ⇒ 检查组内确实是 2 格
    g = group_blocks(["B11", "C11"], spec)
    assert len(g) == 1 and len(g[0]) == 2


def test_objects_are_deterministic():
    spec = GridSpec()
    arena = render_arena(spec, mines=["B1", "G2"], obstacles=["D1"], targets=["B11"])
    reg = _register(arena)
    a = detect_field_objects(arena.image, reg, SYNTH_FIELD_PALETTE, FIELD_SPECS)
    b = detect_field_objects(arena.image, reg, SYNTH_FIELD_PALETTE, FIELD_SPECS)
    assert a.to_json() == b.to_json()


def test_rotated_and_tilted_frame():
    spec = GridSpec()
    arena = render_arena(spec, rot_deg=23.0, tilt=0.22, mines=["B1", "H3"],
                         obstacles=["D1", "A5"], targets=["B11", "C11"])
    reg = _register(arena)
    objs = detect_field_objects(arena.image, reg, SYNTH_FIELD_PALETTE, FIELD_SPECS)
    assert objs.mines == ["B1", "H3"]
    assert objs.obstacles == ["A5", "D1"]
    assert objs.targets == ["B11", "C11"]


def test_yellow_label_box_with_text_is_rejected_as_non_target():
    """图纸里的黄色**标注框**（起终点/无人机点，框内印字）不能当成雷。

    判据来自实测：真雷 bbox 内深色像素占比 ≤0.0014，标注框 0.086~0.105 ⇒ 阈值 0.02。
    """
    spec = GridSpec()
    arena = render_arena(spec, style="drawing", mines=["B1"], draw_border=False)
    reg = _register(arena)
    objs = detect_field_objects(arena.image, reg, MAP_IMAGE, MAP_IMAGE_SPECS)
    assert objs.mines == ["B1"]
    # 在 H9 画一个"黄框 + 深色文字"的标注框
    img = Image.fromarray(arena.image.copy())
    d = ImageDraw.Draw(img)
    p = reg.cell_polygon_px("H9")
    d.rectangle([p[:, 0].min() + 2, p[:, 1].min() + 2,
                 p[:, 0].max() - 2, p[:, 1].max() - 2], fill=(235, 205, 60))
    d.text((p[:, 0].min() + 10, p[:, 1].min() + 12), "无人机", fill=(60, 60, 60))
    d.text((p[:, 0].min() + 10, p[:, 1].min() + 26), "起点", fill=(60, 60, 60))
    objs2 = detect_field_objects(np.asarray(img), reg, MAP_IMAGE, MAP_IMAGE_SPECS)
    assert objs2.mines == ["B1"], objs2.mines
    assert any("深色" in r["reason"] for r in objs2.rejected), objs2.rejected


def test_real_map_drawing_matches_frozen_drawing_truth():
    """**真实图纸回归**：map_src.png 逐格复现**这张图纸自己的**标注（冻结点位）。

    标定参数取自 ``identify_report.txt`` 原文（格宽/格高/格心相位），不是写死的魔数。

    ★ t59 变更：原来拿 ``config/map.yaml`` 当真值。2026-10-14 官方贴图重掷后
    ``map.yaml`` 已描述**另一张图**（雷 C1/A3/…、靶 E5/F5、通 K5/K9），两者不再同一张图
    ⇒ 本测试改为对比 :data:`DRAWING_TRUTH`（冻结点位，见文件头注释）；
    "真值 == 现役 map.yaml" 的对账由 ``test_field_map.py`` 的官方图端到端测试承担。
    """
    if not (MAP_SRC.is_file() and IDENTIFY.is_file()):
        return                                        # 只读文件不在 ⇒ 跳过
    text = IDENTIFY.read_text(encoding="utf-8")
    m = re.search(r"格宽\s*([\d.]+)\s*px、格高\s*([\d.]+)\s*px；"
                  r"格心相位\s*x=([\d.]+)\s*y=([\d.]+)", text)
    assert m, "识图报告格式变了：找不到格网反推行"
    cw, ch, cx0, cy0 = (float(g) for g in m.groups())
    u0, u1 = cx0 - cw / 2.0, cx0 - cw / 2.0 + 14 * cw
    v0, v1 = cy0 - ch / 2.0, cy0 - ch / 2.0 + 11 * ch
    img = np.asarray(Image.open(MAP_SRC).convert("RGB"))
    res = PerceptionPipeline(config_for("map_image")).process(
        img, corners_px=[(u1, v1), (u1, v0), (u0, v0), (u0, v1)])   # N11,N1,A1,A11
    assert res.objects.mines == DRAWING_TRUTH["mines"], (res.objects.mines,
                                                         DRAWING_TRUTH["mines"])
    assert res.objects.obstacles == DRAWING_TRUTH["obstacles"]
    assert res.objects.targets == DRAWING_TRUTH["targets"]
    assert res.objects.bridges == DRAWING_TRUTH["bridges"]
    # 被剔除的必须是"带文字的标注框"（深色像素占比超限 / 被图标打断导致面积或填充率不达标），
    # 而不是"漏检"——每条剔除都带明确理由。
    assert res.objects.rejected, "真实图纸上应该有被剔除的标注框"
    for r in res.objects.rejected:
        assert ("深色" in r["reason"] or "填充率" in r["reason"]
                or "面积比" in r["reason"]), r
