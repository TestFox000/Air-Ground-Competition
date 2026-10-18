# -*- coding: utf-8 -*-
r"""t5a：片级覆盖 / 负证据报文的单测（structural + e2e）。

三组断言，对应任务卡的三条验收：

1. **合成场景 7 片齐全**：``s3`` 片清单 = 7 片、并集 154/154 格、
   每片 ``coverage.complete=true``；
2. **``complete:false`` 不产生 ABSENT**：不完整片 ``coverage_absent_cells`` 返回
   ``None``（UNKNOWN），而完整片返回补集（真正的负面证据）；
3. **旧路径不回归**：不给 ``--slices`` 时 ``field_verify`` 形状与旧版逐字节相同
   （旧 85 项另有全量回归，本文件只钉协议面）。

e2e 用的是合成图（``tests/synth.py``）：真值已知、可复现，且**不依赖**现场相机/标定。
"""

from __future__ import annotations

import json
import re
import tempfile
from pathlib import Path

from PIL import Image

from .. import cli as cli_mod
from ..car_pose import CarSpec
from ..coverage import (DEFAULT_SLICE_ROUTES, cells_of_route, parse_slice_spec,
                        plan_for)
from ..grid import GridSpec, RegistrationError
from ..objects import FIELD_SPECS
from ..pipeline import PerceptionConfig, PerceptionPipeline
from ..protocol import (COVERAGE_MODES, build_field_verify, coverage_absent_cells,
                        encode, field_verify_absent, make_scan_id, validate_payload)
from .synth import SYNTH_FIELD_PALETTE, render_arena

# ---- 合成全要素场景（8 雷 + 8 障 + 2 靶，跨全部 7 片；K 列留给桥面）----
MINES = ["A1", "C2", "E6", "J4", "L8", "B8", "H11", "N2"]
OBSTACLES = ["C3", "F5", "D9", "G10", "L3", "M8", "B1", "N10"]
TARGETS = ["M2", "M3"]
ARROWS = [("K5", 90.0), ("K9", -90.0)]

_CFG = PerceptionConfig(palette=SYNTH_FIELD_PALETTE, object_specs=FIELD_SPECS,
                        car_spec=CarSpec(cls="__none__"))
_ARENA = None


def _scene():
    """合成场景只渲染一次（~0.1 s），多个用例共用。"""
    global _ARENA
    if _ARENA is None:
        _ARENA = render_arena(GridSpec(), mines=MINES, obstacles=OBSTACLES,
                              targets=TARGETS, deck_col="K", arrows=ARROWS)
    return _ARENA


def _corners(arena):
    return [arena.corners_px[t] for t in ("N11", "N1", "A1", "A11")]


def _process(plan: SlicePlan | None = None, **kw):
    arena = _scene()
    return PerceptionPipeline(_CFG).process(arena.image, corners_px=_corners(arena),
                                           slices=plan, **kw)


def _field_verifies(res):
    return [m for m in res.messages if m["type"] == "field_verify"]


# --------------------------------------------------------------------------- #
#  ① 片清单几何
# --------------------------------------------------------------------------- #
def test_seven_slices_cover_full_field():
    """``s3`` = 7 片、并集 154/154；每片格数逐片钉死（几何回归）。"""
    plan = plan_for("s3")
    reps = plan.report()
    assert [r.slice.id for r in reps] == ["S1", "S2", "S3", "S4", "S5", "S6", "S7"]
    assert [(r.slice.index, r.slice.of) for r in reps] == [
        (i, 7) for i in range(1, 8)]
    assert [len(r.cells) for r in reps] == [15, 24, 12, 42, 42, 42, 28]
    assert all(r.cells for r in reps), "每片都必须有覆盖格"
    union = plan.union_cells()
    assert len(union) == 154 and union == plan.spec.all_blocks(), \
        "7 片并集必须覆盖整场（154 格）"
    # 每片格清单必须是网格序且无重复（否则同一片两次运行无法逐字节比对）
    for r in reps:
        assert list(r.cells) == [b for b in plan.spec.all_blocks() if b in set(r.cells)]
        assert len(set(r.cells)) == len(r.cells)
    # 默认 7 片的航点定义与 t4 §6.3 的 S3' 一致（40 个航点）
    assert sum(len(sl.route) for sl in plan.slices) == 40
    assert DEFAULT_SLICE_ROUTES[0][1] == ("N8", "M8", "L8", "K8")


def test_t1_and_full_presets_and_custom_routes():
    """``t1`` 只覆盖 39/154（= t4 §8.3 实测），``full`` 1 片 154 格。"""
    t1 = plan_for("t1")
    assert [r.slice.id for r in t1.report()] == ["S1", "S2", "S3"]
    assert len(t1.union_cells()) == 39
    assert set(t1.union_cells()) < set(t1.spec.all_blocks())

    full = plan_for("full")
    reps = full.report()
    assert len(reps) == 1 and reps[0].slice.id == "S1"
    assert full.mode == "full" and len(reps[0].cells) == 154

    custom = parse_slice_spec("P1=A8>N8;P2=N11>B11")
    assert [(s.id, s.index, s.of) for s in custom.slices] == [
        ("P1", 1, 2), ("P2", 2, 2)]
    # 走一条整行横带：行 8 上下各 1 行 + 两端圆头 ⇒ 3 行 × 14 列
    assert len(custom.report()[0].cells) == 42

    # 单航点（point 口径）也能出片：3×3 邻域
    one = parse_slice_spec("K5", mode="point")
    assert len(one.report()[0].cells) == 9


def test_parse_slice_spec_rejects_bad_input():
    for bad in ("", "   "):
        try:
            parse_slice_spec(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"空片规格应报错：{bad!r}")
    for bad in ("Z9>N8", "N8>Q1", "A0>N1"):
        try:
            parse_slice_spec(bad)
        except RegistrationError:
            pass
        else:
            raise AssertionError(f"非法航点应报错：{bad!r}")
    try:
        parse_slice_spec("P1=A1>N1;P1=N11>B11")
    except ValueError:
        pass
    else:
        raise AssertionError("重复片 id 应报错")
    # mark_incomplete：按 id 或 1-based 序号；不存在的片必须报错
    plan = plan_for("s3").mark_incomplete(["S2", 5])
    assert [c.complete for c in plan.report()] == [
        True, False, True, True, False, True, True]
    try:
        plan.mark_incomplete(["S99"])
    except ValueError:
        pass
    else:
        raise AssertionError("不存在的片应报错")


def test_cells_of_route_clamps_and_modes():
    """坐标系口径：``corridor_m`` 只影响带子宽度，格心判据不含端点外推。"""
    spec = GridSpec()
    # point 口径 = 3×3 邻域，顺序恒为网格序（行 4/5/6 × 列 J/K/L）
    assert cells_of_route(["K5"], spec, mode="point") == [
        "J4", "K4", "L4", "J5", "K5", "L5", "J6", "K6", "L6"]
    # 走廊比半格还窄 ⇒ 只剩航点所在格
    assert cells_of_route(["K5"], spec, corridor_m=0.2) == ["K5"]
    assert cells_of_route(["K5", "K6"], spec, corridor_m=0.1) == ["K5", "K6"]
    # 走廊里"格心距离 ≤ 1.5 格"才进片：0.525 m 下 K5 的上下各一格也算
    assert cells_of_route(["K5", "K6"], spec, corridor_m=0.525) == [
        "J4", "K4", "L4", "J5", "K5", "L5", "J6", "K6", "L6",
        "J7", "K7", "L7"]
    try:
        cells_of_route(["K5"], spec, mode="who_knows")
    except ValueError:
        pass
    else:
        raise AssertionError("未知 mode 应报错")


# --------------------------------------------------------------------------- #
#  ② 报文结构 + 负证据语义
# --------------------------------------------------------------------------- #
def test_field_verify_coverage_shape():
    msg = build_field_verify(
        [], [], ts=1000.0,
        coverage={"cells": ["A1", "a2", "A3"], "complete": True, "mode": "STRIP"},
        slice={"id": "s1", "index": 1, "of": 7, "route": "N8>M8"},
        scan_id="20261017T0930_abc123", seq=7)
    assert msg["mines"] == [] and msg["obstacles"] == []
    assert msg["coverage"] == {"cells": ["A1", "A2", "A3"], "complete": True,
                               "mode": "strip"}
    assert msg["slice"] == {"id": "S1", "index": 1, "of": 7, "route": "N8>M8"}
    assert msg["scan_id"] == "20261017T0930_abc123"
    assert msg["ts"] == 1000.0 and msg["seq"] == 7
    assert validate_payload(msg, GridSpec()) == []
    # JSON 编码往返（车侧就是收到这个字符串）
    back = json.loads(encode(msg))
    assert back["coverage"]["complete"] is True
    # 只带覆盖、不带 mines/obstacles 也是合法载荷（空片语义）
    slim = build_field_verify(
        coverage={"cells": ["A1"], "complete": True},
        slice={"id": "S1", "index": 1, "of": 1}, scan_id="x")
    assert "mines" not in slim and validate_payload(slim, GridSpec()) == []
    # 旧调用形状不变（逐键相等，没有多出 coverage 等键）
    old = build_field_verify(["b1", "g2"], ["d1"], ts=1000.0)
    assert old == {"type": "field_verify", "mines": ["B1", "G2"],
                   "obstacles": ["D1"], "ts": 1000.0}


def test_build_field_verify_rejects_bad_coverage():
    spec = GridSpec()
    good_cov = {"cells": ["A1", "A2"], "complete": True}
    good_slice = {"id": "S1", "index": 1, "of": 7}
    cases = [
        dict(coverage=good_cov, slice=good_slice),                 # 缺 scan_id
        dict(coverage=good_cov, scan_id="x"),                      # 缺 slice
        dict(coverage={"cells": [], "complete": True},
             slice=good_slice, scan_id="x"),                       # 空格清单
        dict(coverage={"cells": ["A1", "A1"], "complete": True},
             slice=good_slice, scan_id="x"),                       # 重复格
        dict(coverage={"cells": ["Z9"], "complete": True},
             slice=good_slice, scan_id="x"),                       # 非法格
        dict(coverage={"cells": ["A1"], "complete": "yes"},
             slice=good_slice, scan_id="x"),                       # complete 非 bool
        dict(coverage={"cells": ["A1"], "complete": True, "mode": "grid"},
             slice=good_slice, scan_id="x"),                       # mode 不在白名单
        dict(coverage=good_cov,
             slice={"id": "", "index": 1, "of": 7}, scan_id="x"),  # 空 id
        dict(coverage=good_cov,
             slice={"id": "S1", "index": 8, "of": 7}, scan_id="x"),  # index > of
        dict(coverage=good_cov,
             slice={"id": "S1", "index": 1, "of": 7, "route": "Z9>N8"},
             scan_id="x"),                                         # route 非法
        dict(coverage=good_cov, slice=good_slice, scan_id="x y"),  # 空白 scan_id
        dict(coverage=good_cov, slice=good_slice, scan_id="x" * 80),  # 超长
        dict(slice=good_slice, scan_id="x"),                       # 只有 slice/scan_id
    ]
    for kw in cases:
        try:
            build_field_verify(mines=[], obstacles=[], spec=spec, **kw)
        except ValueError:
            continue
        raise AssertionError(f"应拒绝：{kw}")
    # 校验器也要能报出来（同一套规则，发送端自检）
    assert validate_payload({"type": "field_verify", "mines": [],
                             "coverage": {"cells": ["A1"], "complete": "x"},
                             "slice": good_slice, "scan_id": "x"}, spec)
    assert validate_payload({"type": "field_verify", "mines": [],
                             "slice": good_slice, "scan_id": "x"}, spec)
    # 严格模式（给 spec）才查格名；照车侧口径不查
    bad_cell = {"type": "field_verify", "mines": [],
                "coverage": {"cells": ["Z9"], "complete": True},
                "slice": good_slice, "scan_id": "x"}
    assert validate_payload(bad_cell, spec)
    assert validate_payload(bad_cell, None) == []


def test_complete_false_never_produces_absent():
    """验收核心：``complete:false`` ⇒ UNKNOWN，绝不产生 ABSENT。"""
    cells = ["A1", "A2", "A3"]
    reported = ["A1"]
    assert coverage_absent_cells({"cells": cells, "complete": True}, reported) == [
        "A2", "A3"]
    assert coverage_absent_cells({"cells": cells, "complete": False},
                                 reported) is None
    # 缺 complete 键：不许猜（分不清"空的"与"没扫"）
    try:
        coverage_absent_cells({"cells": cells}, reported)
    except ValueError:
        pass
    else:
        raise AssertionError("缺 complete 应报错")
    # 载荷级入口：mines/obstacles/targets 都算"已报出"
    payload = {"type": "field_verify", "mines": ["A1"], "obstacles": ["A2"],
               "targets": ["A3"], "coverage": {"cells": cells, "complete": True}}
    assert field_verify_absent(payload) == []
    payload["coverage"] = {"cells": cells, "complete": False}
    assert field_verify_absent(payload) is None
    # 旧报文（无 coverage）没有负证据语义
    try:
        field_verify_absent({"type": "field_verify", "mines": ["A1"]})
    except ValueError:
        pass
    else:
        raise AssertionError("无 coverage 的报文应报错")


def test_make_scan_id_format_and_determinism():
    a = make_scan_id(1792331234.5, token="a1b2c3")
    b = make_scan_id(1792331234.5, token="a1b2c3")
    assert a == b and re.fullmatch(r"\d{8}T\d{4}_[0-9a-f]{6}", a), a
    # 非法 token 字符被过滤后仍只有 6 位
    c = make_scan_id(1792331234.5, token="a-1 b2_c3")
    assert re.fullmatch(r"\d{8}T\d{4}_[0-9a-f]{6}", c), c
    # 不带 token 时随机 6 位（不相等是期望行为）
    assert make_scan_id(1792331234.5) != make_scan_id(1792331234.5)
    assert set(COVERAGE_MODES) == {"strip", "point", "full"}


# --------------------------------------------------------------------------- #
#  ③ 端到端：7 片齐全 + 不完整片 + 缺类拒发
# --------------------------------------------------------------------------- #
def test_pipeline_seven_slices_full_element_report():
    res = _process(plan_for("s3"), scan_id="20261017T0930_abc123")
    fvs = _field_verifies(res)
    assert len(fvs) == 7
    assert [m["slice"]["id"] for m in fvs] == ["S1", "S2", "S3", "S4", "S5", "S6", "S7"]
    assert [m["slice"]["index"] for m in fvs] == [1, 2, 3, 4, 5, 6, 7]
    assert all(m["slice"]["of"] == 7 for m in fvs)
    assert all(m["coverage"]["complete"] is True for m in fvs)
    assert all(m["scan_id"] == "20261017T0930_abc123" for m in fvs)
    assert {m["coverage"]["mode"] for m in fvs} == {"strip"}

    # 并集 = 整场；逐片格数与几何层一致
    union = set()
    for m in fvs:
        union.update(m["coverage"]["cells"])
    assert len(union) == 154
    assert [len(m["coverage"]["cells"]) for m in fvs] == [15, 24, 12, 42, 42, 42, 28]

    # 全要素：8 雷 / 8 障 / 2 靶 一个不少（跨片并集；同格同片只报一次）
    mines = sorted({b for m in fvs for b in m["mines"]})
    obs = sorted({b for m in fvs for b in m["obstacles"]})
    targets = sorted({b for m in res.messages if m["type"] == "target_zone"
                      for b in m["blocks"]})
    assert mines == sorted(MINES), mines
    assert obs == sorted(OBSTACLES), obs
    assert targets == sorted(TARGETS), targets
    # 每条报文都过发送端自检 + 车侧镜像（不能落进 missing/unknown）
    for m in res.messages:
        assert validate_payload(m, GridSpec()) == [], m
    for parsed in res.diagnostics["mirror_car_parse"]:
        assert parsed["raw_ok"] and "missing_fields" not in parsed["action"], parsed
        assert "未知类型" not in parsed["action"], parsed

    # diagnostics：ABSENT 是"片内未报"的补集（如 S1 15 格只报了 L8/M8）
    cov = res.diagnostics["coverage"]
    assert cov["n_slices"] == 7 and cov["cells_union"] == 154
    assert cov["incomplete_slices"] == []
    assert len(cov["absent_per_slice"]["S1"]) == 15 - 2
    # 各片各说各的：S1 只报 S1 覆盖内的雷（L8），不把别片的雷算进来
    s1 = fvs[0]
    assert s1["mines"] == ["L8"] and s1["obstacles"] == ["M8"]
    assert field_verify_absent(s1) == [c for c in s1["coverage"]["cells"]
                                       if c not in ("L8", "M8")]


def test_pipeline_incomplete_slice_is_unknown():
    """``complete:false`` 的片照旧报看得见的，但**不产生** ABSENT。"""
    plan = plan_for("s3").mark_incomplete(["S4", 6])       # S4 按 id、S6 按序号
    res = _process(plan, scan_id="scanX")
    fvs = _field_verifies(res)
    by_id = {m["slice"]["id"]: m for m in fvs}
    assert by_id["S4"]["coverage"]["complete"] is False
    assert by_id["S6"]["coverage"]["complete"] is False
    assert by_id["S1"]["coverage"]["complete"] is True
    cov = res.diagnostics["coverage"]
    assert cov["incomplete_slices"] == ["S4", "S6"]
    assert cov["absent_per_slice"]["S4"] is None
    assert cov["absent_per_slice"]["S6"] is None
    assert isinstance(cov["absent_per_slice"]["S1"], list)
    # 不完整片仍然"只报看得见的"：S4 里的 A1/C2/B1/C3 该报还得报
    assert "A1" in by_id["S4"]["mines"] and "B1" in by_id["S4"]["obstacles"]
    assert field_verify_absent(by_id["S4"]) is None


def test_pipeline_omit_empty_classes_keeps_old_car_quiet():
    """过渡开关：空类不发空表（旧车侧记 missing_fields，而不是一条假差异）。"""
    # 稀疏场景：只有一个雷，绝大多数片是"扫过且空"——正好是空表的用武之地
    arena = render_arena(GridSpec(), mines=["A1"], obstacles=[], targets=[],
                         deck_col="K")
    plan = plan_for("s3")
    cfg = _CFG
    full = PerceptionPipeline(cfg).process(arena.image, corners_px=_corners(arena),
                                           slices=plan, scan_id="s")
    slim = PerceptionPipeline(cfg).process(arena.image, corners_px=_corners(arena),
                                           slices=plan, scan_id="s",
                                           omit_empty_classes=True)
    full_by = {m["slice"]["id"]: m for m in _field_verifies(full)}
    slim_by = {m["slice"]["id"]: m for m in _field_verifies(slim)}
    # S2（K 列走廊）里没有任何雷/障 ⇒ 空表 vs 省略键
    assert full_by["S2"]["mines"] == [] and full_by["S2"]["obstacles"] == []
    assert "mines" not in slim_by["S2"] and "obstacles" not in slim_by["S2"]
    assert slim_by["S2"]["coverage"]["cells"] == full_by["S2"]["coverage"]["cells"]
    # 有东西的片两版一样（A1 在 S4）
    assert full_by["S4"]["mines"] == ["A1"] and slim_by["S4"]["mines"] == ["A1"]
    for m in slim.messages:
        assert validate_payload(m, GridSpec()) == [], m
    # 旧车侧口径：省掉键 = missing_fields；空表 = 一条"仅本地"的假差异
    from ..protocol import mirror_car_side_parse
    assert "missing_fields" in mirror_car_side_parse(
        encode(slim_by["S2"]))["action"]
    assert "差集" in mirror_car_side_parse(encode(full_by["S2"]))["action"]


def test_pipeline_refuses_coverage_without_object_classes():
    """检不了雷/障/靶就不许声称覆盖（否则"没报"会被当 ABSENT）。"""
    arena = _scene()
    from ..colors import FIELD_DEFAULT
    cfg = PerceptionConfig(palette=FIELD_DEFAULT)        # 实物出厂：只有 deck/arrow
    try:
        PerceptionPipeline(cfg).process(arena.image, corners_px=_corners(arena),
                                        slices=plan_for("s3"))
    except ValueError as exc:
        assert "不能声称覆盖" in str(exc) and "mine" in str(exc)
    else:
        raise AssertionError("缺目标类时出覆盖必须抛错")
    # 不切片时照旧能跑（只出 heartbeat/bridge，不宣称扫过什么）
    res = PerceptionPipeline(cfg).process(arena.image, corners_px=_corners(arena))
    assert [m["type"] for m in res.messages][:1] == ["heartbeat"]
    assert _field_verifies(res) == []


def test_pipeline_slice_messages_are_deterministic():
    """同一帧、同一片清单 ⇒ 报文逐字节相同（``ts``/``seq`` 本就该不同 ⇒ 先剥掉）。

    resend 语义正是建在这个性质上：BEST_EFFORT 要周期重发同一片，除时间戳外
    内容一致才谈得上 ``(scan_id, slice.id)`` 幂等（决策层方案 §5.1-4）。
    """
    plan = plan_for("s3")

    def strip(res):
        return [json.dumps({k: v for k, v in m.items() if k not in ("ts", "seq")},
                           ensure_ascii=False, sort_keys=True)
                for m in res.messages]

    r1 = _process(plan, scan_id="20261017T0930_abc123")
    r2 = _process(plan, scan_id="20261017T0930_abc123")
    assert strip(r1) == strip(r2)
    assert r1.scan_id == r2.scan_id == "20261017T0930_abc123"
    # 片清单本身（几何）也必须可复现
    assert [c.to_json() for c in r1.slices] == [c.to_json() for c in r2.slices]


def test_cli_slices_wiring_and_coverage_report():
    """CLI 真路径：``coverage`` 子命令出片清单，``detect --slices`` 出逐片报文。"""
    parser = cli_mod.build_parser()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        jf = td / "plan.json"
        args = parser.parse_args(["coverage", "--slices", "s3", "--json", str(jf),
                                  "--quiet"])
        assert args.func(args) == 0
        doc = json.loads(jf.read_text(encoding="utf-8"))
        assert doc["n_slices"] == 7 and doc["cells_union"] == 154
        assert [s["id"] for s in doc["slices"]] == ["S1", "S2", "S3", "S4", "S5",
                                                   "S6", "S7"]

        # detect：图纸口径（map_image 调色板自带雷/障/靶类）走一遍 CLI 全链路
        arena = render_arena(GridSpec(), style="drawing", mines=MINES,
                             obstacles=OBSTACLES, targets=TARGETS,
                             bridges=["K5", "K9"], draw_border=False)
        img = td / "drawing.png"
        Image.fromarray(arena.image).save(img)
        corners = ";".join(f"{t}={arena.corners_px[t][0]:.1f},"
                           f"{arena.corners_px[t][1]:.1f}"
                           for t in ("N11", "N1", "A1", "A11"))
        msgs_path = td / "msgs.jsonl"
        cov_path = td / "cov.json"
        args = parser.parse_args([
            "detect", "--image", str(img), "--corners", corners,
            "--preset", "map_image", "--slices", "s3",
            "--messages", str(msgs_path), "--coverage-report", str(cov_path),
            "--quiet"])
        assert args.func(args) == 0
        rows = [json.loads(x) for x in
                msgs_path.read_text(encoding="utf-8").splitlines()]
        fvs = [m for m in rows if m["type"] == "field_verify"]
        assert len(fvs) == 7
        assert all(m["coverage"]["complete"] is True for m in fvs)
        assert len({m["scan_id"] for m in fvs}) == 1
        assert len({tuple(m["coverage"]["cells"]) for m in fvs}) == 7
        cov = json.loads(cov_path.read_text(encoding="utf-8"))
        assert cov["scan_id"] == fvs[0]["scan_id"]
        assert cov["coverage"]["cells_union"] == 154
        assert cov["message_types"].count("field_verify") == 7

        # 参数守卫：片相关开关不能脱离 --slices 空转
        for argv in (["detect", "--image", str(img), "--corners", corners,
                      "--slice-incomplete", "S1"],
                     ["detect", "--image", str(img), "--corners", corners,
                      "--coverage-report", str(td / "x.json")]):
            args = parser.parse_args(argv)
            try:
                args.func(args)
            except SystemExit as exc:
                assert "需要同时给 --slices" in str(exc), exc
            else:
                raise AssertionError(f"应报错：{argv}")
