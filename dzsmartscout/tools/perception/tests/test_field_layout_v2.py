# -*- coding: utf-8 -*-
r"""field_layout v2 契约测试（AGC3 卡 / B2 线，2026-10-24）。

三段式：
1. **构造器单测**（纯本地，不碰车端代码）：形状/校验/拒造；
2. **真·车端契约**：构造器输出 → encode → json.loads → 喂给 dzrobot_v2 的
   normalize_field_layout **真函数**（不是镜像、不是 mock）——车端收不下 = 红；
3. **漂移守卫**：两侧 FIELD_LAYOUT_VERSION 相等、车端 DRONE_PAYLOAD_TYPES
   含 field_layout、两侧网格几何常量一致——协议任一侧漂移 = 红。
"""

from __future__ import annotations

import json
import pathlib
import sys

from ..grid import GridSpec
from ..protocol import (FIELD_LAYOUT_VERSION, PAYLOAD_TYPES, build_field_layout,
                        encode, mirror_car_side_parse, validate_payload)

# 本文件在 <v1>/dzsmartscout/tools/perception/tests/ ⇒ parents[4] = <v1>
REPO = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "dzrobot_v2" / "src" / "dzbrain"))

from dzbrain.core import geometry as car_geo                       # noqa: E402
from dzbrain.tasks.drone import (FIELD_LAYOUT_VERSION as CAR_VER,  # noqa: E402
                                 DRONE_PAYLOAD_TYPES as CAR_TYPES,
                                 normalize_field_layout)

SPEC = GridSpec()
#: 一片合理的已观测区（N/M/L/K 列 5~10 行），够放 start/end/target
KNOWN = ["N10", "N9", "N8", "N7", "N6", "N5",
         "M10", "M9", "M8", "M7", "M6", "M5",
         "L10", "L9", "L8", "L7", "L6", "L5",
         "K10", "K9", "K8", "K7", "K6", "K5"]


def _build(**over):
    kw = dict(version=3, start="N10", end="N5", known=KNOWN,
              bridges=["K9"], spec=SPEC,
              mines=["M8"], obstacles=["L7"], targets=["N6"],
              target_cell="N6", ts=1700000000.0, seq=42)
    kw.update(over)
    return build_field_layout(**kw)


def _car_accept(msg):
    """构造器输出走一遍 encode→json.loads（模拟线上 JSON 往返）再喂真函数。"""
    return normalize_field_layout(json.loads(encode(msg)))


# --------------------------------------------------------------------------- #
# 1. 构造器
# --------------------------------------------------------------------------- #
def test_build_minimal_shape():
    msg = _build()
    assert msg["type"] == "field_layout"
    assert msg["version"] == 3
    assert msg["start"] == "N10" and msg["end"] == "N5"
    assert msg["known"] == KNOWN
    assert msg["mines"] == ["M8"] and msg["obstacles"] == ["L7"]
    assert msg["targets"] == ["N6"]
    assert msg["bridges"] == ["K9"]
    assert msg["target_cell"] == "N6"
    assert msg["segment_id"] == 3          # 缺省 = version（恒显式发送）
    assert msg["ts"] == 1700000000.0 and msg["seq"] == 42
    assert "barrier_col_from" not in msg   # 三键全不给 ⇒ 键都不出现
    assert "barrier_row_hi" not in msg and "barrier_row_lo" not in msg


def test_build_barrier_trio():
    msg = _build(barrier_col_from="d", barrier_row_hi=5, barrier_row_lo=8)
    assert msg["barrier_col_from"] == "D"   # 大写归一
    assert msg["barrier_row_hi"] == 5 and msg["barrier_row_lo"] == 8


def test_build_rejects():
    # known 空
    try:
        _build(known=[]); assert False, "known 空应抛"
    except ValueError as e:
        assert "known" in str(e)
    # 要素 ∉ known
    try:
        _build(mines=["A1"]); assert False, "A1 ∉ known 应抛"
    except ValueError as e:
        assert "未观测格" in str(e)
    # bridges 空 / 非法格
    try:
        _build(bridges=[]); assert False, "bridges 空应抛"
    except ValueError:
        pass
    try:
        _build(bridges=["Z99"]); assert False, "非法格名应抛"
    except ValueError:
        pass
    # bridge 不在 known 是**合法**的（照图，车侧不受限）
    msg = _build(bridges=["K9", "K4"])      # K4 ∉ KNOWN
    assert msg["bridges"] == ["K9", "K4"]
    # target_cell ∉ known / 空
    try:
        _build(target_cell="A1"); assert False, "target ∉ known 应抛"
    except ValueError as e:
        assert "known" in str(e)
    assert _build(target_cell="")["target_cell"] == ""
    # version 非法
    for bad in (True, -1, "3", 3.0):
        try:
            _build(version=bad); assert False, f"version={bad!r} 应抛"
        except ValueError:
            pass
    # barrier 半套 / 区间非法
    try:
        _build(barrier_row_hi=5); assert False, "半套 barrier 应抛"
    except ValueError as e:
        assert "三键" in str(e)
    try:
        _build(barrier_col_from="D", barrier_row_hi=7, barrier_row_lo=6)
        assert False, "hi>=lo 应抛"
    except ValueError:
        pass
    # segment_id 非法
    try:
        _build(segment_id=-1); assert False
    except ValueError:
        pass


# --------------------------------------------------------------------------- #
# 2. 真·车端契约（normalize_field_layout 是车端真函数，不是镜像）
# --------------------------------------------------------------------------- #
def test_car_accepts_builder_output():
    layout, reason = _car_accept(_build())
    assert layout is not None, f"车端拒收：{reason}"
    assert reason == ""
    assert layout["version"] == 3
    assert layout["start"] == "N10" and layout["end"] == "N5"
    assert layout["known"] == KNOWN            # v2 键原样保留
    assert layout["mines"] == ["M8"]
    assert layout["bridges"] == ["K9"]
    assert layout["target_cell"] == "N6"
    assert layout["segment_id"] == 3
    assert layout["barrier_col_from"] == car_geo.BARRIER_COL_FROM   # 缺省回落
    assert layout["barrier_row_hi"] == car_geo.BARRIER_ROW_HI
    assert layout["barrier_row_lo"] == car_geo.BARRIER_ROW_LO


def test_car_accepts_full_barrier_override():
    layout, reason = _car_accept(_build(barrier_col_from="C",
                                        barrier_row_hi=5, barrier_row_lo=8))
    assert layout is not None, f"车端拒收：{reason}"
    assert layout["barrier_col_from"] == "C"
    assert layout["barrier_row_hi"] == 5 and layout["barrier_row_lo"] == 8


def test_car_rejects_feature_outside_known():
    """构造器拦得住的错，车端也必须拒（双保险语义一致）。"""
    payload = {"type": "field_layout", "version": 1, "start": "N10", "end": "N5",
               "known": KNOWN, "mines": ["A1"], "obstacles": [], "targets": [],
               "bridges": ["K9"]}
    layout, reason = normalize_field_layout(payload)
    assert layout is None
    assert "未观测格" in reason


def test_v1_v2_discrimination_preserved():
    """v1 报文（无 known）车端照收且**不补键** ⇒ 不会误开步进循环；
    本构造器恒带 known ⇒ 走 v2。两条路径都要在。"""
    v1 = {"type": "field_layout", "version": 1, "start": "N10", "end": "N5",
          "mines": [], "obstacles": [], "targets": [], "bridges": ["K9"]}
    layout, reason = normalize_field_layout(v1)
    assert layout is not None, reason
    assert "known" not in layout             # v1 语义：不补键
    layout2, _ = _car_accept(_build())
    assert "known" in layout2                # v2：有键


def test_validate_and_mirror_clean():
    msg = _build()
    assert validate_payload(msg, SPEC) == []
    res = mirror_car_side_parse(encode(msg))
    assert res["raw_ok"] and res["type"] == "field_layout"
    assert "P_FIELD_LAYOUT" in res["action"]


# --------------------------------------------------------------------------- #
# 3. 漂移守卫（两侧任一边改了协议，这里红）
# --------------------------------------------------------------------------- #
def test_version_parity():
    assert FIELD_LAYOUT_VERSION == CAR_VER == 2


def test_type_registered_both_sides():
    assert "field_layout" in PAYLOAD_TYPES
    assert "field_layout" in CAR_TYPES


def test_grid_parity():
    """发送端 GridSpec 与车端 geometry 的几何常量必须一致（地图漂移守卫）。"""
    assert SPEC.cols == car_geo.MAP_COLS
    assert SPEC.rows == car_geo.MAP_ROWS
    assert SPEC.col_letters == car_geo.COL_LETTERS
    assert SPEC.barrier_row_hi == car_geo.BARRIER_ROW_HI
    assert SPEC.barrier_row_lo == car_geo.BARRIER_ROW_LO
