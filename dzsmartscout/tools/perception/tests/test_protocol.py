# -*- coding: utf-8 -*-
r"""协议单测：字段必须与车侧 ``drone.py`` 完全一致，且**真源可核对**。

这里做三件事：
1. 逐字段断言 4 种报文的形状（键名/大小写/类型）；
2. 用 ``map.yaml`` / ``robot.yaml`` 的**原文**核对格名与超时参数（不靠记忆）；
3. 用 ``mirror_car_side_parse`` 复述车侧判定，并镜像它的差集逻辑，证明
   "照 map.yaml 发 ⇒ 一致，改一格 ⇒ 报差异"。
"""

from __future__ import annotations

import json
import pathlib
import re

from ..grid import GridSpec
from ..protocol import (BRIDGE_HALVES, CAR_TO_DRONE_TOPIC, DRONE_TO_CAR_TOPIC,
                        PAYLOAD_TYPES, QOS_DRONE_LINK, build_bridge_passable,
                        build_field_verify, build_heartbeat, build_target_zone,
                        encode, mirror_car_side_parse, recommended_cadence,
                        validate_payload)

# 本文件在 <v1>/dzsmartscout/tools/perception/tests/ ⇒ parents[4] = <v1>
REPO = pathlib.Path(__file__).resolve().parents[4]
MAP_YAML = REPO / "dzrobot_v2" / "config" / "map.yaml"
ROBOT_YAML = REPO / "dzrobot_v2" / "config" / "robot.yaml"


def _yaml_list(text: str, key: str) -> list[str]:
    """只解析 ``key: [ "A1", ... ]`` 这种一行式列表（够用，且不引第三方依赖）。"""
    m = re.search(rf"^\s*{key}:\s*\[(.*?)\]", text, re.M)
    if not m:
        raise AssertionError(f"map.yaml 里找不到 {key}: [...]")
    return [s.strip().strip('"').strip("'") for s in m.group(1).split(",") if s.strip()]


def _yaml_scalar(text: str, key: str) -> str | None:
    m = re.search(rf"^\s*{key}:\s*([^#\n]+)", text, re.M)
    return m.group(1).strip() if m else None


def test_map_yaml_is_the_source_of_truth_for_cells():
    """拿仓库里 config/map.yaml 的原文核对 —— 不是我记忆里的格名。

    ★ t59（2026-10-15）：``map.yaml`` 已换成**第 5 张新地图 · 2026-10-14 官方贴图重掷版**
    （用户当面贴图 + 队长/感知线双独立识读逐格一致）。本测试随之更新为**现役真值**，
    并保留原设计：全部格名必须落在 14×11 网格内、桥的两个通格必须**一上一下**。
    """
    assert MAP_YAML.is_file(), f"找不到 {MAP_YAML}"
    text = MAP_YAML.read_text(encoding="utf-8")
    assert _yaml_list(text, "mines") == ["C1", "A3", "E3", "G4", "I6", "B7", "D9", "H10"], \
        "现役 map.yaml 的雷点位变了 —— 请与感知线/队长重新对账后再改本断言"
    assert _yaml_list(text, "obstacles") == ["A1", "E2", "H4", "D8", "I8", "G9", "B11", "F11"]
    assert _yaml_list(text, "targets") == ["E5", "F5"]
    assert _yaml_list(text, "bridges") == ["K5", "K9"]
    assert _yaml_scalar(text, "start").strip('"') == "N10"
    assert _yaml_scalar(text, "end").strip('"') == "N5"
    # 结论：桥的两个通格必须一上一下（K5 upper / K9 lower），与半场划分自洽
    spec = GridSpec()
    assert spec.half_of_block("K5") == "upper"
    assert spec.half_of_block("K9") == "lower"
    for block in (_yaml_list(text, "mines") + _yaml_list(text, "obstacles")
                  + _yaml_list(text, "targets") + _yaml_list(text, "bridges")):
        spec.block_to_index(block)          # 全部格名必须在网格内（不抛异常即通过）


def test_robot_yaml_timeouts_match_protocol_assumptions():
    """心跳 3 s / 情报 30 s 是车侧配置，不是我编的常量。"""
    assert ROBOT_YAML.is_file(), f"找不到 {ROBOT_YAML}"
    text = ROBOT_YAML.read_text(encoding="utf-8")
    assert _yaml_scalar(text, "heartbeat_timeout_s") == "3.0"
    assert _yaml_scalar(text, "info_stale_s") == "30.0"
    assert _yaml_scalar(text, "enabled") == "true"
    cad = recommended_cadence()
    assert cad["topic"] == "/dzlink/drone_to_car"
    assert cad["qos"] == {"depth": 10, "reliability": "best_effort"}


def test_topic_qos_constants():
    assert DRONE_TO_CAR_TOPIC == "/dzlink/drone_to_car"
    assert CAR_TO_DRONE_TOPIC == "/dzlink/car_to_drone"
    assert QOS_DRONE_LINK["reliability"] == "best_effort"
    assert QOS_DRONE_LINK["depth"] == 10
    assert PAYLOAD_TYPES == ("heartbeat", "bridge_passable", "target_zone", "field_verify")
    assert BRIDGE_HALVES == ("lower", "upper")


def test_builders_produce_exact_shapes():
    hb = build_heartbeat(ts=1000.0)
    assert hb == {"type": "heartbeat", "ts": 1000.0}
    bp = build_bridge_passable("LOWER", "k10", 0.87, ts=1000.0)
    assert bp == {"type": "bridge_passable", "half": "lower", "block": "K10",
                  "confidence": 0.87, "ts": 1000.0}
    tz = build_target_zone(["b11", "C11"], ts=1000.0)
    assert tz == {"type": "target_zone", "blocks": ["B11", "C11"], "ts": 1000.0}
    fv = build_field_verify(["b1", "g2"], ["d1"], ts=1000.0)
    assert fv == {"type": "field_verify", "mines": ["B1", "G2"], "obstacles": ["D1"],
                  "ts": 1000.0}
    # seq 只在给了的时候出现（车侧靠它决定回不回 ack）
    assert "seq" not in build_heartbeat()
    assert build_heartbeat(seq=7)["seq"] == 7
    for payload in (hb, bp, tz, fv):
        assert validate_payload(payload, GridSpec()) == []


def test_builders_reject_invalid_input():
    for fn in (lambda: build_bridge_passable("middle", "K10"),
               lambda: build_bridge_passable("", "K10"),
               lambda: build_bridge_passable("lower", ""),
               lambda: build_target_zone([]),
               lambda: build_field_verify()):
        try:
            fn()
        except ValueError:
            continue
        raise AssertionError("非法输入应抛 ValueError")


def test_encode_style_matches_car_side_separators():
    s = encode({"type": "heartbeat", "ts": 1.0})
    assert s == '{"type":"heartbeat","ts":1.0}'
    assert " " not in s and json.loads(s)["type"] == "heartbeat"
    # 非 ASCII 不转义（与 drone.py 的 ensure_ascii=False 同）
    assert "桥" in encode({"type": "heartbeat", "note": "桥洞"})


def test_validate_payload_catches_sender_side_mistakes():
    spec = GridSpec()
    assert validate_payload(build_bridge_passable("lower", "K10", 0.5), spec) == []
    assert validate_payload({"type": "bridge_passable", "half": "mid", "block": "K10"}, spec)
    assert validate_payload({"type": "bridge_passable", "half": "lower", "block": "Z9"}, spec)
    assert validate_payload({"type": "bridge_passable", "half": "lower", "block": "A0"}, spec)
    assert validate_payload({"type": "target_zone", "blocks": []}, spec)
    assert validate_payload({"type": "field_verify", "ts": 1.0}, spec)
    assert validate_payload({"type": "bridge_passable", "half": "lower", "block": "K10",
                             "confidence": 1.5}, spec)
    assert validate_payload({"type": "hello"}, spec)
    assert validate_payload("not a dict", spec)
    # 严格模式才查格名；照车侧口径（drone.py:280）不查格名也算合法
    assert validate_payload({"type": "bridge_passable", "half": "lower",
                             "block": "Z9"}, None) == []


def test_mirror_car_side_parse_all_types():
    r = mirror_car_side_parse(encode(build_heartbeat()))
    assert r["raw_ok"] and "ONLINE" in r["action"]
    r = mirror_car_side_parse(encode(build_bridge_passable("lower", "K10", 0.9)))
    assert "P_DRONE" in r["action"] and "收到时刻" in " ".join(r["notes"])
    r = mirror_car_side_parse(encode(build_target_zone(["B11", "C11"])))
    assert "差集" in r["action"]
    r = mirror_car_side_parse(encode(build_field_verify(["B1"], ["D1"])))
    assert "mines" in r["action"] and "obstacles" in r["action"]
    # 坏报文：非字符串 / 非法 JSON / 非对象
    assert mirror_car_side_parse(123)["raw_ok"] is False
    assert mirror_car_side_parse("{oops")["raw_ok"] is False
    assert mirror_car_side_parse("[1,2]")["raw_ok"] is False
    # 缺字段 / 未知类型
    assert "missing_fields" in mirror_car_side_parse(
        encode({"type": "bridge_passable", "half": "middle", "block": "K10"}))["action"]
    assert "missing_fields" in mirror_car_side_parse(
        encode({"type": "target_zone", "blocks": []}))["action"]
    assert "未知类型" in mirror_car_side_parse(
        encode({"type": "who_knows"}))["action"]
    # seq ⇒ 车侧回 ack
    r = mirror_car_side_parse(encode(build_bridge_passable("upper", "K5", 0.5, seq=3)))
    assert any("seq=3" in n for n in r["notes"])


def _mirror_diff(got: list[str], local: list[str]) -> str:
    """镜像 drone.py:329-344 `_diff`：一致 = 不报差异。"""
    g, l = set(got), set(local)
    if g == l:
        return ""
    return f"仅无人机{sorted(g - l)} 仅本地{sorted(l - g)}"


def test_map_consistent_payload_yields_no_diff_and_one_change_does():
    """照 map.yaml 发 ⇒ 车侧零差异；改一格 ⇒ 立刻报出来（核对功能是活的）。"""
    text = MAP_YAML.read_text(encoding="utf-8")
    mines = _yaml_list(text, "mines")
    obstacles = _yaml_list(text, "obstacles")
    targets = _yaml_list(text, "targets")
    payload = build_field_verify(mines, obstacles)
    assert _mirror_diff(payload["mines"], mines) == ""
    assert _mirror_diff(payload["obstacles"], obstacles) == ""
    assert _mirror_diff(build_target_zone(targets)["blocks"], targets) == ""
    # 漏一格 / 多一格都要能报出来
    assert "仅本地" in _mirror_diff(mines[:-1], mines)
    assert "仅无人机" in _mirror_diff(mines + ["J7"], mines)
    # 且这些报文本身是合法的（不会被 missing_fields 吃掉）
    for p in (payload, build_target_zone(targets)):
        assert validate_payload(p, GridSpec()) == []
        assert mirror_car_side_parse(encode(p))["raw_ok"]


def test_bridge_passable_for_both_halves_of_current_map():
    """当前地图两个通格：K5(upper) / K10(lower) —— 报文与半场必须自洽。"""
    spec = GridSpec()
    for half, block in (("upper", "K5"), ("lower", "K10")):
        assert spec.half_of_block(block) == half
        msg = build_bridge_passable(half, block, 0.75, seq=1)
        assert validate_payload(msg, spec) == []
        parsed = mirror_car_side_parse(encode(msg))
        assert "P_DRONE" in parsed["action"]
        # block 大小写任意（车侧会 .upper()）
        assert validate_payload({"type": "bridge_passable", "half": half.upper(),
                                 "block": block.lower()}, spec) == []
