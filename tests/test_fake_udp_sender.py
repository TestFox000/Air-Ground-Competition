"""fake_udp_sender.parse_scenario 测试（D13 迁移守护）。

背景：默认场景旧值 "B3:mine" 在 D13 枚举（ObjectType 已删 MINE/ARROW）下
直接 SystemExit"未知类别"——默认一跑就崩。这里锁死：
  - 默认场景可解析（paper 普通格落格判=雷 + 蓝靶，满足门控）；
  - 13 类枚举名全部可用；
  - 已删除的旧名 mine/arrow 与乱名必须报"未知类别"。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

spec = importlib.util.spec_from_file_location(
    "fake_udp_sender", REPO / "tools" / "fake_udp_sender.py")
fus = importlib.util.module_from_spec(spec)
sys.modules["fake_udp_sender"] = fus
spec.loader.exec_module(fus)

from ag_integrate.intel_protocol import ObjectType  # noqa: E402


def test_default_scene_parses():
    objs = fus.parse_scenario(fus.DEFAULT_SCENARIO)
    assert len(objs) == 2
    assert objs[0].type is ObjectType.PAPER         # B3 普通格 paper = 雷
    assert objs[1].type is ObjectType.TARGET_BLUE   # H7 蓝靶


def test_all_d13_names_accepted():
    """13 类枚举名（小写）全部可解析，且类型一一对应。"""
    for t in ObjectType:
        (obj,) = fus.parse_scenario(f"C5:{t.name.lower()}")
        assert obj.type is t


def test_retired_names_rejected():
    """旧 7cls 名 mine/arrow 已删除，必须报"未知类别"而不是静默崩别的。"""
    for legacy in ("mine", "arrow"):
        with pytest.raises(SystemExit) as exc:
            fus.parse_scenario(f"B3:{legacy}")
        assert "未知类别" in str(exc.value)


def test_garbage_name_rejected():
    with pytest.raises(SystemExit) as exc:
        fus.parse_scenario("B3:not_a_class")
    assert "未知类别" in str(exc.value)


def test_bad_format_rejected():
    with pytest.raises(SystemExit):
        fus.parse_scenario("B3")            # 缺类别
    with pytest.raises(SystemExit):
        fus.parse_scenario("33:paper")      # 格号非法
