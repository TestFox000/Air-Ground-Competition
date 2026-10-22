# -*- coding: utf-8 -*-
"""orin_broadcast_route 守护：测试源必须发**可解析的正确帧**，且不吞空路线。

背景：这个工具是给别人抓包/联调用的，一旦编码错位，对方会把"坏帧"当成
我们协议的问题排查半天。所以锁两件事：
  1. route_plan.json -> (reset, route) 的取法；
  2. 编出来的 0x4003 能被 parse_car_command 原样解回。
"""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

from ag_integrate.intel_protocol import parse_car_command

REPO = Path(__file__).resolve().parent.parent
TOOLS = REPO / "tools"


def _load():
    spec = importlib.util.spec_from_file_location(
        "_bc_route", TOOLS / "orin_broadcast_route.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod          # 模块内有 dataclass，必须先登记
    spec.loader.exec_module(mod)
    return mod


def test_payload_roundtrip_and_load_plan(tmp_path):
    mod = _load()
    plan = {"car_cmds": {"reset": "reset:N10,x", "route": "N10 K10 H9,x"}}
    p = tmp_path / "route_plan.json"
    p.write_text(json.dumps(plan), encoding="utf-8")
    reset, route = mod.load_plan(p)
    assert (reset, route) == ("reset:N10,x", "N10 K10 H9,x")
    frame = parse_car_command(mod.make_payload(route, reset))
    assert frame.route == route and frame.reset == reset


def test_missing_route_is_rejected(tmp_path):
    """空路线必须报错：发一帧"空航点"比不发更危险（对端可能清空任务）。"""
    mod = _load()
    p = tmp_path / "route_plan.json"
    p.write_text(json.dumps({"car_cmds": {"reset": "reset:N10,x", "route": "  "}}),
                 encoding="utf-8")
    try:
        mod.load_plan(p)
    except ValueError as exc:
        assert "car_cmds.route" in str(exc)
    else:
        raise AssertionError("空路线必须被拒绝")
