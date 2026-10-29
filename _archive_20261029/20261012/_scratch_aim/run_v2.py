# -*- coding: utf-8 -*-
"""用原型 aim_v2 跑 test_aim.py（不改动正式源码）。

    python _scratch_aim\\run_v2.py

做法：把 aim_v2.py 以 `dzbrain.tasks.aim` 的名字装进 sys.modules，
后续 `from dzbrain.tasks.aim import ...` 全部命中原型。
"""
from __future__ import annotations

import importlib.util
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "dzbrain"))
sys.path.insert(0, str(ROOT / "src" / "dzbrain" / "tests"))
sys.stdout.reconfigure(errors="replace")

import dzbrain.tasks  # noqa: E402  （先建父包，相对导入才有 parent）

spec = importlib.util.spec_from_file_location(
    "dzbrain.tasks.aim", pathlib.Path(__file__).resolve().parent / "aim_v2.py")
proto = importlib.util.module_from_spec(spec)
sys.modules["dzbrain.tasks.aim"] = proto
spec.loader.exec_module(proto)

import test_aim  # noqa: E402

test_aim.main()
