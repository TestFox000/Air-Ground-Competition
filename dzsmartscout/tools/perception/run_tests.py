# -*- coding: utf-8 -*-
r"""离线测试驱动（本机没有 pytest）。

用法（任选其一，都在 ``dzsmartscout/tools`` 目录下执行）：

    python -m perception.run_tests                 # 全部
    python -m perception.run_tests grid colors      # 只跑名字含这些关键字的模块
    python perception/run_tests.py                  # 直接跑脚本也行（自带引导）

退出码 0 = 全过，1 = 有失败。每个 ``test_*`` 都是普通断言函数，pytest 也能直接收集。
"""

from __future__ import annotations

import importlib
import sys
import time
import traceback

if __package__ in (None, ""):            # 允许 `python perception/run_tests.py`
    import pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    __package__ = "perception"

MODULES = [
    "perception.tests.test_colors",
    "perception.tests.test_grid",
    "perception.tests.test_objects",
    "perception.tests.test_car_pose",
    "perception.tests.test_bridge_arrow",
    "perception.tests.test_protocol",
    "perception.tests.test_coverage",
    "perception.tests.test_field_map",
    "perception.tests.test_pipeline_e2e",
    "perception.tests.test_field_layout_v2",
]


def run(modules=None, verbose: bool = True) -> tuple[int, int, list[str]]:
    """返回 ``(通过数, 失败数, 失败详情)``。"""
    names = list(modules or MODULES)
    passed = failed = 0
    failures: list[str] = []
    for name in names:
        try:
            mod = importlib.import_module(name)
        except Exception:                                  # noqa: BLE001
            failed += 1
            failures.append(f"[import] {name}\n{traceback.format_exc()}")
            continue
        tests = sorted((n, getattr(mod, n)) for n in dir(mod) if n.startswith("test_"))
        if verbose:
            print(f"\n=== {name} ({len(tests)} 项) ===")
        for tname, fn in tests:
            if not callable(fn):
                continue
            t0 = time.perf_counter()
            try:
                fn()
            except Exception:                              # noqa: BLE001
                failed += 1
                failures.append(f"[FAIL] {name}.{tname}\n{traceback.format_exc()}")
                if verbose:
                    print(f"  FAIL  {tname}")
            else:
                passed += 1
                dt = (time.perf_counter() - t0) * 1000.0
                if verbose:
                    print(f"  ok    {tname}  ({dt:.0f} ms)")
    return passed, failed, failures


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    modules = MODULES
    if argv:
        picked = [m for m in MODULES if any(a.lower() in m.lower() for a in argv)]
        if not picked:
            print(f"没有匹配的测试模块：{argv}")
            return 2
        modules = picked
    passed, failed, failures = run(modules)
    print(f"\n{'=' * 60}\n通过 {passed} / 失败 {failed}")
    for f in failures:
        print("\n" + f)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
