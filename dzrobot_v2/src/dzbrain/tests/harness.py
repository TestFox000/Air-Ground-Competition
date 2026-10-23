# -*- coding: utf-8 -*-
"""极简测试工具（本机无 pytest，项目惯例是纯脚本测试）。

用法：
    from harness import check, section, finish
    check("N10 中心 = (0.525, 0.175)", geometry.block_to_meters("N10") == (0.525, 0.175))
    ...
    finish()   # 打印汇总并以 0/1 退出
"""

from __future__ import annotations

import math
import sys
import traceback

# ★ Windows GBK 控制台兜底：测试打印里含 ✅/⚠️/❌ 等字符时，stdout 是 GBK 编码，
#   直接 print 会抛 UnicodeEncodeError 且**把整个测试脚本炸掉**（不是"字打不出来"）。
#   errors="replace" 让不可编码字符退化为 "?"，测试照常跑完。
try:
    sys.stdout.reconfigure(errors="replace")
    sys.stderr.reconfigure(errors="replace")
except Exception:
    pass

_PASS = 0
_FAIL: list[str] = []
_CURRENT = ""


def section(title: str) -> None:
    global _CURRENT
    _CURRENT = title
    print(f"\n=== {title} ===")


def check(name: str, cond: bool, detail: str = "") -> bool:
    global _PASS
    if cond:
        _PASS += 1
        print(f"  PASS  {name}" + (f"   [{detail}]" if detail else ""))
    else:
        _FAIL.append(f"{_CURRENT} :: {name}" + (f"   [{detail}]" if detail else ""))
        print(f"  FAIL  {name}" + (f"   [{detail}]" if detail else ""))
    return bool(cond)


def close(a: float, b: float, tol: float = 1e-9) -> bool:
    return abs(a - b) <= tol


def check_close(name: str, a: float, b: float, tol: float = 1e-9) -> bool:
    return check(name, close(a, b, tol), f"got {a!r}, want {b!r} (tol {tol:g})")


def check_raises(name: str, exc_type: type, fn, *args, **kwargs) -> bool:
    try:
        fn(*args, **kwargs)
    except exc_type as exc:
        return check(name, True, f"抛出 {type(exc).__name__}: {exc}")
    except Exception as exc:  # 抛了别的类型
        return check(name, False, f"期望 {exc_type.__name__}，实得 {type(exc).__name__}: {exc}")
    return check(name, False, f"期望抛 {exc_type.__name__}，但没有抛")


def finish() -> None:
    print("\n" + "=" * 70)
    total = _PASS + len(_FAIL)
    if _FAIL:
        print(f"结果：{_PASS}/{total} 通过，{len(_FAIL)} 失败")
        for item in _FAIL:
            print(f"  FAIL  {item}")
        print("=" * 70)
        sys.exit(1)
    print(f"结果：{_PASS}/{total} 全部通过 ✅")
    print("=" * 70)
    sys.exit(0)


def run_case(name: str, fn) -> None:
    """把"会抛异常的子用例"包起来，异常记为失败而不是中断整个脚本。"""
    try:
        fn()
    except Exception:
        check(f"{name}（未抛异常）", False, traceback.format_exc(limit=2).strip().splitlines()[-1])


__all__ = ["check", "check_close", "check_raises", "close", "finish", "section", "run_case",
           "math"]
