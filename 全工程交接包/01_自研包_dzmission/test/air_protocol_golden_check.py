#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""air_protocol_golden_check.py —— 比对 C++ 自检输出与 golden 期望值

`test/air_protocol_selftest.cpp` 的文件头注释写着：
    "输出格式：每行 'KEY <hex>'，由 test/air_protocol_cpp_check.py 读入并与
     Python 参考实现（air_protocol.py）的输出逐字节比对。"
但那个脚本一直不存在。本脚本就是它。

它做两件事：
  1. 读入 C++ 自检的实际输出（文件或 stdin，或 --run 直接编译并执行）
  2. 与 test/air_protocol_golden.txt（由 air_protocol_golden_gen.py 生成）比对

三种用法
--------
  # A. 直接跑（本机有 g++ 时最省事）
  python3 test/air_protocol_golden_check.py --run

  # B. 已经编译好了，喂输出进来
  /tmp/air_selftest | python3 test/air_protocol_golden_check.py

  # C. 输出存成了文件
  python3 test/air_protocol_golden_check.py --actual /tmp/out.txt

退出码：0 = 全部一致；1 = 有差异；2 = 用法/环境问题。
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
GOLDEN = os.path.join(HERE, "air_protocol_golden.txt")


def parse_vectors(text: str) -> dict:
    """从输出里提取 KEY <hex> 行；忽略 [ OK ]/[FAIL]/# 等其它行"""
    out = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("[") or line.startswith("="):
            continue
        parts = line.split()
        if len(parts) != 2:
            continue
        key, val = parts
        # 只接受纯十六进制
        try:
            bytes.fromhex(val)
        except ValueError:
            continue
        out[key] = val.lower()
    return out


def compile_and_run(verbose: bool) -> tuple:
    """编译 test/air_protocol_selftest.cpp 并运行，返回 (text, error)"""
    cxx = None
    for cand in ("g++", "clang++", "c++"):
        p = shutil.which(cand)
        if p:
            cxx = p
            break
    if cxx is None:
        return None, "找不到 C++ 编译器（g++ / clang++ / c++）"

    exe = os.path.join("/tmp", "air_selftest") if os.name != "nt" else \
        os.path.join(os.environ.get("TEMP", "."), "air_selftest.exe")
    cmd = [cxx, "-std=c++17", "-O1", "-Wall", "-Wextra",
           "-I", os.path.join(ROOT, "include"),
           os.path.join(ROOT, "src", "air_protocol.cpp"),
           os.path.join(HERE, "air_protocol_selftest.cpp"),
           "-o", exe]
    if verbose:
        print("编译: " + " ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        return None, "编译失败:\n" + r.stderr

    r2 = subprocess.run([exe], capture_output=True, text=True)
    if verbose:
        sys.stderr.write(r2.stderr)
    return r2.stdout, None


def main() -> int:
    ap = argparse.ArgumentParser(description="比对 C++ 自检输出与 golden 期望值")
    ap.add_argument("--run", action="store_true",
                    help="现场编译并运行 air_protocol_selftest.cpp")
    ap.add_argument("--actual", help="从文件读入 C++ 自检输出（默认读 stdin）")
    ap.add_argument("--golden", default=GOLDEN, help="golden 文件路径")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    # ---- 读 golden ----
    if not os.path.exists(args.golden):
        print("找不到 golden 文件: %s" % args.golden)
        print("请先运行: python3 test/air_protocol_golden_gen.py")
        return 2
    with open(args.golden, encoding="utf-8") as f:
        golden = parse_vectors(f.read())
    print("golden 向量: %d 条  (%s)" % (len(golden), args.golden))

    # ---- 取实际输出 ----
    if args.run:
        text, err = compile_and_run(args.verbose)
        if err:
            print(err)
            return 2
        actual_text = text
        print("已编译并运行 air_protocol_selftest.cpp")
    elif args.actual:
        with open(args.actual, encoding="utf-8") as f:
            actual_text = f.read()
        print("实际输出: %s" % args.actual)
    else:
        if sys.stdin.isatty():
            print("请通过 --run 现场编译，或用管道/--actual 提供 C++ 自检输出。")
            print("例如:  /tmp/air_selftest | python3 %s" % os.path.basename(__file__))
            return 2
        actual_text = sys.stdin.read()
        print("实际输出: stdin")

    actual = parse_vectors(actual_text)

    # ---- 比对 ----
    print()
    print("=" * 74)
    print("逐字节比对")
    print("=" * 74)

    fails = []
    ok_count = 0

    # golden 里每条都必须在实际输出中出现且相同
    for key in sorted(golden):
        want = golden[key]
        if key not in actual:
            print("  [MISS] %-26s C++ 自检没有输出这个键" % key)
            fails.append(key)
            continue
        got = actual[key]
        if got == want:
            ok_count += 1
            if args.verbose:
                print("  [ OK ] %-26s %s" % (key, got))
        else:
            print("  [FAIL] %-26s" % key)
            print("          期望 %s" % want)
            print("          实际 %s" % got)
            # 指出第一处不同，便于定位
            n = min(len(want), len(got))
            for i in range(0, n, 2):
                if want[i:i + 2] != got[i:i + 2]:
                    print("          首个差异: 第 %d 字节 期望 %s 实际 %s"
                          % (i // 2, want[i:i + 2], got[i:i + 2]))
                    break
            else:
                print("          长度不同: 期望 %d 字节 实际 %d 字节"
                      % (len(want) // 2, len(got) // 2))
            fails.append(key)

    # 实际输出里多出来的键（提示，不算失败）
    extra = sorted(set(actual) - set(golden))

    print()
    print("=" * 74)
    if extra:
        print("提示: C++ 自检多输出了 %d 个 golden 里没有的键：" % len(extra))
        for k in extra:
            print("        %s" % k)
        print("      （不算失败，但建议同步到 air_protocol_golden_gen.py）")
        print()

    if fails:
        print("✗ %d 项不一致（一致 %d 项）：" % (len(fails), ok_count))
        for k in fails:
            print("      - %s" % k)
        print()
        print("注意: 若只是 golden 过期，请重跑")
        print("      python3 test/air_protocol_golden_gen.py")
        return 1

    print("✓ 全部 %d 条向量与参考实现逐字节一致" % ok_count)
    return 0


if __name__ == "__main__":
    sys.exit(main())
