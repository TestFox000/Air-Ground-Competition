#!/usr/bin/env python3
"""一键运行入口（对齐官方技术总框架）。

本机：
    python run.py test     # 单元测试（默认）
    python run.py check    # 环境体检
    python run.py log      # 复盘报告：读黑匣子日志，看哪一步卡住、为什么

实车运行（车端香橙派，需 ROS2）：
    ros2 launch ag_integrate integrate.launch.py
    # 默认 dry_run:=true 只算不发；验证后 dry_run:=false，节点消费
    # dzuav 的二进制 UDP:50000 情报，下发官方 path_executor 的 /nav/waypoint。

远程机器操作用 tools/remote_run.py（SSH 上传/执行/取回）。
"""

from __future__ import annotations

import argparse
import importlib
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent

for stream in (sys.stdout, sys.stderr):
    try:
        stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def _has_module(name: str) -> bool:
    try:
        importlib.import_module(name)
        return True
    except Exception:
        return False


def cmd_test(args) -> int:
    cmd = [sys.executable, "-m", "pytest", "tests", "-q"]
    if args.keyword:
        cmd += ["-k", args.keyword]
    # 透传未识别的 pytest 参数（如 -x / --lf / -m），由 parse_known_args 收集
    cmd += getattr(args, "pytest_extra", [])
    return subprocess.call(cmd, cwd=str(ROOT))


def cmd_check(_args) -> int:
    print(f"Python  : {sys.version.split()[0]} ({sys.executable})")
    for name, required, hint in (
        ("numpy", True, "pip install numpy"),
        ("pytest", True, "pip install pytest"),
        ("yaml", False, "pip install PyYAML"),
        ("cv2", False, "pip install opencv-python（仅离线可视化需要）"),
        ("paramiko", False, "pip install paramiko（远程 SSH 需要）"),
    ):
        has = _has_module(name)
        mark = "OK " if has else ("缺失" if required else "可选")
        print(f"{name:<9} : {mark}" + ("" if has else f"   -> {hint}"))

    for tool in ("git", "ros2", "colcon"):
        print(f"{tool:<7} : {shutil.which(tool) or '未安装'}")
    print("\n结论: " + ("环境可用" if _has_module('numpy') and _has_module('pytest') else "缺少必需依赖"))
    return 0


def cmd_log(args) -> int:
    argv = [sys.executable, str(ROOT / "tools" / "log_report.py")]
    if args.dir:
        argv += ["--dir", args.dir]
    if args.out:
        argv += ["--out", args.out]
    if args.json:
        argv.append("--json")
    if args.strict:
        argv.append("--strict")
    return subprocess.call(argv, cwd=str(ROOT))


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="air_ground 一键入口（官方框架）")
    sub = ap.add_subparsers(dest="cmd")

    p_test = sub.add_parser("test", help="单元测试（默认）")
    p_test.add_argument("-k", "--keyword", default="", help="只跑匹配的用例")
    p_test.set_defaults(func=cmd_test)

    sub.add_parser("check", help="环境体检").set_defaults(func=cmd_check)

    p_log = sub.add_parser("log", help="复盘报告")
    p_log.add_argument("--dir", default=None, help="日志目录")
    p_log.add_argument("--out", default=None, help="报告另存为 Markdown")
    p_log.add_argument("--json", action="store_true", help="输出 JSON")
    p_log.add_argument("--strict", action="store_true", help="有严重结论时退出码为 1")
    p_log.set_defaults(func=cmd_log)

    return ap


def main(argv: list[str] | None = None) -> int:
    ap = build_parser()
    args, extra = ap.parse_known_args(argv)
    if not getattr(args, "func", None):
        # 不带子命令 = 跑测试
        args, extra = ap.parse_known_args(["test"] + list(argv or []))
    args.pytest_extra = extra  # 未识别的参数（如 -x/--lf）透传给 pytest
    return int(args.func(args) or 0)


if __name__ == "__main__":
    raise SystemExit(main())
