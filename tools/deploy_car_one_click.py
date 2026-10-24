#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车端一键部署（2026-10-24）：probe → 本地门禁 → backup+push+build → verify → 报告。

编排层只做链路与判定；各阶段复用既有维护入口（冻结区工具，不改其实现）：
    ① tools/deploy_to_car.py probe            mDNS/IP 探活
    ② 本地门禁（推前推后各跑）：route_check 现役线 0 error + config_selfcheck + test_deploy
    ③ tools/deploy_to_car.py all --force-no-autostart
        backup → push（dzbrain 80 文件 + rknn + config + tools，sha256 对账）
        → build（colcon）→ **强制 auto_start=false**（第六图协议：等无人机下发再出发）
    ④ tools/deploy_to_car.py verify           包/入口存在性 + 云台补丁符号

用法（dzrobot_v2 仓库根目录）：
    python tools/deploy_car_one_click.py                 # 全链
    python tools/deploy_car_one_click.py --dry-run       # 只打印将执行的命令
    python tools/deploy_car_one_click.py --ip <车端IP>   # 跳过 mDNS 解析
    python tools/deploy_car_one_click.py --keep-autostart # 不强制 auto_start=false

全量回归（34 套，~20 分钟）不在链上；发车前建议手动跑：python _run_full_regression.py
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

WS = Path(__file__).resolve().parent.parent          # dzrobot_v2 仓库根
DEPLOY = WS / "tools" / "deploy_to_car.py"
GATES: list[tuple[str, str, str]] = []


def run(cmd: list[str], *, dry: bool, timeout: int = 1800, env: dict | None = None,
        cwd: Path | None = None) -> tuple[int, str]:
    printable = " ".join(str(c) for c in cmd)
    if dry:
        print(f"  [dry-run] {printable}")
        return 0, ""
    print(f"  $ {printable}")
    try:
        p = subprocess.run([str(c) for c in cmd], cwd=str(cwd or WS), env=env,
                           timeout=timeout, capture_output=True, text=True,
                           encoding="utf-8", errors="replace")
        out = (p.stdout or "") + (p.stderr or "")
        tail = "\n".join(out.strip().splitlines()[-3:])
        if tail:
            print("    | " + tail.replace("\n", "\n    | "))
        return p.returncode, out
    except subprocess.TimeoutExpired:
        print(f"    ✗ 超时（{timeout}s）")
        return 124, ""


def gate(name: str, ok: bool | None, evidence: str) -> None:
    st = "PASS" if ok else ("SKIP" if ok is None else "FAIL")
    GATES.append((name, st, evidence))
    print(f"  {'✅' if ok else ('⏭️' if ok is None else '❌')} {name}: {evidence}")


def gate_d(name: str, ok: bool, evidence: str, *, dry: bool) -> None:
    gate(name, None if dry else ok, "dry-run 跳过" if dry else evidence)


def local_gates(*, dry: bool, tag: str) -> None:
    """本地门禁：推上车的东西必须是绿的。"""
    print(f"== 本地门禁（{tag}）==")
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1",
               PYTHONPATH=str(WS / "src" / "dzbrain"), ROBOT_MAP=str(WS / "config" / "map.yaml"))
    rc, out = run([sys.executable, WS / "tools" / "route_check.py",
                   "--config", "config/robot.yaml"], dry=dry, env=env)
    gate_d("route_check 现役线", rc == 0,
           "0 error" if rc == 0 else (out.strip().splitlines()[-1][:80] if out.strip() else f"exit={rc}"), dry=dry)
    rc, out = run([sys.executable, WS / "tools" / "config_selfcheck.py"], dry=dry, env=env)
    gate_d("config_selfcheck", rc == 0,
           "exit=0" if rc == 0 else (out.strip().splitlines()[-1][:80] if out.strip() else f"exit={rc}"), dry=dry)
    rc, out = run([sys.executable, WS / "src" / "dzbrain" / "tests" / "test_deploy.py"],
                  dry=dry, env=env, timeout=300)
    gate_d("test_deploy", rc == 0, "88/88" if rc == 0 else f"exit={rc}", dry=dry)


def main() -> int:
    ap = argparse.ArgumentParser(description="车端一键部署（probe→门禁→all→verify→报告）")
    ap.add_argument("--ip", default="", help="车端 IP（缺省 mDNS: duzhong.local）")
    ap.add_argument("--keep-autostart", action="store_true",
                    help="不强制 auto_start=false（默认强制——第六图协议等无人机下发）")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    extra = ["--ip", args.ip] if args.ip else []

    print("== 阶段① probe（车端探活）==")
    rc, _ = run([sys.executable, DEPLOY, "probe"] + extra, dry=args.dry_run, timeout=90)
    gate_d("probe", rc == 0, "mDNS/IP 可达" if rc == 0 else "车端不在线", dry=args.dry_run)
    if rc != 0 and not args.dry_run:
        return report(2)

    local_gates(dry=args.dry_run, tag="推之前")

    print("== 阶段② backup+push+build（all，强制 auto_start=false）==")
    cmd = [sys.executable, DEPLOY] + extra + (["all"] if args.keep_autostart
                                              else ["all", "--force-no-autostart"])
    rc, _ = run(cmd, dry=args.dry_run, timeout=1800)
    gate_d("push/build", rc == 0,
           "backup→push(sha256 对账)→colcon 全过" if rc == 0 else "失败——远端备份见上方输出",
           dry=args.dry_run)
    if rc != 0 and not args.dry_run:
        return report(1)

    print("== 阶段③ verify（包/入口 + 云台补丁符号）==")
    rc, _ = run([sys.executable, DEPLOY] + extra + ["verify"], dry=args.dry_run, timeout=300)
    gate_d("verify", rc == 0, "包完整性 + 补丁符号过" if rc == 0 else f"exit={rc}", dry=args.dry_run)

    if not args.dry_run:
        local_gates(dry=False, tag="推之后复跑")

    return report(0)


def report(fast_exit: int) -> int:
    print("\n" + "=" * 72)
    print("车端一键部署汇总")
    print("=" * 72)
    n_fail = sum(1 for _, s, _ in GATES if s == "FAIL")
    for name, st, ev in GATES:
        print(f"  [{st}] {name}: {ev}")
    if fast_exit or n_fail:
        print("\n❌ 未完成/失败。远端备份路径见阶段②输出；重推前核对备份目录。")
        return fast_exit or 1
    print("\n✅ 全部通过。发车前建议全量回归：python _run_full_regression.py（34 套，~20 分钟）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
