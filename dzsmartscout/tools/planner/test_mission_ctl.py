#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dzlink_mission_ctl 的单测（不 import rclpy，Windows 本机可跑）。

覆盖：
  1. --dry-run 全链（plan→gate→send 报文），退出码 0，计时 < 15s
  2. run 模式 --from-start：cmd-file 里的 start 指令能被消费并触发一次 pipeline
  3. 确定性：同输入两次 dry-run 的 token 串一致

用法（工作区根）：
  python dzsmartscout\\tools\\planner\\test_mission_ctl.py
"""
from __future__ import annotations

import os
import pathlib
import subprocess
import sys
import tempfile
import time

HERE = pathlib.Path(__file__).resolve().parent
CTL = HERE / "dzlink_mission_ctl.py"
BUDGET_S = 15.0

ENV = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1",
           PYTHONDONTWRITEBYTECODE="1")


def run_cli(args: list[str], timeout: float = 60) -> tuple[int, str, float]:
    t0 = time.perf_counter()
    p = subprocess.run([sys.executable, str(CTL)] + args, capture_output=True,
                       text=True, encoding="utf-8", errors="replace",
                       cwd=str(HERE), env=ENV, timeout=timeout)
    return p.returncode, (p.stdout or "") + (p.stderr or ""), time.perf_counter() - t0


def _tok_line(out: str) -> str:
    import re
    pat = re.compile(r"^[A-N](?:1[01]|[1-9])(?:[*,](?:x|y))?\*?$")
    best = ""
    for line in out.splitlines():
        # 兼容两种形态：dry-run 的纯 token 行，或 run 模式的 `tokens = N10 M10 ...`
        body = line.strip()
        if body.startswith("tokens = "):
            body = body[len("tokens = "):]
        toks = body.split()
        if len(toks) >= 8 and all(pat.match(t) for t in toks):
            best = " ".join(toks)
    return best


def main() -> int:
    fails = 0
    total = 0

    def check(name: str, cond: bool, detail: str = "") -> None:
        nonlocal fails, total
        total += 1
        mark = "PASS" if cond else "FAIL"
        print(f"  [{mark}] {name}" + (f"  {detail}" if detail else ""))
        if not cond:
            fails += 1

    print("== 1. dry-run 全链 ==")
    rc, out, dt = run_cli(["dry-run"])
    toks = _tok_line(out)
    check("退出码 0", rc == 0, f"rc={rc}")
    check("拿到 token 串", bool(toks), f"{len(toks.split())} token")
    check("计时 < 15s", dt < BUDGET_S, f"{dt:.2f}s")
    check("打印达标行", "达标" in out, "")

    print("== 2. 确定性（两次 dry-run token 一致） ==")
    rc2, out2, _ = run_cli(["dry-run"])
    toks2 = _tok_line(out2)
    check("两次 token 一致", toks == toks2 and bool(toks))

    print("== 3. run 模式 --from-start 消费 start 触发 ==")
    fd, tmp = tempfile.mkstemp(suffix=".jsonl")
    os.close(fd)
    pathlib.Path(tmp).write_text(
        '{"type":"heartbeat"}\n{"type":"start","note":"ssh"}\n', encoding="utf-8")
    try:
        proc = subprocess.Popen(
            [sys.executable, str(CTL), "run", "--cmd-file", tmp,
             "--from-start", "--wait-match", "0.2"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, encoding="utf-8", errors="replace", cwd=str(HERE), env=ENV)
        # run 是常驻，给 5s 让它消费 + 触发一次，再 terminate
        time.sleep(5.0)
        proc.terminate()
        try:
            out3, _ = proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            out3, _ = proc.communicate()
        check("run 模式触发了一次 pipeline", "收到 start 指令" in out3, "")
        check("run 模式产出 token", bool(_tok_line(out3)), "")
        check("run 模式打印计时", "指令→下发动作完成" in out3, "")
        check("run 模式本机无 ROS 时如实报下发失败", "下发失败" in out3,
              "（本机无 rclpy，预期 rc=2）")
    finally:
        pathlib.Path(tmp).unlink(missing_ok=True)

    print(f"\n结果：{total - fails}/{total} 通过")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
