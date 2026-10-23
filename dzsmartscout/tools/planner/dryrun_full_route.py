#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""dryrun_full_route.py —— 用**规划器产出的整条路线**跑一遍下发端干跑（不发车、不连 ROS）

验收④前半：supervisor / 发送器本地干跑输出整线报文。
链路：planner(prod tree) → token 串 → dzlink_supervisor.py route --dry-run
                                      → dz_send_waypoint.py route --dry-run
证据落 EVIDENCE/dryrun_full_route.txt。

用法: python dryrun_full_route.py
"""
from __future__ import annotations

import os
import pathlib
import re
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
VENDOR = HERE / "vendor" / "dzbrain_ws"
LINK = WORKSPACE / "dzsmartscout" / "tools" / "link"
EVID = HERE / "EVIDENCE"


def planner_tokens(engine: str) -> tuple[str, int]:
    """跑产物树规划器出整线；engine 由调用方从 config 读（`planner.engine`）。"""
    cmd = [sys.executable, str(VENDOR / "tools" / "plan_route.py"), "--emit", "tokens",
           "--engine", engine,
           "--config", str(VENDOR / "config" / "robot.yaml"),
           "--map", str(VENDOR / "config" / "map.yaml")]
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1",
               PYTHONDONTWRITEBYTECODE="1")
    p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                       errors="replace", cwd=str(VENDOR), env=env, timeout=600)
    pat = re.compile(r"^[A-N](?:1[01]|[1-9])(?:[*,](?:x|y))?\*?$")
    out = ""
    for line in (p.stdout or "").splitlines():
        body = line.strip().replace("\x1b", "")
        toks = body.split()
        if len(toks) >= 8 and all(pat.match(t) for t in toks):
            out = body
    return out, p.returncode


def main() -> int:
    EVID.mkdir(parents=True, exist_ok=True)
    import planner_engine as PE
    engine = PE.read_engine(VENDOR)["engine"]
    route, rc = planner_tokens(engine)
    lines = ["=== 整线干跑（不发车、不连 ROS）===",
             "★ 引擎 = planner.engine = %r（与 tasks/mission.py:906 同源口径；"
             "plan_route.py 的 --engine 默认 tour，必须显式传）" % engine,
             "planner 退出码=%d" % rc,
             "路线（%d token）= %s" % (len(route.split()), route), ""]
    print("引擎=%s  路线（%d token）= %s" % (engine, len(route.split()), route))

    cases = [
        ("dzlink_supervisor.py route --dry-run（无人机侧常驻节点的下发路径）",
         [sys.executable, str(LINK / "dzlink_supervisor.py"), "route", route, "--dry-run"]),
        ("dz_send_waypoint.py route --dry-run（一次性发送器）",
         [sys.executable, str(LINK / "dz_send_waypoint.py"), "route", route, "--dry-run"]),
    ]
    ok = True
    for title, cmd in cases:
        env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1",
                   PYTHONDONTWRITEBYTECODE="1")
        p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                           errors="replace", cwd=str(WORKSPACE), env=env, timeout=300)
        lines += ["--- %s ---" % title, "$ " + " ".join(cmd), (p.stdout or "").strip()]
        if (p.stderr or "").strip():
            lines.append("[stderr] " + (p.stderr or "").strip())
        lines.append("EXIT=%d" % p.returncode)
        lines.append("")
        print("\n--- %s ---\n%s\nEXIT=%d" % (title, (p.stdout or "").strip(), p.returncode))
        if p.returncode != 0 or route not in (p.stdout or ""):
            ok = False

    lines.append("结论：%s" % ("两条下发路径都能干跑输出**整线报文**（含完整 %d token，引擎 %s）✅"
                               % (len(route.split()), engine)
                               if ok else "有条路径未输出整线报文 ❌"))
    (EVID / "dryrun_full_route.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n### ok=%s  证据: EVIDENCE/dryrun_full_route.txt" % ok)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
