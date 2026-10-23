#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""run_plan_and_compare.py —— 无人机侧(产物树) vs 小车侧(真源树) 同一输入跑 plan_route，逐 token 对拍

验收②的实测脚本：
  · 两侧都用**官方第五张图** `config/map.yaml` + 同版 `config/robot.yaml`
  · 各跑 2 次（判确定性）
  · 逐 token 比较两侧输出
  · 两侧各跑一次 route_check（--check，要求 0 error）
  · 证据落 EVIDENCE/plan_compare.json + EVIDENCE/plan_tokens.txt

用法: python run_plan_and_compare.py
"""
from __future__ import annotations

import json
import os
import pathlib
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
VENDOR = HERE / "vendor" / "dzbrain_ws"
WORKSPACE = HERE.parents[2]
SOURCE = WORKSPACE / "dzrobot_v2"
EVID = HERE / "EVIDENCE"


def _token_line(stdout: str) -> str:
    """从 plan_route 的输出里挑出**路线 token 串**那一行（--check 会在后面再打校验行，
    所以不能想当然取最后一行）。判据：含空格分隔且形如 `XXnn[,x|y][*]` 的 token 行。"""
    import re
    best = ""
    pat = re.compile(r"^[A-N](?:1[01]|[1-9])(?:[*,](?:x|y))?\*?$")
    for line in stdout.splitlines():
        body = line.strip().replace("\x1b", "")
        if not body or body.startswith(("[", "✅", "⚠", "❌", "🎉", "$")):
            continue
        toks = body.split()
        if len(toks) >= 8 and all(pat.match(t) for t in toks):
            best = body
    return best


def run(tool_root: pathlib.Path, engine: str, extra: list[str] | None = None, timeout=600):
    # ★★ 收口轮（队长实锤）：`plan_route.py --engine` 默认是 `tour`，而实车走 `planner.engine`
    #    （第五张图 = tracker-safe）⇒ **必须显式传 --engine**，否则就是"机算 40 / 车跑 44"的错位。
    cmd = [sys.executable, str(tool_root / "tools" / "plan_route.py"),
           "--emit", "tokens",
           "--engine", engine,
           "--config", str(tool_root / "config" / "robot.yaml"),
           "--map", str(tool_root / "config" / "map.yaml")]
    cmd += extra or []
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1",
               # ★ 真源树 dzrobot_v2 是**只读**边界：禁止解释器往里写 __pycache__/*.pyc
               PYTHONDONTWRITEBYTECODE="1")
    t0 = time.time()
    p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                       errors="replace", cwd=str(tool_root), env=env, timeout=timeout)
    return {"cmd": " ".join(cmd), "root": str(tool_root), "rc": p.returncode,
            "tokens": _token_line(p.stdout or "") if p.returncode == 0 else "",
            "stdout": p.stdout or "", "stderr": p.stderr or "",
            "secs": round(time.time() - t0, 2)}


def main() -> int:
    EVID.mkdir(parents=True, exist_ok=True)
    res: dict = {"when": time.strftime("%Y-%m-%d %H:%M:%S"), "cases": {}}

    # ★★ 先读"车侧现在用哪个引擎"（与 mission.py:399 同源，用 vendor 的 Config 读同一个键）
    import planner_engine as PE
    eng = PE.read_engine(VENDOR, source_ws=SOURCE)
    engine = eng["engine"]
    res["engine"] = eng
    print("== 引擎口径（读 planner.engine，与 mission.py 同源）==")
    print("   planner.engine = %r  →  engine=%s（tracker_max_seconds=%s / max_seconds=%s）"
          % (eng["raw"], engine, eng["tracker_max_seconds"], eng["max_seconds"]))
    if eng.get("source_engine") and eng["source_engine"] != eng["raw"]:
        print("   ⚠️ 真源 config 的 engine=%r 与 vendor 快照 %r 不一致 —— 先重跑同步"
              % (eng["source_engine"], eng["raw"]))
    for w in eng["warnings"]:
        print("   ⚠️ " + w)
    mission_blocks = PE.read_mission_blocks(SOURCE / "config" / "robot.yaml")
    res["mission_blocks"] = mission_blocks
    print("   板载回退路线 mission_blocks = %d token" % len(mission_blocks))

    print("== 无人机侧（产物树，将随 sync 推到机上）==")
    v1 = run(VENDOR, engine)
    v2 = run(VENDOR, engine)
    print("   run1 rc=%d  %s" % (v1["rc"], v1["tokens"][:80]))
    print("   run2 rc=%d  %s" % (v2["rc"], v2["tokens"][:80]))
    res["cases"]["vendor"] = {"run1": v1, "run2": v2,
                              "deterministic": v1["tokens"] == v2["tokens"] and v1["rc"] == 0}

    print("== 小车侧（真源树 dzrobot_v2）==")
    s1 = run(SOURCE, engine)
    s2 = run(SOURCE, engine)
    print("   run1 rc=%d  %s" % (s1["rc"], s1["tokens"][:80]))
    print("   run2 rc=%d  %s" % (s2["rc"], s2["tokens"][:80]))
    res["cases"]["source"] = {"run1": s1, "run2": s2,
                              "deterministic": s1["tokens"] == s2["tokens"] and s1["rc"] == 0}

    tv, ts = v1["tokens"], s1["tokens"]
    res["same_tokens"] = (tv == ts and bool(tv))
    res["token_count"] = len(tv.split())
    print("\n== 两侧逐 token 对拍 ==")
    print("   产物树 tokens(%d) vs 真源树 tokens(%d) → %s"
          % (len(tv.split()), len(ts.split()), "IDENTICAL ✅" if res["same_tokens"] else "DIFFERENT ❌"))
    if not res["same_tokens"]:
        a, b = tv.split(), ts.split()
        for i, (x, y) in enumerate(zip(a, b)):
            if x != y:
                print("   首个差异 @%d: %s vs %s" % (i, x, y)); break
        print("   长度差：%d vs %d" % (len(a), len(b)))

    print("\n== route_check（0 error 才算过）==")
    for name, root in (("vendor", VENDOR), ("source", SOURCE)):
        c = run(root, engine, ["--check"])
        err = c["rc"] == 0 and tv == c["tokens"]
        res["cases"][name]["check"] = {"rc": c["rc"], "tokens_same_as_plain": tv == c["tokens"],
                                       "tail": "\n".join(c["stdout"].strip().splitlines()[-6:])}
        print("   %-7s --check rc=%d tokens一致=%s" % (name, c["rc"], tv == c["tokens"]))
        for line in c["stdout"].strip().splitlines()[-4:]:
            print("      " + line.replace("\x1b", "")[:150])

    res["all_pass"] = (res["cases"]["vendor"]["deterministic"] and res["cases"]["source"]["deterministic"]
                       and res["same_tokens"]
                       and res["cases"]["vendor"]["check"]["rc"] == 0
                       and res["cases"]["source"]["check"]["rc"] == 0)

    # ★ 与"板载回退路线 mission_blocks"对拍（t57 落盘口径：tracker-safe 产物应逐位一致）
    mb = mission_blocks
    mb_same = bool(mb) and (" ".join(mb) == tv)
    res["mission_blocks_same"] = mb_same
    print("\n== 与板载回退路线 mission_blocks 对拍（%d token）==" % len(mb))
    if mb_same:
        print("   逐 token 相同 ✅（车侧自规划产物 == 落盘的回退路线）")
    elif mb:
        a = tv.split()
        print("   ⚠️ 不同：机算 %d token / mission_blocks %d token" % (len(a), len(mb)))
        for i, (x, y) in enumerate(zip(a, mb)):
            if x != y:
                print("      首个差异 @%d: 机算 %s vs mission_blocks %s" % (i, x, y)); break

    (EVID / "plan_compare.json").write_text(json.dumps(res, ensure_ascii=False, indent=2),
                                            encoding="utf-8")
    led = json.loads((HERE / "HASH_LEDGER.json").read_text(encoding="utf-8")) \
        if (HERE / "HASH_LEDGER.json").is_file() else {"files": []}
    h = {e["rel"]: e["sha256"][:20] for e in led.get("files", [])}
    (EVID / "plan_tokens.txt").write_text(
        "官方第五张图 config/map.yaml（同步自 dzrobot_v2，sha256 %s）\n"
        "输入约束 config/robot.yaml（sha256 %s，账本生成时间 %s）\n"
        "★ 引擎 = planner.engine = %r（与 tasks/mission.py:906 plan_route_from_config 同源口径）\n"
        "   —— `tools/plan_route.py` 的 --engine 默认是 tour，**必须显式传**，否则会跑出另一条路线\n\n"
        "[无人机侧=产物树 vendor/dzbrain_ws]\n%s\n\n"
        "[小车侧=真源树 dzrobot_v2]\n%s\n\n"
        "逐 token 相同：%s（%d 个 token）\n"
        "与板载回退路线 mission_blocks（%d token）逐 token 相同：%s\n"
        % (h.get("config/map.yaml", "?"), h.get("config/robot.yaml", "?"), led.get("generated_at", "?"),
           engine, tv, ts, res["same_tokens"], res["token_count"], len(mb), mb_same),
        encoding="utf-8")
    # route_check 全文留档（验收要说"0 error"，要把告警也写清楚）
    for name in ("vendor", "source"):
        c = run(VENDOR if name == "vendor" else SOURCE, engine, ["--check"])
        (EVID / ("route_check_%s.txt" % name)).write_text(
            "$ %s\n\n%s\n%s\nrc=%d\n" % (c["cmd"], c["stdout"], c["stderr"], c["rc"]),
            encoding="utf-8")
    print("\n### engine=%s  all_pass=%s  mission_blocks_same=%s" % (engine, res["all_pass"], mb_same))
    print("### 证据: EVIDENCE/plan_compare.json / plan_tokens.txt")
    return 0 if (res["all_pass"] and mb_same) else 1


if __name__ == "__main__":
    sys.exit(main())
