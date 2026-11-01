#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""run_plan_and_compare.py —— 无人机侧(产物树) vs 小车侧(真源树) 同一输入跑 plan_route，逐 token 对拍

验收②的实测脚本：
  · 两侧都用**仓库内逐字快照地图**（`map_source.resolve()` 缺省 = fixtures/map_field7_t24.yaml）
    + 同版 `config/robot.yaml`
  · 各跑 2 次（判确定性）
  · 逐 token 比较两侧输出
  · 两侧各跑一次 route_check（--check，要求 0 error）
  · 证据（卡 WB-10-EVIDENCE-20261101）：**默认纯只读，不写盘**。加 `--write-evidence` 才落，
    且全部**按参数指纹另名**（`plan_compare_<引擎>_<地图sha前8>.json` 等），不碰任何既有/
    历史证据 —— 原先每次运行都覆写 plan_compare.json / plan_tokens.txt / route_check_*.txt，
    一次交叉实验就把别人的对拍证据顶掉。

★ 地图（卡 WB-9-VENDOR-20261031）：两侧树里**都没有** `config/map.yaml` 了
  （真源按 D-054 裁决4 移除；vendor 副本被 WB-1 移出工作树）⇒ 本脚本改由 `map_source`
  解析到仓库内逐字快照（与账本旧 `config/map.yaml` 条目 sha256 `385e0718…` 逐字节同源）。
  **禁止**把地图拷回 vendor（D-054/D-062）。覆盖：`--map` > `DZ_PLANNER_MAP` > `ROBOT_MAP`。

用法: python run_plan_and_compare.py [--map PATH] [--engine tour|tracker-safe] [--write-evidence]
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import re
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
VENDOR = HERE / "vendor" / "dzbrain_ws"
WORKSPACE = HERE.parents[2]
SOURCE = WORKSPACE / "dzrobot_v2"
EVID = HERE / "EVIDENCE"

if str(HERE) not in sys.path:                 # 同目录 map_source.py
    sys.path.insert(0, str(HERE))
import map_source                             # noqa: E402

#: 路线 token 串里一个 token 的合法形态 —— **对齐真源解析器声明的语法**
#: `vendor/dzbrain_ws/src/dzbrain/dzbrain/route/parser.py:_CANON_TAIL_RE`：
#:   格名 [R:] + `*`/`:SHOOT_STOP` + `:SCAN_STOP`/`:SCAN` + `:PIVOT` + `,x|,y`/`:x|:y`
#: ★ 卡 WB-9 踩到的坑：本脚本原先只认 `(?:[*,](?:x|y))?`，`F4:PIVOT` / `I4:PIVOT` 这种
#:   真源合法 token 会被判成"不是 token 行" ⇒ 整条串抓不到（`0 token`，误报"两侧 DIFFERENT"）。
#:   这不是地图问题，是**消费侧正则比真源语法旧**；只放宽"识别 token 行"，不改校验期望。
TOKEN_RE = re.compile(
    r"^(?:[Rr]:)?[A-N](?:1[01]|[1-9])"
    r"(?:\*|:SHOOT_STOP)?"
    r"(?::SCAN_STOP|:SCAN)?"
    r"(?::PIVOT)?"
    r"(?:[*,](?:x|y)|:(?:X|Y))?$",
    re.IGNORECASE)


def _token_line(stdout: str) -> str:
    """从 plan_route 的输出里挑出**路线 token 串**那一行（--check 会在后面再打校验行，
    所以不能想当然取最后一行）。判据：含空格分隔且形如 `XXnn[,x|y][*][:PIVOT]` 的 token 行。"""
    best = ""
    pat = TOKEN_RE
    for line in stdout.splitlines():
        body = line.strip().replace("\x1b", "")
        if not body or body.startswith(("[", "✅", "⚠", "❌", "🎉", "$")):
            continue
        toks = body.split()
        if len(toks) >= 8 and all(pat.match(t) for t in toks):
            best = body
    return best


def _param_fp(engine: str, minfo) -> str:
    """证据文件名里的**参数指纹**：引擎 + 地图 sha256 前 8 位（与 dryrun 同口径）。"""
    eng = re.sub(r"[^A-Za-z0-9._-]+", "-", engine).strip("-") or "engine"
    return "%s_%s" % (eng, minfo.sha256[:8])


def run(tool_root: pathlib.Path, engine: str, map_path: pathlib.Path,
        extra: list[str] | None = None, timeout=600):
    # ★★ 收口轮（队长实锤）：`plan_route.py --engine` 默认是 `tour`，而实车走 `planner.engine`
    #    ⇒ **必须显式传 --engine**，否则就是"机算 40 / 车跑 44"的错位。
    # ★ 地图（卡 WB-9）：两侧树里都没有 config/map.yaml 了，统一用 map_source 解析到
    #   仓库内逐字快照 —— 两侧**喂同一份字节**，对拍才有意义。
    cmd = [sys.executable, str(tool_root / "tools" / "plan_route.py"),
           "--emit", "tokens",
           "--engine", engine,
           "--config", str(tool_root / "config" / "robot.yaml"),
           "--map", str(map_path)]
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
    ap = argparse.ArgumentParser(description="两侧 plan_route 逐 token 对拍")
    ap.add_argument("--map", default=None,
                    help="覆盖地图 yaml（缺省=map_source 具名缺省 field7：仓库内逐字快照；"
                         "也可用 DZ_PLANNER_MAP / ROBOT_MAP 覆盖）")
    ap.add_argument("--engine", default=None, help="覆盖引擎（缺省=读 config 的 planner.engine）")
    ap.add_argument("--write-evidence", action="store_true",
                    help="显式落证：产物按参数指纹另名（不覆盖既有证据）；缺省=纯只读，不写盘")
    args = ap.parse_args()

    try:
        minfo = map_source.resolve(args.map)
    except map_source.MapSourceError as exc:
        print("❌ 地图解析失败：%s" % exc)
        return 2

    res: dict = {"when": time.strftime("%Y-%m-%d %H:%M:%S"), "cases": {}}

    # ★★ 先读"车侧现在用哪个引擎"（与 mission.py:399 同源，用 vendor 的 Config 读同一个键）
    import planner_engine as PE
    eng = PE.read_engine(VENDOR, source_ws=SOURCE)
    engine = args.engine or eng["engine"]
    res["engine"] = eng
    res["map_source"] = {"path": str(minfo.path), "sha256": minfo.sha256,
                         "size": minfo.size, "origin": minfo.origin, "name": minfo.name,
                         "why": map_source.WHY_FIELD7}
    print("== 引擎口径（读 planner.engine，与 mission.py 同源）==")
    print("   planner.engine = %r  →  engine=%s%s（tracker_max_seconds=%s / max_seconds=%s）"
          % (eng["raw"], engine,
             "（命令行 --engine 覆盖）" if args.engine else "",
             eng["tracker_max_seconds"], eng["max_seconds"]))
    print("== 地图（WB-9：两侧树都不再有 config/map.yaml，改喂仓库内逐字快照）==")
    print("   %s" % minfo.describe())
    if eng.get("source_engine") and eng["source_engine"] != eng["raw"]:
        print("   ⚠️ 真源 config 的 engine=%r 与 vendor 快照 %r 不一致 —— 先重跑同步"
              % (eng["source_engine"], eng["raw"]))
    for w in eng["warnings"]:
        print("   ⚠️ " + w)
    mission_blocks = PE.read_mission_blocks(SOURCE / "config" / "robot.yaml")
    res["mission_blocks"] = mission_blocks
    print("   板载回退路线 mission_blocks = %d token" % len(mission_blocks))

    print("== 无人机侧（产物树，将随 sync 推到机上）==")
    v1 = run(VENDOR, engine, minfo.path)
    v2 = run(VENDOR, engine, minfo.path)
    print("   run1 rc=%d  %s" % (v1["rc"], v1["tokens"][:80]))
    print("   run2 rc=%d  %s" % (v2["rc"], v2["tokens"][:80]))
    res["cases"]["vendor"] = {"run1": v1, "run2": v2,
                              "deterministic": v1["tokens"] == v2["tokens"] and v1["rc"] == 0}

    print("== 小车侧（真源树 dzrobot_v2）==")
    s1 = run(SOURCE, engine, minfo.path)
    s2 = run(SOURCE, engine, minfo.path)
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
        c = run(root, engine, minfo.path, ["--check"])
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

    if not args.write_evidence:
        # ★ 卡 WB-10：默认**只读**，不写任何 EVIDENCE 产物（也不建目录）
        print("\n### engine=%s  all_pass=%s  mission_blocks_same=%s"
              "（只读模式：未写任何文件；要落证加 --write-evidence）"
              % (engine, res["all_pass"], mb_same))
        return 0 if (res["all_pass"] and mb_same) else 1

    EVID.mkdir(parents=True, exist_ok=True)
    _fp = _param_fp(engine, minfo)

    def _ev(stem: str, ext: str) -> pathlib.Path:
        """按参数指纹另名的证据路径（**绝不**指向既有/历史证据）。"""
        return EVID / ("%s_%s.%s" % (stem, _fp, ext))

    _ev("plan_compare", "json").write_text(json.dumps(res, ensure_ascii=False, indent=2),
                                            encoding="utf-8")
    led = json.loads((HERE / "HASH_LEDGER.json").read_text(encoding="utf-8")) \
        if (HERE / "HASH_LEDGER.json").is_file() else {"files": []}
    h = {e["rel"]: e["sha256"][:20] for e in led.get("files", [])}
    _ev("plan_tokens", "txt").write_text(
        "地图 = %s（%s%s，%d B，sha256 %s）\n"
        "   ★ WB-9：两侧树都不再有 config/map.yaml（D-054 裁决4 + WB-1 移出 vendor 副本）⇒\n"
        "     改喂仓库内逐字快照；这份快照与账本旧 config/map.yaml 条目逐字节同源。\n"
        "   ★ 选图理由：%s\n"
        "输入约束 config/robot.yaml（sha256 %s，账本生成时间 %s）\n"
        "★ 引擎 = %r（与 tasks/mission.py:906 plan_route_from_config 同源口径）\n"
        "   —— `tools/plan_route.py` 的 --engine 默认是 tour，**必须显式传**，否则会跑出另一条路线\n\n"
        "[无人机侧=产物树 vendor/dzbrain_ws]\n%s\n\n"
        "[小车侧=真源树 dzrobot_v2]\n%s\n\n"
        "逐 token 相同：%s（%d 个 token）\n"
        "与板载回退路线 mission_blocks（%d token）逐 token 相同：%s\n"
        % (minfo.path.name, minfo.origin,
           ("，具名 %r" % minfo.name) if minfo.name else "", minfo.size, minfo.sha256[:20],
           map_source.WHY_FIELD7,
           h.get("config/robot.yaml", "?"), led.get("generated_at", "?"),
           engine, tv, ts, res["same_tokens"], res["token_count"], len(mb), mb_same),
        encoding="utf-8")
    # route_check 全文留档（验收要说"0 error"，要把告警也写清楚）
    for name in ("vendor", "source"):
        c = run(VENDOR if name == "vendor" else SOURCE, engine, minfo.path, ["--check"])
        _ev("route_check_%s" % name, "txt").write_text(
            "$ %s\n\n%s\n%s\nrc=%d\n" % (c["cmd"], c["stdout"], c["stderr"], c["rc"]),
            encoding="utf-8")
    print("\n### engine=%s  all_pass=%s  mission_blocks_same=%s" % (engine, res["all_pass"], mb_same))
    print("### 证据（参数指纹 %s）: EVIDENCE/plan_compare_%s.json / plan_tokens_%s.txt / "
          "route_check_{vendor,source}_%s.txt" % (_fp, _fp, _fp, _fp))
    print("    ★ 既有/历史证据未被改动（本脚本只写带指纹的新名字）")
    return 0 if (res["all_pass"] and mb_same) else 1


if __name__ == "__main__":
    sys.exit(main())
