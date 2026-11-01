#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""dryrun_full_route.py —— 用**规划器产出的整条路线**跑一遍下发端干跑（不发车、不连 ROS）

验收④前半：supervisor / 发送器本地干跑输出整线报文。
链路：planner(prod tree) → token 串 → dzlink_supervisor.py route --dry-run
                                      → dz_send_waypoint.py route --dry-run
★ 证据（卡 WB-10-EVIDENCE-20261101）：**默认纯只读，不写任何文件**。
  要落证据必须显式加 `--write-evidence`，产物**按参数指纹另名**：
  `EVIDENCE/dryrun_full_route_<引擎>_<地图sha前8>.txt`。
  **本脚本永不写 `EVIDENCE/dryrun_full_route.txt`** —— 那是权威证据；
  一次交叉实验就能把它悄悄换成别人的结果（WB-9 的交付证据本轮被覆盖两次，
  `git status` 只显示「改了一个文件」，看不出是谁、用什么参数改的）。
  更新权威证据是**人工**动作，不能是「顺手跑一下」的副作用。

★ 地图（卡 WB-9-VENDOR-20261031）：`vendor/dzbrain_ws/config/map.yaml` 已被 WB-1 按
  D-054 裁决4 / D-062 移出工作树 ⇒ 本脚本**不再**指向它，改由 `map_source.resolve()`
  解析（缺省 = 仓库内逐字快照 `dzrobot_v2/src/dzbrain/tests/fixtures/map_field7_t24.yaml`，
  与 HASH_LEDGER.json 记的 config/map.yaml 逐字节同源）。**绝不**把地图拷回 vendor。
  覆盖方式：`--map <path>` > 环境变量 `DZ_PLANNER_MAP` > `ROBOT_MAP` > 具名缺省。

用法: python dryrun_full_route.py [--map PATH] [--engine tour|tracker-safe] [--write-evidence]

机侧脚本日志（卡 WB-10 第②条）：调用 `dzlink_supervisor.py` 时**一律显式 `--no-log`**。
  原先它靠 `default_log_path()`，Windows 分支取 `os.getcwd()`，而本脚本把 cwd 设成
  工作区根 ⇒ 每跑一次干跑就在**工作区根**掉一个未跟踪 `dzlink_supervisor.jsonl`
  （captain 本轮已手工清过两次，机制没修下次还会掉）。
  `--no-log` 是**显式指令**（不落日志），不是 cwd 兜底；dry-run 的报文已完整写进
  证据正文，JSONL 里只会重复记一遍 dry 的 route_queued，没有额外信息。
"""
from __future__ import annotations

import argparse
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

if str(HERE) not in sys.path:                 # 同目录 map_source.py
    sys.path.insert(0, str(HERE))
import map_source                             # noqa: E402

#: 路线 token 串里一个 token 的合法形态 —— **对齐真源解析器声明的语法**
#: `vendor/dzbrain_ws/src/dzbrain/dzbrain/route/parser.py:_CANON_TAIL_RE`：
#:   格名 [R:] + `*`/`:SHOOT_STOP` + `:SCAN_STOP`/`:SCAN` + `:PIVOT` + `,x|,y`/`:x|:y`
#: ★ 卡 WB-9 踩到的坑：本脚本原先只认 `(?:[*,](?:x|y))?`，`F4:PIVOT` 这种合法 token
#:   会被判成"不是 token 行" ⇒ 整条串抓不到（`0 token`）。这不是地图问题，是**消费侧**
#:   正则比真源语法旧。这里只放宽"识别 token 行"的能力，**不改任何规划/校验期望**。
TOKEN_RE = re.compile(
    r"^(?:[Rr]:)?[A-N](?:1[01]|[1-9])"
    r"(?:\*|:SHOOT_STOP)?"
    r"(?::SCAN_STOP|:SCAN)?"
    r"(?::PIVOT)?"
    r"(?:[*,](?P<hx>x|y)|:(?P<hcx>X|Y))?$",
    re.IGNORECASE)


def _param_fp(engine: str, minfo) -> str:
    """证据文件名里的**参数指纹**：引擎 + 地图 sha256 前 8 位。

    为什么带指纹：同参数两次跑必然落同一个名字（可复跑、可逐位比 sha256）；
    换参数就自动换名字 ⇒ **不会**出现「我跑一次实验，把你那次的结果顶掉」。
    """
    eng = re.sub(r"[^A-Za-z0-9._-]+", "-", engine).strip("-") or "engine"
    return "%s_%s" % (eng, minfo.sha256[:8])


def planner_tokens(engine: str, map_path: pathlib.Path) -> tuple[str, int]:
    """跑产物树规划器出整线；engine 由调用方从 config 读（`planner.engine`），
    map 由 `map_source` 解析（缺省=仓库内逐字快照，不再指向 vendor 下被移出的副本）。"""
    cmd = [sys.executable, str(VENDOR / "tools" / "plan_route.py"), "--emit", "tokens",
           "--engine", engine,
           "--config", str(VENDOR / "config" / "robot.yaml"),
           "--map", str(map_path)]
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1",
               PYTHONDONTWRITEBYTECODE="1")
    p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                       errors="replace", cwd=str(VENDOR), env=env, timeout=600)
    pat = TOKEN_RE
    out = ""
    for line in (p.stdout or "").splitlines():
        body = line.strip().replace("\x1b", "")
        toks = body.split()
        if len(toks) >= 8 and all(pat.match(t) for t in toks):
            out = body
    return out, p.returncode


def main() -> int:
    ap = argparse.ArgumentParser(description="整线下发干跑（不发车、不连 ROS）")
    ap.add_argument("--map", default=None,
                    help="覆盖地图 yaml（缺省=map_source 具名缺省 field7：仓库内逐字快照）")
    ap.add_argument("--engine", default=None,
                    help="覆盖引擎（缺省=读 config 的 planner.engine，与 mission.py 同源）")
    ap.add_argument("--write-evidence", action="store_true",
                    help="显式落证：写 EVIDENCE/dryrun_full_route_<引擎>_<地图sha前8>.txt；缺省（不带本开关）= 纯只读，不写任何文件")
    args = ap.parse_args()
    try:
        minfo = map_source.resolve(args.map)
    except map_source.MapSourceError as exc:
        print("❌ 地图解析失败：%s" % exc)
        return 2

    import planner_engine as PE
    engine = args.engine or PE.read_engine(VENDOR)["engine"]
    route, rc = planner_tokens(engine, minfo.path)
    lines = ["=== 整线干跑（不发车、不连 ROS）===",
             "★ 引擎 = %r（%s；与 tasks/mission.py:906 同源口径；"
             "plan_route.py 的 --engine 默认 tour，必须显式传）"
             % (engine, "命令行 --engine" if args.engine else "planner.engine"),
             "★ 地图 = %s" % minfo.describe(),
             "planner 退出码=%d" % rc,
             "路线（%d token）= %s" % (len(route.split()), route), ""]
    print("引擎=%s  地图=%s  路线（%d token）= %s"
          % (engine, minfo.path.name, len(route.split()), route))

    cases = [
        # ★ 卡 WB-10 第②条：--no-log 显式关掉 JSONL 落盘，不让它靠 cwd 兜底
        #   （原先 cwd=工作区根 + default_log_path() 取 os.getcwd() ⇒ 根目录掉 jsonl）
        ("dzlink_supervisor.py route --dry-run（无人机侧常驻节点的下发路径）",
         [sys.executable, str(LINK / "dzlink_supervisor.py"), "route", route, "--dry-run",
          "--no-log"]),
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
    if not args.write_evidence:
        # ★ 默认纯只读：一个文件都不写（不写 EVIDENCE/，也不写工作区根）
        print("\n### ok=%s  （只读模式：未写任何文件；要落证加 --write-evidence）" % ok)
        return 0 if ok else 1

    EVID.mkdir(parents=True, exist_ok=True)
    fp = _param_fp(engine, minfo)
    evid = EVID / ("dryrun_full_route_%s.txt" % fp)
    evid.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n### ok=%s  证据: %s（参数指纹 %s）"
          % (ok, evid.relative_to(WORKSPACE).as_posix(), fp))
    print("     ★ 权威证据 EVIDENCE/dryrun_full_route.txt **未被改动**（本脚本永不写它）")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
