#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""dzlink_mission_ctl.py —— 无人机侧「空地指挥链」编排节点（本卡新增）

一句话：把「收到开始指令 → 规划 → route_check 门禁 → 下发 /nav/waypoint → 打印全程耗时」
串成**一条命令**，让「地面站按下开始 → 小车 15s 内动」这条链路有唯一的编排入口和计时证据。

架构位置（无人机 = 主脑）：
```
地面站(QGC/SSH) ──触发──► 本节点 ──(0.75s 规划+门禁)──► 规划器 vendor/dzbrain_ws
                                                        │  token 串
                                                        ▼
                              dz_send_waypoint.py route ──► /nav/waypoint (RELIABLE) ──► 小车
```

三条命令形态（都走同一套 plan→gate→send 管线，只是"触发源"和"是否真的发 DDS"不同）：
  · run    --cmd '{"type":"start"}'            # 从 cmd-file 收开始指令（调试期 SSH 触发）
  · once                                        # 立即开始一次（本地/机上直接跑）
  · dry-run                                     # 规划+门禁，只打印报文不连 ROS（可 Windows 本地跑）

与既有组件的关系（**不重复造轮子，只编排**）：
  · 规划：子进程调 vendor/dzbrain_ws/tools/plan_route.py --emit json --engine <config> --check
    —— 引擎**从 config 读**（planner.engine），不写死（t60 收口轮教训）。
  · 下发：子进程调 dz_send_waypoint.py route "<tokens>"（RELIABLE x3 已是既有契约）。
  · 门禁：plan_route.py 的 --check 内部就是 route_check.py 0 error；本节点只看退出码。

计时口径（15s 判据）：`t0 = 收到 start 指令时刻`，`t1 = 路线下发完成时刻`，
`t_car_move` 由车侧 car_state.phase 变化兜底（见 run 的 --listen 说明），本节点打印
`指令→规划→门禁→下发` 各段耗时，车侧发车时刻以车端日志为准（本节点不假装知道车何时动）。

单源码纪律：本文件只读 vendor 树；真源仍是 dzrobot_v2。禁止手工改 vendor/**。
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent                 # 本节点所在目录

# ---- 路径自适应：本机(Windows, dzsmartscout) 与 无人机(~/dzlink_tools + ~/dzplanner) 两种布局 ----
# 优先级：环境变量显式指定 > 本机 dzsmartscout 布局 > 无人机 ~/ 布局
_ENV_VENDOR = os.environ.get("DZ_PLANNER_WS")                   # 可选：规划器 vendor 树根
_ENV_LINK = os.environ.get("DZ_LINK_DIR")                       # 可选：dz_send_waypoint 所在目录

if _ENV_VENDOR:
    VENDOR = pathlib.Path(_ENV_VENDOR).expanduser()
else:
    _cand_vendor = HERE / "vendor" / "dzbrain_ws"               # 本机 dzsmartscout/tools/planner
    _cand_vendor2 = pathlib.Path("~/dzplanner/dzbrain_ws").expanduser()   # 无人机
    VENDOR = _cand_vendor if (_cand_vendor / "tools" / "plan_route.py").is_file() else _cand_vendor2

if _ENV_LINK:
    LINK_DIR = pathlib.Path(_ENV_LINK).expanduser()
else:
    _cand_link = HERE.parent / "link"                           # 本机 dzsmartscout/tools/link
    _cand_link2 = pathlib.Path("~/dzlink_tools").expanduser()   # 无人机
    LINK_DIR = _cand_link if (_cand_link / "dz_send_waypoint.py").is_file() else _cand_link2

PLAN_ROUTE = VENDOR / "tools" / "plan_route.py"
SEND_WAYPOINT = LINK_DIR / "dz_send_waypoint.py"

BUDGET_S = 15.0                                                # 15s 判据（指令→下发完成）

_UTF8 = {"PYTHONIOENCODING": "utf-8", "PYTHONUTF8": "1",
         "PYTHONDONTWRITEBYTECODE": "1"}


def _read_engine() -> tuple[str, dict]:
    """用 vendor 树同款 Config 读 planner.engine（与 mission.py 同源口径，不写死）。"""
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))          # 确保 planner_engine.py 可被 import（本机/无人机同目录）
    import planner_engine as PE
    info = PE.read_engine(VENDOR)
    engine = info.get("engine") or "tracker-safe"
    return engine, info


def _plan(engine: str, *, map_path: str | None = None, cross_lower: str | None = None,
          cross_upper: str | None = None) -> tuple[int, dict, float]:
    """跑规划器 --emit json --check，返回 (rc, json_payload, elapsed_s)。"""
    cfg = VENDOR / "config" / "robot.yaml"
    mpath = map_path or (VENDOR / "config" / "map.yaml")
    cmd = [sys.executable, str(PLAN_ROUTE), "--emit", "json", "--engine", engine,
           "--config", str(cfg), "--map", str(mpath), "--check"]
    if cross_lower:
        cmd += ["--cross-lower", str(cross_lower).upper()]
    if cross_upper:
        cmd += ["--cross-upper", str(cross_upper).upper()]
    env = dict(os.environ, **_UTF8)
    t0 = time.perf_counter()
    p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                       errors="replace", cwd=str(VENDOR), env=env, timeout=600)
    dt = time.perf_counter() - t0
    payload: dict = {}
    if p.returncode == 0:
        s = (p.stdout or "").strip()
        try:
            # --emit json 输出就是一段纯 JSON（含嵌套），直接整段解析
            payload = json.loads(s)
        except Exception:
            # 兜底：若夹带了非 JSON 行，取首个 { 到最后一个 } 之间的子串再试
            try:
                payload = json.loads(s[s.find("{"):s.rfind("}") + 1])
            except Exception:
                payload = {"_raw": s[-2000:]}
    return p.returncode, payload, dt


def _tokens_from_payload(payload: dict) -> list[str]:
    """从 --emit json 的 payload 里抽出第一条 plan 的 tokens。"""
    for plan in (payload.get("plans") or []):
        toks = plan.get("tokens")
        if toks:
            return [str(t) for t in toks]
    return []


def _send_route(tokens: list[str], *, dry_run: bool, wait_match_s: float) -> tuple[int, str, float]:
    """调 dz_send_waypoint.py route 下发；dry_run 时只打印报文。"""
    route_text = " ".join(tokens)
    cmd = [sys.executable, str(SEND_WAYPOINT), "route", route_text,
           "--wait-match", str(wait_match_s)]
    if dry_run:
        cmd.append("--dry-run")
    env = dict(os.environ, **_UTF8)
    t0 = time.perf_counter()
    p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                       errors="replace", cwd=str(LINK_DIR), env=env, timeout=120)
    dt = time.perf_counter() - t0
    return p.returncode, (p.stdout or "") + (p.stderr or ""), dt


def _run_pipeline(*, map_path: str | None, cross_lower: str | None, cross_upper: str | None,
                  dry_run: bool, wait_match_s: float, t0: float) -> int:
    """plan → gate → send 主链，逐段计时，返回 0=成功。"""
    def _p(msg: str) -> None:
        print(msg, flush=True)   # ★ run 模式常驻且 stdout 常被重定向，逐行 flush 才能实时看到

    engine, einfo = _read_engine()
    _p(f"[mission_ctl] 引擎 = {engine!r}（读 planner.engine，与 mission.py 同源）")
    for k in ("tracker_max_seconds", "max_seconds", "allow_reverse", "fallback_to_tokens"):
        _p(f"  planner.{k} = {einfo.get(k)}")

    t_plan0 = time.perf_counter()
    rc, payload, dt_plan = _plan(engine, map_path=map_path,
                                 cross_lower=cross_lower, cross_upper=cross_upper)
    if rc != 0:
        _p(f"❌ 规划+门禁失败（rc={rc}），详情：")
        _p("  " + json.dumps(payload, ensure_ascii=False)[:2000])
        return 1
    tokens = _tokens_from_payload(payload)
    if not tokens:
        _p("❌ 规划成功但没拿到 token 列表")
        return 1
    _p(f"✅ 规划+route_check 门禁通过（rc=0，耗时 {dt_plan*1000:.0f} ms，"
       f"{len(tokens)} token）")
    _p("  tokens = " + " ".join(tokens))

    rc_send, out, dt_send = _send_route(tokens, dry_run=dry_run, wait_match_s=wait_match_s)
    _p(out.strip())
    send_ok = rc_send in (0, 3)
    if rc_send not in (0, 3):
        _p(f"❌ 下发失败（rc={rc_send}）—— 常见：本机无 ROS 环境（先 source /opt/ros/humble/setup.bash）")
    elif rc_send == 3:
        _p("⚠️ 下发完成但**没有任何车侧订阅者**（车没起栈 / 域号不一致 / 组播被拦）")

    # ★ 计时**无论下发成败都打印**：15s 判据针对的是「规划+门禁+下发动作」这段，
    #   下发失败（如无 ROS）是环境问题，不是计时超时；分开呈现，不混为一谈。
    total = time.perf_counter() - t0
    _p("=" * 60)
    _p(f"[计时] 指令→下发动作完成 = {total:.2f}s（预算 {BUDGET_S}s，"
       f"{'✅ 达标' if total <= BUDGET_S else '❌ 超预算'}）")
    _p(f"   ├─ 规划+门禁  {dt_plan*1000:6.0f} ms")
    _p(f"   └─ 下发(含匹配等待)  {dt_send*1000:6.0f} ms")
    _p("   车侧实际发车时刻以车端日志/`car_state.phase` 变化为准（本节点不假装知道）")
    return 0 if send_ok else 1


def _tail_cmd_file(path: pathlib.Path, from_start: bool = False) -> list[dict]:
    """读 cmd-file（JSONL），返回 (是否从头读)。用于调试期 SSH 触发。"""
    if not path.is_file():
        return []
    lines = path.read_text(encoding="utf-8").splitlines()
    cmds = []
    for ln in lines:
        ln = ln.strip()
        if not ln:
            continue
        try:
            cmds.append(json.loads(ln))
        except Exception:
            continue
    return cmds


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="无人机侧空地指挥链编排：收指令→规划→门禁→下发→计时")
    sub = ap.add_subparsers(dest="cmd")

    def common(p):
        p.add_argument("--map", default=None, help="覆盖地图（默认 vendor config/map.yaml）")
        p.add_argument("--cross-lower", default=None, help="下半场穿洞格（无人机情报）")
        p.add_argument("--cross-upper", default=None, help="上半场穿洞格（无人机情报）")
        p.add_argument("--wait-match", type=float, default=3.0,
                       help="下发前等 DDS 匹配的秒数（默认 3）")

    p = sub.add_parser("once", help="立即执行一次 plan→gate→send")
    common(p)

    p = sub.add_parser("dry-run", help="plan→gate→只打印报文，不连 ROS（可本地跑）")
    common(p)

    p = sub.add_parser("run", help="常驻：尾随 cmd-file，收到 start 指令即执行")
    p.add_argument("--cmd-file", default="~/dzlink_tools/cmd.jsonl",
                   help="命令文件（JSONL，追加 {\"type\":\"start\"} 触发）")
    p.add_argument("--from-start", action="store_true", help="从文件头读历史命令")
    common(p)

    args = ap.parse_args(argv)
    if args.cmd not in ("once", "dry-run", "run"):
        ap.print_help()
        return 2

    t0 = time.perf_counter()
    if args.cmd == "dry-run":
        return _run_pipeline(map_path=args.map, cross_lower=args.cross_lower,
                             cross_upper=args.cross_upper, dry_run=True,
                             wait_match_s=args.wait_match, t0=t0)

    if args.cmd == "once":
        return _run_pipeline(map_path=args.map, cross_lower=args.cross_lower,
                             cross_upper=args.cross_upper, dry_run=False,
                             wait_match_s=args.wait_match, t0=t0)

    # ---- run：常驻尾随 cmd-file，收到 start 即触发（调试期 SSH 触发入口） ----
    cmd_path = pathlib.Path(args.cmd_file).expanduser()
    print(f"[mission_ctl] 常驻，尾随 {cmd_path}；追加 {{\"type\":\"start\"}} 触发一次", flush=True)
    consumed = 0

    def _trigger(c: dict) -> None:
        t_start = time.perf_counter()
        print(f"\n[mission_ctl] 收到 start 指令（{c}），开始规划…", flush=True)
        _run_pipeline(map_path=args.map, cross_lower=args.cross_lower,
                      cross_upper=args.cross_upper, dry_run=False,
                      wait_match_s=args.wait_match, t0=t_start)

    if args.from_start:
        lines = _tail_cmd_file(cmd_path)
        for c in lines:
            consumed += 1
            if str(c.get("type", "")).lower() == "start":
                _trigger(c)
    try:
        while True:
            lines = _tail_cmd_file(cmd_path)
            if len(lines) > consumed:
                for c in lines[consumed:]:
                    consumed += 1
                    if str(c.get("type", "")).lower() == "start":
                        _trigger(c)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[mission_ctl] 已退出")
        return 0


if __name__ == "__main__":
    sys.exit(main())
