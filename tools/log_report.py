#!/usr/bin/env python3
"""把黑匣子日志变成一份**跑完就能看**的复盘报告。

    python tools/log_report.py                  # 自动找最新日志，取最后一局
    python tools/log_report.py --all            # 不切局，全部事件
    python tools/log_report.py --dir /tmp/ag_logs/20260927
    python tools/log_report.py --out run.md     # 顺便存成文件
    python tools/log_report.py --json           # 机器可读（CI 用）
    python tools/log_report.py --strict         # 有 critical 就退出码 1
    python tools/log_report.py --list           # 只列出找到了哪些日志目录

报告分四段：结论（自动诊断）→ 概览指标 → 时间轴 → 事件统计。
"结论"里的每一条都带**下一步该查什么**，不用再去 grep 原始日志。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
# 每个包目录都要入 path（src/<pkg>/<pkg>/ 两层布局），与 tests/conftest.py 一致
for _p in [SRC] + sorted(p for p in SRC.iterdir() if p.is_dir()):
    sys.path.insert(0, str(_p))

from ag_core.blackbox import (  # noqa: E402
    EV,
    Event,
    collect_jsonl,
    default_log_dir,
    find_log_dirs,
    read_events,
    session_start_index,
)
from ag_core.log_diagnosis import SEVERITY_ORDER, diagnose, summarize  # noqa: E402

SEV_LABEL = {"critical": "严重", "warn": "注意", "info": "信息", "ok": "正常"}
SEV_MARK = {"critical": "[严重]", "warn": "[注意]", "info": "[信息]", "ok": "[正常]"}


# ---------------------------------------------------------------- 事件转人话
def _cells(v: Any) -> str:
    if not v:
        return "-"
    if isinstance(v, (list, tuple)):
        return ",".join(str(x) for x in v)
    return str(v)


def describe(e: Event) -> str:
    """把一条事件写成一行能看懂的话（时间轴用）。"""
    ev, g = e.ev, e.get
    if ev == EV.SESSION_BEGIN:
        return (f"开局 {g('start_cell')}->{g('goal_cell')} "
                f"dry_run={g('dry_run')} budget={g('budget_s')}s")
    if ev == EV.NODE_READY:
        return f"{e.node} 启动 pid={g('pid')} host={g('host')}"
    if ev == EV.NODE_ARGS:
        keys = ("cols", "rows", "min_votes", "dry_run", "allow_classes", "enable_fire")
        kv = ", ".join(f"{k}={g(k)}" for k in keys if g(k) is not None)
        return kv or "(无关键参数)"
    if ev == EV.INTEL_RECV:
        ok = "收下" if g("ok", True) else f"丢弃({g('why')})"
        return (f"{ok} seq={g('seq')} 雷[{_cells(g('mines'))}] "
                f"障[{_cells(g('obstacles'))}] 桥[{g('bridge_a')},{g('bridge_b')}] "
                f"靶[{_cells(g('target_cells'))}] 面={g('target_face')}")
    if ev == EV.INTEL_STABLE:
        return (f"情报收敛: 雷[{_cells(g('mines'))}] 障[{_cells(g('obstacles'))}] "
                f"桥[{g('bridge_a')},{g('bridge_b')}] 靶[{_cells(g('target_cells'))}] "
                f"面={g('target_face')}")
    if ev == EV.INTEL_ACCUM:
        return f"累积: 雷[{_cells(g('mines'))}] 靶[{_cells(g('target_cells'))}]"
    if ev == EV.PLAN_BUILT:
        return (f"{g('tasks')} 段, 打击点[{_cells(g('shoot_stops'))}], "
                f"eta={g('eta_s')}s")
    if ev == EV.ROUTE_PUBLISH:
        return f"[{g('kind')}] {g('cmd')}" + ("  (dry_run)" if g("dry_run") else "")
    if ev == EV.ROUTE_BLOCKED:
        return f"未发车: {g('reason')} — {g('detail')}"
    if ev == EV.ROUTE_REJECT:
        return f"路线非法: {g('reason')} — {g('detail')}"
    if ev == EV.SHOOT_COUNT:
        return f"累计发射 {g('value')} 发"
    if ev == EV.AIM_PASS:
        return f"放行 cls={g('cls')}（累计放行 {g('passed_total')}）"
    if ev == EV.AIM_BLOCK:
        return (f"拦截 cls={g('cls')} why={g('why')}"
                f"（累计拦截 {g('blocked_total')} / 放行 {g('passed_total')}）")
    if ev == EV.FIRE_REQUEST:
        return f"请求开火: {g('reason')}"
    if ev == EV.FIRE_DENIED:
        return f"拒绝开火: {g('reason')}"
    if ev == EV.FIRE_SENT:
        return "已发射"
    if ev == EV.REFEREE_STATE:
        return (f"血={g('hp')} 弹={g('ammo')} 已射={g('shots')} "
                f"受击={g('hit')} 靶血={g('target_hp')}")
    if ev == EV.LINK_TX:
        return f"发帧 seq={g('seq')} {g('bytes')}B"
    if ev == EV.LINK_RX:
        return f"收帧 seq={g('seq')} {g('bytes')}B ok={g('ok', True)}"
    if ev == EV.LINK_DROP:
        return f"丢帧 seq={g('seq')} why={g('why')}"
    if ev == EV.VISION_DETECT:
        return f"检测 {g('count')} 个目标"
    if ev == EV.HOMOGRAPHY:
        return (f"单应{'OK' if g('ok') else '无效'}（{g('stage')}）"
                f"{'' if g('ok') else '：' + str(g('reason'))}")
    if ev == EV.VISION_MODEL:
        return f"模型 {g('model')} 解码={g('kpt_unit')}/{g('cls_activation')}"
    if ev == EV.VISION_STALE:
        return (f"{g('note')}：{g('gap_s')}s 无图像"
                f"（已收到 {g('images')} 帧）")
    if ev == EV.MISSION_TASK:
        return (f"执行[{g('index')}/{g('total')}] {g('type')} -> {g('cell')} "
                f"+{g('score')} eta={g('eta_s')}s")
    if ev == EV.MISSION_DONE:
        return f"计划执行完（共 {g('total')} 段）"
    if ev == EV.ROS_LOG:
        return f"({e.lvl}) {g('msg')}"
    if ev == EV.NODE_ERROR:
        return f"异常 {g('where')}: {g('error')}"
    extra = {k: v for k, v in e.data.items() if not k.startswith("_")}
    return json.dumps(extra, ensure_ascii=False, default=str) if extra else ""


# ---------------------------------------------------------------- 渲染
def render_markdown(paths: Sequence[Path], events: List[Event], bad: int,
                    max_gap: float, timeline: int, note: str = "",
                    nodes_all=None) -> str:
    m = summarize(events, nodes_all=nodes_all)
    m["blackbox_errors"] = bad
    findings = diagnose(events, max_gap_s=max_gap, nodes_all=nodes_all)

    L: List[str] = []
    L.append("# 跑车复盘报告")
    L.append("")
    if note:
        L.append(f"> {note}")
        L.append("")
    L.append(f"- 日志文件：{len(paths)} 个")
    for p in paths:
        L.append(f"  - `{p}`")
    L.append(f"- 事件数：**{m['events']}**（无法解析的行：{bad}）")
    L.append(f"- 涉及节点：{', '.join(m['nodes']) or '-'}")
    L.append(f"- 时长：{m['span_s']:.1f}s，最长静默 {m['max_gap_s']:.1f}s")
    L.append("")

    # ---- 一、结论 ----
    L.append("## 一、结论（自动诊断）")
    L.append("")
    if not findings:
        L.append("没有发现异常。")
    else:
        L.append("| 级别 | 结论 | 说明 | 下一步 |")
        L.append("|---|---|---|---|")
        for f in findings:
            L.append(f"| {SEV_LABEL.get(f.severity, f.severity)} | {f.title} | "
                     f"{f.detail.replace('|', '/')} | {f.hint.replace('|', '/')} |")
    L.append("")

    # ---- 二、概览 ----
    L.append("## 二、概览")
    L.append("")
    rows = [
        ("情报帧（收到 / 无效）", f"{m['intel_recv']} / {m['intel_recv_bad']}"),
        ("情报收敛", f"{m['intel_stable']} 次"
                     + (f"，首次 T+{m['intel_first_stable_s']}s"
                        if m["intel_first_stable_s"] is not None else "")),
        ("链路（发 / 收 / 丢）", f"{m['link_tx']} / {m['link_rx']} / {m['link_drop']}"),
        ("规划次数", m["plan_built"]),
        ("路线下发（reset / route）", f"{m['reset_sent']} / {m['route_sent']}"),
        ("未发车次数", m["route_blocked"] + m["route_reject"]),
        ("停靠打击点", _cells(m["shoot_stops"])),
        ("车端上报发射数", m["shots"] if m["shots"] is not None else "-"),
        ("瞄准闸门（放行 / 拦截）", f"{m['aim_pass']} / {m['aim_block']}"),
        ("开火（请求 / 拒 / 发射）",
         f"{m['fire_request']} / {m['fire_denied']} / {m['fire_sent']}"),
        ("裁判状态更新", m["referee_updates"]),
        ("相机断流次数", m["vision_stale"]),
        ("dry_run", m["dry_run"]),
    ]
    L.append("| 指标 | 值 |")
    L.append("|---|---|")
    for k, v in rows:
        L.append(f"| {k} | {v} |")
    if m["block_reasons"]:
        L.append(f"| 未发车原因分布 | {m['block_reasons']} |")
    if m["link_drop_reasons"]:
        L.append(f"| 丢帧原因分布 | {m['link_drop_reasons']} |")
    if m["aim_block_cls"]:
        L.append(f"| 拦截类别分布（1=蓝敌 2=红友） | {m['aim_block_cls']} |")
    L.append("")

    # ---- 三、时间轴 ----
    L.append("## 三、时间轴")
    L.append("")
    if not events:
        L.append("（无事件）")
    else:
        t0 = events[0].ts
        shown = events
        truncated = 0
        if timeline > 0 and len(events) > timeline:
            # 头尾各留一半：开头是"怎么起来的"，结尾是"最后卡在哪"
            head = timeline // 2
            tail = timeline - head
            shown = events[:head] + events[-tail:]
            truncated = len(events) - len(shown)
        L.append("| 时间 | 节点 | 事件 | 内容 |")
        L.append("|---|---|---|---|")
        for e in shown:
            L.append(f"| T+{e.ts - t0:.1f}s | {e.node} | {e.ev} | "
                     f"{describe(e).replace('|', '/')} |")
        if truncated:
            L.append(f"| ... | | | （省略中间 {truncated} 条，用 `--timeline 0` 全看） |")
    L.append("")

    # ---- 四、事件统计 ----
    L.append("## 四、事件统计")
    L.append("")
    L.append("| 事件 | 次数 |")
    L.append("|---|---|")
    for k, v in sorted(m["kinds"].items(), key=lambda kv: -kv[1]):
        L.append(f"| `{k}` | {v} |")
    L.append("")
    return "\n".join(L)


def print_summary(paths: Sequence[Path], events: List[Event], bad: int,
                  max_gap: float, nodes_all=None) -> List[Any]:
    """控制台摘要：**先看结论**，细节去报告里找。"""
    m = summarize(events, nodes_all=nodes_all)
    m["blackbox_errors"] = bad
    findings = diagnose(events, max_gap_s=max_gap, nodes_all=nodes_all)
    counts = {s: sum(1 for f in findings if f.severity == s) for s in SEVERITY_ORDER}

    print("=" * 74)
    print("跑车复盘")
    print("=" * 74)
    print(f"日志目录: {paths[0].parent if paths else '-'}")
    print(f"文件 {len(paths)} 个 | 事件 {m['events']} 条 | 坏行 {bad} | "
          f"时长 {m['span_s']:.1f}s | 最长静默 {m['max_gap_s']:.1f}s")
    print(f"节点: {', '.join(m['nodes']) or '-'}")
    print(f"路线: reset×{m['reset_sent']} route×{m['route_sent']} | "
          f"打击点[{_cells(m['shoot_stops'])}] | "
          f"情报 {m['intel_recv']} 帧 / 收敛 {m['intel_stable']} 次")
    print("-" * 74)
    if not findings:
        print("没有发现异常。")
    for f in findings:
        print(f"{SEV_MARK.get(f.severity, '[?]')} {f.title}")
        if f.detail:
            print(f"        {f.detail}")
        if f.hint:
            print(f"        -> {f.hint}")
    print("-" * 74)
    print(f"严重 {counts['critical']} | 注意 {counts['warn']} | "
          f"信息 {counts['info']} | 正常 {counts['ok']}")
    return findings


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="黑匣子日志复盘报告")
    ap.add_argument("--dir", default=None,
                    help=f"日志目录（默认 {default_log_dir()}，可用 AG_LOG_DIR 覆盖）")
    ap.add_argument("--session", default=None, help="只读某个会话子目录")
    ap.add_argument("--all", action="store_true",
                    help="不切局，读目录里全部事件（默认只从最后一个 session.begin 开始）")
    ap.add_argument("--max-gap", type=float, default=30.0, help="静默告警阈值（秒）")
    ap.add_argument("--timeline", type=int, default=300,
                    help="时间轴最多显示多少条（0 = 全部）")
    ap.add_argument("--out", default=None, help="把 Markdown 报告写到文件")
    ap.add_argument("--json", action="store_true", help="输出 JSON（供脚本消费）")
    ap.add_argument("--strict", action="store_true", help="有 critical 时退出码为 1")
    ap.add_argument("--list", action="store_true", help="只列出找到的日志目录")
    args = ap.parse_args(argv)

    try:  # Windows 控制台默认 gbk，中文报告会炸
        sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    except Exception:  # noqa: BLE001
        pass

    root = Path(args.dir) if args.dir else default_log_dir()
    if args.list:
        dirs = find_log_dirs(root)
        if not dirs:
            print(f"{root} 下没有找到任何 .jsonl 日志")
            return 0
        print(f"{root} 下的日志目录（按最新写入排序）：")
        for d in dirs:
            files = collect_jsonl(d)
            newest = max((f.stat().st_mtime for f in files), default=0)
            import datetime
            ts = datetime.datetime.fromtimestamp(newest).strftime("%Y-%m-%d %H:%M:%S")
            print(f"  {ts}  {len(files):3d} 个文件  {d}")
        return 0

    target = root / args.session if args.session else root
    if not collect_jsonl(target):
        print(f"没找到日志：{target}", file=sys.stderr)
        dirs = find_log_dirs(root)
        if dirs:
            print("这些位置有日志：", file=sys.stderr)
            for d in dirs[:5]:
                print(f"  {d}", file=sys.stderr)
        else:
            print("先跑一局，或用 AG_LOG_DIR 指定日志根目录。", file=sys.stderr)
        return 2

    paths = collect_jsonl(target)
    events, bad = read_events(paths)
    # 节点集合按**全部**事件算（指标仍只看最后一局）：
    # referee_node 比 car_node 先启动，它的 node.ready 落在"最后一局"窗口之前，
    # 只按窗口算会误判成"referee_node 没在跑"（2026-09-28 实测踩到）。
    nodes_all = {e.node for e in events if e.node}
    note = ""
    if not args.all:
        idx = session_start_index(events)
        if idx > 0:
            note = (f"只统计**最后一局**（从第 {idx + 1} 条 session.begin 起）；"
                    f"用 `--all` 看全部 {len(events)} 条。")
            events = events[idx:]
        elif events and events[0].ev != EV.SESSION_BEGIN:
            note = "（没有 session.begin，按全部事件统计）"

    if args.json:
        m = summarize(events, nodes_all=nodes_all)
        m["blackbox_errors"] = bad
        print(json.dumps({
            "paths": [str(p) for p in paths],
            "bad_lines": bad,
            "summary": m,
            "findings": [f.to_dict() for f in
                         diagnose(events, max_gap_s=args.max_gap, nodes_all=nodes_all)],
        }, ensure_ascii=False, indent=2, default=str))
        return 0

    findings = print_summary(paths, events, bad, args.max_gap, nodes_all=nodes_all)
    md = render_markdown(paths, events, bad, args.max_gap, args.timeline, note,
                         nodes_all=nodes_all)
    if args.out:
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(md, encoding="utf-8")
        print(f"Markdown 报告已写入: {out}")
    else:
        print("（想存成文件加 --out report.md；机器可读加 --json）")

    if args.strict and any(f.severity == "critical" for f in findings):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
