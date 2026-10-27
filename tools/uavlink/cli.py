# uavlink.cli —— 由 tools/uav_link.py 拆出（argparse 入口与 run/ctl/logs/report 子命令）。与原文件逐行一致（AST 校验过）。
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import socket
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path
from uavlink.blackbox import BlackboxWriter
from uavlink.config import ALL_ACTIONS, COMMAND_MAP, HOST_CACHE, ORIN_REPO, ORIN_USER_DEFAULT, TMUX_SESSION, _LOGS_STAGE_CMD
from uavlink.host import probe_ssh, read_password, resolve_host, save_cached_host
from uavlink.link import OrinLink, _connect, _exec_collect, _link_preflight, _sftp_get_tree, _summary_lines, _upload_sampler
from uavlink.report import cmd_report
from uavlink.rules import UavRuleEngine
from uavlink.safety import check_gate, clamp_alt, confirm_word, countdown
from uavlink.sampler_src import sampler_launch_cmd
from uavlink.telemetry import TelemetryState, parse_sampler_line, render_channel_failure, render_command_map, render_dashboard

def cmd_run(args) -> int:  # pragma: no cover - 主流程需要真实无人机，纯逻辑已单测
    host = resolve_host(args)
    if not host:
        print("[失败] 未给 --host 且无上次缓存。用法: "
              "python tools/uav_link.py run --host <Orin IP>", file=sys.stderr)
        return 2
    bad = probe_ssh(host, args.port)
    if bad:
        print(f"[失败] {bad}", file=sys.stderr)
        return 4
    pw = read_password(args)
    save_cached_host(host, getattr(args, "host_cache", HOST_CACHE))

    sampler_args = " --no-dzuav" if args.no_dzuav else ""
    link = OrinLink(host, args.orin_user, pw, args.port, sampler_args)
    link.start()
    print(f"[机载] 已发起 SSH 遥测 {args.orin_user}@{host} …（Ctrl-C 优雅退出）")

    out_path = Path(args.out) if args.out else \
        Path("logs") / "uav_link" / (time.strftime("%Y%m%d_%H%M%S") + ".jsonl")
    box = BlackboxWriter(out_path)
    print(f"[黑匣子] {out_path}")

    state = TelemetryState()
    engine = UavRuleEngine()
    engine.expect_dzuav = not args.no_dzuav
    alerts: list[dict] = []
    n_rows = 0
    hello_shown = False
    err_shown = False
    last_print = 0.0
    t_start = time.time()
    try:
        while True:
            t = time.time()
            latest_rows = []
            while True:
                try:
                    latest_rows.append(link.queue.get_nowait())
                except queue.Empty:
                    break
            for t_recv, obj in latest_rows:
                n_rows += 1
                box.write(obj, recv=t_recv)
                if obj["kind"] == "hello" and not hello_shown:
                    hello_shown = True
                    tf = obj["topics_found"]
                    print(f"[机载] 采样器发现 {len(tf)} 个话题: "
                          + (", ".join(tf[:12]) + (" …" if len(tf) > 12 else "")
                             if tf else "（空——MicroXRCEAgent 可能没起）"))
                elif obj["kind"] == "error":
                    if not err_shown:
                        err_shown = True
                        print(f"[机载] 采样器报错：{obj['error']}")
                elif obj["kind"] == "sample":
                    state.feed(t_recv, obj)
                    engine.note_sample(t_recv, obj.get("topic", "?"),
                                       obj.get("hz"))
            if link.error and not err_shown:
                err_shown = True
                print(f"[机载] 链路异常：{link.error}")

            fired = engine.update(t, state.engine_input())
            for a in fired:
                print(f"[告警] {a['rule']}: {a['msg']}")
                box.write({"src": "alert", "rule": a["rule"], "msg": a["msg"]},
                          recv=t)
            alerts.extend(fired)

            if t - last_print >= 0.5:   # 2Hz 仪表盘
                last_print = t
                active = ",".join(sorted({a["rule"] for a in alerts
                                          if t - a["t"] <= 5.0})) or "无"
                print(render_dashboard(state, active, now=t))
            if link.error and link.queue.empty():
                break   # 链路已死且无积余，退出循环走收尾
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n[退出] Ctrl-C，回收远程进程组…")
    finally:
        link.stop()
        link.join(timeout=5)
        span = time.time() - t_start
        lines = _summary_lines(state, engine.counts, span, n_rows)
        for ln in lines:
            print(ln)
        box.write({"summary": lines})
        box.close()
        print(f"[黑匣子已收尾] {out_path}")
    return 0
# ---------------------------------------------------------------- ctl 调控支配（全部显式闸门）

def cmd_ctl(args) -> int:  # pragma: no cover - 远端交互不测，闸门/映射纯逻辑已单测
    if not args.action and not args.print_caps:
        print("[失败] 缺动作：uav_link.py ctl <" + "/".join(ALL_ACTIONS) + ">\n"
              + render_command_map(), file=sys.stderr)
        return 2
    action = args.action
    if action:
        # 安全闸门第一关：高风险没 --yes 直接拒（碰网络之前就拒）
        deny = check_gate(action, args.yes)
        if deny:
            print(f"[拒绝] {deny}", file=sys.stderr)
            return 2

    host, pw = _link_preflight(args)
    if not host:
        return 4
    if action:
        # 倒计时挪到 preflight 之后：主机都够不着就别吓人倒数；
        # Ctrl-C 友好中止——倒数期间反悔随时可以
        try:
            countdown(action, yes=args.yes)
        except KeyboardInterrupt:
            print("\n[中止] Ctrl-C，未执行任何动作")
            return 130
        # kill 双闸门第二关：--yes 之外再逐字输入全词 "kill"
        # （对齐 flight_console.py:619；shell 历史里 land/kill 只差一个上翻键）
        if not confirm_word(action):
            return 2

    # SSH 成功后才建回执黑匣子（连接失败路径不留未关闭的句柄）
    try:
        cli = _connect(host, args.orin_user, pw, args.port)
    except Exception as e:  # noqa: BLE001 - 网络层异常原样报出便于现场判断
        print(f"[失败] SSH 连接失败: {type(e).__name__}: {e}", file=sys.stderr)
        return 4
    box = BlackboxWriter(Path("logs") / "uav_link" / "ctl_receipts.jsonl", append=True)

    try:
        if args.print_caps:
            _upload_sampler(cli)
            rc, out, err = _exec_collect(cli, sampler_launch_cmd(" --print-caps"),
                                         timeout=60)
            sys.stdout.write(out)
            if err.strip():
                sys.stderr.write("--- stderr ---\n" + err[:500])
            print()
            print(render_command_map())
            return 0 if rc == 0 else 1

        if action in COMMAND_MAP:
            _upload_sampler(cli)
            extra = f" --do {action}"
            if action == "takeoff":
                alt = clamp_alt(args.alt)
                if alt != args.alt:
                    print(f"[注意] --alt {args.alt} 超出 0.5~10m，已夹到 {alt}m")
                extra += f" --alt {alt}"
            rc, out, err = _exec_collect(cli, sampler_launch_cmd(extra), timeout=90)
            ctl_row = None
            found_topics: list[str] = []
            for line in out.splitlines():
                obj = parse_sampler_line(line)
                if not obj:
                    continue
                if obj["kind"] == "hello":
                    found_topics = obj["topics_found"]
                elif obj["kind"] == "ctl":
                    ctl_row = obj
                elif obj["kind"] == "warn":
                    # 采样器执行前的醒目提示（直发飞控绕过状态机防护等）
                    print("!" * 60, file=sys.stderr)
                    print(f"!! [机载警告] {obj['msg']}", file=sys.stderr)
                    print("!" * 60, file=sys.stderr)
                elif obj["kind"] == "error":
                    print(f"[机载] {obj['error']}", file=sys.stderr)
            ok = bool(ctl_row and ctl_row["ok"])
            detail = ctl_row["detail"] if ctl_row else \
                f"采样器未回执（退出码 {rc}）: {err.strip()[:300]}"
            if ok:
                print(f"[完成] ctl {action}：{detail}")
            else:
                print(render_channel_failure(action, found_topics) if not ctl_row
                      else f"[失败] ctl {action}：{detail}", file=sys.stderr)
            box.write({"src": "ctl", "action": action, "ok": ok, "detail": detail})
            return 0 if ok else 5

        # ---- 栈生命周期（tmux / orin_bringup.sh，不走 ROS）----
        script = f"{ORIN_REPO}/tools/orin_bringup.sh"
        if action == "stack-status":
            remote = (f"if [ -f {script} ]; then bash {script} status; "
                      f"else tmux has-session -t {TMUX_SESSION} 2>/dev/null && "
                      f"tmux list-windows -t {TMUX_SESSION} "
                      f"-F '#{{window_name}} #{{pane_current_command}}' "
                      f"|| echo 'tmux 会话 {TMUX_SESSION} 不存在'; fi")
        else:  # stack-restart（已过 --yes 闸门）
            remote = (f"if [ -f {script} ]; then bash {script} down; sleep 2; "
                      f"bash {script} up; "
                      f"else echo 'Orin 上未找到 {script}（先 orin_deploy 同步仓库）'; "
                      f"exit 6; fi")
        # exec_command 本就过登录 shell，if/&&/|| 直接可用（不再套 bash -lc：
        # remote 内含单引号，外层再单引号包裹会被截断）
        rc, out, err = _exec_collect(cli, remote, timeout=120)
        sys.stdout.write(out)
        if err.strip():
            sys.stderr.write("--- stderr ---\n" + err[:500])
        box.write({"src": "ctl", "action": action, "ok": rc == 0,
                   "detail": out.strip()[:300]})
        return 0 if rc == 0 else 1
    finally:
        box.close()
        cli.close()
def cmd_logs(args) -> int:  # pragma: no cover - 需要真实无人机
    host, pw = _link_preflight(args)
    if not host:
        return 4
    out_dir = Path(args.out) if args.out else \
        Path("logs") / "uav_link" / time.strftime("%Y%m%d_%H%M%S")
    try:
        cli = _connect(host, args.orin_user, pw, args.port)
    except Exception as e:  # noqa: BLE001
        print(f"[失败] SSH 连接失败: {type(e).__name__}: {e}", file=sys.stderr)
        return 4
    try:
        rc, out, err = _exec_collect(cli, f"bash -lc '{_LOGS_STAGE_CMD}'",
                                     timeout=120)
        if rc != 0:
            print(f"[警告] 机载暂存脚本退出码 {rc}（继续尽量取回）: "
                  f"{err.strip()[:300]}", file=sys.stderr)
        n = _sftp_get_tree(cli, "/tmp/uav_logs_stage", out_dir)
    finally:
        cli.close()
    # 清单与大小
    print(f"[已取回] {n} 个文件 -> {out_dir}")
    total = 0
    for p in sorted(out_dir.rglob("*")):
        if p.is_file():
            sz = p.stat().st_size
            total += sz
            print(f"  {p.relative_to(out_dir)}  {sz / 1024:.1f} KiB")
    print(f"  合计 {total / 1024:.1f} KiB")
    return 0
# ---------------------------------------------------------------- CLI

def _add_link_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--host", default=None,
                   help="Orin IP（给过一次就缓存到 uav_link.local.env，之后可省）")
    p.add_argument("--orin-user",
                   default=os.environ.get("AG_ORIN_USER") or ORIN_USER_DEFAULT,
                   help="机载用户名（可用 AG_ORIN_USER 环境变量；默认 "
                        f"{ORIN_USER_DEFAULT}，本机 Orin 实为 duzhong）")
    p.add_argument("--port", type=int, default=22)
    p.add_argument("--password", default=None,
                   help="机载密码；更推荐 AG_ORIN_PASS 环境变量")
    p.add_argument("--host-cache", default=str(HOST_CACHE),
                   help="--host 缓存文件路径（默认 logs/uav_link/uav_link.local.env；"
                        "换机/换网段测试时可指向别处，避免互相覆盖）")
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="uav_link.py",
        description="无人机地面站：看得见（遥测）、控得住（调控）、查得回（黑匣子+复盘）"
                    "（详见文件头说明）")
    sub = ap.add_subparsers(dest="cmd")

    rn = sub.add_parser("run", help="只读遥测 + 黑匣子 + 实时告警（默认）")
    _add_link_args(rn)
    rn.add_argument("--out", default=None,
                    help="黑匣子路径（默认 logs/uav_link/<时间戳>.jsonl）")
    rn.add_argument("--no-dzuav", action="store_true",
                    help="不并 dzuav 健康通道（也不评 DZUAV_SILENT）")
    rn.set_defaults(func=cmd_run)

    ct = sub.add_parser("ctl", help="调控支配（默认什么都不做，高风险要 --yes）")
    ct.add_argument("action", nargs="?", choices=ALL_ACTIONS,
                    help="arm/disarm/takeoff/land/return/kill/stack-status/stack-restart")
    ct.add_argument("--yes", action="store_true",
                    help="高风险动作确认闸门；对倒计时动作跳过 3s 倒计时")
    ct.add_argument("--alt", type=float, default=2.0,
                    help="takeoff 目标高度（米，默认 2.0）")
    ct.add_argument("--print-caps", action="store_true",
                    help="打印采样器实际发现的话题/命令通道 + 命令映射表后退出")
    _add_link_args(ct)
    ct.set_defaults(func=cmd_ctl)

    lg = sub.add_parser("logs", help="取回机载日志（dzuav jsonl/ROS log/tmux 窗口）")
    _add_link_args(lg)
    lg.add_argument("--out", default=None,
                    help="本地目录（默认 logs/uav_link/<时间戳>/）")
    lg.set_defaults(func=cmd_logs)

    rp = sub.add_parser("report", help="离线复盘黑匣子（严重/注意/信息 分级）")
    rp.add_argument("blackbox", help="黑匣子 jsonl 路径")
    rp.set_defaults(func=cmd_report)
    return ap
def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    # 默认子命令是 run：`uav_link.py --host ...` 等价 `uav_link.py run --host ...`
    subs = {"run", "ctl", "logs", "report"}
    if argv and argv[0] not in subs and argv[0] not in ("-h", "--help"):
        if argv[0].startswith("-"):
            argv.insert(0, "run")
        else:
            print(f"未知子命令: {argv[0]}（应为 run / ctl / logs / report）",
                  file=sys.stderr)
            return 2
    elif not argv:
        argv = ["run"]
    ap = build_parser()
    args = ap.parse_args(argv)
    if not getattr(args, "func", None):
        ap.print_help()
        return 0
    # ⚠️ 同 remote_run.py：Windows 控制台默认 GBK，先开容错防止特殊符号崩整条命令
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(errors="replace")
        except Exception:  # noqa: BLE001
            pass
    return args.func(args)
