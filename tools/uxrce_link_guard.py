#!/usr/bin/env python3
"""uXRCE 链路三态守卫：区分"链路健康 / Agent 死 / 飞控侧卡死"。

背景（2026-10-17 现场实锤）：飞控 uXRCE 客户端卡死后，机上既有 agent
watchdog 只会无脑循环重启 Agent（实测 61 秒一轮），既无效也无提示——
排查耗时最长的问题。本工具做三态判定并给出对症处置：

  LINK_OK       fmu 话题有数据流               -> 无需动作
  AGENT_DOWN    Agent 进程不存在               -> 现有 wdog 会自动拉起；
                                                  持续 DOWN 才需人工
  FC_SIDE_STUCK Agent 活 + fmu 无数据 + Agent 日志出现
                TermiosAgentLinux init 循环(>=3 次)          -> 飞控侧客户端
                卡死，**重启 Agent 无效**，必须飞控断电 5 秒重上电，
                等 ttyACM0 重新枚举后再重启 Agent

判据细节：
  * init 循环计数取自最新 agent 日志尾部（每轮 Agent 启动打一条
    "TermiosAgentLinux ... init ... running"，wdog 循环重启会持续累加）
  * fmu 数据流用 timeout ros2 topic echo --once vehicle_status 探测
  * 本工具纯诊断只读，不杀不启任何进程

用法（密码与 patch_deploy 同约定 AG_ORIN_PASS）:
    python tools/uxrce_link_guard.py --host 192.168.55.1           # 单次判定
    python tools/uxrce_link_guard.py --host ... --loop --interval 15  # 持续监护
    python tools/uxrce_link_guard.py --host ... --json               # 机读输出
退出码：0=LINK_OK / 1=AGENT_DOWN / 2=FC_SIDE_STUCK / 3=探测失败
可挂现有 wdog / cron：`--loop` 模式发现 FC_SIDE_STUCK 时每轮醒目告警。
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import remote_run  # noqa: E402

GUARD_TAG = "[uxrce_link_guard]"
REMOTE_ENV = ("source /opt/ros/humble/setup.bash 2>/dev/null; "
              "source ~/dzpx4/install/setup.bash 2>/dev/null; ")

LINK_OK, AGENT_DOWN, FC_SIDE_STUCK, PROBE_FAIL = 0, 1, 2, 3
STATE_NAMES = {LINK_OK: "LINK_OK", AGENT_DOWN: "AGENT_DOWN",
               FC_SIDE_STUCK: "FC_SIDE_STUCK", PROBE_FAIL: "PROBE_FAIL"}

# 判定阈值
INIT_LOOP_STUCK = 3      # Agent 日志尾部 init 出现次数 >= 此值 = 卡死嫌疑
TAIL_LINES = 200         # 检查的日志尾部行数


def read_password(args) -> str:
    pw = args.password or os.environ.get("AG_ORIN_PASS") or os.environ.get("AG_CAR_PASS")
    if pw:
        return pw
    import getpass
    if not sys.stdin.isatty():
        raise SystemExit("缺少密码：set AG_ORIN_PASS=******（或 --password）")
    return getpass.getpass(f"{args.user}@{args.host} 的密码: ")


def run(host, user, pw, port, cmd, timeout=40, tries=3):
    for i in range(1, tries + 1):
        try:
            code, out, err = remote_run.run(host, user, pw, cmd, timeout=timeout, port=port)
            return out
        except Exception as e:  # noqa: BLE001
            if i == tries:
                print(f"{GUARD_TAG} 连接失败：{type(e).__name__}")
            time.sleep(3)
    return None


def probe(host, user, pw, port):
    """单次三态探测，返回 (state, detail_dict)。"""
    d = {}

    # 1) Agent 进程
    out = run(host, user, pw, port, "pgrep -xc MicroXRCEAgent 2>/dev/null || echo 0")
    if out is None:
        return PROBE_FAIL, {"error": "SSH 不通"}
    d["agent_procs"] = int(out.strip().splitlines()[-1] or 0)

    # 2) fmu 数据流（vehicle_status 一帧，6s 超时）
    out = run(host, user, pw, port,
              f"{REMOTE_ENV} timeout 8 ros2 topic echo --once "
              "/fmu/out/vehicle_status --field arming_state 2>/dev/null",
              timeout=40)
    d["fmu_stream"] = bool(out and re.search(r"\d", out))

    if d["fmu_stream"]:
        return LINK_OK, d

    if d["agent_procs"] == 0:
        return AGENT_DOWN, d

    # 3) Agent 日志 init 循环计数（Agent 活 + 无数据 -> 卡死嫌疑确认）
    out = run(host, user, pw, port,
              f"F=$(ls -t ~/drone_logs/agent_*.log 2>/dev/null | head -1); "
              f"echo \"log=$F\"; tail -{TAIL_LINES} \"$F\" 2>/dev/null | "
              "grep -c 'TermiosAgentLinux.*init'")
    m = re.search(r"(\d+)\s*$", (out or "").strip())
    d["init_loops"] = int(m.group(1)) if m else 0
    d["agent_log"] = (re.search(r"log=(\S+)", out or "") or [None, ""])[1] \
        if out else ""

    if d["init_loops"] >= INIT_LOOP_STUCK:
        return FC_SIDE_STUCK, d
    # Agent 活、无数据、但 init 循环未达标：可能是刚重启/慢启动
    return PROBE_FAIL, d


def render(state, d):
    lines = []
    if state == LINK_OK:
        lines.append(f"{GUARD_TAG} LINK_OK：链路健康，fmu 数据流正常")
    elif state == AGENT_DOWN:
        lines.append(f"{GUARD_TAG} AGENT_DOWN：MicroXRCEAgent 未运行")
        lines.append(f"{GUARD_TAG}   -> 现有 wdog 会自动拉起；持续 DOWN 再人工处理")
    elif state == FC_SIDE_STUCK:
        lines.append(f"{GUARD_TAG} FC_SIDE_STUCK：飞控侧 uXRCE 客户端卡死！")
        lines.append(f"{GUARD_TAG}   证据：Agent 存活({d.get('agent_procs')} 进程) + "
                     f"fmu 无数据 + Agent 日志 init 循环 {d.get('init_loops')} 次")
        lines.append(f"{GUARD_TAG}   处置（重启 Agent 无效，勿再循环重启）：")
        lines.append(f"{GUARD_TAG}     1. 飞控断电 5 秒后重上电")
        lines.append(f"{GUARD_TAG}     2. 等 /dev/ttyACM0 重新枚举")
        lines.append(f"{GUARD_TAG}     3. 重启 MicroXRCEAgent（wdog 会自动拉起亦可）")
        lines.append(f"{GUARD_TAG}     4. 复跑本工具确认 LINK_OK")
    else:
        lines.append(f"{GUARD_TAG} PROBE_FAIL：探测未决（{d}）——"
                     "Agent 在但数据未到/刚重启，稍后复测")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="uxrce_link_guard.py",
                                 description="uXRCE 链路三态守卫（LINK_OK/AGENT_DOWN/FC_SIDE_STUCK）")
    ap.add_argument("--host", required=True)
    ap.add_argument("--user", default="duzhong")
    ap.add_argument("--password", default=None)
    ap.add_argument("--port", type=int, default=22)
    ap.add_argument("--loop", action="store_true", help="持续监护模式")
    ap.add_argument("--interval", type=float, default=15.0, help="循环间隔秒（默认 15）")
    ap.add_argument("--json", action="store_true", help="JSON 机读输出（单次模式）")
    args = ap.parse_args(argv)
    pw = read_password(args)

    if not args.loop:
        state, d = probe(args.host, args.user, pw, args.port)
        if args.json:
            print(json.dumps({"state": STATE_NAMES[state], "detail": d},
                             ensure_ascii=False))
        else:
            print(render(state, d))
        return state

    # 持续监护：状态变化时全量输出，未变时 60s 心跳一行
    last_state, last_full = None, 0.0
    while True:
        try:
            state, d = probe(args.host, args.user, pw, args.port)
            now = time.time()
            if state != last_state or now - last_full >= 60:
                print(render(state, d), flush=True)
                last_state, last_full = state, now
        except KeyboardInterrupt:
            print(f"{GUARD_TAG} 退出", flush=True)
            return 0
        time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())
