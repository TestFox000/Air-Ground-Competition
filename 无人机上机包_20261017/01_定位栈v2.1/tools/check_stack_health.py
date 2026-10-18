#!/usr/bin/env python3
"""无人机定位栈一键体检：双实例 / EKF reset 风暴 / 韧性告警扫描。

背景（2026-10-17 日志实锤）：10-16 21:39:53~21:46:38 手动栈与 autostart 栈
并行——双 FAST-LIO 同发 /localization 406 秒（交错流即 26~585m 位置跳变的
直接来源）、双 px4_odom_convert 同发 VIO 93 秒，EKF 被两路测量撕扯出
reset 风暴。起飞前跑本工具 30 秒排掉这类问题。

用法（密码与 patch_deploy 同约定）:
    set AG_ORIN_PASS=******
    python tools/check_stack_health.py --host 192.168.55.1
    python tools/check_stack_health.py --host ... --kill-dup   # 自动杀多余实例（保留最早启动的）

检查项：
  1) 节点进程数：livox_ros_driver2_node / fastlio_mapping /
     px4_odom_convert / MicroXRCEAgent —— 每类应恰好 1
  2) 话题发布者数：/localization、/fmu/in/vehicle_visual_odometry —— 应恰好 1
  3) EKF xy_reset_counter 静置 10s 增长 —— 应 <=2
  4) 最近日志扫描：DUPLICATE / SLAM INSANE / RESET STORM 告警
依赖: pip install paramiko（复用 tools/remote_run.py 原语）
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import remote_run  # noqa: E402

# (显示名, pgrep 精确进程名, 期望实例数)
PROC_CHECKS = [
    ("livox 雷达驱动", "livox_ros_driver2_node", 1),
    ("FAST-LIO 建图节点", "fastlio_mapping", 1),
    ("VIO 转换节点", "px4_odom_convert", 1),
    ("uXRCE Agent", "MicroXRCEAgent", 1),
]

TOPIC_CHECKS = [
    ("SLAM 位姿源 /localization", "/localization"),
    ("VIO 喂 EKF /fmu/in/vehicle_visual_odometry", "/fmu/in/vehicle_visual_odometry"),
]

REMOTE_ENV = ("source /opt/ros/humble/setup.bash; "
              "source ~/dzpx4/install/setup.bash 2>/dev/null; ")


def read_password(args) -> str:
    pw = args.password or os.environ.get("AG_ORIN_PASS") or os.environ.get("AG_CAR_PASS")
    if pw:
        return pw
    import getpass
    if not sys.stdin.isatty():
        raise SystemExit("缺少密码：set AG_ORIN_PASS=******（或 --password）")
    return getpass.getpass(f"{args.user}@{args.host} 的密码: ")


def run(host, user, pw, port, cmd, timeout=60):
    for attempt in range(1, 4):
        try:
            code, out, err = remote_run.run(host, user, pw, cmd, timeout=timeout, port=port)
            if code == 0:
                return out
        except Exception as e:  # noqa: BLE001
            print(f"[..] 连接重试（{attempt}/3）: {type(e).__name__}")
            time.sleep(5)
    raise SystemExit(f"[失败] 远程命令无响应：{cmd[:80]}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="check_stack_health.py",
                                 description="无人机定位栈一键体检（双实例/reset 风暴/告警）")
    ap.add_argument("--host", required=True)
    ap.add_argument("--user", default="duzhong")
    ap.add_argument("--password", default=None)
    ap.add_argument("--port", type=int, default=22)
    ap.add_argument("--kill-dup", action="store_true",
                    help="发现同类多实例时自动杀掉较新实例（保留最早启动的）")
    ap.add_argument("--start-cmd", default=None,
                    help="栈未运行时的拉起命令（autostart v3 模式下定位栈需手动启动，"
                         "如 --start-cmd 'bash ~/start_drone.sh'）；执行后自动复检")
    args = ap.parse_args(argv)
    pw = read_password(args)
    host, user, port = args.host, args.user, args.port

    problems = []

    # ---- 1) 进程数 ----
    print("=" * 62)
    print("[1/4] 节点进程数检查（每类应恰好 1 个）")
    print("=" * 62)
    for label, proc, expect in PROC_CHECKS:
        out = run(host, user, pw, port,
                  f"pgrep -xc {proc} 2>/dev/null || echo 0")
        n = int(out.strip().splitlines()[-1] or 0)
        status = "OK " if n == expect else "FAIL"
        print(f"  [{status}] {label} ({proc}): {n} 个（期望 {expect}）")
        if n > expect:
            problems.append(f"{label} 多实例: {n} 个")
            pids = run(host, user, pw, port,
                       f"pgrep -x {proc} 2>/dev/null | sort -n").split()
            print(f"         PIDs: {' '.join(pids)}")
            if args.kill_dup:
                # 保留最早启动（PID 最小），杀其余
                victims = pids[1:]
                for v in victims:
                    run(host, user, pw, port, f"kill {v} 2>/dev/null")
                    print(f"         [kill-dup] 已 kill 较新实例 PID {v}（保留 {pids[0]}）")
                time.sleep(2)
                n2 = run(host, user, pw, port, f"pgrep -xc {proc} 2>/dev/null || echo 0")
                print(f"         清理后剩余: {n2.strip().splitlines()[-1]} 个")
        elif n == 0:
            if args.start_cmd:
                # autostart v3（flight-link only）模式下定位栈需手动启动：
                # 体检发现未跑时用 --start-cmd 提供的命令一键拉起并复检
                print(f"         [start-cmd] {label} 未运行，执行拉起：{args.start_cmd}")
                run(host, user, pw, port, args.start_cmd, timeout=90)
                time.sleep(10)
                out2 = run(host, user, pw, port,
                           f"pgrep -xc {proc} 2>/dev/null || echo 0")
                n2 = int(out2.strip().splitlines()[-1] or 0)
                if n2 == expect:
                    print(f"  [OK ] {label} ({proc}): 拉起成功，{n2} 个")
                else:
                    print(f"  [FAIL] {label} ({proc}): 拉起后仍 {n2} 个")
                    problems.append(f"{label} 拉起失败（start-cmd 无效）")
            else:
                problems.append(f"{label} 未运行（autostart v3 模式不自启，"
                                "可加 --start-cmd 'bash ~/start_drone.sh' 一键拉起）")

    # ---- 2) 话题发布者数 ----
    print()
    print("=" * 62)
    print("[2/4] 话题发布者数检查（应恰好 1 —— 双发布者=双实例打架）")
    print("=" * 62)
    for label, topic in TOPIC_CHECKS:
        out = run(host, user, pw, port,
                  f"{REMOTE_ENV} ros2 topic info {topic} 2>/dev/null | grep 'Publisher count'")
        m = re.search(r"Publisher count:\s*(\d+)", out)
        n = int(m.group(1)) if m else -1
        if n == 1:
            print(f"  [OK ] {label}: 1 个发布者")
        elif n < 0:
            print(f"  [FAIL] {label}: 话题无响应（栈未启动或 DDS 不通）")
            problems.append(f"{topic} 话题无响应（若 Agent 在跑但 /fmu/* 全无数据，"
                            "跑 tools/uxrce_link_guard.py 区分：飞控侧卡死需断电复位）")
        else:
            print(f"  [FAIL] {label}: {n} 个发布者 —— 双实例正在交错发布！")
            problems.append(f"{topic} 有 {n} 个发布者")
            if args.kill_dup:
                print("         [kill-dup] 话题级双发布者需人工定位进程（上方进程检查已处理同名多实例）")

    # ---- 3) EKF reset 增长 ----
    print()
    print("=" * 62)
    print("[3/4] EKF xy_reset_counter 静置 10s 增长（应 <=2）")
    print("=" * 62)
    out = run(host, user, pw, port,
              f"{REMOTE_ENV} "
              "A=$(timeout 6 ros2 topic echo --once /fmu/out/vehicle_local_position --field xy_reset_counter 2>/dev/null); "
              "sleep 10; "
              "B=$(timeout 6 ros2 topic echo --once /fmu/out/vehicle_local_position --field xy_reset_counter 2>/dev/null); "
              "echo \"$A $B\"", timeout=90)
    m = re.search(r"(\d+)\s+(\d+)\s*$", out.strip())
    if m:
        a, b = int(m.group(1)), int(m.group(2))
        growth = b - a
        status = "OK " if growth <= 2 else "FAIL"
        print(f"  [{status}] reset_counter: {a} -> {b}（10s 增长 {growth}）")
        if growth > 2:
            problems.append(f"EKF reset 风暴持续中（10s +{growth}）")
            print("         -> 上游 SLAM 输出异常或双实例；结合 [1][2] 结果与节点 INSANE/STORM 告警定位")
    else:
        print(f"  [WARN] 无法采样 reset_counter（输出: {out.strip()[:60]}）——飞控链路可能未通")

    # ---- 4) 最近告警 ----
    print()
    print("=" * 62)
    print("[4/4] 最近 5 分钟节点告警扫描")
    print("=" * 62)
    out = run(host, user, pw, port,
              "F=$(ls -t ~/drone_logs/node_odom_*.log 2>/dev/null | head -1); "
              "echo \"log: $F\"; "
              "find ~/drone_logs -name 'node_odom_*.log' -mmin -5 2>/dev/null | "
              "xargs grep -h 'DUPLICATE\\|INSANE\\|STORM' 2>/dev/null | tail -5; "
              "echo SCAN_DONE")
    lines = [l for l in out.splitlines() if l.strip() and l.strip() != "SCAN_DONE"]
    alarm_lines = [l for l in lines if "DUPLICATE" in l or "INSANE" in l or "STORM" in l]
    if alarm_lines:
        print(f"  [FAIL] 最近 5 分钟有 {len(alarm_lines)} 条防护告警：")
        for l in alarm_lines[-5:]:
            print(f"         {l.strip()[:130]}")
        problems.append(f"最近告警 {len(alarm_lines)} 条（DUPLICATE/INSANE/STORM）")
    else:
        print("  [OK ] 最近 5 分钟无 DUPLICATE/INSANE/STORM 告警")
        for l in lines[:1]:
            print(f"         {l.strip()[:100]}")

    # ---- 汇总 ----
    print()
    print("=" * 62)
    if problems:
        print(f"[体检结论] FAIL —— {len(problems)} 项问题：")
        for p in problems:
            print(f"  * {p}")
        print("处理建议：")
        print("  * 多实例：确认要保留的栈（tmux agup 或手动），kill 其余；或加 --kill-dup 自动清理同名多实例")
        print("  * reset 风暴：单实例仍风暴 -> 停机位置远离建图原点，SLAM 无法配准，")
        print("    挪机或重开 ICP auto-init 后复测")
        return 1
    print("[体检结论] PASS —— 定位栈健康，可以起飞（仍需 30s /fmu/in/vehicle_visual_odometry 43Hz 自检）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
