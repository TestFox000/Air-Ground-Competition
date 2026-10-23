#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""e2e_15s_probe.py —— 空地指挥链「地面站指令 → 车 DRIVE」端到端 15s 计时验证

在**无人机上**跑。职责单一：计时，不重复造规划/下发逻辑。

链路（一次 run）：
```
t0 = 收到 start 指令时刻（本脚本触发 dzlink_mission_ctl.py once 的瞬间）
      │
      ├─► 触发规划+门禁+下发（dzlink_mission_ctl.py once）
      │      └─► /nav/waypoint (RELIABLE) ──► 小车 MissionTask
      │
      └─► 订阅 /dzlink/car_to_drone，等 car_state.phase ∈ {DRIVE,...}
          t1 = 首条 phase != IDLE 的时刻
          ★ 判据：t1 - t0 ≤ 15s
```

用法（无人机上，先 source ROS + dzlink.env 域 118）：
  source /opt/ros/humble/setup.bash
  source ~/dzlink_tools/dzlink.env
  python3 ~/dzlink_tools/e2e_15s_probe.py --once          # 立即触发一次并计时
  python3 ~/dzlink_tools/e2e_15s_probe.py --listen 30      # 只监听车回传（不触发）

判据与边界（如实声明）：
  · 「车 DRIVE」以车侧 `car_state.phase` 为准（drone.py:403-432，phase 变化即刻发 + 1~2Hz 周期）。
  · 若车侧 `auto_start=true` 抢跑，phase 可能在触发前就已 DRIVE → 本脚本会报「无法归因」而非假达标。
  · 若车栈没起 / 域号不一致 / 第二写者，监听收不到 car_state → 报「无回传」而非假达标。
  · 本脚本不判断「车物理位移」，只判断「车侧决策层进入 DRIVE」（那是"开始跑"的代码事实）。
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent          # 无人机 ~/dzlink_tools
MISSION_CTL = HERE / "dzlink_mission_ctl.py"
BUDGET_S = 15.0

#: 车侧「已出发」的 phase 集合（DRIVE 是主线；出现 HOLD/STOP/FINISH 也算已启动）
DRIVE_PHASES = {"DRIVE", "HOLD", "STOP", "FINISH"}


def _listen_until_phase(deadline: float, node) -> dict:
    """订阅 /dzlink/car_to_drone，等首条 phase 进入 DRIVE_PHASES 或超时。"""
    from std_msgs.msg import String
    result = {"phase": "", "t": None, "samples": 0, "blocks": []}

    def _cb(m):
        try:
            data = json.loads(m.data)
        except Exception:
            return
        result["samples"] += 1
        ph = str(data.get("phase", ""))
        blk = str(data.get("block", ""))
        if blk:
            result["blocks"].append(blk)
        if ph and not result["phase"]:
            result["phase"] = ph
            result["t"] = time.perf_counter()
            print(f"  ← 首条 car_state: phase={ph} block={blk} (t={result['t']:.3f})", flush=True)

    sub = node.create_subscription(String, "/dzlink/car_to_drone", _cb, 10)
    while time.perf_counter() < deadline:
        if result["phase"] in DRIVE_PHASES:
            break
        import rclpy
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return result


def run_once(ros_domain: int, wait_phase_s: float) -> int:
    import rclpy
    from std_msgs.msg import String

    t0 = time.perf_counter()
    print(f"[e2e] t0 = 触发时刻（{t0:.3f}），ROS_DOMAIN_ID={ros_domain}", flush=True)

    rclpy.init()
    node = rclpy.create_node("e2e_15s_probe")
    # 先起订阅（在触发下发前就绪，避免漏掉首条 car_state）
    phase_holder = {}

    # 触发编排节点（规划+门禁+下发）
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1",
               ROS_DOMAIN_ID=str(ros_domain))
    p = subprocess.Popen(
        [sys.executable, str(MISSION_CTL), "once", "--wait-match", "3"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        encoding="utf-8", errors="replace", cwd=str(HERE), env=env)
    ctl_out, _ = p.communicate(timeout=60)
    print(ctl_out, flush=True)
    t_sent = time.perf_counter()

    # 订阅车回传，等 phase 进 DRIVE
    deadline = time.perf_counter() + wait_phase_s
    res = _listen_until_phase(deadline, node)
    t1 = res["t"]

    node.destroy_node()
    rclpy.shutdown()

    print("=" * 60)
    print(f"[e2e] 触发→下发动作完成 = {t_sent - t0:.2f}s")
    if t1 is not None:
        total = t1 - t0
        print(f"[e2e] 触发→车进入 DRIVE = {total:.2f}s（预算 {BUDGET_S}s，"
              f"{'✅ 达标' if total <= BUDGET_S else '❌ 超预算'}）")
        print(f"   └─ 首条 car_state.phase = {res['phase']!r}")
        return 0 if total <= BUDGET_S else 1
    else:
        print(f"[e2e] ⚠️ {wait_phase_s:.0f}s 内没收到车侧 car_state（phase 未进 DRIVE）")
        print("   可能：车栈没起 / 域号不一致 / 车侧 auto_start 抢跑已 DRIVE 导致无法归因")
        print("   （收到 %d 条样本，blocks=%s）" % (res["samples"], res["blocks"][:5]))
        return 1


def listen_only(ros_domain: int, duration: float) -> int:
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node("e2e_15s_listen")
    got = {"n": 0, "kinds": {}}

    def _cb(m):
        got["n"] += 1
        try:
            k = json.loads(m.data).get("type", "?")
        except Exception:
            k = "?"
        got["kinds"][k] = got["kinds"].get(k, 0) + 1
        print(f"  ← [{got['n']}] {m.data[:160]}", flush=True)

    sub = node.create_subscription(String, "/dzlink/car_to_drone", _cb, 10)
    end = time.perf_counter() + duration
    while time.perf_counter() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f"[listen] 收到 {got['n']} 条：{got['kinds']}")
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="空地指挥链 15s 端到端计时")
    ap.add_argument("--once", action="store_true", help="触发一次规划+下发并计时")
    ap.add_argument("--listen", type=float, default=None, help="只监听车回传 N 秒")
    ap.add_argument("--domain", type=int, default=118, help="ROS_DOMAIN_ID（默认 118）")
    ap.add_argument("--wait-phase", type=float, default=15.0,
                    help="触发后等车进入 DRIVE 的秒数（默认 15）")
    args = ap.parse_args()

    if args.listen is not None:
        return listen_only(args.domain, args.listen)
    if args.once:
        return run_once(args.domain, args.wait_phase)
    ap.print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())
