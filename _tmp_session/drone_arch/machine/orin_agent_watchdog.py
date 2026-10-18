#!/usr/bin/env python3
"""飞控链路看门狗：飞控每次断电重上电后，自动重建 uXRCE 会话，全程无人操作。

为什么需要它（2026-10-16 实机踩坑）：
  * NxtPX4 飞控 USB 断电重上电后，串口 /dev/ttyACMx 会重新枚举；
  * MicroXRCEAgent 持有的旧串口句柄随之失效（进程还活着，但再也收不到
    飞控 uXRCE 流量），PX4 端客户端干等不重连——遥测全部消失；
  * 唯一恢复手段是重启 Agent 进程。人工每次 tmux 重启窗口繁琐易忘，
    于是本脚本常驻守护：发现「串口在 + 遥测静默」就自动重启 Agent。

状态机（每 --check-every 秒评估一次）：
  设备(ttyACM0)不存在      -> OFFLINE：静默等待飞控上电（绝不空转重启）
  设备在 + 遥测正常(<15s)   -> ONLINE：什么都不做
  设备在 + 遥测静默(>15s)   -> 会话死：pkill 旧 Agent -> setsid 拉起新 Agent
  设备在 + Agent 进程死了    -> 直接拉起新 Agent
  重启后 --cooldown 秒内      -> 冷却：不重复重启（给 PX4 客户端连接时间）

用法（Orin 上，一般由 tmux 窗口 wdog 常驻；也可手动前台跑观察）:
    python3 orin_agent_watchdog.py                    # 默认 ttyACM0
    python3 orin_agent_watchdog.py --device /dev/ttyACM1
    python3 orin_agent_watchdog.py --restart-once     # 一键恢复模式（不守护，干完就退）

日志: ~/drone_logs/agent_watchdog.log（每次重启留痕）+ 每 30s 健康行。
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import time
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
except ImportError:
    raise SystemExit("缺少 rclpy：需要在 Orin 的 ROS2 环境里运行")
try:
    from px4_msgs.msg import VehicleLocalPosition
except ImportError:
    raise SystemExit("缺 px4_msgs：需 source ~/dzpx4/install/setup.bash（看门狗依赖遥测话题判活）")

LOG_DIR = Path("~/drone_logs").expanduser()
WATCH_LOG = LOG_DIR / "agent_watchdog.log"
AGENT_LOG = LOG_DIR / "agent_watchdog_agent.log"

HEARTBEAT_TOPIC = "/fmu/out/vehicle_local_position"  # 飞控遥测的"心跳"：有它=会话活


def log(msg: str) -> None:
    line = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] [watchdog] {msg}"
    print(line, flush=True)
    try:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        with open(WATCH_LOG, "a", encoding="utf-8") as f:
            f.write(line + "\n")
    except OSError:
        pass


def device_alive(dev: str) -> bool:
    return os.path.exists(dev)


def agent_alive() -> bool:
    r = subprocess.run(["pgrep", "-f", "MicroXRCEAgent serial"],
                       capture_output=True)
    return r.returncode == 0


def restart_agent(dev: str) -> None:
    """杀旧起新：新 Agent 用 setsid 后台 + 日志续写（与手工恢复流程一致）。"""
    subprocess.run(["pkill", "-f", "MicroXRCEAgent serial"], capture_output=True)
    time.sleep(1.0)
    # 不经 tmux（避免嵌套引号/窗口生命周期耦合），setsid 脱离守护进程生命周期
    cmd = (f"setsid MicroXRCEAgent serial --dev {dev} -b 921600 "
           f">> {AGENT_LOG} 2>&1")
    subprocess.Popen(["bash", "-lc", cmd],
                     start_new_session=True,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    log(f"Agent 已重启（{dev}），旧日志见 {AGENT_LOG}")


class Watchdog(Node):
    def __init__(self, args):
        super().__init__("fmu_agent_watchdog")
        self.args = args
        self.last_msg = time.monotonic()      # 启动即计时：飞控若已上电但会话死，15s 内自动修
        self.last_restart = 0.0
        self.restarts = 0
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        # 消息不做任何解析，只当"飞控会话活着"的信号
        self.create_subscription(VehicleLocalPosition, HEARTBEAT_TOPIC,
                                 self._on_msg, qos)
        self.last_hb = 0.0

    def _on_msg(self, _msg) -> None:
        self.last_msg = time.monotonic()

    def spin_loop(self, stopping: dict) -> None:
        while rclpy.ok() and not stopping["flag"]:
            rclpy.spin_once(self, timeout_sec=self.args.check_every)
            now = time.monotonic()
            silent = now - self.last_msg
            if now - self.last_hb >= 30.0:
                self.last_hb = now
                dev = "在" if device_alive(self.args.device) else "无"
                ag = "活" if agent_alive() else "死"
                log(f"健康: 串口{dev} Agent{ag} 遥测静默{silent:.0f}s 重启{self.restarts}次")
            if not device_alive(self.args.device):
                continue  # 飞控没上电：等待，不动 Agent
            if now - self.last_restart < self.args.cooldown:
                continue  # 重启冷却期：给 PX4 客户端连接时间
            if not agent_alive():
                log("Agent 进程消失，直接拉起")
                self._do_restart()
            elif silent > self.args.silence_sec:
                log(f"遥测静默 {silent:.0f}s（会话死），重启 Agent 重建会话")
                self._do_restart()

    def _do_restart(self) -> None:
        restart_agent(self.args.device)
        self.restarts += 1
        self.last_restart = time.monotonic()
        self.last_msg = time.monotonic()  # 重置计时，进入新一轮静默观察


def main() -> int:
    ap = argparse.ArgumentParser(description="飞控 uXRCE 链路看门狗（会话死自动重启 Agent）")
    ap.add_argument("--device", default="/dev/ttyACM0", help="飞控串口（默认 /dev/ttyACM0）")
    ap.add_argument("--silence-sec", type=float, default=15.0,
                    help="遥测静默多少秒判定会话死（默认 15）")
    ap.add_argument("--cooldown", type=float, default=60.0,
                    help="一次重启后的冷却期秒数（默认 60）")
    ap.add_argument("--check-every", type=float, default=2.0,
                    help="巡检间隔秒（默认 2）")
    ap.add_argument("--restart-once", action="store_true",
                    help="一键恢复模式：立即重启 Agent 后退出（不常驻守护）")
    args = ap.parse_args()

    if args.restart_once:
        log("一键恢复：重启 Agent 后退出")
        restart_agent(args.device)
        return 0

    stopping = {"flag": False}

    def on_sig(_s, _f):
        stopping["flag"] = True

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    rclpy.init()
    node = Watchdog(args)
    log(f"看门狗启动: device={args.device} silence={args.silence_sec}s "
        f"cooldown={args.cooldown}s（飞控重上电后会话静默即自动恢复，无需人工）")
    try:
        node.spin_loop(stopping)
    finally:
        log("看门狗退出")
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
