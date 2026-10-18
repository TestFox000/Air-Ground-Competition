#!/usr/bin/env python3
"""飞控链路看门狗：飞控每次断电重上电后，自动重建 uXRCE 会话；会话救不回来时停止乱重启。

为什么需要它（2026-10-16 实机踩坑）：
  * NxtPX4 飞控 USB 断电重上电后，串口 /dev/ttyACMx 会重新枚举；
  * MicroXRCEAgent 持有的旧串口句柄随之失效（进程还活着，但再也收不到
    飞控 uXRCE 流量），PX4 端客户端干等不重连——遥测全部消失；
  * 唯一恢复手段是重启 Agent 进程；若 PX4 侧也卡死，则必须给飞控断电。

为什么要 FC_OFFLINE（2026-10-17 实机踩坑，本补丁新增）：
  * Orin 重启而飞控一直带电时，PX4 的 uxrce_dds_client 不会自己重连；
  * 旧版看门狗每 61 s 杀一次 Agent 重启，永远救不回来——
    日志里刷的是"健康: 串口在 Agent活 遥测静默 60s"，看起来像在工作，
    实际是无效重启，还会把串口/Owner 状态反复搅乱；
  * 新版：连续 --max-dead-rounds(3) 轮重启后仍 0 帧 ⇒ 状态 FC_OFFLINE，
    停止重启，只按 --offline-warn-period(300 s) 告警"需给飞控断电重启"。

状态机（每 --check-every 秒评估一次）：
  设备(ttyACM0)不存在          -> DEVICE_OFFLINE：静默等待飞控上电（绝不空转重启）
  设备在 + 遥测正常(<silence)  -> ONLINE：什么都不做
  设备在 + 遥测静默(>silence)  -> RECOVERING：pkill 旧 Agent -> setsid 拉起新 Agent
  同一轮重启后 --cooldown 秒内  -> 冷却：给 PX4 客户端连接时间，不重复重启
  Agent 进程死了               -> 直接拉起（同样计入一轮）
  连续 N 轮重启后仍 0 帧        -> FC_OFFLINE：停止重启 + 降频告警（需人工断电）
  FC_OFFLINE 下设备消失又出现   -> 视为已给飞控断电重启 -> 复位，重新允许自动恢复
  FC_OFFLINE 下收到遥测         -> 自动解除，回到 ONLINE

用法（Orin 上；一般由 tmux 窗口 wdog 常驻，也可手动前台跑观察）:
    python3 orin_agent_watchdog.py                    # 默认 ttyACM0
    python3 orin_agent_watchdog.py --device /dev/ttyACM1
    python3 orin_agent_watchdog.py --max-dead-rounds 3 --offline-warn-period 300
    python3 orin_agent_watchdog.py --restart-once     # 一键恢复模式（不守护，干完就退）

日志: ~/drone_logs/agent_watchdog.log
      + 状态化健康行（ONLINE/RECOVERING 每 --health-period 秒；FC_OFFLINE/DEVICE_OFFLINE 每 --offline-warn-period 秒）

域号（2026-10-17 补）：拉起 Agent 时显式转发本进程的 ROS_DOMAIN_ID，避免 cron / 非登录 shell
环境下 Agent 落到域 0 而 PX4 在域 118 造成"永远无遥测"的假故障。
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

# ---- 状态常量（纯字符串，便于日志直接读） ----
ONLINE = "ONLINE"
RECOVERING = "RECOVERING"
FC_OFFLINE = "FC_OFFLINE"
DEVICE_OFFLINE = "DEVICE_OFFLINE"

FC_OFFLINE_HINT = (
    "需要给飞控断电重启（拔电池/断 USB 供电 5 秒后重新上电），"
    "等串口重新枚举后再重启 Agent；Orin 单侧重启救不回来"
)


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


def restart_agent(dev: str, domain: str = "") -> None:
    """杀旧起新：新 Agent 用 setsid 后台 + 日志续写（与手工恢复流程一致）。

    domain 非空时显式前置 ROS_DOMAIN_ID=...，保证 Agent 的 DDS 参与者与看门狗
    的遥测订阅、与 PX4 的 UXRCE_DDS_DOM_ID 落在同一域（2026-10-17 补）。
    """
    subprocess.run(["pkill", "-f", "MicroXRCEAgent serial"], capture_output=True)
    time.sleep(1.0)
    env_prefix = f"ROS_DOMAIN_ID={domain} " if domain else ""
    # 不经 tmux（避免嵌套引号/窗口生命周期耦合），setsid 脱离守护进程生命周期
    cmd = (f"{env_prefix}setsid MicroXRCEAgent serial --dev {dev} -b 921600 "
           f">> {AGENT_LOG} 2>&1")
    subprocess.Popen(["bash", "-lc", cmd],
                     start_new_session=True,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    log(f"Agent 已重启（{dev}，ROS_DOMAIN_ID={domain or '<继承>'}），旧日志见 {AGENT_LOG}")


class SessionTracker:
    """纯状态机：不依赖 ROS、不读墙钟，便于本地回放自测。"""

    def __init__(self, max_dead_rounds: int) -> None:
        self.max_dead_rounds = max(0, int(max_dead_rounds))
        self.state = RECOVERING
        self.dead_rounds = 0              # 连续"重启后仍无遥测"的轮数
        self.telemetry_since_restart = True

    # --- 事件入口，返回需要写日志的文本列表 ---
    def device_missing(self) -> list[str]:
        if self.state == DEVICE_OFFLINE:
            return []
        self.state = DEVICE_OFFLINE
        self.dead_rounds = 0
        self.telemetry_since_restart = True
        return ["设备不存在 → 状态 DEVICE_OFFLINE（等待飞控上电，不做任何重启）"]

    def device_back(self) -> list[str]:
        if self.state != DEVICE_OFFLINE:
            return []
        self.state = RECOVERING
        self.dead_rounds = 0
        self.telemetry_since_restart = True
        return ["设备重新出现 → 状态 RECOVERING（飞控可能已断电重启，重新允许自动恢复）"]

    def telemetry(self) -> list[str]:
        msgs: list[str] = []
        if self.state == FC_OFFLINE:
            msgs.append("FC_OFFLINE 解除：收到飞控遥测，会话已恢复（断电重启生效）")
        elif self.dead_rounds > 0:
            msgs.append(f"会话已恢复：收到飞控遥测（此前连续 {self.dead_rounds} 轮无遥测）")
        self.state = ONLINE
        self.dead_rounds = 0
        self.telemetry_since_restart = True
        return msgs

    def restart_done(self) -> int:
        self.dead_rounds += 1
        self.telemetry_since_restart = False
        if self.state != FC_OFFLINE:
            self.state = RECOVERING
        return self.dead_rounds

    def should_give_up(self) -> bool:
        return (self.max_dead_rounds > 0
                and not self.telemetry_since_restart
                and self.dead_rounds >= self.max_dead_rounds)

    def enter_fc_offline(self) -> str:
        self.state = FC_OFFLINE
        return (f"连续 {self.dead_rounds} 轮重启后遥测仍为 0 帧 → 状态 FC_OFFLINE，"
                f"停止自动重启；{FC_OFFLINE_HINT}")


class Watchdog(Node):
    def __init__(self, args):
        super().__init__("fmu_agent_watchdog")
        self.args = args
        self.tracker = SessionTracker(args.max_dead_rounds)
        self.domain = os.environ.get("ROS_DOMAIN_ID", "")
        self.last_msg = time.monotonic()      # 启动即计时：飞控若已上电但会话死，silence 后修一次
        self.last_restart = 0.0
        self.restarts = 0
        self.last_hb = 0.0
        self.last_warn = 0.0
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        # 消息不做任何解析，只当"飞控会话活着"的信号
        self.create_subscription(VehicleLocalPosition, HEARTBEAT_TOPIC,
                                 self._on_msg, qos)

    def _on_msg(self, _msg) -> None:
        self.last_msg = time.monotonic()
        for line in self.tracker.telemetry():
            log(line)

    # ---- 纯决策：显式传入时间与三个布尔量，便于自测 ----
    def _tick(self, now: float, silent: float,
              device_ok: bool, agent_ok: bool) -> None:
        tr = self.tracker
        if not device_ok:
            for line in tr.device_missing():
                log(line)
            self._maybe_warn(
                now, f"状态=DEVICE_OFFLINE 串口 {self.args.device} 未出现；"
                     f"等待飞控上电（断电重启后设备会重新枚举）")
            return

        for line in tr.device_back():
            log(line)
            self.last_msg = now            # 新设备给一个 silence 宽限
            self.last_restart = 0.0        # 解除冷却，允许立刻恢复

        if tr.state == FC_OFFLINE:
            self._maybe_warn(
                now, f"状态=FC_OFFLINE 已连续 {tr.dead_rounds} 轮重启仍无遥测；"
                     f"{FC_OFFLINE_HINT}")
            return

        if now - self.last_restart < self.args.cooldown:
            self._maybe_health(now, silent)
            return

        if not agent_ok:
            log("Agent 进程消失，直接拉起")
            self._do_restart(now)
            return

        if silent > self.args.silence_sec:
            if tr.should_give_up():
                log(tr.enter_fc_offline())
                self.last_warn = now
                return
            total = tr.max_dead_rounds if tr.max_dead_rounds > 0 else "∞"
            log(f"遥测静默 {silent:.0f}s（会话死），重启 Agent 重建会话"
                f"（第 {tr.dead_rounds + 1}/{total} 轮）")
            self._do_restart(now)
            return

        self._maybe_health(now, silent)

    def _do_restart(self, now: float) -> None:
        restart_agent(self.args.device, self.domain)
        self.restarts += 1
        self.tracker.restart_done()
        self.last_restart = now
        self.last_msg = now               # 重置计时，进入新一轮静默观察
        self.last_warn = now

    def _maybe_health(self, now: float, silent: float) -> None:
        if now - self.last_hb < self.args.health_period:
            return
        self.last_hb = now
        log(f"健康: 状态={self.tracker.state} 串口在 Agent活 "
            f"遥测静默{silent:.0f}s 重启{self.restarts}次 "
            f"连续无遥测{self.tracker.dead_rounds}轮 域={self.domain or '<未设=0>'}")

    def _maybe_warn(self, now: float, msg: str) -> None:
        if now - self.last_warn < self.args.offline_warn_period:
            return
        self.last_warn = now
        log(f"[需人工] {msg}")

    def spin_loop(self, stopping: dict) -> None:
        while rclpy.ok() and not stopping["flag"]:
            rclpy.spin_once(self, timeout_sec=self.args.check_every)
            now = time.monotonic()
            self._tick(now, now - self.last_msg,
                       device_alive(self.args.device), agent_alive())


def main() -> int:
    ap = argparse.ArgumentParser(description="飞控 uXRCE 链路看门狗（会话死自动重启 Agent，连续失败转 FC_OFFLINE）")
    ap.add_argument("--device", default="/dev/ttyACM0", help="飞控串口（默认 /dev/ttyACM0）")
    ap.add_argument("--silence-sec", type=float, default=15.0,
                    help="遥测静默多少秒判定会话死（默认 15）")
    ap.add_argument("--cooldown", type=float, default=60.0,
                    help="一次重启后的冷却期秒数（默认 60）")
    ap.add_argument("--check-every", type=float, default=2.0,
                    help="巡检间隔秒（默认 2）")
    ap.add_argument("--max-dead-rounds", type=int, default=3,
                    help="连续多少轮重启后仍 0 帧即转 FC_OFFLINE 并停止重启（默认 3；0=永不放弃，旧行为）")
    ap.add_argument("--offline-warn-period", type=float, default=300.0,
                    help="FC_OFFLINE/DEVICE_OFFLINE 下的告警间隔秒（默认 300）")
    ap.add_argument("--health-period", type=float, default=30.0,
                    help="ONLINE/RECOVERING 健康行间隔秒（默认 30）")
    ap.add_argument("--restart-once", action="store_true",
                    help="一键恢复模式：立即重启 Agent 后退出（不常驻守护）")
    args = ap.parse_args()

    if args.restart_once:
        log("一键恢复：重启 Agent 后退出")
        restart_agent(args.device, os.environ.get("ROS_DOMAIN_ID", ""))
        return 0

    stopping = {"flag": False}

    def on_sig(_s, _f):
        stopping["flag"] = True

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    rclpy.init()
    node = Watchdog(args)
    log(f"看门狗启动: device={args.device} silence={args.silence_sec}s "
        f"cooldown={args.cooldown}s max_dead_rounds={args.max_dead_rounds} "
        f"offline_warn={args.offline_warn_period}s")
    if node.domain:
        log(f"域号: ROS_DOMAIN_ID={node.domain}（遥测订阅与新 Agent 同域）")
    else:
        log("警告: ROS_DOMAIN_ID 未设置，本进程落在域 0。若 PX4 的 UXRCE_DDS_DOM_ID 已改 118，"
            "看门狗将永远收不到遥测并误判会话死；请在 autostart_agup.sh 里 export ROS_DOMAIN_ID=118")
    try:
        node.spin_loop(stopping)
    finally:
        log("看门狗退出")
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
