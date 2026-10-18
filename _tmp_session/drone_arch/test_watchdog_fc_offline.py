#!/usr/bin/env python3
"""看门狗 FC_OFFLINE 状态机回放自测（无需 ROS / 不碰真实串口）。

做法：在 import 补丁脚本前注入假的 rclpy / px4_msgs 模块，再用显式时间戳
驱动 Watchdog._tick()，验证状态迁移、重启次数上限、告警降频。

运行：
    python3 test_watchdog_fc_offline.py
退出码 0 = 全部通过。
"""
from __future__ import annotations

import importlib.util
import os
import sys
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
TARGET = HERE / "orin_agent_watchdog_fc_offline.py"

# ---------- 1. 注入假 ROS 模块 ----------
rclpy = types.ModuleType("rclpy")
rclpy.ok = lambda: True
rclpy.init = lambda *a, **k: None
rclpy.try_shutdown = lambda *a, **k: None
rclpy.spin_once = lambda *a, **k: None

node_mod = types.ModuleType("rclpy.node")


class _Node:
    def __init__(self, name: str = "") -> None:
        self.node_name = name

    def create_subscription(self, *a, **k):
        self.subscription_args = (a, k)
        return None


node_mod.Node = _Node

qos_mod = types.ModuleType("rclpy.qos")


class _QoSProfile:
    def __init__(self, depth: int = 10, reliability=None) -> None:
        self.depth = depth
        self.reliability = reliability


class _ReliabilityPolicy:
    BEST_EFFORT = "best_effort"
    RELIABLE = "reliable"


qos_mod.QoSProfile = _QoSProfile
qos_mod.ReliabilityPolicy = _ReliabilityPolicy

px4_msgs = types.ModuleType("px4_msgs")
px4_msg = types.ModuleType("px4_msgs.msg")
px4_msg.VehicleLocalPosition = type("VehicleLocalPosition", (), {})

rclpy.node = node_mod
rclpy.qos = qos_mod
px4_msgs.msg = px4_msg
sys.modules.setdefault("rclpy", rclpy)
sys.modules.setdefault("rclpy.node", node_mod)
sys.modules.setdefault("rclpy.qos", qos_mod)
sys.modules.setdefault("px4_msgs", px4_msgs)
sys.modules.setdefault("px4_msgs.msg", px4_msg)

# ---------- 2. 导入被测脚本 ----------
spec = importlib.util.spec_from_file_location("wd_under_test", TARGET)
assert spec and spec.loader, "cannot load target"
wd = importlib.util.module_from_spec(spec)
sys.modules["wd_under_test"] = wd
spec.loader.exec_module(wd)

# ---------- 3. 替身：日志与 Agent 重启 ----------
LOG: list[str] = []
RESTARTS: list[tuple[float, str]] = []


def fake_log(msg: str) -> None:
    LOG.append(msg)


def fake_restart(dev: str, domain: str = "") -> None:
    RESTARTS.append((dev, domain))


wd.log = fake_log
wd.restart_agent = fake_restart

FAILS: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> None:
    print(f"{'PASS' if cond else 'FAIL'}  {name}" + (f"  [{detail}]" if detail and not cond else ""))
    if not cond:
        FAILS.append(name)


def new_node(max_dead_rounds: int = 3):
    args = types.SimpleNamespace(
        device="/dev/ttyACM0",
        silence_sec=15.0,
        cooldown=60.0,
        check_every=2.0,
        max_dead_rounds=max_dead_rounds,
        offline_warn_period=300.0,
        health_period=30.0,
        restart_once=False,
    )
    LOG.clear()
    RESTARTS.clear()
    return wd.Watchdog(args)


def replay(node, t0: float, seconds: float, device_ok=True, agent_ok=True):
    t = t0
    end = t0 + seconds
    while t < end:
        node._tick(t, t - node.last_msg, device_ok, agent_ok)
        t += 2.0
    return t


# ================= 场景 A：会话死，3 轮后进 FC_OFFLINE 并停止重启 =================
node = new_node(3)
t = 1000.0
node.last_msg = t
node.last_restart = 0.0
t = replay(node, t, 900.0)

check("A1 重启次数=3（达到上限后不再重启）", len(RESTARTS) == 3, f"restarts={len(RESTARTS)}")
check("A2 状态=FC_OFFLINE", node.tracker.state == wd.FC_OFFLINE, node.tracker.state)
check("A3 进入 FC_OFFLINE 有明确日志", any("FC_OFFLINE，停止自动重启" in x for x in LOG))
check("A4 日志提示需给飞控断电", any("断电重启" in x for x in LOG))
fake_online = [x for x in LOG if "健康: 状态=ONLINE" in x]
check("A5 FC_OFFLINE 期间不刷 ONLINE 假健康行", len(fake_online) <= 1, f"{len(fake_online)} 条")
warns = [x for x in LOG if "[需人工]" in x]
check("A6 FC_OFFLINE 告警降频（900s 内 <=4 条）", len(warns) <= 4, f"{len(warns)} 条")

# ================= 场景 B：FC_OFFLINE 下收到遥测 → 自动解除 =================
node._on_msg(None)
check("B1 收到遥测后回到 ONLINE", node.tracker.state == wd.ONLINE, node.tracker.state)
check("B2 计数器清零", node.tracker.dead_rounds == 0)
check("B3 有恢复日志", any("FC_OFFLINE 解除" in x for x in LOG))

# ================= 场景 C：设备消失 → DEVICE_OFFLINE；重新出现 → 复位 =================
node2 = new_node(3)
t = 2000.0
node2.last_msg = t
node2.last_restart = 0.0
node2._tick(t, 999.0, False, True)
check("C1 设备不在 → DEVICE_OFFLINE", node2.tracker.state == wd.DEVICE_OFFLINE, node2.tracker.state)
check("C2 设备不在不重启", len(RESTARTS) == 0, f"restarts={len(RESTARTS)}")
t = replay(node2, t, 120.0, device_ok=False)
check("C3 长时间无设备仍不重启", len(RESTARTS) == 0, f"restarts={len(RESTARTS)}")
node2._tick(t, 0.0, True, True)
check("C4 设备回来 → RECOVERING", node2.tracker.state == wd.RECOVERING, node2.tracker.state)
t = replay(node2, t, 300.0)
check("C5 设备回来后恢复自动重启", len(RESTARTS) >= 1, f"restarts={len(RESTARTS)}")

# ================= 场景 D：max_dead_rounds=0 保持旧行为（永不放弃） =================
node3 = new_node(0)
t = 3000.0
node3.last_msg = t
node3.last_restart = 0.0
t = replay(node3, t, 600.0)
check("D1 关闭上限后持续重启（>3 次）", len(RESTARTS) > 3, f"restarts={len(RESTARTS)}")
check("D2 状态不会进 FC_OFFLINE", node3.tracker.state != wd.FC_OFFLINE, node3.tracker.state)

# ================= 场景 E：遥测正常 → 不重启 =================
node4 = new_node(3)
t = 4000.0
node4.last_msg = t
node4.last_restart = 0.0
for i in range(300):                     # 600 s，每 2 s 一帧遥测
    t += 2.0
    node4._on_msg(None)                  # 遥测必须走 _on_msg（真实路径），状态下 ONLINE
    node4._tick(t, 0.5, True, True)
check("E1 遥测正常时不重启", len(RESTARTS) == 0, f"restarts={len(RESTARTS)}")
check("E2 状态保持 ONLINE", node4.tracker.state == wd.ONLINE, node4.tracker.state)

# ================= 场景 F：Agent 进程消失 → 立即拉起 =================
node5 = new_node(3)
t = 5000.0
node5.last_msg = t - 60.0
node5.last_restart = 0.0
node5._tick(t, 60.0, True, False)
check("F1 Agent 死 → 重启一次", len(RESTARTS) == 1, f"restarts={len(RESTARTS)}")
check("F2 重启计数=1", node5.tracker.dead_rounds == 1, str(node5.tracker.dead_rounds))

# ================= 场景 G：域号透传 =================
wd_args = types.SimpleNamespace(device="/dev/ttyACM0", silence_sec=15.0, cooldown=60.0,
                                check_every=2.0, max_dead_rounds=3,
                                offline_warn_period=300.0, health_period=30.0,
                                restart_once=False)
os.environ["ROS_DOMAIN_ID"] = "118"
node6 = wd.Watchdog(wd_args)
check("G1 读取到域号 118", node6.domain == "118", node6.domain)
os.environ.pop("ROS_DOMAIN_ID", None)

print()
if FAILS:
    print(f"RESULT: FAIL（{len(FAILS)} 项）: " + ", ".join(FAILS))
    sys.exit(1)
print("RESULT: PASS（全部断言通过）")
sys.exit(0)
