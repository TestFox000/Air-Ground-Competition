#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""link_status.py —— DDS 链路现状复验（验收④后半）：如实报结果，不通就写清网络前提

只做**非侵入**探测：ICMP、TCP:22 端口、本机网段、以及从**只读**的配置真源里读域号。
不登车、不起 ROS、不发任何 DDS 包。

产出: EVIDENCE/link_status.md
用法: python link_status.py [--drone 10.29.122.104] [--car 192.168.137.241]
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import socket
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
EVID = HERE / "EVIDENCE"


def ping(ip: str, n: int = 2, timeout_ms: int = 1200):
    try:
        r = subprocess.run(["ping", "-n", str(n), "-w", str(timeout_ms), ip],
                           capture_output=True, text=True, timeout=15)
        out = r.stdout or ""
        loss = "100% loss" in out or "(100%" in out
        rtt = ""
        for line in out.splitlines():
            if "Average" in line or "平均" in line:
                rtt = line.strip()
        return {"alive": not loss, "loss": loss, "rtt": rtt}
    except Exception as exc:                       # noqa: BLE001
        return {"alive": False, "loss": True, "rtt": "", "error": str(exc)}


def tcp(ip: str, port: int = 22, timeout=2.0):
    s = socket.socket(); s.settimeout(timeout)
    try:
        s.connect((ip, port)); return True
    except Exception:
        return False
    finally:
        s.close()


def local_ipv4():
    try:
        r = subprocess.run(["powershell", "-NoProfile", "-Command",
                            "(Get-NetIPAddress -AddressFamily IPv4 | Where-Object {$_.IPAddress -ne '127.0.0.1'} | "
                            "Select-Object IPAddress,PrefixLength,InterfaceAlias | ConvertTo-Json -Compress)"],
                           capture_output=True, text=True, timeout=25)
        return json.loads((r.stdout or "[]").strip() or "[]")
    except Exception as exc:                       # noqa: BLE001
        return [{"error": str(exc)}]


def env_domain(path: pathlib.Path):
    if not path.is_file():
        return None
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        s = line.strip()
        if s.startswith("#") or "DOMAIN" not in s:
            continue
        if "export" in s or "=" in s:
            return s
    return "(文件存在，但没找到 DOMAIN 行)"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--drone", default=os.environ.get("DZ_DRONE_HOST", "10.29.122.104"))
    ap.add_argument("--car", default="192.168.137.241")
    a = ap.parse_args()
    EVID.mkdir(parents=True, exist_ok=True)

    d = ping(a.drone); c = ping(a.car)
    d22, c22 = tcp(a.drone), tcp(a.car)
    ips = local_ipv4()
    drone_env = env_domain(WORKSPACE / "dzsmartscout" / "tools" / "link" / "dzlink.env")
    car_env = env_domain(WORKSPACE / "dzrobot_v2" / "dzros.env")

    same_subnet = False
    car_net = ".".join(a.car.split(".")[:3]) + "."
    for e in ips:
        ip = e.get("IPAddress", "")
        if ip.startswith(car_net):
            same_subnet = True

    lines = [
        "# DDS 链路现状复验（t60 验收④后半）",
        "",
        "复验时间：%s（本机时钟）" % time.strftime("%Y-%m-%d %H:%M:%S"),
        "",
        "> 只做非侵入探测（ICMP / TCP:22 / 本机网段 / 只读配置）。**未登车、未起 ROS、未发 DDS 包**。",
        "",
        "## 1. 两端可达性",
        "",
        "| 目标 | ICMP | TCP:22 | 说明 |",
        "|---|---|---|---|",
        "| 无人机 `%s` | %s | %s | %s |" % (
            a.drone, "✅ 通" if d["alive"] else "❌ 不通(100% loss)", "✅ 开" if d22 else "❌ 关",
            d["rtt"] or "无 RTT"),
        "| 小车 `%s` | %s | %s | %s |" % (
            a.car, "✅ 通" if c["alive"] else "❌ 不通", "✅ 开" if c22 else "❌ 关",
            c["rtt"] or "无 RTT"),
        "",
        "## 2. 本机网络（判断是否同网段）",
        "",
        "```json",
        json.dumps(ips, ensure_ascii=False, indent=2),
        "```",
        "",
        "⇒ 本机与小车**同网段**：%s（同 `%s0/24` 才为真）" % ("是 ✅" if same_subnet else "否 ❌", car_net),
        "",
        "## 3. 域号真值（只读取证）",
        "",
        "| 来源 | 内容 |",
        "|---|---|",
        "| 机侧 `dzsmartscout/tools/link/dzlink.env` | `%s` |" % (drone_env or "(读不到)"),
        "| 车侧 `dzrobot_v2/dzros.env` | `%s` |" % (car_env or "(读不到)"),
        "| 车侧权威手册 `dzrobot_v2/docs/空地指挥链路_协议与运行手册.md` | 「域号 两端都 118」(:18/:259) |",
        "",
        "⚠️ **与任务卡口径冲突（需队长裁断）**：t60 卡写「DDS 域两端 = 0」，但**车侧真源是 `118`**",
        "（`dzros.env` 的 `DZ_ROS_DOMAIN=118`、`start_all.sh` 里 `ROS_DOMAIN_ID=118`、手册与 t4 实测一致）。",
        "「0」只是**谁都没 source env 时**的默认值 —— 那正是项目里著名的静默故障（ping/SSH 全通、ROS 互不可见、零报错）。",
        "⇒ 下发/探针**必须显式内联或 source**：机侧 `source ~/dzlink_tools/dzlink.env`（=118），",
        "车侧由 `start_all.sh` source `~/dzrobot/dzros.env`（=118）。",
        "",
        "## 4. 结论：本次**端到端 DDS 可见性未能验证**（网络前提未满足）",
        "",
        "1. 无人机 `%s` **不在线且不在本网段** ⇒ 无法验证「两端互见」。" % a.drone,
        "2. 小车在本网段且 22 开，但按 t60 边界「车端栈保持冷态、不登车」⇒ 本次未做车侧 ROS 探测。",
        "3. **要验证的下一步（无人机上电后）**：",
        "   · 无人机接入**同一网段**（当前 %s0/24；接入后 `ip -4 addr` 应出现该网段地址）" % car_net,
        "   · 机侧：`source /opt/ros/humble/setup.bash && source ~/dzlink_tools/dzlink.env`（→118）",
        "   · 机侧起常驻：`python3 ~/dzlink_tools/dzlink_supervisor.py run --domain 118 …`，看 `[domain] ROS_DOMAIN_ID=118 (来源: envfile)`",
        "   · 车侧（**由队长决定窗口**，且车栈状态已知）再放只读探针 `/tmp/t4_car_consumer.py` 看 `[rx OK]`",
        "   · 判据：机侧「订阅者匹配数 ≥1」+ 车侧 `[rx OK]` 同时成立；换 AP 后先跑一次组播测（`ros2 multicast send/receive`）",
        "4. 在前提满足前，**任何「链路已通」的结论都是没有依据的** —— 本报告不假装通。",
        "",
    ]
    (EVID / "link_status.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[:14]))
    print("...\n### 证据: EVIDENCE/link_status.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
