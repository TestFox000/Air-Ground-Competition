# -*- coding: utf-8 -*-
"""MAVLink 直连探测（备用通道）：监听 PX4 的 MAVLink UDP 心跳。

用途：机载 PX4 主通道是 MicroXRCEAgent DDS 桥（不是裸 MAVLink），但若 PX4
额外起了 mavlink UDP 实例（常见默认 14550），笔记本可用本脚本/QGC 直连。

用法（笔记本）：
    python mavlink_probe.py                 # 监听 0.0.0.0:14550，等 15s
    python mavlink_probe.py --port 14550 --wait 30
    python mavlink_probe.py --active 192.168.55.1   # 主动模式：先发心跳勾引对端回包

判读：
    收到 HEARTBEAT → 打印 sysid/compid/autopilot/type，说明 MAVLink 通道可用，
                     QGroundControl 也能连（QGC 默认同样监听 UDP 14550）。
    超时无心跳     → PX4 未暴露 MAVLink UDP，主通道只能走 SSH+仓库工具
                     （uav_link.py / flight_console.py），属预期架构，不是故障。
"""
from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=14550, help="监听 UDP 端口")
    ap.add_argument("--wait", type=float, default=15.0, help="等待心跳秒数")
    ap.add_argument("--active", metavar="HOST", default=None,
                    help="主动模式：向 HOST:14550 先发本端心跳再监听")
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{args.port}", dialect="common", autoreconnect=False)

    if args.active:
        # 伪装成 GCS(sysid=255) 发心跳，PX4 的 mavlink 实例收到后通常会开始回发
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        print(f"active 模式：已向 {args.active}:{args.port} 发送 GCS 心跳")

    print(f"监听 udp:0.0.0.0:{args.port}，等待 {args.wait:.0f}s ...")
    deadline = time.time() + args.wait
    seen = set()
    while time.time() < deadline:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        key = (msg.get_srcSystem(), msg.get_srcComponent())
        try:
            ap_name = mavutil.mavlink.enums["MAV_AUTOPILOT"][msg.autopilot].name
            ty_name = mavutil.mavlink.enums["MAV_TYPE"][msg.type].name
        except Exception:
            ap_name, ty_name = str(msg.autopilot), str(msg.type)
        print(f"HEARTBEAT sysid={key[0]} compid={key[1]} "
              f"autopilot={ap_name} type={ty_name}")
        seen.add(key)
        # 主动模式下对端可能要求持续心跳，补发
        if args.active:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    if seen:
        print(f"\nOK: 收到 {len(seen)} 个组件的心跳，MAVLink 直连可用（QGC 亦可连）")
        return 0
    print("\nNO_MAVLINK: 超时无心跳——PX4 未在本端口暴露 MAVLink，"
          "主通道请走 SSH + tools/uav_link.py（XRCE-DDS 架构下属预期）")
    return 1


if __name__ == "__main__":
    sys.exit(main())
