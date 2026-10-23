#!/usr/bin/env python3
"""假机载 UDP 发送器：用**车端同一个编码器** intel_protocol.build 广播合法情报帧。

用途：无人机未上线时，在任何机器上模拟 dzuav 的 UDP:50000 广播，
驱动车端 integrate_node 跑通"收包 -> 解析 -> 态势归并 -> 门控 -> 路线"全链路。

与机上 UdpBroadcaster 字节级一致（build 与 parse 互为逆运算，CRC16/魔数/后缀
完全对应 reference/onboard/dzuav/src/UdpBroadcaster.cpp）。

用法：
    # 默认场景：1 个雷(PAPER) + 1 个蓝靶（满足 min_mines=1, require_face=True 门控）
    tools/fake_udp_sender.py

    # 自定义场景，用格号:类别 列表
    tools/fake_udp_sender.py --scenario "B3:paper,D5:obstacle,H7:target_blue"

    # 单发后退出（集成测试用）
    tools/fake_udp_sender.py --once

    # 改端口/频率/目标（部分 Windows 广播不回环，本机自收彩排走 --to 127.0.0.1）
    tools/fake_udp_sender.py --port 50000 --rate 10 --to 127.0.0.1

类别名（对应 ObjectType）：paper cell_label car target_red target_blue obstacle
                           bucket_red bucket_green bucket_cyan car_start car_end
                           uav_start uav_end
"""
from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src" / "ag_integrate"))

from ag_integrate.field_frame import CellRef, cell_to_world  # noqa: E402
from ag_integrate.intel_protocol import (  # noqa: E402
    IntelFrame,
    IntelObject,
    ObjectType,
    build,
)

# 2026-10-16 起 13 类：旧 7 类的 mine→paper（语义由车端 world_model 按落格判雷）
DEFAULT_SCENARIO = "B3:paper,H7:target_blue"


def parse_scenario(text: str) -> list[IntelObject]:
    objs: list[IntelObject] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        cell_s, _, type_s = part.partition(":")
        cell_s = cell_s.strip().upper()
        type_s = type_s.strip().lower()
        if not cell_s or not type_s:
            raise SystemExit(f"场景项格式错误（应为 格号:类别）: {part!r}")
        if not (cell_s[0].isalpha() and cell_s[1:].isdigit()):
            raise SystemExit(f"格号非法: {cell_s!r}")
        col = ord(cell_s[0]) - ord("A")
        row = int(cell_s[1:]) - 1
        cell = CellRef(col, row)
        cell.validate()
        x, y = cell_to_world(cell)
        try:
            otype = ObjectType[type_s.upper()]
        except KeyError:
            raise SystemExit(
                f"未知类别 {type_s!r}，可选: {', '.join(t.name.lower() for t in ObjectType)}"
            )
        objs.append(IntelObject(otype, x, y, 0.08, 0.06))
    return objs


def send(sock: socket.socket, port: int, frame: IntelFrame, dest: str) -> int:
    data = build(frame)
    sock.sendto(data, (dest, port))
    return len(data)


def main() -> int:
    ap = argparse.ArgumentParser(description="假机载 UDP:50000 发送器（复用 intel_protocol.build）")
    ap.add_argument("--scenario", default=DEFAULT_SCENARIO,
                    help=f"场景：格号:类别 列表，逗号分隔（默认 {DEFAULT_SCENARIO}）")
    ap.add_argument("--port", type=int, default=50000)
    ap.add_argument("--rate", type=float, default=5.0, help="发送频率 Hz（循环模式）")
    ap.add_argument("--once", action="store_true", help="只发一帧后退出")
    ap.add_argument("--to", default="255.255.255.255",
                    help="目标地址（默认全网广播；本机自收彩排用 127.0.0.1）")
    args = ap.parse_args()

    objs = parse_scenario(args.scenario)
    if not objs:
        raise SystemExit("场景为空，无目标可发送")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    names = {t.value: t.name for t in ObjectType}
    print(f"[fake_udp] -> {args.to}:{args.port} objects={len(objs)} "
          f"mode={'once' if args.once else f'loop@{args.rate}Hz'}")
    for o in objs:
        print(f"  {names[o.type]:11s} ({o.x:.3f},{o.y:.3f})")

    try:
        if args.once:
            ts = int(time.time() * 1000)
            n = send(sock, args.port, IntelFrame(ts, tuple(objs)), args.to)
            print(f"[fake_udp] 已发送 1 帧 ({n} 字节)")
            return 0
        period = 1.0 / args.rate
        count = 0
        while True:
            ts = int(time.time() * 1000)
            n = send(sock, args.port, IntelFrame(ts, tuple(objs)), args.to)
            count += 1
            if count == 1 or count % 10 == 0:
                print(f"[fake_udp] sent #{count} ({n} 字节), ts={ts}")
            time.sleep(period)
    except KeyboardInterrupt:
        print(f"\n[fake_udp] 停止，共发送 {count} 帧")
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
