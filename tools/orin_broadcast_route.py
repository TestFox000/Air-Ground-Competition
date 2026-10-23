#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""持续广播 0x4003 车命令（航点序列）——车端联调 / 抓包用的测试源。

与生产链的区别：`orin_plan_sidecar.py` 只在"有新路线"时发（之后每 2s 重发），
需要相机情报才会产出路线；本工具**不依赖相机、不监听 50000**，只把一份现成的
`route_plan.json` 里的车命令按固定频率一直广播，方便别人随时抓包/接车联调。

用法（机上仓库根目录）：
    python3 tools/orin_broadcast_route.py                     # 自动取最新 route_plan.json
    python3 tools/orin_broadcast_route.py --plan /tmp/x/route_plan.json
    python3 tools/orin_broadcast_route.py --route "N10 K10 ... ,x" --reset "reset:N10,x"
    python3 tools/orin_broadcast_route.py --hz 5 --port 50000 --broadcast-addr 255.255.255.255
    python3 tools/orin_broadcast_route.py --duration 60       # 跑 60s 自动停（默认 0=一直发）
    python3 tools/orin_broadcast_route.py --dry-run           # 只打印帧信息，不广播

帧内容：与生产链同一编码器 `intel_protocol.build_car_command`（msg_id=0x4003），
每次发送用**当前时间戳**（便于对端判断"源还活着"）。车端 onboard 模式只取
(reset, route)，不按 ts 判陈旧，因此刷新时间戳不影响执行语义。

⚠️ 这是**实发**：车上若处于 onboard 模式且底盘使能，会按该路线行驶。
   仅用于测试场地；不测试时 pkill -f orin_broadcast_route.py 停掉。
"""
from __future__ import annotations

import argparse
import glob
import socket
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO / "src" / "ag_integrate") not in sys.path:
    sys.path.insert(0, str(REPO / "src" / "ag_integrate"))

from ag_integrate.intel_protocol import (      # noqa: E402
    CarCommandFrame, build_car_command, parse_car_command)


def load_plan(path: Path) -> tuple[str, str]:
    """从 route_plan.json 取 (reset, route)；缺字段直接报错而不是发空帧。"""
    import json
    data = json.loads(path.read_text(encoding="utf-8"))
    cmds = data.get("car_cmds") or {}
    reset, route = str(cmds.get("reset", "")), str(cmds.get("route", ""))
    if not route.strip():
        raise ValueError(f"{path} 里没有 car_cmds.route（不是规划产物？）")
    return reset, route


def find_latest_plan() -> Path:
    """最新路线：视频侦察缓存 /tmp/ag_recon/*/route_plan.json 或机载实时 /tmp/ag_plan_live。"""
    cands = [Path(p) for p in glob.glob("/tmp/ag_recon/*/route_plan.json")]
    cands += [Path("/tmp/ag_plan_live/route_plan.json")]
    cands = [p for p in cands if p.is_file()]
    if not cands:
        raise FileNotFoundError(
            "没找到 route_plan.json。先跑一次视频侦察（tools/orin_video_recon.sh）"
            "或用 --plan 指定路径")
    return max(cands, key=lambda p: p.stat().st_mtime)


def make_payload(route: str, reset: str) -> bytes:
    """当前时间戳编码一帧；解析回环自检，编码器/协议改动会当场暴露。"""
    payload = build_car_command(
        CarCommandFrame(int(time.time() * 1000), reset, route))
    back = parse_car_command(payload)
    if back.route != route or back.reset != reset:
        raise RuntimeError("0x4003 编码回环不一致（协议实现有 bug）")
    return payload


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="持续广播 0x4003 车命令（测试源；不依赖相机/规划进程）")
    ap.add_argument("--plan", type=str, default=None,
                    help="route_plan.json 路径（默认自动取最新）")
    ap.add_argument("--route", type=str, default=None,
                    help="直接给车命令串（与 --plan 二选一）")
    ap.add_argument("--reset", type=str, default="",
                    help="reset 命令串（默认取 route_plan 的 car_cmds.reset）")
    ap.add_argument("--hz", type=float, default=2.0, help="发送频率（默认 2Hz=0.5s 一帧）")
    ap.add_argument("--port", type=int, default=50000)
    ap.add_argument("--broadcast-addr", default="255.255.255.255")
    ap.add_argument("--duration", type=float, default=0.0, help=">0 时跑够秒数自动停")
    ap.add_argument("--dry-run", action="store_true", help="只打印帧信息，不广播")
    args = ap.parse_args(argv)

    try:
        if args.route:
            route, reset = args.route, args.reset
        else:
            plan = Path(args.plan) if args.plan else find_latest_plan()
            reset, route = load_plan(plan)
            print(f"[bc] 路线来源: {plan}")
        payload = make_payload(route, reset)
    except (OSError, ValueError, RuntimeError, FileNotFoundError) as exc:
        print(f"[失败] {exc}", file=sys.stderr)
        return 2

    cells = [c for c in route.replace(",", " ").split() if c]
    print(f"[bc] route {len(cells)} token | 帧长 {len(payload)} B | "
          f"{args.hz:g} Hz -> {args.broadcast_addr}:{args.port}"
          f"{'（dry-run，不发送）' if args.dry_run else ''}")
    print(f"[bc] reset={reset!r}")
    print(f"[bc] 前 48 字节: {payload[:48].hex()}")
    if args.dry_run:
        return 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    period = 1.0 / max(0.1, args.hz)
    start, sent, last_report = time.monotonic(), 0, time.monotonic()
    print("[bc] 开始持续广播（Ctrl+C 或 pkill -f orin_broadcast_route.py 停止）")
    try:
        while True:
            payload = make_payload(route, reset)      # 每帧刷新时间戳
            try:
                sock.sendto(payload, (args.broadcast_addr, args.port))
                sent += 1
            except OSError as exc:
                print(f"[bc] 发送失败: {type(exc).__name__}: {exc}", file=sys.stderr)
            now = time.monotonic()
            if now - last_report >= 30.0:
                print(f"[bc] 已发 {sent} 帧（{sent / (now - start):.1f} 帧/秒有效）")
                last_report = now
            if args.duration > 0 and now - start >= args.duration:
                break
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[bc] 收到中断")
    finally:
        sock.close()
    print(f"[bc] 结束：共发 {sent} 帧")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
