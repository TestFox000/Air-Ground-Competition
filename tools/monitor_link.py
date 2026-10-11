#!/usr/bin/env python3
"""空地协作实时监测工具：监听 UDP:50000，把无人机情报画到场地格子图上。

不依赖 ROS / OpenCV，Windows 直接跑。一机两用：
  * 无人机未到货时，配合 tools/fake_udp_sender.py / tools/sim_match.py 彩排全链路；
  * 真机联调时，作为"地面态势看板"盯无人机定位、目标分布、链路质量。

功能：
  1. 解析 0x4001（态势：雷/障碍/桥洞/红蓝靶）+ 0x4002（车位姿：位置+朝向）
  2. 14×11 场地格子图实时刷新（终端 ASCII，--plot 改 matplotlib）
  3. 链路统计：接收帧率、CRC 丢帧、延迟、连续定位成功率
  4. --car-log 同时 tail 车端黑匣子 .jsonl，显示最新规划/路线事件

用法：
    python tools/monitor_link.py                      # 终端 ASCII 看板
    python tools/monitor_link.py --plot               # matplotlib 实时图
    python tools/monitor_link.py --car-log /tmp/ag_logs/20261009  # 同时看车端日志
    python tools/monitor_link.py --once               # 收一帧打印后退出
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src" / "ag_integrate"))

from ag_integrate.field_frame import (  # noqa: E402
    CellRef, COLS, ROWS, LETTERS, X0, Y0, CELL_SIZE, world_to_cell,
)
from ag_integrate.intel_protocol import (  # noqa: E402
    IntelFrame, IntelObject, ObjectType, CarPose,
    parse, parse_car_pose, detect_msg_id, MSG_ID, MSG_ID_CAR_POSE,
)

# 场地显示符号
SYMBOLS = {
    ObjectType.MINE: "M",       # 雷
    ObjectType.OBSTACLE: "#",   # 障碍
    ObjectType.ARROW: "A",      # 桥洞可通行格
    ObjectType.TARGET_BLUE: "B",# 敌方蓝靶
    ObjectType.TARGET_RED: "R", # 友方红靶
    ObjectType.CAR: "C",        # 车
    ObjectType.CELL_LABEL: ".", # 格号（不画在态势图上）
}
COLOR_RESET = "\033[0m"
COLORS = {
    ObjectType.MINE: "\033[93m",       # 黄
    ObjectType.OBSTACLE: "\033[90m",   # 灰
    ObjectType.ARROW: "\033[96m",      # 青
    ObjectType.TARGET_BLUE: "\033[94m",# 蓝
    ObjectType.TARGET_RED: "\033[91m", # 红
    ObjectType.CAR: "\033[92m",        # 绿
}


@dataclass
class Snapshot:
    """一帧态势 + 车位姿（合并后用于渲染）。"""
    ts_ms: int = 0
    objects: list[IntelObject] = field(default_factory=list)
    car_pose: Optional[CarPose] = None
    recv_ts: float = 0.0

    def cell_objects(self) -> dict[str, list[ObjectType]]:
        out: dict[str, list[ObjectType]] = {}
        for o in self.objects:
            try:
                cell = world_to_cell(o.x, o.y)
            except ValueError:
                continue
            out.setdefault(cell.name, []).append(o.type)
        return out


@dataclass
class LinkStats:
    total: int = 0
    good: int = 0
    bad: int = 0
    localized: int = 0
    first_ts: Optional[float] = None
    last_ts: Optional[float] = None
    rtt_ms: deque = field(default_factory=lambda: deque(maxlen=50))

    def record(self, ok: bool, frame_ts_ms: int, recv_ts: float, has_objects: bool) -> None:
        self.total += 1
        if ok:
            self.good += 1
            if has_objects:
                self.localized += 1
        else:
            self.bad += 1
        if self.first_ts is None:
            self.first_ts = recv_ts
        self.last_ts = recv_ts
        if has_objects:
            delay = (recv_ts * 1000) - frame_ts_ms
            if 0 < delay < 5000:
                self.rtt_ms.append(delay)

    @property
    def fps(self) -> float:
        if self.first_ts is None or self.last_ts is None:
            return 0.0
        span = self.last_ts - self.first_ts
        if span <= 0:
            return 0.0
        return self.good / span

    @property
    def drop_rate(self) -> float:
        return self.bad / self.total if self.total else 0.0

    @property
    def avg_delay_ms(self) -> float:
        if not self.rtt_ms:
            return 0.0
        return sum(self.rtt_ms) / len(self.rtt_ms)


def render_ascii(snap: Snapshot, stats: LinkStats, car_ev: str) -> str:
    """渲染 14×11 场地 ASCII 图 + 链路统计。"""
    grid: list[list[str]] = [["·"] * COLS for _ in range(ROWS)]
    color_grid: list[list[str]] = [[""] * COLS for _ in range(ROWS)]
    for name, types in snap.cell_objects().items():
        col = LETTERS.index(name[0])
        row = int(name[1:]) - 1
        if not (0 <= col < COLS and 0 <= row < ROWS):
            continue
        # 优先级：靶 > 雷 > 障碍 > 桥洞 > 车 > 格号
        priority = [ObjectType.TARGET_BLUE, ObjectType.TARGET_RED,
                    ObjectType.MINE, ObjectType.OBSTACLE,
                    ObjectType.ARROW, ObjectType.CAR]
        chosen = next((t for t in priority if t in types), types[0])
        grid[row][col] = SYMBOLS.get(chosen, "?")
        color_grid[row][col] = COLORS.get(chosen, "")

    # 车位姿叠加
    if snap.car_pose is not None:
        try:
            c = world_to_cell(snap.car_pose.x, snap.car_pose.y)
            grid[c.row][c.col] = "C"
            color_grid[c.row][c.col] = COLORS[ObjectType.CAR]
        except ValueError:
            pass

    L: list[str] = []
    L.append("=" * 72)
    L.append(f"空地协作态势看板  |  接收 {stats.total} 帧  "
             f"好={stats.good} 坏={stats.bad} 丢包={stats.drop_rate:.1%}  "
             f"FPS={stats.fps:.1f}  延迟={stats.avg_delay_ms:.0f}ms")
    L.append(f"定位成功={stats.localized} 帧  "
             f"目标数={len(snap.objects)}  "
             f"车位姿={'有' if snap.car_pose else '无'}")
    L.append("-" * 72)
    # 列标头
    L.append("   " + " ".join(f"{c:^2}" for c in LETTERS))
    for r in range(ROWS - 1, -1, -1):  # 行号从上到下递减（11 在顶）
        row_label = f"{r + 1:2d}"
        cells = []
        for c in range(COLS):
            sym = grid[r][c]
            col = color_grid[r][c]
            cells.append(f"{col}{sym:^2}{COLOR_RESET}" if col else f"{sym:^2}")
        L.append(f"{row_label} " + "".join(cells) + f" {row_label}")
    L.append("   " + " ".join(f"{c:^2}" for c in LETTERS))
    L.append("-" * 72)
    # 图例与目标清单
    legend = "  ".join(f"{COLORS[t]}{SYMBOLS[t]}={t.name.lower()}{COLOR_RESET}"
                       for t in (ObjectType.MINE, ObjectType.OBSTACLE,
                                 ObjectType.ARROW, ObjectType.TARGET_BLUE,
                                 ObjectType.TARGET_RED, ObjectType.CAR))
    L.append(legend)
    # 目标清单
    counts: dict[str, int] = {}
    for o in snap.objects:
        counts[o.type.name] = counts.get(o.type.name, 0) + 1
    L.append(f"目标: {counts or '无'}")
    if snap.car_pose:
        L.append(f"车: x={snap.car_pose.x:.3f} y={snap.car_pose.y:.3f} "
                 f"yaw={snap.car_pose.yaw:.3f}rad ({__import__('math').degrees(snap.car_pose.yaw):.1f}°)")
    if car_ev:
        L.append(f"[车端] {car_ev}")
    L.append("=" * 72)
    return "\n".join(L)


def tail_latest_event(log_dir: Path) -> str:
    """读车端黑匣子最后一条事件（取最新修改的 .jsonl）。"""
    if not log_dir.exists():
        return ""
    files = sorted(log_dir.rglob("*.jsonl"), key=lambda p: p.stat().st_mtime)
    if not files:
        return ""
    try:
        with open(files[-1], "r", encoding="utf-8", errors="replace") as fh:
            lines = fh.readlines()
        for line in reversed(lines):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except Exception:
                continue
            ev = rec.get("ev", "")
            # 只显示对协作有意义的事件
            if ev in ("plan.built", "route.publish", "route.blocked",
                      "intel.recv", "node.ready", "fire.sent", "shoot.count"):
                extra = {k: v for k, v in rec.items()
                         if k not in ("ts", "mono", "seq", "node", "ev", "lvl", "session")}
                return f"{ev} {json.dumps(extra, ensure_ascii=False)}"
    except Exception:
        return ""
    return ""


def main() -> int:
    ap = argparse.ArgumentParser(description="空地协作实时监测（UDP:50000 态势看板）")
    ap.add_argument("--port", type=int, default=50000, help="UDP 监听端口")
    ap.add_argument("--plot", action="store_true", help="用 matplotlib 实时画图")
    ap.add_argument("--car-log", type=Path, default=None,
                    help="车端黑匣子日志目录，同时显示最新规划/路线事件")
    ap.add_argument("--once", action="store_true", help="收一帧打印后退出")
    ap.add_argument("--no-color", action="store_true", help="禁用 ANSI 颜色")
    args = ap.parse_args()

    if args.no_color:
        for k in list(COLORS):
            COLORS[k] = ""
        global COLOR_RESET
        COLOR_RESET = ""

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", args.port))
    sock.settimeout(1.0)
    print(f"[monitor] 监听 UDP:{args.port}（Ctrl+C 退出）")

    snap = Snapshot()
    stats = LinkStats()
    last_car_ev = ""
    car_log_dir = args.car_log

    fig = ax = im = None
    if args.plot:
        try:
            import matplotlib
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(9, 7))
            plt.ion()
        except ImportError:
            print("[monitor] matplotlib 未安装，回退 ASCII 模式")
            args.plot = False

    try:
        while True:
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue
            recv_ts = time.time()
            msg_id = detect_msg_id(data)
            if msg_id is None:
                stats.record(False, 0, recv_ts, False)
                continue

            if msg_id == MSG_ID:
                try:
                    frame = parse(data)
                except ValueError:
                    stats.record(False, 0, recv_ts, False)
                    continue
                snap = Snapshot(ts_ms=frame.timestamp_ms,
                                objects=list(frame.objects),
                                car_pose=snap.car_pose,
                                recv_ts=recv_ts)
                stats.record(True, frame.timestamp_ms, recv_ts, bool(frame.objects))
            elif msg_id == MSG_ID_CAR_POSE:
                try:
                    pose = parse_car_pose(data)
                except ValueError:
                    continue
                snap.car_pose = pose
                stats.record(True, pose.timestamp_ms, recv_ts, True)
            else:
                continue

            if car_log_dir is not None:
                ev = tail_latest_event(car_log_dir)
                if ev:
                    last_car_ev = ev

            if args.once:
                print(render_ascii(snap, stats, last_car_ev))
                return 0

            if args.plot and fig is not None:
                _render_plot(ax, snap, stats)
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            else:
                # 清屏重绘
                if os.name == "nt":
                    os.system("cls")
                else:
                    sys.stdout.write("\033[2J\033[H")
                print(render_ascii(snap, stats, last_car_ev))
    except KeyboardInterrupt:
        print("\n[monitor] 停止")
        return 0


def _render_plot(ax, snap: Snapshot, stats: LinkStats) -> None:
    import numpy as np
    ax.clear()
    grid = np.zeros((ROWS, COLS, 3), dtype=float)
    # 背景
    grid[:] = [0.95, 0.95, 0.95]
    color_map = {
        ObjectType.MINE: [1.0, 0.8, 0.0],
        ObjectType.OBSTACLE: [0.4, 0.4, 0.4],
        ObjectType.ARROW: [0.0, 0.8, 0.8],
        ObjectType.TARGET_BLUE: [0.2, 0.4, 1.0],
        ObjectType.TARGET_RED: [1.0, 0.2, 0.2],
        ObjectType.CAR: [0.0, 0.8, 0.0],
    }
    for name, types in snap.cell_objects().items():
        col = LETTERS.index(name[0])
        row = int(name[1:]) - 1
        for t in (ObjectType.TARGET_BLUE, ObjectType.TARGET_RED, ObjectType.MINE,
                  ObjectType.OBSTACLE, ObjectType.ARROW, ObjectType.CAR):
            if t in types:
                grid[row, col] = color_map[t]
                break
    if snap.car_pose is not None:
        try:
            c = world_to_cell(snap.car_pose.x, snap.car_pose.y)
            grid[c.row, c.col] = color_map[ObjectType.CAR]
        except ValueError:
            pass
    ax.imshow(grid, origin="upper", extent=[-0.5, COLS - 0.5, ROWS - 0.5, -0.5])
    ax.set_xticks(range(COLS))
    ax.set_xticklabels(list(LETTERS))
    ax.set_yticks(range(ROWS))
    ax.set_yticklabels([str(r + 1) for r in range(ROWS)])
    ax.set_title(f"态势  FPS={stats.fps:.1f} 丢包={stats.drop_rate:.1%} "
                 f"延迟={stats.avg_delay_ms:.0f}ms 目标={len(snap.objects)}")
    ax.grid(True, color="white", linewidth=0.5)


if __name__ == "__main__":
    raise SystemExit(main())
