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
import math
import os
import socket
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src" / "ag_integrate"))

from ag_integrate.field_frame import (  # noqa: E402
    COLS, ROWS, LETTERS, world_to_cell,
)
from ag_integrate.intel_protocol import (  # noqa: E402
    IntelObject, ObjectType, CarPose,
    parse, parse_car_pose, detect_msg_id, MSG_ID, MSG_ID_CAR_POSE,
)

# ---- 显示折叠：13 类 D13 目标 -> 看板符号键 ----
# 2026-10-16 起模型是 hbb_d13_best.pt（13 类），本文件此前还按 7 类写死
# ObjectType.MINE / ObjectType.ARROW——这两个成员已不存在，模块导入即
# AttributeError，看板根本起不来。现按真实枚举重建。
#
# PAPER 的语义按落格判定（与 world_model.py:120-127 同口径）：K 列桥洞区的
# 纸片 = 通行箭头，其余格的纸片 = 雷。看板必须照抄这条规则，否则操作员看到
# 的态势和车上大脑相信的态势不是一回事。
_BRIDGE_COL = "K"

_KIND_BY_TYPE = {
    ObjectType.CELL_LABEL: "label",
    ObjectType.CAR: "car",
    ObjectType.TARGET_RED: "red",
    ObjectType.TARGET_BLUE: "blue",
    ObjectType.OBSTACLE: "obstacle",
    ObjectType.BUCKET_RED: "bucket_r",
    ObjectType.BUCKET_GREEN: "bucket_g",
    ObjectType.BUCKET_CYAN: "bucket_c",
    ObjectType.CAR_START: "car_start",
    ObjectType.CAR_END: "car_end",
    ObjectType.UAV_START: "uav_start",
    ObjectType.UAV_END: "uav_end",
}

SYMBOLS = {
    "mine": "M",         # PAPER（非 K 列）= 雷
    "arrow": "A",        # PAPER（K 列）= 桥洞可通行箭头
    "obstacle": "#",     # 障碍
    "blue": "B",         # 敌方蓝靶（可打）
    "red": "R",          # 友方红靶（禁火）
    "car": "C",          # 车
    "bucket_r": "r",     # 红桶（禁火）
    "bucket_g": "g",     # 绿桶
    "bucket_c": "c",     # 青桶
    "car_start": "S",    # 智能车起点
    "car_end": "E",      # 智能车终点
    "uav_start": "s",    # 无人机起点
    "uav_end": "e",      # 无人机终点
    "label": ".",        # 地面格号（不画在态势图上）
    "unknown": "?",      # 枚举新增了类别但看板没跟上
}
COLOR_RESET = "\033[0m"
COLORS = {
    "mine": "\033[93m",        # 黄
    "arrow": "\033[96m",       # 青
    "obstacle": "\033[90m",    # 灰
    "blue": "\033[94m",        # 蓝
    "red": "\033[91m",         # 红
    "car": "\033[92m",         # 绿
    "bucket_r": "\033[91m",    # 红
    "bucket_g": "\033[92m",    # 绿
    "bucket_c": "\033[96m",    # 青
    "car_start": "\033[92m",   # 绿
    "car_end": "\033[92m",     # 绿
    "uav_start": "\033[95m",   # 紫
    "uav_end": "\033[95m",     # 紫
    "label": "",               # 格号：有意不着色（SYMBOLS 注释：不画在态势图上）
    # 品红加粗 = 故意刺眼。走到这个颜色说明协议加了类别而看板没跟上，
    # 正是当年让本工具 import 即崩的那种漂移；宁可难看，也不许它 quietly
    # 混在一片无色字符里被当成正常目标忽略过去。
    "unknown": "\033[1;95m",
}

# 同格多目标的显示优先级：靶 > 雷 > 障碍 > 桥洞 > 桶 > 起止点 > 车 > 格号
_PRIORITY = ("blue", "red", "mine", "obstacle", "arrow",
             "bucket_r", "bucket_g", "bucket_c",
             "car_start", "car_end", "uav_start", "uav_end", "car", "label")


def _kind(t: ObjectType, cell_name: str) -> str:
    """目标类别 -> 符号键。PAPER 按落格分雷/箭头，未知类别落到 'unknown'。"""
    if t is ObjectType.PAPER:
        return "arrow" if cell_name.startswith(_BRIDGE_COL) else "mine"
    return _KIND_BY_TYPE.get(t, "unknown")


@dataclass
class Snapshot:
    """一帧态势 + 车位姿（合并后用于渲染）。"""
    ts_ms: int = 0
    objects: list[IntelObject] = field(default_factory=list)
    car_pose: Optional[CarPose] = None
    recv_ts: float = 0.0

    def cell_objects(self) -> dict[str, list[str]]:
        """格名 -> 该格上的符号键列表（PAPER 已按落格分成雷/箭头）。"""
        out: dict[str, list[str]] = {}
        for o in self.objects:
            try:
                cell = world_to_cell(o.x, o.y)
            except ValueError:
                continue
            out.setdefault(cell.name, []).append(_kind(o.type, cell.name))
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
    for name, kinds in snap.cell_objects().items():
        col = LETTERS.index(name[0])
        row = int(name[1:]) - 1
        if not (0 <= col < COLS and 0 <= row < ROWS):
            continue
        chosen = next((k for k in _PRIORITY if k in kinds), kinds[0])
        grid[row][col] = SYMBOLS.get(chosen, "?")
        color_grid[row][col] = COLORS.get(chosen, "")

    # 车位姿叠加
    if snap.car_pose is not None:
        try:
            c = world_to_cell(snap.car_pose.x, snap.car_pose.y)
            grid[c.row][c.col] = SYMBOLS["car"]
            color_grid[c.row][c.col] = COLORS["car"]
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
    legend = "  ".join(f"{COLORS.get(k, '')}{SYMBOLS[k]}={k}{COLOR_RESET}"
                       for k in ("mine", "arrow", "obstacle", "blue", "red",
                                 "bucket_r", "bucket_g", "bucket_c", "car"))
    L.append(legend)
    # 目标清单（按原始 13 类名统计，PAPER 的雷/箭头之分只在图上体现）
    counts: dict[str, int] = {}
    for o in snap.objects:
        counts[o.type.name] = counts.get(o.type.name, 0) + 1
    L.append(f"目标: {counts or '无'}")
    if snap.car_pose:
        L.append(f"车: x={snap.car_pose.x:.3f} y={snap.car_pose.y:.3f} "
                 f"yaw={snap.car_pose.yaw:.3f}rad "
                 f"({math.degrees(snap.car_pose.yaw):.1f}°)")
    if car_ev:
        L.append(f"[车端] {car_ev}")
    L.append("=" * 72)
    return "\n".join(L)


def _tail_lines(path: Path, max_bytes: int = 65536) -> list[str]:
    """只从文件尾回溯读最后一段（默认 64KB），避免每帧全量 readlines 的 O(n²)。

    黑匣子 .jsonl 随比赛持续增长，监测循环每帧都调本函数，全量读会越跑越慢。
    """
    size = path.stat().st_size
    with open(path, "rb") as fh:
        fh.seek(max(0, size - max_bytes))
        data = fh.read()
    lines = data.decode("utf-8", errors="replace").splitlines()
    if size > max_bytes and lines:
        lines = lines[1:]  # 首行可能只读到半截，丢弃
    return lines


def tail_latest_event(log_dir: Path) -> str:
    """读车端黑匣子最后一条事件（取最新修改的 .jsonl）。"""
    if not log_dir.exists():
        return ""
    files = sorted(log_dir.rglob("*.jsonl"), key=lambda p: p.stat().st_mtime)
    if not files:
        return ""
    try:
        for line in reversed(_tail_lines(files[-1])):
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
    if os.name == "nt":
        # 一次性开启 Windows 控制台的 ANSI 转义支持（空命令触发 conhost 初始化），
        # 之后清屏直接用 \033[2J\033[H，不再每帧 os.system("cls") 起进程
        os.system("")

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
                # 0x4002 是车位姿帧，不含场上目标：has_objects 必须为 False，
                # 否则每个位姿帧都被误计入"定位成功"帧数，虚增定位成功率
                stats.record(True, pose.timestamp_ms, recv_ts, False)
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
                # 清屏重绘：统一用 ANSI 转义（Windows 已在启动时开启 VT 支持），
                # 不再每帧 os.system("cls") 起进程
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
        "mine": [1.0, 0.8, 0.0],
        "arrow": [0.0, 0.8, 0.8],
        "obstacle": [0.4, 0.4, 0.4],
        "blue": [0.2, 0.4, 1.0],
        "red": [1.0, 0.2, 0.2],
        "bucket_r": [0.8, 0.3, 0.3],
        "bucket_g": [0.3, 0.8, 0.3],
        "bucket_c": [0.3, 0.8, 0.8],
        "car": [0.0, 0.8, 0.0],
        "car_start": [0.6, 0.9, 0.6],
        "car_end": [0.6, 0.9, 0.6],
        "uav_start": [0.7, 0.4, 0.9],
        "uav_end": [0.7, 0.4, 0.9],
    }
    for name, kinds in snap.cell_objects().items():
        col = LETTERS.index(name[0])
        row = int(name[1:]) - 1
        if not (0 <= col < COLS and 0 <= row < ROWS):
            continue
        chosen = next((k for k in _PRIORITY if k in kinds), kinds[0])
        if chosen in color_map:
            grid[row, col] = color_map[chosen]
    if snap.car_pose is not None:
        try:
            c = world_to_cell(snap.car_pose.x, snap.car_pose.y)
            grid[c.row, c.col] = color_map["car"]
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
