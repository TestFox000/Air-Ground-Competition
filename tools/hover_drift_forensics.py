#!/usr/bin/env python3
# coding=utf-8
"""悬停/漂移取证分析：control.log 状态窗口 × odom.log "position delta NED" 序列。

背景：dzpx4 栈的 px4_odom_convert 以 ~43Hz 往 stdout 打无时间戳的
"position delta NED [m]: dx=.. dy=.. dz=.." 行（发给 PX4 EKF2 的 VIO 位姿
单步增量），而 control.log 里 px4_control 状态机的每次迁移都带 ROS 时间戳。
本工具做的事：
  1) 用 odom.log 里带时间戳的 INFO/WARN 行当锚点，按行号在相邻锚点间线性
     插值，给每个 delta 行重建时间戳（锚点间实测速率约 43Hz）；
  2) 从 control.log 划出状态窗口（DISARMED 地面基线 / TAKEOFF / HOVER /
     LANDING）与 ARM/LAND/脱管事件；
  3) 对每个窗口统计 NED 三轴漂移：净位移、单步 RMS、等效速度 RMS、
     单步峰值、方向偏置（均值），并统计窗内 EKF frame reset 次数；
  4) 多会话交叉输出 Markdown 表格。

用法：
    python tools/hover_drift_forensics.py <会话目录或日志对> ...
    python tools/hover_drift_forensics.py logs/uav/flight2 --glob
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

# ── 日志行模式 ─────────────────────────────────────────────────
TS_RE = re.compile(r"\[(?:INFO|WARN|ERROR)\] \[(\d+\.\d+)\]")
ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
DELTA_RE = re.compile(
    r"position delta NED \[m\]: dx=(-?[\d.eE+-]+)\s+dy=(-?[\d.eE+-]+)\s+"
    r"dz=(-?[\d.eE+-]+)")
FIRST_RE = re.compile(
    r"first position NED \[m\]: x=(-?[\d.eE+-]+)\s+y=(-?[\d.eE+-]+)\s+"
    r"z=(-?[\d.eE+-]+)")
STATE_RE = re.compile(r"state: (\w+) -> (\w+) \((.*)\)")
ARM_RE = re.compile(r"ARM sent, target=\((-?[\d.]+), (-?[\d.]+), (-?[\d.]+)\)"
                    r" yaw=(-?[\d.]+)")
RESET_RE = re.compile(r"EKF frame reset detected \(xy_reset=(\d+) "
                      r"heading_reset=(\d+)\)")
CTRL_TS_RE = re.compile(r"\[(?:INFO|WARN|ERROR)\] \[(\d+\.\d+)\] "
                        r"\[px4_control_node\]")

NOMINAL_RATE_HZ = 43.5  # 锚点间实测标称速率（2026-10 六个会话均 ≈43Hz）


# ── 数据结构 ───────────────────────────────────────────────────
@dataclass
class DeltaSample:
    t: float           # 重建时间戳（s）
    dx: float
    dy: float
    dz: float


@dataclass
class StateWindow:
    state: str
    t0: float
    t1: float | None = None  # None = 会话结束仍未退出

    @property
    def duration(self) -> float | None:
        return None if self.t1 is None else self.t1 - self.t0


@dataclass
class OdomSeries:
    samples: list[DeltaSample]
    resets: list[tuple[float, int, int]]   # (t, xy_reset, heading_reset)
    first_position: tuple[float, float, float] | None = None
    anchor_count: int = 0


@dataclass
class WindowStats:
    n: int
    duration: float
    net: tuple[float, float, float]        # 窗内净位移 NED
    step_rms: tuple[float, float, float]   # 单步增量 RMS（m/步）
    vel_rms: tuple[float, float, float]    # 等效速度 RMS（m/s，×实测速率）
    max_step: tuple[float, float, float]   # 单步峰值 |d|
    mean_step: tuple[float, float, float]  # 方向偏置（均值/步）
    resets: int = 0                        # 窗内 EKF reset 事件数


# ── 解析 ───────────────────────────────────────────────────────
def parse_odom_log(path: Path) -> OdomSeries:
    """解析 px4_odom_convert 日志：delta 行 + 时间锚点 + EKF reset 事件。"""
    anchors: list[tuple[int, float]] = []   # (行号, t)
    raw_deltas: list[tuple[int, float, float, float]] = []
    resets: list[tuple[float, int, int]] = []
    first_pos = None
    with open(path, encoding="utf-8", errors="replace") as f:
        for i, line in enumerate(f):
            m = TS_RE.search(line)
            if m:
                t = float(m.group(1))
                anchors.append((i, t))
                r = RESET_RE.search(line)
                if r:
                    resets.append((t, int(r.group(1)), int(r.group(2))))
            d = DELTA_RE.search(line)
            if d:
                raw_deltas.append((i,) + tuple(float(g) for g in d.groups()))
            elif first_pos is None:
                fp = FIRST_RE.search(line)
                if fp:
                    first_pos = tuple(float(g) for g in fp.groups())

    def t_of(line_idx: int) -> float | None:
        """按相邻锚点行号线性插值；末尾锚点之后按标称速率外推。"""
        prev = None
        nxt = None
        for li, t in anchors:
            if li <= line_idx:
                prev = (li, t)
            else:
                nxt = (li, t)
                break
        if prev and nxt and nxt[0] > prev[0]:
            frac = (line_idx - prev[0]) / (nxt[0] - prev[0])
            return prev[1] + frac * (nxt[1] - prev[1])
        if prev and line_idx > prev[0]:
            return prev[1] + (line_idx - prev[0]) / NOMINAL_RATE_HZ
        return None

    samples = []
    for li, dx, dy, dz in raw_deltas:
        t = t_of(li)
        if t is not None:
            samples.append(DeltaSample(t, dx, dy, dz))
    return OdomSeries(samples=samples, resets=resets, first_position=first_pos,
                      anchor_count=len(anchors))


def parse_control_log(path: Path) -> tuple[list[StateWindow], list[tuple[float, str]]]:
    """解析 px4_control 日志：状态窗口序列 + 关键事件（ARM/LAND/脱管/拒绝）。"""
    windows: list[StateWindow] = []
    events: list[tuple[float, str]] = []
    cur: StateWindow | None = None
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = CTRL_TS_RE.search(line)
            if not m:
                continue
            t = float(m.group(1))
            s = STATE_RE.search(line)
            if s:
                new_state = s.group(2)
                if cur is not None:
                    cur.t1 = t
                    windows.append(cur)
                cur = StateWindow(state=new_state, t0=t)
                events.append((t, f"state->{new_state} ({s.group(3)})"))
                continue
            a = ARM_RE.search(line)
            if a:
                events.append((t, f"ARM target=({a.group(1)},{a.group(2)},"
                                  f"{a.group(3)}) yaw={a.group(4)}"))
                continue
            for key in ("ARM refused", "left OFFBOARD", "LAND ignored",
                        "DISARM sent", "DO_SET_MODE"):
                if key in line:
                    detail = line.rstrip().split("[px4_control_node]:")[-1]
                    events.append((t, ANSI_RE.sub("", detail).strip()))
                    break
    if cur is not None:
        windows.append(cur)
    return windows, events


# ── 统计 ───────────────────────────────────────────────────────
def window_stats(series: OdomSeries, t0: float, t1: float,
                 resets: list[tuple[float, int, int]] | None = None
                 ) -> WindowStats | None:
    """窗口 [t0,t1] 内 delta 序列的漂移统计；样本不足返回 None。"""
    sel = [s for s in series.samples if t0 <= s.t <= t1]
    if len(sel) < 2:
        return None
    n = len(sel)
    dur = sel[-1].t - sel[0].t
    rate = (n - 1) / dur if dur > 0 else NOMINAL_RATE_HZ

    cols = [[s.dx for s in sel], [s.dy for s in sel], [s.dz for s in sel]]
    net = tuple(sum(c) for c in cols)
    step_rms = tuple(math.sqrt(sum(v * v for v in c) / n) for c in cols)
    vel_rms = tuple(r * rate for r in step_rms)
    max_step = tuple(max(abs(v) for v in c) for c in cols)
    mean_step = tuple(sum(c) / n for c in cols)
    n_reset = 0
    if resets:
        n_reset = sum(1 for t, _, _ in resets if t0 <= t <= t1)
    return WindowStats(n=n, duration=dur, net=net, step_rms=step_rms,
                       vel_rms=vel_rms, max_step=max_step,
                       mean_step=mean_step, resets=n_reset)


def fmt3(v: tuple[float, float, float], prec: int = 3) -> str:
    return "(" + ",".join(f"{x:+.{prec}f}" for x in v) + ")"


def render_session_md(name: str, series: OdomSeries,
                      windows: list[StateWindow],
                      events: list[tuple[float, str]]) -> str:
    """单会话 Markdown 报告段。"""
    lines = [f"### 会话 {name}", ""]
    lines.append(f"- delta 样本 {len(series.samples)} 个，时间锚点 "
                 f"{series.anchor_count} 个，EKF reset 事件 {len(series.resets)} 次")
    if not windows:
        lines.append("- control.log 无状态迁移（本会话未起飞或无飞行）")
    rows = []
    for w in windows:
        if w.t1 is None:
            continue
        st = window_stats(series, w.t0, w.t1, series.resets)
        if st is None:
            rows.append((w.state, w.t0, w.t1, None))
        else:
            rows.append((w.state, w.t0, w.t1, st))
    if rows:
        lines.append("")
        lines.append("| 窗口 | 时长s | 样本 | 净位移NED(m) | 速度RMS(m/s) "
                     "| 单步峰值(m) | reset |")
        lines.append("|---|---|---|---|---|---|---|")
        for state, t0, t1, st in rows:
            if st is None:
                lines.append(f"| {state} | {t1-t0:.1f} | <2 | - | - | - | - |")
                continue
            lines.append(
                f"| {state} | {st.duration:.1f} | {st.n} | "
                f"{fmt3(st.net)} | {fmt3(st.vel_rms, 2)} | "
                f"{fmt3(st.max_step, 2)} | {st.resets} |")
    if events:
        lines.append("")
        lines.append("关键事件：")
        for t, detail in events:
            lines.append(f"- t={t:.2f} {detail}")
    return "\n".join(lines)


def find_session_pairs(root: Path) -> list[tuple[str, Path, Path]]:
    """在目录里找 <前缀>_control.log / <前缀>_odom.log 配对。"""
    out = []
    for ctrl in sorted(root.rglob("*_control.log")):
        odom = ctrl.with_name(ctrl.name.replace("_control.log", "_odom.log"))
        if odom.exists():
            out.append((ctrl.name.replace("_control.log", ""), ctrl, odom))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="悬停漂移取证分析")
    ap.add_argument("paths", nargs="+", type=Path,
                    help="会话目录（自动配对 *_control.log/*_odom.log）或单个 odom 日志")
    args = ap.parse_args(argv)

    for p in args.paths:
        pairs = find_session_pairs(p) if p.is_dir() else []
        if not pairs and p.is_file():
            series = parse_odom_log(p)
            print(f"### {p.name}: 样本 {len(series.samples)}，"
                  f"reset {len(series.resets)} 次（无 control 配对）")
            continue
        for name, ctrl, odom in pairs:
            series = parse_odom_log(odom)
            windows, events = parse_control_log(ctrl)
            print(render_session_md(name, series, windows, events))
            print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
