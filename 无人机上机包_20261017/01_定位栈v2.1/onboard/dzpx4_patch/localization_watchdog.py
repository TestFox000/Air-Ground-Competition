#!/usr/bin/env python3
"""定位质量 watchdog（localization_watchdog.py，v2.1，2026-10-17）。

背景（真机日志回溯实锤）：机上既有 watchdog 只监控串口 / MicroXRCEAgent /
遥测链路，对定位质量零监控——EKF 每秒 reset 风暴持续 14 小时无人察觉。
本脚本补齐该缺口：周期跟踪 ~/drone_logs/node_odom_*.log 中 px4_odom_convert
v2.1 新增的两类结构化输出——

  [px4_odom_convert_health]   3s 周期健康行：guard 状态 / L1~L3 近期触发 /
                              EKF reset 速率 / SLAM 频率与跳变质量
  [px4_odom_convert_blackbox] 事件 4 行记录：L1 触发/恢复、L2 接受/节流、
                              L3 风暴起止

发现异常只输出醒目告警行（ERROR/WARN），不杀进程、不动节点——纯可观测性，
交给人/上游 watchdog 决策。健康行停更（节点死/SLAM 断流）同样告警。

部署（tools/patch_deploy.py 自动完成，无需手动）：
  推送到 ~/drone_tools/localization_watchdog.py 并 nohup setsid 启动，
  日志落 ~/drone_logs/localization_watchdog.log。

手动运行：
  python3 localization_watchdog.py --log-dir ~/drone_logs
  python3 localization_watchdog.py --log-dir ~/drone_logs --once   # 单轮调试

systemd 形态（如需开机自启，可选）：
  /etc/systemd/system/localization-watchdog.service
    [Unit]
    Description=px4_odom_convert localization quality watchdog
    After=network.target
    [Service]
    User=duzhong
    ExecStart=/usr/bin/python3 /home/duzhong/drone_tools/localization_watchdog.py \
              --log-dir /home/duzhong/drone_logs
    Restart=always
    RestartSec=5
    [Install]
    WantedBy=multi-user.target

本地等价测试：tests/test_localization_watchdog.py（健康行/blackbox 由
tests/test_blackbox_health_v2.py 的等价状态机生成，全链路无 ROS2 依赖）。
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import sys
import time

WATCH_TAG = "[localization_watchdog]"
HEALTH_PREFIX = "[px4_odom_convert_health]"
BB_PREFIX = "[px4_odom_convert_blackbox]"

_INT_RE = re.compile(r"^-?\d+$")


class WatchConfig:
    """告警阈值（与 jump guard v2 同源的门限注明出处）。"""

    def __init__(self):
        self.l1_sustain_s = 10.0      # L1 锁定持续该时长升级 ERROR（恢复需 2s，10s=异常）
        self.min_slam_hz = 5.0        # SLAM 断流/低频门限（正常 ~43Hz）
        self.health_stale_s = 10.0    # 健康行停更门限（正常 3s 一条）
        self.hold_ratio_warn = 0.5    # 单 3s 窗口 hold 占比预警
        self.ekf_reset_warn = 3       # 60s 窗 reset 预警线（风暴线=5，与 L3 同源）
        self.bump_rate_per_min = 6.0  # 重锚频率预警（L2 节流下正常 <=6/min）
        self.alert_repeat_s = 15.0    # 持续告警的重发间隔（防刷屏）


# ============================================================================
# 解析层（与 px4_odom_convert v2.1 输出格式一一对应）
# ============================================================================

def _coerce(v: str):
    if _INT_RE.match(v):
        return int(v)
    try:
        return float(v)
    except ValueError:
        return v


def parse_kv(text: str) -> dict:
    """'k=v k=v ...' -> dict（值自动转 int/float，其余保持字符串；
    含空格的值（如 delta_ned=[..]）只取首段，watchdog 不依赖该字段）。"""
    out = {}
    for tok in text.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = _coerce(v)
    return out


def parse_health_line(line: str):
    """解析健康行；非健康行返回 None。"""
    idx = line.find(HEALTH_PREFIX)
    if idx < 0:
        return None
    return parse_kv(line[idx + len(HEALTH_PREFIX):])


class BlackboxStream:
    """把连续 4 行 blackbox 记录聚合为一条事件 dict。

    行样式（writeBlackbox 固定输出）：
      [px4_odom_convert_blackbox] event=L1_TRIGGER t=... trans=... armed=...
      [px4_odom_convert_blackbox]   jump dpos_m=... dyaw_deg=... ...
      [px4_odom_convert_blackbox]   ekf ev_reset=... resets_60s=... ...
      [px4_odom_convert_blackbox]   slam_q win_max_m=... ...
    """

    def __init__(self):
        self.current = None

    def feed(self, line: str):
        """喂一行；一条记录（4 行）完整时返回事件 dict，否则 None。"""
        idx = line.find(BB_PREFIX)
        if idx < 0:
            return None
        rest = line[idx + len(BB_PREFIX):].strip()
        if rest.startswith("event="):
            self.current = parse_kv(rest)
            return None
        if self.current is not None and rest:
            section, _, kv = rest.partition(" ")
            self.current[section] = parse_kv(kv)
            if section == "slam_q":  # 第 4 行 -> 记录完成
                done = self.current
                self.current = None
                return done
        return None


# ============================================================================
# 判定层
# ============================================================================

def analyze_blackbox_event(ev: dict, cfg: WatchConfig):
    """blackbox 事件 -> 即时告警 (level, code, msg)；无告警返回 None。"""
    name = ev.get("event")
    if name == "L1_TRIGGER":
        d = ev.get("jump", {}).get("dpos_m", "?")
        return ("ERROR", "L1_TRIGGERED",
                f"SLAM 单帧跳变 {d}m 超熔断门限，L1 锁定输出（定位不可信，"
                f"blackbox t={ev.get('t')} 可归因）")
    if name == "L3_STORM_ON":
        n = ev.get("ekf", {}).get("resets_60s", "?")
        return ("ERROR", "EKF_STORM_ONSET",
                f"EKF reset 风暴置位：60s 窗 {n} 次，一切重锚已冻结"
                f"（10-17 实测同型风暴曾持续 14 小时无人察觉）")
    if name == "L1_RECOVER_INPLACE":
        return ("INFO", "L1_RECOVERED", "SLAM 原位恢复，L1 熔断解除")
    if name == "L1_RECOVER_REANCHOR":
        return ("INFO", "L2_REANCHORED", "SLAM 新锚点稳定，已平滑重锚")
    if name == "L2_ACCEPT":
        return ("INFO", "L2_REANCHORED", "真实大位移已接受并重锚")
    if name == "L2_THROTTLED":
        return ("INFO", "L2_THROTTLED",
                "大位移确认但 bump 节流中（输出保持稳定，10s 后复查）")
    if name == "L3_STORM_CLEAR":
        return ("INFO", "EKF_STORM_CLEARED", "EKF reset 风暴解除，恢复正常防护")
    return None


def analyze_health(h: dict, cfg: WatchConfig):
    """健康行 -> 持续性条件告警列表 [(level, code, msg)]。"""
    out = []
    guard = str(h.get("guard", ""))
    if "L1_LOCK" in guard:
        age = h.get("l1_age_s", -1.0)
        if isinstance(age, (int, float)) and age >= cfg.l1_sustain_s:
            out.append(("ERROR", "L1_LOCK",
                        f"SLAM 熔断(L1)已持续 {age:.0f}s——定位输出锁定不可信 "
                        f"(slam_hz={h.get('slam_hz')}, "
                        f"jump_max_m={h.get('jump_max_m')})"))
        else:
            out.append(("WARN", "L1_LOCK",
                        f"SLAM 熔断(L1)锁定中 (age={age}s)，等待稳定恢复"))
    if h.get("storm") == 1 or "STORM" in guard:
        out.append(("ERROR", "EKF_STORM_ACTIVE",
                    f"EKF reset 风暴中：60s 窗 {h.get('ekf_reset_60s')} 次 "
                    f"reset，一切重锚已冻结"))
    elif (isinstance(h.get("ekf_reset_60s"), int)
          and h["ekf_reset_60s"] >= cfg.ekf_reset_warn):
        out.append(("WARN", "EKF_RESET_ELEVATED",
                    f"EKF reset 偏多：60s 窗 {h['ekf_reset_60s']} 次"
                    f"（风暴线 5，与 L3 同源）"))
    hz = h.get("slam_hz")
    if isinstance(hz, (int, float)) and hz < cfg.min_slam_hz:
        out.append(("ERROR", "SLAM_STREAM_LOW",
                    f"SLAM 输出仅 {hz}Hz（正常 ~43Hz）——FAST-LIO 断流/卡死风险"))
    hr = h.get("hold_ratio")
    if isinstance(hr, (int, float)) and hr > cfg.hold_ratio_warn:
        out.append(("WARN", "HOLD_RATIO_HIGH",
                    f"guard hold 占比 {hr:.0%}——SLAM 质量劣化，大量帧被 hold"))
    return out


def node_process_alive():
    """扫 /proc 判断 px4_odom_convert 是否在跑；非 Linux/不可读返回 None
    （调用方按"无法判定"处理，不影响 HEALTH_STALE 告警）。"""
    try:
        pids = os.listdir("/proc")
    except OSError:
        return None
    for pid in pids:
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode("utf-8", "ignore")
        except OSError:
            continue
        if "px4_odom_convert" in cmd and "localization_watchdog" not in cmd:
            return True
    return False


# ============================================================================
# 状态机：告警去重 / 升级 / 恢复通知
# ============================================================================

class LocalizationWatchdog:
    """喂日志行 -> 产出告警行。wall 可注入（测试用假时钟）。"""

    def __init__(self, cfg=None, wall=None):
        self.cfg = cfg or WatchConfig()
        self.wall = wall or time.time
        self.bb = BlackboxStream()
        self.last_health = None
        self.last_health_wall = None
        self.bump_prev = None       # (bump_total, wall)
        self.active = {}            # code -> level（持续中的条件告警）
        self.last_emit = {}         # code -> wall（最近一次发出时间）
        self.started_wall = self.wall()
        self.emitted = []           # 全部输出行（测试/复盘用）

    def _emit(self, level, code, msg):
        ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.wall()))
        line = f"{WATCH_TAG} {ts} {level} [{code}] {msg}"
        self.emitted.append(line)
        return line

    def _update_persistent(self, desired):
        """desired：当前健康行推出的条件告警集。新增/升级/到重发间隔的
        发出；条件消失的补"已恢复"。返回本次产生的行。"""
        lines = []
        for level, code, msg in desired:
            if code not in self.active or level != self.active[code]:
                self.active[code] = level
                self.last_emit[code] = self.wall()
                lines.append(self._emit(level, code, msg))
            elif self.wall() - self.last_emit.get(code, 0) >= self.cfg.alert_repeat_s:
                self.last_emit[code] = self.wall()
                lines.append(self._emit(level, code, msg))
        dcodes = {c for _, c, _ in desired}
        for code in sorted(self.active):
            if code not in dcodes:
                del self.active[code]
                lines.append(self._emit("INFO", code, "已恢复，定位质量信号恢复正常"))
        return lines

    def _on_health(self, h):
        self.last_health = h
        w = self.wall()
        self.last_health_wall = w
        desired = analyze_health(h, self.cfg)
        # bump 速率（bump_total 差分，跨健康行）
        bt = h.get("bump_total")
        if isinstance(bt, (int, float)) and self.bump_prev is not None:
            pbt, pw = self.bump_prev
            dt = w - pw
            if dt > 0 and bt > pbt and (bt - pbt) / dt * 60.0 > self.cfg.bump_rate_per_min:
                rate = (bt - pbt) / dt * 60.0
                desired.append(("WARN", "BUMP_RATE_HIGH",
                                f"重锚过频：{rate:.1f} 次/分钟（L2 节流下正常 "
                                f"<= {self.cfg.bump_rate_per_min:.0f}/min）"))
        if isinstance(bt, (int, float)):
            self.bump_prev = (bt, w)
        return self._update_persistent(desired)

    def feed_line(self, line):
        """喂一行日志；返回本次产生的告警行列表。"""
        out = []
        h = parse_health_line(line)
        if h is not None:
            out.extend(self._on_health(h))
        ev = self.bb.feed(line)
        if ev is not None:
            alert = analyze_blackbox_event(ev, self.cfg)
            if alert:
                level, code, msg = alert
                out.append(self._emit(level, code, msg))
        return out

    def poll(self):
        """周期检查（每轮循环调用）：健康行停更 + 节点存活。"""
        lines = []
        w = self.wall()
        if self.last_health_wall is not None:
            gap = w - self.last_health_wall
        elif w - self.started_wall > self.cfg.health_stale_s:
            gap = w - self.started_wall
        else:
            gap = 0.0
        if gap > self.cfg.health_stale_s:
            alive = node_process_alive()
            if alive is False:
                code, msg = ("NODE_DOWN",
                             "px4_odom_convert 进程不存在且健康行停更——"
                             "定位链路已死，立即检查")
            else:
                code, msg = ("HEALTH_STALE",
                             f"健康行已 {gap:.0f}s 未更新（正常 3s 一条）——"
                             f"节点卡死或 SLAM 断流")
            if (code not in self.active or
                    w - self.last_emit.get(code, 0) >= self.cfg.alert_repeat_s):
                self.active[code] = "ERROR"
                self.last_emit[code] = w
                lines.append(self._emit("ERROR", code, msg))
        else:
            for code in ("HEALTH_STALE", "NODE_DOWN"):
                if code in self.active:
                    del self.active[code]
                    lines.append(self._emit("INFO", code,
                                            "已恢复，健康行恢复更新"))
        return lines


# ============================================================================
# 主循环：跟踪最新 node_odom_*.log 增量读取
# ============================================================================

def newest_node_log(log_dir):
    """最新 node_odom_*.log（按修改时间）；无则 None。"""
    logs = sorted(glob.glob(os.path.join(log_dir, "node_odom_*.log")),
                  key=os.path.getmtime)
    return logs[-1] if logs else None


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="localization_watchdog.py",
        description="px4_odom_convert 定位质量 watchdog（v2.1，被动观测，不杀进程）")
    ap.add_argument("--log-dir", default="~/drone_logs",
                    help="node_odom_*.log 所在目录（默认 ~/drone_logs）")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="扫描间隔秒（默认 1.0）")
    ap.add_argument("--once", action="store_true",
                    help="单轮扫描后退出（调试 / 接入既有 watchdog 拉起）")
    args = ap.parse_args(argv)

    log_dir = os.path.expanduser(args.log_dir)
    print(f"{WATCH_TAG} 启动：监控 {log_dir} 下 node_odom_*.log 的定位质量信号",
          flush=True)
    wd = LocalizationWatchdog()
    path = None
    fh = None
    offset = 0
    pending = ""

    try:
        while True:
            newest = newest_node_log(log_dir)
            if newest != path:
                if fh:
                    fh.close()
                path = newest
                offset = 0
                pending = ""
                fh = None
                if path:
                    print(f"{WATCH_TAG} 跟踪日志：{path}", flush=True)
            if fh is None and path:
                try:
                    fh = open(path, "rb")
                except OSError as e:
                    print(f"{WATCH_TAG} WARN [LOG_OPEN] 打开日志失败：{e}",
                          flush=True)
            if fh:
                fh.seek(offset)
                chunk = fh.read()
                if chunk:
                    offset = fh.tell()
                    pending += chunk.decode("utf-8", "replace")
                    while "\n" in pending:
                        raw, pending = pending.split("\n", 1)
                        for line in wd.feed_line(raw):
                            print(line, flush=True)
            for line in wd.poll():
                print(line, flush=True)
            if args.once:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print(f"{WATCH_TAG} 退出", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
