#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AimBench v1 — 云台自瞄调试台架（车端单文件，一个命令 = 一个可归档实验）。

为什么要有它
------------
过去调云台是「SSH 上去 → 改 yaml → gimbal_tune.sh restart(18s) → 手推靶/发假话题
→ 若干条 topic echo → 再写一个一次性脚本分析」。10-17 现场 7 次采样全部被 /tmp
覆盖丢失，每个结论都无法复现。这个工具把一次实验变成标准化产物：

    run_dir/
      meta.json          场景 / 参数快照 / 安全动作 / 代码哈希 / 主机信息
      data.csv           全链路同步时序（视觉、反馈、指令、状态、开火、真值）
      params_*.yaml      ros2 param dump（如可用）
      quick_metrics.json 车端即时统计（详细报告在 PC 上用 bench_analyze.py 生成）

三种模式
--------
  selftest  无 ROS。用简化云台模型生成一份合成 HIL 数据，用于本地验证分析器。
  capture   只读录制真实链路（det_node + 云台），不发任何控制指令。
  hil       无靶闭环台架：停掉 4_det，用真实云台反馈合成 offset_center，按脚本注入
            目标/车体转动/遮挡/类别切换，同时录全部信号。用于调 PID、lead、smooth、
            状态机 —— 不需要真靶、不消耗弹药。真实云台仍会转动！

统一 CSV（长表）
---------------
  t,kind,a,b,c,d,e,f
  vis   dx, dy, cls, box_w, box_h
  fb    pitch_tick, yaw_tick, speed_pitch, speed_yaw, load_pitch, load_yaw
  cmd   pitch_cmd, yaw_cmd, speed_pitch, speed_yaw
  st    aim_status(0/1/2)
  ref   target_beta_deg, base_psi_deg, gimbal_deg, track_err_deg, dx_published, dx_raw
  lead  lead_px
  gate  fire_gate, fire_ready
  shot  shoot_count
  laser 1（只在开火脉冲出现）

安全
----
  * hil 若检测到 /LaserShot_Command 有订阅者（dzjudgment 在跑），默认先停 dzjudgment，
    实验结束再拉起；--allow-laser 可显式允许出光。
  * hil 默认停 4_det（否则两路 offset_center 混在一起），结束恢复。
  * 以上动作都写进 meta.json 的 safety_actions，可审计。

用法示例（车端，已 source ROS 环境）
------------------------------------
  python3 aim_bench.py selftest --scenario ramp --seconds 20 --out runs/selftest
  python3 aim_bench.py capture  --seconds 60 --tag real_target_baseline
  python3 aim_bench.py hil      --scenario ramp --seconds 25 --latency-ms 267 --tag kp10
  python3 aim_bench.py hil      --scenario base_rotate --seconds 20 --tag gyroff
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import random
import re
import signal
import statistics
import subprocess
import sys
import time

TOOL_VERSION = "aim-bench/1.0"
CSV_HEADER = ["t", "kind", "a", "b", "c", "d", "e", "f"]
KINDS = ("vis", "fb", "cmd", "st", "ref", "lead", "gate", "shot", "laser")

CENTER_TICK = 2047.0
TICKS_PER_DEG = 11.378
# px_per_deg = px_per_tick(0.94) * ticks_per_deg(11.378) ≈ 10.695
PX_PER_DEG = 0.94 * TICKS_PER_DEG

_DEFAULT_PID = os.path.expanduser("~/robot_logs/pids")
_DEFAULT_LOG = os.path.expanduser("~/robot_logs")


# ======================================================================
# 场景（纯数学，无 ROS）
# ======================================================================
SCENARIOS = {
    "static": dict(kind="static", amp=0.0, speed=0.0,
                   desc="静止靶 0°，检查稳态中心偏置/抖动"),
    "step": dict(kind="step", amp=15.0,
                 desc="0°→+15°→-15° 阶跃，检查超调/稳定时间"),
    "ramp": dict(kind="ramp", amp=15.0, speed=12.0,
                 desc="三角波 ±15° @12°/s（约等于轨道靶 4cm/s@0.7m）"),
    "rail": dict(kind="ramp", amp=25.0, speed=3.3,
                 desc="★真实轨道靶口径：±25° @3.3°/s（4cm/s@0.7m，atan(4/70)≈3.28°/s；±25°≈0.7m 轨道全幅）"),
    "ramp_fast": dict(kind="ramp", amp=20.0, speed=30.0,
                      desc="三角波 ±20° @30°/s，检查大机动余量"),
    "sine": dict(kind="sine", amp=15.0, freq=0.1,
                 desc="正弦 ±15° @0.1Hz，检查相位滞后"),
    "sine_fast": dict(kind="sine", amp=10.0, freq=0.5,
                      desc="正弦 ±10° @0.5Hz，检查跟踪带宽"),
    "stopgo": dict(kind="stopgo", amp=15.0, speed=12.0, dwell=1.0,
                   desc="轨道端停 1s 的往复运动，检查折返窗口/超调"),
    "occlusion": dict(kind="occlusion", amp=15.0, speed=12.0,
                      drop_period=3.0, drop=0.30,
                      desc="ramp + 每 3s 丢 0.3s，检查预测滑行/重捕"),
    "class_switch": dict(kind="class_switch",
                         desc="静止靶但类别 1/2 每秒切换，检查类别锁"),
    "base_rotate": dict(kind="base_rotate", amp=15.0, freq=0.5,
                        desc="靶惯性系静止、车体 ±15°@0.5Hz 转，检查 gyro_ff"),
    "combo": dict(kind="combo", amp=15.0, speed=12.0,
                  psi_amp=10.0, psi_freq=0.3,
                  desc="目标三角波 + 车体正弦转，行进间锁靶近似"),
}


def scenario_cfg(name):
    return dict(SCENARIOS.get(name, SCENARIOS["ramp"]))


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _triangle(t, amp, speed):
    """三角波：t=0 在 -amp，半周期到 +amp；斜率 = speed（度/s）。"""
    amp = max(abs(amp), 1e-6)
    speed = max(abs(speed), 1e-6)
    period = 4.0 * amp / speed
    ph = (t % period) / period
    if ph < 0.5:
        return -amp + 4.0 * amp * ph
    return amp - 4.0 * amp * (ph - 0.5)


def _tri_from_zero(t, amp, speed):
    """三角波，但 t=0 时目标在 0°（更像轨道中央起步，避免开头大阶跃污染指标）。"""
    return _triangle(t + max(abs(amp), 1e-6) / max(abs(speed), 1e-6), amp, speed)


def _stopgo(t, amp, speed, dwell):
    amp = max(abs(amp), 1e-6)
    speed = max(abs(speed), 1e-6)
    seg = amp / speed
    period = 4.0 * seg + 2.0 * dwell
    p = t % period
    if p < seg:
        return -amp + speed * p
    p -= seg
    if p < dwell:
        return amp
    p -= dwell
    if p < 2.0 * seg:
        return amp - speed * p
    p -= 2.0 * seg
    if p < dwell:
        return -amp
    p -= dwell
    return -amp + speed * p


def target_beta_deg(sc, t):
    """目标在惯性系的方位角（度）。"""
    kind = sc["kind"]
    if kind == "static":
        return 0.0
    if kind == "step":
        amp = sc["amp"]
        if t < 1.5:
            return 0.0
        if t < 3.5:
            return amp
        if t < 5.5:
            return -amp
        return 0.0
    if kind in ("ramp", "occlusion", "combo"):
        return _tri_from_zero(t, sc["amp"], sc["speed"])
    if kind == "ramp_fast":
        return _tri_from_zero(t, sc["amp"], sc["speed"])
    if kind == "sine":
        return sc["amp"] * math.sin(2.0 * math.pi * sc["freq"] * t)
    if kind == "sine_fast":
        return sc["amp"] * math.sin(2.0 * math.pi * sc["freq"] * t)
    if kind == "stopgo":
        return _stopgo(t, sc["amp"], sc["speed"], sc["dwell"])
    if kind == "class_switch":
        return 0.0
    if kind == "base_rotate":
        return 0.0
    return _triangle(t, sc.get("amp", 15.0), sc.get("speed", 12.0))


def base_psi_deg(sc, t):
    """车体（相机基座）惯性偏航角（度）。"""
    if sc["kind"] == "base_rotate":
        return sc["amp"] * math.sin(2.0 * math.pi * sc["freq"] * t)
    if sc["kind"] == "combo":
        return sc["psi_amp"] * math.sin(2.0 * math.pi * sc["psi_freq"] * t)
    return 0.0


def class_for(sc, t):
    if sc["kind"] == "class_switch":
        return 1 if int(t) % 2 == 0 else 2
    return 1  # 蓝靶


def is_dropout(sc, t):
    if sc["kind"] != "occlusion":
        return False
    period = sc["drop_period"]
    return (t % period) > (period - sc["drop"])


def scenario_meta(name):
    sc = scenario_cfg(name)
    return {"name": name, "desc": sc.get("desc", ""),
            "params": {k: v for k, v in sc.items() if k not in ("kind", "desc")},
            "kind": sc["kind"]}


# ======================================================================
# 进程 / 文件公共工具
# ======================================================================
def _pid_of(pidfile):
    try:
        with open(os.path.expanduser(pidfile), "r", encoding="utf-8") as fh:
            return int(fh.read().strip())
    except Exception:
        return None


def _pid_cmdline(pid):
    """Linux /proc/<pid>/cmdline；进程不存在返回 None。"""
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as fh:
            return fh.read().replace(b"\x00", b" ").decode("utf-8", "replace")
    except Exception:
        return None


def _group_alive(pid):
    try:
        os.killpg(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except Exception:
        # 非进程组时退回单 pid 探测
        try:
            os.kill(pid, 0)
            return True
        except Exception:
            return False


def stop_from_pidfile(pidfile, expect=None):
    """按 pid 文件停掉整个进程组；返回被停的 pid 或 None。

    - 先校验 /proc/<pid>/cmdline 含 expect（防 pid 被复用误杀）；
    - 只有确实杀掉/进程已不在时才删 pid 文件；
    - 杀不掉/校验失败返回 None，并保留 pid 文件供人工处理。
    """
    pidfile = os.path.expanduser(pidfile)
    pid = _pid_of(pidfile)
    if pid is None:
        return None
    cmd = _pid_cmdline(pid)
    if cmd is None:
        # 进程已经不在了：清掉陈旧 pid 文件
        try:
            os.remove(pidfile)
        except Exception:
            pass
        return None
    if expect and expect not in cmd:
        print("[aim_bench] pid 文件 %s 指向 pid=%d 但 cmdline 不含 '%s'，拒绝误杀：%s"
              % (pidfile, pid, expect, cmd[:160]), file=sys.stderr)
        return None
    killed = False
    try:
        os.killpg(pid, signal.SIGTERM)
        killed = True
    except ProcessLookupError:
        pass
    except Exception:
        try:
            os.kill(pid, signal.SIGTERM)
            killed = True
        except Exception:
            pass
    time.sleep(1.5)
    if _group_alive(pid):
        try:
            os.killpg(pid, signal.SIGKILL)
            killed = True
        except Exception:
            try:
                os.kill(pid, signal.SIGKILL)
                killed = True
            except Exception:
                pass
        time.sleep(0.5)
    if _group_alive(pid):
        print("[aim_bench] 进程 pid=%d 仍存活，未删除 pid 文件 %s" % (pid, pidfile), file=sys.stderr)
        return None
    try:
        os.remove(pidfile)
    except Exception:
        pass
    return pid if killed else None


def spawn_detached(cmd, logpath, pidfile, cwd=None, env_extra=None):
    """按项目习惯 setsid 起进程并写 pid 文件。返回 pid。"""
    logpath = os.path.expanduser(logpath)
    pidfile = os.path.expanduser(pidfile)
    os.makedirs(os.path.dirname(logpath), exist_ok=True)
    os.makedirs(os.path.dirname(pidfile), exist_ok=True)
    env = os.environ.copy()
    env.update(env_extra or {})
    log = open(logpath, "a", encoding="utf-8", errors="replace")
    proc = subprocess.Popen(["bash", "-lc", cmd], stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True,
                            cwd=os.path.expanduser(cwd) if cwd else None, env=env)
    with open(pidfile, "w", encoding="utf-8") as fh:
        fh.write(str(proc.pid))
    return proc.pid


def parse_param_snapshot(path):
    """从 ros2 param dump 的 YAML 里解析 ros__parameters 标量（不依赖 PyYAML）。"""
    vals = {}
    if not path:
        return vals
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            text = fh.read()
    except Exception:
        return vals
    in_params = False
    for line in text.splitlines():
        if re.match(r"^\s*ros__parameters:\s*$", line):
            in_params = True
            continue
        if not in_params:
            continue
        m = re.match(r"^ {2,}([A-Za-z_][A-Za-z0-9_]*):\s*(.*?)\s*$", line)
        if not m:
            continue
        key, raw = m.group(1), m.group(2)
        if raw in ("true", "false"):
            vals[key] = (raw == "true")
            continue
        try:
            vals[key] = float(raw)
            continue
        except Exception:
            pass
        vals[key] = raw.strip("'\"")
    return vals


def dump_params(outdir):
    """ros2 param dump 快照。返回 {node: filename}。"""
    files = {}
    os.makedirs(outdir, exist_ok=True)
    # 清掉上一次同名 run 遗留的 params 快照，避免"参数看起来是这轮的"
    try:
        for fn in os.listdir(outdir):
            if fn.startswith("params_") and fn.endswith(".yaml"):
                try:
                    os.remove(os.path.join(outdir, fn))
                except Exception:
                    pass
    except Exception:
        pass
    for node in ("/dzrknn_dataprocess", "/det_node"):
        try:
            r = subprocess.run(["ros2", "param", "dump", node],
                               capture_output=True, text=True, timeout=15)
        except Exception:
            continue
        if r.returncode == 0 and r.stdout.strip():
            fname = "params_" + node.strip("/").replace("/", "_") + ".yaml"
            with open(os.path.join(outdir, fname), "w", encoding="utf-8") as fh:
                fh.write(r.stdout)
            files[node] = fname
    return files


def _self_hash():
    try:
        with open(os.path.abspath(__file__), "rb") as fh:
            return hashlib.sha256(fh.read()).hexdigest()[:16]
    except Exception:
        return "?"


def _host_info():
    import platform
    import socket
    return {"hostname": socket.gethostname(), "platform": platform.platform(),
            "python": sys.version.split()[0]}


def default_out(mode, tag):
    stamp = time.strftime("%Y%m%d_%H%M%S")
    safe = "".join(c if (c.isalnum() or c in "-_.") else "_" for c in tag)[:40]
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs",
                        "%s_%s_%s" % (stamp, mode, safe))


def write_run(outdir, rows, meta, quick):
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "data.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(CSV_HEADER)
        w.writerows(rows)
    with open(os.path.join(outdir, "meta.json"), "w", encoding="utf-8") as fh:
        json.dump(meta, fh, ensure_ascii=False, indent=2)
    with open(os.path.join(outdir, "quick_metrics.json"), "w", encoding="utf-8") as fh:
        json.dump(quick, fh, ensure_ascii=False, indent=2)
    return outdir


# ======================================================================
# 车端即时统计（不依赖 numpy/matplotlib）
# ======================================================================
def _num(x):
    try:
        return float(x)
    except Exception:
        return None


def quick_metrics(rows, mode):
    out = {"mode": mode, "rows": len(rows)}
    end_t = 0.0
    for r in rows:
        v = _num(r[0])
        if v is not None and v > end_t:
            end_t = v
    st = [(float(r[0]), int(float(r[2]))) for r in rows
          if r[1] == "st" and _num(r[2]) is not None]
    if st:
        trans = []
        last = None
        for t, s in st:
            if s != last:
                trans.append({"t": round(t, 2), "state": s})
                last = s
        out["state_transitions"] = trans
        total = 0.0
        for i, (t, s) in enumerate(st):
            t_next = st[i + 1][0] if i + 1 < len(st) else max(end_t, t)
            if s == 2:
                total += max(0.0, t_next - t)
        out["track_seconds"] = round(total, 2)
    ref = [r for r in rows if r[1] == "ref"]
    if ref:
        errs = [abs(float(r[5])) for r in ref if _num(r[5]) is not None]
        if errs:
            out["track_err_deg"] = {
                "n": len(errs),
                "mean_abs": round(statistics.fmean(errs), 3),
                "p90": round(sorted(errs)[int(0.9 * (len(errs) - 1))], 3),
                "max": round(max(errs), 3),
            }
    shots = [(float(r[0]), int(float(r[2]))) for r in rows
             if r[1] == "shot" and _num(r[2]) is not None]
    if shots:
        out["shots"] = shots[-1][1]
        out["first_shot_t"] = round(shots[0][0], 2) if shots[-1][1] > 0 else None
    return out


# ======================================================================
# selftest：无 ROS 的合成 HIL 运行
# ======================================================================
def run_selftest(args):
    sc = scenario_cfg(args.scenario)
    tpd, center = TICKS_PER_DEG, CENTER_TICK
    rows = []
    dt = 1.0 / args.rate_hz
    g = 0.0          # 真云台相对角（度）
    gdot = 0.0       # 云台角速度（度/s）
    rate_limit = 176.0
    last_shot_t = -10.0
    shots = 0
    track_t0 = None
    lat = max(0.0, args.latency_ms / 1000.0)
    buf = []
    prev_dx = None
    prev_beta = None
    prev_t = None
    d_filt = 0.0
    rnd = random.Random(7)
    started_mono = time.time()
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S")
    t = 0.0
    while t < args.seconds:
        beta = target_beta_deg(sc, t)
        psi = base_psi_deg(sc, t)
        if prev_beta is None or prev_t is None:
            dbdt = 0.0
        else:
            dbdt = (beta - prev_beta) / max(1e-6, t - prev_t)
        prev_beta, prev_t = beta, t
        err = beta - psi - g
        drop = is_dropout(sc, t)
        if drop:
            dx_raw, cls = 0.0, 0
        else:
            dx_raw = PX_PER_DEG * err + rnd.gauss(0.0, args.noise_px)
            cls = class_for(sc, t)
        buf.append((t, dx_raw))
        if lat > 0:
            while len(buf) >= 2 and buf[1][0] <= t - lat:
                buf.pop(0)
            dx_pub = buf[0][1] if buf and buf[0][0] <= t - lat else dx_raw
        else:
            dx_pub = dx_raw
            if len(buf) > 200:
                buf = buf[-100:]
        # 简化版节点：P + 一阶 D + 提前量（近似真实链路的 lead 前馈）
        lead_px = dbdt * PX_PER_DEG * getattr(args, "lead_time_s", 0.0)
        ctrl_px = dx_pub + lead_px
        d_raw = 0.0 if prev_dx is None else (dx_pub - prev_dx) / dt
        d_filt = 0.3 * d_filt + 0.7 * d_raw
        cmd_rate = _clamp(0.40 * ctrl_px + 0.03 * d_filt, -rate_limit, rate_limit)
        gdot += (cmd_rate - gdot) * _clamp(dt / 0.08, 0.0, 1.0)
        gdot = _clamp(gdot, -rate_limit, rate_limit)
        g += gdot * dt
        prev_dx = dx_pub
        if t >= 2.0 and track_t0 is None:
            track_t0 = t
        state = 2 if track_t0 is not None else 1
        pitch = int(center)
        yaw = int(round(center + g * tpd))
        rows.append([f"{t:.4f}", "st", state, "", "", "", "", ""])
        rows.append([f"{t:.4f}", "vis", int(round(dx_pub)), 0, cls, 60, 60, ""])
        rows.append([f"{t:.4f}", "ref", f"{beta:.4f}", f"{psi:.4f}", f"{g:.4f}",
                     f"{err:.4f}", f"{dx_pub:.2f}", f"{dx_raw:.2f}"])
        if getattr(args, "lead_time_s", 0.0) > 0:
            rows.append([f"{t:.4f}", "lead", round(lead_px, 2), "", "", "", "", ""])
        rows.append([f"{t:.4f}", "fb", pitch, yaw, int(round(abs(gdot) * tpd)),
                     int(round(gdot * tpd)), 0, 0])
        rows.append([f"{t:.4f}", "cmd", pitch, yaw, int(round(abs(gdot) * tpd)),
                     int(round(gdot * tpd)), "", ""])
        # 假开火：瞄准误差（含提前量）进入动态框且冷却到（近似节点判据）
        if state == 2 and abs(ctrl_px) <= 18.0 and (t - last_shot_t) >= 0.8 and t > 4.0:
            shots += 1
            last_shot_t = t
            rows.append([f"{t:.4f}", "shot", shots, "", "", "", "", ""])
        t += dt
    outdir = args.out or default_out("selftest", args.scenario)
    meta = {
        "tool": TOOL_VERSION, "mode": "selftest", "scenario": scenario_meta(args.scenario),
        "args": vars(args), "host": _host_info(), "started_at": started_at,
        "ended_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "duration_requested": args.seconds,
        "duration_actual": round(time.time() - started_mono, 3),
        "code_sha256_16": _self_hash(),
        "note": "无 ROS 的合成数据：云台/控制律用简化模型，用于本地验证分析器/画图；不代表真实控制环。",
    }
    quick = quick_metrics(rows, "selftest")
    write_run(outdir, rows, meta, quick)
    print("[selftest] %d rows -> %s" % (len(rows), outdir))
    print("[selftest] quick:", json.dumps(quick, ensure_ascii=False))
    return 0


# ======================================================================
# ROS 模式
# ======================================================================
def _run_ros(args):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.executors import ExternalShutdownException
        from std_msgs.msg import Float32, Int32, Int32MultiArray, UInt8
        from sensor_msgs.msg import Imu
    except Exception as exc:  # noqa: BLE001
        print("ROS 模式需要 rclpy/ROS2 环境：%s" % exc, file=sys.stderr)
        print("车上先 source /opt/ros/humble/setup.bash 与 ~/dzrobot/install/setup.bash", file=sys.stderr)
        return 2

    be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST)
    sc = scenario_cfg(args.scenario) if args.cmd == "hil" else None
    mode = args.cmd

    class BenchNode(Node):
        def __init__(self):
            super().__init__("aim_bench")
            self.mode = mode
            self.recording = False
            self.rows = []
            self.t0 = time.time()
            self.fb = [CENTER_TICK, CENTER_TICK, 0, 0, 0, 0]
            self.cmd = [CENTER_TICK, CENTER_TICK, 0, 0]
            self.aim_state = 0
            self.last_state = None
            self.last_shot = None
            self.gate = (0, 0)
            self.safety_actions = []
            self.params_files = {}
            self.param_values = {}
            self.effective = {}
            self.det_pid = None
            self.judgment_pid = None
            self.start_scan = (mode == "hil") or (getattr(args, "scan", None) is not None)
            self.prev_beta = None
            self.last_tick = None
            self.raw_buf = []
            self.last_gate_pub = None
            self.gates_on = bool(getattr(args, "gates", False))
            self.pub_imu = None
            self.last_fb_t = None
            self.feedback_lost = False
            self.laser_unsafe = False
            self.stop_requested = False
            self.started_at = None
            self.started_mono = None
            self.scan_started = False
            self.owns_outputs = False
            self.yaw_sign = 1
            self.yaw_center = CENTER_TICK
            self.ticks_per_deg = TICKS_PER_DEG
            self.px_per_deg = PX_PER_DEG
            self._derived = False

            self.pub_scan = self.create_publisher(UInt8, "/nav/scan_request", 10)
            self.pub_reset = self.create_publisher(UInt8, "/shoot_reset", 10)
            if mode == "hil":
                self.pub_vis = self.create_publisher(Int32MultiArray, "offset_center", be)
                self.pub_lead = self.create_publisher(Float32, "/aim/lead_x", be)
                self.pub_gate = self.create_publisher(UInt8, "/aim/fire_gate", 10)
                self.pub_ready = self.create_publisher(UInt8, "/aim/fire_ready", 10)
                self.pub_laser_switch = self.create_publisher(UInt8, "/laser_switch", 10)
                if getattr(args, "fake_imu", False) and sc["kind"] in ("base_rotate", "combo"):
                    self.pub_imu = self.create_publisher(Imu, args.imu_topic, 10)

            # 注意：hil 模式不要订阅 offset_center / lead / 两把钥匙 —— 这些是本节点自己发的，
            # 同进程自收会造成重复行；HIL 的 vis/lead/gate 行在 _on_tick 里显式记录。
            if mode == "capture":
                self.create_subscription(Int32MultiArray, "offset_center", self._on_vis, be)
                self.create_subscription(Float32, "/aim/lead_x", self._on_lead, be)
                self.create_subscription(UInt8, "/aim/fire_gate", self._on_gate, 10)
                self.create_subscription(UInt8, "/aim/fire_ready", self._on_ready, 10)
            self.create_subscription(Int32MultiArray, "camemo_feedback", self._on_fb, be)
            self.create_subscription(Int32MultiArray, "CameMo_Conl", self._on_cmd, 50)
            self.create_subscription(Int32, "/aim/status", self._on_st, 10)
            self.create_subscription(Int32, "/shoot_count", self._on_shot, 10)
            self.create_subscription(UInt8, "LaserShot_Command", self._on_laser, 10)
            self.create_timer(1.0 / args.rate_hz, self._on_tick)
            if mode == "hil":
                self.create_timer(0.5, self._check_laser)
            if self.pub_imu is not None:
                self.create_timer(0.02, self._on_imu)

        # ---------- rclpy graph：区分"本节点自己的端点"与"外来端点" ----------
        def _is_own(self, info):
            try:
                return (info.node_name == self.get_name()
                        and info.node_namespace.rstrip("/") == self.get_namespace().rstrip("/"))
            except Exception:
                return False

        def _names(self, infos):
            out = []
            for i in infos:
                try:
                    out.append("%s/%s" % (i.node_namespace.rstrip("/"), i.node_name))
                except Exception:
                    out.append(str(i))
            return out

        def _foreign_pubs(self, topic):
            try:
                infos = self.get_publishers_info_by_topic(self.resolve_topic_name(topic))
            except Exception:
                return []
            return self._names([i for i in infos if not self._is_own(i)])

        def _foreign_subs(self, topic):
            try:
                infos = self.get_subscriptions_info_by_topic(self.resolve_topic_name(topic))
            except Exception:
                return []
            return self._names([i for i in infos if not self._is_own(i)])

        def _check_laser(self):
            if self.mode != "hil" or getattr(args, "allow_laser", False):
                return
            if self.recording and self._foreign_subs("LaserShot_Command"):
                self.laser_unsafe = True

        # ---------- 记录 ----------
        def _t(self):
            return time.time() - self.t0

        def _row(self, kind, a=None, b=None, c=None, d=None, e=None, f=None):
            if not self.recording:
                return
            self.rows.append([f"{self._t():.4f}", kind,
                              "" if a is None else a, "" if b is None else b,
                              "" if c is None else c, "" if d is None else d,
                              "" if e is None else e, "" if f is None else f])

        def _on_vis(self, m):
            if not list(m.data):
                return
            d = list(m.data) + [0] * 8
            self._row("vis", d[0], d[1], d[2], d[3], d[4])

        def _on_fb(self, m):
            d = list(m.data)
            if len(d) < 6:
                return
            self.fb = d[:6]
            self.last_fb_t = self._t()
            self._row("fb", *d[:6])

        def _on_cmd(self, m):
            if not list(m.data):
                return
            d = list(m.data) + [0] * 4
            self.cmd = d[:4]
            self._row("cmd", *d[:4])

        def _on_st(self, m):
            self.aim_state = int(m.data)
            if m.data != self.last_state:
                self.last_state = m.data
                self._row("st", m.data)

        def _on_lead(self, m):
            self._row("lead", float(m.data))

        def _on_gate(self, m):
            self.gate = (int(m.data), self.gate[1])
            self._row("gate", self.gate[0], self.gate[1])

        def _on_ready(self, m):
            self.gate = (self.gate[0], int(m.data))
            self._row("gate", self.gate[0], self.gate[1])

        def _on_shot(self, m):
            if m.data != self.last_shot:
                self.last_shot = m.data
                self._row("shot", int(m.data))

        def _on_laser(self, m):
            if int(m.data) == 1:
                self._row("laser", 1)

        # ---------- HIL 合成视觉 ----------
        def _on_tick(self):
            if self.mode != "hil" or not self.recording:
                return
            t = self._t()
            if self.last_fb_t is None or (t - self.last_fb_t) > 0.30:
                self.feedback_lost = True
                return
            beta = target_beta_deg(sc, t)
            psi = base_psi_deg(sc, t)
            g = self.yaw_sign * (self.fb[1] - self.yaw_center) / self.ticks_per_deg
            err = beta - psi - g
            drop = is_dropout(sc, t)
            if drop:
                dx_raw, dy_raw, cls, box = 0.0, 0.0, 0, 0
            else:
                dx_raw = self.px_per_deg * err + random.gauss(0.0, args.noise_px)
                dy_raw = random.gauss(0.0, args.noise_px * 0.5)
                cls = class_for(sc, t)
                box = 60
            # 链路延迟：只延迟视觉值，不回传采样时间
            self.raw_buf.append((t, dx_raw, dy_raw, cls, box))
            lat = max(0.0, args.latency_ms / 1000.0)
            if lat > 0:
                while len(self.raw_buf) >= 2 and self.raw_buf[1][0] <= t - lat:
                    self.raw_buf.pop(0)
                if self.raw_buf and self.raw_buf[0][0] <= t - lat:
                    _, dx_pub, dy_pub, cls, box = self.raw_buf[0]
                else:
                    dx_pub, dy_pub = dx_raw, dy_raw
            else:
                dx_pub, dy_pub = dx_raw, dy_raw
                if len(self.raw_buf) > 200:
                    self.raw_buf = self.raw_buf[-100:]

            msg = Int32MultiArray()
            msg.data = [int(round(dx_pub)), int(round(dy_pub)), int(cls),
                        int(box), int(box), 0, 0, 90]
            self.pub_vis.publish(msg)
            self._row("vis", int(round(dx_pub)), int(round(dy_pub)), int(cls),
                      int(box), int(box))
            self._row("ref", f"{beta:.4f}", f"{psi:.4f}", f"{g:.4f}", f"{err:.4f}",
                      f"{dx_pub:.2f}", f"{dx_raw:.2f}")

            # 提前量：模拟 dzbrain 的 /aim/lead_x（0 = 不发布）
            if args.lead_time_s > 0:
                if self.prev_beta is None or self.last_tick is None:
                    dbdt = 0.0
                else:
                    dbdt = (beta - self.prev_beta) / max(1e-6, t - self.last_tick)
                lead_px = _clamp(dbdt * self.px_per_deg * args.lead_time_s, -60.0, 60.0)
                self.pub_lead.publish(Float32(data=float(lead_px)))
                self._row("lead", round(lead_px, 2))
            self.prev_beta = beta
            self.last_tick = t

            # 两把钥匙（模拟 dzbrain）：默认关，--gates 打开
            if self.gates_on and t >= args.gate_after:
                want = (1, 1)
                if self.last_gate_pub != want:
                    self.last_gate_pub = want
                    self.pub_gate.publish(UInt8(data=1))
                    self.pub_ready.publish(UInt8(data=1))
                    self._row("gate", 1, 1)

        def _on_imu(self):
            if self.mode != "hil" or not self.recording:
                return
            t = self._t()
            eps = 1e-3
            psi1 = base_psi_deg(sc, t - eps)
            psi2 = base_psi_deg(sc, t + eps)
            wz = math.radians((psi2 - psi1) / (2.0 * eps)) if sc["kind"] in ("base_rotate", "combo") else 0.0
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.angular_velocity.z = float(wz)
            self.pub_imu.publish(msg)

        # ---------- 准备/收尾 ----------
        def _spin(self, seconds):
            end = time.time() + seconds
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.02)

        def _wait_until(self, pred, timeout, desc):
            end = time.time() + timeout
            while time.time() < end:
                if pred():
                    return True
                self._spin(0.1)
            return pred()

        def _clear_fire_latches(self):
            """清掉本节点发过的 latch：fire_gate / fire_ready / lead_x。"""
            if self.mode != "hil" or not self.owns_outputs:
                return
            try:
                self.pub_gate.publish(UInt8(data=0))
                self.pub_ready.publish(UInt8(data=0))
                self.pub_lead.publish(Float32(data=0.0))
                self._spin(0.1)
            except Exception:
                pass

        def _clear_all(self):
            try:
                if self.scan_started:
                    self.pub_scan.publish(UInt8(data=0))
                    self._spin(0.15)
            except Exception:
                pass
            self._clear_fire_latches()
            # HIL 没打真弹，结束时把软件弹药计数复位，避免污染下一次真实链路
            if self.mode == "hil" and self.owns_outputs and not getattr(args, "allow_laser", False):
                try:
                    self.pub_laser_switch.publish(UInt8(data=0))
                    self.pub_reset.publish(UInt8(data=1))
                    self._spin(0.05)
                except Exception:
                    pass

        def _derive_geometry(self):
            """从参数快照自动取几何量；CLI 显式值优先。"""
            pv = self.param_values
            try:
                if getattr(args, "yaw_sign", None) is not None:
                    self.yaw_sign = int(args.yaw_sign)
                elif "yaw_deg_sign" in pv:
                    self.yaw_sign = 1 if float(pv["yaw_deg_sign"]) >= 0 else -1
                else:
                    self.yaw_sign = 1
                self.yaw_center = (float(args.yaw_center_tick) if getattr(args, "yaw_center_tick", None) is not None
                                   else float(pv.get("gimbal_center_tick", CENTER_TICK)))
                self.ticks_per_deg = (float(args.yaw_ticks_per_deg) if getattr(args, "yaw_ticks_per_deg", None) is not None
                                      else float(pv.get("yaw_ticks_per_deg", TICKS_PER_DEG)))
                if self.ticks_per_deg <= 1.0:
                    self.ticks_per_deg = TICKS_PER_DEG
                if getattr(args, "px_per_deg", None) is not None:
                    self.px_per_deg = float(args.px_per_deg)
                elif getattr(args, "px_per_tick", None) is not None:
                    self.px_per_deg = float(args.px_per_tick) * self.ticks_per_deg
                else:
                    pxpt = float(pv.get("px_per_tick", 0.94) or 0.94)
                    self.px_per_deg = pxpt * self.ticks_per_deg
            except Exception as exc:  # noqa: BLE001
                self.safety_actions.append("几何参数自动推导失败，用默认值：%s" % exc)
            self.effective = {
                "yaw_sign": self.yaw_sign,
                "yaw_center_tick": self.yaw_center,
                "yaw_ticks_per_deg": self.ticks_per_deg,
                "px_per_deg": round(self.px_per_deg, 4),
                "use_external_fire_gate": self.param_values.get("use_external_fire_gate"),
                "lead_apply_gain": self.param_values.get("lead_apply_gain"),
                "gyro_ff_gain": self.param_values.get("gyro_ff_gain"),
                "gyro_ff_sign": self.param_values.get("gyro_ff_sign"),
                "px_per_tick": self.param_values.get("px_per_tick"),
            }

        def _restore(self):
            """恢复被本工具停掉的进程；成功/失败都写 safety_actions。"""
            if self.judgment_pid is not None:
                if getattr(args, "restart_judgment", True):
                    try:
                        spawn_detached(args.judgment_launch,
                                       os.path.join(_DEFAULT_LOG, "6_judgment.log"),
                                       args.judgment_pidfile, cwd="~/dzrobot")
                        self.safety_actions.append("恢复 6_judgment")
                    except Exception as exc:  # noqa: BLE001
                        self.safety_actions.append("恢复 6_judgment 失败：%s" % exc)
                else:
                    self.safety_actions.append("未恢复 6_judgment（--no-restart-judgment）")
            if self.det_pid is not None:
                if getattr(args, "restart_det", True):
                    env_extra = {}
                    disp = getattr(args, "det_display", None)
                    if disp:
                        env_extra["DISPLAY"] = disp
                    try:
                        spawn_detached(args.det_launch,
                                       os.path.join(_DEFAULT_LOG, "4_det.log"),
                                       args.det_pidfile, cwd="~/dzrobot", env_extra=env_extra)
                        self.safety_actions.append("恢复 4_det（DISPLAY=%s）" % (disp or "<未设置>"))
                        try:
                            ok = self._wait_until(lambda: bool(self._foreign_pubs("offset_center")), 6.0, "det 恢复")
                        except Exception:
                            ok = False
                        self.safety_actions.append("4_det 恢复%s" % ("成功（offset_center 有发布者）" if ok else "失败/超时（仍无 offset_center 发布者）"))
                    except Exception as exc:  # noqa: BLE001
                        self.safety_actions.append("恢复 4_det 失败：%s" % exc)
                else:
                    self.safety_actions.append("未恢复 4_det（--no-restart-det）")

        def prepare(self):
            self._spin(1.0)
            # 0) 先做参数快照（必须在停 4_det 之前，否则 det 参数丢失）
            self.params_files = dump_params(args.out)
            self.param_values = parse_param_snapshot(
                self.params_files.get("/dzrknn_dataprocess"))
            self._derive_geometry()
            guard = not getattr(args, "ignore_param_guard", False)

            if self.mode == "hil":
                pv = self.param_values
                known = bool(pv)
                # 功能开关与实验意图不匹配时，默认拒绝（否则数据会"看起来有 gate/lead 但节点没吃"）
                if args.gates and known and not pv.get("use_external_fire_gate", False) and guard:
                    raise RuntimeError(
                        "开了 --gates，但车端 use_external_fire_gate=false ⇒ 两把钥匙不会被节点采用。\n"
                        "  ｜先把 use_external_fire_gate 改 true 并重启 5_gimbal；确知后果可加 --ignore-param-guard")
                if args.lead_time_s > 0 and known and float(pv.get("lead_apply_gain", 0.0) or 0.0) <= 0.0 and guard:
                    raise RuntimeError(
                        "开了 --lead-time-s，但车端 lead_apply_gain<=0 ⇒ /aim/lead_x 不会被采用。\n"
                        "  ｜先把 lead_apply_gain 改 1.0 并重启 5_gimbal；确知后果可加 --ignore-param-guard")
                if args.fake_imu and known and float(pv.get("gyro_ff_gain", 0.0) or 0.0) == 0.0 and guard:
                    raise RuntimeError(
                        "开了 --fake-imu，但车端 gyro_ff_gain=0 ⇒ IMU 前馈不会生效。\n"
                        "  ｜先把 gyro_ff_gain 打开并重启 5_gimbal；确知后果可加 --ignore-param-guard")
                if args.fake_imu:
                    if getattr(args, "imu_topic", "/imu_data") == "/imu_data" and self._foreign_pubs("/imu_data"):
                        raise RuntimeError(
                            "fake-imu 发布到 /imu_data，会和 dzactuator 真 IMU 混流。\n"
                            "  ｜把 5_gimbal 以 `--ros-args -r /imu_data:=/bench_imu` 重启，并加 --imu-topic /bench_imu")
                if not known:
                    self.safety_actions.append("警告：未取到车端参数快照，功能开关无法预检（按当前配置运行）")

                # 任务层检查放在动任何硬件之前：车可能会自己开走
                mission = list(self._foreign_pubs("pursuitAngle"))
                mpid = _pid_of(os.path.expanduser("~/robot_logs/pids/7_path.pid"))
                if mpid and _pid_cmdline(mpid):
                    mission.append("7_path(pid=%d)" % mpid)
                if mission and not getattr(args, "allow_mission", False):
                    raise RuntimeError(
                        "检测到任务层在跑（%s）。\n"
                        "  ｜HIL 前先停 7_path/dzbrain（或加 --allow-mission，确认车不会动）"
                        % ", ".join(mission))
                # 从这里开始，本工具会向 fire_gate/fire_ready/lead_x 写值，结束时要负责清零
                self.owns_outputs = True

                # 1) 激光安全：默认无条件停 dzjudgment（不依赖一次图发现）
                if not args.allow_laser:
                    try:
                        self.pub_laser_switch.publish(UInt8(data=0))
                        self._spin(0.05)
                        self.safety_actions.append("已发 /laser_switch=0（关连续激光路径）")
                    except Exception:
                        pass
                if not args.allow_laser:
                    if not args.stop_judgment:
                        self.safety_actions.append("按 --no-stop-judgment 保留 dzjudgment")
                    else:
                        pid = stop_from_pidfile(args.judgment_pidfile, expect="judgment")
                        if pid:
                            self.judgment_pid = pid
                            self.safety_actions.append("停 6_judgment(pid=%s) 防真出光" % pid)
                        else:
                            self.safety_actions.append("未停到 6_judgment（pid 文件缺失/进程不在）")
                    ok = self._wait_until(
                        lambda: not self._foreign_subs("LaserShot_Command"), 5.0, "激光链路")
                    if not ok:
                        raise RuntimeError(
                            "LaserShot_Command 仍有外来订阅者：%s\n"
                            "  ｜请手动停 dzjudgment；确实要出光再加 --allow-laser"
                            % ", ".join(self._foreign_subs("LaserShot_Command")))
                else:
                    self.safety_actions.append("--allow-laser：允许真实激光出光")

                # 2) det 必须停
                pubs = self._foreign_pubs("offset_center")
                if pubs:
                    if not args.stop_det:
                        raise RuntimeError(
                            "offset_center 有外来发布者：%s（4_det 在跑）。HIL 必须停 det：\n"
                            "  ｜去掉 --no-stop-det，或先手动停 4_det" % ", ".join(pubs))
                    pid = stop_from_pidfile(args.det_pidfile, expect="det_node")
                    if pid:
                        self.det_pid = pid
                        self.safety_actions.append("停 4_det(pid=%s)%s" % (
                            pid, "，结束后恢复" if args.restart_det else "（--no-restart-det）"))
                    if not self._wait_until(
                            lambda: not self._foreign_pubs("offset_center"), 6.0, "停 det"):
                        raise RuntimeError(
                            "停止 4_det 后 offset_center 仍有发布者：%s，拒绝混流运行"
                            % ", ".join(self._foreign_pubs("offset_center")))

                # 3) 任务层在跑会抢 /nav/scan_request、/aim/fire_ready、/aim/lead_x，且可能发车
                mission = list(self._foreign_pubs("pursuitAngle"))
                mpid = _pid_of(os.path.expanduser("~/robot_logs/pids/7_path.pid"))
                if mpid and _pid_cmdline(mpid):
                    mission.append("7_path(pid=%d)" % mpid)
                if mission and not getattr(args, "allow_mission", False):
                    raise RuntimeError(
                        "检测到任务层在跑（%s）。\n"
                        "  ｜HIL 前先停 7_path/dzbrain（或加 --allow-mission，确认车不会动）"
                        % ", ".join(mission))

                # 4) 链路存在性
                missing = []
                if not self._foreign_pubs("camemo_feedback"):
                    missing.append("camemo_feedback 无发布者（1_chassis/dzactuator 未跑？）")
                if not self._foreign_subs("offset_center"):
                    missing.append("offset_center 无订阅者（5_gimbal 未跑？）")
                if not self._foreign_subs("CameMo_Conl"):
                    missing.append("CameMo_Conl 无订阅者（dzactuator 未跑？）")
                if not self._foreign_pubs("/aim/status"):
                    missing.append("/aim/status 无发布者（5_gimbal 未跑？）")
                if args.fake_imu and not self._foreign_subs(args.imu_topic):
                    missing.append("fake-imu：%s 无订阅者（5_gimbal 未 remap 到该话题？）" % args.imu_topic)
                if missing:
                    raise RuntimeError(
                        "HIL 需要 1_chassis + 5_gimbal 都在跑；当前缺：\n  - " + "\n  - ".join(missing))

                # 5) 等第一帧云台反馈
                if not self._wait_until(lambda: self.last_fb_t is not None, 3.0, "等反馈"):
                    raise RuntimeError("3s 内没有 camemo_feedback，HIL 无法闭环")

                # 6) 确认扫描状态（不是 fire-and-forget）
                ok = False
                for _ in range(40):
                    self.pub_scan.publish(UInt8(data=2))
                    self.scan_started = True
                    self._spin(0.1)
                    if self.aim_state in (1, 2):
                        ok = True
                        break
                if not ok:
                    raise RuntimeError(
                        "发 /nav/scan_request=2 后 4s 内 /aim/status 仍=%s（IDLE），拒绝产出无意义数据"
                        % self.aim_state)
            else:
                if not self._foreign_pubs("offset_center"):
                    self.safety_actions.append("警告：offset_center 无发布者（4_det 未运行？capture 会没有视觉行）")

            # 7) 开始记录：清掉可能残留的 latch，再打 t0
            self._clear_fire_latches()
            self.t0 = time.time()
            self.started_mono = self.t0
            self.started_at = time.strftime("%Y-%m-%dT%H:%M:%S")
            self.recording = True
            self._row("st", self.aim_state)  # ★ 必发初始状态，否则"从未变化"的 TRACK 会丢段
            if self.mode == "hil":
                self.pub_scan.publish(UInt8(data=2))
                self.scan_started = True
                self._spin(0.1)
                self.pub_reset.publish(UInt8(data=1))
                self._row("gate", 0, 0)
            elif getattr(args, "scan", None) is not None:
                self.pub_scan.publish(UInt8(data=int(args.scan)))
                self.scan_started = True
                self._spin(0.2)
            self.get_logger().info("AimBench %s 开始录制：%s" % (self.mode, args.out))

        def finish(self):
            self.recording = False
            self._clear_all()
            meta = {
                "tool": TOOL_VERSION,
                "mode": self.mode,
                "scenario": scenario_meta(args.scenario) if self.mode == "hil" else None,
                "args": {k: v for k, v in vars(args).items() if k != "cmd"},
                "host": _host_info(),
                "started_at": self.started_at or time.strftime("%Y-%m-%dT%H:%M:%S"),
                "ended_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "duration_requested": getattr(args, "seconds", None),
                "duration_actual": (round(time.time() - self.started_mono, 3)
                                    if self.started_mono else None),
                "rows": len(self.rows),
                "safety_actions": self.safety_actions,
                "params_snapshots": self.params_files,
                "effective_params": self.effective,
                "env": {k: os.environ.get(k) for k in
                        ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION")},
                "code_sha256_16": _self_hash(),
                "csv_schema": {
                    "vis": "dx,dy,cls,box_w,box_h", "fb": "pitch_tick,yaw_tick,speed_pitch,speed_yaw,load_pitch,load_yaw",
                    "cmd": "pitch_cmd,yaw_cmd,speed_pitch,speed_yaw", "st": "aim_status",
                    "ref": "target_beta_deg,base_psi_deg,gimbal_deg,track_err_deg,dx_published,dx_raw",
                    "lead": "lead_px", "gate": "fire_gate,fire_ready", "shot": "shoot_count", "laser": "1",
                },
            }
            quick = quick_metrics(self.rows, self.mode)
            write_ok = True
            try:
                write_run(args.out, self.rows, meta, quick)
                print("[aim_bench] %s 录制结束：%d 行 -> %s"
                      % (self.mode, len(self.rows), args.out))
                print("[aim_bench] quick:", json.dumps(quick, ensure_ascii=False))
            except Exception as exc:  # noqa: BLE001
                write_ok = False
                print("[aim_bench] 写盘失败（仍会恢复现场）：%s" % exc, file=sys.stderr)
            finally:
                # ★ 无论写盘成功与否都必须恢复 det/judgment
                self._restore()
            meta["safety_actions"] = self.safety_actions
            meta["write_ok"] = write_ok
            try:
                os.makedirs(args.out, exist_ok=True)
                with open(os.path.join(args.out, "meta.json"), "w", encoding="utf-8") as fh:
                    json.dump(meta, fh, ensure_ascii=False, indent=2)
            except Exception:
                pass

    rclpy.init(args=[])
    node = None
    rc = 0
    try:
        node = BenchNode()

        def _on_sig(signum, frame):  # noqa: ARG001
            if node is not None:
                node.stop_requested = True

        for _sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
            if _sig is None:
                continue
            try:
                signal.signal(_sig, _on_sig)
            except Exception:
                pass
        node.prepare()
        end = time.time() + args.seconds
        while time.time() < end and not node.stop_requested:
            if node.feedback_lost:
                raise RuntimeError("camemo_feedback 中断 >300ms（dzactuator/串口挂了？）")
            if node.laser_unsafe:
                raise RuntimeError("检测到新的 LaserShot_Command 订阅者，立即中止（防真出光）")
            rclpy.spin_once(node, timeout_sec=0.02)
        if node.stop_requested:
            print("[aim_bench] 收到 SIGINT/SIGTERM/SIGHUP，提前结束", file=sys.stderr)
    except KeyboardInterrupt:
        print("[aim_bench] Ctrl-C，提前结束", file=sys.stderr)
    except ExternalShutdownException:
        # rclpy 内置信号处理可能先关 context；仍需走 finish 恢复现场
        if node is not None:
            node.stop_requested = True
    except Exception as exc:  # noqa: BLE001
        print("[aim_bench] 运行失败：%s" % exc, file=sys.stderr)
        rc = 3
    finally:
        if node is not None:
            try:
                node.finish()
            except Exception as exc:  # noqa: BLE001
                print("[aim_bench] 收尾失败：%s" % exc, file=sys.stderr)
                rc = 4
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass
    return rc


# ======================================================================
# CLI
# ======================================================================
def add_common(p, hil):
    p.add_argument("--out", default=None, help="输出 run 目录（默认 runs/<时间戳>_<tag>）")
    p.add_argument("--tag", default=None, help="实验标签，写进目录名")
    p.add_argument("--seconds", type=float, default=30.0, help="录制/实验时长（秒）")
    p.add_argument("--rate-hz", type=float, default=30.0, help="合成视觉/采样频率（capture 也用）")
    if hil:
        p.add_argument("--scenario", default="ramp", choices=sorted(SCENARIOS),
                       help="HIL 场景（默认 ramp）")
        p.add_argument("--latency-ms", type=float, default=0.0,
                       help="模拟视觉链路延迟（真实链路约 267ms；调控制先 0，验 lead 用 267）")
        p.add_argument("--noise-px", type=float, default=2.0, help="视觉噪声 σ（px）")
        p.add_argument("--px-per-deg", type=float, default=None,
                       help="像素/度；默认从 px_per_tick×yaw_ticks_per_deg 推导")
        p.add_argument("--px-per-tick", type=float, default=None,
                       help="显式指定像素/刻度（默认从参数快照取，缺省 0.94）")
        p.add_argument("--yaw-center-tick", type=float, default=None,
                       help="云台水平零位刻度；默认从参数快照 gimbal_center_tick 取（2047）")
        p.add_argument("--yaw-sign", type=int, default=None, choices=(1, -1),
                       help="反馈刻度增大对应图像 dx 的方向；默认从参数快照 yaw_deg_sign 取。"
                            "若 HIL 一开就发散，改 -1")
        p.add_argument("--yaw-ticks-per-deg", type=float, default=None,
                       help="水平刻度/度；默认从参数快照 yaw_ticks_per_deg 取（11.378）")
        p.add_argument("--lead-time-s", type=float, default=0.0,
                       help="若>0，按目标角速度×该时间发布 /aim/lead_x（模拟 dzbrain 提前量）")
        p.add_argument("--gates", action="store_true",
                       help="模拟两把钥匙 fire_gate/fire_ready（测开火链用；默认关）")
        p.add_argument("--fake-imu", action="store_true",
                       help="发布假 IMU（仅 base_rotate/combo；需把 5_gimbal 的 /imu_data remap 到 --imu-topic）")
        p.add_argument("--imu-topic", default="/imu_data",
                       help="fake-imu 发布的话题（默认 /imu_data；建议 remap 后用 /bench_imu）")
        p.add_argument("--gate-after", type=float, default=3.0, help="开火钥匙从第几秒开始给")
        p.add_argument("--allow-laser", action="store_true", help="允许真实激光出光（默认拒绝）")
        p.add_argument("--allow-mission", action="store_true",
                       help="允许任务层（pursuitAngle/dzbrain）在跑（默认检测到就拒绝）")
        p.add_argument("--ignore-param-guard", action="store_true",
                       help="跳过功能开关预检（--gates/lead/imu 与车端参数不匹配时仍运行，数据可能误导）")
        p.add_argument("--stop-judgment", dest="stop_judgment", action="store_true", default=True)
        p.add_argument("--no-stop-judgment", dest="stop_judgment", action="store_false")
        p.add_argument("--restart-judgment", dest="restart_judgment", action="store_true", default=True)
        p.add_argument("--no-restart-judgment", dest="restart_judgment", action="store_false")
        p.add_argument("--judgment-pidfile", default=os.path.join(_DEFAULT_PID, "6_judgment.pid"))
        p.add_argument("--judgment-launch", default="ros2 launch dzjudgment judgment.launch.py")
        p.add_argument("--stop-det", dest="stop_det", action="store_true", default=True)
        p.add_argument("--no-stop-det", dest="stop_det", action="store_false")
        p.add_argument("--restart-det", dest="restart_det", action="store_true", default=True)
        p.add_argument("--no-restart-det", dest="restart_det", action="store_false")
        p.add_argument("--det-pidfile", default=os.path.join(_DEFAULT_PID, "4_det.pid"))
        p.add_argument("--det-launch", default="ros2 launch det_node det_node.launch.py")
        p.add_argument("--det-display", default=os.environ.get("GUI_DISPLAY", ":1"),
                       help="恢复 4_det 时注入的 DISPLAY（默认 :1；传空串=不设置）")
    else:
        p.add_argument("--scan", type=int, default=None, choices=(0, 1, 2),
                       help="可选：开始时发 /nav/scan_request（0/1/2）；默认不发")


def main(argv=None):
    p = argparse.ArgumentParser(
        description="AimBench v1：云台调试台架（capture / hil / selftest）",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    ps = sub.add_parser("selftest", help="无 ROS 合成数据，本地验证分析器")
    ps.add_argument("--scenario", default="ramp", choices=sorted(SCENARIOS))
    ps.add_argument("--seconds", type=float, default=20.0)
    ps.add_argument("--out", default=None)
    ps.add_argument("--rate-hz", type=float, default=30.0)
    ps.add_argument("--latency-ms", type=float, default=267.0)
    ps.add_argument("--noise-px", type=float, default=2.0)
    ps.add_argument("--lead-time-s", type=float, default=0.267,
                    help="selftest 控制器里的提前量（模拟 /aim/lead_x；0=无提前量）")

    pc = sub.add_parser("capture", help="只读录制真实链路（--scan 可选发扫描请求）")
    add_common(pc, hil=False)

    ph = sub.add_parser("hil", help="无靶闭环台架（会转云台；默认停 det/judgment）")
    add_common(ph, hil=True)

    args = p.parse_args(argv)
    if getattr(args, "rate_hz", 30.0) <= 0:
        p.error("--rate-hz 必须 > 0")
    if getattr(args, "seconds", 1.0) <= 0:
        p.error("--seconds 必须 > 0")
    if args.cmd == "selftest":
        return run_selftest(args)
    if not args.out:
        tag = args.tag or args.scenario if args.cmd == "hil" else (args.tag or "capture")
        args.out = default_out(args.cmd, tag)
    if args.tag is None:
        args.tag = os.path.basename(args.out)
    return _run_ros(args)


if __name__ == "__main__":
    raise SystemExit(main())
