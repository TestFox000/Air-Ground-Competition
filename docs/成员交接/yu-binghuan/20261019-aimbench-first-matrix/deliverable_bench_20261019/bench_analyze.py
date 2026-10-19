#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AimBench 分析器 — 把一次 run（data.csv + meta.json）变成一份可对比的报告。

设计目标：**只用一个判读口径**，替代过去散落的 aim_analyze / aim_cap4_analyze /
stats_oc2 / summarize_aim / jitter / vstats 等一次性脚本。

输入（任一）：
  * AimBench run 目录（aim_bench.py 产出，含 meta.json + data.csv）
  * 旧格式 CSV：
      - log_aim.py（t,kind,a..e；kind=off/cmd/fb/st）
      - aim_capture.py（无表头；kind=V/G/S）
      - old topic-echo CSV（t,oc,... / st）

输出（默认写回 run 目录）：
  metrics.json   全部指标（机器可读）
  report.md      中文报告（人读；含 PASS/FAIL 判定）
  plots.png      时序图（HIL: 误差/真值 vs 实际；真实: 视觉/反馈/状态）
  并追加一行到 _cloud/bench/experiments.csv（实验台账，--no-ledger 可关）

用法：
  python bench_analyze.py runs/20261019_2130_hil_ramp --compare runs/20261019_2050_hil_ramp
  python bench_analyze.py samples/aim_cap4_pid_kp20_ki12_kd15_20261017.csv
  python bench_analyze.py runs/xxx --no-plot
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
LEDGER = os.path.join(HERE, "experiments.csv")

# 与 robot.yaml / C++ 一致的判读常量（可在 CLI 覆盖前先按项目默认）
BOX_W_FAR, BOX_W_NEAR = 30.0, 120.0
DZ_FAR, DZ_NEAR = 14.0, 26.0
DZ_FIXED = 8.0
STABLE_PX_PER_FRAME = 1.5
FPS_NOMINAL = 30.0
TICKS_PER_DEG = 11.378
CENTER_TICK = 2047.0

STATE_NAME = {0: "IDLE", 1: "SCAN", 2: "TRACK"}


# ----------------------------------------------------------------------
# 读取 / 归一化
# ----------------------------------------------------------------------
def _f(x):
    try:
        if x is None or x == "":
            return None
        v = float(x)
        return v if math.isfinite(v) else None
    except Exception:
        return None


def load_csv(path):
    """返回 rows = [(t, kind, [numeric...]), ...]。

    支持：
      - AimBench 长表：t,kind,a...
      - 旧 log_aim：t,kind,a...（off/cmd/fb/st）
      - kind-first 无表头：V,0.03,... / G,... / S,...
      - UTF-8 BOM
    """
    rows = []
    with open(path, "r", encoding="utf-8-sig", errors="replace", newline="") as fh:
        first = fh.readline()
        fh.seek(0)
        kind_first = False
        has_header = False
        try:
            head = [h.strip().lower() for h in next(csv.reader([first]))]
        except Exception:
            head = []
        if head and head[0] in ("t", "time", "timestamp"):
            has_header = True
        elif len(head) >= 2 and head[0] in ("kind", "type"):
            has_header = True
            kind_first = True
        reader = csv.reader(fh)
        if has_header:
            next(reader, None)
        for r in reader:
            if not r or len(r) < 2:
                continue
            kind = (r[1] or "").strip()
            t = _f(r[0])
            vals = [_f(x) for x in r[2:]]
            if kind_first:
                kind = (r[0] or "").strip()
                t = _f(r[1])
                vals = [_f(x) for x in r[2:]]
            elif t is None:
                # 无表头 kind-first 兜底
                k = (r[0] or "").strip()
                t2 = _f(r[1])
                if k and t2 is not None:
                    kind, t = k, t2
            if t is None:
                continue
            rows.append((t, kind, vals))
    return rows


def normalize(rows):
    """把旧 kind 统一成 vis/fb/cmd/st/ref/lead/gate/shot/laser。"""
    alias = {
        "vis": "vis", "off": "vis", "oc": "vis", "v": "vis",
        "fb": "fb", "g": "fb", "feedback": "fb",
        "cmd": "cmd", "c": "cmd", "conl": "cmd",
        "st": "st", "s": "st", "status": "st",
        "ref": "ref", "lead": "lead", "gate": "gate",
        "shot": "shot", "laser": "laser",
    }
    out = []
    for t, kind, vals in rows:
        k = alias.get(kind.strip().lower())
        if k is None:
            continue
        out.append((t, k, vals))
    return out


def load_run(path):
    """返回 dict(meta, series, params, source)。"""
    meta = {}
    params = {}
    if os.path.isdir(path):
        csv_path = os.path.join(path, "data.csv")
        meta_path = os.path.join(path, "meta.json")
        if not os.path.exists(csv_path):
            raise SystemExit("目录里没有 data.csv：%s" % path)
        if os.path.exists(meta_path):
            with open(meta_path, "r", encoding="utf-8-sig") as fh:
                meta = json.load(fh)
        try:
            fname = (meta.get("params_snapshots") or {}).get("/dzrknn_dataprocess")
            if fname:
                params = parse_param_file(os.path.join(path, fname))
        except Exception:
            params = {}
    else:
        csv_path = path
    rows = normalize(load_csv(csv_path))
    series = {"vis": [], "fb": [], "cmd": [], "st": [], "ref": [], "lead": [],
              "gate": [], "shot": [], "laser": []}
    for t, k, vals in rows:
        series[k].append((t, vals))
    series["st"] = [(t, int(v[0])) for t, v in series["st"]
                    if v and _f(v[0]) is not None]
    series["shot"] = [(t, int(v[0])) for t, v in series["shot"]
                      if v and _f(v[0]) is not None]
    for k in series:
        series[k].sort(key=lambda x: x[0])
    return {"meta": meta, "series": series, "params": params, "csv_path": csv_path,
            "source": os.path.basename(csv_path), "run_dir": path if os.path.isdir(path) else None}


def parse_param_file(path):
    """解析 ros2 param dump 的标量（只取 /dzrknn_dataprocess 快照）。"""
    vals = {}
    try:
        with open(path, "r", encoding="utf-8-sig") as fh:
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
        except Exception:
            vals[key] = raw.strip("'\"")
    return vals


# ----------------------------------------------------------------------
# 工具
# ----------------------------------------------------------------------
def pct(vals, p):
    vals = sorted(vals)
    if not vals:
        return None
    k = min(len(vals) - 1, max(0, int(round((len(vals) - 1) * p / 100.0))))
    return vals[k]


def stat(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return {"n": 0}
    n = len(vals)
    mean = sum(vals) / n
    return {
        "n": n,
        "mean": round(mean, 4),
        "abs_mean": round(sum(abs(v) for v in vals) / n, 4),
        "median": round(statistics_median(vals), 4),
        "rms": round(math.sqrt(sum(v * v for v in vals) / n), 4),
        "p90_signed": round(pct(vals, 90), 4) if pct(vals, 90) is not None else None,
        "p90_abs": round(pct([abs(v) for v in vals], 90), 4) if vals else None,
        "max_abs": round(max(abs(v) for v in vals), 4),
        "min": round(min(vals), 4),
        "max": round(max(vals), 4),
    }


def statistics_median(vals):
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return 0.0
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def fire_dz(box_w, params=None):
    """复刻 C++ fireDeadzonePx（动态框，int 截断）。

    优先用本 run 的 params_*.yaml 快照（fire_dz_dynamic / far / near / box_w_*）；
    没有快照时用部署默认常量。box_w 缺失/≤0 时 t 钳到 0 ⇒ 远端框。
    """
    p = params or {}

    def fnum(key, default):
        try:
            return float(p.get(key, default))
        except Exception:
            return float(default)

    fixed = fnum("fire_deadzone_px", DZ_FIXED)
    if not bool(p.get("fire_dz_dynamic", True)):
        return float(int(fixed))
    w_far = fnum("box_w_far", BOX_W_FAR)
    w_near = fnum("box_w_near", BOX_W_NEAR)
    dz_far = fnum("fire_dz_far_px", DZ_FAR)
    dz_near = fnum("fire_dz_near_px", DZ_NEAR)
    span = w_near - w_far
    if span <= 0:
        return float(int(fixed))
    if box_w is None:
        box_w = 0.0
    t = (float(box_w) - w_far) / span
    t = min(1.0, max(0.0, t))
    return float(int(dz_far + t * (dz_near - dz_far)))


def _unwrap_tick_delta(y0, y1, mod=4096.0):
    """云台刻度 0/4095 回绕：把差值折到 ±mod/2。"""
    d = y1 - y0
    if d > mod / 2.0:
        d -= mod
    elif d < -mod / 2.0:
        d += mod
    return d


def timeline(series):
    """[(state, t0, t1)]，最后一段用整份数据的最后时间补齐（状态只在变化时上报）。"""
    st = series["st"]
    if not st:
        return []
    end_t = t_end(series)
    segs = []
    cur, t0 = st[0][1], st[0][0]
    for t, s in st[1:]:
        if s != cur:
            segs.append((cur, t0, t))
            cur, t0 = s, t
    segs.append((cur, t0, max(end_t, st[-1][0])))
    return segs


def state_at(segs, t):
    if not segs:
        return None
    if t < segs[0][1]:
        return segs[0][0]
    for s, t0, t1 in segs:
        if t0 <= t <= t1:
            return s
    return segs[-1][0]


def t_end(series):
    ts = []
    for k, v in series.items():
        if v:
            ts.append(v[-1][0])
    return max(ts) if ts else 0.0


def run_name(run):
    if run.get("run_dir"):
        return os.path.basename(os.path.normpath(run["run_dir"]))
    return run.get("source", "run")


# ----------------------------------------------------------------------
# 真实靶 capture 指标
# ----------------------------------------------------------------------
def analyze_real(series, params=None):
    vis = series["vis"]
    fb = series["fb"]
    cmd = series["cmd"]
    if not vis:
        return {"kind": "real", "error": "no vis rows"}
    parsed = []
    for t, v in vis:
        dx = v[0] if len(v) > 0 else None
        dy = v[1] if len(v) > 1 else None
        cls = int(v[2]) if len(v) > 2 and v[2] is not None else None
        bw = v[3] if len(v) > 3 else None
        if dx is None:
            continue
        parsed.append({"t": t, "dx": dx, "dy": dy if dy is not None else 0.0,
                       "cls": cls, "box_w": bw})
    if not parsed:
        return {"kind": "real", "error": "no parsable vis rows"}
    dur = max(t_end(series), parsed[-1]["t"]) or 1e-9
    det = [p for p in parsed
           if (p["cls"] is not None and p["cls"] > 0) or (p["cls"] is None and (p["dx"] or p["dy"]))]
    if det:
        adx = [abs(p["dx"]) for p in det]
        ady = [abs(p["dy"]) for p in det]
        sdx = [p["dx"] for p in det]
        inbox = [p for p in det
                 if abs(p["dx"]) <= fire_dz(p["box_w"], params)
                 and abs(p["dy"]) <= fire_dz(p["box_w"], params)]
        sat = [x for x in adx if x >= 200]
        ddx = [abs(det[i + 1]["dx"] - det[i]["dx"]) * (1.0 / 30.0)
               / max(det[i + 1]["t"] - det[i]["t"], 1e-6)
               for i in range(len(det) - 1)]
        cls_hist = {}
        for p in det:
            k = p["cls"] if p["cls"] is not None else -1
            cls_hist[k] = cls_hist.get(k, 0) + 1
        real = {
            "total_frames": len(parsed),
            "det_frames": len(det),
            "det_rate_pct": round(100.0 * len(det) / max(1, len(parsed)), 2),
            "fps": round(len(parsed) / dur, 2),
            "dx_abs_median": pct(adx, 50),
            "dx_abs_p90": pct(adx, 90),
            "dx_abs_max": max(adx),
            "dx_signed_median": pct(sdx, 50),
            "dy_abs_median": pct(ady, 50),
            "dy_abs_p90": pct(ady, 90),
            "in_box": len(inbox),
            "in_box_pct": round(100.0 * len(inbox) / max(1, len(det)), 1),
            "frame_ddx_median": pct(ddx, 50),
            "frame_ddx_p90": pct(ddx, 90),
            "saturation_pct": round(100.0 * len(sat) / max(1, len(det)), 2),
            "class_hist": {str(k): v for k, v in sorted(cls_hist.items())},
        }
    else:
        real = {"total_frames": len(parsed), "det_frames": 0,
                "det_rate_pct": 0.0, "fps": round(len(parsed) / dur, 2)}
    # 云台反馈
    if len(fb) >= 2:
        rates = []
        for i in range(1, len(fb)):
            t0, t1 = fb[i - 1][0], fb[i][0]
            dt = t1 - t0
            if dt <= 1e-5:
                continue
            y0 = fb[i - 1][1][1] if len(fb[i - 1][1]) > 1 else None
            y1 = fb[i][1][1] if len(fb[i][1]) > 1 else None
            if y0 is not None and y1 is not None:
                rates.append(abs(_unwrap_tick_delta(y0, y1)) / dt / TICKS_PER_DEG)
        real["gimbal_rate_dps"] = {
            "median": round(statistics_median(rates), 2) if rates else None,
            "p95": round(pct(rates, 95), 2) if rates else None,
            "max": round(max(rates), 2) if rates else None,
        }
    if cmd:
        sp = []
        for _, v in cmd:
            for ix in (2, 3):
                if len(v) > ix and v[ix] is not None:
                    sp.append(abs(v[ix]))
        real["cmd_speed_max"] = max(sp) if sp else None
        if len(cmd) >= 2:
            dstep = []
            for i in range(1, len(cmd)):
                y0 = cmd[i - 1][1][1] if len(cmd[i - 1][1]) > 1 else None
                y1 = cmd[i][1][1] if len(cmd[i][1]) > 1 else None
                if y0 is not None and y1 is not None:
                    dstep.append(abs(_unwrap_tick_delta(y0, y1)))
            if dstep:
                real["cmd_jitter"] = {"step_median_ticks": pct(dstep, 50),
                                      "step_p90_ticks": pct(dstep, 90),
                                      "max_step_ticks": max(dstep)}
    # 8 条判读（与 stats_oc2.py 同口径）
    checks = []
    if det:
        checks = [
            ("入画率 > 90%", real["det_rate_pct"] > 90.0, "%.1f%%" % real["det_rate_pct"]),
            ("abs(dx) 中位 < 50px", (real["dx_abs_median"] or 0) < 50, "%s px" % real["dx_abs_median"]),
            ("框内率 > 0（开火必要条件）", real["in_box"] > 0, "%.1f%%" % real["in_box_pct"]),
            ("框内率 > 30%（稳健可打）", real["in_box_pct"] > 30.0, "%.1f%%" % real["in_box_pct"]),
            ("帧间 abs(Δdx) 中位 <= 3px/帧", (real["frame_ddx_median"] or 0) <= 3,
             "%s px/帧" % real["frame_ddx_median"]),
            ("中心偏置 abs(中位) <= 15px", abs(real["dx_signed_median"] or 0) <= 15,
             "%s px" % real["dx_signed_median"]),
            ("饱和率 < 5%", real["saturation_pct"] < 5.0, "%.1f%%" % real["saturation_pct"]),
            ("类别单一（无 1/2 反复）", len([k for k in real["class_hist"] if k in ("1", "2")]) <= 1,
             str(real["class_hist"])),
        ]
    real["checks"] = [{"name": n, "pass": bool(ok), "value": v} for n, ok, v in checks]
    real["state_timeline"] = [{"state": STATE_NAME.get(s, s), "t0": round(a, 2), "t1": round(b, 2)}
                              for s, a, b in timeline(series)]
    real["kind"] = "real"
    return real


# ----------------------------------------------------------------------
# HIL 指标（有真值）
# ----------------------------------------------------------------------
def analyze_hil(series, meta):
    ref = series["ref"]
    if not ref:
        return {"kind": "hil", "error": "no ref rows (不是 HIL/selftest 数据？)"}
    segs = timeline(series)
    first_track = None
    for s, t0, _ in segs:
        if s == 2:
            first_track = t0
            break
    samples = []
    for t, v in ref:
        if len(v) < 4 or v[3] is None:
            continue
        beta = v[0] if v[0] is not None else None
        err = v[3]
        state = state_at(segs, t) if segs else 2
        samples.append({"t": t, "beta": beta, "err": err, "state": state})
    if not samples:
        return {"kind": "hil", "error": "ref rows incomplete"}
    tracked = ([s for s in samples if s["t"] >= first_track and s["state"] == 2]
               if first_track is not None else [])
    settled = ([s for s in tracked if s["t"] >= first_track + 1.0]
               if first_track is not None else [])
    use = settled or tracked or samples
    errs = [s["err"] for s in use]
    err_stat = stat(errs)
    beta = [s["beta"] for s in samples if s["beta"] is not None]
    # 目标角速度与滞后估计：所有 |rate|>3 的段都用，err/rate 带符号（可识别 over-lead 负滞后）
    lag_ms = None
    lags = []
    for i in range(1, len(samples)):
        dt = samples[i]["t"] - samples[i - 1]["t"]
        if dt <= 1e-6 or samples[i]["beta"] is None or samples[i - 1]["beta"] is None:
            continue
        rate = (samples[i]["beta"] - samples[i - 1]["beta"]) / dt
        if abs(rate) > 3.0:
            lags.append(1000.0 * samples[i]["err"] / rate)
    if lags:
        lag_ms = round(statistics_median(lags), 1)
    # 指令抖动 / 云台速率
    cmd_jitter = None
    cmd = series["cmd"]
    if len(cmd) >= 2:
        d = []
        for i in range(1, len(cmd)):
            y0 = cmd[i - 1][1][1] if len(cmd[i - 1][1]) > 1 else None
            y1 = cmd[i][1][1] if len(cmd[i][1]) > 1 else None
            if y0 is not None and y1 is not None:
                d.append(abs(_unwrap_tick_delta(y0, y1)))
        if d:
            cmd_jitter = {"step_median_ticks": pct(d, 50), "step_p90_ticks": pct(d, 90),
                          "max_step_ticks": max(d)}
    gimbal_rate = None
    fb = series["fb"]
    if len(fb) >= 2:
        rates = []
        for i in range(1, len(fb)):
            dt = fb[i][0] - fb[i - 1][0]
            if dt <= 1e-5:
                continue
            y0 = fb[i - 1][1][1] if len(fb[i - 1][1]) > 1 else None
            y1 = fb[i][1][1] if len(fb[i][1]) > 1 else None
            if y0 is not None and y1 is not None:
                rates.append(abs(_unwrap_tick_delta(y0, y1)) / dt / TICKS_PER_DEG)
        if rates:
            gimbal_rate = {"median_dps": round(statistics_median(rates), 2),
                           "p95_dps": round(pct(rates, 95), 2),
                           "max_dps": round(max(rates), 2)}
    # 开火
    shots = series["shot"]
    shot_events = []
    if shots:
        last = None
        for t, v in shots:
            if last is None:
                last = v
                if v > 0:
                    shot_events.append(t)
                continue
            if v > last:
                shot_events.extend([t] * (v - last))
            last = v
    # 阶跃场景：稳定时间 / 阶跃峰值 / 阶跃后残余误差
    scenario_name = ((meta.get("scenario") or {}).get("name") or "")
    settle = None
    step_peak = None
    step_residual = None
    if scenario_name == "step" and samples:
        step_ts = []
        for i in range(1, len(samples)):
            if samples[i]["beta"] is None or samples[i - 1]["beta"] is None:
                continue
            if abs(samples[i]["beta"] - samples[i - 1]["beta"]) > 5.0:
                step_ts.append(samples[i]["t"])
        dts = [samples[i]["t"] - samples[i - 1]["t"] for i in range(1, len(samples))
               if samples[i]["t"] > samples[i - 1]["t"]]
        dt_med = statistics_median(dts) if dts else 1.0 / 30.0
        hold_n = max(2, int(round(0.3 / max(dt_med, 1e-3))))  # 0.3s 稳定窗（按实际采样率）
        vals = []
        peaks = []
        residuals = []
        for st in step_ts:
            after = [s for s in samples if s["t"] >= st]
            ok = None
            for j, s in enumerate(after):
                if abs(s["err"]) <= 2.0 and all(abs(a["err"]) <= 2.0
                                                for a in after[j:j + hold_n]):
                    ok = s["t"] - st
                    break
            if ok is not None:
                vals.append(ok)
            window = [abs(s["err"]) for s in after if s["t"] <= st + 2.0]
            if window:
                peaks.append(max(window))
            late = [abs(s["err"]) for s in after if st + 0.5 <= s["t"] <= st + 2.0]
            if late:
                residuals.append(max(late))
        if vals:
            settle = round(statistics_median(vals), 3)
        if peaks:
            step_peak = round(max(peaks), 3)
        if residuals:
            step_residual = round(max(residuals), 3)
    hil = {
        "n_ref": len(samples),
        "n_track": len(tracked),
        "track_seconds": round(max(0.0, t_end(series) - (first_track or 0)), 2) if first_track is not None else 0.0,
        "lock_time_s": round(first_track, 2) if first_track is not None else None,
        "err_deg": err_stat,
        "lag_ms": lag_ms,
        "settle_time_s": settle,
        "step_peak_err_deg": step_peak,
        "step_residual_err_deg": step_residual,
        "cmd_jitter": cmd_jitter,
        "gimbal_rate": gimbal_rate,
        "shots": len(shot_events),
        "first_shot_t": round(shot_events[0], 2) if shot_events else None,
        "state_timeline": [{"state": STATE_NAME.get(s, s), "t0": round(a, 2), "t1": round(b, 2)}
                           for s, a, b in segs],
        "checks": [],
    }

    def _le(v, thr):
        return v is not None and v <= thr

    gates = bool((meta.get("args") or {}).get("gates"))
    checks = [("锁定成功（进入 TRACK）", first_track is not None, "%s s" % hil["lock_time_s"])]
    if scenario_name == "step":
        # 阶跃场景：step_peak 含阶跃本身，仅作参考；验收看阶跃后残余误差 + 稳定时间
        checks.append(("阶跃后残余误差 <= 8.0°", _le(step_residual, 8.0), "%s°" % step_residual))
        checks.append(("阶跃稳定时间 <= 2.0s", _le(settle, 2.0), "%s s" % settle))
    else:
        checks += [
            ("跟踪 RMSE <= 1.5°", _le(err_stat.get("rms"), 1.5), "%s°" % err_stat.get("rms")),
            ("跟踪 P90 <= 3.0°", _le(err_stat.get("p90_abs"), 3.0), "%s°" % err_stat.get("p90_abs")),
            ("跟踪最大误差 <= 8.0°", _le(err_stat.get("max_abs"), 8.0), "%s°" % err_stat.get("max_abs")),
        ]
    if lag_ms is not None and scenario_name not in ("step",):
        checks.append(("等效滞后 <= 150ms", abs(lag_ms) <= 150.0, "%s ms" % lag_ms))
    if gates:
        checks.append(("开火链兑现（≥1 发）", hil["shots"] >= 1, "%d 发" % hil["shots"]))
    hil["checks"] = [{"name": n, "pass": bool(ok), "value": v} for n, ok, v in checks]
    hil["kind"] = "hil"
    return hil


# ----------------------------------------------------------------------
# 画图
# ----------------------------------------------------------------------
def make_plot(run, metrics, out_png):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        print("[analyze] 跳过绘图（matplotlib 不可用：%s）" % exc)
        return False
    s = run["series"]
    segs = timeline(s)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
    for ax in axes:
        ax.grid(True, alpha=0.3)
        for st, t0, t1 in segs:
            if st == 2:
                ax.axvspan(t0, t1, color="#d9f2d9", alpha=0.55, lw=0)
            elif st == 1:
                ax.axvspan(t0, t1, color="#eef2ff", alpha=0.45, lw=0)
    ax = axes[0]
    if metrics.get("kind") == "hil" and s["ref"]:
        ts = [t for t, v in s["ref"] if len(v) > 3 and v[3] is not None]
        es = [v[3] for _, v in s["ref"] if len(v) > 3 and v[3] is not None]
        ax.plot(ts, es, color="#c0392b", lw=1.2, label="tracking error (deg)")
        ax.axhline(0, color="k", lw=0.6)
        for thr in (2.0, -2.0):
            ax.axhline(thr, color="#c0392b", lw=0.6, ls="--", alpha=0.6)
        ax.set_ylabel("error [deg]")
        ax.legend(loc="upper right", fontsize=8)
    elif s["vis"]:
        ts = [t for t, v in s["vis"]]
        dx = [v[0] for _, v in s["vis"]]
        dy = [v[1] if len(v) > 1 and v[1] is not None else 0 for _, v in s["vis"]]
        ax.plot(ts, dx, lw=1.0, label="dx [px]")
        ax.plot(ts, dy, lw=0.8, alpha=0.7, label="dy [px]")
        ax.axhline(0, color="k", lw=0.6)
        ax.set_ylabel("offset [px]")
        ax.legend(loc="upper right", fontsize=8)
    ax = axes[1]
    if s["ref"]:
        ts, beta_rel, g = [], [], []
        for t, v in s["ref"]:
            if len(v) > 2 and v[0] is not None and v[2] is not None:
                psi = v[1] if len(v) > 1 and v[1] is not None else 0.0
                ts.append(t)
                beta_rel.append(v[0] - psi)
                g.append(v[2])
        ax.plot(ts, beta_rel, "--", color="#2c3e50", lw=1.0, label="target bearing (relative)")
        ax.plot(ts, g, color="#2980b9", lw=1.2, label="gimbal angle")
    elif s["fb"]:
        ts = [t for t, v in s["fb"] if len(v) > 1 and v[1] is not None]
        yaw = [(v[1] - CENTER_TICK) / TICKS_PER_DEG for _, v in s["fb"]
               if len(v) > 1 and v[1] is not None]
        if s["cmd"]:
            cts = [t for t, v in s["cmd"] if len(v) > 1 and v[1] is not None]
            cyaw = [(v[1] - CENTER_TICK) / TICKS_PER_DEG for _, v in s["cmd"]
                    if len(v) > 1 and v[1] is not None]
            ax.plot(cts, cyaw, color="#95a5a6", lw=0.8, label="yaw command")
        ax.plot(ts, yaw, color="#2980b9", lw=1.0, label="yaw feedback")
    ax.set_ylabel("yaw [deg]")
    ax.legend(loc="upper right", fontsize=8)
    ax = axes[2]
    if s["vis"]:
        ts = [t for t, v in s["vis"] if len(v) > 2 and v[2] is not None]
        cls = [v[2] for _, v in s["vis"] if len(v) > 2 and v[2] is not None]
        ax.plot(ts, cls, color="#8e44ad", lw=1.0, label="cls (1=blue 2=red 0=none)")
    if s["gate"]:
        ts = [t for t, v in s["gate"]]
        gv = [v[0] + 2 * (v[1] or 0) if len(v) > 1 else v[0] for _, v in s["gate"]]
        ax.step(ts, gv, where="post", color="#16a085", lw=1.0, label="gate/ready code")
    for t in [t for t in _shot_times(s)]:
        ax.axvline(t, color="#e74c3c", lw=1.2, ls=":", alpha=0.9)
    ax.set_ylabel("class / gate")
    ax.set_xlabel("t [s]")
    ax.legend(loc="upper right", fontsize=8)
    fig.suptitle("AimBench: %s" % run_name(run), fontsize=11)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    return True


def _shot_times(series):
    out = []
    last = None
    for t, v in series["shot"]:
        if last is None:
            last = v
            if v > 0:
                out.append(t)
            continue
        if v > last:
            out.extend([t] * (v - last))
        last = v
    return out


# ----------------------------------------------------------------------
# 报告
# ----------------------------------------------------------------------
def _mark(ok):
    return "PASS" if ok else "FAIL"


def build_report(run, metrics, compare=None):
    meta = run["meta"]
    lines = []
    lines.append("# AimBench 报告：%s" % run_name(run))
    lines.append("")
    lines.append("- 生成时间：%s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    lines.append("- 工具：%s" % (meta.get("tool") or "legacy/CSV"))
    mode = meta.get("mode") or metrics.get("kind", "?")
    lines.append("- 模式：**%s**" % mode)
    sc = meta.get("scenario") or {}
    if sc:
        lines.append("- 场景：**%s**（%s）" % (sc.get("name"), sc.get("desc", "")))
    if meta.get("started_at"):
        dur = meta.get("duration_actual") or meta.get("duration_requested") or meta.get("duration")
        lines.append("- 实验时间：%s，时长 %ss" % (meta.get("started_at"), dur))
    lines.append("- 数据：`%s`" % run["csv_path"])
    lines.append("")
    if metrics.get("error"):
        lines.append("> [!] %s" % metrics["error"])
        return "\n".join(lines) + "\n"

    if metrics.get("kind") == "hil":
        lines.append("## 1. 跟踪指标（真值）")
        lines.append("")
        if sc.get("name") == "step":
            lines.append("> step 场景包含阶跃瞬态，RMSE/P90 仅作参考；验收看阶跃峰值/稳定时间。")
            lines.append("")
        lines.append("| 指标 | 值 | 说明 |")
        lines.append("|---|---:|---|")
        e = metrics["err_deg"]
        lines.append("| 跟踪 RMSE | %s° | 稳态段（进入 TRACK 1s 后） |" % e.get("rms"))
        lines.append("| 平均绝对误差 | %s° | |" % e.get("abs_mean"))
        lines.append("| P90 绝对误差 | %s° | |" % e.get("p90_abs"))
        lines.append("| 最大绝对误差 | %s° | |" % e.get("max_abs"))
        lines.append("| 平均偏置 | %s° | 正=云台滞后/偏低 |" % e.get("mean"))
        lines.append("| 等效滞后 | %s ms | 由运动段误差/目标角速度估算（可为负=over-lead） |" % metrics.get("lag_ms"))
        lines.append("| 阶跃峰值误差 | %s° | 仅 step 场景（含阶跃本身，仅参考） |" % metrics.get("step_peak_err_deg"))
        lines.append("| 阶跃后残余误差 | %s° | 仅 step 场景（阶跃 0.5s 后 2s 内最大偏差） |" % metrics.get("step_residual_err_deg"))
        lines.append("| 阶跃稳定时间 | %s s | 仅 step 场景（进入 ±2° 并保持 0.3s） |" % metrics.get("settle_time_s"))
        lines.append("| 首次锁定 | %s s | |" % metrics.get("lock_time_s"))
        lines.append("| TRACK 时长 | %s s | |" % metrics.get("track_seconds"))
        lines.append("| 开火发数 | %s | 首次 %s s |" % (metrics.get("shots"), metrics.get("first_shot_t")))
        lines.append("")
        if metrics.get("cmd_jitter"):
            j = metrics["cmd_jitter"]
            lines.append("指令平滑度：每拍 yaw 指令增量 中位 %s / P90 %s / 最大 %s tick"
                         % (j.get("step_median_ticks"), j.get("step_p90_ticks"), j.get("max_step_ticks")))
            lines.append("")
        if metrics.get("gimbal_rate"):
            g = metrics["gimbal_rate"]
            lines.append("云台实际角速度：中位 %s / P95 %s / 最大 %s °/s"
                         % (g.get("median_dps"), g.get("p95_dps"), g.get("max_dps")))
            lines.append("")
    else:
        lines.append("## 1. 视觉/伺服判读（真实链路）")
        lines.append("")
        lines.append("| 指标 | 值 |")
        lines.append("|---|---:|")
        for k, label in [
            ("total_frames", "视觉帧"), ("det_frames", "有靶帧"),
            ("det_rate_pct", "入画率 %"), ("fps", "帧率"),
            ("dx_abs_median", "abs(dx) 中位 px"), ("dx_abs_p90", "abs(dx) P90 px"),
            ("dx_abs_max", "abs(dx) max px"), ("dx_signed_median", "带符号中位 px"),
            ("dy_abs_median", "abs(dy) 中位 px"), ("in_box", "动态框内帧"),
            ("in_box_pct", "框内率 %"), ("frame_ddx_median", "帧间 abs(Δdx) 中位 px"),
            ("saturation_pct", "饱和率 %"), ("class_hist", "类别分布"),
            ("cmd_speed_max", "指令速度 max"),
        ]:
            if k in metrics:
                lines.append("| %s | %s |" % (label, metrics[k]))
        if metrics.get("gimbal_rate_dps"):
            g = metrics["gimbal_rate_dps"]
            lines.append("| 云台角速度 中位/P95/max °/s | %s / %s / %s |"
                         % (g.get("median"), g.get("p95"), g.get("max")))
        if metrics.get("cmd_jitter"):
            j = metrics["cmd_jitter"]
            lines.append("| 指令每拍增量 中位/P90/max tick | %s / %s / %s |"
                         % (j.get("step_median_ticks"), j.get("step_p90_ticks"), j.get("max_step_ticks")))
        lines.append("")
        lines.append("> 注：in_box 用原始 `dx/dy`（未扣 `lead`/`laser_offset`）；动态框参数优先取本 run 的 "
                     "`params_*.yaml` 快照，旧采集无快照时用部署默认 14/26。旧行缺 `dy` 时按 0。")
        lines.append("")
    lines.append("## 2. 判定")
    lines.append("")
    lines.append("| 判据 | 结果 | 实测 |")
    lines.append("|---|---|---|")
    for c in metrics.get("checks", []):
        lines.append("| %s | **%s** | %s |" % (c["name"], _mark(c["pass"]), c.get("value")))
    lines.append("")
    lines.append("## 3. 状态时间线")
    lines.append("")
    if metrics.get("state_timeline"):
        for s in metrics["state_timeline"]:
            lines.append("- %-5s %6.2f ~ %6.2f s" % (s["state"], s["t0"], s["t1"]))
    else:
        lines.append("> 数据里没有 `/aim/status` 行（旧采集），无法按 IDLE/SCAN/TRACK 分段；"
                     "建议用 `aim_bench.py capture` 重采。")
    lines.append("")
    if meta.get("safety_actions"):
        lines.append("## 4. 安全动作")
        lines.append("")
        for a in meta["safety_actions"]:
            lines.append("- %s" % a)
        lines.append("")
    if compare:
        lines.append("## 5. A/B 对比（baseline：%s）" % compare.get("source"))
        lines.append("")
        kind_cur = metrics.get("kind")
        kind_base = compare["metrics"].get("kind")
        if kind_cur != kind_base:
            lines.append("> 两边类型不同（本次 %s / baseline %s），没有可比字段，跳过对比表。"
                         % (kind_cur, kind_base))
            lines.append("")
        else:
            rows = [("跟踪 RMSE °", "hil", "err_deg.rms"), ("P90 绝对误差 °", "hil", "err_deg.p90_abs"),
                    ("最大误差 °", "hil", "err_deg.max_abs"), ("滞后 ms", "hil", "lag_ms"),
                    ("稳定时间 s", "hil", "settle_time_s"), ("框内率 %", "real", "in_box_pct"),
                    ("abs(dx) 中位 px", "real", "dx_abs_median"), ("入画率 %", "real", "det_rate_pct")]
            lines.append("| 指标 | baseline | 本次 | Δ |")
            lines.append("|---|---:|---:|---:|")
            for label, kind, path in rows:
                if kind_cur != kind:
                    continue
                cur = _dig(metrics, path)
                base = _dig(compare["metrics"], path)
                if cur is None and base is None:
                    continue
                delta = None
                if isinstance(cur, (int, float)) and isinstance(base, (int, float)):
                    delta = round(cur - base, 3)
                lines.append("| %s | %s | %s | %s |" % (label, base, cur, delta))
            lines.append("")
    lines.append("---")
    lines.append("由 `_cloud/bench/bench_analyze.py` 生成。")
    return "\n".join(lines) + "\n"


def _dig(d, path):
    cur = d
    for p in path.split("."):
        if not isinstance(cur, dict) or p not in cur:
            return None
        cur = cur[p]
    return cur


# ----------------------------------------------------------------------
# 台账
# ----------------------------------------------------------------------
def append_ledger(run, metrics, report_path):
    try:
        rel_report = os.path.relpath(report_path, HERE) if report_path else ""
    except Exception:
        rel_report = report_path or ""
    row = {
        "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
        "run": run_name(run),
        "mode": run["meta"].get("mode") or metrics.get("kind", ""),
        "scenario": ((run["meta"].get("scenario") or {}) or {}).get("name", ""),
        "tag": (run["meta"].get("args") or {}).get("tag", ""),
        "rmse": _dig(metrics, "err_deg.rms"),
        "p90_abs": _dig(metrics, "err_deg.p90_abs"),
        "max_abs": _dig(metrics, "err_deg.max_abs"),
        "lag_ms": metrics.get("lag_ms"),
        "settle_s": metrics.get("settle_time_s"),
        "shots": metrics.get("shots", ""),
        "in_box_pct": metrics.get("in_box_pct"),
        "report": rel_report,
    }
    exists = os.path.exists(LEDGER) and os.path.getsize(LEDGER) > 0
    with open(LEDGER, "a", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(row.keys()))
        if not exists:
            w.writeheader()
        w.writerow(row)


# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------
def analyze(path, out=None, plot=True, compare_path=None, ledger=True, strict=False):
    run = load_run(path)
    meta = run["meta"]
    series = run["series"]
    if series["ref"]:
        metrics = analyze_hil(series, meta)
    else:
        metrics = analyze_real(series, run.get("params"))
    metrics["_meta"] = {
        "source": run_name(run),
        "csv": run.get("csv_path"),
        "mode": meta.get("mode"),
        "tag": (meta.get("args") or {}).get("tag"),
        "code_sha256_16": meta.get("code_sha256_16"),
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    outdir = out or run["run_dir"] or os.path.dirname(os.path.abspath(path))
    os.makedirs(outdir, exist_ok=True)

    # 先加载 baseline，避免 compare 失败时只写了一半 metrics 却生成旧报告
    compare = None
    if compare_path:
        base = load_run(compare_path)
        base_metrics = (analyze_hil(base["series"], base["meta"]) if base["series"]["ref"]
                        else analyze_real(base["series"], base.get("params")))
        compare = {"source": run_name(base), "metrics": base_metrics}

    with open(os.path.join(outdir, "metrics.json"), "w", encoding="utf-8") as fh:
        json.dump(metrics, fh, ensure_ascii=False, indent=2)

    png = os.path.join(outdir, "plots.png")
    plot_ok = False
    if plot:
        try:
            plot_ok = make_plot(run, metrics, png)
        except Exception as exc:  # noqa: BLE001
            print("[analyze] 绘图失败：%s" % exc, file=sys.stderr)
        if not plot_ok and os.path.exists(png):
            print("[analyze] 警告：本次绘图失败，%s 是旧图，未更新" % png, file=sys.stderr)
    elif os.path.exists(png):
        print("[analyze] 注意：--no-plot，%s 可能是旧图，未更新" % png, file=sys.stderr)

    report = build_report(run, metrics, compare)
    report_path = os.path.join(outdir, "report.md")
    with open(report_path, "w", encoding="utf-8-sig") as fh:
        fh.write(report)
    print(report)
    print("\n[analyze] metrics -> %s" % os.path.join(outdir, "metrics.json"))
    print("[analyze] report  -> %s" % report_path)
    if plot_ok:
        print("[analyze] plot    -> %s" % png)
    if ledger:
        try:
            append_ledger(run, metrics, report_path)
            print("[analyze] ledger  -> %s" % LEDGER)
        except Exception as exc:  # noqa: BLE001
            print("[analyze] 写台账失败：%s" % exc, file=sys.stderr)
    if strict:
        bad = [c for c in metrics.get("checks", []) if not c.get("pass")]
        if bad:
            print("[analyze] STRICT: %d 项判据 FAIL" % len(bad), file=sys.stderr)
            metrics["_strict_failed"] = len(bad)
    return metrics


def main(argv=None):
    try:
        sys.stdout.reconfigure(errors="replace")
        sys.stderr.reconfigure(errors="replace")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="AimBench 分析器：run 目录/旧 CSV -> metrics + report + plots")
    ap.add_argument("path", help="run 目录，或 CSV 文件")
    ap.add_argument("--out", default=None, help="输出目录（默认写回 run 目录/CSV 所在目录）")
    ap.add_argument("--compare", default=None, help="baseline run 目录/CSV，生成 A/B 表")
    ap.add_argument("--no-plot", action="store_true", help="不画图")
    ap.add_argument("--no-ledger", action="store_true", help="不追加 experiments.csv")
    ap.add_argument("--strict", action="store_true", help="任一判据 FAIL 时退出码 2（CI/benchctl 用）")
    args = ap.parse_args(argv)
    metrics = analyze(args.path, out=args.out, plot=not args.no_plot,
                      compare_path=args.compare, ledger=not args.no_ledger,
                      strict=args.strict)
    if args.strict:
        if metrics.get("error"):
            print("[analyze] STRICT: 数据不完整：%s" % metrics["error"], file=sys.stderr)
            return 2
        if not metrics.get("checks"):
            print("[analyze] STRICT: 没有可判读样本/判据", file=sys.stderr)
            return 2
        if metrics.get("_strict_failed"):
            return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
