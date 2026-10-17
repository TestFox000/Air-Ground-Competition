#!/usr/bin/env python3
"""悬停试飞量化报告：把试飞录制的 CSV 变成"合格/不合格"的数字结论。

为什么需要它：悬停质量靠目视说"还行"无法指导调参——漂移速度、突跳次数、
SLAM/EKF 一致性这些数字才能回答"定位到底准不准、要不要动 EKF 参数"。

输入（机上 ros2 topic echo --csv 录制，拉回本地后喂给本脚本）:
    python tools/hover_report.py <hover_ekf.csv>                    # 基础报告
    python tools/hover_report.py <ekf.csv> --slam <slam.csv>        # 加 SLAM 一致性
    python tools/hover_report.py <ekf.csv> --window 30              # 指定分析最近 30s
判据（与试飞方案对齐）:
    10s 位移<0.3m  漂移<3cm/s  突跳=0  z波动σ<5cm  一致性<5cm -> PASS
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

JITTER_M = 0.10          # 相邻样本位移超过此值记一次"突跳"
PASS_DRIFT_M = 0.30      # 悬停窗总位移合格线
PASS_DRIFT_V = 0.03      # 漂移速度合格线 m/s
PASS_Z_SIGMA = 0.05      # 高度波动合格线 m
PASS_ALIGN = 0.05        # SLAM/EKF 残差合格线 m


def read_topic_csv(path: Path) -> list[dict]:
    """读 ros2 topic echo --csv 输出。首行是 'Type.字段' 列名，布尔是小写 true/false。"""
    rows = []
    with open(path, encoding="utf-8", errors="replace", newline="") as f:
        reader = csv.reader(f)
        header = None
        for raw in reader:
            if not raw or not raw[0].strip():
                continue
            if header is None:
                header = [h.strip() for h in raw]
                continue
            rows.append({header[i]: raw[i].strip() for i in range(min(len(raw), len(header)))})
    return rows


def _col(row: dict, key_sub: str):
    """按列名后缀取值（ros2 --csv 的列名形如 'VehicleLocalPosition.x'）。"""
    for k, v in row.items():
        if k.endswith("." + key_sub):
            return v
    return None


def _f(row: dict, key_sub: str):
    v = _col(row, key_sub)
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


def samples(rows: list[dict], *, need_valid: bool = False) -> list[tuple[float, float, float, float]]:
    """提取 (t_s, x, y, z)。t 用 timestamp 字段（us）；need_valid 时过滤 xy_valid。"""
    out = []
    for r in rows:
        if need_valid:
            v = _col(r, "xy_valid")
            if v is not None and v.strip().lower() == "false":
                continue
        t = _f(r, "timestamp")
        x, y, z = _f(r, "x"), _f(r, "y"), _f(r, "z")
        if None not in (t, x, y, z):
            out.append((t / 1e6, x, y, z))
    out.sort(key=lambda s: s[0])
    return out


def dist(a, b) -> float:
    return math.hypot(b[1] - a[1], b[2] - a[2], b[3] - a[3])


def analyze(pts: list[tuple], label: str) -> dict:
    """核心指标：悬停窗(全窗或最近 --window 秒) 位移/漂移速度/突跳/z 波动。"""
    if len(pts) < 10:
        raise SystemExit(f"有效样本太少({len(pts)})——检查 CSV 是否录上")
    t0, t1 = pts[0][0], pts[-1][0]
    dur = t1 - t0
    jitters = [(pts[i - 1], pts[i]) for i in range(1, len(pts))
               if dist(pts[i - 1], pts[i]) > JITTER_M]
    span = dist(pts[0], pts[-1])
    drift_v = span / dur if dur > 0 else float("inf")
    # 轨迹包络：所有样本到起点/终点的最大距离（捕捉"飞出去又飞回来"的情况）
    max_exc = max(dist(pts[0], p) for p in pts)
    zs = [p[3] for p in pts]
    z_mean = sum(zs) / len(zs)
    z_sigma = (sum((v - z_mean) ** 2 for v in zs) / len(zs)) ** 0.5
    return {"label": label, "dur": dur, "n": len(pts), "span": span,
            "drift_v": drift_v, "jitters": len(jitters), "max_exc": max_exc,
            "z_sigma": z_sigma,
            "jitter_detail": [(f"{a[0]-t0:.1f}s", f"{dist(a, b)*100:.0f}cm") for a, b in jitters[:5]]}


def align_residual(ekf_pts, slam_pts) -> float:
    """SLAM 送入位置 vs EKF 输出位置的最近邻配对残差均值（米）。需两边都有 x/y/z。
    注：EKF NED 与 SLAM 变换后的 NED 在 auto-align 对齐后应基本一致。"""
    j = 0
    res = []
    for p in slam_pts:
        while j + 1 < len(ekf_pts) and abs(ekf_pts[j + 1][0] - p[0]) < abs(ekf_pts[j][0] - p[0]):
            j += 1
        q = ekf_pts[j]
        res.append(math.hypot(p[1] - q[1], p[2] - q[2]))
    return sum(res) / len(res) if res else float("nan")


def verdict(m: dict, align=None) -> bool:
    ok = (m["span"] < PASS_DRIFT_M and m["drift_v"] < PASS_DRIFT_V
          and m["jitters"] == 0 and m["z_sigma"] < PASS_Z_SIGMA)
    if align is not None:
        ok = ok and align < PASS_ALIGN
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description="悬停试飞量化报告（ros2 topic echo --csv -> PASS/FAIL）")
    ap.add_argument("ekf_csv", help="/fmu/out/vehicle_local_position 的 CSV 录制")
    ap.add_argument("--slam", default=None, help="/fmu/in/vehicle_visual_odometry 的 CSV（可选）")
    ap.add_argument("--window", type=float, default=None,
                    help="只分析最近 N 秒（默认全窗）")
    args = ap.parse_args()

    pts = samples(read_topic_csv(Path(args.ekf_csv)), need_valid=True)
    if args.window:
        t_end = pts[-1][0]
        pts = [p for p in pts if p[0] >= t_end - args.window]
    m = analyze(pts, "悬停窗")

    align = None
    if args.slam:
        slam = samples(read_topic_csv(Path(args.slam)), need_valid=True)
        if slam:
            align = align_residual(pts, slam)

    print("=" * 56)
    print("悬停试飞量化报告")
    print("=" * 56)
    print(f"  样本 {m['n']} 个，时长 {m['dur']:.1f}s（{m['label']}）")
    print(f"  1. 总位移        {m['span']*100:6.1f} cm   （<30 合格）")
    print(f"  2. 漂移速度      {m['drift_v']*100:6.2f} cm/s （<3 合格）")
    print(f"  3. 位置突跳      {m['jitters']:6d} 次   （0 合格）")
    if m["jitters"]:
        for t, d in m["jitter_detail"]:
            print(f"       突跳@{t}: {d}")
    print(f"  4. 最大偏移      {m['max_exc']*100:6.1f} cm   （辅助参考）")
    print(f"  5. 高度波动σ     {m['z_sigma']*100:6.1f} cm   （<5 合格）")
    if align is not None:
        print(f"  6. SLAM↔EKF残差 {align*100:6.1f} cm   （<5 合格）")
    ok = verdict(m, align)
    print("-" * 56)
    if ok:
        print("  结论: PASS —— 定位链路量化达标，可转入正式航线验证")
    else:
        why = []
        if m["span"] >= PASS_DRIFT_M:
            why.append("位移超标(慢性漂移→SLAM精度问题)")
        if m["drift_v"] >= PASS_DRIFT_V:
            why.append("漂移速度超标")
        if m["jitters"]:
            why.append("有突跳(EKF/SLAM打架→查融合新息/对齐)")
        if m["z_sigma"] >= PASS_Z_SIGMA:
            why.append("高度波动大(查气压计/EKF2_EV_z)")
        if align is not None and align >= PASS_ALIGN:
            why.append("SLAM/EKF残差大(EKF不信任SLAM→调EKF2_EV噪声参数)")
        print("  结论: FAIL —— " + "; ".join(why))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
