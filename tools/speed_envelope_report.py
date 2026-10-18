"""循迹速度包线实测报告器（docs/实测方案-车端三项-20261016.md 配套工具）。

输入：逐档提速试验录下的轨迹 CSV（列：t,x,y[,...]，表头可选），
每档一段（每档一个文件），加**命令直线两端点**（车端 odom 系坐标，
见方案文档 §1：取 reset 锚定格心与目标格心）。
输出：各档相对命令直线的横向偏差（均值/最大/P95）与失格判定
（>半格 17.5cm 即失格），给出最大安全巡航速度（包线拐点）。

为什么命令线必须显式给定而不是从轨迹拟合：恒定横向漂移的轨迹
本身仍是直线，最小二乘会把漂移吸收进截距、偏差恒为 0——
命令线是"车该走哪"，只能来自发车指令的格心，不能来自车实际走了哪。

用法（本地分析，不碰车）：
    python tools/speed_envelope_report.py --line 1.225,0.175,3.72,0.175 \
        --csv seg_0.2.csv seg_0.3.csv --speed 0.2 0.3
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

# 失格阈值：轮心横向偏差超过半格（17.5cm）即认为该档不可用
HALF_CELL_M = 0.175
# 判定门限：均值超阈 或 P95 超阈 都算失格（P95 抗单帧毛刺，均值抗系统性漂移）
P95_QUANTILE = 0.95


def parse_csv(path: str | Path) -> list[tuple[float, float, float]]:
    """读轨迹 CSV -> [(t, x, y), ...]。

    接受两种形态：带表头（自动跳过非数字行）或裸三列。
    utf-8-sig：Excel 另存的 CSV 带 BOM，不剥掉首列 float() 会炸。
    """
    rows: list[tuple[float, float, float]] = []
    with open(path, "r", encoding="utf-8-sig", errors="replace") as fp:
        for line in fp:
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 3:
                continue
            try:
                rows.append((float(parts[0]), float(parts[1]), float(parts[2])))
            except ValueError:
                continue   # 表头/注释行
    return rows


def lateral_deviations(rows: list[tuple[float, float, float]],
                       p1: tuple[float, float],
                       p2: tuple[float, float]) -> list[float]:
    """轨迹点到命令直线 p1→p2 的横向偏差序列（米）。"""
    ax, ay = p2[0] - p1[0], p2[1] - p1[1]
    norm = math.hypot(ax, ay)
    if norm < 1e-9:
        raise ValueError("命令线两端点重合")
    return [abs(ax * (p1[1] - y) - (p1[0] - x) * ay) / norm
            for _t, x, y in rows]


def percentile(values: list[float], q: float) -> float:
    if not values:
        return float("inf")
    s = sorted(values)
    k = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[k]


def analyze_speed_run(rows: list[tuple[float, float, float]],
                      p1: tuple[float, float],
                      p2: tuple[float, float]) -> dict:
    """单档分析：横向偏差统计 + 失格判定。"""
    devs = lateral_deviations(rows, p1, p2)
    if not devs:
        return {"n": 0, "mean": float("inf"), "max": float("inf"),
                "p95": float("inf"), "fail": True}
    mean = sum(devs) / len(devs)
    return {
        "n": len(devs),
        "mean": mean,
        "max": max(devs),
        "p95": percentile(devs, P95_QUANTILE),
        "fail": mean > HALF_CELL_M or percentile(devs, P95_QUANTILE) > HALF_CELL_M,
    }


def _speed_key(label: str):
    """档位排序键：数字标签按数值排；非数字标签（如默认的 CSV 文件名）
    退回字符串序排在数值档之后——不给 --speed 时裁决也不许崩。"""
    try:
        return (0, float(label), "")
    except ValueError:
        return (1, 0.0, label)


def envelope_verdict(runs: dict[str, dict]) -> tuple[str, str]:
    """包线裁决：通过档的最大速度；全部失格给出结论。"""
    speeds = sorted(runs, key=_speed_key)
    passed = [s for s in speeds if not runs[s]["fail"]]
    if not passed:
        worst = speeds[0] if speeds else "?"
        return "无", f"最低档 {worst} 已失格——循迹链路存在问题，先修再测"
    best = passed[-1]
    nxt_idx = speeds.index(best) + 1
    note = "已是最高测试档" if nxt_idx >= len(speeds) else (
        f"下一档 {speeds[nxt_idx]} 失格（均值/P95 超 {HALF_CELL_M*100:.0f}cm）")
    return best, note


def _parse_point(text: str) -> tuple[float, float]:
    x, y = text.split(",")
    return float(x), float(y)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="循迹速度包线实测报告器")
    ap.add_argument("--csv", nargs="+", required=True,
                    help="轨迹 CSV（可多文件，每档一个）")
    ap.add_argument("--line", required=True,
                    help="命令直线两端点 x1,y1,x2,y2（车端 odom 系；"
                         "取 reset 锚定格心与目标格心，见方案文档 §1）")
    ap.add_argument("--speed", nargs="*", default=None,
                    help="与 --csv 对应的档位标签（m/s），如 0.2 0.3 0.4；"
                         "不给则取 CSV 文件名（去扩展名）作标签")
    args = ap.parse_args(argv)

    parts = args.line.replace(";", ",").split(",")
    if len(parts) != 4:
        print("--line 需要 x1,y1,x2,y2 四个数", file=sys.stderr)
        return 2
    p1 = (float(parts[0]), float(parts[1]))
    p2 = (float(parts[2]), float(parts[3]))

    # 默认标签取 CSV 文件名（去扩展名）：旧默认"档1"会让裁决的 float(标签) 直接崩
    labels = args.speed or [Path(p).stem for p in args.csv]
    if args.speed and len(args.speed) != len(args.csv):
        print("--speed 数量须与 --csv 一致", file=sys.stderr)
        return 2
    runs: dict[str, dict] = {}
    for path, label in zip(args.csv, labels):
        rows = parse_csv(path)
        runs[label] = analyze_speed_run(rows, p1, p2)
        r = runs[label]
        print(f"[{label} m/s] 采样 {r['n']} 点 | 横向偏差 均值 "
              f"{r['mean']*100:5.1f}cm 最大 {r['max']*100:5.1f}cm "
              f"P95 {r['p95']*100:5.1f}cm | "
              f"{'❌失格' if r['fail'] else '✅合格'}")

    best, note = envelope_verdict(runs)
    print("-" * 60)
    print(f"循迹速度包线：最大安全巡航 ≈ {best} m/s。{note}")
    print(f"判定阈值：横向偏差 均值/P95 > {HALF_CELL_M*100:.0f}cm（半格）即失格；")
    print("回填 match_params.yaml 的 time_cruise_mps 并重估路线时间预算。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
