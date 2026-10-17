"""speed_envelope_report 分析核心单测（tools/ 实测方案配套）。"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from speed_envelope_report import (  # noqa: E402
    analyze_speed_run, envelope_verdict, lateral_deviations, parse_csv,
)

# 命令直线：车端 odom 系一段东西向直线（y=1.2）
LINE = ((0.5, 1.2), (3.5, 1.2))


def _line_rows(n=100, drift=0.0, wave=0.0):
    """沿命令直线行进的轨迹 + 恒定横向漂移 + 可选正弦摆动。"""
    return [(i * 0.03, 0.5 + i * 0.03, 1.2 + drift
             + wave * math.sin(i / 5.0)) for i in range(n)]


def test_lateral_deviation_of_on_line_trajectory_is_zero():
    devs = lateral_deviations(_line_rows(), *LINE)
    assert max(devs) < 1e-6


def test_constant_drift_measured_as_deviation():
    devs = lateral_deviations(_line_rows(drift=0.10), *LINE)
    assert abs(sum(devs) / len(devs) - 0.10) < 1e-6


def test_run_fail_verdict_uses_half_cell():
    ok = analyze_speed_run(_line_rows(drift=0.05), *LINE)
    bad = analyze_speed_run(_line_rows(drift=0.20), *LINE)
    assert not ok["fail"]
    assert bad["fail"]
    assert bad["mean"] > 0.175


def test_envelope_verdict_picks_last_passed_speed():
    runs = {"0.2": {"fail": False}, "0.3": {"fail": False}, "0.4": {"fail": True}}
    best, note = envelope_verdict(runs)
    assert best == "0.3"
    assert "0.4" in note


def test_parse_csv_skips_header(tmp_path):
    p = tmp_path / "traj.csv"
    p.write_text("t,x,y\n0.0,0.5,1.2\n0.1,0.53,1.2\n", encoding="utf-8")
    rows = parse_csv(p)
    assert len(rows) == 2 and math.isclose(rows[1][1], 0.53)
