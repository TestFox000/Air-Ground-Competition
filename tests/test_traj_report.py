"""轨迹复盘判据的回归测试（tools/traj_report.py）。

为什么要有：这些判据是 2026-09-30 定位到"车在画圆"的唯一依据。
**判据算错 = 我们会对着一个不存在的故障排查一整晚**，所以给它们钉死测试：

- 单向画圆  → 净转角 ≈ ±360°/圈、转向反转 ≈ 0 次
- 笔直前进  → 净转角 ≈ 0
- 蛇形左右摆 → 反转次数很多、净转角 ≈ 0（用来和上一条区分）

事故背景：真发车脚本只报"位移 1.741 m"，看着像车动了；而实际是走了 12.68 m
只净移动 1.74 m、净转角 +936°（半径 0.78 m 画了 2.6 圈）。
只看位移永远发现不了，靠的就是下面这几个数。
"""

import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import traj_report  # noqa: E402


def _write_traj(path, pts):
    """按 `ros2 topic echo --field pose.pose.position` 的格式写一帧一帧的坐标。"""
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        for x, y in pts:
            f.write(f"x: {x}\ny: {y}\nz: 0.2\n---\n")


def _circle(n=250, r=0.8, laps=1.0, hz=50.0):
    """半径 r、转 laps 圈的轨迹（默认 5 s、一整圈）。"""
    return [(r * math.cos(2 * math.pi * laps * i / n),
             r * math.sin(2 * math.pi * laps * i / n)) for i in range(n)]


def test_load_parses_ros_echo_format(tmp_path):
    p = tmp_path / "traj.txt"
    _write_traj(p, [(0.0, 0.0), (0.1, 0.0), (0.2, 0.1)])
    pts = traj_report.load(str(p))
    assert len(pts) == 3
    assert pts[0] == (0.0, 0.0)
    assert pts[-1] == (0.2, 0.1)


def test_heading_stats_detects_circle(tmp_path):
    """画圆：净转角 ≈ 360°、且**不反向**（这是区分画圆与蛇形的关键）。"""
    p = tmp_path / "traj.txt"
    _write_traj(p, _circle())
    pts = traj_report.load(str(p))
    net, total, flips, n = traj_report.heading_stats(pts, 50.0)
    assert n > 10
    assert abs(abs(net) - 360.0) < 20.0, f"一整圈的净转角应接近 360°，实际 {net}"
    assert flips == 0, f"单向画圆不该有转向反转，实际 {flips} 次"
    # 净转角 ≈ 累计转角 = 一直朝同一边转
    assert abs(net) > total * 0.95


def test_heading_stats_straight_line_is_zero(tmp_path):
    """直线：净转角 ≈ 0，不能误报成画圆。"""
    p = tmp_path / "traj.txt"
    _write_traj(p, [(i * 0.005, 0.0) for i in range(250)])
    pts = traj_report.load(str(p))
    net, _, flips, _ = traj_report.heading_stats(pts, 50.0)
    assert abs(net) < 5.0
    assert flips == 0


def test_heading_stats_snake_has_many_flips(tmp_path):
    """蛇形摆：反转次数很多、净转角接近 0 —— 用来和画圆区分开。"""
    p = tmp_path / "traj.txt"
    pts = []
    for i in range(500):                      # 正弦波：左右来回摆
        pts.append((i * 0.004, 0.15 * math.sin(i * 0.08)))
    _write_traj(p, pts)
    loaded = traj_report.load(str(p))
    net, _, flips, _ = traj_report.heading_stats(loaded, 50.0)
    assert flips > 3, f"蛇形应该有多次转向反转，实际 {flips}"
    assert abs(net) < 180.0


def test_cli_reports_circle_on_circle_traj(tmp_path):
    """端到端：给一段画圆轨迹，命令行必须把"单向画圆"报出来。"""
    p = tmp_path / "traj.txt"
    _write_traj(p, _circle(laps=2.6))
    r = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "traj_report.py"), str(p)],
        cwd=str(ROOT), capture_output=True, timeout=120,
        encoding="utf-8", errors="replace",
    )
    assert r.returncode == 0, r.stderr
    # 认准结论行（"→ 单向画圆："）—— 图例里也出现过"单向画圆"四个字，别拿它当判据
    assert "→ 单向画圆：" in r.stdout
    # 走得多、净位移小，且 bbox 只有圈那么大 —— 三条要一起出现才判得准
    assert "转向反转" in r.stdout
    assert "活动范围" in r.stdout


def test_cli_does_not_cry_circle_on_straight(tmp_path):
    """反证：笔直走的时候不能误报画圆（否则天天排查假故障）。"""
    p = tmp_path / "traj.txt"
    _write_traj(p, [(i * 0.005, 0.0) for i in range(500)])
    r = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "traj_report.py"), str(p)],
        cwd=str(ROOT), capture_output=True, timeout=120,
        encoding="utf-8", errors="replace",
    )
    assert r.returncode == 0, r.stderr
    assert "→ 单向画圆：" not in r.stdout   # 图例里的"单向画圆"不算，只认结论行
