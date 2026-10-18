"""悬停漂移取证工具（tools/hover_drift_forensics.py）的测试。

测的不是"能不能跑"，而是**数字对不对**：
    1. 无时间戳的 delta 行按锚点行号插值重建时间——插值位置必须精确
    2. control.log 状态窗口/事件（ARM 目标、ARM 拒绝、脱管）要完整切出
    3. 窗口统计（净位移/RMS/峰值/偏置/reset 计数）与手算一致
    4. main() 能在目录里自动配对 control/odom 日志出报告
"""

import importlib.util
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def _load_tool(name: str):
    """按路径加载 tools/*.py（tools 不在包的 import 路径里）。

    必须先注册进 sys.modules 再 exec：dataclass 的 `X | None` 注解解析时
    要回查 sys.modules[cls.__module__]，缺了会 AttributeError。"""
    spec = importlib.util.spec_from_file_location(
        f"_tool_{name}", ROOT / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)  # type: ignore[union-attr]
    return mod


forensics = _load_tool("hover_drift_forensics")


def _write_odom_log(path: Path) -> None:
    """造一份带锚点的 odom 日志：101 行 10s 跨度（100 个 delta 夹在两个锚点间）。

    行号 0: 锚点 t=100.0
    行号 1..100: delta 行（dx=0.01*1, dy=-0.01, dz=0.001 恒定）
    行号 101: 锚点 t=110.0（含 EKF reset 字样）
    """
    lines = ["[px4_odom_convert-1] [INFO] [100.000000000] [px4_odom_convert]: "
             "Odometry initialized, start publishing PX4 VIO"]
    for _ in range(100):
        lines.append("[px4_odom_convert-1] [px4_odom_convert] position delta "
                     "NED [m]: dx=0.0100  dy=-0.0100  dz=0.0010")
    lines.append("[px4_odom_convert-1] [WARN] [110.000000000] "
                 "[px4_odom_convert]: EKF frame reset detected (xy_reset=2 "
                 "heading_reset=1); alignment transform stays frozen, EKF "
                 "will re-converge to SLAM")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_control_log(path: Path) -> None:
    """造一份带完整状态链的 control 日志（时间戳与 odom 日志同一时钟）。"""
    lines = [
        "[px4_control_node-1] [INFO] [99.000000000] [px4_control_node]: "
        "px4_control_node up. takeoff=0.50m",
        "[px4_control_node-1] [INFO] [101.000000000] [px4_control_node]: "
        "state: DISARMED -> ARMING (arm command sent)",
        "[px4_control_node-1] [INFO] [101.000000000] [px4_control_node]: "
        "ARM sent, target=(1.50, -0.25, -0.50) yaw=0.12",
        "[px4_control_node-1] [INFO] [101.500000000] [px4_control_node]: "
        "state: ARMING -> TAKEOFF (armed detected)",
        "[px4_control_node-1] [INFO] [104.000000000] [px4_control_node]: "
        "state: TAKEOFF -> HOVER (reached takeoff altitude)",
        "[px4_control_node-1] [WARN] [106.000000000] [px4_control_node]: "
        "ARM refused: state=HOVER (must be DISARMED/LANDED)",
        "[px4_control_node-1] [INFO] [108.000000000] [px4_control_node]: "
        "state: HOVER -> LANDING (self-managed descent begin)",
        "[px4_control_node-1] [WARN] [109.000000000] [px4_control_node]: "
        "left OFFBOARD (nav_state=2), yielding control",
        "[px4_control_node-1] [INFO] [109.000000000] [px4_control_node]: "
        "state: LANDING -> LANDED (PX4 confirmed disarmed after landing)",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ── 时间重建 ───────────────────────────────────────────────────

def test_anchor_interpolation_midpoint(tmp_path):
    """两锚点正中间的 delta 行，重建时间必须是中点。"""
    odom = tmp_path / "s_odom.log"
    _write_odom_log(odom)
    series = forensics.parse_odom_log(odom)
    assert len(series.samples) == 100
    # 行号 1..100 夹在锚点行 0(t=100) 与 101(t=110) 之间，线性插值
    first = series.samples[0]
    last = series.samples[-1]
    assert first.t == pytest.approx(100.0 + 10.0 / 101.0, abs=1e-6)
    assert last.t == pytest.approx(100.0 + 10.0 * 100 / 101.0, abs=1e-6)
    # 样本值本身也要对
    assert first.dx == pytest.approx(0.01)
    assert first.dy == pytest.approx(-0.01)


def test_reset_events_parsed(tmp_path):
    odom = tmp_path / "s_odom.log"
    _write_odom_log(odom)
    series = forensics.parse_odom_log(odom)
    assert series.resets == [(110.0, 2, 1)]


# ── control 日志窗口/事件 ──────────────────────────────────────

def test_control_windows_and_events(tmp_path):
    ctrl = tmp_path / "s_control.log"
    _write_control_log(ctrl)
    windows, events = forensics.parse_control_log(ctrl)
    states = [(w.state, w.t0, w.t1) for w in windows]
    assert states[0] == ("ARMING", 101.0, 101.5)
    assert states[1] == ("TAKEOFF", 101.5, 104.0)
    assert states[2] == ("HOVER", 104.0, 108.0)
    assert states[3] == ("LANDING", 108.0, 109.0)
    assert states[4] == ("LANDED", 109.0, None)  # 会话结束未退出
    # ARM 目标坐标
    arm = [d for t, d in events if d.startswith("ARM target")]
    assert arm == ["ARM target=(1.50,-0.25,-0.50) yaw=0.12"]
    # 拒绝与脱管
    assert any("ARM refused" in d for _, d in events)
    assert any("left OFFBOARD" in d for _, d in events)


# ── 窗口统计数学正确性 ─────────────────────────────────────────

def test_window_stats_math():
    """恒定增量序列：net=n*step，step_rms=|step|，mean_step=step（方向偏置）。"""
    samples = [forensics.DeltaSample(t=100.0 + i * 0.1,
                                     dx=0.02, dy=-0.01, dz=0.0)
               for i in range(51)]  # t=100.0..105.0, 10Hz
    series = forensics.OdomSeries(samples=samples, resets=[(102.0, 3, 1)])
    st = forensics.window_stats(series, 100.0, 105.0, series.resets)
    assert st is not None
    assert st.n == 51
    assert st.duration == pytest.approx(5.0)
    assert st.net[0] == pytest.approx(51 * 0.02)
    assert st.net[1] == pytest.approx(51 * -0.01)
    assert st.step_rms[0] == pytest.approx(0.02)
    # 等效速度 RMS = 步 RMS × 实测速率（51 样本 5s → 10Hz）
    assert st.vel_rms[0] == pytest.approx(0.2)
    assert st.max_step[1] == pytest.approx(0.01)
    assert st.mean_step[0] == pytest.approx(0.02)   # +X 方向偏置
    assert st.resets == 1


def test_window_stats_too_few_samples():
    series = forensics.OdomSeries(
        samples=[forensics.DeltaSample(t=100.0, dx=0.1, dy=0.0, dz=0.0)],
        resets=[])
    assert forensics.window_stats(series, 99.0, 105.0) is None


# ── 端到端：目录配对 + Markdown 输出 ──────────────────────────

def test_main_directory_pairs(tmp_path, capsys):
    _write_control_log(tmp_path / "20990101_000000_control.log")
    _write_odom_log(tmp_path / "20990101_000000_odom.log")
    rc = forensics.main([str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "会话 20990101_000000" in out
    assert "| HOVER |" in out
    # HOVER 窗口 104→108：约 40 个 delta 样本，净 X 位移应 ≈ 样本数×0.01
    hover_row = [ln for ln in out.splitlines() if ln.startswith("| HOVER")][0]
    m = __import__("re").search(r"\(\+0\.\d+", hover_row)
    assert m, hover_row
    net_x = float(hover_row.split("|")[4].strip().split(",")[0].strip("("))
    assert 0.3 < net_x < 0.5  # ~40 样本 × 0.01
