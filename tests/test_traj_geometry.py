"""轨迹几何判定的回归测试（tools/traj_geometry.py）。

为什么要有：2026-09-30 复盘发现"车在画圆"之后，还差一步才能定性——
**圆画在哪儿、一开始朝哪开**。那次的车一收到路线就朝第一路点的反方向开，
几何判定给出的结论是"转向环从发车即发散"，这直接推翻了"调前视距离"
的修法。判据错了就会修错方向，所以钉死测试。
"""

import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "traj_geometry.py"


def _write_traj(path, pts):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        for x, y in pts:
            f.write(f"x: {x}\ny: {y}\nz: 0.2\n---\n")


def _run(traj):
    r = subprocess.run([sys.executable, str(TOOL), str(traj), "--first", "K9"],
                       cwd=str(ROOT), capture_output=True, timeout=120,
                       encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stderr
    return r.stdout


def test_diverged_from_start_is_flagged(tmp_path):
    """反向起步 + 全程不进目标邻域 → 必须报"从发车即发散"。

    构造：N10=(1.5,0.5)，K9=(2.5,3.5)；轨迹一开始就朝 -x 跑（真实现场的形状）。
    """
    p = tmp_path / "traj.txt"
    pts = [(1.5 - 0.1 * i, 0.5 - 0.01 * i) for i in range(1, 30)]     # 朝 -x 方向
    _write_traj(p, pts)
    out = _run(p)
    assert "反向" in out
    assert "发散" in out
    assert "不在" in out          # K9 不在轨迹范围内


def test_correct_direction_is_not_flagged(tmp_path):
    """反证：朝目标方向开（+x/+y）不能误报发散。"""
    p = tmp_path / "traj.txt"
    pts = [(1.5 + 0.05 * i, 0.5 + 0.15 * i) for i in range(1, 30)]    # 朝 K9 方向
    _write_traj(p, pts)
    out = _run(p)
    assert "发散" not in out


def test_cell_to_block_matches_serial_reader_log():
    """坐标约定回归：N10 必须等于 serial_reader 日志里的 (1.50, 0.50)。

    这一条是"topic 是 block 单位"的全部依据，错了整个几何判定都会歪。
    """
    sys.path.insert(0, str(ROOT / "tools"))
    import traj_geometry as tg
    assert tg.cell_to_block("N10") == (1.5, 0.5)
    assert tg.cell_to_block("K9") == (2.5, 3.5)
    assert tg.cell_to_block("A1") == (10.5, 13.5)
    assert tg.cell_to_block("N11") == (0.5, 0.5)
