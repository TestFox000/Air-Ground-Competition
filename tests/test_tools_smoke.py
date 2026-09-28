"""工具脚本冒烟测试：每个带 argparse 的 tools/*.py 都要能跑起 `--help`。

为什么需要：`tools/` 下的脚本各自做 `sys.path` 设置，一旦包布局变了
（真实事故：`ag_core` 改成两层布局后，`sim_selfcheck.py` 报
`ModuleNotFoundError: No module named 'ag_core.grid_map'`，
而单元测试全绿、CI 也没拦住 —— 因为没人 import 这些脚本）。
"""

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"


def _tools_with_argparse():
    """扫描 tools/ 及其子目录（如 tools/ml/）下所有可独立执行的脚本。"""
    out = []
    for f in sorted(TOOLS.rglob("*.py")):
        if "__pycache__" in f.parts:
            continue
        text = f.read_text(encoding="utf-8", errors="replace")
        if "argparse" in text and "__main__" in text:
            out.append(f)
    return out


TOOL_FILES = _tools_with_argparse()


def test_found_tools():
    assert TOOL_FILES, "tools/ 下没找到带 argparse 的脚本，检查一下目录"


@pytest.mark.parametrize("tool", TOOL_FILES, ids=lambda p: p.name)
def test_tool_help_runs(tool: Path):
    text = tool.read_text(encoding="utf-8", errors="replace")
    if "paramiko" in text:
        pytest.importorskip("paramiko", reason="远程执行工具需要 paramiko（见 requirements-extra.txt）")
    # ⚠️ 必须显式 utf-8：Windows 下 text=True 默认用 GBK 解码，
    # 工具里的中文 usage 会直接抛 UnicodeDecodeError
    r = subprocess.run(
        [sys.executable, str(tool), "--help"],
        cwd=str(ROOT), capture_output=True, timeout=120,
        encoding="utf-8", errors="replace",
    )
    assert r.returncode == 0, (
        f"{tool.name} --help 失败（退出码 {r.returncode}）\n"
        f"--- stdout ---\n{r.stdout[-2000:]}\n--- stderr ---\n{r.stderr[-2000:]}"
    )
    assert "usage" in (r.stdout + r.stderr).lower(), f"{tool.name} 没有打印 usage"


@pytest.mark.parametrize("tool", TOOL_FILES, ids=lambda p: p.name)
def test_tool_sets_up_src_paths(tool: Path):
    """脚本必须把 src 下**所有**包目录都加进 sys.path，否则两层布局会 import 失败。"""
    text = tool.read_text(encoding="utf-8", errors="replace")
    if "sys.path" not in text:
        pytest.skip("该脚本不依赖本项目模块")
    if "iterdir()" in text or "conftest" in text:
        return
    # 允许显式罗列，但必须覆盖 ag_core（最容易漏的一个）
    assert "ag_core" in text or "iterdir()" in text, (
        f"{tool.name} 的 sys.path 设置看起来没覆盖 src/ag_core"
    )
