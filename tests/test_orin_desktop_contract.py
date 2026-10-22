# -*- coding: utf-8 -*-
"""桌面入口契约：薄壳 + 仓库单源 + 摘要独立成文件。

2026-10-22 实测事故：`orin_install_desktop.sh` 曾把整个菜单（约 160 行 bash）
和一段嵌入式 Python heredoc 写进 `~/start.sh`。后果有三个：
  1. 改菜单必须重刷机器，桌面副本与源码静默漂移；
  2. heredoc 里引号错一个字符整段崩（实测「未预期的记号 "("」）；
  3. 运行中覆盖脚本会让 bash 按错位偏移读到 Python 行。

本测试锁住"薄壳"这个结构，防止回退。
"""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
TOOLS = REPO / "tools"


def _text(name: str) -> str:
    return (TOOLS / name).read_text(encoding="utf-8")


def test_menu_is_repo_single_source():
    menu = TOOLS / "orin_menu.sh"
    assert menu.is_file(), "菜单逻辑必须在仓库 tools/orin_menu.sh 里"
    text = menu.read_text(encoding="utf-8")
    for key in ("start_stack", "diag", "broadcast_start", "broadcast_stop",
                "recon_video", "show_plan", "menu()"):
        assert key in text, f"orin_menu.sh 缺少 {key}"


def test_installer_generates_thin_shell_only():
    text = _text("orin_install_desktop.sh")
    assert "tools/orin_menu.sh" in text, "薄壳必须指向仓库菜单"
    assert "start_stack()" not in text, "菜单业务逻辑不该再写进桌面副本"
    assert "╔" not in text, "菜单框不该嵌在安装器里"
    assert "exec bash \"$REPO/tools/orin_menu.sh\"" in text


def test_menu_has_specific_failure_advice():
    """每条失败路径都要给出可执行命令，不能只说"看上方报错"。"""
    text = _text("orin_menu.sh")
    for hint in ("pkill -f '[o]rin_agent_watchdog.py'", "ALLOW_STRAY=1",
                 "tools/orin_bringup.sh status", "orin_fix_pyvision.sh"):
        assert hint in text, f"缺少具体处理建议：{hint}"


def test_video_recon_has_no_embedded_python():
    text = _text("orin_video_recon.sh")
    assert "<<'PY'" not in text, "嵌入式 Python heredoc 已抽走，不能回退"
    assert "tools/orin_print_summary.py" in text
    assert "analyze_log" in text, "失败原因分析是必配"


def test_replay_last_prefers_stride5_cache():
    """"最近一次"必须优先正式结果（stride5）。

    2026-10-22 实测：按 mtime 取最新会选中刚跑过的 stride1 反例（14 雷），
    菜单 9 直接给出错误路线。
    """
    text = _text("orin_video_recon.sh")
    assert "pick_cache '*-s5/'" in text
    assert "采用缓存" in text


def test_print_summary_reads_products(tmp_path, capsys):
    spec = importlib.util.spec_from_file_location(
        "_orin_summary", TOOLS / "orin_print_summary.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)

    state = {
        "frames_sampled": 444, "frames_grid_ok": 288,
        "mines_video": ["A2", "B1", "C3", "E2", "E3", "G3", "G9", "I2"],
        "passable_video": ["K10", "K4"], "obstacles_video": ["C9"],
        "targets_video": {"red": ["D5"], "blue": ["B7"]},
    }
    plan = {
        "route_len_cells": 41, "compact": ["a"] * 32, "turns": 22,
        "car_cmds": {"reset": "reset:N10,x", "route": "N10 K10 N5,x",
                     "shoot_cells": ["D8"]},
        "check": {"排雷覆盖": "全部覆盖", "障碍避让": "通过"},
    }
    (tmp_path / "field_state.json").write_text(
        json.dumps(state, ensure_ascii=False), encoding="utf-8")
    (tmp_path / "route_plan.json").write_text(
        json.dumps(plan, ensure_ascii=False), encoding="utf-8")

    assert mod.main([str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "（8/8 颗，官方基线 8）" in out
    assert "路线 41 格 / 压缩 32 点 / 转向 22 次" in out
    assert "射击停靠: ['D8']" in out


def test_print_summary_reports_missing_products(tmp_path, capsys):
    spec = importlib.util.spec_from_file_location(
        "_orin_summary2", TOOLS / "orin_print_summary.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    assert mod.main([str(tmp_path)]) == 2
    err = capsys.readouterr().err
    assert "处理" in err and "field_state.json" in err
