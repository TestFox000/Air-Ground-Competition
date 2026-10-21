# -*- coding: utf-8 -*-
"""脚本可运行性守护：演示脚本必须真跑得起来、CLI 报错必须能照着做。

两条真实欠账换来的，都不是假设：

  * ``demo_route.py`` 在改走 `compact_and_adapt` 之后仍引用已删除的
    ``SHOOT_KEEP`` / ``_orthonames``：路线、车命令全都正常打印，**最后一步
    NameError 崩掉**——"看着跑通了"的假绿，与 tools/ 里 monitor_link /
    preflight 那两起同款（脚本不参与业务逻辑，天然逃过单元测试）。
  * ``video_recon.py`` 的失败提示写着"加 --debug 看堆栈"，而 argparse 根本
    不认这个参数，照做只会得到 "unrecognized arguments: --debug"。

因此本文件的判据是"真的执行一遍"：子进程跑脚本、看退出码与关键输出，
而不是 import 一下就算过（import 不会触发模块底部的脚本体）。
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

from ag_integrate import video_recon as vr

REPO = Path(__file__).resolve().parent.parent
RECON = REPO / "src" / "ag_integrate" / "ag_integrate" / "video_recon.py"
DEMO = REPO / "demo_route.py"


def _run(script: Path, *args: str, timeout: int = 300) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(script), *args],
                          capture_output=True, text=True, encoding="utf-8",
                          errors="replace", timeout=timeout, cwd=str(REPO))


# ---------------------------------------------------------------- 演示脚本真跑
def test_demo_route_runs_to_completion():
    """端到端跑一遍：退出码 0、射击停靠格非空、渲染图落盘。

    崩溃点在模块末尾（画完图之前），所以只断言"中途打印过的行"会漏检——
    必须同时看退出码与最后几行。
    """
    proc = _run(DEMO)
    assert proc.returncode == 0, (
        f"demo_route.py 退出码 {proc.returncode}\n"
        f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}")
    assert "排雷覆盖: 全部覆盖" in proc.stdout, proc.stdout[-800:]
    assert "射击停靠: 缺失" not in proc.stdout, (
        "演示场景的射击停靠格丢了（KEEP 推导或靶面链路断了）\n" + proc.stdout[-800:])
    # 渲染步（PIL + 中文字体）必须真跑完
    assert "可视化:" in proc.stdout, (
        "脚本没走到渲染步就退出了（末尾崩过 SHOOT_KEEP / _orthonames 两个悬空名）\n"
        + proc.stdout[-800:])


def test_demo_compare_runs_to_completion():
    proc = _run(REPO / "demo_compare.py")
    assert proc.returncode == 0, (
        f"demo_compare.py 退出码 {proc.returncode}\nstderr:\n{proc.stderr}")
    assert "无挡板" in proc.stdout and "L列挡板" in proc.stdout, proc.stdout[-500:]


# ---------------------------------------------------------------- CLI 报错可照做
def test_cli_missing_video_exits_2_with_actionable_message(tmp_path):
    """视频不存在：给结论 + 建议，不甩 traceback（机上 VNC 里翻不动）。"""
    proc = _run(RECON, str(tmp_path / "nope.mp4"))
    assert proc.returncode == 2, proc.stderr
    assert "[失败] 视频不存在" in proc.stderr, proc.stderr
    assert "Traceback" not in proc.stderr, "默认不该吐堆栈"
    # 失败在解析模型之前，不留空产物目录
    assert "(未创建)" in proc.stderr, proc.stderr


def test_cli_debug_flag_actually_works(tmp_path):
    """--debug 必须被 argparse 认识并且真的把堆栈抛出来。

    事故形态：提示语写着"加 --debug"，参数却没注册 → 用户照做得到
    "unrecognized arguments: --debug"，真正想看的堆栈永远看不到。
    """
    proc = _run(RECON, str(tmp_path / "nope.mp4"), "--debug")
    assert proc.returncode != 2 or "unrecognized arguments" in proc.stderr, (
        f"--debug 没被识别（argparse 报错）:\n{proc.stderr}")
    assert "Traceback" in proc.stderr and "FileNotFoundError" in proc.stderr, (
        f"--debug 没有输出原始堆栈:\n{proc.stderr}")


def test_cli_bad_model_path_says_where_to_look(tmp_path):
    video = tmp_path / "clip.mp4"
    video.write_bytes(b"")                      # 只要"文件存在"，模型解析先于解码
    proc = _run(RECON, str(video), "--d13", str(tmp_path / "no.onnx"))
    assert proc.returncode == 2
    assert "--d13 指定的模型不存在" in proc.stderr, proc.stderr


# ---------------------------------------------------------------- 模型定位真源
def test_model_search_prefers_canonical_pointer():
    """现役模型只认 models/release 的 current 指针，且与部署脚本同一份名单。

    事故形态：搜索路径写死训练目录（dk_train_pack_* / cloud_train_*），目录一
    清理，默认调用就报"未找到模型"，逼人手传 --d13/--v4d——手传路径无人校验
    版本，7 类旧模型照跑（机上按 17 通道硬校验，上机才炸）。
    """
    assert vr.D13_SEARCH[0] == "models/release/current_hbb_detector"
    assert vr.V4D_SEARCH[0] == "models/release/current_cell_classifier"
    src = (REPO / "tools" / "orin_deploy.py").read_text(encoding="utf-8")
    m = re.search(r"POINTER_FILES\s*=\s*\(([^)]*)\)", src)
    assert m, "orin_deploy.py 找不到 POINTER_FILES"
    assert re.findall(r'"([^"]+)"', m.group(1)) == [
        Path(vr.D13_SEARCH[0]).name, Path(vr.V4D_SEARCH[0]).name], (
        "video_recon 的指针名与 tools/orin_deploy.py 的 POINTER_FILES 不一致")


def test_find_model_resolves_current_pointer(tmp_path, monkeypatch):
    """指针文件（内容 = 同目录模型文件名）必须被解析成真实 ONNX。"""
    rel = tmp_path / "models" / "release"
    rel.mkdir(parents=True)
    onnx = rel / "hbb_detector_d13_20261016.onnx"
    onnx.write_bytes(b"stub")
    (rel / "current_hbb_detector").write_text(onnx.name + "\n", encoding="utf-8")
    monkeypatch.chdir(tmp_path)
    assert vr._find_model(vr.D13_SEARCH, None, "d13") == onnx


def test_find_model_error_lists_tried_paths_and_fixes(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    with pytest.raises(FileNotFoundError) as ei:
        vr._find_model(("models/release/current_hbb_detector",), None, "d13")
    msg = str(ei.value)
    assert "current_hbb_detector" in msg, "报错没列出找过的位置"
    assert "--d13" in msg and "release_models.py" in msg, "报错没给可照做的修法"


def test_find_model_falls_back_to_glob_entry(tmp_path, monkeypatch):
    """机上模型按 glob 约定取最新（~/dzuav/models 那份仓库拷贝没有指针）。"""
    models = tmp_path / "models"
    models.mkdir()
    older = models / "hbb_d13_640.onnx"
    newer = models / "hbb_d13_1280.onnx"
    older.write_bytes(b"old")
    newer.write_bytes(b"new")
    os.utime(older, (1_600_000_000, 1_600_000_000))
    monkeypatch.chdir(tmp_path)
    got = vr._find_model((str(models / "hbb*d13*.onnx"),), None, "d13")
    assert got == newer, "glob 项没有取最新的一份"


# ---------------------------------------------------------------- 无 CJK 降级
def test_render_degrades_to_ascii_without_cjk_font(tmp_path, monkeypatch):
    """精简镜像（无中文字体）：渲染必须退化成 ASCII，不许 UnicodeEncodeError。

    2026-10-21 机上实测：写死 Windows 字体 → 落到 PIL 默认位图字体（latin-1），
    画第一个汉字 "墙" 就崩，而且崩在"检测/规划全跑完"之后，整份产物拿不到。
    """
    from PIL import ImageFont

    monkeypatch.setattr(vr, "load_cjk_font",
                        lambda size: (ImageFont.load_default(), False))
    state = {
        "mines_video": ["F5"], "passable_video": ["K5"], "obstacles_video": ["C9"],
        "targets_video": {"red": [], "blue": ["B7"]},
        "car_start_video": [], "car_end_video": [], "uav_marks_video": [],
        "votes_detail": {}, "coverage": {}, "phantom_log": {},
    }
    vr.plan_from_state(state, tmp_path)
    assert (tmp_path / "route_render.png").stat().st_size > 0
