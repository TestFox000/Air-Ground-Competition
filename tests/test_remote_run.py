"""remote_run 的 _expand_tilde 修复单测（并行会话旁支合法化）。

背景：SFTP 协议不展开 `~`，服务器把 '~/x' 当字面目录名 → ENOENT
（2026-10-16 orin_deploy 首传踩到）。修复 = _expand_tilde() 用
sftp.normalize('.') 拿真实家目录后展开。这里 mock sftp 验证两形态。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

pytest.importorskip("paramiko")   # remote_run 模块顶层即 import paramiko

REPO = Path(__file__).resolve().parents[1]


def load_module(name: str, file: Path):
    spec = importlib.util.spec_from_file_location(name, file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


rr = load_module("remote_run", REPO / "tools" / "remote_run.py")


class _FakeSftp:
    """最小 SFTP 替身：normalize('.') 返回登录家目录绝对路径，并记录调用。"""

    def __init__(self) -> None:
        self.calls: list[str] = []

    def normalize(self, p: str) -> str:
        self.calls.append(p)
        return "/home/nvidia"


def test_expand_tilde_home_form():
    """~/x 形态：展开成 <家目录>/x，且只在 ~ 形态时调 normalize。"""
    sftp = _FakeSftp()
    assert rr._expand_tilde(sftp, "~/dzuav_deploy/air_ground") == \
        "/home/nvidia/dzuav_deploy/air_ground"
    assert rr._expand_tilde(sftp, "~/x") == "/home/nvidia/x"
    assert sftp.calls == [".", "."]


def test_expand_tilde_absolute_untouched():
    """/abs 与相对路径形态：原样返回，不得调 normalize（证明没碰网络）。"""
    class _Bomb:
        def normalize(self, p):
            raise AssertionError("非 ~ 路径不得调 normalize")

    assert rr._expand_tilde(_Bomb(), "/abs/path") == "/abs/path"
    assert rr._expand_tilde(_Bomb(), "relative/dir") == "relative/dir"
