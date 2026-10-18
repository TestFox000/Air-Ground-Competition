"""诊断工具 run 包装的 allow_nonzero 语义测试（P1-6 自杀式诊断修复）。

背景：check_stack_health.py / factory_flight_diag.py 的旧 run 把远程命令的
非零退出码当"连接无响应"重试后 SystemExit——但 `ros2 topic echo --once`
超时（124）、`grep` 无匹配（1）恰恰是要诊断的症状，导致"话题无响应"分支
永远不可达。修复后：allow_nonzero=True 返回 (code, out) 由检查项自判，
只有 SSH 连接异常才 SystemExit。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
TOOLS = REPO / "tools"


def _load(name, file):
    spec = importlib.util.spec_from_file_location(name, file)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


stack_health = _load("check_stack_health", TOOLS / "check_stack_health.py")
factory_diag = _load("factory_flight_diag", TOOLS / "factory_flight_diag.py")


@pytest.fixture()
def no_sleep(monkeypatch):
    """重试间隔 5s 不真实等。"""
    monkeypatch.setattr("time.sleep", lambda *_a, **_k: None)


def _patch_remote_run(monkeypatch, mod, *, result=None, exc=None):
    """把工具模块里的 remote_run.run 换成假实现。"""
    calls = []

    def fake_run(host, user, pw, cmd, timeout, port=22):
        calls.append(cmd)
        if exc is not None:
            raise exc
        return result

    monkeypatch.setattr(mod.remote_run, "run", fake_run)
    return calls


@pytest.mark.parametrize("mod", [stack_health, factory_diag])
def test_allow_nonzero_returns_code_and_out(monkeypatch, no_sleep, mod):
    """超时 124（topic echo 症状）不再 SystemExit，而是返回 (124, out) 供自判。"""
    _patch_remote_run(monkeypatch, mod, result=(124, "", "timeout"))
    code, out = mod.run("h", "u", "p", 22, "timeout 8 ros2 topic echo --once /x",
                        allow_nonzero=True)
    assert code == 124
    assert out == ""


@pytest.mark.parametrize("mod", [stack_health, factory_diag])
def test_grep_no_match_is_reachable(monkeypatch, no_sleep, mod):
    """grep 无匹配（退出码 1）同样放行——'话题无响应'分支由此可达。"""
    _patch_remote_run(monkeypatch, mod, result=(1, "", ""))
    code, out = mod.run("h", "u", "p", 22, "ros2 topic info /x | grep 'Publisher count'",
                        allow_nonzero=True)
    assert code == 1 and out == ""


@pytest.mark.parametrize("mod", [stack_health, factory_diag])
def test_zero_exit_still_returns_plain_out(monkeypatch, no_sleep, mod):
    """默认模式（allow_nonzero=False）零退出照旧返回纯 out 字符串。"""
    _patch_remote_run(monkeypatch, mod, result=(0, "3\n", ""))
    out = mod.run("h", "u", "p", 22, "pgrep -xc fastlio_mapping || echo 0")
    assert out == "3\n"


@pytest.mark.parametrize("mod", [stack_health, factory_diag])
def test_nonzero_without_allow_still_systemexit(monkeypatch, no_sleep, mod):
    """默认模式非零退出：保持旧行为（重试后 SystemExit），不静默吞掉。"""
    calls = _patch_remote_run(monkeypatch, mod, result=(2, "", "boom"))
    with pytest.raises(SystemExit):
        mod.run("h", "u", "p", 22, "some_cmd")
    assert len(calls) == 3   # 重试 3 次后才放弃


@pytest.mark.parametrize("mod", [stack_health, factory_diag])
def test_connection_exception_systemexit_even_with_allow_nonzero(
        monkeypatch, no_sleep, mod):
    """SSH 连接异常即使 allow_nonzero=True 也必须 SystemExit（连接不通才配叫无响应）。"""
    _patch_remote_run(monkeypatch, mod, exc=OSError("WinError 10054"))
    with pytest.raises(SystemExit):
        mod.run("h", "u", "p", 22, "true", allow_nonzero=True)
