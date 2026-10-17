"""remote_run.read_password 统一密码守卫的单测（2026-10-17 收编配套）。

背景：patch_deploy/orin_deploy/car_deploy/preflight 的密码守卫原是同一段
逻辑的五个副本，报错文案各异；现已全部收编到 remote_run.read_password
（参数化 env_name/flag/prompt_user/fallback_envs/hint）。本文件钉住：
  * 优先级：--password > 主环境变量 > fallback_envs > 交互提问
  * 参数化：env_name（AG_ORIN_PASS/AG_CAR_PASS 语义不变）、flag（preflight
    机载侧用 orin_password 属性）、prompt_user（getpass 提示语身份串）
  * 非交互守卫：无 tty 且无密码时必须响亮 SystemExit（且点名该设哪个环境
    变量），绝不静默死等标准输入（2026-10-01 现场坑：卡死被误判成"车不通"）
"""

from __future__ import annotations

import importlib.util
import sys
import types
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


def _args(**kw):
    """最小 argparse 替身：默认无密码、带 user/host。"""
    base = {"password": None, "user": "duzhong", "host": "192.0.2.1"}
    base.update(kw)
    return types.SimpleNamespace(**base)


class _NoTty:
    """非交互终端替身（CI/管道形态）。"""
    def isatty(self):
        return False


class _Tty:
    """交互终端替身。"""
    def isatty(self):
        return True


# ---------------------------------------------------------------- 优先级与参数化

def test_flag_password_wins_over_env(monkeypatch):
    """--password 优先于一切环境变量。"""
    monkeypatch.setenv("AG_CAR_PASS", "env_pw")
    assert rr.read_password(_args(password="cli_pw")) == "cli_pw"


def test_default_env_is_car_pass(monkeypatch):
    """兼容形态：裸 read_password(args) 仍读 AG_CAR_PASS（remote_run 原语义）。"""
    monkeypatch.setenv("AG_CAR_PASS", "car_pw")
    assert rr.read_password(_args()) == "car_pw"


def test_env_name_parameterized(monkeypatch):
    """机载工具传 env_name="AG_ORIN_PASS"：读机载变量，不认车端变量。"""
    monkeypatch.setenv("AG_ORIN_PASS", "orin_pw")
    monkeypatch.setenv("AG_CAR_PASS", "car_pw")
    assert rr.read_password(_args(), env_name="AG_ORIN_PASS") == "orin_pw"


def test_fallback_envs_used_only_when_primary_missing(monkeypatch):
    """patch_deploy 形态：主变量缺失才兜底 AG_CAR_PASS；主变量在则不兜底。"""
    monkeypatch.delenv("AG_ORIN_PASS", raising=False)
    monkeypatch.setenv("AG_CAR_PASS", "car_pw")
    assert rr.read_password(_args(), env_name="AG_ORIN_PASS",
                            fallback_envs=("AG_CAR_PASS",)) == "car_pw"
    monkeypatch.setenv("AG_ORIN_PASS", "orin_pw")
    assert rr.read_password(_args(), env_name="AG_ORIN_PASS",
                            fallback_envs=("AG_CAR_PASS",)) == "orin_pw"


def test_flag_parameterized_orin_password(monkeypatch):
    """preflight 机载侧形态：密码读 args.orin_password 而不是 args.password。"""
    monkeypatch.delenv("AG_ORIN_PASS", raising=False)
    a = _args()
    a.orin_password = "flag_pw"
    assert rr.read_password(a, env_name="AG_ORIN_PASS",
                            flag="orin_password") == "flag_pw"


# ---------------------------------------------------------------- 非交互守卫

def test_non_tty_without_password_raises_and_names_env(monkeypatch):
    """无 tty + 无密码：SystemExit，且报错点名该设的环境变量（不静默挂起）。"""
    monkeypatch.delenv("AG_ORIN_PASS", raising=False)
    monkeypatch.delenv("AG_CAR_PASS", raising=False)
    monkeypatch.setattr(sys, "stdin", _NoTty())
    with pytest.raises(SystemExit) as exc:
        rr.read_password(_args(), env_name="AG_ORIN_PASS")
    assert "AG_ORIN_PASS" in str(exc.value)


def test_non_tty_hint_appended(monkeypatch):
    """remote_run 自用 hint：非交互报错尾部带车端密码文档指引。"""
    monkeypatch.delenv("AG_CAR_PASS", raising=False)
    monkeypatch.setattr(sys, "stdin", _NoTty())
    with pytest.raises(SystemExit) as exc:
        rr.read_password(_args(), hint=rr._CAR_DOCS_HINT)
    assert "05_VNC" in str(exc.value)


# ---------------------------------------------------------------- 交互提问

def test_tty_prompt_uses_prompt_user(monkeypatch):
    """有 tty：走 getpass，提示语用自定义 prompt_user 身份串。"""
    import getpass
    prompts = []
    monkeypatch.delenv("AG_CAR_PASS", raising=False)
    monkeypatch.setattr(sys, "stdin", _Tty())
    monkeypatch.setattr(getpass, "getpass",
                        lambda p: prompts.append(p) or "typed_pw")
    assert rr.read_password(_args(), prompt_user="duzhong@192.168.55.1") == "typed_pw"
    assert prompts == ["duzhong@192.168.55.1 的密码: "]


def test_tty_prompt_defaults_to_user_at_host(monkeypatch):
    """prompt_user 缺省时取 args.user@args.host。"""
    import getpass
    prompts = []
    monkeypatch.delenv("AG_CAR_PASS", raising=False)
    monkeypatch.setattr(sys, "stdin", _Tty())
    monkeypatch.setattr(getpass, "getpass",
                        lambda p: prompts.append(p) or "typed_pw")
    rr.read_password(_args(user="nvidia", host="10.0.0.2"))
    assert prompts == ["nvidia@10.0.0.2 的密码: "]


# ---------------------------------------------------------------- 收编后的委托壳

def test_orin_deploy_delegates_with_orin_env(monkeypatch):
    """orin_deploy.read_password 壳：仍走 AG_ORIN_PASS。"""
    od = load_module("orin_deploy", REPO / "tools" / "orin_deploy.py")
    monkeypatch.setenv("AG_ORIN_PASS", "orin_pw")
    assert od.read_password(_args()) == "orin_pw"


def test_patch_deploy_delegates_with_car_fallback(monkeypatch):
    """patch_deploy.read_password 壳：AG_ORIN_PASS 缺失时仍兜底 AG_CAR_PASS。"""
    pd = load_module("patch_deploy", REPO / "tools" / "patch_deploy.py")
    monkeypatch.delenv("AG_ORIN_PASS", raising=False)
    monkeypatch.setenv("AG_CAR_PASS", "car_pw")
    assert pd.read_password(_args()) == "car_pw"


def test_car_deploy_delegates_with_car_env(monkeypatch):
    """car_deploy.get_password 壳：仍走 AG_CAR_PASS。"""
    cd = load_module("car_deploy", REPO / "tools" / "car_deploy.py")
    monkeypatch.setenv("AG_CAR_PASS", "car_pw")
    assert cd.get_password(_args()) == "car_pw"
