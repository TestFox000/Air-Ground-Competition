"""uav_wizard_menu 单测：零交互、零子进程、零网络（pick/wizard/prober 全注入）。

背景：菜单从 bat 搬进 Python（2026-10-21 闪退修复的方案一）——bat 只剩哑启动器。
这里锁住菜单的行为：分发正确、env 记忆、主机名/用户名白名单、EOF 不死循环。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]


def load_module(name: str, file: Path):
    spec = importlib.util.spec_from_file_location(name, file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


menu = load_module("uav_wizard_menu", REPO / "tools" / "uav_wizard_menu.py")


class Script:
    """按序吐出输入；记下每条 prompt（wizard 后的回车确认也在流里）。"""

    def __init__(self, *answers: str):
        self.answers = list(answers)
        self.prompts: list[str] = []

    def __call__(self, prompt: str = "") -> str:
        self.prompts.append(prompt)
        if not self.answers:
            raise EOFError
        return self.answers.pop(0)


class FakeWizard:
    def __init__(self):
        self.calls: list[tuple[tuple, str | None]] = []

    def __call__(self, args, password=None):
        self.calls.append((tuple(args), password))
        return 0


# ---------------------------------------------------------------- 白名单
class TestValidation:
    @pytest.mark.parametrize("host", ["10.171.116.104", "orin.local", "::1", "a-b_c.d"])
    def test_valid_host_accepts(self, host):
        assert menu.valid_host(host)

    @pytest.mark.parametrize("host", ["", "a b", "x;rm -rf", "1.2.3.4 `reboot`",
                                      "a|b", "a&b", "a\nb", "a\tb"])
    def test_valid_host_rejects(self, host):
        assert not menu.valid_host(host)

    @pytest.mark.parametrize("user", ["duzhong", "orin.op", "a_b-1"])
    def test_valid_user_accepts(self, user):
        assert menu.valid_user(user)

    @pytest.mark.parametrize("user", ["", "a b", "a;b", "root;id", "a:b", "a\nb"])
    def test_valid_user_rejects(self, user):
        assert not menu.valid_user(user)


# ---------------------------------------------------------------- env 记忆
class TestEnv:
    def test_roundtrip(self, tmp_path, monkeypatch):
        env_file = tmp_path / "uav_wizard.local.env"
        monkeypatch.setattr(menu, "ENV_FILE", env_file)
        menu.save_env("10.1.2.3", "duzhong")
        assert env_file.read_text(encoding="utf-8") == "ORIN_HOST=10.1.2.3\nORIN_USER=duzhong\n"
        assert menu.load_env() == {"ORIN_HOST": "10.1.2.3", "ORIN_USER": "duzhong"}

    def test_missing_file(self, tmp_path, monkeypatch):
        monkeypatch.setattr(menu, "ENV_FILE", tmp_path / "nope.env")
        assert menu.load_env() == {}


# ---------------------------------------------------------------- 菜单分发
class TestMenuLoop:
    def run(self, cfg, answers, wizard=None, prober=lambda h: True,
            askpw=lambda: None, opener=lambda: None):
        w = wizard or FakeWizard()
        rc = menu.menu_loop(cfg, pick=Script(*answers), wizard=w,
                            prober=prober, opener=opener, askpw=askpw)
        return rc, w

    def test_option_0_exits_clean(self):
        rc, _ = self.run({}, ["0"])
        assert rc == 0

    def test_eof_exits_not_loop(self):
        rc, _ = self.run({}, [])
        assert rc == 0        # EOF 直接退出（旧 bat 会死循环打印菜单）

    def test_rehearse_dispatch(self):
        rc, w = self.run({}, ["3", "", "0"])
        assert rc == 0
        assert w.calls == [(("--rehearse",), None)]

    def test_panel_uses_saved_host(self):
        rc, w = self.run({"ORIN_HOST": "10.1.2.3", "ORIN_USER": "duzhong"},
                         ["1", "", "0"], askpw=lambda: "pw")
        assert rc == 0
        args, pw = w.calls[0]
        assert args[:4] == ("--host", "10.1.2.3", "--user", "duzhong")
        assert pw == "pw"

    def test_panel_no_host_prompts_then_saves(self, tmp_path, monkeypatch):
        monkeypatch.setattr(menu, "ENV_FILE", tmp_path / "env")
        rc, w = self.run({}, ["1", "10.9.8.7", "", "", "0"])
        assert rc == 0
        assert w.calls[0][0][:4] == ("--host", "10.9.8.7", "--user", "duzhong")
        assert menu.load_env() == {"ORIN_HOST": "10.9.8.7", "ORIN_USER": "duzhong"}

    def test_panel_rejects_invalid_host_then_back_to_menu(self):
        rc, w = self.run({}, ["1", "bad host; whoami", "0"])
        assert rc == 0
        assert w.calls == []          # 非法主机名不得进入向导

    def test_check_mode_appends_flag(self):
        rc, w = self.run({"ORIN_HOST": "10.1.2.3", "ORIN_USER": "duzhong"},
                         ["2", "", "0"])
        assert rc == 0
        assert "--check" in w.calls[0][0]

    def test_set_host_saves(self, tmp_path, monkeypatch):
        env_file = tmp_path / "env"
        monkeypatch.setattr(menu, "ENV_FILE", env_file)
        rc, _ = self.run({}, ["4", "10.0.0.8", "orin", "0"])
        assert rc == 0
        assert menu.load_env() == {"ORIN_HOST": "10.0.0.8", "ORIN_USER": "orin"}

    def test_set_host_rejects_bad_user(self, tmp_path, monkeypatch):
        env_file = tmp_path / "env"
        monkeypatch.setattr(menu, "ENV_FILE", env_file)
        rc, _ = self.run({}, ["4", "10.0.0.8", "bad user", "0"])
        assert rc == 0
        assert env_file.exists() is False

    def test_saved_bad_user_blocks_launch_with_hint(self, capsys):
        rc, w = self.run({"ORIN_HOST": "10.1.2.3", "ORIN_USER": "bad;user"},
                         ["1", "0"])
        assert rc == 0
        assert w.calls == []          # env 里被污染的用户名不能直通向导
        assert "选项 [4]" in capsys.readouterr().out

    def test_open_manual_dispatch(self):
        opened = []
        rc, _ = self.run({}, ["5", "0"], opener=lambda: opened.append(1))
        assert rc == 0 and opened == [1]


# ---------------------------------------------------------------- 退出健壮性
class TestEofEverywhere:
    """回归（2026-10-22 复审）：EOF 只在菜单选项处被捕获，"回车返回菜单"/
    选项 4 输入等 pick 点裸抛 traceback——违背"EOF 优雅退出"契约。"""

    def test_eof_at_return_to_menu_prompt(self):
        w = FakeWizard()
        rc = menu.menu_loop({}, pick=Script("3"), wizard=w,   # 第二个 pick 即 EOF
                            prober=lambda h: True, opener=lambda: None,
                            askpw=lambda: None)
        assert rc == 0
        assert w.calls == [(("--rehearse",), None)]   # 向导已跑过，退出不崩

    def test_eof_at_option4_prompt(self):
        rc = menu.menu_loop({}, pick=Script("4"), wizard=FakeWizard(),
                            prober=lambda h: True, opener=lambda: None)
        assert rc == 0


# ---------------------------------------------------------------- 密码环境还原
class TestPasswordEnvRestore:
    """回归（2026-10-22 复审）：run_wizard 退出时无条件 pop AG_ORIN_PASS——
    用户在 shell 里设好的密码被第一次调用抹掉，后续面板又要重新问。"""

    def _stub_wizard(self, seen):
        class Stub:
            def main(self, argv):
                import os as _os
                seen.append(_os.environ.get("AG_ORIN_PASS"))
                return 0
        return Stub()

    def test_restores_user_provided_env(self, monkeypatch):
        monkeypatch.setenv("AG_ORIN_PASS", "user_pw")
        seen = []
        monkeypatch.setitem(sys.modules, "uav_wizard", self._stub_wizard(seen))
        assert menu.run_wizard(["--host", "x"], "menu_pw") == 0
        assert seen == ["menu_pw"]                       # 向导内看到菜单传的
        import os
        assert os.environ["AG_ORIN_PASS"] == "user_pw"   # 退出后还原用户原值

    def test_removes_menu_password_when_no_prior(self, monkeypatch):
        monkeypatch.delenv("AG_ORIN_PASS", raising=False)
        seen = []
        monkeypatch.setitem(sys.modules, "uav_wizard", self._stub_wizard(seen))
        assert menu.run_wizard(["--host", "x"], "menu_pw") == 0
        import os
        assert "AG_ORIN_PASS" not in os.environ          # 没有原值则清干净

    def test_no_password_leaves_env_untouched(self, monkeypatch):
        monkeypatch.setenv("AG_ORIN_PASS", "user_pw")
        seen = []
        monkeypatch.setitem(sys.modules, "uav_wizard", self._stub_wizard(seen))
        assert menu.run_wizard(["--rehearse"], None) == 0
        import os
        assert os.environ["AG_ORIN_PASS"] == "user_pw"
        assert seen == ["user_pw"]                       # 向导看到的是用户原值
