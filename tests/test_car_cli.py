"""车端动作入口 `car.py` 的测试。

它和 `run.py` 是两个入口，刻意分开：
    run.py  只在本机跑（测试/仿真/复盘），不会动车
    car.py  对车下命令，**`move --really-move` 会让车真的动起来**

所以这里最要紧的两条测试是：
    1. `move` **默认不带** AG_CONFIRM_MOVE（不给这个环境变量，车端脚本就只 dry_run）
    2. `--really-move` 才把它加上（而且只有它加）
字段契约那几条是防"入口少个参数直接崩"（`run.py all` 踩过）。
"""

import argparse
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import car  # noqa: E402

SUBCMDS = ["restore", "move", "stop", "logs"]

REQUIRED_ATTRS = {
    "restore": ("host", "user", "func"),
    "move": ("host", "user", "really_move", "run_s", "use_air", "no_judgment", "timeout", "func"),
    "stop": ("host", "user", "func"),
    "logs": ("host", "user", "remote", "to", "out", "strict", "func"),
}


def _parse(argv):
    return car.build_parser().parse_args(argv)


@pytest.mark.parametrize("sub", SUBCMDS)
def test_subcommand_help_runs(sub):
    import subprocess
    r = subprocess.run([sys.executable, str(ROOT / "car.py"), sub, "--help"],
                       cwd=str(ROOT), capture_output=True, timeout=120,
                       encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stderr[-1200:]
    assert "usage" in r.stdout.lower()


@pytest.mark.parametrize("sub", sorted(REQUIRED_ATTRS))
def test_namespace_has_required_attrs(sub):
    ns = _parse([sub, "192.0.2.1"])
    missing = [a for a in REQUIRED_ATTRS[sub] if not hasattr(ns, a)]
    assert not missing, f"{sub}: 缺少 {missing}"
    assert callable(ns.func)


def test_no_subcommand_prints_help_and_returns_2(capfd):
    assert car.main([]) == 2
    assert "usage" in capfd.readouterr().out.lower()


def _capture_script_env(monkeypatch):
    """截获 _script 调用，返回 (返回值, 收到的参数)。"""
    seen = {}

    def fake_script(name, host, user, timeout, env=None, out=None):
        seen["name"] = name
        seen["env"] = list(env or [])
        seen["host"] = host
        return 0

    monkeypatch.setattr(car, "_script", fake_script)
    return seen


def test_move_default_does_not_confirm_motion(monkeypatch, capfd):
    """★ 最关键的一条：默认不许发车。"""
    seen = _capture_script_env(monkeypatch)
    assert car.main(["move", "192.0.2.1"]) == 0
    out = capfd.readouterr().out
    assert "AG_CONFIRM_MOVE=1" not in seen["env"], "默认不该带发车确认"
    assert seen["name"] == "car_move_dryrun.sh"
    assert "车不会动" in out


def test_move_really_move_adds_confirm(monkeypatch, capfd):
    seen = _capture_script_env(monkeypatch)
    assert car.main(["move", "192.0.2.1", "--really-move", "--run-s", "30"]) == 0
    out = capfd.readouterr().out
    assert "AG_CONFIRM_MOVE=1" in seen["env"]
    assert "AG_RUN_S=30" in seen["env"]
    assert "真发车" in out                      # 必须给出显眼警告
    assert "遥控器" in out                      # 提醒能紧急接管


def test_move_passes_flags_through(monkeypatch):
    seen = _capture_script_env(monkeypatch)
    car.main(["move", "192.0.2.1", "--use-air", "--no-judgment"])
    assert "AG_USE_AIR=1" in seen["env"]
    assert "AG_NO_JUDGMENT=1" in seen["env"]


def test_restore_uses_restore_script(monkeypatch):
    seen = _capture_script_env(monkeypatch)
    car.main(["restore", "192.0.2.1"])
    assert seen["name"] == "car_restore_official.sh"


def test_stop_uses_stop_script(monkeypatch):
    seen = _capture_script_env(monkeypatch)
    car.main(["stop", "192.0.2.1"])
    assert seen["name"] == "car_stop_all.sh"


def test_logs_bad_host_reports_clearly(monkeypatch, capsys, tmp_path):
    monkeypatch.setattr(car, "_remote", lambda _args: 4)
    rc = car.main(["logs", "192.0.2.1", "--to", str(tmp_path / "x")])
    assert rc == 4
    assert "取回失败" in capsys.readouterr().err


def test_scripts_are_not_confused_with_run_py():
    """`car.py` 和 `run.py` 必须是两个独立入口 —— 别把动车的东西并进 run.py。"""
    run_src = (ROOT / "run.py").read_text(encoding="utf-8")
    assert "car_move_dryrun" not in run_src, "动车的脚本不该出现在 run.py 里"
    assert "AG_CONFIRM_MOVE" not in run_src
    doc = (ROOT / "car.py").read_text(encoding="utf-8")
    assert "run.py" in doc and "别混" in doc
