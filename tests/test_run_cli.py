"""一键入口 `run.py` 的测试。

为什么值得单独测：`run.py` 是所有人（和 CI、和双击 run.bat 的人）唯一的入口，
一旦它坏了，**所有**能力都用不了。而它恰恰最不容易被单元测试覆盖 ——
真实事故就是 `python run.py all`（README 里写着的命令）因为少一个参数直接崩掉，
而 433 项单测全绿。

这里分三层：
    1. 每个子命令的 `--help` 能跑（argparse 没写坏）
    2. 解析出的字段能喂饱对应的 cmd_* 函数（这类"少一个字段"的崩溃）
    3. `log` 子命令真的能出报告（接线没断）
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import run  # noqa: E402

SUBCMDS = ["demo", "test", "check", "sim", "all", "log", "preflight",
           "car-fetch", "car-rehearse", "car-camera"]


def _parse(argv):
    return run.build_parser().parse_args(argv)


def test_parser_lists_every_subcommand():
    ns = run.build_parser().parse_args([])
    assert ns.cmd is None          # 不给子命令 → main 会退化成 demo


@pytest.mark.parametrize("sub", SUBCMDS)
def test_subcommand_help_runs(sub):
    # 必须显式 utf-8：Windows 下默认 GBK 会把中文 usage 解炸
    r = subprocess.run([sys.executable, str(ROOT / "run.py"), sub, "--help"],
                       cwd=str(ROOT), capture_output=True, timeout=120,
                       encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stderr[-1500:]
    assert "usage" in r.stdout.lower()


#: 每个子命令解析后**必须**带齐的字段 —— 就是它要调用的 cmd_* 会读的那些。
#: 少一个就是运行时 AttributeError（`all` 那次事故就是这么来的）。
REQUIRED_ATTRS = {
    "demo": ("steps", "confirm_frames", "mode", "noise_ratio", "seed", "func"),
    # all 复用 cmd_test + cmd_demo，所以两边的字段都得有
    "all": ("steps", "confirm_frames", "mode", "noise_ratio", "seed", "keyword", "func"),
    "test": ("keyword", "func"),
    "check": ("func",),
    "sim": ("mode", "func"),
    "log": ("dir", "session", "all", "json", "strict", "out", "max_gap", "timeline",
            "list", "func"),
    "car-fetch": ("host", "user", "remote", "to", "out", "strict", "func"),
    "car-rehearse": ("host", "user", "remote", "to", "out", "strict",
                     "build", "ws", "run_s", "func"),
    "car-camera": ("host", "user", "func"),
    "preflight": ("params", "run_tests", "func"),
}


@pytest.mark.parametrize("sub", sorted(REQUIRED_ATTRS))
def test_subcommand_namespace_has_required_attrs(sub):
    argv = [sub] + (["192.0.2.1"] if sub.startswith("car-") else [])
    ns = _parse(argv)
    missing = [a for a in REQUIRED_ATTRS[sub] if not hasattr(ns, a)]
    assert not missing, f"{sub}: 解析结果缺少 {missing}（cmd_* 会 AttributeError）"
    assert callable(ns.func)


def test_default_subcommand_is_demo(monkeypatch):
    """`python run.py` 不带子命令 = demo（双击 run.bat 的行为）。"""
    seen = {}

    def fake(args):
        seen["cmd"] = getattr(args, "cmd", None)
        seen["steps"] = args.steps
        return 0

    # set_defaults 在 build_parser() 时才取模块全局的 cmd_demo，所以这里能替换掉
    monkeypatch.setattr(run, "cmd_demo", fake)
    assert run.main([]) == 0
    assert seen == {"cmd": "demo", "steps": 60}


def test_run_bat_and_sh_reference_existing_entry():
    """双击用的 run.bat / run.sh 必须还在，且指向 run.py。"""
    for name in ("run.bat", "run.sh"):
        text = (ROOT / name).read_text(encoding="utf-8", errors="replace")
        assert "run.py" in text, f"{name} 没调用 run.py"


# ------------------------------------------------------------------ log 接线
def _fake_log(tmp_path: Path) -> Path:
    """造一份最小日志：收敛了、发了路线、有打击点。"""
    d = tmp_path / "sess"
    d.mkdir(parents=True, exist_ok=True)
    records = [
        {"ev": "session.begin", "start_cell": "N10", "goal_cell": "N11", "dry_run": False},
        {"ev": "intel.recv", "ok": True, "seq": 1, "mines": ["C3", "G8", "B2"]},
        {"ev": "intel.stable", "mines": ["C3", "G8", "B2"]},
        {"ev": "plan.built", "tasks": 3, "shoot_stops": ["D2"], "eta_s": 40.0},
        {"ev": "route.publish", "kind": "reset", "cmd": "reset:N10,y"},
        {"ev": "route.publish", "kind": "route", "cmd": "C3 G8 D2* N11,y",
         "shoot_stops": ["D2"]},
    ]
    lines = []
    for i, r in enumerate(records):
        rec = {"ts": 1000.0 + i, "mono": float(i), "seq": i + 1, "node": "car_node",
               "lvl": "info", "session": "s"}
        rec.update(r)
        lines.append(json.dumps(rec, ensure_ascii=False))
    (d / "car_node-1.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return tmp_path


def test_run_log_wires_through_to_report(tmp_path, capfd):
    """`run.py log --dir X` 必须真的出报告（子进程调用接线正确）。

    用 capfd 而不是 capsys：报告是**子进程**打印的，capsys 只拦 Python 层的
    sys.stdout，抓不到子进程直接写 fd 1 的内容。
    """
    d = _fake_log(tmp_path)
    assert run.main(["log", "--dir", str(d)]) == 0
    out = capfd.readouterr().out
    assert "跑车复盘" in out
    assert "严重 0" in out
    assert "D2" in out                      # 打击点要能看到


def test_run_log_out_writes_file(tmp_path):
    d = _fake_log(tmp_path)
    out_file = tmp_path / "report.md"
    assert run.main(["log", "--dir", str(d), "--out", str(out_file)]) == 0
    assert out_file.is_file()
    assert "跑车复盘" in out_file.read_text(encoding="utf-8")


def test_run_log_strict_returns_one_on_critical(tmp_path):
    """`--strict` 透传：有严重结论时退出码 1，可直接用于 CI。"""
    d = tmp_path / "sess"
    d.mkdir(parents=True, exist_ok=True)
    rec = {"ts": 1.0, "mono": 0.0, "seq": 1, "node": "car_node", "lvl": "info",
           "session": "s", "ev": "session.begin", "dry_run": False,
           "start_cell": "N10", "goal_cell": "N11"}
    rec2 = {"ts": 2.0, "mono": 1.0, "seq": 2, "node": "car_node", "lvl": "info",
            "session": "s", "ev": "intel.stable", "mines": ["C3"]}   # 收敛了却没发车
    (d / "car_node-1.jsonl").write_text(
        "\n".join(json.dumps(x, ensure_ascii=False) for x in (rec, rec2)) + "\n",
        encoding="utf-8")
    assert run.main(["log", "--dir", str(tmp_path), "--strict"]) == 1


def test_car_fetch_bad_host_does_not_crash(tmp_path, monkeypatch, capsys):
    """车连不上时要给人话提示，而不是抛栈。"""
    def fake_remote(_args):
        return 4

    monkeypatch.setattr(run, "_remote", fake_remote)
    rc = run.main(["car-fetch", "192.0.2.1", "--to", str(tmp_path / "x")])
    assert rc == 4
    assert "取回失败" in capsys.readouterr().err
