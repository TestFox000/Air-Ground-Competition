"""双机协作同步 `tools/sync_progress.py` 的测试。

这个脚本的**安全属性**比功能更重要：它跑在两台机器之间，
一旦"自动猜冲突"或"force 覆盖"，就会静默吃掉对方的工作。
所以下面既有功能测试，也有几条**安全契约**测试 —— 它们盯的是源码里
不许出现危险操作，属于回归防线。
"""

import importlib.util
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
NEED_GIT = pytest.mark.skipif(shutil.which("git") is None, reason="没装 git")


def _load():
    spec = importlib.util.spec_from_file_location(
        "sync_progress", ROOT / "tools" / "sync_progress.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)  # type: ignore[union-attr]
    return mod


# ------------------------------------------------------------------ 安全契约
def _git_call_args() -> list:
    """取出源码里**真正传给 git 的字面量参数**。

    为什么不直接搜源码文本：注释、docstring、以及"告诉用户怎么手工解决冲突"
    的说明里都会出现 `reset --hard`、`rebase --continue` 这些字样 ——
    那些是文字，不是行为。只有作为参数传进 git 的才是真的会执行。
    """
    import ast

    tree = ast.parse((ROOT / "tools" / "sync_progress.py").read_text(encoding="utf-8"))
    args: list = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and (
                isinstance(node.func, ast.Name) or isinstance(node.func, ast.Attribute)):
            for a in node.args:
                if isinstance(a, ast.Constant) and isinstance(a.value, str):
                    args.append(a.value)
    return args


def test_no_destructive_git_arguments():
    """★ 这些参数会**吃掉对方的提交**，一个都不许出现在 git 调用里。

    写得再方便，只要有一条 `push --force`，某天就会把另一台机器一天的活覆盖掉。
    """
    args = _git_call_args()
    for forbidden in ("--force", "-f", "--hard", "-fd", "--autostash", "clean"):
        assert forbidden not in args, f"同步脚本真会执行危险操作: {forbidden}"


def test_conflict_is_left_to_the_human():
    """★ 冲突时不许自动选边，也不许一 abort 了事。

    - `-X` / `--merge` / `--strategy` = 替用户猜"留哪一份"，绝对不行
    - `--abort` = 会把冲突上下文一起丢掉，用户就没法对着两边原文决定了
      （2026-09-28 与 B 机对齐：采用它们"留在原地"的做法）
    - `--continue` 只允许在"冲突文件已全部解决"之后调用，
      由下面的端到端测试验证（不能让它变成替用户"接着做完"）
    """
    args = _git_call_args()
    for forbidden in ("-X", "--merge", "--strategy", "--abort"):
        assert forbidden not in args, f"冲突处理里不该出现: {forbidden}"
    src = (ROOT / "tools" / "sync_progress.py").read_text(encoding="utf-8")
    assert "git add" in src and "rebase --abort" in src, "得告诉人怎么收场"


def test_hook_disables_prompt_to_avoid_hanging_commit():
    """钩子里必须禁掉交互提示，否则凭据没配好时用户的 `git commit` 会被卡住。"""
    mod = _load()
    assert "GIT_TERMINAL_PROMPT=0" in mod.HOOK_BODY


def test_git_calls_never_prompt(monkeypatch, tmp_path):
    """所有 git 调用都要带 GIT_TERMINAL_PROMPT=0（否则非交互场景会挂死）。"""
    mod = _load()
    seen = {}

    class FakeRun:
        returncode = 0
        stdout = ""
        stderr = ""

    def fake_run(cmd, cwd=None, capture_output=True, encoding=None, errors=None,
                 env=None, creationflags=None):
        seen["env"] = env
        seen["flags"] = creationflags
        return FakeRun()

    monkeypatch.setattr(mod.subprocess, "run", fake_run)
    mod.Git(tmp_path)("status")
    assert seen["env"]["GIT_TERMINAL_PROMPT"] == "0"


# ------------------------------------------------------------------ 功能
def test_help_runs():
    r = subprocess.run([sys.executable, str(ROOT / "tools" / "sync_progress.py"), "--help"],
                       capture_output=True, timeout=120, encoding="utf-8", errors="replace")
    assert r.returncode == 0
    assert "sync" in r.stdout.lower() or "同步" in r.stdout


def test_repo_problems_reports_non_repo(tmp_path, monkeypatch):
    mod = _load()
    monkeypatch.setattr(mod, "ROOT", tmp_path)
    problems = mod.repo_problems(mod.Git(tmp_path))
    assert problems and "不是 git 仓库" in problems[0]


@NEED_GIT
def test_repo_problems_clean_on_fresh_repo(tmp_path, monkeypatch):
    """刚 init 的新仓库：有提交就应当无问题（这条同时保证 fsck 检查不会误报）。"""
    mod = _load()
    git = mod.Git(tmp_path)
    git("init", "-b", "main")
    git("config", "user.email", "t@t")
    git("config", "user.name", "t")
    (tmp_path / "a.txt").write_text("hi", encoding="utf-8")
    git("add", "-A")
    git("commit", "-m", "init")
    monkeypatch.setattr(mod, "ROOT", tmp_path)
    assert mod.repo_problems(git) == []


@NEED_GIT
def test_status_without_remote_says_so(tmp_path, monkeypatch, capsys):
    """没配远端时要明确说"两台机器还谈不上协作"，而不是假装成功。"""
    mod = _load()
    git = mod.Git(tmp_path)
    git("init", "-b", "main")
    git("config", "user.email", "t@t")
    git("config", "user.name", "t")
    (tmp_path / "a.txt").write_text("hi", encoding="utf-8")
    git("add", "-A")
    git("commit", "-m", "init")
    monkeypatch.setattr(mod, "ROOT", tmp_path)
    rc = mod.do_status(git, "origin", "main", mod.Logger(None))
    out = capsys.readouterr().out
    assert rc == 3
    assert "远端没配" in out


@NEED_GIT
def test_dirty_files_and_ahead_behind(tmp_path):
    mod = _load()
    git = mod.Git(tmp_path)
    git("init", "-b", "main")
    git("config", "user.email", "t@t")
    git("config", "user.name", "t")
    (tmp_path / "a.txt").write_text("hi", encoding="utf-8")
    git("add", "-A")
    git("commit", "-m", "init")
    assert mod.dirty_files(git) == []
    (tmp_path / "b.txt").write_text("new", encoding="utf-8")
    assert mod.dirty_files(git) == ["b.txt"]


def test_logger_never_raises_on_bad_path(tmp_path):
    """日志写不进去（比如路径是个文件）也绝不能影响同步。"""
    mod = _load()
    bad = tmp_path / "a_file"
    bad.write_text("x", encoding="utf-8")
    lg = mod.Logger(str(bad / "nope" / "x.log"))
    lg("hello")          # 不该抛异常
    lg.close()


# ------------------------------------------------------------------ 真实冲突（端到端）
def _git_in(cwd: Path, *args: str) -> None:
    subprocess.run(["git", *args], cwd=str(cwd), check=True, capture_output=True)


def _cfg(cwd: Path) -> None:
    _git_in(cwd, "config", "user.email", "t@t")
    _git_in(cwd, "config", "user.name", "t")


def _conflicting_pair(tmp_path: Path):
    """搭一对仓库并制造真冲突：bare 当远端，work 当本机，other 当"对面那台机器"。"""
    bare = tmp_path / "bare.git"
    subprocess.run(["git", "init", "--bare", "-b", "main", str(bare)],
                   check=True, capture_output=True)
    work = tmp_path / "work"
    subprocess.run(["git", "init", "-b", "main", str(work)], check=True, capture_output=True)
    _cfg(work)
    (work / "x.txt").write_text("base\n", encoding="utf-8")
    _git_in(work, "add", "-A")
    _git_in(work, "commit", "-m", "base")
    _git_in(work, "remote", "add", "origin", str(bare))
    _git_in(work, "push", "-u", "origin", "main")

    other = tmp_path / "other"
    subprocess.run(["git", "clone", str(bare), str(other)], check=True, capture_output=True)
    _cfg(other)

    (other / "x.txt").write_text("theirs\n", encoding="utf-8")   # 对面先改同一行
    _git_in(other, "add", "-A")
    _git_in(other, "commit", "-m", "theirs")
    _git_in(other, "push")

    (work / "x.txt").write_text("mine\n", encoding="utf-8")     # 本机也改同一行
    _git_in(work, "add", "-A")
    _git_in(work, "commit", "-m", "mine")
    return work, other


@NEED_GIT
def test_conflict_stops_and_keeps_markers_for_human(tmp_path, monkeypatch, capsys):
    """★ 两边改同一行：必须停下来、**保留冲突标记**，绝不自动决定留哪一份。"""
    work, _ = _conflicting_pair(tmp_path)
    mod = _load()
    monkeypatch.setattr(mod, "ROOT", work)
    g = mod.Git(work)

    rc = mod.do_sync(g, "origin", "main", "T", mod.Logger(None))
    out = capsys.readouterr().out
    assert rc == 2
    assert "冲突文件" in out and "x.txt" in out
    assert mod.in_conflict(g) is True, "不能自动 abort —— 那样冲突上下文就一起丢了"
    assert "<<<<<<<" in (work / "x.txt").read_text(encoding="utf-8")


@NEED_GIT
def test_resolved_conflict_finishes_on_next_run(tmp_path, monkeypatch, capsys):
    """★ 用户解完冲突再跑一次，脚本要接着把这次 rebase 收尾。

    这正是 B 机那版的死角：提示写着"解决完再跑一次脚本"，但脚本开头的冲突守卫
    会直接再报一次冲突 —— 用户**原地打转**出不去。
    """
    work, _ = _conflicting_pair(tmp_path)
    mod = _load()
    monkeypatch.setattr(mod, "ROOT", work)
    g = mod.Git(work)

    assert mod.do_sync(g, "origin", "main", "T", mod.Logger(None)) == 2
    capsys.readouterr()

    (work / "x.txt").write_text("merged\n", encoding="utf-8")   # 由人来决定留什么
    _git_in(work, "add", "x.txt")

    rc = mod.do_sync(g, "origin", "main", "T", mod.Logger(None))
    out = capsys.readouterr().out
    assert rc == 0, out
    assert mod.in_conflict(g) is False
    assert (work / "x.txt").read_text(encoding="utf-8") == "merged\n"
