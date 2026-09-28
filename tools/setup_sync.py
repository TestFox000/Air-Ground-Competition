#!/usr/bin/env python3
"""一键配好双机同步（每台机器只跑一次）

用法：

    python tools/setup_sync.py --machine B --token <Gitee私人令牌>

它会依次做这些事（**幂等，重复跑不会出问题**）：

    1. 接上 Gitee 远端（没有就加，有就核对，地址不对就改）
    2. 配提交身份（仓库级，不动你的全局 git 配置）
    3. 配免交互凭据（写成 store 方式，并踢掉会捣乱系统级 helper）
    4. 装 post-commit 钩子 —— 提交即推送
    5. 建定时轮询 —— Windows 用 schtasks，Linux 用 crontab
    6. 自检并打印结果

只配钩子、不要定时任务：

    python tools/setup_sync.py --machine B --no-schedule

换定时间隔（分钟）：

    python tools/setup_sync.py --machine B --interval 5
"""

from __future__ import annotations

import argparse
import os
import platform
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_REMOTE = "https://gitee.com/circle-of-joy/air_ground.git"
DEFAULT_USER = "circle-of-joy"
DEFAULT_EMAIL = "circle-of-joy@noreply.gitee.com"
TASK_NAME = "air_ground_sync"

_NO_WINDOW = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0

OK = "[ OK ]"
WARN = "[WARN]"
FAIL = "[FAIL]"


def git(*args: str) -> tuple[int, str, str]:
    proc = subprocess.run(
        ["git", *args],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        creationflags=_NO_WINDOW,
    )
    return proc.returncode, (proc.stdout or "").strip(), (proc.stderr or "").strip()


def is_repo() -> bool:
    return git("rev-parse", "--is-inside-work-tree")[0] == 0


# --------------------------------------------------------------------------- 1
def step_remote(url: str) -> bool:
    code, current, _ = git("remote", "get-url", "origin")
    if code != 0:
        code, out, err = git("remote", "add", "origin", url)
        if code != 0:
            print(f"{FAIL} 添加远端失败：{err or out}")
            return False
        print(f"{OK} 远端已添加：{url}")
        return True

    if current != url:
        git("remote", "set-url", "origin", url)
        print(f"{OK} 远端地址已更新：{current} -> {url}")
    else:
        print(f"{OK} 远端已就绪：{url}")
    return True


# --------------------------------------------------------------------------- 2
def step_identity() -> None:
    name = git("config", "--local", "user.name")[1]
    email = git("config", "--local", "user.email")[1]
    if name and email:
        print(f"{OK} 提交身份已就绪：{name} <{email}>")
        return
    git("config", "--local", "user.name", DEFAULT_USER)
    git("config", "--local", "user.email", DEFAULT_EMAIL)
    print(f"{OK} 已配提交身份：{DEFAULT_USER} <{DEFAULT_EMAIL}>")
    print("      想换成自己的邮箱：git config --local user.email \"你的邮箱\"")


# --------------------------------------------------------------------------- 3
def step_credential(token: str) -> bool:
    if not token:
        print(f"{WARN} 没给 --token，跳过凭据配置（已配过的机器可以不加）")
        return True

    # 仓库级把 helper 链重置成只有 store：
    # 系统级的 Git Credential Manager 会反复送出一个错误的旧用户名，
    # 导致 push 永远认证失败，而且非交互环境下它连弹窗都不弹。
    git("config", "--local", "credential.helper", "")
    git("config", "--local", "--add", "credential.helper", "store")

    cred_file = Path.home() / ".git-credentials"
    keep: list[str] = []
    if cred_file.exists():
        try:
            keep = [
                ln
                for ln in cred_file.read_text(encoding="utf-8", errors="replace").splitlines()
                if ln.strip() and "gitee.com" not in ln
            ]
        except OSError:
            keep = []

    keep.append(f"https://{DEFAULT_USER}:{token}@gitee.com")
    cred_file.write_text("\n".join(keep) + "\n", encoding="utf-8")
    if os.name != "nt":
        try:
            cred_file.chmod(0o600)
        except OSError:
            pass

    print(f"{OK} 凭据已写入 {cred_file}（helper 链 = store）")
    print("      令牌是明文存的，只有本机当前用户能读")
    return True


# --------------------------------------------------------------------------- 4
def step_hook() -> bool:
    """委托给 sync_progress.py 装钩子 —— 钩子只留一套实现，免得两边打架。"""
    if not (ROOT / ".git").is_dir():
        print(f"{FAIL} 这里不是 Git 仓库，跳过钩子")
        return False
    proc = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "sync_progress.py"), "--install-hook"],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        creationflags=_NO_WINDOW,
    )
    if proc.returncode != 0:
        print(f"{FAIL} 装钩子失败：{(proc.stderr or proc.stdout).strip()}")
        return False
    print(f"{OK} 已装「提交即推送」钩子")
    return True


# --------------------------------------------------------------------------- 5
def step_schedule_windows(interval: int, machine: str, log: Path, mode: str) -> bool:
    pythonw = Path(sys.executable)
    candidate = pythonw.with_name("pythonw.exe")
    if candidate.exists():
        pythonw = candidate
    else:
        print(f"{WARN} 找不到 pythonw.exe，会用 python.exe（每轮会闪一个黑框）")

    script = ROOT / "tools" / "sync_progress.py"
    # schtasks 要求整个命令作为一个参数；用 list 形式交给 subprocess，它会正确传递内嵌引号
    tr = f'"{pythonw}" "{script}"{mode} --machine {machine} --log "{log}"'
    proc = subprocess.run(
        [
            "schtasks", "/create", "/tn", TASK_NAME,
            "/sc", "minute", "/mo", str(interval), "/f", "/tr", tr,
        ],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        creationflags=_NO_WINDOW,
    )
    if proc.returncode != 0:
        print(f"{FAIL} 建定时任务失败：{(proc.stderr or proc.stdout).strip()}")
        return False
    print(f"{OK} 定时轮询已建：每 {interval} 分钟（任务名 {TASK_NAME}）")
    print(f"      日志：{log}")
    return True


def step_schedule_linux(interval: int, machine: str, log: Path, mode: str) -> bool:
    script = ROOT / "tools" / "sync_progress.py"
    try:
        current = subprocess.run(
            ["crontab", "-l"], capture_output=True, text=True, encoding="utf-8", errors="replace"
        ).stdout
    except FileNotFoundError:
        print(f"{FAIL} 这台机器没有 crontab，跳过定时轮询（钩子仍然有效）")
        return False

    lines = [ln for ln in (current or "").splitlines() if "sync_progress.py" not in ln]
    lines.append(
        f"*/{interval} * * * * cd {ROOT} && {sys.executable} tools/sync_progress.py"
        f"{mode} --machine {machine} --log {log} >/dev/null 2>&1"
    )
    proc = subprocess.run(
        ["crontab", "-"],
        input="\n".join(lines) + "\n",
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if proc.returncode != 0:
        print(f"{FAIL} 写 crontab 失败：{(proc.stderr or proc.stdout).strip()}")
        return False
    print(f"{OK} 定时轮询已建：每 {interval} 分钟（crontab）")
    print(f"      日志：{log}")
    return True


def step_schedule(interval: int, machine: str, full_auto: bool) -> bool:
    log = Path.home() / "air_ground_sync.log"
    # 默认只拉取：定时任务每 N 分钟跑一次，如果让它自动提交，
    # 会把你**还没写完**的代码提交并推给对方，跟看板 §6「push 前必须本地全绿」直接冲突。
    # 推送方向交给 post-commit 钩子即可，实时性一点不损失（见看板 §9.2③）。
    mode = "" if full_auto else " --pull-only"
    if platform.system() == "Windows":
        return step_schedule_windows(interval, machine, log, mode)
    return step_schedule_linux(interval, machine, log, mode)


# --------------------------------------------------------------------------- 6
def step_selfcheck() -> None:
    print()
    print("-" * 60)
    proc = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "sync_progress.py"), "--status"],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        creationflags=_NO_WINDOW,
    )
    if (proc.stdout or "").strip():
        print((proc.stdout or "").strip())
    if (proc.stderr or "").strip():
        print((proc.stderr or "").strip())
    print("-" * 60)
    if proc.returncode != 0:
        print(f"{WARN} 自检有异常，看上面的输出")
    else:
        print(f"{OK} 自检通过")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="一键配好双机同步：远端 / 身份 / 凭据 / 钩子 / 定时轮询",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--machine", required=True, help="本机代号，A 或 B（写进提交信息）")
    ap.add_argument("--token", default="", help="Gitee 私人令牌（不填则跳过凭据配置）")
    ap.add_argument("--remote", default=DEFAULT_REMOTE, help="远端仓库地址")
    ap.add_argument("--interval", type=int, default=3, help="定时轮询间隔，分钟（默认 3）")
    ap.add_argument("--no-schedule", action="store_true", help="只配钩子，不建定时任务")
    ap.add_argument(
        "--full-auto",
        action="store_true",
        help="定时任务也自动提交+推送（默认只拉取 --pull-only，理由见看板 §9.2③）",
    )
    args = ap.parse_args()

    print(f"配置双机同步 —— 本机代号 {args.machine}")
    print(f"项目目录：{ROOT}")
    print()

    if not is_repo():
        print(f"{FAIL} {ROOT} 不是 Git 仓库。先把仓库 clone 下来再跑这个脚本：")
        print(f"       git clone {args.remote} air_ground")
        return 1

    ok = step_remote(args.remote)
    step_identity()
    if not step_credential(args.token):
        ok = False
    if not step_hook():
        ok = False

    if args.no_schedule:
        print(f"{WARN} 按 --no-schedule 要求，不建定时任务")
    elif not step_schedule(args.interval, args.machine, args.full_auto):
        ok = False

    step_selfcheck()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
