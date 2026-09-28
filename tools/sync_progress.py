#!/usr/bin/env python3
"""双机进度同步（air_ground）

一条命令完成「拉取对方进展 → 提交自己的改动 → 推给对方」。
**冲突或异常时一律停下来报清楚，绝不自动猜测、绝不覆盖对方的工作。**

用法：

    python tools/sync_progress.py                 # 完整同步（拉 → 提交 → 推）
    python tools/sync_progress.py --pull-only     # 只看对方进展，不动自己的改动
    python tools/sync_progress.py --status        # 只看差异概况，不碰任何东西
    python tools/sync_progress.py --install-hook  # 装 post-commit 钩子：每次提交立刻推送

配合 Windows 定时任务即可近似「实时」（每 3 分钟一轮）：

    schtasks /create /tn air_ground_sync /sc minute /mo 3 /f ^
      /tr "python \"E:\\CodeBuddy\\20260927120106\\air_ground\\tools\\sync_progress.py\""

为什么不让网盘直接同步项目目录？
    网盘会同时改写 .git/index、refs 等内部文件，两机并发时极易**静默损坏仓库**，
    而且冲突时只会生成一堆「副本(2)」文件，没有任何合并语义。
    Git 的冲突是显式的、可解决的，这是它值得多一层的原因。
"""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

OK = "[ OK ]"
WARN = "[WARN]"
FAIL = "[FAIL]"


def git(*args: str) -> tuple[int, str, str]:
    """执行 git 命令。返回 (returncode, stdout, stderr)。"""
    proc = subprocess.run(
        ["git", "-c", "core.quotepath=false", *args],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return proc.returncode, (proc.stdout or "").strip(), (proc.stderr or "").strip()


def git_ok(*args: str) -> bool:
    return git(*args)[0] == 0


def is_repo() -> bool:
    return git_ok("rev-parse", "--is-inside-work-tree")


def has_remote() -> bool:
    return bool(git("remote")[1])


def current_branch() -> str:
    code, out, _ = git("rev-parse", "--abbrev-ref", "HEAD")
    return out if code == 0 and out else "main"


def dirty_files() -> list[str]:
    code, out, _ = git("status", "--porcelain")
    if code != 0 or not out:
        return []
    return [ln for ln in out.splitlines() if ln.strip()]


def in_conflict() -> bool:
    """是否卡在未完成的 rebase / merge 里。"""
    gitdir = ROOT / ".git"
    return (gitdir / "rebase-merge").exists() or (gitdir / "rebase-apply").exists() or (
        gitdir / "MERGE_HEAD"
    ).exists()


def print_conflict_help() -> None:
    print()
    print(f"{FAIL} 有文件两边都改了，Git 无法自己决定留哪个 —— 需要人来判断。")
    print("     冲突文件：")
    code, out, _ = git("diff", "--name-only", "--diff-filter=U")
    for name in (out.splitlines() if code == 0 and out else []):
        print(f"       - {name}")
    print()
    print("     处理办法：")
    print("       1) 打开上述文件，把 <<<<<<< ======= >>>>>>> 之间的内容理清，只留正确的那份")
    print("       2) git add <文件>")
    print("       3) python tools/sync_progress.py        # 重新跑一遍即可继续")
    print()
    print("     想直接放弃本次拉取、回到动手前的状态：")
    print("       git rebase --abort")
    print("       git stash list                          # 你的改动可能被暂存在这里")
    print("       git stash pop")


def cmd_status() -> int:
    branch = current_branch()
    print(f"仓库   : {ROOT}")
    print(f"分支   : {branch}")

    changed = dirty_files()
    if changed:
        print(f"{WARN} 未提交改动 {len(changed)} 处：")
        for line in changed[:15]:
            print(f"       {line}")
        if len(changed) > 15:
            print(f"       ...（还有 {len(changed) - 15} 处）")
    else:
        print(f"{OK} 工作区干净")

    if not has_remote():
        print(f"{FAIL} 没有配置远端（git remote 是空的）—— 同步还差这一步")
        return 1

    remote_url = git("remote", "get-url", "origin")[1]
    print(f"远端   : {remote_url}")

    code, local, _ = git("rev-parse", "HEAD")
    if code != 0:
        print(f"{WARN} 还没有任何提交")
        return 0

    code, upstream, _ = git("rev-parse", f"origin/{branch}")
    if code != 0:
        print(f"{WARN} 远端还没有 {branch} 分支（首次推送前是正常的）")
        return 0

    ahead = git("rev-list", "--count", f"origin/{branch}..HEAD")[1]
    behind = git("rev-list", "--count", f"HEAD..origin/{branch}")[1]
    print(f"差异   : 本地领先 {ahead} 个提交 / 落后 {behind} 个提交")
    return 0


def cmd_pull(pull_only: bool, machine: str, autocommit: bool) -> int:
    if in_conflict():
        print(f"{FAIL} 上一次同步的冲突还没解决完，先按下面提示处理：")
        print_conflict_help()
        return 1

    branch = current_branch()
    had_changes = bool(dirty_files())

    # ---- 1. 拉取对方进展 ----
    # --rebase：把本地提交叠在对方提交之上，历史保持一条直线，不做多余的合并提交
    # --autostash：本地还有没提交的改动时先自动暂存，拉完再放回来（否则 Git 直接拒绝）
    print(f"→ 拉取 origin/{branch} …")
    code, out, err = git("pull", "--rebase", "--autostash", "origin", branch)
    if code != 0:
        if in_conflict():
            print_conflict_help()
        else:
            print(f"{FAIL} 拉取失败：")
            print(f"       {err or out}")
            print("       常见原因：远端地址不对 / 没配凭据 / 网络不通")
        return 1
    if "up to date" in out.lower() or "已经是最新" in out:
        print(f"{OK} 已是最新，对方没有新提交")
    else:
        print(f"{OK} 拉到对方的新提交")
        print("       （改动进的是你正在编辑的目录，编辑器里可能需要刷新）")

    if pull_only:
        return 0

    # ---- 2. 提交自己的改动 ----
    changed = dirty_files()
    if not changed:
        print(f"{OK} 本地没有新改动，无需提交")
        return 0

    if not autocommit:
        print(f"{WARN} 有 {len(changed)} 处改动但未提交（--no-commit 生效），本次不同步给对方")
        return 0

    print(f"→ 提交本地 {len(changed)} 处改动 …")
    code, out, err = git("add", "-A")
    if code != 0:
        print(f"{FAIL} git add 失败：{err or out}")
        return 1

    stamp = datetime.now().strftime("%Y-%m-%d %H:%M")
    message = f"sync({machine}): {stamp}"
    code, out, err = git("-c", "commit.gpgsign=false", "commit", "-m", message)
    if code != 0:
        print(f"{FAIL} 提交失败：{err or out}")
        return 1
    print(f"{OK} 已提交：{message}")

    # ---- 3. 推给对方 ----
    print(f"→ 推送到 origin/{branch} …")
    code, out, err = git("push", "origin", f"HEAD:{branch}")
    if code != 0:
        print(f"{FAIL} 推送失败：")
        print(f"       {err or out}")
        print("       若是被拒绝（non-fast-forward）：先再跑一次本脚本，它会先拉再推")
        return 1
    print(f"{OK} 已推送")

    if had_changes:
        print()
        print("提示：这次连你没写完的改动一起提交了。"
              "正编辑到一半不想同步，下次用 --no-commit。")
    return 0


def cmd_install_hook() -> int:
    """装 post-commit 钩子：每次 commit 自动 push，把同步做到「提交即推送」。"""
    hook = ROOT / ".git" / "hooks" / "post-commit"
    if not (ROOT / ".git").is_dir():
        print(f"{FAIL} 这里还不是 Git 仓库")
        return 1

    hook.write_text(
        "#!/bin/sh\n"
        "# 由 tools/sync_progress.py --install-hook 生成\n"
        "# 每次提交后自动推送；失败不阻塞提交（同步是锦上添花，不能挡住干活）\n"
        "git push origin HEAD 2>&1 | sed 's/^/[post-commit] /' || true\n",
        encoding="utf-8",
        newline="\n",
    )
    try:
        hook.chmod(0o755)
    except OSError:
        pass  # Windows 上 chmod 基本无效，但 Git for Windows 不依赖它
    print(f"{OK} 已安装 {hook}")
    print("     以后每次 git commit 都会自动推送")
    print("     删掉这个文件即可取消")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="双机进度同步：拉取对方进展 → 提交自己的改动 → 推给对方",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--status", action="store_true", help="只显示差异概况，不改动任何东西")
    ap.add_argument("--pull-only", action="store_true", help="只拉取对方进展，不提交自己的改动")
    ap.add_argument("--no-commit", action="store_true", help="拉取后不自动提交本地改动")
    ap.add_argument("--install-hook", action="store_true", help="安装 post-commit 钩子（提交即推送）")
    ap.add_argument("--machine", default="", help="本机代号（写进提交信息，默认取主机名）")
    args = ap.parse_args()

    if not is_repo():
        print(f"{FAIL} 当前不是 Git 仓库：{ROOT}")
        return 1

    machine = args.machine or socket.gethostname().split(".")[0]

    if args.install_hook:
        return cmd_install_hook()
    if args.status:
        return cmd_status()
    if not has_remote():
        print(f"{FAIL} 没有配置远端，无法同步。先执行：")
        print("       git remote add origin https://gitee.com/<你的用户名>/air_ground.git")
        return 1
    return cmd_pull(args.pull_only, machine, autocommit=not args.no_commit)


if __name__ == "__main__":
    sys.exit(main())
