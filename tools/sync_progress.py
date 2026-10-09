"""air_ground 仓库同步脚本（轻量化版）。

用法：
    python tools/sync_progress.py --pull-only      # 只拉取，不动工作区
    python tools/sync_progress.py --status         # 查看本地与远端差多少
    python tools/sync_progress.py --install-hook   # 安装 post-commit 钩子（提交即推送）

设计原则：
- --pull-only：工作区有未提交改动时拒绝合并，绝不乱动用户的东西
- 钩子：提交后自动 push，推送失败不影响提交（|| true）
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

_NO_WINDOW = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0


def _run(cmd: list[str], check: bool = False) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd,
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        creationflags=_NO_WINDOW,
        check=check,
    )


def _is_dirty() -> bool:
    r = _run(["git", "status", "--porcelain"])
    return bool(r.stdout.strip())


def cmd_status(_args) -> int:
    """显示本地与远端的差异。"""
    fetch = _run(["git", "fetch", "origin", "main"])
    if fetch.returncode != 0:
        print(f"[ERROR] fetch 失败：{fetch.stderr.strip()}")
        return 1

    local = _run(["git", "rev-parse", "HEAD"]).stdout.strip()
    remote = _run(["git", "rev-parse", "origin/main"]).stdout.strip()

    if local == remote:
        print("[OK] 本地与远端一致")
        return 0

    ahead = _run(["git", "rev-list", "--count", f"origin/main..{local}"]).stdout.strip()
    behind = _run(["git", "rev-list", "--count", f"{local}..origin/main"]).stdout.strip()
    print(f"[INFO] 本地领先 {ahead} 个提交，落后 {behind} 个提交")
    return 0


def cmd_pull_only(_args) -> int:
    """只拉取，工作区脏时拒绝合并。"""
    if _is_dirty():
        print("[SKIP] 工作区有未提交改动，拒绝合并（--pull-only）")
        return 5

    fetch = _run(["git", "fetch", "origin", "main"])
    if fetch.returncode != 0:
        print(f"[ERROR] fetch 失败：{fetch.stderr.strip()}")
        return 1

    local = _run(["git", "rev-parse", "HEAD"]).stdout.strip()
    remote = _run(["git", "rev-parse", "origin/main"]).stdout.strip()

    if local == remote:
        print("[OK] 已是最新，无需合并")
        return 0

    merge = _run(["git", "merge", "--ff-only", "origin/main"])
    if merge.returncode != 0:
        print(f"[ERROR] 合并失败：{merge.stderr.strip()}")
        print("[ACTION] 请手动解决冲突")
        return 1

    print("[OK] 已拉取并合并远端更新")
    return 0


def cmd_install_hook(args) -> int:
    """安装 post-commit 钩子（提交即推送）。"""
    machine = args.machine or "A"
    hook_path = REPO_ROOT / ".git" / "hooks" / "post-commit"

    if os.name == "nt":
        hook_content = f"""#!/bin/sh
# 提交即推送到 Gitee
git push origin main || true
"""
    else:
        hook_content = f"""#!/bin/sh
# 提交即推送到 Gitee
git push origin main || true
"""

    hook_path.write_text(hook_content, encoding="utf-8", newline="\n")
    os.chmod(hook_path, 0o755)
    print(f"[OK] post-commit 钩子已安装（machine={machine}）")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="air_ground 仓库同步")
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("status", help="查看本地与远端差异")
    pull = sub.add_parser("pull-only", help="只拉取，不动工作区")
    pull.set_defaults(func=cmd_pull_only)

    hook = sub.add_parser("install-hook", help="安装 post-commit 钩子")
    hook.add_argument("--machine", default="A", help="机器代号（A/B）")
    hook.set_defaults(func=cmd_install_hook)

    status = sub.add_parser("status", help="查看状态")
    status.set_defaults(func=cmd_status)

    # 兼容旧参数风格
    parser.add_argument("--pull-only", action="store_true", help="只拉取")
    parser.add_argument("--status", action="store_true", help="查看状态")
    parser.add_argument("--install-hook", action="store_true", help="安装钩子")
    parser.add_argument("--machine", default=None, help="机器代号")

    args = parser.parse_args()

    if args.pull_only:
        return cmd_pull_only(args)
    if args.status:
        return cmd_status(args)
    if args.install_hook:
        return cmd_install_hook(args)
    if args.command:
        return args.func(args)

    parser.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
