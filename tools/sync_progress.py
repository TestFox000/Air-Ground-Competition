#!/usr/bin/env python3
"""双机协作同步：一条命令走完「拉对方进展 → 提交自己的改动 → 推给对方」。

两份机器同时改一个项目，最容易出事的不是冲突本身，而是**冲突被自动"猜"掉**。
所以这个脚本给自己定了三条死规矩：

    1. 绝不用 `reset --hard` / `checkout -f` / `push --force` —— 任何一条都会吃掉对方的提交。
    2. 冲突时不猜、不自动 merge：把冲突文件列清楚，**回退到安全状态**，让人来决定。
    3. 远端不可达 / 没凭据 / 仓库损坏时，只报清楚原因，**不动本地任何东西**。

用法（两条机器各配一次，之后日常就敲第一条）：

    python tools/sync_progress.py                  # 拉 + 提交 + 推（日常就这一条）
    python tools/sync_progress.py --status         # 只看两边差多少，不碰任何东西
    python tools/sync_progress.py --pull-only      # 只拉对方进展，含自己未提交改动时不动
    python tools/sync_progress.py --no-commit      # 拉完不自动提交（改到一半时用）
    python tools/sync_progress.py --push-only      # 只推送（给 post-commit 钩子用）
    python tools/sync_progress.py --install-hook   # 装「提交即推送」钩子
    python tools/sync_progress.py --uninstall-hook # 卸掉它
    python tools/sync_progress.py --machine B      # 提交信息里标明是谁推的

    --log PATH        把输出追加到日志（定时任务用）
    --remote NAME     远端名，默认 origin
    --branch NAME     分支名，默认当前分支

退出码：0 正常 ｜ 2 有冲突 ｜ 3 仓库/远端没配好 ｜ 4 其它错误 ｜ 5 有本地改动但被 --no-commit 留住
"""

from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[1]

# Windows 控制台默认 GBK：远端/日志里有 `⚠` 之类字符时，print 会直接抛
# UnicodeEncodeError 把整条命令搞崩（车端脚本实测踩过）。这里一律容错。
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(errors="replace")
    except Exception:  # noqa: BLE001
        pass


# ----------------------------------------------------------------- 基础工具
class Git:
    """薄薄一层 git 调用。所有可能失败的地方都返回 (code, out)，不抛异常。"""

    def __init__(self, cwd: Path) -> None:
        self.cwd = cwd

    def __call__(self, *args: str, check: bool = False) -> Tuple[int, str]:
        cmd = ["git", *args]
        # GIT_TERMINAL_PROMPT=0：凭据没配好时**立刻报错**，而不是弹一个没人能回答的
        # 用户名/密码提示把进程挂住（post-commit 钩子和定时任务里尤其致命 ——
        # 用户只是提交个代码，却被 push 的密码提示卡在那儿）。
        env = {**os.environ, "GIT_TERMINAL_PROMPT": "0"}
        try:
            p = subprocess.run(cmd, cwd=str(self.cwd), capture_output=True, env=env,
                               encoding="utf-8", errors="replace")
        except FileNotFoundError:
            return 127, "找不到 git 命令（装一个：winget install Git.Git）"
        out = (p.stdout or "") + (("\n" + p.stderr) if p.stderr and p.returncode else "")
        if check and p.returncode != 0:
            raise RuntimeError(f"git {' '.join(args)} 失败：{out.strip()}")
        return p.returncode, out.strip()

    def ok(self, *args: str) -> bool:
        return self("rev-parse", *args)[0] == 0 if args else False

    def lines(self, *args: str) -> List[str]:
        code, out = self(*args)
        return out.splitlines() if code == 0 and out else []


def _say(msg: str = "") -> None:
    print(msg, flush=True)


class Logger:
    """同时打到屏幕和（可选的）日志文件，给定时任务留痕。"""

    def __init__(self, path: Optional[str]) -> None:
        self.fh = None
        if path:
            try:
                p = Path(path)
                p.parent.mkdir(parents=True, exist_ok=True)
                self.fh = p.open("a", encoding="utf-8", errors="replace")
            except OSError:
                self.fh = None          # 日志写不进去也绝不能影响同步

    def __call__(self, msg: str = "") -> None:
        _say(msg)
        if self.fh:
            try:
                self.fh.write(msg + "\n")
                self.fh.flush()
            except OSError:
                self.fh = None

    def close(self) -> None:
        if self.fh:
            try:
                self.fh.close()
            except OSError:
                pass


# ----------------------------------------------------------------- 状态检查
def repo_problems(g: Git) -> List[str]:
    """返回仓库层面的硬问题（空列表 = 能用）。"""
    problems = []
    if not (ROOT / ".git").exists():
        problems.append("这里不是 git 仓库（缺 .git）—— 先按 协作看板.md §10 接入")
        return problems
    if g("rev-parse", "--verify", "HEAD")[0] != 0:
        problems.append("仓库里还没有任何提交（HEAD 无效）")
    # 只查连通性，不扫全库（大仓库上 `fsck` 很慢，这里要的是"能不能干活"）
    code, out = g("fsck", "--connectivity-only", "--no-progress")
    if code != 0:
        first = next((ln for ln in out.splitlines() if ln.strip()), "")
        problems.append(f"仓库对象有问题（git fsck 报错）：{first}")
    return problems


def remote_url(g: Git, remote: str) -> str:
    return g("remote", "get-url", remote)[1].strip() if g("remote", "get-url", remote)[0] == 0 else ""


def upstream(g: Git, branch: str, remote: str) -> str:
    """返回上游引用名（`origin/main`），没有配则返回空串。"""
    code, out = g("rev-parse", "--abbrev-ref", f"{branch}@{{upstream}}")
    return out if code == 0 and "@{upstream}" not in out else f"{remote}/{branch}"


def ahead_behind(g: Git, up: str) -> Tuple[Optional[int], Optional[int]]:
    code, out = g("rev-list", "--left-right", "--count", f"{up}...HEAD")
    if code != 0 or not out:
        return None, None
    parts = out.split()
    if len(parts) != 2:
        return None, None
    behind, ahead = int(parts[0]), int(parts[1])       # left=远端独有(落后) right=本地独有(领先)
    return ahead, behind


def dirty_files(g: Git) -> List[str]:
    return [ln[3:].strip() for ln in g.lines("status", "--porcelain") if ln.strip()]


# ----------------------------------------------------------------- 各个动作
def do_status(g: Git, remote: str, branch: str, log: Logger) -> int:
    """只读：两边差多少、有哪些本地改动。"""
    log(f"仓库     : {ROOT}")
    log(f"分支     : {branch}")
    url = remote_url(g, remote)
    log(f"远端     : {remote} -> {url or '（没配！见 协作看板.md §9）'}")

    dirty = dirty_files(g)
    log(f"本地改动 : {len(dirty)} 个文件" + ("" if not dirty else "（未提交）"))
    for f in dirty[:10]:
        log(f"           {f}")
    if len(dirty) > 10:
        log(f"           …还有 {len(dirty) - 10} 个")

    if not url:
        log("\n结论：远端没配 —— 现在只在本机提交，两台机器还谈不上协作。")
        return 3

    code, out = g("fetch", "--prune", remote)
    if code != 0:
        log(f"\n[!] 取远端失败（网络/凭据）：{out.splitlines()[-1] if out else ''}")
        return 3

    up = upstream(g, branch, remote)
    ahead, behind = ahead_behind(g, up)
    if ahead is None:
        log(f"\n远端还没有 {up}（第一次推送时会建）")
    elif ahead == 0 and behind == 0:
        log(f"\n结论：和 {up} 完全一致，没事可做。")
    else:
        log(f"\n结论：比 {up} 领先 {ahead} 个提交、落后 {behind} 个提交")
        for ln in g.lines("log", "--oneline", f"{up}..HEAD")[:5]:
            log(f"  我这边独有: {ln}")
        for ln in g.lines("log", "--oneline", f"HEAD..{up}")[:5]:
            log(f"  对方独有  : {ln}")
        if behind:
            log("  -> 跑 `python tools/sync_progress.py` 把对方的合过来")
    return 0


def commit_local(g: Git, machine: str, log: Logger) -> int:
    """把当前改动提交掉。返回 0=已提交 1=没有改动 4=失败。"""
    dirty = dirty_files(g)
    if not dirty:
        return 1
    log(f"[2/4] 提交本地改动（{len(dirty)} 个文件）")
    if g("add", "-A")[0] != 0:
        log("      [!] git add 失败")
        return 4
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M")
    msg = f"chore({machine}): 自动同步 {stamp}（{len(dirty)} 个文件）"
    code, out = g("commit", "-q", "-m", msg)
    if code != 0:
        log(f"      [!] 提交失败：{out.splitlines()[-1] if out else ''}")
        return 4
    log(f"      已提交：{msg}")
    return 0


def rebase_onto_upstream(g: Git, up: str, log: Logger) -> int:
    """把本地提交叠到远端之上。返回 0=好 2=冲突 3=没上游 4=其它失败。"""
    if g("rev-parse", "--verify", up)[0] != 0:
        log(f"[3/4] 远端还没有 {up}，跳过合并（第一次推送会建它）")
        return 3
    ahead, behind = ahead_behind(g, up)
    if behind == 0:
        log(f"[3/4] 不需要合并（对方没有新提交）")
        return 0

    log(f"[3/4] 合并对方的 {behind} 个提交（rebase，不改写对方历史）")
    code, out = g("rebase", up)
    if code == 0:
        log("      合并完成")
        return 0

    # 冲突：绝不自动猜。列清楚，然后回退到安全状态（我们的提交原封不动还在）。
    files = [ln.split(":", 1)[-1].strip() for ln in g.lines("diff", "--name-only", "--diff-filter=U")]
    if not files:
        files = [ln.strip() for ln in out.splitlines() if ln.startswith("CONFLICT")][:20]
    log("")
    log("      ============ 有冲突，已停下，没有替你决定任何事 ============")
    for f in files or ["(没能列出文件名，见下面的 git 输出)"]:
        log(f"      冲突文件: {f}")
    g("rebase", "--abort")            # 回退到"我方提交完好、对方未合并"的状态
    log("      已执行 git rebase --abort：你的提交完好保留，对方的提交也没被动过。")
    log("      两种处理方式（选一种）：")
    log("        a) 找对方一起看：把冲突文件里 `<<<<<<<` / `=======` / `>>>>>>>` 三段商量着改完，")
    log("           再跑一次 `python tools/sync_progress.py`；")
    log("        b) 你自己直接改：git pull --rebase origin <分支>，按提示逐个解决后 git rebase --continue。")
    if out:
        tail = "\n".join("        " + ln for ln in out.splitlines()[-6:])
        log("      --- git 原始输出（尾部）---")
        log(tail)
    return 2


def push(g: Git, remote: str, branch: str, log: Logger, retry: bool = True) -> int:
    """推送。被拒（对方有新提交）时拉一次再试一次，绝不 force。"""
    log(f"[4/4] 推送到 {remote}/{branch}")
    code, out = g("push", remote, branch)
    if code == 0:
        log("      推送完成")
        return 0

    low = out.lower()
    if "no upstream" in low or "set-upstream" in low or "has no upstream" in low:
        code, out = g("push", "-u", remote, branch)
        if code == 0:
            log("      推送完成（已绑定上游）")
            return 0

    if retry and ("rejected" in low or "non-fast-forward" in low or "fetch first" in low):
        log("      远端有新提交，先合并再试一次（不会 force 覆盖）")
        rc = rebase_onto_upstream(g, upstream(g, branch, remote), log)
        if rc == 2:
            return 2
        if rc in (0, 3):
            code, out = g("push", remote, branch)
            if code == 0:
                log("      推送完成")
                return 0

    last = next((ln for ln in reversed(out.splitlines()) if ln.strip()), "")
    log(f"      [!] 推送失败：{last}")
    if any(k in low for k in ("authentication", "permission", "403", "401", "invalid username", "could not read username")):
        log("      看起来是凭据问题 —— 按 协作看板.md §9.5 配好 store 凭据（令牌）")
    return 4


def do_sync(g: Git, remote: str, branch: str, machine: str, log: Logger,
            no_commit: bool = False, pull_only: bool = False,
            push_only: bool = False) -> int:
    url = remote_url(g, remote)
    if not url:
        log("[!] 没配远端。按 协作看板.md §9 配一次，之后就不用管了：")
        log(f"    git remote add {remote} <仓库地址>")
        return 3

    log(f"--- 同步开始 {datetime.now():%F %T}（{machine} 机，分支 {branch}）---")

    log("[1/4] 取对方进展")
    code, out = g("fetch", "--prune", remote)
    if code != 0:
        log(f"      [!] 取不到（网络或凭据问题）：{out.splitlines()[-1] if out else ''}")
        log("      本地一个字节都没动。")
        return 3
    up = upstream(g, branch, remote)

    if push_only:
        log("[2-3/4] --push-only：跳过提交与合并")
        return push(g, remote, branch, log)

    if not no_commit:
        rc = commit_local(g, machine, log)
        if rc == 1:
            log("[2/4] 本地没有改动，跳过提交")
        elif rc == 4:
            return 4
    else:
        log("[2/4] --no-commit：本地改动原样留着")

    if pull_only and dirty_files(g):
        log("[3/4] --pull-only 且工作区还有未提交改动 —— 不合并，避免动到你的东西")
        log(f"      对方的 {up} 已经取到本地，处理完自己的改动后再跑一次即可")
        return 5

    rc = rebase_onto_upstream(g, up, log)
    if rc == 2:
        return 2

    if pull_only:
        log("[4/4] --pull-only：不推送")
        return 0

    push_rc = push(g, remote, branch, log)
    log(f"--- 同步结束（退出码 {push_rc}）---")
    return push_rc


# ----------------------------------------------------------------- 钩子
HOOK_BODY = """#!/bin/sh
# 由 tools/sync_progress.py --install-hook 生成。
# 提交后自动推给对方；推不上去（断网/没凭据）也**绝不让提交失败**。
# GIT_TERMINAL_PROMPT=0 是必须的：否则凭据没配好时 git 会弹密码提示，
# 把用户的 `git commit` 直接卡在那里。
GIT_TERMINAL_PROMPT=0 python "{script}" --push-only --machine {machine} --log "{logfile}" >/dev/null 2>&1 || true
exit 0
"""


def install_hook(g: Git, machine: str, logfile: str, log: Logger, remove: bool = False) -> int:
    hooks = ROOT / ".git" / "hooks"
    if not hooks.is_dir():
        log("[!] 找不到 .git/hooks —— 这里不是仓库？")
        return 3
    path = hooks / "post-commit"
    if remove:
        if path.exists():
            path.unlink()
            log(f"已卸载钩子：{path}")
        else:
            log("本来就没装钩子")
        return 0
    script = Path(__file__).resolve()
    path.write_text(HOOK_BODY.format(script=script.as_posix(), machine=machine, logfile=logfile),
                    encoding="utf-8", newline="\n")     # 必须 LF：Git Bash 才能执行
    try:
        path.chmod(0o755)
    except OSError:
        pass                       # Windows 上无所谓，git 自己会调 sh
    log(f"已安装「提交即推送」钩子：{path}")
    log(f"  以后每次 git commit 都会自动 push 给对方（失败也不影响提交）。")
    log(f"  不想要了：python tools/sync_progress.py --uninstall-hook")
    return 0


# ----------------------------------------------------------------- 入口
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="双机协作同步：拉对方进展 → 提交自己的改动 → 推给对方",
        epilog="冲突时它会停下并把冲突文件列清楚，绝不自动猜、绝不覆盖对方的工作。")
    ap.add_argument("--status", action="store_true", help="只看两边差多少，不碰任何东西")
    ap.add_argument("--pull-only", action="store_true", help="只拉对方进展，不推送")
    ap.add_argument("--push-only", action="store_true", help="只推送（钩子用）")
    ap.add_argument("--no-commit", action="store_true", help="拉取后不自动提交本地改动")
    ap.add_argument("--install-hook", action="store_true", help="装「提交即推送」钩子")
    ap.add_argument("--uninstall-hook", action="store_true", help="卸掉钩子")
    ap.add_argument("--machine", default=os.environ.get("AG_MACHINE") or socket.gethostname()[:12],
                    help="机器标识（写进提交信息，默认取主机名；建议 A / B）")
    ap.add_argument("--remote", default="origin")
    ap.add_argument("--branch", default=None, help="默认当前分支")
    ap.add_argument("--log", default=None, help="把输出追加到这个文件")
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    log = Logger(args.log)
    try:
        g = Git(ROOT)

        problems = repo_problems(g)
        if problems:
            log("[!] 仓库不能用，先解决这个：")
            for p in problems:
                log(f"    - {p}")
            log("\n    （如果 .git 被网盘同步搞坏了：把工作树复制到安全处，"
                "然后照 协作看板.md §10 重新 clone 一份）")
            return 3

        branch = args.branch or (g("rev-parse", "--abbrev-ref", "HEAD")[1] or "main")

        if args.install_hook or args.uninstall_hook:
            logfile = args.log or str(Path.home() / "air_ground_sync.log")
            return install_hook(g, args.machine, logfile, log, remove=args.uninstall_hook)

        if args.status:
            return do_status(g, args.remote, branch, log)

        return do_sync(g, args.remote, branch, args.machine, log,
                       no_commit=args.no_commit, pull_only=args.pull_only,
                       push_only=args.push_only)
    finally:
        log.close()


if __name__ == "__main__":
    raise SystemExit(main())
