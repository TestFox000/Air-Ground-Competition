#!/usr/bin/env python3
"""无人机向导菜单：无人机向导.bat 的后端（bat 只是哑启动器）。

为什么菜单在 Python 而不在 bat：cmd 解析 UTF-8 中文批处理有分块解码缺陷
（2026-10-21 实锤：菜单行被啃成 'xxx' is not recognized），LF 行尾还会让
goto 崩掉。中文全部收进本文件后，bat 只剩三行纯 ASCII，彻底免疫。

职责（对齐旧 bat 的能力）：
  * 依赖自检：paramiko 缺失只警告（彩排模式不需要它）；
  * Orin IP/用户名记忆：tools/uav_wizard.local.env（只存主机名，绝不存密码）；
  * 密码：getpass 不回显，仅进程内传给向导（AG_ORIN_PASS 环境变量，不落盘）；
  * 菜单分发：面板/全量自检/彩排/改 IP/开手册——进程内直调 uav_wizard.main，
    不经 shell、不起子进程（无注入面，还省一次解释器启动）。

菜单与面板键位说明见 docs/无人机操作手册.md §0.1。
"""
from __future__ import annotations

import os
import re
import socket
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
REPO = TOOLS.parent
ENV_FILE = TOOLS / "uav_wizard.local.env"
MANUAL = REPO / "docs" / "无人机操作手册.md"

# 主机名/用户名白名单：来自用户输入，会进向导参数与 env 文件，收窄到
# 无元字符字符集（IPv4/v6、DNS 名、常见 Linux 用户名都在内）。
_HOST_RE = re.compile(r"^[A-Za-z0-9._:-]{1,100}$")
_USER_RE = re.compile(r"^[A-Za-z0-9._-]{1,32}$")


def valid_host(host: str) -> bool:
    return bool(_HOST_RE.match(host or ""))


def valid_user(user: str) -> bool:
    return bool(_USER_RE.match(user or ""))


def run_wizard(args: list[str], password: str | None = None) -> int:
    """进程内直调 uav_wizard.main(argv)（默认实现；单测注入替身）。

    向导 argparse 的 .error()/缺密码守卫走 SystemExit，在这里收敛成退出码，
    不打断菜单循环。
    """
    prev_pw = os.environ.get("AG_ORIN_PASS")   # 用户可能在 shell 里设过——退出时还原
    if password:
        os.environ["AG_ORIN_PASS"] = password
    if str(TOOLS) not in sys.path:
        sys.path.insert(0, str(TOOLS))
    import uav_wizard
    try:
        return uav_wizard.main(args) or 0
    except SystemExit as e:
        return e.code if isinstance(e.code, int) else 2
    finally:
        if prev_pw is not None:
            os.environ["AG_ORIN_PASS"] = prev_pw
        else:
            os.environ.pop("AG_ORIN_PASS", None)


def load_env() -> dict[str, str]:
    """读 tools/uav_wizard.local.env 的 KEY=VALUE（不存在返回空）。"""
    cfg: dict[str, str] = {}
    if ENV_FILE.exists():
        for line in ENV_FILE.read_text(encoding="utf-8").splitlines():
            key, sep, value = line.partition("=")
            if sep and key.strip():
                cfg[key.strip()] = value.strip()
    return cfg


def save_env(host: str, user: str):
    ENV_FILE.write_text(f"ORIN_HOST={host}\nORIN_USER={user}\n", encoding="utf-8")


def probe_tcp(host: str, port: int = 22, timeout: float = 2.0) -> bool:
    """SSH 端口探活（比 ICMP 可靠，ICMP 常被禁）。"""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def ask_password() -> str | None:
    """不回显问密码；EOF/直接回车返回 None（交给向导自己的守卫报错）。"""
    import getpass
    try:
        return getpass.getpass("Orin SSH 密码（输入不回显）: ") or None
    except (EOFError, KeyboardInterrupt):
        return None


def open_manual():
    if not MANUAL.exists():
        print(f"[提示] 手册不存在: {MANUAL}")
        return
    try:
        os.startfile(str(MANUAL))   # Windows
    except AttributeError:          # pragma: no cover - 非 Windows 兜底
        import webbrowser
        webbrowser.open(MANUAL.as_uri())


def print_menu(cfg: dict[str, str]):
    host = cfg.get("ORIN_HOST", "未设置")
    print(
        f"\n==================================================\n"
        f"  无人机操作向导  （手册: docs/无人机操作手册.md）\n"
        f"==================================================\n"
        f"  Orin: {host}\n"
        f"--------------------------------------------------\n"
        f"  [1] 向导面板（真机，推荐赛前走一遍）\n"
        f"  [2] 全量自检 --check（非交互，打印报告）\n"
        f"  [3] 彩排模式 --rehearse（无硬件训练/演示）\n"
        f"  [4] 修改 Orin IP / 用户名\n"
        f"  [5] 打开操作手册\n"
        f"  [0] 退出\n"
        f"--------------------------------------------------",
        flush=True)


def _ask_host(pick) -> str:
    """向用户要一个合法主机名；非法/留空返回空串（调用方回菜单）。"""
    raw = (pick("输入 Orin IP（留空返回菜单）: ") or "").strip()
    if not raw:
        return ""
    if not valid_host(raw):
        print(f"[提示] 主机名含非法字符（只允许字母数字 . _ : -）: {raw!r}")
        return ""
    return raw


def menu_loop(cfg: dict[str, str], pick=input, wizard=run_wizard,
              prober=probe_tcp, opener=open_manual, askpw=ask_password) -> int:
    """菜单主循环。pick/wizard/prober/opener/askpw 可注入，单测零交互零进程。

    返回退出码：0 正常；EOF/Ctrl+C 也按正常退出（双击窗口不闪退）。
    """
    while True:
        print_menu(cfg)
        try:
            raw = pick("选择 [1]: ")
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        choice = (raw or "").strip() or "1"
        if choice == "0":
            return 0
        try:
            _Dispatch(cfg, pick, wizard, prober, opener, askpw)(choice)
        except (EOFError, KeyboardInterrupt):
            # EOF 也可能出现在"回车返回菜单"/_ask_host/选项4输入等任意 pick：
            # 全部按优雅退出处理，不能裸抛 traceback（双击窗口不闪退契约）。
            print()
            return 0


class _Dispatch:
    """单个菜单选项的处理（从 menu_loop 抽出，使 EOF 保护覆盖所有 pick 点）。"""

    def __init__(self, cfg, pick, wizard, prober, opener, askpw):
        self.cfg, self.pick, self.wizard = cfg, pick, wizard
        self.prober, self.opener, self.askpw = prober, opener, askpw

    def __call__(self, choice: str):
        if choice in ("1", "2"):
            cfg = self.cfg
            host = cfg.get("ORIN_HOST", "")
            if not valid_host(host):
                host = _ask_host(self.pick)
                if not host:
                    return None
                cfg["ORIN_HOST"] = host
                save_env(host, cfg.get("ORIN_USER", "duzhong"))
            user = cfg.get("ORIN_USER", "duzhong")
            if not valid_user(user):
                print(f"[提示] 已存用户名非法: {user!r}，请用选项 [4] 重设")
                return None
            if self.prober(host):
                print(f"[通过] Orin {host}:22 可达")
            else:
                print(f"[提示] {host}:22 连不上（可能未上电/不在同网段），仍继续尝试 SSH")
            args = ["--host", host, "--user", user]
            if choice == "2":
                args.append("--check")
            rc = self.wizard(args, self.askpw())
            if rc != 0:
                print(f"[提示] 向导退出码 {rc}")
            self.pick("回车返回菜单...")
        elif choice == "3":
            self.wizard(["--rehearse"])
            self.pick("回车返回菜单...")
        elif choice == "4":
            host = (self.pick("输入 Orin IP: ") or "").strip()
            user = (self.pick("输入 SSH 用户名（回车默认 duzhong）: ") or "").strip() or "duzhong"
            if not host or not valid_host(host):
                print("[提示] 主机名非法或为空，未保存")
            elif not valid_user(user):
                print(f"[提示] 用户名非法（只允许字母数字 . _ -）: {user!r}，未保存")
            else:
                self.cfg["ORIN_HOST"], self.cfg["ORIN_USER"] = host, user
                save_env(host, user)
                print(f"[已保存] {host} / {user}（密码永不保存）")
        elif choice == "5":
            self.opener()
        else:
            print("[提示] 无效选项")
        return None


def main() -> int:
    # Windows conhost 激活 VT 转义（面板要用）+ 管道/重定向下统一 UTF-8
    os.system("")
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except Exception:  # noqa: BLE001
            pass
    try:
        import paramiko  # noqa: F401
    except ImportError:
        print("[提示] 缺 paramiko：真机模式不可用（彩排不受影响）\n"
              "       安装：python -m pip install paramiko\n")
    return menu_loop(load_env())


if __name__ == "__main__":
    sys.exit(main())
