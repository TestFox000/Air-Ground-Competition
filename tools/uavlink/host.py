# uavlink.host —— 由 tools/uav_link.py 拆出（--host 缓存与密码读取、SSH 预探测）。与原文件逐行一致（AST 校验过）。
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import socket
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path
from uavlink.config import HOST_CACHE

# ---------------------------------------------------------------- 主机解析 / 密码 / TCP 探测

def load_cached_host(path=HOST_CACHE) -> str | None:
    """从缓存文件读上次 --host（uav_link.local.env，形如 UAV_HOST=...）。"""
    try:
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line.startswith("UAV_HOST="):
                v = line.split("=", 1)[1].strip()
                return v or None
    except OSError:
        pass
    return None
def save_cached_host(host: str, path=HOST_CACHE) -> None:
    """记住本次 --host，下次免输（纯便利缓存，不是秘密）。"""
    try:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(f"UAV_HOST={host}\n", encoding="utf-8", newline="\n")
    except OSError:
        pass
def resolve_host(args) -> str | None:
    """--host 优先，其次上次缓存；都没有返回 None（调用方给清晰报错）。"""
    host = getattr(args, "host", None)
    if host:
        return host
    return load_cached_host(getattr(args, "host_cache", HOST_CACHE))
def read_password(args) -> str:
    """密码优先级：--password > AG_ORIN_PASS 环境变量 > 交互提问。

    ⚠️ 与 tools/remote_run.py 同一坑：没有交互终端时 getpass 会**静默死等**
    （看着像"卡死"），这里直接报错并说明该给什么。
    """
    pw = getattr(args, "password", None) or os.environ.get("AG_ORIN_PASS")
    if pw:
        return pw
    import getpass
    if not sys.stdin.isatty():
        raise SystemExit(
            "缺少机载端密码（当前没有交互终端，无法弹出密码提问）：\n"
            "  ① 设环境变量：set AG_ORIN_PASS=<密码>\n"
            "  ② 或命令行传入：--password <密码>"
        )
    return getpass.getpass(f"{args.orin_user}@{args.host} 的密码: ")
def probe_ssh(host: str, port: int = 22, timeout: float = 3.0) -> str | None:
    """连接前 TCP 探测：端口不通返回清晰报错文本，通则 None。

    省得在 paramiko 的长超时里干等，还把"无人机没开机/不在网段"误判成 SSH 故障。
    """
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return None
    except OSError as e:
        return (f"SSH 端口不可达 {host}:{port}（{type(e).__name__}: {e}）——"
                f"查无人机是否开机、笔记本是否同网段、IP 是否变了")
