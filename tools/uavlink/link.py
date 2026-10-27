# uavlink.link —— 由 tools/uav_link.py 拆出（Orin SSH 链路与远程执行/SFTP）。与原文件逐行一致（AST 校验过）。
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
from uavlink.config import HOST_CACHE, RATE_CAPS_HZ, SAMPLER_REMOTE
from uavlink.host import probe_ssh, read_password, resolve_host, save_cached_host
from uavlink.sampler_src import SAMPLER_SRC, sampler_launch_cmd
from uavlink.telemetry import TelemetryState, parse_sampler_line

# ---------------------------------------------------------------- SSH 链路（笔记本端）

class OrinLink(threading.Thread):
    """SSH 遥测链路：上传采样器 → setsid 流式跑 → NDJSON 入队。

    远程用 setsid 自成进程组，stop() 时 pkill 整组回收（docs 里有孤儿进程教训）。
    启动命令见 sampler_launch_cmd()：dzros.env（域配置）与 setup.bash 都必须
    显式 source，否则跨不到机载 ROS 域。
    """

    def __init__(self, host: str, user: str, password: str, port: int = 22,
                 sampler_args: str = "") -> None:
        super().__init__(daemon=True, name="uav-link-orin")
        self.host, self.user, self.password, self.port = host, user, password, port
        self.sampler_args = sampler_args
        self.queue: queue.Queue = queue.Queue()
        self.stop_evt = threading.Event()
        self.error: str | None = None
        self._cli = None

    def run(self) -> None:  # pragma: no cover - 需要真实无人机，本地不测
        try:
            import paramiko
        except ImportError:
            self.error = "缺少 paramiko，请先执行: python -m pip install paramiko"
            return
        try:
            cli = paramiko.SSHClient()
            cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            cli.connect(hostname=self.host, port=self.port, username=self.user,
                        password=self.password, timeout=30, banner_timeout=30,
                        auth_timeout=30)
            self._cli = cli
            sftp = cli.open_sftp()
            with sftp.open(SAMPLER_REMOTE, "w") as f:
                f.write(SAMPLER_SRC)
            sftp.close()
            _, stdout, stderr = cli.exec_command(
                sampler_launch_cmd(self.sampler_args), get_pty=False)
            errbuf: list[str] = []

            def _drain_stderr() -> None:
                # 持续消费 stderr（rclpy 告警走这边）：SSH 通道窗口被灌满会
                # 堵死远程进程；只留尾部几段供报错用
                try:
                    while True:
                        chunk = stderr.read(4096)
                        if not chunk:
                            break
                        errbuf.append(chunk.decode("utf-8", "replace"))
                        del errbuf[:-8]
                except Exception:  # noqa: BLE001
                    pass

            drain = threading.Thread(target=_drain_stderr, daemon=True,
                                     name="uav-link-stderr")
            drain.start()
            for line in iter(stdout.readline, ""):
                if self.stop_evt.is_set():
                    break
                obj = parse_sampler_line(line)
                if obj is not None:
                    self.queue.put((time.time(), obj))
            if not self.stop_evt.is_set():
                code = stdout.channel.recv_exit_status()
                drain.join(timeout=2.0)
                err = "".join(errbuf)[-500:]
                self.error = f"采样器退出（码 {code}）: {err.strip()}"
        except Exception as e:  # noqa: BLE001 - 网络层异常原样上报
            if not self.stop_evt.is_set():
                self.error = f"{type(e).__name__}: {e}"

    def stop(self) -> None:  # pragma: no cover - 需要真实无人机，本地不测
        self.stop_evt.set()
        try:
            if self._cli is not None:
                # setsid 让采样器自成进程组，pkill 整组回收，不留孤儿。
                # 训练线教训：pkill -f 的字面串会匹配到包装 shell 自己的命令行，
                # 用 [.] 正则技巧避开自匹配
                self._cli.exec_command("pkill -f 'uav_sampler[.]py'", timeout=5)
        except Exception:  # noqa: BLE001
            pass
        try:
            if self._cli is not None:
                self._cli.close()
        except Exception:  # noqa: BLE001
            pass
def _connect(host: str, user: str, password: str, port: int = 22, timeout: float = 30.0):
    """建立一次性 SSH 连接（ctl/logs 用；run 走 OrinLink 线程）。"""
    import paramiko
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(hostname=host, port=port, username=user, password=password,
                timeout=timeout, banner_timeout=timeout, auth_timeout=timeout)
    return cli
def _upload_sampler(cli) -> None:  # pragma: no cover - 需要真实无人机
    sftp = cli.open_sftp()
    try:
        with sftp.open(SAMPLER_REMOTE, "w") as f:
            f.write(SAMPLER_SRC)
    finally:
        sftp.close()
def _exec_collect(cli, cmd: str, timeout: float = 60.0):  # pragma: no cover
    """执行远程命令收全量输出，返回 (退出码, stdout, stderr)。"""
    _, stdout, stderr = cli.exec_command(cmd, timeout=timeout, get_pty=False)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return stdout.channel.recv_exit_status(), out, err
def _link_preflight(args) -> tuple[str, str] | tuple[None, None]:
    """ctl/logs 共用的前置：解析主机 → TCP 探测 → 密码。失败打印并返回 (None, None)。"""
    host = resolve_host(args)
    if not host:
        print("[失败] 未给 --host 且无上次缓存（先跑一次 --host <IP> 成功后会记住）",
              file=sys.stderr)
        return None, None
    bad = probe_ssh(host, getattr(args, "port", 22))
    if bad:
        print(f"[失败] {bad}", file=sys.stderr)
        return None, None
    pw = read_password(args)
    save_cached_host(host, getattr(args, "host_cache", HOST_CACHE))
    return host, pw
# ---------------------------------------------------------------- run 主模式（只读遥测 + 黑匣子）

def _summary_lines(state: TelemetryState, counts: Counter, span_s: float,
                   n_rows: int) -> list[str]:
    """收尾统计：消息计数/丢包估计/各告警命中数。"""
    lines = ["==== 收尾统计 ===="]
    lines.append(f"时长 {span_s:.1f}s，黑匣子 {n_rows} 行")
    per = ", ".join(f"{k}x{v}" for k, v in sorted(state.counts.items())) or "无"
    lines.append(f"采样消息: {per}")
    # 丢包估计：按 attitude 声明 10Hz 名义对比实收
    att = state.counts.get("attitude", 0)
    if att and span_s > 1.0:
        expect = span_s * RATE_CAPS_HZ["attitude"]
        lines.append(f"丢包估计: attitude 实收 {att} / 名义 {expect:.0f}"
                     f"（≈{max(0.0, 1 - att / expect) * 100:.0f}% 缺口，链路/飞控双端都可能）")
    lines.append("告警命中: " + (", ".join(f"{k}x{v}" for k, v in counts.items())
                                 if counts else "无"))
    return lines
def _sftp_get_tree(cli, remote: str, local: Path) -> int:  # pragma: no cover
    """递归取回远程目录到本地，返回文件数。"""
    import stat as stat_mod
    sftp = cli.open_sftp()
    n = 0
    try:
        def walk(r: str, l: Path) -> None:
            nonlocal n
            l.mkdir(parents=True, exist_ok=True)
            for e in sftp.listdir_attr(r):
                rp, lp = f"{r}/{e.filename}", l / e.filename
                if stat_mod.S_ISDIR(e.st_mode):
                    walk(rp, lp)
                else:
                    sftp.get(rp, str(lp))
                    n += 1
        st = sftp.stat(remote)
        if stat_mod.S_ISDIR(st.st_mode):
            walk(remote, local)
        else:
            local.parent.mkdir(parents=True, exist_ok=True)
            sftp.get(remote, str(local))
            n = 1
    finally:
        sftp.close()
    return n
