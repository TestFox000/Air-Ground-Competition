#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t4_ssh.py —— 无人机/小车统一远端执行器（本地 Windows 用，仅本任务验证所需的最小工具）

为什么要有它：
  * 无人机 `duzhong@10.29.122.104` 用免密密钥；
  * 小车 `duzhong@10.29.122.188` **只有密码认证**（本机没有 sshpass/plink），
    必须走 paramiko；
  * PowerShell 5.1 给 ssh.exe 传参会吃引号/换行 —— 把要跑的东西写成脚本再传过去最稳。

四个子命令：
  exec <dev> "<cmd>"            直接跑一条命令
  run  <dev> <local.sh> [...]   上传脚本到 /tmp 再跑（自动 sed -i 's/\r$//' 去 CR）
  put  <dev> <local> <remote>   上传文件（自动 mkdir -p 目标目录、去 CR）
  get  <dev> <remote> <local>   下载文件
  bg   <dev> "<cmd>" <logfile>  远端后台跑（setsid nohup），立刻返回

安全/边界：
  * 只用于只读侦察 + /tmp 探针；不做任何持久化配置修改。
  * 小车密码从环境变量 DZ_CAR_PASS 取，缺省 'duzhong'（本仓库已知的 VNC/登录口令）。
"""
import os
import posixpath
import sys
import time

import paramiko

try:  # Windows 控制台默认 GBK，远端 UTF-8 输出会炸 —— 强制 UTF-8/可替换
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

CAR_HOST = os.environ.get("DZ_CAR_HOST", "10.29.122.188")
CAR_USER = os.environ.get("DZ_CAR_USER", "duzhong")
CAR_PASS = os.environ.get("DZ_CAR_PASS", "duzhong")
DRONE_HOST = os.environ.get("DZ_DRONE_HOST", "10.29.122.104")
DRONE_USER = os.environ.get("DZ_DRONE_USER", "duzhong")
DRONE_KEY = os.environ.get("DZ_DRONE_KEY", os.path.expanduser("~/.ssh/id_rsa"))


def connect(dev):
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    if dev == "car":
        c.connect(CAR_HOST, username=CAR_USER, password=CAR_PASS,
                  timeout=15, look_for_keys=False, allow_agent=False)
    elif dev == "drone":
        kw = dict(username=DRONE_USER, timeout=15, allow_agent=False)
        if os.path.isfile(DRONE_KEY):
            kw["key_filename"] = DRONE_KEY
            kw["look_for_keys"] = False
        c.connect(DRONE_HOST, **kw)
    else:
        raise SystemExit("dev must be car|drone")
    return c


def run(c, cmd, timeout=300):
    chan = c.get_transport().open_session()
    chan.settimeout(timeout)
    chan.set_combine_stderr(True)   # ★ 必须：否则远端 stderr 走 SSH 扩展通道，会被丢掉
    chan.exec_command(cmd)
    out = b""
    while True:
        if chan.recv_ready():
            out += chan.recv(65536)
        elif chan.exit_status_ready() and not chan.recv_ready():
            break
        else:
            time.sleep(0.05)
    while chan.recv_ready():
        out += chan.recv(65536)
    rc = chan.recv_exit_status()
    chan.close()
    return rc, out.decode("utf-8", "replace")


def put(c, local, remote):
    sftp = c.open_sftp()
    d = posixpath.dirname(remote)
    if d:
        # 只新建目录，不覆盖
        parts = d.strip("/").split("/")
        cur = ""
        for p in parts:
            cur += "/" + p
            try:
                sftp.stat(cur)
            except IOError:
                sftp.mkdir(cur)
    data = open(local, "rb").read().replace(b"\r\n", b"\n")
    with sftp.open(remote, "wb") as f:
        f.write(data)
    sftp.close()
    return len(data)


def get(c, remote, local):
    sftp = c.open_sftp()
    sftp.get(remote, local)
    n = sftp.stat(remote).st_size
    sftp.close()
    return n


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    sub, dev = sys.argv[1], sys.argv[2]
    c = connect(dev)
    try:
        if sub == "exec":
            rc, out = run(c, sys.argv[3])
            sys.stdout.write(out)
            print("\n### exit=%d" % rc)
            return rc
        if sub == "run":
            local = sys.argv[3]
            args = " ".join(sys.argv[4:])
            base = posixpath.basename(local.replace("\\", "/"))
            base = base[:-3] if base.endswith(".py") else base[:-3] if base.endswith(".sh") else base
            remote = "/tmp/t4_%s_%s.sh" % (dev, base)
            if local.endswith(".py"):
                remote = "/tmp/t4_%s_%s.py" % (dev, base)
            n = put(c, local, remote)
            rc0, _ = run(c, "chmod +x %s" % remote)
            interp = "python3" if remote.endswith(".py") else "bash"
            rc, out = run(c, "%s %s %s" % (interp, remote, args))
            sys.stdout.write(out)
            print("\n### uploaded %s (%d bytes, CR-normalized)  exit=%d" % (remote, n, rc))
            return rc
        if sub == "put":
            n = put(c, sys.argv[3], sys.argv[4])
            print("### put %s -> %s (%d bytes)" % (sys.argv[3], sys.argv[4], n))
            return 0
        if sub == "get":
            n = get(c, sys.argv[3], sys.argv[4])
            print("### get %s -> %s (%d bytes)" % (sys.argv[3], sys.argv[4], n))
            return 0
        if sub == "bg":
            cmd, logf = sys.argv[3], sys.argv[4]
            full = ("setsid nohup bash -c %s > %s 2>&1 < /dev/null & echo BG_PID=$!; sleep 1; exit 0"
                    % (shell_quote(cmd), logf))
            rc, out = run(c, full)
            sys.stdout.write(out)
            print("### bg started, log=%s exit=%d" % (logf, rc))
            return 0
        raise SystemExit("unknown subcommand " + sub)
    finally:
        c.close()


def shell_quote(s):
    return "'" + s.replace("'", "'\"'\"'") + "'"


if __name__ == "__main__":
    sys.exit(main())
