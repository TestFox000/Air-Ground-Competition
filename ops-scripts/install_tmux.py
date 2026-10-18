# -*- coding: utf-8 -*-
"""在 Orin 上安装 tmux（飞控三件套的编排依赖）。sudo 密码即登录密码。"""
import sys
import paramiko

HOST, USER, PASS = "192.168.55.1", "duzhong", "duzhong"

def run(c, cmd, timeout=240):
    _, out, err = c.exec_command(cmd, timeout=timeout)
    o = out.read().decode("utf-8", "replace")
    e = err.read().decode("utf-8", "replace")
    rc = out.channel.recv_exit_status()
    return rc, o, e

def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=8,
              banner_timeout=8, auth_timeout=8)

    rc, o, _ = run(c, "which tmux && tmux -V", timeout=10)
    if rc == 0:
        print("已装好:", o.strip())
        c.close()
        return 0

    print("安装 tmux（sudo apt-get install，可能要 1~2 分钟）…")
    # -S 从 stdin 读密码；-E 保留环境（代理设置若有）；DEBIAN_FRONTEND 防交互卡住
    cmd = ("echo 'duzhong' | sudo -S -E env DEBIAN_FRONTEND=noninteractive "
           "apt-get install -y tmux 2>&1")
    rc, o, e = run(c, cmd, timeout=280)
    tail = (o + e).strip().splitlines()[-8:]
    print("\n".join(tail))
    if rc != 0:
        print(f"\nAPT_FAIL rc={rc}——可能是机载没外网（NCM 不带 NAT）。"
              "可先试 apt-get update，或让 Orin 上 WiFi 后重试。")
        c.close()
        return 1

    rc, o, _ = run(c, "tmux -V", timeout=10)
    print("验证:", o.strip() if rc == 0 else "仍不可用")
    c.close()
    return 0 if rc == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
