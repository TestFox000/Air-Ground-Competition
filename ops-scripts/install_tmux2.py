# -*- coding: utf-8 -*-
"""安装 tmux（带断线重试）：apt 前先检查；连接断了重连再看，apt 本身幂等续传。"""
import sys
import time
import paramiko

HOST, USER, PASS = "192.168.55.1", "duzhong", "duzhong"

def connect():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=8,
              banner_timeout=8, auth_timeout=8)
    return c

def run(c, cmd, timeout=60):
    _, out, err = c.exec_command(cmd, timeout=timeout)
    o = out.read().decode("utf-8", "replace")
    e = err.read().decode("utf-8", "replace")
    return out.channel.recv_exit_status(), o, e

def tmux_ok(c):
    try:
        rc, o, _ = run(c, "tmux -V", timeout=10)
        return rc == 0, o.strip()
    except Exception:
        return False, ""

def main():
    for attempt in range(1, 5):
        try:
            c = connect()
            ok, ver = tmux_ok(c)
            if ok:
                print(f"tmux 已就绪: {ver}")
                c.close()
                return 0
            if attempt > 1:
                print(f"— 第 {attempt} 次连接恢复，继续安装")
            # 后台脱离执行，断线不杀 apt；日志落 /tmp/apt_tmux.log
            cmd = ("echo 'duzhong' | sudo -S bash -c "
                   "'DEBIAN_FRONTEND=noninteractive apt-get install -y tmux "
                   "> /tmp/apt_tmux.log 2>&1; echo RC=$? >> /tmp/apt_tmux.log' "
                   "&")
            run(c, cmd, timeout=15)
            # 轮询结果（断线则重连）
            for _ in range(40):
                time.sleep(6)
                try:
                    rc, o, _ = run(c, "tail -3 /tmp/apt_tmux.log; which tmux",
                                   timeout=10)
                    if "RC=" in o:
                        print(o.strip())
                        ok, ver = tmux_ok(c)
                        c.close()
                        if ok:
                            print(f"tmux 已就绪: {ver}")
                            return 0
                        print("apt 结束但 tmux 仍缺——看上面日志尾部")
                        return 1
                except Exception as ex:
                    print(f"轮询断线（{type(ex).__name__}），重连…")
                    break
            c.close()
        except Exception as e:
            print(f"第 {attempt} 次连接失败: {type(e).__name__}: {e}")
            time.sleep(3)
    print("多次重试仍未完成")
    return 1

if __name__ == "__main__":
    sys.exit(main())
