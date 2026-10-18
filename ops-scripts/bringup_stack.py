# -*- coding: utf-8 -*-
"""Orin 一键：部署 watchdog → 自检 → 起全栈（bringup）→ 加 bb/wd 窗口 → 核验。

只起栈和观测，不发任何飞行命令（不 arm、不起飞）。
"""
import sys
import time
import paramiko

HOST, USER, PASS = "192.168.55.1", "duzhong", "duzhong"
LOCAL_WD = (r"C:\Users\29212\CodeBuddy\20260927120106\air_ground"
            r"\tools\orin_watchdog.py")
TOOLS = "/home/duzhong/dzuav_deploy/air_ground/tools"

def connect():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=6,
              banner_timeout=6, auth_timeout=6)
    return c

def run(c, cmd, timeout=60):
    _, out, err = c.exec_command(cmd, timeout=timeout)
    o = out.read().decode("utf-8", "replace")
    e = err.read().decode("utf-8", "replace")
    return out.channel.recv_exit_status(), o, e

def with_retry(fn, tries=4, label=""):
    for i in range(1, tries + 1):
        try:
            return fn()
        except Exception as e:
            print(f"  [{label}] 第{i}次失败 {type(e).__name__}，重试…")
            time.sleep(2)
    raise RuntimeError(f"{label} 多次重试失败")

def main():
    # ---- 1. 部署 watchdog ----
    print("==> 部署 orin_watchdog.py")
    def _upload():
        c = connect()
        try:
            sftp = c.open_sftp()
            sftp.put(LOCAL_WD, f"{TOOLS}/orin_watchdog.py")
            sftp.close()
            rc, o, e = run(c, f"ls -la {TOOLS}/orin_watchdog.py", 10)
            print(" ", o.strip())
        finally:
            c.close()
    with_retry(_upload, label="upload")

    # ---- 2. watchdog --check ----
    print("==> watchdog --check")
    def _check():
        c = connect()
        try:
            rc, o, e = run(c, "bash -lc 'source /opt/ros/humble/setup.bash; "
                              "source ~/dzpx4/install/setup.bash; "
                              f"python3 {TOOLS}/orin_watchdog.py --check'", 60)
            print(o.strip())
            if e.strip():
                print("  stderr:", e.strip()[:300])
            return rc
        finally:
            c.close()
    rc = with_retry(_check, label="check")
    if rc != 0:
        print("watchdog 自检失败，终止")
        return 1

    # ---- 3. 起全栈 ----
    print("==> orin_bringup.sh up（起栈约 15s，不含起飞命令）")
    def _up():
        c = connect()
        try:
            rc, o, e = run(c, f"bash {TOOLS}/../tools/orin_bringup.sh up 2>&1",
                           timeout=120)
            print("\n".join(o.strip().splitlines()[-14:]))
            return rc
        finally:
            c.close()
    with_retry(_up, label="bringup")

    # ---- 4. 加 bb / wd 窗口（黑匣子 + 守卫，不依赖地面台）----
    print("==> 拉起黑匣子(bb)与守卫(wd)窗口")
    def _bbwd():
        c = connect()
        try:
            env = ("source /opt/ros/humble/setup.bash; "
                   "source ~/dzpx4/install/setup.bash; ")
            cmds = [
                # 幂等：先杀同名窗口再建
                "tmux kill-window -t agup:bb 2>/dev/null; "
                f"tmux new-window -d -t agup -n bb 'bash -lc \"{env}"
                f"python3 {TOOLS}/orin_flight_blackbox.py --log-dir ~/ag_flight_logs\"'",
                "tmux kill-window -t agup:wd 2>/dev/null; "
                f"tmux new-window -d -t agup -n wd 'bash -lc \"{env}"
                f"python3 {TOOLS}/orin_watchdog.py\"'",
            ]
            for cmd in cmds:
                rc, o, e = run(c, cmd, 15)
                if rc != 0:
                    print("  窗口命令失败:", e.strip()[:200])
        finally:
            c.close()
    with_retry(_bbwd, label="bbwd")
    print("  守卫介入授权：LAND=开，KILL=关（--enable-kill 才授权电机停转）")

    # ---- 5. 核验：窗口存活 + 飞控话题流速 ----
    time.sleep(8)
    print("==> 核验")
    def _verify():
        c = connect()
        try:
            rc, o, _ = run(c, "bash /home/duzhong/dzuav_deploy/air_ground/tools/"
                              "orin_bringup.sh status 2>&1", 15)
            print(o.strip())
            rc, o, _ = run(c,
                "bash -lc 'source /opt/ros/humble/setup.bash; "
                "source ~/dzpx4/install/setup.bash; "
                "timeout 6 ros2 topic hz /fmu/out/vehicle_local_position 2>&1 "
                "| head -3'", 25)
            print("local_position hz:", o.strip() or "(无数据——飞控串口桥未通？)")
        finally:
            c.close()
    with_retry(_verify, label="verify")
    print("\nDONE")
    return 0

if __name__ == "__main__":
    sys.exit(main())
