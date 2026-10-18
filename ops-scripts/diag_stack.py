# -*- coding: utf-8 -*-
"""起栈后诊断：窗口内容 / 设备文件 / 话题清单，并补拉 bb 窗口。"""
import sys
import time
import paramiko

HOST, USER, PASS = "192.168.55.1", "duzhong", "duzhong"

def connect():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=6,
              banner_timeout=6, auth_timeout=6)
    return c

def run(c, cmd, timeout=30):
    _, out, err = c.exec_command(cmd, timeout=timeout)
    return (out.read().decode("utf-8", "replace")
            + err.read().decode("utf-8", "replace"))

def with_retry(fn, tries=4, label=""):
    for i in range(1, tries + 1):
        try:
            return fn()
        except Exception as e:
            print(f"  [{label}] 第{i}次失败 {type(e).__name__}，重试…")
            time.sleep(2)
    raise RuntimeError(label)

def main():
    def _diag():
        c = connect()
        try:
            print("===== 设备文件 =====")
            print(run(c, "ls /dev/ttyACM* /dev/video* 2>&1 | head -10").strip())
            for w in ("px4", "dzuav", "control", "wd"):
                print(f"\n===== tmux {w} 窗口尾部 =====")
                print(run(c, f"tmux capture-pane -pt agup:{w} -S -12 2>&1").strip()[-700:])
            print("\n===== ros2 topic list（前 20）=====")
            print(run(c, "bash -lc 'source /opt/ros/humble/setup.bash; "
                         "source ~/dzpx4/install/setup.bash; "
                         "timeout 8 ros2 topic list 2>&1 | head -20'", 20).strip())
        finally:
            c.close()
    with_retry(_diag, label="diag")

    def _bb():
        c = connect()
        try:
            env = ("source /opt/ros/humble/setup.bash; "
                   "source ~/dzpx4/install/setup.bash; ")
            run(c, "tmux kill-window -t agup:bb 2>/dev/null; "
                   f"tmux new-window -d -t agup -n bb 'bash -lc \"{env}"
                   "python3 /home/duzhong/dzuav_deploy/air_ground/tools/"
                   "orin_flight_blackbox.py --log-dir ~/ag_flight_logs\"'", 15)
            print("\n===== bb 窗口补拉 =====")
            print(run(c, "tmux list-windows -t agup -F '#{window_name}'", 10))
        finally:
            c.close()
    with_retry(_bb, label="bb")
    return 0

if __name__ == "__main__":
    sys.exit(main())
