# -*- coding: utf-8 -*-
"""推送修复版黑匣子 + --list-topics 自检（整段操作带重试，NCM 链路抖动对策）。"""
import sys
import time
import paramiko

HOST, USER, PASS = "192.168.55.1", "duzhong", "duzhong"
LOCAL = (r"C:\Users\29212\CodeBuddy\20260927120106\air_ground"
         r"\tools\orin_flight_blackbox.py")
REMOTE = "/home/duzhong/dzuav_deploy/air_ground/tools/orin_flight_blackbox.py"

def attempt():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=6,
              banner_timeout=6, auth_timeout=6)
    try:
        sftp = c.open_sftp()
        sftp.put(LOCAL, REMOTE)
        sftp.close()
        _, out, _ = c.exec_command(f"ls -la {REMOTE}", timeout=10)
        print("已上传:", out.read().decode().strip())
        cmd = ("bash -lc 'source /opt/ros/humble/setup.bash; "
               "source ~/dzpx4/install/setup.bash; "
               "python3 ~/dzuav_deploy/air_ground/tools/orin_flight_blackbox.py "
               "--list-topics'")
        _, out, err = c.exec_command(cmd, timeout=90)
        print("\n===== --list-topics =====")
        print(out.read().decode("utf-8", "replace").strip())
        e = err.read().decode("utf-8", "replace").strip()
        if e:
            print("--- stderr ---")
            print(e[:800])
    finally:
        c.close()

def main():
    for i in range(1, 7):
        try:
            attempt()
            return 0
        except Exception as e:
            print(f"第{i}次失败: {type(e).__name__}: {e}，2s 后重试")
            time.sleep(2)
    print("多次重试仍失败——链路持续不通，等对面插稳后再跑")
    return 1

if __name__ == "__main__":
    sys.exit(main())
