# -*- coding: utf-8 -*-
"""核查电池话题是否在发布（bridge 里 battery 一直 None）。"""
import sys
import time
import paramiko

HOST, USER, PASS = "192.168.55.1", "duzhong", "duzhong"

CMD = ("bash -lc 'source /opt/ros/humble/setup.bash; "
       "source ~/dzpx4/install/setup.bash; "
       "timeout 8 ros2 topic hz /fmu/out/battery_status 2>&1 | head -4; "
       "echo ---; "
       "timeout 8 ros2 topic echo --once /fmu/out/battery_status 2>&1 | head -12'")

def main():
    for i in range(1, 6):
        try:
            c = paramiko.SSHClient()
            c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            c.connect(HOST, username=USER, password=PASS, timeout=6,
                      banner_timeout=6, auth_timeout=6)
            _, out, err = c.exec_command(CMD, timeout=40)
            print((out.read().decode("utf-8", "replace")
                   + err.read().decode("utf-8", "replace")).strip())
            c.close()
            return 0
        except Exception as e:
            print(f"第{i}次连接失败: {type(e).__name__}")
            time.sleep(2)
    return 1

if __name__ == "__main__":
    sys.exit(main())
