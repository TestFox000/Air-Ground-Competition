# -*- coding: utf-8 -*-
"""在 Orin 上空跑黑匣子 --list-topics：验证订阅注册（类型导入/节点创建），并确认 tmux。"""
import sys
import paramiko

HOST, USER, PASS = "192.168.55.1", "duzhong", "duzhong"

CMDS = [
    ("tmux", "tmux -V"),
    ("blackbox_list_topics",
     "bash -lc 'source /opt/ros/humble/setup.bash; "
     "source ~/dzpx4/install/setup.bash; "
     "python3 ~/dzuav_deploy/air_ground/tools/orin_flight_blackbox.py --list-topics'"),
]

def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=8,
              banner_timeout=8, auth_timeout=8)
    for name, cmd in CMDS:
        print(f"===== [{name}]")
        try:
            _, out, err = c.exec_command(cmd, timeout=90)
            o = out.read().decode("utf-8", "replace")
            e = err.read().decode("utf-8", "replace")
            rc = out.channel.recv_exit_status()
            print(o.strip())
            if e.strip():
                print("--- stderr ---")
                print(e.strip()[:600])
            print(f"[rc={rc}]")
        except Exception as ex:
            print(f"CMD_FAIL: {type(ex).__name__}: {ex}")
    c.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
