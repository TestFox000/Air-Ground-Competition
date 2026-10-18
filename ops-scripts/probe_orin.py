# -*- coding: utf-8 -*-
"""只读探测 Orin：连接、系统信息、飞控栈状态。不改动任何机载状态。"""
import sys
import paramiko

HOST, USER, PASS = "192.168.55.1", "duzhong", "duzhong"

CMDS = [
    ("identity",   "hostname && whoami && uname -m"),
    ("uptime",     "uptime"),
    ("tmux",       "tmux ls 2>&1 | head -20"),
    ("dzpx4_proc", "pgrep -af 'MicroXRCE|px4|fastlio|livox' | head -15"),
    ("dzros.env",  "cat ~/dzrobot/dzros.env 2>/dev/null | head -10"),
    ("ros_ver",    "ls /opt/ros/ 2>/dev/null"),
    ("mavlink_udp","ss -ulpn 2>/dev/null | grep -E '14550|14580|8888' || echo NO_MAVLINK_UDP"),
    ("dzuav_dir",  "ls ~/dzuav_deploy/air_ground 2>/dev/null | head -10 || echo NO_DEPLOY_DIR"),
    ("disk",       "df -h / | tail -1"),
]

def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        c.connect(HOST, username=USER, password=PASS, timeout=8,
                  banner_timeout=8, auth_timeout=8)
    except Exception as e:
        print(f"SSH_CONNECT_FAIL: {type(e).__name__}: {e}")
        return 1
    print(f"SSH_OK {USER}@{HOST}")
    for name, cmd in CMDS:
        print(f"\n===== [{name}] $ {cmd}")
        try:
            _, out, err = c.exec_command(cmd, timeout=10)
            txt = (out.read().decode("utf-8", "replace")
                   + err.read().decode("utf-8", "replace")).strip()
            print(txt if txt else "(empty)")
        except Exception as e:
            print(f"CMD_FAIL: {e}")
    c.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
