# -*- coding: utf-8 -*-
"""只读：摸清 Orin 上今天飞行日志的分布（不落盘、不改状态）。"""
import sys
import paramiko

HOST, USER, PASS = "192.168.55.1", "duzhong", "duzhong"

CMDS = [
    ("now_uptime",   "date; uptime"),
    ("bb_logs",      "ls -lat ~/ag_flight_logs/ 2>/dev/null | head -12 || echo NO_BB_ROOT"),
    ("bb_sessions",  "for d in ~/ag_flight_logs/*/; do echo \"== $d\"; ls -la \"$d\" | head -6; done 2>/dev/null | head -40"),
    ("ag_logs_tmp",  "ls -lat /tmp/ag_logs/ ~/ag_logs/ 2>/dev/null | head -15 || echo NO_AG_LOGS"),
    ("ros_log",      "ls -lat ~/.ros/log/ 2>/dev/null | head -12 || echo NO_ROS_LOG"),
    ("ulog",         "find /home/duzhong -maxdepth 4 -name '*.ulg' -mtime -2 2>/dev/null | head -10; echo ---; ls -lat ~/dzpx4/log* ~/log 2>/dev/null | head -8"),
    ("dzuav_out",    "ls -lat ~/dzuav/logs ~/dzuav_deploy/air_ground/logs 2>/dev/null | head -10 || echo NO_DZUAV_LOGS"),
    ("history",      "tail -60 ~/.bash_history 2>/dev/null"),
    ("blackbox_dep", "ls -la ~/dzuav_deploy/air_ground/tools/orin_flight_blackbox.py 2>/dev/null"),
    ("systemd",      "systemctl list-units --type=service --state=running 2>/dev/null | grep -iE 'px4|ros|livox|fastlio|dz' || echo NO_MATCHING_SERVICES"),
]

def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=8,
              banner_timeout=8, auth_timeout=8)
    for name, cmd in CMDS:
        print(f"\n===== [{name}]")
        try:
            _, out, err = c.exec_command(cmd, timeout=20)
            txt = (out.read().decode("utf-8", "replace")
                   + err.read().decode("utf-8", "replace")).strip()
            print(txt if txt else "(empty)")
        except Exception as e:
            print(f"CMD_FAIL: {e}")
    c.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
