# -*- coding: utf-8 -*-
"""只读探测 Orin 第二轮：飞控工具依赖的机载环境前提。"""
import sys
import paramiko

HOST, USER, PASS = "192.168.55.1", "duzhong", "duzhong"

CMDS = [
    ("tmux_bin",    "which tmux || echo TMUX_MISSING"),
    ("ros2_cli",    "bash -lc 'source /opt/ros/humble/setup.bash 2>/dev/null; which ros2' || echo ROS2_CLI_MISSING"),
    ("dzpx4_ws",    "ls ~/dzpx4/install/setup.bash 2>/dev/null || echo NO_DZPX4_WS"),
    ("dzpx4_pkgs",  "bash -lc 'source /opt/ros/humble/setup.bash 2>/dev/null; [ -f ~/dzpx4/install/setup.bash ] && source ~/dzpx4/install/setup.bash; ros2 pkg list 2>/dev/null | grep -E \"^px4_msgs$|^px4_control$\"'"),
    ("dzrobot_dir", "ls -d ~/dzrobot 2>/dev/null || echo NO_DZROBOT_DIR"),
    ("domain_env",  "env | grep -i ROS_DOMAIN || echo NO_DOMAIN_IN_DEFAULT_SHELL"),
    ("python3",     "python3 --version"),
    ("dzuav_tools", "ls ~/dzuav_deploy/air_ground/tools/ 2>/dev/null | head -15"),
]

def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=8,
              banner_timeout=8, auth_timeout=8)
    for name, cmd in CMDS:
        print(f"\n===== [{name}]")
        try:
            _, out, err = c.exec_command(cmd, timeout=15)
            txt = (out.read().decode("utf-8", "replace")
                   + err.read().decode("utf-8", "replace")).strip()
            print(txt if txt else "(empty)")
        except Exception as e:
            print(f"CMD_FAIL: {e}")
    c.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
