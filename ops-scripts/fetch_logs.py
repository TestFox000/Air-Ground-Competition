# -*- coding: utf-8 -*-
"""只读：列出 Orin 上今天所有 ROS 日志 + 拉取下午飞行窗口的关键日志到笔记本。"""
import sys
import paramiko

HOST, USER, PASS = "192.168.55.1", "duzhong", "duzhong"
LOCAL_DIR = r"C:\Users\29212\Documents\kimi\tasks\2026-10-16\17-42-24-35e32761\flight_logs_1016"

LIST_CMD = (
    "find ~/.ros/log -type f -name '*.log' -newermt '2026-10-16 00:00' "
    "-printf '%T@ %s %p\n' 2>/dev/null | sort -n | "
    "awk '{printf \"%s %8d  %s\\n\", strftime(\"%H:%M\", $1), $2, $3}'"
)

def main():
    import os
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASS, timeout=8,
              banner_timeout=8, auth_timeout=8)

    print("===== 今天的 ROS 日志（按时间）=====")
    _, out, err = c.exec_command(LIST_CMD, timeout=30)
    listing = out.read().decode("utf-8", "replace")
    print(listing if listing.strip() else "(none)")

    # 选飞行窗口关键节点日志 + 黑匣子两件（小文件全拿，大文件截取也行——先全拿）
    want_substr = ("px4_control_node", "px4_teleop_node", "fastlio",
                   "sensor_combined_to_imu", "path_record", "path_follow",
                   "px4_path", "odom_convert", "goto")
    files = []
    for line in listing.splitlines():
        parts = line.split()
        if len(parts) >= 3 and any(s in parts[2] for s in want_substr):
            files.append(parts[2])
    files = sorted(set(files))
    print(f"\n===== 选中 {len(files)} 个文件取回 =====")
    os.makedirs(LOCAL_DIR, exist_ok=True)
    sftp = c.open_sftp()
    for f in files:
        local = os.path.join(LOCAL_DIR, os.path.basename(f))
        try:
            sftp.get(f, local)
            print(f"  ok {os.path.basename(f)} ({os.path.getsize(local)} B)")
        except Exception as e:
            print(f"  FAIL {f}: {e}")
    # 黑匣子两个会话（全部，很小）
    for sess in ("20261016_175854", "20261016_175917"):
        rdir = f"/home/duzhong/ag_flight_logs/{sess}"
        try:
            for e in sftp.listdir_attr(rdir):
                local = os.path.join(LOCAL_DIR, f"bb_{sess}_{e.filename}")
                sftp.get(f"{rdir}/{e.filename}", local)
                print(f"  ok bb_{sess}_{e.filename} ({os.path.getsize(local)} B)")
        except Exception as ex:
            print(f"  FAIL {rdir}: {ex}")
    sftp.close()
    c.close()
    print("\nDONE ->", LOCAL_DIR)
    return 0

if __name__ == "__main__":
    sys.exit(main())
