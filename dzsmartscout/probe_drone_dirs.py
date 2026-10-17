# -*- coding: utf-8 -*-
"""Find project/workspace directories on the drone for file-browser shortcuts."""
import paramiko

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect("10.230.128.188", username="root", password="duzhong",
               timeout=10, allow_agent=False, look_for_keys=False)

CMDS = [
    ("root-home", "ls -la /root/ | head -30"),
    ("duzhong-home", "ls -la /home/duzhong/ | head -30"),
    ("workspaces", "find /root /home/duzhong -maxdepth 3 -name 'src' -type d 2>/dev/null | head; find /root /home/duzhong -maxdepth 2 -iname '*ws*' -o -maxdepth 2 -iname '*smartscout*' -o -maxdepth 2 -iname '*px4*' 2>/dev/null | head -20"),
    ("ros", "ls /opt/ros/ 2>/dev/null; echo ---; ls -d ~/PX4* ~/px4* 2>/dev/null"),
]
for label, cmd in CMDS:
    stdin, stdout, stderr = client.exec_command(cmd, timeout=20)
    out = stdout.read().decode("utf-8", "replace").strip()
    print(f"===== {label} =====")
    print(out if out else "(no output)")
    err = stderr.read().decode("utf-8", "replace").strip()
    if err:
        print(f"[stderr] {err[:300]}")
    print()
client.close()
