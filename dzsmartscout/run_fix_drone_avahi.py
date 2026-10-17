# -*- coding: utf-8 -*-
"""Upload remote_fix_drone_avahi.sh to drone and run it with sudo, then verify."""
import paramiko

HOST, USER, PW = "10.240.174.104", "duzhong", "duzhong"
LOCAL_SH = r"D:\User\Desktop\dzsmartscout\remote_fix_drone_avahi.sh"

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PW, timeout=15,
               allow_agent=False, look_for_keys=False)

sftp = client.open_sftp()
sftp.put(LOCAL_SH, "/tmp/fix_drone_avahi.sh")
sftp.close()

cmd = f"echo {PW} | sudo -S -p '' bash /tmp/fix_drone_avahi.sh"
stdin, stdout, stderr = client.exec_command(cmd, timeout=90)
out = stdout.read().decode("utf-8", "replace")
err = stderr.read().decode("utf-8", "replace")
print(out)
if err:
    print(f"[stderr]\n{err[:800]}")

print("\n== journal ==")
stdin, stdout, stderr = client.exec_command(
    "journalctl -u avahi-daemon --no-pager -n 12 2>/dev/null | tail -12", timeout=30)
print(stdout.read().decode("utf-8", "replace"))

print("== local resolve dzsmart.local ==")
for c in ["getent hosts dzsmart.local", "getent ahostsv4 dzsmart.local",
          "hostname"]:
    stdin, stdout, stderr = client.exec_command(c, timeout=30)
    o = stdout.read().decode("utf-8", "replace").strip()
    print(f"$ {c}\n{o}")

client.close()
