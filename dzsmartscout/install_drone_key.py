# -*- coding: utf-8 -*-
"""ssh-copy-id equivalent: append the PC's public key to the drone's
~/.ssh/authorized_keys, then verify passwordless login works."""
import os
import paramiko

HOST, USER, PW = "dzsmart.local", "duzhong", "duzhong"
pub_path = os.path.expanduser("~/.ssh/id_rsa.pub")
pub = open(pub_path, "r", encoding="utf-8").read().strip()
print("local pubkey:", pub.split()[-1][:24], "...")

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PW, timeout=15,
               allow_agent=False, look_for_keys=False)

def sh(cmd, timeout=30, sudo=False):
    if sudo:
        cmd = f"echo {PW} | sudo -S -p '' sh -c '{cmd}'"
    i, o, e = client.exec_command(cmd, timeout=timeout)
    return o.read().decode("utf-8", "replace").strip(), e.read().decode("utf-8", "replace").strip()

# Show current state
out, _ = sh("ls -la ~/.ssh 2>/dev/null || echo '(no .ssh)'; wc -l ~/.ssh/authorized_keys 2>/dev/null || echo 0")
print("before:\n", out)

# Append key idempotently (single quotes: key has no single quotes; use double-quoted awk-free approach)
script = f"""
mkdir -p ~/.ssh && chmod 700 ~/.ssh
touch ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys
if ! grep -qF '{pub}' ~/.ssh/authorized_keys; then echo '{pub}' >> ~/.ssh/authorized_keys; echo ADDED; else echo ALREADY-PRESENT; fi
wc -l ~/.ssh/authorized_keys
"""
out, err = sh(script)
print("install:\n", out)
if err:
    print("[stderr]", err[:300])

client.close()

# Verify passwordless login via real ssh.exe (BatchMode)
import subprocess
r = subprocess.run(
    ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
     "-o", "ConnectTimeout=10", f"{USER}@{HOST}", "hostname"],
    capture_output=True, text=True, timeout=30)
print("BatchMode exit:", r.returncode, "| out:", r.stdout.strip(), "| err:", r.stderr.strip()[:200])
