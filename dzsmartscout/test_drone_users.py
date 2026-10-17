# -*- coding: utf-8 -*-
"""Check which users can log in to the drone (new IP) with password duzhong."""
import paramiko

HOST = "10.240.174.104"
for user in ("root", "duzhong"):
    try:
        c = paramiko.SSHClient()
        c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        c.connect(HOST, username=user, password="duzhong",
                  timeout=8, allow_agent=False, look_for_keys=False)
        stdin, stdout, _ = c.exec_command("whoami; echo HOME=$HOME; hostname -I")
        print(f"{user}: OK -> {stdout.read().decode().strip().splitlines()}")
        c.close()
    except Exception as e:
        print(f"{user}: FAIL {type(e).__name__}: {e}")
