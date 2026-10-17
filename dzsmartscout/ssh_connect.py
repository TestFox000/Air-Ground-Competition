# -*- coding: utf-8 -*-
"""SSH to the drone at 10.230.128.188 with password auth (paramiko)."""
import paramiko

HOST = "10.230.128.188"
PORT = 22
PASSWORD = "duzhong"
USERNAMES = ["root", "duzhong", "admin", "user", "nvidia", "ubuntu", "pi"]

def try_connect(username):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        HOST,
        port=PORT,
        username=username,
        password=PASSWORD,
        timeout=10,
        allow_agent=False,
        look_for_keys=False,
        banner_timeout=10,
        auth_timeout=10,
    )
    return client

def run(client, cmd, timeout=15):
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return out, err

def main():
    connected = None
    for user in USERNAMES:
        try:
            client = try_connect(user)
            print(f"[OK] login succeeded as '{user}'@{HOST}")
            connected = (user, client)
            break
        except paramiko.AuthenticationException:
            print(f"[FAIL] auth rejected for user '{user}'")
        except Exception as e:
            print(f"[FAIL] user '{user}': {type(e).__name__}: {e}")
            break  # network-level problem, no point trying other users

    if not connected:
        print("RESULT: no credentials worked")
        return 1

    user, client = connected
    commands = [
        ("whoami", "whoami"),
        ("hostname", "hostname"),
        ("uname -a", "kernel"),
        ("cat /etc/os-release | head -3", "os"),
        ("uptime", "uptime"),
        ("df -h | head -8", "disk"),
        ("ip -br addr 2>/dev/null || ifconfig", "network"),
    ]
    for cmd, label in commands:
        try:
            out, err = run(client, cmd)
            print(f"--- {label} ---")
            print(out.strip())
            if err.strip():
                print(f"[stderr] {err.strip()}")
        except Exception as e:
            print(f"--- {label} --- ERROR: {e}")

    client.close()
    print("RESULT: connected as", user)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
