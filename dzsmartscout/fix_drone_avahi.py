# -*- coding: utf-8 -*-
"""Fix avahi config (deny-interfaces must be under [server], not [rlimits]),
fix /etc/hosts 127.0.1.1 -> dzsmart, restart avahi, verify IPv4 advert."""
import paramiko

HOST, USER, PW = "10.240.174.104", "duzhong", "duzhong"

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PW, timeout=15,
               allow_agent=False, look_for_keys=False)

def sh(cmd, timeout=60, sudo=False):
    if sudo:
        cmd = f"echo {PW} | sudo -S -p '' sh -c {cmd!r}"
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", "replace").strip()
    err = stderr.read().decode("utf-8", "replace").strip()
    return out, err

print("== remove misplaced deny-interfaces line ==")
out, err = sh("sed -i '/^deny-interfaces=docker0$/d' /etc/avahi/avahi-daemon.conf; grep -n 'deny-interfaces' /etc/avahi/avahi-daemon.conf || echo 'line removed'", sudo=True)
print(out)
if err: print(f"[stderr] {err[:300]}")

print("\n== insert deny-interfaces under [server] ==")
out, err = sh("sed -i '/^\\[server\\]$/a deny-interfaces=docker0' /etc/avahi/avahi-daemon.conf; grep -n -A1 '^\\[server\\]' /etc/avahi/avahi-daemon.conf | head -6", sudo=True)
print(out)
if err: print(f"[stderr] {err[:300]}")

print("\n== fix /etc/hosts 127.0.1.1 ==")
out, err = sh("sed -i 's/^127.0.1.1.*$/127.0.1.1\\tdzsmart/' /etc/hosts; grep '127.0.1.1' /etc/hosts", sudo=True)
print(out)
if err: print(f"[stderr] {err[:300]}")

print("\n== restart avahi ==")
out, err = sh("systemctl restart avahi-daemon; sleep 2; systemctl is-active avahi-daemon", sudo=True, timeout=60)
print(out)
if err: print(f"[stderr] {err[:300]}")

print("\n== avahi journal ==")
out, _ = sh("journalctl -u avahi-daemon --no-pager -n 12 2>/dev/null | tail -12", sudo=True)
print(out)

print("\n== local resolve dzsmart.local ==")
out, _ = sh("getent hosts dzsmart.local; echo ---; getent ahostsv4 dzsmart.local || echo NO-IPV4")
print(out)

client.close()
