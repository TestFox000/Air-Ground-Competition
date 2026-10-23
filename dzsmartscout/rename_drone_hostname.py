# -*- coding: utf-8 -*-
"""Drone at 10.240.174.104: rename hostname -> dzsmart so mDNS publishes dzsmart.local.
Also deny docker0 in avahi so dzsmart.local only advertises the Wi-Fi address."""
import paramiko

HOST, USER, PW = "10.240.174.104", "duzhong", "duzhong"

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PW, timeout=15,
               allow_agent=False, look_for_keys=False)

def sh(cmd, timeout=40, sudo=False):
    if sudo:
        cmd = f"echo {PW} | sudo -S -p '' sh -c '{cmd}'"
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", "replace").strip()
    err = stderr.read().decode("utf-8", "replace").strip()
    return out, err

print("== rename hostname -> dzsmart ==")
out, err = sh("hostnamectl set-hostname dzsmart && hostnamectl --static", sudo=True)
print(out)
if err:
    print(f"[stderr] {err[:300]}")

print("\n== update /etc/hosts 127.0.1.1 ==")
out, err = sh("sed -i 's/^127\\.0\\.1\\.1\\s.*/127.0.1.1\\tdzsmart/' /etc/hosts && grep '127.0.1.1' /etc/hosts", sudo=True)
print(out)
if err:
    print(f"[stderr] {err[:300]}")

print("\n== deny docker0 in avahi config ==")
out, err = sh("grep -q '^deny-interfaces' /etc/avahi/avahi-daemon.conf || echo 'deny-interfaces=docker0' >> /etc/avahi/avahi-daemon.conf; grep '^deny-interfaces' /etc/avahi/avahi-daemon.conf", sudo=True)
print(out)
if err:
    print(f"[stderr] {err[:300]}")

print("\n== restart avahi ==")
out, err = sh("systemctl restart avahi-daemon && sleep 2 && systemctl is-active avahi-daemon", sudo=True, timeout=60)
print(out)
if err:
    print(f"[stderr] {err[:300]}")

print("\n== avahi journal ==")
out, _ = sh("journalctl -u avahi-daemon --no-pager -n 10 2>/dev/null | tail -10", sudo=True)
print(out)

print("\n== local resolve dzsmart.local ==")
out, _ = sh("getent hosts dzsmart.local && echo LOCAL-RESOLVE-OK || echo LOCAL-RESOLVE-FAILED")
print(out)

print("\n== final hostname ==")
out, _ = sh("hostname")
print(out)

client.close()
