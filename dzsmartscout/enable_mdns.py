# -*- coding: utf-8 -*-
"""Inspect (and if needed install/enable) Avahi mDNS on the drone so that
duzhong.local resolves to it, mirroring the car's setup."""
import paramiko

HOST, USER, PW = "10.230.128.188", "root", "duzhong"

def sh(client, cmd, timeout=30):
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", "replace").strip()
    err = stderr.read().decode("utf-8", "replace").strip()
    return out, err

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PW, timeout=10,
               allow_agent=False, look_for_keys=False)

print("===== before =====")
for label, cmd in [
    ("hostname", "hostname; cat /etc/hostname"),
    ("avahi-pkg", "dpkg-query -W -f='${Status} ${Version}\\n' avahi-daemon 2>/dev/null || echo NOT-INSTALLED"),
    ("avahi-service", "systemctl is-enabled avahi-daemon 2>&1; systemctl is-active avahi-daemon 2>&1"),
    ("mdns-5353", "ss -lunp 2>/dev/null | grep 5353 || echo 'no UDP 5353 listener'"),
    ("nsswitch", "grep '^hosts' /etc/nsswitch.conf"),
    ("resolved", "resolvectl status 2>/dev/null | head -20"),
]:
    out, err = sh(client, cmd)
    print(f"--- {label} ---")
    print(out or "(no output)")
    if err:
        print(f"[stderr] {err[:300]}")

# ── act ─────────────────────────────────────────────────────────────────────
pkg, _ = sh(client, "dpkg-query -W -f='${Status}' avahi-daemon 2>/dev/null")
if "install ok installed" not in pkg:
    print("\n===== installing avahi-daemon =====")
    out, err = sh(client,
        "export DEBIAN_FRONTEND=noninteractive; "
        "apt-get install -y avahi-daemon 2>&1 || "
        "(apt-get update 2>&1 | tail -3; apt-get install -y avahi-daemon 2>&1 | tail -10)",
        timeout=240)
    print((out or "")[-1500:])
    if err:
        print(f"[stderr] {err[-500:]}")

print("\n===== enabling avahi =====")
out, err = sh(client, "systemctl enable --now avahi-daemon 2>&1; sleep 1; "
                      "systemctl is-enabled avahi-daemon; systemctl is-active avahi-daemon", timeout=60)
print(out)
if err:
    print(f"[stderr] {err[:300]}")

print("\n===== after =====")
for label, cmd in [
    ("avahi-service", "systemctl is-enabled avahi-daemon; systemctl is-active avahi-daemon"),
    ("mdns-5353", "ss -lunp 2>/dev/null | grep 5353 || echo 'no UDP 5353 listener'"),
    ("publish", "grep -E '^(enable|publish|use-ipv|deny-interfaces|allow-interfaces)' /etc/avahi/avahi-daemon.conf 2>/dev/null | grep -v '^#'"),
    ("local-resolve", "getent hosts duzhong.local || echo 'local resolve failed'"),
    ("avahi-log", "journalctl -u avahi-daemon --no-pager -n 8 2>/dev/null | tail -8"),
]:
    out, err = sh(client, cmd)
    print(f"--- {label} ---")
    print(out or "(no output)")
    if err:
        print(f"[stderr] {err[:200]}")

client.close()
