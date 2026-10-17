# -*- coding: utf-8 -*-
"""On the drone (new address, login as duzhong + sudo): check avahi state,
fix .local resolution by adding the mdns nss module, verify resolution.
Idempotent. Password 'duzhong' piped to sudo as needed."""
import paramiko

HOST, USER, PW = "10.240.174.104", "duzhong", "duzhong"

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PW, timeout=15,
               allow_agent=False, look_for_keys=False)

SUDO = f"echo {PW} | sudo -S -p '' "

def sh(cmd, timeout=40, sudo=False):
    full = (SUDO + cmd) if sudo else cmd
    stdin, stdout, stderr = client.exec_command(full, timeout=timeout)
    out = stdout.read().decode("utf-8", "replace").strip()
    err = stderr.read().decode("utf-8", "replace").strip()
    return out, err

print("== identity ==")
out, _ = sh("whoami; hostname; cat /etc/hostname; uptime")
print(out)

print("\n== sudo check ==")
out, err = sh("id", sudo=True)
print(out or f"SUDO FAILED: {err[:200]}")

print("\n== avahi service ==")
out, _ = sh("systemctl is-enabled avahi-daemon; systemctl is-active avahi-daemon", sudo=True)
print(out)

print("\n== avahi journal (last 10) ==")
out, _ = sh("journalctl -u avahi-daemon --no-pager -n 10 2>/dev/null | tail -10", sudo=True)
print(out)

print("\n== nsswitch hosts line ==")
out, _ = sh("grep '^hosts' /etc/nsswitch.conf", sudo=True)
print(out)

hosts_line = out
if "mdns" not in hosts_line.split("hosts:", 1)[-1]:
    print("\n== patching nsswitch.conf (backup first) ==")
    out, err = sh("cp -a /etc/nsswitch.conf /etc/nsswitch.conf.bak-mdns && "
                  "sed -i 's|^hosts:.*|hosts:          files mymachines mdns_minimal [NOTFOUND=return] resolve [!UNAVAIL=return] dns myhostname|' /etc/nsswitch.conf && "
                  "grep '^hosts' /etc/nsswitch.conf", sudo=True)
    print(out)
    if err:
        print(f"[stderr] {err[:300]}")
else:
    print("== mdns already in nsswitch, no change ==")

print("\n== local resolve after ==")
out, err = sh("getent hosts duzhong.local && echo LOCAL-RESOLVE-OK || echo LOCAL-RESOLVE-FAILED")
print(out)
if err:
    print(f"[stderr] {err[:200]}")

print("\n== mDNS listeners on 5353 ==")
out, _ = sh("ss -lunp 2>/dev/null | grep 5353 || echo 'no UDP 5353 listener'", sudo=True)
print(out)

print("\n== avahi publish config (non-comment) ==")
out, _ = sh("grep -E '^(enable|use-ipv|publish-workstation|publish-hinfo|allow-interfaces|deny-interfaces)' /etc/avahi/avahi-daemon.conf | grep -v '^#'", sudo=True)
print(out or "(all defaults)")

client.close()
