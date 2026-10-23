# -*- coding: utf-8 -*-
"""One-shot drone setup for the plugin:
  1. add the mdns nss module so the drone itself resolves duzhong.local
     (stock Ubuntu ships mdns_minimal; this image dropped it — backup first);
  2. install this PC's ssh public key for user duzhong (and root), which the
     plugin's BatchMode probe / file layer depend on."""
import os
import pathlib
import socket
import paramiko

HOST = "duzhong.local"
FALLBACK_IP = "192.168.43.17"  # last known address of duzhong.local
PUB = pathlib.Path(os.path.expanduser(r"~\.ssh\id_rsa.pub")).read_text(encoding="utf-8").strip()

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
for target in (HOST, FALLBACK_IP):
    try:
        client.connect(target, username="root", password="duzhong", timeout=10,
                       allow_agent=False, look_for_keys=False)
        print(f"connected via {target}")
        break
    except (socket.gaierror, OSError) as e:
        print(f"{target}: {e}")
else:
    raise SystemExit("no reachable address for the drone")

def sh(cmd, timeout=30):
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    return (stdout.read().decode("utf-8", "replace").strip(),
            stderr.read().decode("utf-8", "replace").strip())

# 1 ── nsswitch mdns module ────────────────────────────────────────────────
line, _ = sh("grep '^hosts' /etc/nsswitch.conf")
print("nsswitch before:", line)
if "mdns" not in line:
    out, err = sh("cp -a /etc/nsswitch.conf /etc/nsswitch.conf.bak-mdns && "
                  "sed -i 's|^hosts:.*|hosts:          files mymachines mdns_minimal [NOTFOUND=return] resolve [!UNAVAIL=return] dns myhostname|' /etc/nsswitch.conf && "
                  "grep '^hosts' /etc/nsswitch.conf")
    print("nsswitch after: ", out)
    if err:
        print("[stderr]", err[:200])
else:
    print("mdns already present, no change")

out, err = sh("getent hosts duzhong.local && echo LOCAL-RESOLVE-OK || echo LOCAL-RESOLVE-FAILED")
print("drone self-resolve:", out)

# 2 ── authorized_keys for duzhong + root ──────────────────────────────────
for user, home in (("duzhong", "/home/duzhong"), ("root", "/root")):
    cmd = (
        f"mkdir -p {home}/.ssh && chmod 700 {home}/.ssh && "
        f"touch {home}/.ssh/authorized_keys && chmod 600 {home}/.ssh/authorized_keys && "
        f"grep -qF '{PUB.split()[1]}' {home}/.ssh/authorized_keys || "
        f"echo '{PUB}' >> {home}/.ssh/authorized_keys; "
        f"chown -R {user}:{user} {home}/.ssh; "
        f"wc -l < {home}/.ssh/authorized_keys"
    )
    out, err = sh(cmd)
    print(f"authorized_keys for {user}: {out.splitlines()[-1] if out else '(no output)'} lines",
          ("| stderr: " + err[:150]) if err else "")

# sanity: sudo for duzhong? just report
out, _ = sh("id duzhong; grep -E '^duzhong' /etc/sudoers 2>/dev/null || echo 'no sudoers line'")
print(out)
client.close()
print("DONE")
