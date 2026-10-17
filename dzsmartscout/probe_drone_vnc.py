# -*- coding: utf-8 -*-
"""Probe the drone: VNC servers, SSH keys, desktop env, ports."""
import paramiko

HOST, USER, PW = "10.230.128.188", "root", "duzhong"

CMDS = [
    ("vnc-servers", "command -v vncserver Xtigervnc x11vnc wayvnc tigervncserver 2>/dev/null; ls /usr/bin/*vnc* /usr/local/bin/*vnc* 2>/dev/null | head"),
    ("vnc-services", "systemctl list-units --type=service --all 2>/dev/null | grep -i -E 'vnc|x11|desktop|gdm|lightdm' | head; systemctl is-active vncserver@:1 2>/dev/null"),
    ("listening", "ss -ltnp 2>/dev/null | head -20"),
    ("x-display", "echo DISPLAY=$DISPLAY; ls /tmp/.X11-unix/ 2>/dev/null; ps aux | grep -E 'Xorg|Xvnc|Xtigervnc|wayland|labwc|weston|openbox|xfce' | grep -v grep | head"),
    ("ssh-dir", "ls -la /root/.ssh/ 2>/dev/null; ls -la /root/.ssh/authorized_keys 2>/dev/null && wc -l /root/.ssh/authorized_keys"),
    ("home", "ls /root/ | head -20; echo ---; ls /home/ 2>/dev/null"),
    ("pkg", "dpkg -l 2>/dev/null | grep -i -E 'vnc|tight|tiger' | head; apt list --installed 2>/dev/null | grep -i vnc | head"),
    ("desktop-env", "cat /etc/X11/default-display-manager 2>/dev/null; loginctl list-sessions 2>/dev/null | head"),
]

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PW, timeout=10, allow_agent=False, look_for_keys=False)
print(f"connected to {HOST} as {USER}\n")

for label, cmd in CMDS:
    stdin, stdout, stderr = client.exec_command(cmd, timeout=20)
    out = stdout.read().decode("utf-8", "replace").strip()
    err = stderr.read().decode("utf-8", "replace").strip()
    print(f"===== {label} =====")
    print(out if out else "(no output)")
    if err:
        print(f"[stderr] {err[:400]}")
    print()
client.close()
