# -*- coding: utf-8 -*-
"""Verify whether 10.240.174.104 is our drone: ssh banner + password login +
hostname/kernel check, and probe VNC 5901."""
import socket
import paramiko

CANDIDATES = ["10.240.174.104"]

def banner(ip, port, timeout=3):
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect((ip, port))
        return s.recv(128).decode("utf-8", "replace").strip()
    except Exception as e:
        return f"error: {e}"
    finally:
        s.close()

def tcp(ip, port, timeout=1):
    s = socket.socket()
    s.settimeout(timeout)
    try:
        return s.connect_ex((ip, port)) == 0
    finally:
        s.close()

for ip in CANDIDATES:
    print(f"===== {ip} =====")
    print("ssh banner:", banner(ip, 22))
    print("vnc 5901 open:", tcp(ip, 5901))
    print("adbd 5555 open:", tcp(ip, 5555))
    try:
        c = paramiko.SSHClient()
        c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        c.connect(ip, username="root", password="duzhong", timeout=8,
                  allow_agent=False, look_for_keys=False)
        stdin, stdout, _ = c.exec_command(
            "hostname; uname -r; ip -4 addr show wlan0 2>/dev/null | grep inet; "
            "systemctl is-active avahi-daemon vncserver@:1 2>/dev/null", timeout=15)
        print("LOGIN OK as root:\n" + stdout.read().decode("utf-8", "replace").strip())
        c.close()
    except Exception as e:
        print(f"login failed: {type(e).__name__}: {e}")
