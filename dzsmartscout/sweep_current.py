# -*- coding: utf-8 -*-
"""ICMP sweep of the CURRENT subnet (10.240.174.0/24) plus a second TCP:22
pass, to find the drone after the LAN renumbered."""
import concurrent.futures
import ipaddress
import subprocess
import socket

NET = ipaddress.ip_network("10.240.174.0/24")
hosts = [str(h) for h in NET.hosts()]

def icmp(ip):
    try:
        r = subprocess.run(["ping", "-n", "1", "-w", "700", ip],
                           capture_output=True, timeout=3)
        return ip if r.returncode == 0 else None
    except Exception:
        return None

def tcp22(ip):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.4)
    try:
        return ip if s.connect_ex((ip, 22)) == 0 else None
    finally:
        s.close()

with concurrent.futures.ThreadPoolExecutor(max_workers=100) as pool:
    alive = [x for x in pool.map(icmp, hosts) if x]
print("ICMP alive:", ", ".join(alive) or "(none)")

with concurrent.futures.ThreadPoolExecutor(max_workers=100) as pool:
    ssh = [x for x in pool.map(tcp22, hosts) if x]
print("TCP:22 open:", ", ".join(ssh) or "(none)")

# For every live host, also probe 5901 (VNC) and 5555 (adbd) to recognise the drone
for ip in alive:
    for port in (5901, 5555, 22):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.4)
        ok = s.connect_ex((ip, port)) == 0
        s.close()
        if ok:
            print(f"  {ip}:{port} open")
