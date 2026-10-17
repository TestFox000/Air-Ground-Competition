# -*- coding: utf-8 -*-
"""Find the drone after the LAN subnet changed: sweep the local /24 for SSH
port 22 and for mDNS name duzhong.local, plus a reverse-connect test."""
import concurrent.futures
import ipaddress
import socket
import subprocess
import time

MYIP = "10.240.174.14"
NET = ipaddress.ip_network("10.240.174.0/24")

def check(ip, port=22, timeout=0.4):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        return s.connect_ex((str(ip), port)) == 0
    finally:
        s.close()

hosts = [h for h in NET.hosts() if str(h) != MYIP]
print(f"sweeping {len(hosts)} hosts for :22 ...")
open_ips = []
with concurrent.futures.ThreadPoolExecutor(max_workers=128) as pool:
    for ip, ok in zip(hosts, pool.map(lambda h: check(h), hosts)):
        if ok:
            open_ips.append(str(ip))
print("port 22 open on:", open_ips or "(none)")

# mDNS name check, a few times (multicast needs the right interface up)
for attempt in range(3):
    try:
        addrs = socket.getaddrinfo("duzhong.local", None)
        print("duzhong.local ->", sorted({a[4][0] for a in addrs}))
        break
    except OSError as e:
        print(f"mDNS attempt {attempt + 1}: {e}")
        time.sleep(2)

# Who is hosting duzhong.local? ask avahi-resolve if present, else skip
try:
    out = subprocess.run(["avahi-resolve", "-n", "duzhong.local"],
                         capture_output=True, text=True, timeout=6)
    print("avahi-resolve:", out.stdout.strip() or out.stderr.strip())
except (FileNotFoundError, subprocess.TimeoutExpired) as e:
    print("avahi-resolve unavailable:", e)

# Any ARP neighbors right now
try:
    out = subprocess.run(["arp", "-a"], capture_output=True, text=True, timeout=6)
    print("\nARP table:\n" + out.stdout)
except Exception as e:
    print("arp failed:", e)
