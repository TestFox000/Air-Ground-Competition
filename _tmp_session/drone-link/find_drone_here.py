#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
find_drone_here.py —— 在当前网段里找无人机（只做 TCP:22 连接探测 + 一次 ICMP，只读不登录）

为什么需要：无人机历史地址 10.29.122.104 已不可达（100% 丢包 + SSH 超时），
而车端已换到 192.168.137.241（本机 WLAN 192.168.137.51/24），需要先确认机载端
是不是也搬到了这个网段。

用法: python find_drone_here.py [网段前缀]      默认 192.168.137.
输出: 有 22 端口响应的主机列表（含反查名），以及 192.168.55.1（Jetson USB gadget 常用址）探测。
"""
import concurrent.futures as cf
import socket
import subprocess
import sys


def tcp_open(ip, port=22, timeout=0.6):
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect((ip, port))
        return True
    except Exception:
        return False
    finally:
        s.close()


def ping(ip, timeout_ms=900):
    try:
        r = subprocess.run(["ping", "-n", "1", "-w", str(timeout_ms), ip],
                           capture_output=True, text=True, timeout=5)
        return "TTL=" in (r.stdout or "")
    except Exception:
        return False


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else "192.168.137."
    ips = [prefix + str(i) for i in range(1, 255)]
    print("== 扫描 %s0/24 的 TCP:22（并发 64，超时 0.6s）==" % prefix)
    live22 = []
    with cf.ThreadPoolExecutor(max_workers=64) as ex:
        for ip, ok in zip(ips, ex.map(tcp_open, ips)):
            if ok:
                live22.append(ip)
                print("   [22 open] %s" % ip)
    print("== 有 22 端口的主机：%s ==" % (live22 or "无"))

    print("== 对上述主机做一次 ICMP + 反查名 ==")
    for ip in live22:
        try:
            name = socket.gethostbyaddr(ip)[0]
        except Exception:
            name = "?"
        print("   %-16s ping=%-5s name=%s" % (ip, ping(ip), name))

    print("== 额外探测：192.168.55.1（Jetson USB-C gadget 常用址）==")
    print("   tcp22=%s ping=%s" % (tcp_open("192.168.55.1"), ping("192.168.55.1")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
