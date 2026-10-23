#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t4_mcast_count.py —— 组播转发能力的**定量**测试（AP 是否放过 DDS 发现用的组播）

背景：ROS 2 默认（rmw_fastrtps_cpp）靠 UDP 组播做发现。AP 开客户端隔离 / 禁组播时，
      ping、SSH、网页全通，ROS 死活看不见对方且零报错 —— 所以必须单独量一次。

用法：
  A 机：t4_mcast_count.py recv --duration 20 --out /tmp/m.json
  B 机：t4_mcast_count.py send --count 20 --interval 0.3 --tag DRONE
（默认组 225.0.0.1:49150，与 `ros2 multicast` 一致；可用 --group/--port 覆盖）

只做原始 UDP 收发，不需要 ROS、不写任何配置。
"""
import argparse
import json
import socket
import struct
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("role", choices=["send", "recv"])
    ap.add_argument("--group", default="225.0.0.1")
    ap.add_argument("--port", type=int, default=49150)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--count", type=int, default=20)
    ap.add_argument("--interval", type=float, default=0.3)
    ap.add_argument("--tag", default=socket.gethostname())
    ap.add_argument("--iface-ip", default=None, help="在哪块网卡上收（建议显式给 WiFi IP）")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    res = {"role": a.role, "host": socket.gethostname(), "group": a.group,
           "port": a.port, "tag": a.tag}
    t0 = time.time()

    if a.role == "send":
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 4)
        if a.iface_ip:
            s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                         socket.inet_aton(a.iface_ip))
        sent = 0
        for i in range(a.count):
            body = ("DZ t4 mcast %s #%d" % (a.tag, i)).encode()
            s.sendto(body, (a.group, a.port))
            sent += 1
            res.setdefault("sent_bodies", []).append(body.decode())
            time.sleep(a.interval)
        res["sent"] = sent
        s.close()
    else:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("", a.port))
        except OSError as e:
            s.bind((a.group, a.port))
            res["bind_note"] = str(e)
        iface = socket.inet_aton(a.iface_ip) if a.iface_ip else struct.pack("=I", socket.INADDR_ANY)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                     struct.pack("=4s4s", socket.inet_aton(a.group), iface))
        s.settimeout(1.0)
        got = []
        while time.time() - t0 < a.duration:
            try:
                data, addr = s.recvfrom(2048)
            except socket.timeout:
                continue
            got.append({"from": "%s:%d" % (addr[0], addr[1]), "data": data.decode("utf-8", "replace")})
        res["received"] = len(got)
        res["senders"] = sorted({g["from"] for g in got})
        res["bodies"] = got[:30]
        res["elapsed"] = round(time.time() - t0, 2)
        s.close()

    res["t_start"] = t0
    txt = json.dumps(res, ensure_ascii=False, indent=2)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt)
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
