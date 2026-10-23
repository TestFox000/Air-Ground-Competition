#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t4_push_probes.py —— 把 t4 探针推到两台真机的 /tmp（只写 /tmp，不动任何持久配置）

用法: python dzsmartscout/tools/link/probe/t4_push_probes.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t4_ssh  # noqa: E402

FILES = [
    "t4_rosenv.sh", "t4_qos_probe.py", "t4_mcast_count.py", "t4_qos_info.py",
    "t4_car_consumer.py", "t4_car_sender.py", "t4_topicinfo.sh", "t4_car_freeze.sh",
]


def main():
    devs = sys.argv[1:] or ["drone", "car"]
    for dev in devs:
        c = t4_ssh.connect(dev)
        try:
            for f in FILES:
                src = os.path.join(HERE, f)
                dst = "/tmp/" + f
                n = t4_ssh.put(c, src, dst)
                rc, out = t4_ssh.run(c, "chmod +x %s; md5sum %s" % (dst, dst))
                print("[%s] %s <- %d bytes  %s" % (dev, dst, n, out.strip()))
        finally:
            c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
