# -*- coding: utf-8 -*-
"""彩排监听器：用车端同一个 intel_protocol.parse 校验 sim_match 广播的 UDP 帧。"""
import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground",
                               "src", "ag_integrate"))
from ag_integrate.intel_protocol import parse, parse_car_pose  # 车端解析器

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", 50000))
sock.settimeout(90)
names = {0: "mine", 4: "cell_label"}
n_intel = n_pose = 0
t0 = time.time()
try:
    while time.time() - t0 < 90:
        data, addr = sock.recvfrom(4096)
        try:
            frame = parse(data)
            n_intel += 1
            mines = [o for o in frame.objects if o.type == 0]
            print(f"[0x4001] {len(frame.objects)} 目标 (mine={len(mines)}), "
                  f"首个雷=({mines[0].x:.2f},{mines[0].y:.2f})" if mines
                  else f"[0x4001] {len(frame.objects)} 目标", flush=True)
        except ValueError:
            frame = parse_car_pose(data)
            n_pose += 1
            print(f"[0x4002] 车位姿 x={frame.x:.2f} y={frame.y:.2f} "
                  f"yaw={frame.yaw:.3f}", flush=True)
        if n_intel >= 3:
            break
except socket.timeout:
    print("TIMEOUT: 90 秒未收到帧", flush=True)
print(f"RESULT: intel帧={n_intel} pose帧={n_pose}", flush=True)
