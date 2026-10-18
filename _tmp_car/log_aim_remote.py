#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Aim/PID 实车录制探针：把 offset_center / CameMo_Conl / aim_status 写成 CSV。

用法: python3 log_aim.py <out.csv> <seconds>
字段: t(秒,相对起点), kind(off/cmd/st), a..e
  off : a=dx(px) b=dy(px) c=cls(1蓝 2红 0无) d=框宽 e=框高
  cmd : a=Position_0(pitch) b=Position_1(yaw) c=Speed_0 d=Speed_1
  st  : a=状态(0 IDLE / 1 SCAN / 2 TRACK)
"""
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32


class Probe(Node):
    def __init__(self, path):
        super().__init__('aim_probe')
        self.t0 = time.time()
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'kind', 'a', 'b', 'c', 'd', 'e'])
        self.create_subscription(Int32MultiArray, '/offset_center', self.cb_off, 20)
        self.create_subscription(Int32MultiArray, '/CameMo_Conl', self.cb_cmd, 50)
        self.create_subscription(Int32MultiArray, '/camemo_feedback', self.cb_fb, 50)
        self.create_subscription(Int32, '/aim/status', self.cb_st, 20)

    def _row(self, kind, vals):
        v = [int(x) for x in vals]
        v = (v + [''] * 5)[:5]
        self.w.writerow(['%.4f' % (time.time() - self.t0), kind] + v)

    def cb_off(self, m):
        self._row('off', list(m.data[:5]))

    def cb_cmd(self, m):
        self._row('cmd', list(m.data[:4]))

    def cb_fb(self, m):
        # camemo_feedback: 下位机回传，前两位是实际位置(ticks)，3/4 位是实际速度
        self._row('fb', list(m.data[:4]))

    def cb_st(self, m):
        self._row('st', [m.data])


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/aim_pid.csv'
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    rclpy.init()
    n = Probe(out)
    end = time.time() + dur
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.f.close()
    print('done', out, dur)


if __name__ == '__main__':
    main()
