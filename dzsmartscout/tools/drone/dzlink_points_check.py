#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dzlink_points_check —— 无人机侧（或车侧）点云消费自检
验证 /dzlink/points 真的能反序列化、点数字段正确、首点有真实数值；
并核对 /dzlink/seq 的到达情况。

用法: python3 dzlink_points_check.py [观测秒数，默认 10]
"""
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int64


class Check(Node):
    def __init__(self):
        super().__init__('dzlink_points_check')
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, '/dzlink/points', self.cb_pc, qos)
        self.create_subscription(Int64, '/dzlink/seq', self.cb_seq, qos)
        self.frames = 0
        self.first = None
        self.seq_recv = 0
        self.seq_min = None
        self.seq_max = None
        self.t0 = time.time()

    def cb_pc(self, m):
        self.frames += 1
        if self.first is None:
            raw = bytes(m.data)
            x, y, z, i = struct.unpack_from('<ffff', raw, 0)
            # 统计非零点数（确认不是空壳点云）
            n = m.width * m.height
            nz = 0
            step = m.point_step
            for k in range(0, min(n, 2000)):
                px, py, pz, _ = struct.unpack_from('<ffff', raw, k * step)
                if px != 0.0 or py != 0.0 or pz != 0.0:
                    nz += 1
            self.first = dict(width=m.width, height=m.height, point_step=m.point_step,
                              row_step=m.row_step, frame_id=m.header.frame_id,
                              stamp=f'{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}',
                              bytes=len(raw), x=x, y=y, z=z, intensity=i,
                              nonzero_in_2000=nz)

    def cb_seq(self, m):
        self.seq_recv += 1
        v = int(m.data)
        self.seq_min = v if self.seq_min is None else min(self.seq_min, v)
        self.seq_max = v if self.seq_max is None else max(self.seq_max, v)


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = Check()
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.1)
    el = time.time() - t0

    print('=== /dzlink/points ===')
    if node.first:
        f = node.first
        print(f'  frames={node.frames}  hz={node.frames/el:.2f}')
        print(f'  width={f["width"]} height={f["height"]} point_step={f["point_step"]} '
              f'row_step={f["row_step"]} bytes={f["bytes"]}')
        print(f'  frame_id={f["frame_id"]} stamp={f["stamp"]}')
        print(f'  first_point x={f["x"]:.4f} y={f["y"]:.4f} z={f["z"]:.4f} '
              f'intensity={f["intensity"]:.1f}')
        print(f'  前 2000 点中非零点数={f["nonzero_in_2000"]}  → {"真实点云" if f["nonzero_in_2000"]>0 else "可疑空壳"}')
    else:
        print('  ❌ 未收到任何 PointCloud2')

    print('=== /dzlink/seq ===')
    if node.seq_recv:
        span = (node.seq_max - node.seq_min + 1) if node.seq_max is not None else 0
        loss = (1.0 - node.seq_recv / span) * 100.0 if span > 0 else float('nan')
        print(f'  recv={node.seq_recv} min={node.seq_min} max={node.seq_max} span={span}')
        print(f'  hz={node.seq_recv/el:.2f}  丢包率={loss:.2f}%')
    else:
        print('  ❌ 未收到任何 Int64')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
