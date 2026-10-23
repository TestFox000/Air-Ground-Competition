#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pc2_check —— 标准 PointCloud2 消费自检（无人机侧/车侧通用）

用法: python3 pc2_check.py [话题, 默认 /livox/lidar] [观测秒数, 默认 8]
输出: 帧数/频率、width/height/point_step、字段表、frame_id、首点数值、非零点比例
"""
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2


class Check(Node):
    def __init__(self, topic):
        super().__init__('pc2_check')
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, topic, self.cb, qos)
        self.frames = 0
        self.info = None
        self.widths = {}

    def cb(self, m):
        self.frames += 1
        self.widths[m.width] = self.widths.get(m.width, 0) + 1
        if self.info is None:
            raw = bytes(m.data)
            fields = [(f.name, f.offset, f.datatype) for f in m.fields]
            n = m.width * m.height
            # 按字段偏移解析首点
            def get_f32(off):
                return struct.unpack_from('<f', raw, off)[0]
            off = {name: o for name, o, _ in fields}
            first = {k: get_f32(v) for k, v in off.items() if k in ('x', 'y', 'z', 'intensity', 'reflectivity')}
            nz = 0
            for k in range(min(n, 3000)):
                if get_f32(off.get('x', 0) + k * m.point_step) != 0.0:
                    nz += 1
            self.info = dict(width=m.width, height=m.height, point_step=m.point_step,
                             row_step=m.row_step, fields=fields, frame_id=m.header.frame_id,
                             stamp=f'{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}',
                             data_bytes=len(raw), first=first, nz_in_3000=nz)


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/livox/lidar'
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
    rclpy.init()
    node = Check(topic)
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.1)
    el = time.time() - t0

    print(f'=== {topic} ===')
    if node.info:
        i = node.info
        print(f'  frames={node.frames}  hz={node.frames/el:.2f}')
        print(f'  width={i["width"]} height={i["height"]} point_step={i["point_step"]} '
              f'row_step={i["row_step"]} data={i["data_bytes"]}B')
        print(f'  frame_id={i["frame_id"]} stamp={i["stamp"]}')
        print(f'  fields={i["fields"]}')
        print(f'  first_point={ {k: round(v,4) for k,v in i["first"].items()} }')
        print(f'  前 3000 点非零(x!=0)={i["nz_in_3000"]}  → '
              f'{"真实点云" if i["nz_in_3000"] > 0 else "可疑空壳"}')
        print(f'  宽度分布(前几帧)={dict(list(node.widths.items())[:3])}')
    else:
        print('  ❌ 未收到任何 PointCloud2（类型或 QoS 不匹配？）')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
