#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dzlink_livox_bridge —— 无人机侧「点云标准化 + 序号信标」桥接节点

目标：让小车在不安装 livox_ros_driver2 的前提下消费点云，并提供可做
      逐条丢包对账的单调序号流。

发布：
  /dzlink/points  sensor_msgs/msg/PointCloud2   (默认 10 Hz, 字段 x,y,z,intensity float32)
  /dzlink/seq     std_msgs/msg/Int64            (默认 50 Hz, 单调递增序号)

订阅：
  /livox/lidar    livox_ros_driver2/msg/CustomMsg

启动示例：
  source /opt/ros/humble/setup.bash
  source ~/dzpx4/install/setup.bash
  python3 dzlink_livox_bridge.py --ros-args -p seq_rate:=50.0
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Int64
from livox_ros_driver2.msg import CustomMsg

try:
    import numpy as np
    HAVE_NP = True
except Exception:                                    # pragma: no cover
    np = None
    HAVE_NP = False

import struct

# PointCloud2 字段定义（与 pcl/PointXYZI 布局一致）
_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]
_POINT_STEP = 16


class LivoxBridge(Node):
    def __init__(self):
        super().__init__('dzlink_livox_bridge')

        self.declare_parameter('seq_rate', 50.0)
        self.declare_parameter('points_topic', '/dzlink/points')
        self.declare_parameter('seq_topic', '/dzlink/seq')
        self.declare_parameter('in_topic', '/livox/lidar')

        seq_rate = float(self.get_parameter('seq_rate').value)
        points_topic = self.get_parameter('points_topic').value
        seq_topic = self.get_parameter('seq_topic').value
        in_topic = self.get_parameter('in_topic').value

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)

        self.pub_pc = self.create_publisher(PointCloud2, points_topic, qos)
        self.pub_seq = self.create_publisher(Int64, seq_topic, qos)
        self.sub = self.create_subscription(CustomMsg, in_topic, self.on_cloud, qos)

        self.seq = 0
        self.timer = self.create_timer(1.0 / seq_rate, self.on_seq)

        # 统计
        self.frames = 0
        self.pts_total = 0
        self.t0 = time.time()
        self.last_log = self.t0
        self.last_frames = 0
        self.hz_samples = []
        self.last_frame_t = None

        self.get_logger().info(
            'bridge up: %s(CustomMsg) -> %s(PointCloud2), seq=%s@%.1fHz, numpy=%s'
            % (in_topic, points_topic, seq_topic, seq_rate, HAVE_NP))

    # ---------- 序号信标 ----------
    def on_seq(self):
        m = Int64()
        m.data = self.seq
        self.pub_seq.publish(m)
        self.seq += 1

    # ---------- 点云转换 ----------
    def on_cloud(self, msg):
        n = len(msg.points)

        if HAVE_NP:
            xs = np.fromiter((p.x for p in msg.points), dtype=np.float32, count=n)
            ys = np.fromiter((p.y for p in msg.points), dtype=np.float32, count=n)
            zs = np.fromiter((p.z for p in msg.points), dtype=np.float32, count=n)
            rs = np.fromiter((p.reflectivity for p in msg.points), dtype=np.float32, count=n)
            arr = np.empty((n, 4), dtype=np.float32)
            arr[:, 0] = xs
            arr[:, 1] = ys
            arr[:, 2] = zs
            arr[:, 3] = rs
            data = arr.tobytes()
        else:
            buf = bytearray(n * _POINT_STEP)
            off = 0
            for p in msg.points:
                struct.pack_into('<ffff', buf, off, p.x, p.y, p.z, float(p.reflectivity))
                off += _POINT_STEP
            data = bytes(buf)

        out = PointCloud2()
        out.header = msg.header                 # 保留驱动时间戳与 frame_id(livox_frame)
        out.height = 1
        out.width = n
        out.fields = _FIELDS
        out.is_bigendian = False
        out.point_step = _POINT_STEP
        out.row_step = _POINT_STEP * n
        out.data = data
        out.is_dense = False                    # MID-360 存在无效点，勿声明稠密
        self.pub_pc.publish(out)

        # 统计
        now = time.time()
        self.frames += 1
        self.pts_total += n
        if self.last_frame_t is not None:
            dt = now - self.last_frame_t
            if dt > 0:
                self.hz_samples.append(1.0 / dt)
                if len(self.hz_samples) > 200:
                    self.hz_samples.pop(0)
        self.last_frame_t = now

        if now - self.last_log >= 5.0:
            dur = now - self.last_log
            fps = (self.frames - self.last_frames) / dur
            hz = sum(self.hz_samples) / len(self.hz_samples) if self.hz_samples else 0.0
            # subs_* = DDS 已匹配的订阅者数量（>0 即证明反向发现成立，见长时测试判据）
            self.get_logger().info(
                'STAT frames=%d total=%d pts_last=%d pub_hz=%.2f avg_dt_hz=%.2f bytes=%d '
                'seq=%d subs_points=%d subs_seq=%d'
                % (self.frames, self.pts_total, n, fps, hz, len(data), self.seq,
                   self.pub_pc.get_subscription_count(), self.pub_seq.get_subscription_count()))
            self.last_log = now
            self.last_frames = self.frames


def main():
    rclpy.init()
    node = LivoxBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
