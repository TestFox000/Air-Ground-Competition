#!/usr/bin/env python3
"""MID-360 read-only characterization.

Run over ssh via stdin pipe (writes nothing on the drone):
  Get-Content -Raw tools/drone/lidar_char.py | ssh drone 'source ...; python3 -'

QoS is BEST_EFFORT on purpose: a RELIABLE subscriber back-pressures the
Livox driver, and a slow rclpy consumer then drags the real 10 Hz stream
down to ~4.5 Hz.  With BEST_EFFORT the subscriber drops instead, so the
lidar keeps running at its true rate and our numbers stay honest.
Frame RATE is therefore NOT measured here -- use `ros2 topic hz` (C++)
as the authority.

Measures:
  1. points per frame        (expect 200000 pts/s / 10 Hz = 20000)
  2. invalid (0,0,0) fraction
  3. range percentiles, valid points only
  4. azimuth / elevation coverage  (expect 360 deg / -7..+52 deg)
  5. reflectivity distribution
  6. azimuth 36-bin uniformity
  7. densest 2x2 deg cell -> in-situ range noise proxy
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from livox_ros_driver2.msg import CustomMsg

NFRAMES = 8


class Char(Node):
    def __init__(self):
        super().__init__('dz_lidar_char')
        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(CustomMsg, '/livox/lidar', self.cb, qos)
        self.stamps = []
        self.counts = []
        self.frames = []
        self.done = False

    def cb(self, msg):
        self.stamps.append(self.get_clock().now().nanoseconds * 1e-9)
        self.counts.append(int(msg.point_num))
        if msg.point_num > 0:
            a = np.array([[p.x, p.y, p.z, p.reflectivity]
                          for p in msg.points], dtype=np.float64)
        else:
            a = np.zeros((0, 4))
        self.frames.append(a)
        if len(self.frames) >= NFRAMES:
            self.done = True

    def report(self):
        n = len(self.frames)
        print('RECEIVED_FRAMES %d' % n)
        if n == 0:
            print('NO_DATA: QoS mismatch or publisher absent')
            return
        c = np.array(self.counts)
        print('PTS_PER_FRAME min=%d mean=%.1f max=%d'
              % (c.min(), c.mean(), c.max()))

        allp = np.vstack([f for f in self.frames if len(f)])
        xyz = allp[:, :3]
        r = np.linalg.norm(xyz, axis=1)
        nz = r > 1e-6
        print('INVALID_ZERO_POINTS %d / %d = %.2f %%'
              % (int((~nz).sum()), len(r), 100.0 * (~nz).sum() / len(r)))

        rv = r[nz]
        for q in (1, 5, 25, 50, 75, 90, 99, 99.9):
            print('RANGE_VALID p%-5g = %.3f m' % (q, np.percentile(rv, q)))
        print('RANGE_VALID min=%.3f max=%.3f mean=%.3f'
              % (rv.min(), rv.max(), rv.mean()))

        v = xyz[nz]
        az = np.degrees(np.arctan2(v[:, 1], v[:, 0]))
        hor = np.hypot(v[:, 0], v[:, 1])
        el = np.degrees(np.arctan2(v[:, 2], hor))
        print('AZ  min=%.1f max=%.1f' % (az.min(), az.max()))
        print('EL  min=%.1f max=%.1f  (datasheet -7 .. +52)'
              % (el.min(), el.max()))

        refl = allp[nz, 3]
        print('REFLECT p10=%.1f p50=%.1f p90=%.1f max=%.1f'
              % tuple(np.percentile(refl, [10, 50, 90, 100])))

        h, _ = np.histogram(az, bins=36, range=(-180.0, 180.0))
        print('AZ36BIN min=%d max=%d mean=%.0f ratio=%.1fx'
              % (h.min(), h.max(), h.mean(), h.max() / max(1, h.min())))

        he, _ = np.histogram(el, bins=12, range=(-7.0, 53.0))
        print('EL12BIN -7..53 step5deg: ' + ' '.join('%d' % x for x in he))

        ai = np.clip(((az + 180.0) // 2.0).astype(np.int64), 0, 179)
        ei = np.clip(((el + 90.0) // 2.0).astype(np.int64), 0, 89)
        key = ai * 1000 + ei
        uniq, inv, cnt = np.unique(key, return_inverse=True,
                                   return_counts=True)
        # skip the (0-ish) origin cluster so the noise proxy is a real surface
        order = np.argsort(cnt)[::-1]
        for k in order[:3]:
            sel = rv[inv == k]
            if len(sel) > 20:
                print('DENSE_CELL az_idx=%d el_idx=%d n=%d range_mean=%.3f '
                      'range_std=%.4f m'
                      % (uniq[k] // 1000, uniq[k] % 1000, len(sel),
                         sel.mean(), sel.std()))
        print('CELLS_HIT %d / 16200 potential 2deg cells' % len(uniq))


def main():
    rclpy.init()
    node = Char()
    t_end = time.time() + 40.0
    while rclpy.ok() and not node.done and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not node.done:
        print('TIMEOUT, frames=%d' % len(node.frames))
    node.report()
    try:
        node.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
