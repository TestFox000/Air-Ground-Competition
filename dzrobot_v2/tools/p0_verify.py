#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
P0 四项专项验证器
================================================================
实时记录并判定：
  P0-1 云台扫描范围钳位  —— 记录 yaw 刻度 min/max，判定是否超出 [lo, hi]
  P0-2 单目标连发上限    —— 判定同一目标是否连发超过 N 发
  P0-3 越界急停          —— 监控信念位姿是否出地图
  P0-4 仅打蓝靶          —— 记录每次开火时的目标类别

用法:  nohup python3 -u p0_verify.py > ~/robot_logs/p0_verify.log 2>&1 &
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, Int32MultiArray

T0 = time.time()

# 扫描范围（中心 2047，±90° × 11.378 刻度/度）
YAW_LO, YAW_HI = 1023, 3071
PER_TARGET_MAX = 3          # 与 max_shots_per_target 一致
# 地图边界（与 path_executor 的 MAP_* 一致）
MAP_X = (0.175, 3.675)
MAP_Y = (0.175, 4.725)
MARGIN = 0.60


def ts():
    return "%7.1fs" % (time.time() - T0)


def line(tag, msg):
    print("[%s] %-12s %s" % (ts(), tag, msg), flush=True)


class P0(Node):
    def __init__(self):
        super().__init__('p0_verify')
        self.pos_min, self.pos_max = 99999, -99999
        self.overflow = 0
        self.shots = None
        self.last_cls = None
        self.chain = 0
        self.chain_max = 0
        self.fired = []          # (时刻, 序号, 类别, dx, dy)
        self.oob = 0
        self.hb = {}
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Int32, '/shoot_count', self.on_shot, 10)
        self.create_subscription(Int32MultiArray, '/camemo_feedback', self.on_fb, be)
        self.create_subscription(Int32MultiArray, '/offset_center', self.on_det, 10)
        try:
            from nav_msgs.msg import Odometry
            self.create_subscription(Odometry, '/odom/localization', self.on_odom, 10)
        except Exception:
            pass

    def on_fb(self, m):
        self.hb['fb'] = time.time()
        if len(m.data) > 1:
            p = m.data[1]
            self.pos_min = min(self.pos_min, p)
            self.pos_max = max(self.pos_max, p)
            if p < YAW_LO - 5 or p > YAW_HI + 5:
                self.overflow += 1

    def on_det(self, m):
        d = list(m.data)
        if d[2] != 0:
            self.last_cls = d[2]
            self._det = (d[0], d[1])

    def on_odom(self, m):
        p = m.pose.pose.position
        if not (MAP_X[0] - MARGIN <= p.x <= MAP_X[1] + MARGIN
                and MAP_Y[0] - MARGIN <= p.y <= MAP_Y[1] + MARGIN):
            self.oob += 1

    def on_shot(self, m):
        if m.data == self.shots:
            return
        self.shots = m.data
        cls = {0: '无目标', 1: '蓝靶', 2: '红靶'}.get(self.last_cls or 0, '?')
        d = getattr(self, '_det', (0, 0))
        # P0-2：同一类别连续开火计数（模拟"同一目标连发"）
        if cls == self.last_shot_cls:
            self.chain += 1
        else:
            if self.chain > self.chain_max:
                self.chain_max = self.chain
            self.chain = 1
            self.last_shot_cls = cls
        self.chain_max = max(self.chain_max, self.chain)
        line("★开火 #%d" % m.data, "目标=%s conf 偏移=(%+d,%+d) 同目标连发第 %d 发"
             % (cls, d[0], d[1], self.chain))
        self.fired.append((ts(), m.data, cls, d[0], d[1]))

    last_shot_cls = None

    def summary(self):
        print("", flush=True)
        print("=" * 68, flush=True)
        print("  P0 四项验证结果  (用时 %s)" % ts(), flush=True)
        print("-" * 68, flush=True)
        ok1 = self.overflow == 0
        print("  P0-1 扫描范围钳位 : yaw 范围 %d~%d  越界帧 %d  %s"
              % (self.pos_min, self.pos_max, self.overflow, "✅通过" if ok1 else "❌越界"),
              flush=True)
        ok2 = self.chain_max <= PER_TARGET_MAX
        print("  P0-2 单目标连发   : 最大连发 %d 发 (上限 %d)  %s"
              % (self.chain_max, PER_TARGET_MAX, "✅通过" if ok2 else "❌超限"), flush=True)
        ok3 = self.oob == 0
        print("  P0-3 越界监控     : 出界帧 %d  %s"
              % (self.oob, "✅全程在地图内" if ok3 else "⚠️曾出界(急停应已触发)"), flush=True)
        reds = [f for f in self.fired if f[2] == '红靶']
        ok4 = len(reds) == 0
        print("  P0-4 仅打蓝靶     : 总开火 %d 发，其中红靶 %d 发  %s"
              % (len(self.fired), len(reds), "✅通过" if ok4 else "❌打了红靶"), flush=True)
        print("-" * 68, flush=True)
        print("  总判定: %s" % ("🎉 P0 四项全部通过" if (ok1 and ok2 and ok3 and ok4)
                                else "⚠️ 存在未通过项，见上"), flush=True)
        print("=" * 68, flush=True)


def main():
    rclpy.init()
    n = P0()
    line("启动", "P0 验证器就绪（云台范围 %d~%d，单目标上限 %d 发）"
         % (YAW_LO, YAW_HI, PER_TARGET_MAX))
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.summary()
        n.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
