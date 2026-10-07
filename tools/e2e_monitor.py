#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
端到端全流程测试 · 事件记录器
================================================================
以「启动时刻」为 T0，把整条链路上的关键事件打上时间戳记录下来：

    · 状态机跳变  IDLE / SCAN / TRACK     ← 视觉 + 云台联动
    · ★ 开火事件  /shoot_count 递增        ← 输出环节，带完整上下文
    · 定位 / 循迹 / 视觉 / 云台指令 心跳监测（中断即报警）

用法：
    nohup python3 -u e2e_monitor.py > ~/robot_logs/e2e_monitor.log 2>&1 &
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, Int32MultiArray

T0 = time.time()
STATE_NAME = {0: "IDLE", 1: "SCAN", 2: "TRACK"}


def ts():
    return "%8.2fs" % (time.time() - T0)


def line(tag, msg):
    print("[%s] %-10s %s" % (ts(), tag, msg), flush=True)


class E2E(Node):
    def __init__(self):
        super().__init__("e2e_monitor")

        self.state = None
        self.shots = None
        self.last_det = None          # (dx, dy, cls, bw, bh, conf)
        self.last_pos = None
        self.last_pursuit = None
        self.hb = {}                  # 话题 -> 最近一次收到的时间
        self.shoot_ctx = []

        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Int32, "/aim/status", self.on_state, 10)
        self.create_subscription(Int32, "/shoot_count", self.on_shoot, 10)
        self.create_subscription(Int32MultiArray, "/offset_center", self.on_det, be)
        self.create_subscription(Int32MultiArray, "/camemo_feedback", self.on_fb, be)
        self.create_subscription(Int32MultiArray, "/CameMo_Conl", self.on_cmd, 10)
        # 定位/循迹用 Twist(geometry_msgs) —— 只做心跳,不解析
        try:
            from geometry_msgs.msg import Twist
            self.create_subscription(Twist, "/pursuitAngle", self.on_pursuit, 10)
        except Exception:
            pass
        try:
            from nav_msgs.msg import Odometry
            self.create_subscription(Odometry, "/odom/localization", self.on_odom, 10)
        except Exception:
            pass

        # 心跳巡检：某话题超过 N 秒没数据就报警（只报一次）
        self.warned = set()
        self.create_timer(2.0, self.health)

    # ---------- 回调 ----------
    def mark(self, topic):
        self.hb[topic] = time.time()

    def on_state(self, m):
        self.mark("/aim/status")
        if m.data != self.state:
            old = STATE_NAME.get(self.state, "?")
            new = STATE_NAME.get(m.data, "?")
            line("状态机", "%s -> %s" % (old, new))
            self.state = m.data

    def on_shoot(self, m):
        self.mark("/shoot_count")
        if m.data != self.shots:
            self.shots = m.data
            d = self.last_det or (0, 0, 0, 0, 0, 0)
            cls = {0: "无目标", 1: "蓝靶", 2: "红靶"}.get(d[2], "?")
            line("★★★ 开火", "#%d  状态=%s  目标=%s conf=%d%%  "
                            "偏移=(%+d,%+d)  框=%dx%d  云台Pos=%s"
                 % (m.data, STATE_NAME.get(self.state, "?"), cls, d[5],
                    d[0], d[1], d[3], d[4], self.last_pos))
            self.shoot_ctx.append((ts(), m.data, cls, d[0], d[1], d[5]))

    def on_det(self, m):
        self.mark("/offset_center")
        d = list(m.data) + [0] * (8 - len(m.data))
        self.last_det = (d[0], d[1], d[2], d[3], d[4], d[7])

    def on_fb(self, m):
        self.mark("/camemo_feedback")
        if len(m.data) > 1:
            self.last_pos = m.data[1]

    def on_cmd(self, m):
        self.mark("/CameMo_Conl")

    def on_pursuit(self, m):
        self.mark("/pursuitAngle")

    def on_odom(self, m):
        self.mark("/odom/localization")

    # ---------- 心跳巡检 ----------
    def health(self):
        for t, desc in (("/odom/localization", "定位"), ("/pursuitAngle", "循迹"),
                        ("/offset_center", "视觉"), ("/camemo_feedback", "云台反馈"),
                        ("/aim/status", "云台状态")):
            last = self.hb.get(t)
            if last is None:
                continue
            age = time.time() - last
            if age > 5.0 and t not in self.warned:
                self.warned.add(t)
                line("⚠️ 中断", "%s (%s) 已 %.1fs 无数据" % (desc, t, age))

    # ---------- 汇总 ----------
    def summary(self):
        print("", flush=True)
        print("=" * 66, flush=True)
        print("  测试用时: %s" % ts(), flush=True)
        print("  最终弹药计数: %s" % (self.shots if self.shots is not None else "无数据"),
              flush=True)
        print("  开火明细:", flush=True)
        if not self.shoot_ctx:
            print("    （本次没有发生开火）", flush=True)
        for c in self.shoot_ctx:
            print("    %s  第 %d 发  目标=%s  偏移=(%+d,%+d)  conf=%d%%"
                  % (c[0], c[1], c[2], c[3], c[4], c[5]), flush=True)
        print("  各话题心跳:", flush=True)
        for t in ("/odom/localization", "/pursuitAngle", "/offset_center",
                  "/camemo_feedback", "/aim/status", "/CameMo_Conl"):
            l = self.hb.get(t)
            print("    %-20s %s" % (t, "有数据" if l else "❌从未收到"), flush=True)
        print("=" * 66, flush=True)


def main():
    rclpy.init()
    n = E2E()
    line("启动", "端到端监控已就绪，T0 = 现在")
    line("启动", "等待全链路数据…（定位/循迹/视觉/云台）")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.summary()
        n.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
