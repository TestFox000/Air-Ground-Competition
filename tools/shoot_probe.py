#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
射击间隔探测   shoot_probe.py
================================================================================
用途：摸清【裁判系统硬件】到底允许多快的射击间隔。

背景：
  · 代码里我们只控制软件侧的冷却（shot_cooldown_ms=500ms、遥控器 0.8s），
    但真正"这一发算不算数"是裁判板决定的 —— 打太快它很可能直接丢弃。
  · 硬件会把结果回传：HpAndHitmsg = [敌HP, 我HP, 被击打, 累计射击数, 剩余弹药]
    所以判据很直接：**发出一发后，累计射击数 +1 / 剩余弹药 -1 = 这一发被接受**。
  · 本脚本按"由慢到快"逐档试探，找出硬件实际接受的最小间隔。

★ 严重警告 —— 会真的消耗弹药！
  每一发都会扣掉硬件的 10 发配额（用完需裁判系统侧复位）。
  所以：
    · 默认 --n 1（每档只打 1 发），总消耗 = 档位数 × n，先用 --dry-run 看会打几发
    · 必须显式加 --confirm 才会真的发射
    · 建议先用大间隔档位，确认链路通了再往下探

用法：
  # 先看要花多少弹药（不发射）
  python3 tools/shoot_probe.py --dry-run

  # 实探：从 2000ms 一直试到 200ms，每档 1 发（共约 6 发）
  python3 tools/shoot_probe.py --intervals 2000,1000,600,400,300,200 --confirm

  # 只想知道某一档能不能打
  python3 tools/shoot_probe.py --intervals 500 --n 2 --confirm
================================================================================
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, UInt8MultiArray

# 与 judgment.cpp 中 case 0x07 的字段顺序一致
IDX_SHOTS = 3   # 累计射击次数（硬件侧）
IDX_AMMO = 4    # 剩余弹药


class Probe(Node):
    def __init__(self):
        super().__init__('shoot_probe')
        self.state = {'shots': None, 'ammo': None}
        self.last_ts = 0.0
        self.create_subscription(UInt8MultiArray, '/HpAndHitmsg', self._on_ref, 10)
        self.pub = self.create_publisher(UInt8, '/LaserShot_Command', 10)

    def _on_ref(self, msg):
        d = list(msg.data)
        if len(d) >= 5:
            self.state['shots'] = d[IDX_SHOTS]
            self.state['ammo'] = d[IDX_AMMO]
            self.last_ts = time.time()

    def wait_refresh(self, timeout=3.0):
        """等一帧新的裁判上报（1Hz 上报，所以最多要等 ~1.2s）"""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - self.last_ts < 0.8 and self.state['shots'] is not None:
                return True
        return self.state['shots'] is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--intervals', default='2000,1000,600,400,300,200',
                    help='要试探的间隔(毫秒)，由慢到快')
    ap.add_argument('--n', type=int, default=1, help='每个间隔打几发')
    ap.add_argument('--confirm', action='store_true', help='真的发射（不加则只预览）')
    ap.add_argument('--dry-run', action='store_true', help='只打印计划，不发射')
    args = ap.parse_args()

    intervals = [int(x) for x in args.intervals.split(',') if x.strip()]
    total = len(intervals) * args.n

    print('=' * 62)
    print('  射击间隔探测')
    print('=' * 62)
    print('  档位(ms): %s' % ', '.join(str(i) for i in intervals))
    print('  每档发数: %d' % args.n)
    print('  ★ 预计消耗弹药: %d 发' % total)
    print('  模式: %s' % ('实弹' if args.confirm else '预览（未加 --confirm，不会发射）'))
    print('=' * 62)

    if args.dry_run or not args.confirm:
        print('\n未发射。确认无误后加 --confirm 再跑。')
        return 0

    rclpy.init()
    node = Probe()
    try:
        print('\n等待裁判上报…')
        if not node.wait_refresh(timeout=5.0):
            print('❌ 收不到 /HpAndHitmsg —— 裁判系统未开启或未连接，先开启再来。')
            return 1
        base_shots = node.state['shots']
        base_ammo = node.state['ammo']
        print('  初始: 累计射击=%s  剩余弹药=%s' % (base_shots, base_ammo))
        if base_ammo == 0:
            print('⚠️  剩余弹药为 0 —— 硬件已打完，软件重置无效，需裁判系统侧复位后再测。')
            return 1

        print('\n%-8s %-6s %-10s %-10s %s' % ('间隔ms', '发', '累计射击', '剩余弹药', '判定'))
        print('-' * 62)
        for gap in intervals:
            for k in range(args.n):
                before_s = node.state['shots']
                node.pub.publish(UInt8(data=1))
                time.sleep(gap / 1000.0)
                # 等裁判把新状态报上来
                node.wait_refresh(timeout=3.0)
                after_s = node.state['shots']
                after_a = node.state['ammo']
                accepted = (after_s is not None and before_s is not None
                            and after_s > before_s)
                print('%-8d %-6d %-10s %-10s %s'
                      % (gap, k + 1, after_s, after_a,
                         '✅ 接受' if accepted else '❌ 被丢弃(太快)'))
                sys.stdout.flush()
                if node.state['ammo'] == 0:
                    print('\n⚠️ 弹药已耗尽，停止。')
                    break
            if node.state['ammo'] == 0:
                break
        print('-' * 62)
        print('\n判读：最后一个"✅ 接受"的间隔，就是硬件实际允许的最小间隔附近。')
        print('      （真实阈值在该档位与下一个"❌"档位之间）')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
