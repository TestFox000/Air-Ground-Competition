#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""瞄准伺服一键整定：px_per_tick 标定 + kp/kd 联动建议 + 滞后改善预估。

【为什么这两步必须一起做】
伺服每帧修正量 = `eff_px / px_per_tick`。真实 px/tick ≈ 0.94，而代码默认 2.0 时：
    每帧只修正 47% 的偏差（应 100%）⇒ 对匀速移动靶留下稳态滞后
    err = 每帧像素位移 / 修正比例 ≈ 4.27px / 0.47 ≈ 9px
再叠加 EMA(0.3) 的 ~10px 滞后 ⇒ 合计 ~19px，而开火框只有 3.5px ⇒『打不到』。
把 px_per_tick 标对后，每帧修正变 100%（死拍），这段滞后≈归零——**但要连带看 kp/kd**：
标定正确后，下发的刻度增量会变成原来的 ~2.1 倍，环路更激进，若原 kp 偏热就会振荡。

用法：
  ① 纯计算（离线，用现场量的数）：
     python aim_tune_once.py --compute --pixels 94 --ticks 100
  ② 车上实时量（需 ROS：让云台锁一个可见参照/靶，然后用遥控把云台转已知刻度）：
     python aim_tune_once.py --measure --ticks-turned 100
"""
from __future__ import annotations

import argparse

TICKS_PER_DEG = 11.378      # yaw_ticks_per_deg
PX_PER_TICK_OLD = 2.0       # 代码默认（现场现值）
KP_OLD, KD_OLD = 8.0, 3.0   # track_kp / track_kd 现值
OMEGA_DEG_S = 12.0          # 标靶匀速段角速度（判别滞后用）


def recompute(pixels: float, ticks: float, px_per_tick_old: float,
              kp_old: float, kd_old: float, omega_deg_s: float):
    p_new = pixels / ticks if ticks else float("nan")
    res = {"px_per_tick_new": p_new, "px_per_tick_old": px_per_tick_old}
    if not (p_new > 0):
        return res
    px_per_deg = p_new * TICKS_PER_DEG
    v_px_frame = omega_deg_s * px_per_deg / 30.0            # 每帧目标像素位移
    k_before = p_new / px_per_tick_old                       # 现状每帧修正比例
    err_before = v_px_frame / k_before
    err_after = v_px_frame / 1.0                             # 标定后≈死拍
    res.update({
        "px_per_deg": px_per_deg,
        "v_px_frame": v_px_frame,
        "k_before": k_before,
        "err_before_px": err_before,
        "err_after_px": err_after,
        "gain_change": px_per_tick_old / p_new,               # 环路增益变化倍数
        "kp_keep": kp_old,
        "kd_keep": kd_old,
        "kp_soft": round(kp_old * p_new / px_per_tick_old, 2),  # 若振荡→按同比例下调
        "kd_soft": round(kd_old * p_new / px_per_tick_old, 2),
    })
    return res


def report(res) -> None:
    if "px_per_tick_new" not in res or not (res["px_per_tick_new"] > 0):
        print("[ERR] px_per_tick 计算失败（刻度数需 >0）")
        return
    r = res
    print("=" * 64)
    print(f"px_per_tick：{r['px_per_tick_old']} → {r['px_per_tick_new']:.3f}"
          f"   （px/deg={r['px_per_deg']:.2f}）")
    print(f"每帧修正比例：{r['k_before']*100:.0f}% → 100%（死拍）")
    print(f"匀速靶稳态滞后：{r['err_before_px']:.1f}px → {r['err_after_px']:.1f}px"
          f"   （ω={OMEGA_DEG_S}°/s）")
    print(f"环路增益变化：×{r['gain_change']:.2f}（下发刻度增量随之变大）")
    print("-" * 64)
    print(f"kp/kd 建议：**先保持 kp={r['kp_keep']}, kd={r['kd_keep']}** "
          f"（吃掉'死拍'收益）")
    print(f"           若出现振荡/超调：下调到 kp={r['kp_soft']}, kd={r['kd_soft']}"
          f"（按同比例）")
    print("-" * 64)
    print("写入 robot.yaml：")
    print(f"  px_per_tick: {r['px_per_tick_new']:.2f}")
    print(f"  track_kp: {r['kp_keep']}      # 振荡则改 {r['kp_soft']}")
    print(f"  track_kd: {r['kd_keep']}      # 振荡则改 {r['kd_soft']}")
    print("回退：px_per_tick 改回 2.0、kp/kd 改回 8.0/3.0")


def measure_live(ticks_turned: float, px_per_tick_old: float,
                 kp_old: float, kd_old: float) -> None:
    """车上实时量：盯 camemo_feedback(刻度) 与 offset_center(dx)，回归 Δpx/Δtick。"""
    try:
        import rclpy                               # noqa: PLC0415
        from rclpy.node import Node                    # noqa: PLC0415
        from rclpy.qos import (QoSProfile, ReliabilityPolicy,   # noqa: PLC0415
                               HistoryPolicy)
        from std_msgs.msg import Int32MultiArray       # noqa: PLC0415
    except ImportError:
        print("[ERR] --measure 模式需要 ROS2(rclpy)——请在车上(source 过 ROS 环境)运行；"
              "离线机器请用 --compute 模式（量好像素后传 --pixels/--ticks）。")
        raise SystemExit(2)

    class Meas(Node):
        def __init__(self):
            super().__init__("aim_tune_once")
            q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(Int32MultiArray, "camemo_feedback", self._fb, q)
            self.create_subscription(Int32MultiArray, "offset_center", self._vis, q)
            self.p0 = None
            self.dx0 = None
            self.p = None
            self.get_logger().info(
                f"就绪：请让云台锁住一个可见靶/参照，然后用遥控把水平轴转 ≈{ticks_turned:.0f} 刻度")
            self.create_timer(0.5, self._tick)

        def _fb(self, m):
            if len(m.data) >= 2:
                self.p = int(m.data[1])            # Position_1
                if self.p0 is None:
                    self.p0 = self.p

        def _vis(self, m):
            if len(m.data) >= 3 and int(m.data[2]) in (1, 2):
                dx = float(m.data[0])
                if self.dx0 is None:
                    self.dx0 = dx
                else:
                    dp = self.p - self.p0 if self.p is not None else 0
                    if abs(dp) >= ticks_turned * 0.9:
                        px = abs(dx - self.dx0)
                        res = recompute(px, abs(dp), px_per_tick_old, kp_old, kd_old,
                                        OMEGA_DEG_S)
                        self.get_logger().info(f"测得 Δpx={px:.0f} Δtick={abs(dp)}")
                        report(res)
                        raise SystemExit(0)

        def _tick(self):
            self.get_logger().info(
                f"等待转动… 起点 tick={self.p0} dx={self.dx0} 当前 tick={self.p}",
                throttle_duration_sec=2.0)

    rclpy.init()
    node = Meas()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--compute", action="store_true", help="离线纯计算")
    ap.add_argument("--measure", action="store_true", help="车上实时测量")
    ap.add_argument("--pixels", type=float, default=0.0, help="云台转动后画面里参照移动的像素数")
    ap.add_argument("--ticks", type=float, default=100.0, help="转动的刻度数")
    ap.add_argument("--px-per-tick-old", type=float, default=PX_PER_TICK_OLD)
    ap.add_argument("--kp-old", type=float, default=KP_OLD)
    ap.add_argument("--kd-old", type=float, default=KD_OLD)
    ap.add_argument("--ticks-turned", type=float, default=100.0)
    a = ap.parse_args()

    if a.measure:
        measure_live(a.ticks_turned, a.px_per_tick_old, a.kp_old, a.kd_old)
    elif a.compute:
        if a.pixels <= 0:
            print("请给 --pixels（转动后参照在画面里移动了多少像素）")
            return
        report(recompute(a.pixels, a.ticks, a.px_per_tick_old, a.kp_old, a.kd_old,
                         OMEGA_DEG_S))
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
