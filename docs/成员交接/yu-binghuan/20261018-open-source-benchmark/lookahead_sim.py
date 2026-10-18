# -*- coding: utf-8 -*-
"""G2 速度自适应前瞻仿真：固定 AIM_DRIVE vs clamp(k·v)（NAV2 RPP 范式）。

吸收来源：Nav2 Regulated Pure Pursuit（前瞻 ld = clamp(k·v, min, max)，
arXiv:2305.20026）+ ETH-PBL MAP-Controller。落点：path_executor.py:187
固定 AIM_DRIVE=0.35。

仿真忠实复刻 path_executor 实码控制律（20Hz 单层 belief 模型）：
  drive: aim = a + min(t·leg_len + LD, leg_len)/leg_len·(b-a)
         PP steer = deg(atan(2·L·sin α / max(dist,0.15)))，限 ±30°
         v = base·(1-0.6|steer|/30)，下限 0.08
         steer += CtePidController.update(cte, 0.05)（出厂 kp115 ki50 kd100）
         cmd = steer + STEER_TRIM(-1°)；物理：psi_dot = v/L·tan(cmd+BIAS(+2°))
  turn : dist_to_b ≤ r_corner 进弧（r_corner=min(R_TURN,0.9·l1,0.9·l2)）
         v=0.14 满舵 30°，yaw 闭环停（lead 6°）→ 出口 snap：
         位置 → 拐点 + r_corner·u_out，yaw → 出向 − sign·(6°±2° 回滞)，
         并叠加 ±2cm 横向位置扰动（弧跟踪残差下限）
  ramp : 拐点前 0.6m 内 base = 0.14 + ramp·(0.32−0.14)

激励场景：
  S1 弯道出口：90° 右弯 + 90° 左弯 S 形路线（直段 3 格=1.05m）
  S2 直段横向注入：10cm 初始偏差（同 cte_pid_sim.py）

对比策略：
  A 现状   AIM_DRIVE = 0.35 固定
  B RPP    LD = clamp(1.1·v, 0.15, 0.35)   —— 直道 v≈0.32 → 0.352≈现状，
                                            低速段前瞻自动收紧
  C 激进   LD = clamp(0.6·v, 0.10, 0.30)
"""
from __future__ import annotations

import json
import math
import random
import statistics
import sys
from pathlib import Path

PKG = Path(r"c:\Users\29212\Documents\Trae-CN\default\fxxy_extract\发学员"
           r"\src(走直角)\car_localization\car_localization")
sys.path.insert(0, str(PKG))
from cte_pid import CtePidController  # noqa: E402

# ---- 实码常量（path_executor.py） ---------------------------------------
L = 0.17                 # WHEELBASE
MAX_STEER = 30.0         # MAX_STEER_DEG（servo limit）
R_TURN = L / math.tan(math.radians(MAX_STEER))     # 0.2944m
V_STRAIGHT, V_TURN = 0.32, 0.14
STEER_TRIM, BIAS = -1.0, 2.0      # deg：静态右修 / 恒定机械左偏
LEAD_DEG = 6.0                    # TURN_YAW_LEAD_DEG
RAMP_M = 0.6                      # 拐点前减速带
DT = 0.05                         # 20Hz
CELL = 0.35

STRATS = {
    "A_固定0.35": lambda v: 0.35,
    "B_RPP(k1.1)": lambda v: max(0.15, min(0.35, 1.1 * v)),
    "C_激进(k0.6)": lambda v: max(0.10, min(0.30, 0.6 * v)),
}


class Sim:
    """单层 belief 仿真：复刻 drive/turn 两模式。"""

    def __init__(self, wps, ld_fn, rng):
        self.wps = wps
        self.ld_fn = ld_fn
        self.rng = rng
        self.x, self.y = wps[0]
        a, b = wps[0], wps[1]
        self.yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        self.seg = 0
        self.mode = "drive"
        self.cte_ctrl = CtePidController()
        self._cte_seg = -1
        self.hist = []
        # turn 状态
        self.turn_sign = 0.0
        self.turn_target = 0.0
        self.turn_start_yaw = 0.0
        self.turn_r = 0.0

    # -- drive 段几何 ---------------------------------------------------
    def _leg(self):
        a, b = self.wps[self.seg], self.wps[self.seg + 1]
        return a, b, math.hypot(b[0] - a[0], b[1] - a[1])

    def _project(self, a, b):
        t = ((self.x - a[0]) * (b[0] - a[0])
             + (self.y - a[1]) * (b[1] - a[1])) / max(
            ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2), 1e-9)
        return min(1.0, max(0.0, t))

    def step(self):
        if self.mode == "turn":
            self._step_turn()
            return
        a, b, leg_len = self._leg()
        t = self._project(a, b)
        dist_to_b = (1.0 - t) * leg_len
        # CTE（+ = 左侧，同实码口径）
        cte = ((self.x - a[0]) * -(b[1] - a[1])
               + (self.y - a[1]) * (b[0] - a[0])) / leg_len
        # base_speed（拐点前 ramp）
        base = V_STRAIGHT
        has_corner = self.seg < len(self.wps) - 2
        if has_corner:
            r = self._corner_radius()
            if dist_to_b <= r:
                self._start_turn(r)
                return
            d_ts = dist_to_b - r
            if d_ts < RAMP_M:
                ramp = max(0.0, d_ts / RAMP_M)
                base = V_TURN + ramp * (V_STRAIGHT - V_TURN)
        # aim point（前瞻 = 策略函数）
        ld = self.ld_fn(base)
        s_ahead = min(t * leg_len + ld, leg_len)
        frac = s_ahead / leg_len
        aim_x = a[0] + frac * (b[0] - a[0])
        aim_y = a[1] + frac * (b[1] - a[1])
        steer, v = self._pure_pursuit(aim_x, aim_y, base)
        # CTE-PID 叠加（新航段复位）
        if self.seg != self._cte_seg:
            self.cte_ctrl.reset()
            self._cte_seg = self.seg
        steer += self.cte_ctrl.update(cte, DT)
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        self._advance(v, steer)
        self.hist.append((self.x, self.y, self.yaw, cte, steer, v))

    def _pure_pursuit(self, aim_x, aim_y, base):
        dx, dy = aim_x - self.x, aim_y - self.y
        dist = math.hypot(dx, dy)
        if dist < 0.01:
            return 0.0, 0.0
        rel_x = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
        rel_y = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
        alpha = math.atan2(rel_y, rel_x)
        if abs(alpha) > 1.75:
            side = 1.0 if rel_y > 0.02 else (-1.0 if rel_y < -0.02 else 0.0)
            return MAX_STEER * (side or 1.0), 0.10
        steer = math.degrees(
            math.atan(2.0 * L * math.sin(alpha) / max(dist, 0.15)))
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        v = base * (1.0 - 0.6 * abs(steer) / MAX_STEER)
        return steer, max(0.08, v)

    # -- turn 段（满舵弧 + yaw 闭环 + snap） ----------------------------
    def _corner_radius(self):
        i = self.seg
        a, b, c = self.wps[i], self.wps[i + 1], self.wps[i + 2]
        l1 = math.hypot(b[0] - a[0], b[1] - a[1])
        l2 = math.hypot(c[0] - b[0], c[1] - b[1])
        return max(0.10, min(R_TURN, 0.9 * l1, 0.9 * l2))

    def _start_turn(self, r):
        a, b, c = (self.wps[self.seg], self.wps[self.seg + 1],
                   self.wps[self.seg + 2])
        d1 = (b[0] - a[0], b[1] - a[1])
        d2 = (c[0] - b[0], c[1] - b[1])
        ang = math.atan2(d1[0] * d2[1] - d1[1] * d2[0],
                         d1[0] * d2[0] + d1[1] * d2[1])
        if abs(ang) < 0.15:
            # 实码直通分支：近共线拐点不进弧
            self.seg += 1
            return
        self.mode = "turn"
        self.turn_sign = 1.0 if ang > 0 else -1.0
        self.turn_target = abs(ang)
        self.turn_start_yaw = self.yaw
        self.turn_r = r

    def _step_turn(self):
        self._advance(V_TURN, self.turn_sign * MAX_STEER)
        rot = self.turn_sign * _norm(self.yaw - self.turn_start_yaw)
        if rot >= self.turn_target - math.radians(LEAD_DEG):
            b, c = self.wps[self.seg + 1], self.wps[self.seg + 2]
            l2 = math.hypot(c[0] - b[0], c[1] - b[1])
            # snap：位置 → 拐点 + r·出向；yaw → 出向 − lead − 回滞
            self.x = b[0] + self.turn_r * (c[0] - b[0]) / l2
            self.y = b[1] + self.turn_r * (c[1] - b[1]) / l2
            exit_yaw = math.atan2(c[1] - b[1], c[0] - b[0])
            err = math.radians(LEAD_DEG + self.rng.uniform(-2.0, 2.0))
            self.yaw = _norm(exit_yaw - self.turn_sign * err)
            # 位置扰动（弧跟踪残差 ±2cm，垂直出线）
            nx, ny = -(c[1] - b[1]) / l2, (c[0] - b[0]) / l2
            off = self.rng.uniform(-0.02, 0.02)
            self.x += nx * off
            self.y += ny * off
            self.seg += 1
            self.mode = "drive"

    def _advance(self, v, steer_deg):
        cmd = steer_deg + STEER_TRIM
        self.yaw = _norm(self.yaw + (v / L) * math.tan(
            math.radians(cmd + BIAS)) * DT)
        self.x += v * math.cos(self.yaw) * DT
        self.y += v * math.sin(self.yaw) * DT

    def done(self):
        if self.mode == "drive":
            _, _, leg_len = self._leg()
            if self.seg >= len(self.wps) - 2:
                fx, fy = self.wps[-1]
                return math.hypot(fx - self.x, fy - self.y) < 0.20
            return False
        return False


def _norm(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def route_s(w, h=3):
    """S 形格网路线（3 格直段 + 90° 右 + 90° 左）：物理坐标 [(x,y), ...]。"""
    p = [(0.0, 0.0), (0.0, w * CELL)]
    p.append((p[-1][0] + w * CELL, p[-1][1]))          # 右转 90°
    p.append((p[-1][0], p[-1][1] + w * CELL))          # 左转 90°
    return p


def run_route(ld_fn, seed):
    rng = random.Random(seed)
    sim = Sim(route_s(3), ld_fn, rng)
    n = 0
    while not sim.done() and n < int(60 / DT):
        sim.step()
        n += 1
    # 指标：过两弯后末段（最后 0.5m 起到到达）的 cte
    h = sim.hist
    if len(h) < 10:
        return None
    # 找第二弯出口后的段：seg 2 = 末段
    tail = [r for r in h if r[3] is not None][-40:]   # 末 2s
    ctes = [abs(r[3]) for r in tail]
    peak = max(ctes)
    rms = math.sqrt(sum(c * c for c in ctes) / len(ctes))
    # 末段恢复时间：从进入末段起 |cte|<5mm 保持
    settle = None
    seg_ctes = [r for r in h]
    for i in range(len(seg_ctes)):
        if all(abs(r[3]) < 0.005 for r in seg_ctes[i:]):
            settle = seg_ctes[i][4] if False else i * DT
            break
    # 舵角抖动：相邻拍变化率
    dsteer = [abs(h[i + 1][4] - h[i][4]) for i in range(len(h) - 1)]
    jitter = max(dsteer)
    # 到达精度
    fx, fy = sim.wps[-1]
    miss = math.hypot(fx - sim.x, fy - sim.y)
    return {"peak": peak, "rms": rms, "settle": settle,
            "jitter": jitter, "miss": miss}


def run_inject(ld_fn, seed, chain=True):
    """S2：直段 10cm 横向注入。

    chain=True  -> S2a 单格段链 0.35m×3（格网真实形态：aim 每段饱和，
                   前瞻策略预期无差异，CTE-PID 承担全部修正）
    chain=False -> S2b 3 格长段 1.05m（前瞻策略生效场景）
    指标：各段段末残差、最终残差、反向穿越（过调）。
    """
    rng = random.Random(seed)
    wps = ([(0.0, 0.0), (0.0, 0.35), (0.0, 0.70), (0.0, 1.05)]
           if chain else [(0.0, 0.0), (0.0, 1.05)])
    sim = Sim(wps, ld_fn, rng)
    sim.x = 0.10
    n = 0
    while math.hypot(wps[-1][0] - sim.x, wps[-1][1] - sim.y) > 0.19 \
            and n < int(30 / DT):
        sim.step()
        n += 1
        if n > int(29 / DT):
            break
    h = sim.hist
    if len(h) < 5:
        return None
    # 各段段末残差：每段最后一拍的 |cte|
    seg_last, cur_seg = {}, 0
    for r in h:
        pass
    # Sim 不记录 seg，改用位置推段末：取 cte 局部（末 3 拍均值）
    ctes = [r[3] for r in h]
    final_res = abs(statistics.mean(ctes[-5:]))
    cross = max((min(ctes), -max(0.0, -min(ctes))), key=abs) \
        if min(ctes) < 0 and max(ctes) > 0 else 0.0
    # 过调幅度 = 穿越后的反向峰值
    overshoot = max(ctes) if (ctes[0] < 0 and max(ctes) > 0.002) else 0.0
    settle = None
    for i in range(len(h)):
        if all(abs(r[3]) < 0.008 for r in h[i:]):
            settle = i * DT
            break
    return {"final_res": final_res, "overshoot": overshoot,
            "settle": settle, "n_steps": n}


def main():
    print(f"R_TURN={R_TURN:.3f}m  满舵30°  TRIM={STEER_TRIM}°  "
          f"BIAS={+BIAS}°  20Hz  lead={LEAD_DEG}°\n")
    results = {}
    for name, fn in STRATS.items():
        rr = [r for r in (run_route(fn, s) for s in range(30)) if r]
        ia = [r for r in (run_inject(fn, s, chain=True) for s in range(30)) if r]
        ib = [r for r in (run_inject(fn, s, chain=False) for s in range(30))
              if r]

        def med(k, arr): return statistics.median(x[k] for x in arr)

        def p95(k, arr):
            v = sorted(x[k] for x in arr)
            return v[int(0.95 * (len(v) - 1))]

        st = [x["settle"] for x in rr if x["settle"] is not None]
        print(f"[S1 弯道出口 S形] {name}: 末段峰值 {med('peak', rr)*100:5.2f}cm "
              f"(p95 {p95('peak', rr)*100:5.2f})  末段RMS {med('rms', rr)*100:5.2f}cm "
              f"恢复 {statistics.median(st) if st else float('nan'):4.1f}s "
              f"抖动 {med('jitter', rr):4.1f}°/拍 到达误差 {med('miss', rr)*100:4.1f}cm")
        ov_a = med("overshoot", ia) * 100
        ov_b = med("overshoot", ib) * 100
        st_a = [x["settle"] for x in ia if x["settle"] is not None]
        st_b = [x["settle"] for x in ib if x["settle"] is not None]
        print(f"[S2a 单格链×3  ] {name}: 最终残差 {med('final_res', ia)*100:5.2f}cm "
              f"过调 {ov_a:5.2f}cm "
              f"稳定 {statistics.median(st_a) if st_a else float('nan'):4.1f}s")
        print(f"[S2b 3格长段   ] {name}: 最终残差 {med('final_res', ib)*100:5.2f}cm "
              f"过调 {ov_b:5.2f}cm "
              f"稳定 {statistics.median(st_b) if st_b else float('nan'):4.1f}s")
        results[name] = {"S1": rr, "S2a": ia, "S2b": ib}
        print()
    dst = Path(__file__).with_name("G2_前瞻策略对比.json")
    dst.write_text(json.dumps(
        {k: {"S1": v["S1"], "S2a": v["S2a"], "S2b": v["S2b"]}
         for k, v in results.items()},
        ensure_ascii=False, indent=1), encoding="utf-8")
    print("已落盘", dst.name)


if __name__ == "__main__":
    main()
