# -*- coding: utf-8 -*-
"""统一跟踪控制器：曲率前馈 + Stanley 型反馈 + 曲率限速，全程闭环。

对原版的拔高点 2：直道/弯道不再切换模式机。前馈 servo.command_for_kappa
按参考曲率打角（含左右不对称档案），反馈只收拾残差：

    steer = ff(κ(s+Δ)) + k_h·e_θ − k_c·atan(k_v·cte / (v+v0))

cte>0 = 车在路径左侧（需右打）。速度按前窗曲率峰值限速 v=√(a_lat/κ)，
附加速度斜坡、终点/停站渐近、到点半径（默认 12cm，原版 15.7cm——
大于排雷触发半径 6cm 是实锤缺陷，dznav 收紧并让雷格走途经语义）。

对原版的拔高点 3（可恢复性）：纯追踪"目标在身后只能越开越远"的死角
（docs/SIL发现-PIVOT过冲-20261016.md 轨迹证据）由两个机制根治：
- REBUILD：cte 超阈或进度停滞时，从当前位置到下一未过途经点直连重建；
- ALIGN_ARC：重建后首段方向在身后（点积<0）时，先原地弧转向对齐，
  不再画大回环等越界急停兜底。

ARC 机动（转角 ≥135° 的掉头弯、终点朝向对齐）保留原版"满舵低速 +
yaw 停止"机制，但出口用软锚点（fusion.anchor）而非硬吸附，且加
行程/时间双保险。

U2（2026-10-18）：cfg.track_mode="pure_pursuit" 时转向律切换为标准
前视弧长公式（κ=2y/d²，δ=atan(L·κ)），并吸收 regulated 思想——
前视距离随速度自适应、指令曲率纳入限速（v≤√(a_lat/κ)）。默认
"stanley" 保持现状，纯配置开关、不删旧路径。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .contract import wrap_pi, MAX_SPEED
from .refpath import ReferencePath, Waypoint
from .servo import ServoModel


@dataclass
class TrackerConfig:
    v_max: float = 0.32            # m/s 巡航（速度包络初值 0.3-0.4）
    v_min: float = 0.08
    a_lat_max: float = 0.20        # m/s² 侧向加速度预算（弯道限速）
    accel: float = 0.80            # m/s² 加速斜坡
    decel: float = 1.20            # m/s² 减速斜坡
    brake_window: float = 0.70     # m 前窗曲率峰值限速距离
    ff_lookahead: float = 0.04     # m 前馈取曲率前移量（过大出口提前撤舵切短弧）
    k_h: float = 30.0              # deg/rad 航向误差增益
    k_c: float = 10.0              # deg 横向误差增益幅值
    k_v: float = 6.0               # 1/(m/s) cte 项速度归一
    v0: float = 0.10               # m/s cte 项防低速发散
    steer_trim_deg: float = 0.0    # 常值机械配平（右偏为负）
    stop_radius: float = 0.12      # m 到点判定半径
    approach_dist: float = 0.45    # m 终点渐近减速起点
    v_end: float = 0.10            # m/s 终点兜底低速
    shoot_stop_radius: float = 0.17   # m 射击停站触发半径（对齐原版）
    shoot_ramp_dist: float = 0.50  # m 停站渐近减速起点
    recover_cte: float = 0.30      # m 触发 REBUILD 的横向偏差
    stuck_time: float = 3.0        # s 有速度但无进度 → REBUILD
    rebuild_limit: int = 3
    arc_v: float = 0.13            # m/s ARC 机动爬行速度
    arc_lead_deg: float = 6.0      # ARC 提前停止角（余下交给闭环/回差）
    arc_travel_scale: float = 1.8  # ARC 行程保险 = 名义弧长 x 该系数
    align_arc_thresh_deg: float = 100.0   # 首段在身后阈值
    anchor_pass: bool = True       # 途经点软锚点（anti-drift）
    # ---- U2 pure pursuit（regulated）对标（2026-10-18）----
    # track_mode="pure_pursuit" 时转向律切换为标准前视弧长公式
    #     κ = 2·y / d²   （y=前视点车体系横向偏移，d=车到前视点弦长）
    #     δ = servo.command_for_kappa(κ)   （ideal 档即 δ=atan(L·κ)）
    # 并吸收 regulated 思想（nav2 regulated_pure_pursuit 同源）：
    # 前视距离随速度自适应（快车看得远），指令曲率纳入限速
    #     v ≤ √(a_lat_max / max(|κ_path|, |κ_cmd|))
    # 对不可原地转向的 Ackermann：直线上的大偏差修正也会自动降速。
    track_mode: str = "stanley"    # "stanley"（现状默认）| "pure_pursuit"
    pp_ld_min: float = 0.14        # m 前视距离下限（含起步低速段）
    pp_ld_max: float = 0.35        # m 前视距离上限
    pp_ld_gain: float = 0.55       # L_d = pp_ld_min + gain·v_cmd（SIL 定标：
                                   #   弯速 v≈0.24 时 L_d≈0.27≈弯道物理半径
                                   #   R=0.272 —— L_d≫R 会切角后欠阻尼编织
                                   #   （K 列实测 K9 0.6cm→K8 7.8cm），增益
                                   #   压到 0.55 后全雷格 ≤6cm 达标）
    pp_cmd_regulate: bool = True   # regulated：指令曲率也参与限速


@dataclass
class Command:
    v: float = 0.0
    steer_deg: float = 0.0


@dataclass
class Tracker:
    path: ReferencePath
    servo: ServoModel
    cfg: TrackerConfig = field(default_factory=TrackerConfig)

    def __post_init__(self):
        self.s_prog = 0.0
        self.v_cmd = 0.0
        self.passed = 0               # 已过途经点（含起点 0）
        self.mode = "track"           # track | align_arc | arc | done | paused
        self.paused = False
        self.rebuilds = 0
        self._arc = None              # 机动上下文
        self._stuck_since = None
        self._last_s = 0.0
        self._travel = None           # 外部注入的累计里程 m
        self._pp_kappa_cmd = 0.0      # U2：pure_pursuit 最新指令曲率（regulated 限速用）
        self.arrived = False
        self.final_heading = None     # rad，终点朝向（executor 注入）

    # ---------------------------------------------------------------- 主入口
    def update(self, x: float, y: float, th: float, dt: float,
               now: float = 0.0) -> tuple[Command, list[dict]]:
        ev: list[dict] = []
        if self.arrived or self.paused:
            self.v_cmd = 0.0
            return Command(0.0, 0.0), ev

        # 位姿非有限 = 定位已崩，任何跟踪计算都无意义。就地停车，
        # 不重建（重建同样吃 x,y,th）、不 ARC。上层会因命令全零而闩锁。
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(th)
                and math.isfinite(dt) and dt > 0.0):
            self.v_cmd = 0.0
            return Command(0.0, 0.0), ev

        if self.mode in ("align_arc", "arc"):
            return self._update_arc(x, y, th, dt, now, ev)

        cmd = self._track(x, y, th, dt, now, ev)
        return cmd, ev

    # ---------------------------------------------------------------- 闭环跟踪
    def _track(self, x, y, th, dt, now, ev) -> Command:
        cfg, path = self.cfg, self.path
        pr = path.project(x, y, self.s_prog)
        # 进度单调（REBUILD 后 s_prog 重置由重建路径承担）
        self.s_prog = max(self.s_prog, pr["s"])

        # ---- 射击停站触发（只看"下一个未过停站"，防路线回折早触发）----
        # 必须排在途经点推进**之前**：推进一旦把 passed 抬过停站，下面那个
        # `i != passed+1: break` 就再也不会看到它了。
        for i in range(self.passed + 1, len(path.waypoints)):
            if i != self.passed + 1:
                break                     # 只允许紧邻的下一个途经点
            w = path.waypoints[i]
            if not w.shoot_stop or w.served:
                continue
            if math.hypot(x - w.x, y - w.y) <= cfg.shoot_stop_radius:
                w.served = True
                self.paused = True
                self.v_cmd = 0.0
                ev.append({"ev": "shoot_stop", "idx": i})
                return Command(0.0, 0.0)
            break

        # ---- 途经点推进 + 软锚点 ----
        while (self.passed + 1 < len(path.waypoints)
               and self.s_prog > path.vertex_s[self.passed + 1] + 0.02):
            self.passed += 1
            w = path.waypoints[self.passed]
            if w.shoot_stop and not w.served:
                # 越过停站却没能在 shoot_stop_radius(0.17m) 内触发过。这是纯
                # 瞬时距离判定的固有漏洞，不是罕见竞态：切角时横向偏差要涨到
                # recover_cte(0.30m) 才触发 rebuild，而 0.30 > 0.17 —— 车完全
                # 可以"过了弧长但从没进过半径"。旧代码就此永久跳过该站，
                # 无告警无补救，漏一站＝那些靶一分不得。
                # 补停优于开走：云台有偏航/俯仰自由度，晚 0.3m 仍能打。
                w.served = True
                self.paused = True
                self.v_cmd = 0.0
                ev.append({"ev": "shoot_stop", "idx": self.passed,
                           "late": True})
                return Command(0.0, 0.0)
            if cfg.anchor_pass and self.passed < len(path.waypoints) - 1:
                ev.append({"ev": "pass_wp", "idx": self.passed})

        # ---- 到点（距离 + 剩余进度双门：防路线中段路过终点旁早收车）----
        fw = path.waypoints[-1]
        d_final = math.hypot(x - fw.x, y - fw.y)
        ahead = path.length - self.s_prog
        if (d_final <= cfg.stop_radius and ahead <= 0.60) or ahead <= 0.03:
            self.arrived = True
            self.v_cmd = 0.0
            ev.append({"ev": "arrived", "d": d_final})
            return Command(0.0, 0.0)

        # ---- ARC 机动入口（掉头弯顶点 / 终点朝向前的顶点）----
        for c in path.corners:
            if c.mode != "arc" or c.idx <= self.passed:
                continue
            if path.vertex_s[c.idx] - self.s_prog <= 0.03:
                self._enter_arc(c, x, y, th, now, ev, at_vertex=True)
                return Command(cfg.arc_v,
                               c.turn_sign * (self.servo.steer_limit_deg - 2.0))
            break

        # ---- 恢复机制：大偏差 / 进度停滞 → 重建 ----
        if self._need_rebuild(pr, dt, now, x, y, th):
            if not self._rebuild(x, y, th, now, ev):
                self.arrived = True
                ev.append({"ev": "arrived", "d": d_final, "forced": True})
                return Command(0.0, 0.0)

        # ---- 转向律（U2：track_mode 二选一，默认 stanley 不变）----
        if cfg.track_mode == "pure_pursuit":
            steer = self._pure_pursuit_steer(x, y, th) + cfg.steer_trim_deg
        else:
            k_ff = self.path.at_s(
                min(self.s_prog + cfg.ff_lookahead, self.path.length))[3]
            ff = self.servo.command_for_kappa(k_ff)
            e_th = wrap_pi(pr["th"] - th)
            cte_term = cfg.k_c * math.atan2(
                cfg.k_v * pr["cte"], self.v_cmd + cfg.v0)
            steer = ff + cfg.k_h * e_th - cte_term + cfg.steer_trim_deg
        # 有限性优先于钳位：Python 的 min/max 遇 NaN 返回第一个实参，
        # 直接写 max(-L, min(L, nan)) 会得到 **+L 满舵**（实测），
        # 车会以最高速打死方向。NaN 先打成 0（回中）再钳位。
        if not math.isfinite(steer):
            steer = 0.0
        steer = max(-self.servo.steer_limit_deg,
                    min(self.servo.steer_limit_deg, steer))

        # ---- 速度规划 ----
        k_peak = self.path.kappa_max_window(self.s_prog, cfg.brake_window)
        # regulated（U2）：指令曲率一并纳入限速——直线上大偏差修正、
        # 末段向终点 homing 产生的 κ_cmd 同样触发降速。路径曲率未知
        # （NaN）时按最保守处理（见下方 else 分支）。
        k_cmd = (abs(self._pp_kappa_cmd)
                 if (cfg.track_mode == "pure_pursuit" and cfg.pp_cmd_regulate)
                 else 0.0)
        if math.isfinite(k_peak):
            v_kappa = math.sqrt(
                cfg.a_lat_max / max(k_peak, k_cmd, 1e-3))
        else:
            # 曲率未知 → 按最保守处理。原版 max(k_peak, 1e-3) 在 k_peak
            # 为 NaN 时返回 NaN（Python max 遇 NaN 返回首参），再被
            # min(v_max, nan, MAX_SPEED) 吞成 **v_max 全速**——
            # "不知道弯有多急"被解释成了"前面没有弯"。
            v_kappa = cfg.v_end if k_cmd <= 0.0 else math.sqrt(
                cfg.a_lat_max / max(k_cmd, 1e-3))
        v_t = min(cfg.v_max, v_kappa, MAX_SPEED)
        # 终点渐近
        if ahead < cfg.approach_dist:
            v_t = min(v_t, max(cfg.v_end,
                               cfg.v_max * ahead / cfg.approach_dist))
        # 射击停站渐近
        for i in range(self.passed + 1, len(self.path.waypoints)):
            w = self.path.waypoints[i]
            if not w.shoot_stop:
                continue
            d_w = math.hypot(x - w.x, y - w.y)
            if d_w < cfg.shoot_ramp_dist:
                v_t = min(v_t, max(0.10,
                                   cfg.v_max * d_w / cfg.shoot_ramp_dist))
            break
        # 斜坡
        if not math.isfinite(self.v_cmd):
            self.v_cmd = 0.0     # 显式复位；原先靠 max(0.0, nan)==0.0 侥幸归零
        dv = v_t - self.v_cmd
        self.v_cmd += max(-cfg.decel * dt, min(cfg.accel * dt, dv))
        self.v_cmd = max(0.0, min(self.v_cmd, MAX_SPEED))
        if self.v_cmd < cfg.v_min and v_t > cfg.v_min:
            self.v_cmd = cfg.v_min

        return Command(self.v_cmd, steer)

    # ---------------------------------------------------------------- U2 纯追踪
    def _pure_pursuit_steer(self, x: float, y: float, th: float) -> float:
        """标准 Pure Pursuit 前视弧长律（U2 对标，track_mode=pure_pursuit）。

        1. 前视点：路径上 s_prog + L_d 处（L_d 随速度自适应：快车看得远，
           慢车看得近——regulated pure pursuit 的标准做法）；超出路径
           末端时夹到终点，天然退化为"向终点 homing"。
        2. 前视点变换到车体系，取横向偏移 ey 与弦长 d，代入经典公式
               κ = 2·ey / d²
           （注意分母是**弦长平方**，不是名义 L_d²：路径末端被夹短时
           用名义值会低估曲率、末端修不动）。
        3. κ 经 servo.command_for_kappa 落舵角（ideal 档即 δ=atan(L·κ)，
           不对称档案自动吃到左右差异——这正是 U2 要对照的标准式）。
        指令曲率存 _pp_kappa_cmd 供 regulated 限速取用。
        """
        cfg = self.cfg
        ld = min(cfg.pp_ld_max,
                 cfg.pp_ld_min + cfg.pp_ld_gain * max(0.0, self.v_cmd))
        px, py, _pth, _pk = self.path.at_s(
            min(self.s_prog + ld, self.path.length))
        dx, dy = px - x, py - y
        c, s = math.cos(th), math.sin(th)
        ey = -dx * s + dy * c          # 车体系横向偏移（左正）
        d = math.hypot(dx * c + dy * s, ey)   # 弦长（车到前视点距离）
        d = max(d, 0.05)               # 下限防除零放大（末段 homing）
        kappa = 2.0 * ey / (d * d)
        if not math.isfinite(kappa):
            kappa = 0.0                # 出口 NaN 防护与 stanley 分支同源
        self._pp_kappa_cmd = kappa
        return self.servo.command_for_kappa(kappa)

    # ---------------------------------------------------------------- ARC 机动
    def _enter_arc(self, corner, x, y, th, now, ev, at_vertex=False,
                   target_th=None):
        """进入 ARC：corner 顶点机动，或对齐 target_th 的原地弧。"""
        if target_th is None:
            target_th = corner.exit_th
        r_est = 1.0 / max(0.05, self.servo.kappa_max(corner.turn_sign
                       if corner.turn_sign else 1.0))
        travel_cap = (corner.angle if at_vertex else abs(
            wrap_pi(target_th - th))) * r_est * self.cfg.arc_travel_scale + 0.15
        self._arc = {
            "sign": (corner.turn_sign if at_vertex
                     else (1.0 if wrap_pi(target_th - th) > 0 else -1.0)),
            "th_start": th,
            "target": target_th,
            "rot_target": abs(wrap_pi(target_th - th))
                          - math.radians(self.cfg.arc_lead_deg),
            "travel_start": None,   # 由 executor 经 travel() 注入里程
            "travel_cap": travel_cap,
            "t_start": now,
            "t_cap": travel_cap / self.cfg.arc_v * 2.5 + 2.0,
            "corner": corner if at_vertex else None,
        }
        self.mode = "arc" if at_vertex else "align_arc"
        ev.append({"ev": "arc_begin", "mode": self.mode})

    def set_travel(self, travel: float):
        """executor 每拍注入融合层累计里程（ARC 行程保险用）。"""
        self._travel = travel
        if self._arc is not None and self._arc["travel_start"] is None:
            self._arc["travel_start"] = travel

    def _update_arc(self, x, y, th, dt, now, ev) -> tuple[Command]:
        a = self._arc
        cfg = self.cfg
        sign = a["sign"]
        rot = sign * wrap_pi(th - a["th_start"])
        done = rot >= a["rot_target"]
        timeout = now - a["t_start"] > a["t_cap"]
        if a["travel_start"] is not None and self._travel is not None:
            done = done or (self._travel - a["travel_start"]) > a["travel_cap"]
        if done or timeout:
            c = a["corner"]
            if c is not None:
                ev.append({"ev": "arc_exit", "x": c.exit_x, "y": c.exit_y,
                           "th": c.exit_th, "idx": c.idx})
                self.passed = max(self.passed, c.idx)
                pr = self.path.project(c.exit_x, c.exit_y, self.s_prog,
                                       back=1.0, fwd=1.0)
                self.s_prog = max(self.s_prog, pr["s"])
            else:
                ev.append({"ev": "align_done", "th": a["target"]})
            self._arc = None
            self.mode = "track"
            self.v_cmd = 0.0
            return Command(0.0, 0.0), ev
        steer = sign * (self.servo.steer_limit_deg - 2.0)
        return Command(cfg.arc_v, steer), ev

    # ---------------------------------------------------------------- 恢复
    def _need_rebuild(self, pr, dt, now, x, y, th) -> bool:
        cte = pr["cte"]
        # 非有限 cte 也必须触发重建：abs(nan) > 阈值 为 False，
        # 原写法会让恢复机制在最该介入的时候恰好失效。
        if not math.isfinite(cte) or abs(cte) > self.cfg.recover_cte:
            return True
        if self.v_cmd > 0.05:
            if self.s_prog - self._last_s < 1e-4:
                if self._stuck_since is None:
                    self._stuck_since = now
                elif now - self._stuck_since > self.cfg.stuck_time:
                    return True
            else:
                self._stuck_since = None
        else:
            self._stuck_since = None
        self._last_s = self.s_prog
        return False

    def _rebuild(self, x, y, th, now, ev) -> bool:
        if self.rebuilds >= self.cfg.rebuild_limit:
            return False
        self.rebuilds += 1
        nxt = min(self.passed + 1, len(self.path.waypoints) - 1)
        # 保留未过停站/途经点标志
        keep = [Waypoint(x, y, label="rebuild")]
        for i in range(nxt, len(self.path.waypoints)):
            keep.append(self.path.waypoints[i])
        from .refpath import build_reference, PathOptions
        # 必须沿用建原路径时的选项：这里曾经硬编码 dev_cap=0.08/
        # dev_cap_critical=0.03，于是运维在 YAML 里调过的值只在首条路径上生效，
        # 第一次重建就被静默丢回默认（arc_fallback_deg 同样会丢）。
        opts = self.path.opts or PathOptions(dev_cap=0.08, dev_cap_critical=0.03)
        self.path = build_reference(keep, self.servo, opts)
        self.s_prog = 0.0
        self.passed = 0
        self._last_s = 0.0
        self._stuck_since = None
        self.v_cmd = 0.0
        ev.append({"ev": "rebuild", "to_idx": nxt})
        # 首段在身后 → 先对齐弧
        th0 = self.path.samples[0][3]
        if abs(wrap_pi(th0 - th)) > math.radians(
                self.cfg.align_arc_thresh_deg):
            self._enter_arc(_AlignCorner(sign=1.0 if wrap_pi(th0 - th) > 0
                                         else -1.0),
                            x, y, th, now, ev, at_vertex=False, target_th=th0)
        return True

    # ---------------------------------------------------------------- 控制
    def resume(self):
        self.paused = False
        self.v_cmd = 0.0


class _AlignCorner:
    """对齐弧的 corner 替身（无真实顶点，仅携带转向符号）。"""

    def __init__(self, sign: float):
        self.turn_sign = sign
        self.angle = 0.0
        self.exit_th = 0.0
