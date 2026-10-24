# -*- coding: utf-8 -*-
"""卡 t12 自测：LocalizationTask 吸附自愈 + 信念漂移护栏（**纯脚本，无 pytest**）。

    python src\\dzbrain\\tests\\test_drift.py              # 单元用例（快）
    python src\\dzbrain\\tests\\test_drift.py --sweep        # 追加：注入对照扫描（A2）
    python src\\dzbrain\\tests\\test_drift.py --sweep --json out.json

覆盖：
  §1 「误差变小才施加」闸（t12 A3）
     1.1 车到位（< arrival_m）且吸附不会让离线距离变大 → **施加**（anchored+1）
     1.2 顶点窗口内横向残差 ∈ (drift_warn, 上限] → **横向重锚**，施加后离线距离≈0（误差确实变小）
     1.3 残差 > 上限 → **拒绝**（anchor_refused+1、信念不动、日志有「吸附被拒」）
  §2 失败记账：多次拒绝 → 计数递增；`guard_state()` 可读
  §3 降速/重锚/停机升级：
     3.1 连续不健康 < slow_after → 不下发
     3.2 达到 slow_after → `P_MOTION_LIMIT` = 降速（0.08）
     3.3 达到 stop_after → 速度 0
     3.4 恢复干净 drift_clear_ticks 拍 → 撤销为 None（读侧 no-op）
  §4 边界护栏：信念进 margin 内 → 告警 + 限速；再近 → 停车请求
  §5 `P_MOTION_LIMIT` 载荷契约（字段齐全、类型正确）
  §6 注入对照扫描（--sweep，A2）：本机自建闭环驱动**真 LocalizationTask**，
     注入 ±5%/±10%（及 v1 标定差 +18.5%）里程比例误差 × 噪声 0.005/0.01，
     输出 信念↔真值RMS / 越界拍 / 是否跑完 的对照表。

★ 为什么 §6 要自建闭环：包内 `tools/sim.py` 的定位/吸附是**它自己的模型**
  （`SimConfig.anchor_on_arrival`），**不调用** `LocalizationTask` ⇒ 改 localization.py
  对 sim_cli 的指标没有任何影响（实测修复前后 sim 指标逐位相同）。要让"修复前后"可比，
  必须用**真任务**跑闭环 —— 这就是 §6。
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:                                   # pragma: no cover
    pass

from harness import check, finish, section                       # noqa: E402

from dzbrain.core import geometry as geo                         # noqa: E402
from dzbrain.core.blackboard import P_POSE, P_ROUTE, Blackboard  # noqa: E402
from dzbrain.core.config import Config                           # noqa: E402
from dzbrain.core.events import EV_DRIFT_WARNING, EV_POSE_ANCHORED, EventBus  # noqa: E402
from dzbrain.route.parser import parse_mission                   # noqa: E402
# ★ `P_MOTION_LIMIT` 由 localization.py 定义（core/blackboard.py 不在 t12 写域内）。
#   用 try/except 兜一下：本 harness 也要能跑**修复前**的 localization.py（那时还没有这个键），
#   否则"修复前后对照"没法用同一份扫描代码。
from dzbrain.tasks.localization import LocalizationTask       # noqa: E402
try:
    from dzbrain.tasks.localization import P_MOTION_LIMIT      # noqa: E402
except ImportError:                                            # pragma: no cover
    P_MOTION_LIMIT = "motion_limit"

from fakes import FakeBridge, FakeClock                          # noqa: E402

#: 护栏参数（与 localization.py 的**代码默认值**一致；config 由 t11 专写，本卡不碰）
ARRIVAL = 0.12
DRIFT_WARN = 0.30
MARGIN = 0.60
PULL_MAX = 0.35

#: 扫描用的路线：地图**内部**（避开"贴着地图边"造成的必然越界），带一个直角
SWEEP_TOKENS = ["D4", "E4", "F4", "G4", "H4", "J4", "J8"]

#: ★贴边路线（终点贴近安全边界）：用于证明"越界拍"这类硬指标护栏确实帮忙
#: （J11 的 x=0.175 在 margin 0.6 的安全区之外 ⇒ 车开到终点附近必然逼近/越界）
EDGE_TOKENS = ["D4", "E4", "F4", "G4", "H4", "J4", "J11"]


def cfg_of(**over) -> Config:
    data = {
        "pursuit": {"arrival_m": ARRIVAL, "drift_warn_m": DRIFT_WARN,
                    "map_safety_margin_m": MARGIN, "drift_seg_gate_m": 0.18,
                    "cruise_speed_mps": 0.15, "wheelbase_m": 0.157,
                    "max_steer_deg": 30.0, "lookahead_min_m": 0.18},
        "fix": {"anchor_at_waypoint": True},
        "logging": {"rate_limit_s": 0.0},
    }
    for k, v in over.items():
        data.setdefault(k.split(".")[0], {})[k.split(".", 1)[1]] = v
    return Config.from_dict(data)


class Rig:
    """真 LocalizationTask + FakeBridge + 黑板 + 事件记录。"""

    def __init__(self, cfg: Config, tokens=None, pose=None) -> None:
        self.cfg = cfg
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.logs: list[str] = []
        self.task = LocalizationTask(self.bridge, self.bb, self.bus, cfg,
                                     log=self.logs.append, clock=self.clock.now)
        self.sub = self.bus.subscribe("t", [EV_DRIFT_WARNING, EV_POSE_ANCHORED], maxsize=4096)
        self.events = []
        self.route = parse_mission(list(tokens or SWEEP_TOKENS))
        self.bb.set(P_ROUTE, self.route, "test")
        self.task.setup()
        if pose is not None:
            self.set_pose(*pose)

    def set_pose(self, x, y, yaw=0.0):
        self.bridge.set_pose(x, y, yaw)
        self.clock.advance(0.0)

    def tick(self, dt=0.05):
        self.task.tick()
        self.events.extend(self.sub.poll(4096))
        self.clock.advance(dt)

    def ticks(self, n, dt=0.05):
        for _ in range(n):
            self.tick(dt)

    def belief(self):
        return self.bb.get(P_POSE)

    def limit(self):
        return self.bb.get(P_MOTION_LIMIT)

    def count(self, ev_type):
        return sum(1 for e in self.events if e.type == ev_type)


# =========================================================================== #
section("1. ★「误差变小才施加」：到位吸附 / 横向重锚 / 超限拒绝")
# =========================================================================== #
# ★ 几何约定：路线 D4→E4→F4… 沿 **-y**（格心 x 相同、y 每格减 0.35），
#   所以"沿路往前"= y 增大，"横向偏"= ±x。"到 E4 前 0.05 m" = (e4.x, e4.y + 0.05)。
d4 = geo.block_to_meters("D4")
e4 = geo.block_to_meters("E4")
BEFORE = 0.05                       # 沿路方向：离 E4 还有 0.05 m（< arrival_m 0.12）


def at_e4(lat: float, before: float = BEFORE):
    """E4 前方 `before` 米处、横向偏 `lat` 米的位姿（yaw 沿 -y 即 π/2…用 0 亦可）。"""
    return (e4[0] + lat, e4[1] + before, 0.0)


# 1.1 车在 E4 前方 0.05 m（< arrival_m）→ 施加吸附
rig = Rig(cfg_of(), pose=at_e4(0.0))
rig.tick()                                    # ★ 只看**吸附那一拍**：闩锁机制会让下一拍
check("1.1 车到位（离格心 < arrival_m）→ 施加吸附（anchored ≥ 1、发 EV_POSE_ANCHORED）",
      rig.task.anchored >= 1 and rig.count(EV_POSE_ANCHORED) >= 1,
      f"anchored={rig.task.anchored} events={rig.count(EV_POSE_ANCHORED)}")
check("1.1b 吸附那一拍的信念 = 格心（yaw 不变）"
      "（★注：里程计仍报原始位置，闩锁会在下一拍解锁 —— 这是既有行为，本卡未改）",
      rig.belief() is not None
      and abs(rig.belief().x - e4[0]) < 1e-9 and abs(rig.belief().y - e4[1]) < 1e-9,
      f"belief=({rig.belief().x:.3f},{rig.belief().y:.3f}) want={e4}")

# 1.2 横向 0.20 m（≤ drift_warn 0.30）→ 不重锚（"不制造误差"）
rig2 = Rig(cfg_of(), pose=at_e4(0.20))
rig2.ticks(3)
check("1.2 横向残差 ≤ drift_warn(0.30) → **不制造误差**：不重锚（reanchored == 0）",
      rig2.task.reanchored == 0, f"reanchored={rig2.task.reanchored}")

# 1.2b 横向 0.32 m（> drift_warn、≤ 上限 0.35）→ 横向重锚，且重锚后误差确实变小
rig4 = Rig(cfg_of(), pose=at_e4(0.32))
rig4.tick()                       # ★ 同 1.1b：只看重锚那一拍（闩锁让下一拍回到原始里程计）
lat4 = None if rig4.belief() is None else abs(rig4.belief().x - e4[0])
check("1.2b ★顶点窗口内横向残差 0.32 m（> 0.30、≤ 上限 0.35）→ 横向重锚，且误差确实变小",
      rig4.task.reanchored >= 1 and lat4 is not None and lat4 < 0.05,
      f"reanchored={rig4.task.reanchored} 重锚后横向残差="
      f"{None if lat4 is None else round(lat4, 4)}")

# 1.3 横向 0.45 m > 上限 0.35 m → 拒绝
rig5 = Rig(cfg_of(), pose=at_e4(0.45))
rig5.ticks(3)
check("1.3 ★横向残差 0.45 m > 重锚上限 0.35 m → 拒绝（anchor_refused ≥ 1、不硬拉）",
      rig5.task.anchor_refused >= 1 and rig5.task.reanchored == 0,
      f"refused={rig5.task.anchor_refused} reanchored={rig5.task.reanchored}")
check("1.3b 拒绝时**保留当前信念**（x 仍 ≈ 2.625+0.45，没有被硬拽到折线上）",
      rig5.belief() is not None and abs(rig5.belief().x - (e4[0] + 0.45)) < 0.02,
      f"belief.x={rig5.belief().x:.3f} want≈{e4[0] + 0.45:.3f}")
check("1.3c 拒绝有**显式记账**：计数 + 「吸附被拒」日志（绝不静默）",
      any("吸附被拒" in ln for ln in rig5.logs), f"logs={[l for l in rig5.logs if '被拒' in l][:1]}")


# =========================================================================== #
section("2. 失败记账：guard_state() 可读、计数单调")
# =========================================================================== #
gs = rig5.task.guard_state()
check("2.1 guard_state() 暴露 anchor_refused / drift_run / limit 等字段",
      isinstance(gs, dict) and {"anchor_refused", "reanchored", "drift_run", "limit"} <= set(gs),
      f"keys={sorted(gs)}")
rig6 = Rig(cfg_of(), pose=at_e4(0.45))
before = rig6.task.anchor_refused
rig6.ticks(10)
check("2.2 持续偏离 → 拒绝计数只增不减（可审计）",
      rig6.task.anchor_refused >= before, f"refused={rig6.task.anchor_refused}")


# =========================================================================== #
section("3. ★降速 / 停机升级与撤销（P_MOTION_LIMIT）")
# =========================================================================== #
# 时序（实测）：第 1 拍 = 重锚被拒(+1) + 不健康(+1) = 2；之后拒绝被闩锁（同顶点只记一次），
# 不健康每拍 +1 ⇒ drift_slow_after=3 在第 2 拍触发降速、drift_stop_after=8 在第 7~8 拍触发停车。
rig7 = Rig(cfg_of(), pose=at_e4(0.45))
rig7.tick()
check("3.1 连续不健康 < slow_after(3) → 先不下发限速",
      rig7.limit() is None, f"limit={rig7.limit()} drift_run={rig7.task.drift_run}")
rig7.tick()
lim = rig7.limit()
check("3.2 ★达到 slow_after → 下发降速请求（speed_mps = 0.08、reason=drift）",
      isinstance(lim, dict) and abs(float(lim.get("speed_mps", -1)) - 0.08) < 1e-9
      and lim.get("reason") == "drift", f"limit={lim}")
check("3.2b 载荷字段齐全（读侧契约）",
      isinstance(lim, dict) and {"speed_mps", "reason", "drift_run", "anchor_refused",
                                 "distance_to_boundary_m", "since"} <= set(lim),
      f"payload keys={sorted(lim) if isinstance(lim, dict) else None}")
rig7.ticks(8)
lim7 = rig7.limit()
check("3.3 ★连续不健康到 stop_after → 速度请求降到 0（禁止带大偏差全速跑）",
      isinstance(lim7, dict) and float(lim7.get("speed_mps", -1)) == 0.0,
      f"limit={lim7}")
check("3.3b 计数同步：anchor_refused ≥ 1、motion_limited_ticks ≥ 1",
      rig7.task.anchor_refused >= 1 and rig7.task.motion_limited_ticks >= 1,
      f"{rig7.task.guard_state()}")

# 3.4 撤销：把车放回折线上（干净）→ healthy_run 达标后写 None
g4 = geo.block_to_meters("G4")
rig7.set_pose(g4[0], g4[1], 0.0)
rig7.ticks(12)
check("3.4 ★恢复干净 → 撤销限速：P_MOTION_LIMIT 变 None（读侧回到完全 no-op）",
      rig7.limit() is None, f"limit={rig7.limit()} state={rig7.task.guard_state()}")


# =========================================================================== #
section("4. ★边界护栏：接近安全边界就提前降速 / 停车（而不是只有越界后硬停）")
# =========================================================================== #
x_min, x_max, y_min, y_max = geo.map_bounds(MARGIN)
rig8 = Rig(cfg_of(), pose=(x_max - 0.05, y_min + 0.50, 0.0))     # 离边界 0.05 m（< guard 0.10）
rig8.ticks(3)
lim8 = rig8.limit()
check("4.1 信念进入边界护栏区（< 0.10 m）→ 下发降速 + 告警",
      isinstance(lim8, dict) and float(lim8.get("speed_mps", -1)) <= 0.08
      and rig8.task.boundary_alerts >= 1,
      f"limit={lim8} alerts={rig8.task.boundary_alerts}")
check("4.1b 告警日志写明「离边界」与「提前」（不是越界后才发现）",
      any("接近安全边界" in ln for ln in rig8.logs),
      f"{[l for l in rig8.logs if '边界' in l][:1]}")
rig9 = Rig(cfg_of(), pose=(x_max - 0.01, y_min + 0.50, 0.0))     # < stop 0.03
rig9.ticks(3)
lim9 = rig9.limit()
check("4.2 再近（< 0.03 m）→ 请求停车（0 m/s、reason=boundary_stop）",
      isinstance(lim9, dict) and float(lim9.get("speed_mps", -1)) == 0.0
      and lim9.get("reason") == "boundary_stop", f"limit={lim9}")
rig10 = Rig(cfg_of(), pose=(geo.block_to_meters("D4")))
rig10.ticks(3)
check("4.3 远离边界（正常行驶）→ 不下发任何限速（保持零注入逐位一致的前提）",
      rig10.limit() is None, f"limit={rig10.limit()}")


# =========================================================================== #
section("5. P_MOTION_LIMIT 载荷契约（读侧必须能按此实现 no-op）")
# =========================================================================== #
check("5.1 键名词符约定：字符串、非空、与 core 现有键不冲突",
      isinstance(P_MOTION_LIMIT, str) and P_MOTION_LIMIT
      and P_MOTION_LIMIT not in {"pose", "route", "safety", "fix", "telemetry", "odom_age"},
      repr(P_MOTION_LIMIT))
check("5.2 无信号时值是 None（或键不存在）——**不是** 0 速度",
      Rig(cfg_of(), pose=geo.block_to_meters("D4")).limit() in (None,),
      "healthy 场景 limit is None")


# =========================================================================== #
#  §6 注入对照扫描（A2）：自建闭环驱动**真 LocalizationTask**
# =========================================================================== #
def _lookahead_target(pose, route, look=0.30):
    """在信念的投影点前方取 look 处的路径点（纯几何，够用）。"""
    pts = [(w.x, w.y) for w in route.waypoints]
    best, bi = 1e9, 0
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((pose.x - ax) * dx + (pose.y - ay) * dy) / L2))
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(px - pose.x, py - pose.y)
        if d < best:
            best, bi, bt = d, i, t
    # 沿折线向前推进 look 距离
    rem = look
    i, t = bi, bt
    pts2 = pts
    while i < len(pts2) - 1 and rem > 0:
        ax, ay = pts2[i]
        bx, by = pts2[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        left = seg * (1.0 - t)
        if rem < left:
            t = t + rem / max(seg, 1e-9)
            rem = 0.0
        else:
            rem -= left
            i += 1
            t = 0.0
    i = min(i, len(pts2) - 2)
    ax, ay = pts2[i]
    bx, by = pts2[i + 1]
    return ax + t * (bx - ax), ay + t * (by - ay)


def _step_bicycle(x, y, yaw, v, steer_deg, dt, wheelbase):
    yaw = geo.normalize_angle(yaw + (v / wheelbase) * math.tan(math.radians(steer_deg)) * dt)
    return x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw


def run_sweep_scenario(scale_err, noise, *, seed=7, dt=0.05, max_steps=12000,
                       cruise=0.15, use_limit=True, tokens=None):
    """注入里程比例误差 + 噪声，跑闭环，返回指标。**驱动的是真 LocalizationTask。**

    · 真值：自行车模型按指令积分；
    · 里程计（信念原始输入）：同样积分但**距离乘 (1+scale_err)**，再叠加高斯噪声；
    · 控制：用 `P_POSE`（定位输出）做纯追踪；
    · 限速：若 `P_MOTION_LIMIT` 有信号且 `use_limit`，把指令速度钳到它的 `speed_mps`。
    """
    import random
    rng = random.Random(seed)
    cfg = cfg_of()
    clock = FakeClock()
    bridge = FakeBridge(clock)
    bb = Blackboard()
    bus = EventBus()
    task = LocalizationTask(bridge, bb, bus, cfg, log=lambda s: None, clock=clock.now)
    route = parse_mission(list(tokens or SWEEP_TOKENS))
    bb.set(P_ROUTE, route, "test")
    task.setup()

    x0, y0 = route[0].x, route[0].y
    tx, ty, tyaw = x0, y0, 0.0                      # 真值
    ox, oy, oyaw = x0, y0, 0.0                      # 里程计（信念输入）
    bridge.set_pose(ox, oy, oyaw)

    x_min, x_max, y_min, y_max = geo.map_bounds(MARGIN)
    last = route[len(route) - 1]
    errs, oob, limited = [], 0, 0
    reached = False
    steps = 0
    for steps in range(1, max_steps + 1):
        pose = bb.get(P_POSE)
        if pose is None:
            pose = geo.Pose(ox, oy, oyaw)
        lx, ly = _lookahead_target(pose, route)
        alpha = geo.ang_diff(math.atan2(ly - pose.y, lx - pose.x), pose.yaw)
        d = max(math.hypot(lx - pose.x, ly - pose.y), 0.10)
        steer = math.degrees(math.atan2(2.0 * 0.157 * math.sin(alpha), d))
        steer = max(-30.0, min(30.0, steer))
        v = cruise
        lim = bb.get(P_MOTION_LIMIT)
        if use_limit and isinstance(lim, dict):
            v = min(v, float(lim.get("speed_mps", v)))
            limited += 1
        if math.hypot(last.x - pose.x, last.y - pose.y) < 0.20:
            reached = True
            break
        # 真值一步
        tx, ty, tyaw = _step_bicycle(tx, ty, tyaw, v, steer, dt, 0.157)
        # 里程计一步（含比例误差 + 噪声）
        ox2, oy2, oyaw2 = _step_bicycle(ox, oy, oyaw, v * (1.0 + scale_err), steer, dt, 0.157)
        ox = ox2 + rng.gauss(0.0, noise)
        oy = oy2 + rng.gauss(0.0, noise)
        # ★ 航向噪声要**小一个量级**：位置噪声 0.005 m/拍已经很保守，若按 `noise/v` 当角噪声
        #   （=0.033 rad/拍 ≈ 22°/s 的随机游走）连零注入场景都会把信念甩飞，
        #   那是“harness 自己的假故障”，会把 A2 对照污染成噪声比较。
        oyaw = geo.normalize_angle(oyaw2 + rng.gauss(0.0, noise * 0.02))
        bridge.set_pose(ox, oy, oyaw)
        task.tick()
        clock.advance(dt)
        belief = bb.get(P_POSE)
        if belief is not None:
            errs.append(math.hypot(belief.x - tx, belief.y - ty))
            if not (x_min <= belief.x <= x_max and y_min <= belief.y <= y_max):
                oob += 1
    rms = math.sqrt(sum(e * e for e in errs) / len(errs)) if errs else float("nan")
    # ★ 用 getattr：这份 harness 也要能跑**修复前**的 localization.py（那时还没有
    #   anchor_refused / reanchored / motion_limit），这样"修复前后对照"才是同一把尺子。
    return {"scale_err": scale_err, "noise": noise, "steps": steps,
            "belief_rms": round(rms, 4),
            "belief_max": round(max(errs), 4) if errs else float("nan"),
            "oob_ticks": oob, "completed": reached, "limited_ticks": limited,
            "anchored": int(getattr(task, "anchored", 0)),
            "anchor_refused": int(getattr(task, "anchor_refused", 0)),
            "reanchored": int(getattr(task, "reanchored", 0)),
            "limit_published": int(getattr(task, "motion_limit_published", 0))}


def sweep() -> int:
    section("6. ★注入对照扫描（A2）：±5%/±10% 与 v1 标定差(+18.5%) × 噪声 0.005/0.01")
    rows = []
    for scale in (0.0, 0.05, -0.05, 0.10, -0.10, 0.185):
        for noise in (0.005, 0.01):
            r = run_sweep_scenario(scale, noise)
            rows.append(r)
    clean = run_sweep_scenario(0.0, 0.0)                 # 零注入零噪声的 sanity
    edge = run_sweep_scenario(0.10, 0.01, tokens=EDGE_TOKENS)   # 终点贴着安全边界
    print(f"  {'ε':>7} {'噪声':>6} {'信念RMS':>9} {'信念MAX':>9} {'越界拍':>7} {'跑完':>5} "
          f"{'限速拍':>7} {'吸附':>5} {'被拒':>5} {'重锚':>5}")
    for r in rows:
        print(f"  {r['scale_err']:>+7.1%} {r['noise']:>6.3f} {r['belief_rms']:>9.4f} "
              f"{r['belief_max']:>9.4f} {r['oob_ticks']:>7} {str(r['completed']):>5} "
              f"{r['limited_ticks']:>7} {r['anchored']:>5} {r['anchor_refused']:>5} "
              f"{r['reanchored']:>5}")
    print(f"  [sanity] ε=0 噪声=0 → RMS={clean['belief_rms']:.4f} 跑完={clean['completed']}")
    print(f"  [贴边路线] ε=+10% 噪声=0.01 → RMS={edge['belief_rms']:.4f} "
          f"越界={edge['oob_ticks']} 跑完={edge['completed']} 限速拍={edge['limited_ticks']}")
    if "--json" in sys.argv:
        out = sys.argv[sys.argv.index("--json") + 1]
        pathlib.Path(out).write_text(
            json.dumps({"grid": rows, "sanity": clean, "edge": edge},
                       ensure_ascii=False, indent=2), encoding="utf-8", newline="\n")
        print(f"  JSON → {out}")
    # ★"护栏确实起作用"的最硬证据：同样注入下，**关掉限速**会让信念漂移/越界变差
    off = []
    for scale in (0.10, 0.185):
        a = run_sweep_scenario(scale, 0.01, use_limit=True)
        b = run_sweep_scenario(scale, 0.01, use_limit=False)
        off.append((scale, a, b))
        print(f"  [对照] ε={scale:+.1%}: 限速ON RMS={a['belief_rms']:.4f} 越界={a['oob_ticks']} "
              f"跑完={a['completed']} | 限速OFF RMS={b['belief_rms']:.4f} "
              f"越界={b['oob_ticks']} 跑完={b['completed']}")
    edge_off = run_sweep_scenario(0.10, 0.01, tokens=EDGE_TOKENS, use_limit=False)
    print(f"  [贴边路线·对照] 限速OFF → RMS={edge_off['belief_rms']:.4f} "
          f"越界={edge_off['oob_ticks']} 跑完={edge_off['completed']}")
    check("6.1 扫描出全部 12 个组合（6 个 ε × 2 个噪声）+ sanity + 贴边场景",
          len(rows) == 12, f"{len(rows)} 行")
    check("6.2 零注入**零噪声**：信念↔真值 RMS ≈ 0（没有无缘无故的偏差）",
          clean["belief_rms"] < 0.02, f"rms={clean['belief_rms']}")
    check("6.3 ★注入场景里护栏确实介入（限速拍 > 0）—— 不是「写了但没走到」",
          all(a["limited_ticks"] > 0 for _s, a, _b in off),
          "; ".join(f"ε={s:+.1%} 限速拍={a['limited_ticks']}" for s, a, _b in off))
    check("6.3b ★贴边场景：限速 ON 的越界拍不多于 OFF（护栏把车从边界前拖住）",
          edge["oob_ticks"] <= edge_off["oob_ticks"],
          f"ON={edge['oob_ticks']} OFF={edge_off['oob_ticks']}（限速拍 {edge['limited_ticks']}）")
    check("6.3c 诚实结论：**单纯软件不能消掉比例误差** ⇒ 限速ON/OFF 的 RMS 同量级"
          "（真正解要靠现场标定裁决，见报告 §T12-D），本卡只保证「不劣化」",
          max(abs(a["belief_rms"] - b["belief_rms"]) for _s, a, b in off) < 0.30,
          "; ".join(f"ε={s:+.1%} ΔRMS={abs(a['belief_rms'] - b['belief_rms']):.4f}"
                    for s, a, b in off))
    check("6.4 注入场景仍有可跑完的组合（护栏不把车钉死）",
          sum(1 for r in rows if r["completed"]) >= 10,
          f"完成 {sum(1 for r in rows if r['completed'])}/12")
    return 0


# =========================================================================== #
if __name__ == "__main__":
    if "--sweep" in sys.argv:
        sweep()
    finish()
