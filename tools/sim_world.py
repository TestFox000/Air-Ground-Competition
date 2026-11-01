# -*- coding: utf-8 -*-
"""WB-2 世界模型（卡 `WB-2-RIG-20261031` · tier-1）——**真值车体 × 里程计生成 × 执行器链**。

## 这个模块存在的理由（不是"又造一个 sim"）

现有两台各缺一半（`00_树.md` §为什么需要这条线，逐条实测）：

| 现状 | 缺口 | 谁补 |
|---|---|---|
| `tools/sim_cli.py`（tier-0） | 有注入，但**没有任务状态机**；自带一套平行投影（`_project_sticky`/`_clamp_forward`/`_advance_anchor`，窗口 0.80/1.50、`allow_backward=True`） | 本模块：**改用生产任务本尊**，投影就是 `Path.project` |
| `main.py --bench --drive` | 有真线程，但**没有注入**（`bridge.set_pose_at_block` 喂理想位姿），也没有执行器链 | 本模块：里程计生成 + `wb_rig.actuator` 四项 |

★ **不新建仿真台副本**这条（`能力索引.md:12`）之所以没被本卡违反，是因为：
  · 本模块**不重写任何物理/决策**，它 import 生产的 `step_bicycle` 做积分、
    生产的 `LocalizationTask/TrackerTask/SafetyTask/MissionTask` 做决策；
  · 它只提供**被控对象**与**观测**（真值世界 + 里程计），这两件事在 `sim.py` 里是
    "平行实现"，在本卡里是"生产线程在环"（`sim.py` 自己头注第 109 行写着
    "sim 不 import tasks/" —— 本模块**恰恰** import tasks/，这是本卡与 tier-0 的根本区别）。

## 单向依赖（不许反向）

```
wb_rig.sched / wb_rig.actuator / wb_rig.arms      ← 编排件（无物理）
        ↓
tools/sim_world.py                                 ← 本模块：真值 + 观测（无决策）
        ↓ import（只用不造）
dzbrain.tools.sim.step_bicycle                     ← 唯一推进真源（与下位机同构）
dzbrain.pursuit.path.Path.project                  ← 唯一投影真源
dzbrain.tasks.{localization,tracker,safety,mission} ← 唯一决策真源
```

## 铁门槛（判据①，机器判据在 `test_wb2_no_second_projection.py`）

本模块内**不许**出现 `def project` / `def _clamp` / `def _advance` 一类的自写实现。
唯一允许的坐标运算是"里程计生成"里那次**注入**（比例虚增 + 陀螺零偏 + 白噪声），
它是**观测生成器**而不是投影：它不回答"车在路线上的哪里"，只回答
"里程计这一拍会报什么数"。两者的区别在测试里有断言（见该文件 §2）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from dzbrain.core import geometry as geo
from dzbrain.pursuit.path import Path
from dzbrain.route.model import Route
from dzbrain.tools.sim import step_bicycle

from wb_rig.arms import ArmProfile

#: 20 Hz 定位重算时里程计帧的标称周期（s）。真车 odom 是 100 Hz、Localization 20 Hz，
#: 所以一次 Localization 重算之间有 5 帧里程计 —— 这个比值是**现场口径**，不是随便取的。
ODOM_PERIOD_S = 0.01


@dataclass
class TrueState:
    """**真值**车体状态（地图系）。只有世界模型与指标统计能读，控制器永远看不到。"""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v_exec: float = 0.0          # 本拍**实际执行**速度（已过执行器链）
    steer_exec: float = 0.0      # 本拍**实际执行**舵角（已过执行器链）
    t: float = 0.0

    def pose(self):
        return geo.Pose(self.x, self.y, self.yaw)

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)


@dataclass(frozen=True)
class Injection:
    """注入档案（由 `wb_rig.arms.ArmProfile` 转换而来，**唯一**注入来源）。"""

    speed_scale_err: float = 0.0   # +0.179 = 里程计比真实多报 17.9%
    yaw_drift_dps: float = 0.0     # 陀螺积分零偏（°/s）
    pos_noise_m: float = 0.0       # 位置白噪声（米，σ；每次重算叠一次）
    seed: int = 20261031

    @classmethod
    def of(cls, arm: ArmProfile, *, seed: int = 20261031) -> "Injection":
        return cls(speed_scale_err=arm.speed_scale_err, yaw_drift_dps=arm.yaw_drift_dps,
                   pos_noise_m=arm.pos_noise_m, seed=seed)

    @property
    def is_zero(self) -> bool:
        """零注入 = ideal 档。这条是 `belief ≡ truth` 强不变量的开关。"""
        return (self.speed_scale_err == 0.0 and self.yaw_drift_dps == 0.0
                and self.pos_noise_m == 0.0)


class DeterministicNoise:
    """**自备**的确定性白噪声（不依赖 `random` 模块的实现细节，跨 Python 版本可复现）。

    为什么不用 `random.Random`：CPython 的 Mersenne Twister 实现虽然稳定，但它
    "稳定"这件事没有写成契约；本卡要求**逐位复现**，所以用一条明确写死的
    LCG（数值检验见 `test_wb2_rig_invariants.py` §6）。同一 `seed` 永远同一条序列。
    """

    __slots__ = ("_state",)
    _A = 6364136223846793005
    _C = 1442695040888963407
    _M = 1 << 64

    def __init__(self, seed: int) -> None:
        self._state = (int(seed) * 2862933555777941757 + 3037000493) % self._M

    def next_u01(self) -> float:
        """[0,1) 均匀。"""
        self._state = (self._A * self._state + self._C) % self._M
        return (self._state >> 11) / float(1 << 53)

    def gauss(self) -> float:
        """标准正态（Box–Muller，用**两**个 u01；不用缓存，保证一一对应关系稳定）。"""
        u1 = max(self.next_u01(), 1e-12)
        u2 = self.next_u01()
        return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)


@dataclass
class OdomSample:
    """一帧**里程计**（= 桥 `pose()` 返回给 Localization 的那个数，带注入误差）。"""

    t: float
    x: float
    y: float
    yaw: float
    #: 本轮累计注入量（诊断用：让人看到"这一帧被加了什么"，而不是只能事后反推）
    scale_err: float = 0.0
    yaw_drift_rad: float = 0.0
    noise_x: float = 0.0
    noise_y: float = 0.0

    def pose(self):
        return geo.Pose(self.x, self.y, self.yaw)


class World:
    """世界模型：真值积分 + 里程计生成。

    生命周期（由 `tools/sim_rig.py` 驱动）::

        world = World(route=..., injection=..., actuator=..., start_pose=...)
        world.step(dt, cmd_v, cmd_steer, now)   # 每 WORLD_DT_S 一拍
        world.odom_frame(now)                  # 每 10 ms 一帧（100 Hz）
        world.truth.pose()                     # 指标统计用（**不进控制回路**）

    ★ 关键不变量（都有断言）：
      1. 零注入时 `odom.pose() == truth.pose()` **逐位相同**（不是"差很小"）；
      2. 真值**只**由 `step_bicycle` 推进（本类不写第二个积分公式）；
      3. `truth` 永远不写进黑盒/不写进桥 —— 控制器只能看见 `odom`。
    """

    def __init__(self, *, route: Route, injection: Injection, actuator=None,
                 start_pose: tuple[float, float, float] | None = None,
                 wheelbase_m: float = 0.157, smooth_window_speed: int | None = None,
                 seed: int | None = None, simulate_odom: bool = True) -> None:
        self.route = route
        self.path = Path(route)                     # ← 生产投影真源（唯一）
        self.injection = injection
        self.actuator = actuator
        self.wheelbase_m = float(wheelbase_m)
        self.simulate_odom = bool(simulate_odom)
        self._rng = DeterministicNoise(injection.seed if seed is None else seed)

        if start_pose is None:
            wp = route.waypoints[0]
            start_pose = (wp.x, wp.y, wp.yaw)
        self.truth = TrueState(x=float(start_pose[0]), y=float(start_pose[1]),
                               yaw=float(start_pose[2]))
        # ---- 里程计**自己的**位姿（航位推算；不是真值，只决定"报什么数"）----
        #   ★★ 为什么必须是**独立积分的一条轨迹**，而不是"在真值上叠加一个偏移"：
        #      底盘里程计是**航位推算**（编码器计数 → 位移 → 位置），比例误差会让它
        #      **沿它自己以为的路线**越走越偏。若只在真值上叠一个"沿当前航向"的偏移，
        #      在拐了 30 个弯的 24.85 m 路线上会得到一个**方向完全错**的报数
        #      （实测：belief 跑到 (3.07, 2.56)，真值在 (2.00, 3.15) —— 差 1.2 m，
        #      且这 1.2 m 的**方向**是"沿最后一段航向"，物理上不存在）。
        #      `sim.py` 的参考实现也是这么做的（同一个积分器、同一个执行量，
        #      信念侧把速度乘 `1+ε`）—— 本模块沿用同一条语义。
        self._odom_x = float(start_pose[0])
        self._odom_y = float(start_pose[1])
        self._odom_yaw = float(start_pose[2])
        self._odom_yaw_drift = 0.0                  # 累计 yaw 零漂（弧度）
        self._last_odom: OdomSample | None = None
        self._noise_xy = (0.0, 0.0)                 # 本拍噪声（每次重算才更新一次）
        self.odom_frames = 0
        self.steps = 0
        #: 真值轨迹（逐拍；指标算 RMS 用。`trace_every` 抽稀见 sim_rig）
        self.trace: list[tuple[float, float, float, float, float]] = []

    # ------------------------------------------------------------------ #
    #  推进（唯一积分入口 = 生产 step_bicycle）
    # ------------------------------------------------------------------ #
    def step(self, dt: float, cmd_v: float, cmd_steer: float, now: float) -> TrueState:
        """推进 `dt` 秒。

        `cmd_v`/`cmd_steer` 是**控制器下发**的命令；若挂了 `actuator`，这里先过执行器链，
        再用**实际执行量**积分（这正是"真实执行器链"的含义：命令不等于执行）。
        """
        if dt <= 0.0:
            raise ValueError(f"dt 必须 > 0，收到 {dt!r}")
        self.steps += 1
        if self.actuator is not None:
            st = self.actuator.tick(now)
            v_exec, steer_exec = st.v_exec, st.steer_exec
        else:
            v_exec, steer_exec = float(cmd_v), float(cmd_steer)

        t = self.truth
        nx, ny, nyaw = step_bicycle(t.x, t.y, t.yaw, v_exec, steer_exec, dt, self.wheelbase_m)
        t.x, t.y, t.yaw = nx, ny, nyaw
        t.v_exec, t.steer_exec, t.t = v_exec, steer_exec, float(now)
        # ---- 里程计自己的航位推算（**同一个积分器**，速度被比例误差放大）----
        #   沿用生产 `step_bicycle`（不写第二个积分公式）；`ideal` 档 ε=0 ⇒ 逐位等于真值。
        v_odom = v_exec * (1.0 + self.injection.speed_scale_err)
        # ★★ 陀螺零偏必须进**积分状态**，不是只在报数时叠一次。为什么：
        #   底盘航位推算是"用当前朝向把位移投影到地图"（`x += v cos(yaw) dt`）。
        #   陀螺零偏让**这个朝向**本身就是错的 ⇒ 位置会沿一条**弯掉**的轨迹累积偏离。
        #   只在"报数"时加漂移，位置就永远等同于真值（实测：单独注入 −0.0705°/s 时
        #   位置误差恒为 0.0000 —— 那是**模型漏了物理**，不是"注入没生效"）。
        #   ⚠️ 但每拍只能加**这一步**的增量（`dps × dt`）；早先误写成"每拍加累计值"
        #   ⇒ 偏航按 t² 增长，40 s 就把车带出走廊、漂移护栏直接停机。
        self._odom_x, self._odom_y, odom_yaw = step_bicycle(
            self._odom_x, self._odom_y, self._odom_yaw, v_odom, steer_exec, dt,
            self.wheelbase_m)
        step_drift = math.radians(self.injection.yaw_drift_dps) * dt
        self._odom_yaw_drift += step_drift
        self._odom_yaw = geo.normalize_angle(odom_yaw + step_drift)
        self.trace.append((float(now), t.x, t.y, t.yaw, v_exec))
        return t

    # ------------------------------------------------------------------ #
    #  观测生成（Localization 每拍读到的那个数）
    # ------------------------------------------------------------------ #
    def odom_frame(self, now: float, *, rng_refresh: bool = False) -> OdomSample:
        """生成一帧里程计。`rng_refresh=True` 时**推进一次**白噪声（Localization 重算用）。

        ★ 三类注入的物理出处（**不是拍脑袋**）：
          · 比例虚增：底盘 `ticksPerMeter` 标定偏大 ⇒ 同样的真实位移，编码器报得更多。
            实现上按"累计距离 × ε"整体加成一个**沿程**偏移，而不是每拍乘一次 ——
            后者的偏差会随拍数指数放大（那是模型 bug，不是被控对象特性）。
          · 陀螺零偏：**增量式**累计（`−0.0705 °/s × t`），不是瞬时跳变。
          · 位置白噪声：每次 Localization 重算叠一次（20 Hz），不是每 10 ms 叠一次 ——
            后者在 100 Hz 下会得到 10 倍大的等效噪声。
        """
        if not self.simulate_odom:
            # 直喂真值（= `--bench --drive` 的旧行为；只给对照用）
            return OdomSample(t=float(now), x=self.truth.x, y=self.truth.y, yaw=self.truth.yaw)
        if rng_refresh and self.injection.pos_noise_m > 0.0:
            sigma = self.injection.pos_noise_m
            self._noise_xy = (self._rng.gauss() * sigma, self._rng.gauss() * sigma)
        inj = self.injection
        # ① 航位推算位置（已在 `step()` 里按 `1+ε` 积分）
        # ② 陀螺零偏已进积分状态（`step()` 里逐拍累进；见那里的长注释）
        # ③ 位置白噪声：**叠加式**（不改积分状态）
        #    ⚠️ 与 `sim.py` 的一处**刻意差异**，必须写明：`sim.py:1093-1095` 把噪声
        #    `bx += gauss*σ` **直接写回积分状态** ⇒ 噪声会随积分路线的几何被放大/旋转，
        #    同一 σ 在不同航向上产生不同量级的等效误差。本台架只按**观测**加噪
        #    （不改 `_odom_x/_odom_y`），于是 σ 就是 σ，指标间可比。
        #    这也解释了为什么本台架不需要 `sim.py` 那条 `_POS_NOISE_LIMIT_M` 上限保护：
        #    观测式噪声没有正反馈路径。
        x = self._odom_x + self._noise_xy[0]
        y = self._odom_y + self._noise_xy[1]
        yaw = self._odom_yaw
        sample = OdomSample(t=float(now), x=x, y=y, yaw=yaw,
                            scale_err=float(inj.speed_scale_err),
                            yaw_drift_rad=self._odom_yaw_drift,
                            noise_x=self._noise_xy[0], noise_y=self._noise_xy[1])
        self._last_odom = sample
        self.odom_frames += 1
        return sample

    def last_odom(self) -> OdomSample | None:
        return self._last_odom

    # ------------------------------------------------------------------ #
    #  观测：横向/沿程（**投影走生产 Path.project**，不写第二份）
    # ------------------------------------------------------------------ #
    def project_truth(self, *, s_hint: float | None = None):
        """真值投影（**指标统计用**；不进控制回路）。"""
        return self.path.project(self.truth.x, self.truth.y, s_hint=s_hint,
                                 window_m=self.path.total_length, allow_backward=True)

    def project_belief(self, pose, *, s_hint: float | None = None, window_m: float = 0.12):
        """信念投影（诊断用；**注意**：它不是控制链里那一条 —— 控制链的投影在
        生产 `LocalizationTask._project` / `TrackerTask.tick` 内部，本台架不碰）。
        """
        return self.path.project(pose.x, pose.y, s_hint=s_hint, window_m=window_m,
                                 allow_backward=False)

    # ------------------------------------------------------------------ #
    #  诊断
    # ------------------------------------------------------------------ #
    def describe(self) -> str:
        inj = self.injection
        zero = "零注入（信念≡真值）" if inj.is_zero else (
            f"里程{inj.speed_scale_err:+.1%} 陀螺{inj.yaw_drift_dps:+.4f}°/s "
            f"噪声σ={inj.pos_noise_m * 1000:.1f}mm")
        return (f"世界模型：路线 {len(self.route)} 点 / {self.path.total_length:.2f} m · "
                f"{zero} · 轴距 {self.wheelbase_m:.3f} m · "
                f"{'执行器链 ON' if self.actuator is not None else '执行器链 OFF（直喂命令）'}")


__all__ = ["World", "TrueState", "OdomSample", "Injection", "DeterministicNoise", "ODOM_PERIOD_S"]
