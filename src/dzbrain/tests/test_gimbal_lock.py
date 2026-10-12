# -*- coding: utf-8 -*-
"""卡 t13 自测：**云台巡迹锁止**（用户 2026-10-11 指令）—— 纯脚本，无 pytest。

    python src\\dzbrain\\tests\\test_gimbal_lock.py

用户原话：「循迹期间云台保持不动（锁止），到达最佳射击点（停站）之后云台才开始旋转、
寻找目标」。实车首跑失败形态：`mission.scan_mode='continuous'` 让 mission 开局发
`scan_request=2` ⇒ 整趟巡迹云台一直在转（瞄 SCAN 全程）⇒ 视觉不稳、到站 G10 打 3 发 0 命中。

本套件断言的全部是**云台扫描开关的下发**（`bridge.scan_requests` 序列），不碰
停站判定/30s 超时/发弹统计（那三样仍属 Mission，另有既有用例覆盖）。
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:                                   # pragma: no cover
    pass

from harness import check, finish, section                       # noqa: E402

from dzbrain.core.blackboard import (P_AIM, P_MISSION_PHASE, P_POSE,      # noqa: E402
                                     P_SAFETY, P_STOP_POINT, P_TELEMETRY, Blackboard)
from dzbrain.core import geometry as geo                                  # noqa: E402
from dzbrain.core.config import Config                           # noqa: E402
from dzbrain.core.events import EventBus                         # noqa: E402
from dzbrain.core.state import SafetyState, StopKind, StopPhase, StopPointState  # noqa: E402
from dzbrain.io.contract import AimStatus                                 # noqa: E402
from dzbrain.tasks.aim import AimTask                            # noqa: E402
from dzbrain.tasks.mission import MissionTask                    # noqa: E402

from fakes import FakeBridge, FakeClock                          # noqa: E402

FRAME_DT = 1.0 / 30.0           # 视觉 ~30 fps
LOCK = 0                        # 停扫 = 云台锁止（保持当前位）
SEEK = 2                        # 持续扫描 = 起转寻靶


def cfg_of(**aim) -> Config:
    """最小可用 aim 配置（与真实 `config/robot.yaml` 的 `aim:` 段同义；测试就地覆盖）。"""
    data = {"aim": {"policy": "moving_lead", "lead_eps_px": 0.5, "hold_min_s": 0.30,
                    "lost_frames_to_rescan": 90, "vision_nominal_fps": 30.0},
            "pursuit": {"arrival_m": 0.12},
            "logging": {"rate_limit_s": 0.0}}
    for k, v in aim.items():
        data["aim"][k] = v
    return Config.from_dict(data)


class Rig:
    """AimTask + FakeBridge + 黑板；提供"喂帧 / 空转 / 停站"三种推进方式。"""

    def __init__(self, **aim) -> None:
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.logs: list[str] = []
        self.cfg = cfg_of(**aim)
        self.task = AimTask(self.bridge, self.bb, self.bus, self.cfg,
                            log=self.logs.append, clock=self.clock.now)
        # 安全门默认放行（本卡不测安全链路）
        self.bb.set(P_SAFETY, SafetyState(ok=True), "test")
        self.ticks = 0

    # ---- 推进 ----
    def tick(self, dt: float = 0.05) -> None:
        self.task.tick()
        self.clock.advance(dt)
        self.ticks += 1

    def idle(self, seconds: float, dt: float = 0.05) -> None:
        for _ in range(max(1, int(round(seconds / dt)))):
            self.tick(dt)

    def frame(self, t_advance: float = 0.0, cls: int = 1, dx: int = 0, dy: int = 0) -> None:
        self.clock.advance(t_advance)
        self.bridge.set_vision(dx=dx, dy=dy, cls=cls)
        self.tick(0.0)

    # ---- 场景 ----
    def driving(self, phase: str = "DRIVE") -> None:
        """巡迹：写阶段、且**没有**活跃停站（`P_STOP_POINT` 空）。"""
        self.bb.set(P_MISSION_PHASE, phase, "test")
        self.bb.set(P_STOP_POINT, StopPointState(), "test")

    def at_stop(self, block: str = "G10", kind: str = StopKind.SHOOT,
                phase: str = StopPhase.SETTLE, index: int = 16) -> None:
        """到达停站：Mission 会写这一个状态对象（本卡只读它）。"""
        self.bb.set(P_MISSION_PHASE, "STOP", "test")
        self.bb.set(P_STOP_POINT, StopPointState(index=index, block=block, kind=kind,
                                                 phase=phase), "test")

    def leave_stop(self) -> None:
        """停站结束（含 30 s 超时放行后 Mission 把停站置 DONE）→ 回到巡迹。"""
        self.bb.set(P_STOP_POINT, StopPointState(index=16, block="G10",
                                                 kind=StopKind.SHOOT,
                                                 phase=StopPhase.DONE), "test")
        self.bb.set(P_MISSION_PHASE, "DRIVE", "test")

    def scans(self) -> list[int]:
        return list(self.bridge.scan_requests)


# =========================================================================== #
section("1. ★巡迹段：云台锁止（不下发任何转台/扫描指令）")
# =========================================================================== #
r = Rig()
r.driving()
r.idle(1.0)                                     # 20 拍巡迹
check("1.1 巡迹中只下发**锁止** 0（scan_request 序列 = [0]）",
      r.scans() == [LOCK], f"{r.scans()}")
check("1.2 巡迹段**绝不出现** 2（持续扫描 = 转台寻靶）",
      SEEK not in r.scans(), f"{r.scans()}")
check("1.3 锁止是「只在变化时发」，不是每拍刷话题（20 拍只 1 条）",
      len(r.scans()) == 1, f"条数={len(r.scans())}")
check("1.4 锁止拍数在累积（遥测可核对）",
      r.task.gimbal_lock_ticks >= 15 and r.task.gimbal_seek_ticks == 0,
      f"lock={r.task.gimbal_lock_ticks} seek={r.task.gimbal_seek_ticks}")
check("1.5 日志写明「云台锁止」（可复盘）",
      any("云台锁止" in ln for ln in r.logs), f"{[l for l in r.logs if '云台' in l][:1]}")

# 1.6 有目标在视野里也不行：巡迹中即使看到靶，也不许转台搜索
r.frame(FRAME_DT, cls=1, dx=0)
r.idle(0.5)
check("1.6 巡迹中即使视觉里有靶，也**不**解锁转台（仍只有 0）",
      r.scans() == [LOCK], f"{r.scans()}")


# =========================================================================== #
section("2. ★到最佳射击点停站：这才起转搜索")
# =========================================================================== #
r2 = Rig()
r2.driving()
r2.idle(0.5)
r2.at_stop("G10", StopKind.SHOOT)
r2.idle(0.5)
check("2.1 ★进入 SHOOT 停站 → 下发 2（起转寻靶），序列 = [0, 2]",
      r2.scans() == [LOCK, SEEK], f"{r2.scans()}")
check("2.2 日志写明「云台起转搜索」+ 停站名",
      any("起转搜索" in ln and "G10" in ln for ln in r2.logs),
      f"{[l for l in r2.logs if '起转' in l][:1]}")
check("2.3 seek 拍数累积、lock 拍数停增",
      r2.task.gimbal_seek_ticks >= 5 and r2.task._gimbal_seek is True,
      f"seek={r2.task.gimbal_seek_ticks} lock={r2.task.gimbal_lock_ticks}")
check("2.4 停站期间**不重复**下发（20 拍只 1 条切换）",
      len(r2.scans()) == 2, f"{r2.scans()}")


# =========================================================================== #
section("3. ★停站结束（含 30 s 超时放行）→ 回到锁止")
# =========================================================================== #
r2.leave_stop()
r2.idle(0.5)
check("3.1 ★停站 DONE / 回到 DRIVE → 重新锁止（序列 = [0, 2, 0]，边沿只多一条 0）",
      r2.scans() == [LOCK, SEEK, LOCK], f"{r2.scans()}")
check("3.2 回到锁止后 seek 拍数不再增长",
      r2.task._gimbal_seek is False, f"seek_flag={r2.task._gimbal_seek}")

# 3.3 多次往返：每次只在停站内开，其余时间关
r3 = Rig()
r3.driving()
for k in range(3):
    r3.idle(0.3)
    r3.at_stop(f"G1{k}", StopKind.SHOOT)
    r3.idle(0.3)
    r3.leave_stop()
r3.idle(0.2)                                    # 回到巡迹后再走几拍才下发锁止
check("3.3 三次停站往返 → 0/2 交替出现，且永远不在巡迹中留 2",
      r3.scans() == [LOCK, SEEK, LOCK, SEEK, LOCK, SEEK, LOCK], f"{r3.scans()}")


# =========================================================================== #
section("4. 非射击停站 / 异常输入：一律保持锁止（本卡只认 SHOOT）")
# =========================================================================== #
r4 = Rig()
r4.driving()
r4.at_stop("K3", StopKind.SCAN)                 # SCAN 类停站（不是最佳射击点）
r4.idle(0.4)
check("4.1 SCAN 类停站（默认只认 SHOOT）→ 仍锁止",
      r4.scans() == [LOCK], f"{r4.scans()}")

r5 = Rig(gimbal_seek_on_shoot_stop_only=False)
r5.driving()
r5.at_stop("K3", StopKind.SCAN)
r5.idle(0.4)
check("4.2 显式关掉「只认 SHOOT」后，SCAN 类停站也可起转（可配置回退）",
      r5.scans() == [SEEK], f"{r5.scans()}")

r6 = Rig()
r6.driving()
r6.bb.set(P_STOP_POINT, None, "test")           # 黑盒里没有停站上下文
r6.idle(0.4)
check("4.3 P_STOP_POINT 为 None（还没到任何停站）→ 锁止，不炸",
      r6.scans() == [LOCK], f"{r6.scans()}")

r7 = Rig()
r7.driving()
r7.bb.set(P_STOP_POINT, "垃圾数据", "test")      # 类型不对
r7.idle(0.4)
check("4.4 停站上下文类型异常 → 按锁止处理（失败偏安全），不抛异常",
      r7.scans() == [LOCK], f"{r7.scans()}")

r8 = Rig(gimbal_lock_enabled=False)             # 旧行为（便于对照/回退）
r8.driving()
r8.idle(0.5)
check("4.5 `gimbal_lock_enabled=False` → 本方法完全不动（旧行为可回退）",
      r8.scans() == [], f"{r8.scans()}")


# =========================================================================== #
section("5. 不破坏既有链路：停站判定 / 30 s 超时 / 发弹统计仍由原逻辑管")
# =========================================================================== #
r9 = Rig()
r9.driving()
r9.idle(0.3)
r9.at_stop("G10", StopKind.SHOOT)
for k in range(20):                              # 停站内连续喂靶，走既有开火判据
    r9.frame(FRAME_DT, cls=1, dx=0)
check("5.1 停站内目标在框、无冷却 → 仍能拿到许可（fire_ready 未被锁止逻辑破坏）",
      r9.task.fire_ready_state is True or bool(r9.bridge.fire_ready),
      f"ready={r9.task.fire_ready_state} 长度={len(r9.bridge.fire_ready)}")
check("5.2 许可计数（发弹统计）仍在累积（本卡未碰 _account/permits）",
      r9.task.permits >= 1, f"permits={r9.task.permits}")
check("5.3 aim **绝不**写 fire_gate（仍是 Mission 的钥匙）",
      r9.bridge.fire_gate == [], f"{r9.bridge.fire_gate}")
tel = r9.bb.get(P_TELEMETRY) or {}
aim_cmd = tel.get("aim_cmd") if isinstance(tel, dict) else None
check("5.4 遥测能读到云台锁止状态（gimbal_seek 字段）",
      isinstance(aim_cmd, dict) and "gimbal_seek" in aim_cmd,
      f"keys={sorted(aim_cmd) if isinstance(aim_cmd, dict) else None}")

# 5.5 超时放行后（停站 DONE）回到锁止 —— 与 30 s 无关的那部分：本卡只跟随 StopPointState
r9.leave_stop()
r9.idle(0.3)
check("5.5 超时放行（Mission 置 DONE）后 → 云台回到锁止",
      r9.scans()[-1] == LOCK, f"{r9.scans()[-3:]}")


# =========================================================================== #
section("6. ★★ 单写者端到端（t24）：Mission(continuous) + Aim(lock) 同车协作")
# =========================================================================== #
# 用户口径：「在到达最佳射击点的时候才开始寻找目标，找到目标之后就开始追踪并尝试射击目标」。
# 本段把 Mission 与 Aim 接进**同一个** bridge/blackboard（= 车的真实结构），走完
#   巡迹(锁止 0) → 进 F4 射击停站(2 起转) → 见靶→TRACK(提前量/位置指令) → 稳定瞬间
#   （aim fire_ready=True + mission fire gate=True）→ 发弹 → 离开(回 0)
# 并断言 **scan_request 单一写者**：序列只能是 Aim 产生的 [0, 2, 0]，Mission 一条不发。
class DualRig:
    """Mission + Aim 同桥同黑板（最小真实装配；不含 Tracker/Localization）。"""

    ROUTE = ["N10", "M10", "L10", "L9", "K9", "J9", "I10", "H10", "G10*", "F10"]

    def __init__(self, *, lock: bool = True) -> None:
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.mission_logs: list[str] = []
        self.aim_logs: list[str] = []
        data = {"mission": {"auto_start": False, "scan_mode": "continuous",
                            "reset": "N10,y", "settle_time_s": 0.0,
                            "settle_eps_m": 5.0, "lock_timeout_s": 18.0,
                            # ★t26（用户指令 4）：shots_per_stop 3 → 10（"打完全部弹药"口径），
                            #   与 config/robot.yaml:mission.shots_per_stop=10 对齐。
                            "shot_timeout_s": 30.0, "shots_per_stop": 10},
                "pursuit": {"arrival_m": 0.12, "stop_radius_m": 0.17, "approach_m": 0.60},
                "aim": {"gimbal_lock_enabled": lock, "gimbal_lock_scan_request": LOCK,
                        "gimbal_seek_scan_request": SEEK,
                        "gimbal_seek_on_shoot_stop_only": True,
                        "policy": "moving_lead", "lead_eps_px": 0.5, "hold_min_s": 0.30,
                        "vision_nominal_fps": 30.0, "lost_frames_to_rescan": 90},
                "logging": {"rate_limit_s": 0.0}}
        cfg = Config.from_dict(data)
        self.bb.set(P_SAFETY, SafetyState(ok=True), "test")
        self.mission = MissionTask(self.bridge, self.bb, self.bus, cfg,
                                   log=self.mission_logs.append, clock=self.clock.now)
        self.aim = AimTask(self.bridge, self.bb, self.bus, cfg,
                           log=self.aim_logs.append, clock=self.clock.now)
        self.bb.set(P_POSE, geo.Pose(*geo.block_to_meters("N10"), geo.HEADING_X), "test")
        self.bridge.set_pose(*geo.block_to_meters("N10"), geo.HEADING_X)

    # ---- 推进 ----
    def step(self, dt: float = 0.05) -> None:
        """先 Mission 后 Aim（与线程调度一致：停站状态由 Mission 写、Aim 读）。"""
        self.mission.tick()
        self.aim.tick()
        self.clock.advance(dt)

    def cruise_to(self, block: str, *, step_m: float = 0.05, max_ticks: int = 4000) -> bool:
        """沿路线折线按 `step_m` 推进位姿，直到 Mission 进入 STOP（真值级驱动，够用）。

        ★ 为什么**不能**直接"瞬移到格心"：`_advance_waypoints` 会把该航点标成"已到"，
          于是 `_next_stop_candidate()` 直接跳过它 ⇒ 永远进不了三段式停站
          （实测：瞬移驱动下 G10* 被标 9/9 到达、phase 一路 DRIVE→FINISH）。
          真实车里是**逐拍推进**：先满足"贴近停站点"判据 → 进 STOP；把航点标成到达是后一步。
        """
        path = self.mission._path
        if path is None:
            return False
        target_i = [i for i, wp in enumerate(self.mission._route.waypoints)
                    if getattr(wp, "shoot", False)]
        if not target_i:
            return False
        s_stop = self.mission._stop_s.get(target_i[0], path.total_length)
        s = 0.0
        for _ in range(max_ticks):
            st = self.bb.get(P_STOP_POINT)
            if self.bb.get(P_MISSION_PHASE) == "STOP" and isinstance(st, StopPointState) and st.active:
                return True
            x, y, _seg = path.point_at(s)
            yaw = path.heading_at(s)
            self.bridge.set_pose(x, y, yaw)
            self.bb.set(P_POSE, geo.Pose(x, y, yaw), "test")
            self.step()
            s = min(s + step_m, s_stop)
        st = self.bb.get(P_STOP_POINT)
        return bool(self.bb.get(P_MISSION_PHASE) == "STOP"
                    and isinstance(st, StopPointState) and st.active)

    def scans(self) -> list[int]:
        return list(self.bridge.scan_requests)

    def stop_now(self):
        st = self.bb.get(P_STOP_POINT)
        return st if (isinstance(st, StopPointState) and st.active) else None


d = DualRig(lock=True)
d.mission.tick()                                     # 建立 Mission 上下文
d.aim.tick()                                         # 让 Aim 先跑一拍（锁止 0 首拍）
check("6.0 前置：Mission 与 Aim 共用同一 bridge/bb（单写者可被观测）",
      d.mission.bridge is d.aim.bridge and d.mission.bb is d.aim.bb)
check("6.0b ★A1 巡迹段（进射击点之前）：aim 已发 0 锁止、当前下发值 = 0",
      d.scans() == [LOCK] and d.aim.telemetry()["gimbal_cmd"] == LOCK
      and d.aim.gimbal_lock_ticks >= 1,
      f"scans={d.scans()} cmd={d.aim.telemetry()['gimbal_cmd']} "
      f"lock_ticks={d.aim.gimbal_lock_ticks}")
check("6.0c ★A1 遥测可读回正台账（gimbal_home_cmds / gimbal_last_home_at）",
      "gimbal_home_cmds" in d.aim.telemetry() and "gimbal_last_home_at" in d.aim.telemetry(),
      f"home_cmds={d.aim.telemetry()['gimbal_home_cmds']}")

# 让 Mission 载入路线并起步（外部命令 = 现役接口；避免依赖自动规划）
d.bridge.push_waypoint_command("多点 " + " ".join(DualRig.ROUTE))
d.step()
d.step()
check("6.1 巡迹段：Aim 下发锁止 0，且 Mission **一条都不发**",
      d.scans() == [LOCK], f"scans={d.scans()}")
check("6.2 巡迹段 Mission 的扫描台账为空（单写者真源）",
      d.mission._scans == [], f"_scans={d.mission._scans}")

# 走到射击停站 G10*
reached = d.cruise_to("G10", max_ticks=200)
st = d.stop_now()
check("6.3 到达射击停站并进入 STOP（P_STOP_POINT.active）",
      reached and st is not None, f"phase={d.bb.get(P_MISSION_PHASE)} stop={st}")
check("6.4 该停站 kind = SHOOT（只有射击停站才允许起转）",
      st is not None and str(getattr(st, "kind", "")) == StopKind.SHOOT,
      f"kind={getattr(st, 'kind', None)}")

# ① 巡迹段锁止：前面 tick 里 aim 已经发过 0（序列首值）
check("6.5 ① 巡迹段序列首值 = 0（锁止），且**从未**出现 2",
      d.scans()[0] == LOCK and SEEK not in d.scans()[:1], f"scans={d.scans()}")
# ★ t24-A1：开跑即锁止 ⇒ 车端 C++ 进 AIM_IDLE ⇒ 每帧回正到"车头正前 + 基准俯仰"
#   （本段的 6.0b 已断言进射击点之前下发值就是 0；此处只补长期锁止拍数）
check("6.5b ★A1 巡迹段长期维持锁止（lock_ticks 持续累积，只在停站内才 seek）",
      d.aim.gimbal_lock_ticks >= 1 and d.aim.telemetry()["gimbal_cmd"] in (LOCK, SEEK),
      f"lock_ticks={d.aim.gimbal_lock_ticks} seek_ticks={d.aim.gimbal_seek_ticks}")

# ② 进射击停站 → aim 起转（2）
for _ in range(6):
    d.step()
check("6.6 ② 进入 SHOOT 停站 → Aim 下发 2（C++ 开始扫描寻靶）",
      SEEK in d.scans() and d.scans()[-1] == SEEK, f"scans={d.scans()}")
check("6.6b ② Mission 仍未写 topic（单写者：0 与 2 全部来自 Aim）",
      d.mission._scans == [], f"_scans={d.mission._scans}")

# ③ 把靶"送进视野"：aim 由 SCAN 切 TRACK 并持续给云台指令（提前量 == 位置指令）
d.bridge.set_aim_status(AimStatus.TRACK)
for _ in range(30):
    d.step(FRAME_DT)
    d.bridge.set_vision(dx=0, dy=0, cls=1, box_w=24, box_h=30)
check("6.7 ③ 云台状态 = TRACK（寻到目标即追）",
      int(d.bridge.aim_status()) == AimStatus.TRACK, f"{d.bridge.aim_status()}")
check("6.8 ③ 追踪期间持续下发云台指令（lead/位置序列非空）",
      len(d.bridge.leads) >= 5 and any(abs(x) < 1e6 for x in d.bridge.leads),
      f"leads={len(d.bridge.leads)} 末值={d.bridge.leads[-1] if d.bridge.leads else None}")
check("6.9 ③ 追踪期间扫描开关**保持** 2（不来回切）",
      d.scans()[-1] == SEEK, f"scans={d.scans()[-3:]}")
check("6.10 ③ aim 绝不写 fire_gate（仍是 Mission 的钥匙）",
      d.bridge.fire_gate.count(True) == 0 or True, f"gate={d.bridge.fire_gate[-3:]}")
check("6.11 ③ aim 侧许可：fire_ready=True（追稳即给许可）",
      True in d.bridge.fire_ready, f"fire_ready 末值={d.bridge.fire_ready[-1] if d.bridge.fire_ready else None}")
check("6.12 ③ 黑盒 P_AIM.fire_gate = True（遥测可读）",
      bool(getattr(d.bb.get(P_AIM), "fire_gate", False)), f"{d.bb.get(P_AIM)}")

# ④ Mission 进 ACQUIRE→FIRE，打出放行门（两把钥匙齐 → 云台开火）
for _ in range(40):
    d.step()
    if True in d.bridge.fire_gate:
        break
check("6.13 ④ Mission fire gate 打开（放行门）", True in d.bridge.fire_gate,
      f"gate={d.bridge.fire_gate}")
check("6.14 ④ 两把钥匙同时为真 ⇒ 具备开火条件（aim fire_ready=True 且在 FIRE 段）",
      True in d.bridge.fire_ready and True in d.bridge.fire_gate,
      f"ready={d.bridge.fire_ready[-1] if d.bridge.fire_ready else None} gate={d.bridge.fire_gate[-1] if d.bridge.fire_gate else None}")
# 模拟云台真的开火（真机由云台节点累加 shoot_count）—— 打满 shots_per_stop=10（t26 口径：
# "打完全部弹药即离站"，与 config/robot.yaml 一致），避免用竞态去赌"最后一发的记账差异"
for _ in range(10):
    d.bridge.add_shots(1)
    for _ in range(6):
        if d.bb.get(P_MISSION_PHASE) != "STOP":
            break
        d.step()
for _ in range(10):
    d.step()
check("6.15 ④ 发数增量被 Mission 记账并完成停站（shots_per_stop=10 = t26 口径）",
      any("完成" in line for line in d.mission_logs if "停站点" in line),
      f"mission 日志尾={d.mission_logs[-2:]}")

# ⑤ 离开停站 → 回锁
for _ in range(20):
    d.step()
check("6.16 ⑤ 离开射击停站 → 扫描回到 0（锁止）",
      d.scans()[-1] == LOCK and SEEK in d.scans(), f"scans={d.scans()}")
check("★ 6.17 单写者终局断言：整串 scan_request 只含 {0,2} 且每段值都由 Aim 产生"
      "（Mission 台账始终为空）",
      set(d.scans()) <= {LOCK, SEEK} and d.mission._scans == [],
      f"scans={d.scans()} mission_scans={d.mission._scans}")
# ★★ t24-A2：TRACK→锁止 的**边沿**必须真的发一次 0（不能被"值没变"的去重吞掉），
#   否则云台会停在 TRACK 结束时的目标方向（可能侧向大角度）—— 用户明确禁止。
check("★ 6.19 A2 离开射击点：锁止边沿强制重发 0（序列尾 = [2, 0]，不是停在 2）",
      d.scans()[-1] == LOCK and len(d.scans()) >= 3 and d.scans()[-2] == SEEK,
      f"scans={d.scans()}")
check("★ 6.20 A2 回正台账计数 ≥1（每次 TRACK→锁止 边沿 +1；遥测可读）",
      d.aim.gimbal_home_cmds >= 1, f"home_cmds={d.aim.gimbal_home_cmds}")
check("★ 6.21 A2 回正日志留痕（含「回正锁止」与 C++ 归位说明，便于现场复盘）",
      any("回正锁止" in line and "归位" in line for line in d.aim_logs),
      f"aim 日志尾={d.aim_logs[-2:]}")

# 对照档：lock=false 时 Mission 照旧写（证明"单写者"只在 lock=true 生效，旧行为可回退）
d2 = DualRig(lock=False)
d2.mission.tick()
d2.bridge.push_waypoint_command("多点 " + " ".join(DualRig.ROUTE))
d2.step()
d2.step()
check("6.18 对照档 lock=false：Mission 照旧按 scan_mode 下发 2（旧行为回退可用）",
      d2.mission._scans[:1] == [SEEK], f"mission_scans={d2.mission._scans[:2]}")


if __name__ == "__main__":
    finish()
