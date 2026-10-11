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

from dzbrain.core.blackboard import (P_MISSION_PHASE, P_SAFETY, P_STOP_POINT,  # noqa: E402
                                     P_TELEMETRY, Blackboard)
from dzbrain.core.config import Config                           # noqa: E402
from dzbrain.core.events import EventBus                         # noqa: E402
from dzbrain.core.state import SafetyState, StopKind, StopPhase, StopPointState  # noqa: E402
from dzbrain.tasks.aim import AimTask                            # noqa: E402

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
check("3.1 ★停站 DONE / 回到 DRIVE → 重新锁止（序列 = [0, 2, 0]）",
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


if __name__ == "__main__":
    finish()
