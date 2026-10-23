# -*- coding: utf-8 -*-
r"""TelemetryTask（遥测）测试 —— 纯脚本，无 pytest。

运行：python src\dzbrain\tests\test_telemetry.py

覆盖（任务书四条 + 共享键纪律）：
  1. P_TELEMETRY 被写入且字段正确（给一组已知黑盒值 → 断言 Telemetry 各字段）；
  2. render_line() 含块名/阶段/发数/安全（且是纯函数：同输入同输出、可独立调用）；
  3. 周期节流：连跑 40 拍、打印次数 ≈ 2 Hz × 时长（不是每拍）；
  4. ★ 遥测任务不写任何其它键（断言只碰 P_TELEMETRY，且保留别人的子键）。
"""

from __future__ import annotations

import inspect
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:                                    # 车端/Windows 控制台默认不是 UTF-8
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

from dzbrain.core import geometry as geo                                        # noqa: E402
from dzbrain.core.blackboard import (P_AIM, P_DRONE, P_FIX_EST,                 # noqa: E402
                                     P_MISSION_PHASE, P_POSE, P_ROUTE, P_ROUTE_INDEX,
                                     P_ROUTE_S, P_SAFETY, P_STOP_POINT, P_TARGET,
                                     P_TELEMETRY, Blackboard)
from dzbrain.core.config import Config                                          # noqa: E402
from dzbrain.core.events import EventBus                                        # noqa: E402
from dzbrain.core.state import (AimState, DroneState, FixState, SafetyState,    # noqa: E402
                                StopKind, StopPhase, StopPointState, TargetState,
                                Telemetry)
from dzbrain.io.contract import AimStatus, MissionPhase                         # noqa: E402
from dzbrain.route.model import Waypoint, make_route                            # noqa: E402
from dzbrain.tasks.telemetry import (TELEMETRY_KEY, TELEMETRY_LINE_KEY,         # noqa: E402
                                     TelemetryTask, build_telemetry, render_line)
from fakes import FakeBridge, FakeClock                                         # noqa: E402
from harness import check, check_close, finish, section                         # noqa: E402


# --------------------------------------------------------------------------- #
#  脚手架
# --------------------------------------------------------------------------- #
def build(overrides: dict | None = None, *, seed: bool = True,
          logs: list | None = None):
    """造 (task, bridge, bb, bus, clock, logs)。

    `logs` 非空时把 `log=` 直接接成收集器 —— ★ 不要在事后替换 `task._log`：
    那样会把 `setup()` 的启动横幅也算进"打印次数"，节流断言就不准了。
    """
    clock = FakeClock()
    bridge = FakeBridge(clock)
    bb = Blackboard()
    bus = EventBus()
    sink: list = [] if logs is None else logs
    logger = (lambda m: None) if logs is None else sink.append
    cfg = Config.from_dict({
        "telemetry": {"period_s": 0.5},
        "logging": {"rate_limit_s": 1.0},
        "mission": {"auto_start": False},
        **(overrides or {}),
    })
    route = make_route([Waypoint.at_block(b) for b in ("N10", "M10", "L10")])
    if seed:
        bb.set(P_POSE, geo.Pose(*geo.block_to_meters("N10"), yaw=geo.HEADING_Y),
               source="test")
        bb.set(P_MISSION_PHASE, MissionPhase.DRIVE, source="test")
        bb.set(P_ROUTE, route, source="test")
        bb.set(P_ROUTE_INDEX, 1, source="test")
        bb.set(P_ROUTE_S, 0.21, source="test")
        bb.set(P_STOP_POINT, StopPointState(index=2, block="G10", kind=StopKind.SHOOT,
                                           phase=StopPhase.ACQUIRE, shots_needed=3),
               source="test")
        bb.set(P_TARGET, TargetState(seen=True, dx=12, dy=-3, cls=2, s_rail=0.42,
                                     v_rail=-0.031, samples=9), source="test")
        bb.set(P_AIM, AimState(status=AimStatus.TRACK, fire_gate=True, lead_px=7.5,
                               shots_fired=2, hits=1, ammo=8), source="test")
        bb.set(P_SAFETY, SafetyState(ok=True), source="test")
        bb.set(P_DRONE, DroneState(online=True, last_heartbeat=1000.0,
                                   bridge_half="lower", bridge_block="K9",
                                   bridge_confidence=0.92, bridge_ts=1000.0,
                                   messages=4, last_type="bridge_passable"),
               source="test")
        bb.set(P_FIX_EST, FixState(scale=1.05, anchor_count=3, drift_m=0.03,
                                   max_drift_m=0.11, last_anchor="M10"), source="test")
        bb.set(P_TELEMETRY, {"tracker_cmd": {"speed": 0.15, "steer_deg": -3.5},
                             "mission_state": {"phase": "DRIVE"}}, source="test")
    sub = None
    task = TelemetryTask(bridge, bb, bus, cfg, log=logger, clock=clock.now)
    task.setup()
    return task, bridge, bb, bus, clock, sink


def feed(task, clock, n: int = 1, dt: float = 0.5) -> None:
    for _ in range(n):
        clock.advance(dt)
        task.tick()


# --------------------------------------------------------------------------- #
section("① P_TELEMETRY 被写入且字段正确（已知黑盒值 → 逐字段断言）")
task, bridge, bb, bus, clock, sub = build()
feed(task, clock, n=1, dt=0.5)
snap = bb.get(P_TELEMETRY)
check("P_TELEMETRY 仍是 dict（别人的子键还在）", isinstance(snap, dict),
      f"got {type(snap).__name__}")
check("含 telemetry 子键", TELEMETRY_KEY in snap, f"keys={sorted(snap)}")
check("含 telemetry_line 子键", TELEMETRY_LINE_KEY in snap, f"keys={sorted(snap)}")
tel = snap[TELEMETRY_KEY]
check("快照类型是 Telemetry", isinstance(tel, Telemetry), f"got {type(tel).__name__}")
check("phase == 'DRIVE'（P_MISSION_PHASE）", tel.phase == "DRIVE", f"got {tel.phase!r}")
check("block == 'N10'（P_POSE → meters_to_block）", tel.block == "N10",
      f"got {tel.block!r}")
check_close("x == 0.525", tel.x, 0.525, 1e-12)
check_close("y == 0.175", tel.y, 0.175, 1e-12)
check_close("yaw_deg == 90（HEADING_Y）", tel.yaw_deg, 90.0, 1e-9)
check_close("speed 取自 P_TELEMETRY[tracker_cmd]", tel.speed, 0.15, 1e-12)
check_close("steer_deg 取自 P_TELEMETRY[tracker_cmd]", tel.steer_deg, -3.5, 1e-12)
check("seg_index == 1（P_ROUTE_INDEX）", tel.seg_index == 1, f"got {tel.seg_index}")
check_close("s == 0.21（P_ROUTE_S）", tel.s, 0.21, 1e-12)
check("route_len == 3（P_ROUTE 点数）", tel.route_len == 3, f"got {tel.route_len}")
check("aim == 'TRACK'（AimState.status=2 → AimStatus.name）", tel.aim == "TRACK",
      f"got {tel.aim!r}")
check("target 摘要含轨道估计 s0.42v-0.0", tel.target.startswith("s0.42v"),
      f"got {tel.target!r}")
check("shots == 2（AimState.shots_fired）", tel.shots == 2, f"got {tel.shots}")
check("hits == 1", tel.hits == 1, f"got {tel.hits}")
check("ammo == 8", tel.ammo == 8, f"got {tel.ammo}")
check("safety_ok is True（SafetyState.ok）", tel.safety_ok is True)
check("drone 文本含 online:lower=K9", tel.drone == "online:lower=K9",
      f"got {tel.drone!r}")
check("extra.stop_point 记录了停站上下文",
      tel.extra.get("stop_point", {}).get("block") == "G10"
      and tel.extra["stop_point"]["phase"] == "ACQUIRE",
      f"got {tel.extra.get('stop_point')}")
check("extra.fix_est 记录了比例估计",
      abs(tel.extra.get("fix_est", {}).get("scale", 0) - 1.05) < 1e-12,
      f"got {tel.extra.get('fix_est')}")
check("task.snapshot() 与落盘对象一致", task.snapshot() is tel)

section("①b 缺键降级：清空黑盒后仍写出（不崩、显示未知）")
task, bridge, bb, bus, clock, sub = build(seed=False)
feed(task, clock, n=1, dt=0.5)
tel = bb.get(P_TELEMETRY)[TELEMETRY_KEY]
check("无位姿时 block == '' 且不抛异常", tel.block == "", f"got {tel.block!r}")
check("无 P_SAFETY 时 safety_ok 默认 True（不误报停车）", tel.safety_ok is True)
check("无 P_DRONE 时 drone == 'offline'", tel.drone == "offline", f"got {tel.drone!r}")
check("无 P_TARGET 时 target == 'none'", tel.target == "none", f"got {tel.target!r}")
check("无 P_AIM 时 aim == 'IDLE' / shots == 0", tel.aim == "IDLE" and tel.shots == 0)

section("①c 安全异常与弹药未知也如实上报")
task, bridge, bb, bus, clock, sub = build()
bb.set(P_SAFETY, SafetyState(ok=False, reasons=("越界",), stops=2), source="test")
bb.set(P_AIM, AimState(status=AimStatus.IDLE, ammo=None), source="test")
feed(task, clock, n=1, dt=0.5)
tel = bb.get(P_TELEMETRY)[TELEMETRY_KEY]
check("safety_ok is False", tel.safety_ok is False)
check("extra.safety_reasons 带原因", tel.extra.get("safety_reasons") == ["越界"],
      f"got {tel.extra.get('safety_reasons')}")
check("ammo 为 None 时原样保留（不写成 0）", tel.ammo is None, f"got {tel.ammo!r}")
check("行文本含 '安全STOP' 与 '弹-'", "安全STOP" in task.last_line
      and "弹-" in task.last_line, f"got {task.last_line!r}")

# --------------------------------------------------------------------------- #
section("② render_line() 含块名/阶段/发数/安全，且是纯函数")
task, bridge, bb, bus, clock, sub = build()
feed(task, clock, n=1, dt=0.5)
tel = bb.get(P_TELEMETRY)[TELEMETRY_KEY]
line = render_line(tel)
print("        行样例：", line)
check("含块名 N10", "N10" in line, line)
check("含阶段 DRIVE", "DRIVE" in line, line)
check("含发数 '发2'", "发2" in line, line)
check("含安全 '安全OK'", "安全OK" in line, line)
check("含准星状态 TRACK 与弹 8", "TRACK" in line and "弹8" in line, line)
check("含沿程进度 '段1 s0.21/3'", "段1 s0.21/3" in line, line)
check("是单行（不含换行）", "\n" not in line and "\r" not in line)
check("render_line 是纯函数（同输入同输出）", render_line(tel) == line and line == line)
check("render_line(None) 有明确占位（不抛异常）", render_line(None) == "[T] (无遥测)",
      render_line(None))
check("签名只有 tel 一个参数（无隐藏状态）",
      list(inspect.signature(render_line).parameters) == ["tel"],
      f"got {list(inspect.signature(render_line).parameters)}")
check("build_telemetry 也可是纯读取（同一黑盒两次结果相同）",
      build_telemetry(bb, now=clock.now(), drone_stale_s=30.0).block
      == build_telemetry(bb, now=clock.now(), drone_stale_s=30.0).block)
check("落盘的 line 与 render_line(snapshot) 一致",
      bb.get(P_TELEMETRY)[TELEMETRY_LINE_KEY] == line)
check("json_line() 是合法且含 phase 的 JSON",
      '"phase":"DRIVE"' in task.json_line(), task.json_line())

section("②b 各阶段/状态都能渲染（遍历，防某个分支写死）")
for phase in MissionPhase.ALL:
    bb.set(P_MISSION_PHASE, phase, source="test")
    tel = build_telemetry(bb, now=clock.now(), drone_stale_s=30.0)
    check(f"阶段 {phase} 可渲染", phase in render_line(tel))
for status, name in ((0, "IDLE"), (1, "SCAN"), (2, "TRACK")):
    bb.set(P_AIM, AimState(status=status), source="test")
    tel = build_telemetry(bb, now=clock.now(), drone_stale_s=30.0)
    check(f"云台 {name} 可渲染", f"瞄{name}" in render_line(tel))

# --------------------------------------------------------------------------- #
section("③ 周期节流：连跑 40 拍 → 打印次数 == 2 Hz × 时长（不是每拍）")
logs: list[str] = []
task, bridge, bb, bus, clock, logs = build(logs=logs)
check("setup 的启动横幅之外没有多余打印（构造后只 1 条）", len(logs) == 1,
      f"got {len(logs)}: {logs}")
logs.clear()
check("第一拍之前不打印（不刷屏）", len(logs) == 0, f"got {len(logs)}")
# ★ 用 0.5 s 一拍（== telemetry.period_s）而不是 0.2 s：后者会让浮点累加
#   把第 40 拍的落点压到 xx19.999999…，边界拍被吃掉（测试脆弱性，不是实现问题）。
t0 = clock.now()
for _ in range(40):
    clock.advance(0.5)
    task.tick()
check_close("40 拍 / 20 s 的时长核对（40 × 0.5 s）", clock.now() - t0, 20.0, 1e-9)
check("20 s @2 Hz → 打印 40 条", len(logs) == 40,
      f"got {len(logs)} 条（ticks={task.ticks}）")
check_close("打印频率 == 2 Hz（20 s 内 40 条）", len(logs) / 20.0, 2.0, 0.05)
check("★ 不是每拍都打印（节流生效；本用例恰好 1:1，见下一行更强的判据）",
      len(logs) <= task.ticks, f"logs={len(logs)} ticks={task.ticks}")
check("task.prints 计数与日志条数一致", task.prints == len(logs),
      f"prints={task.prints} logs={len(logs)}")
check("日志行与最后一行一致", logs[-1] == task.last_line)
check("黑盒里每拍只有 1 个 Telemetry 快照（没有堆积）",
      isinstance(bb.get(P_TELEMETRY)[TELEMETRY_KEY], Telemetry))

section("③a ★ 任务拍频 5× 于周期 → 打印仍按 2 Hz（每 5 拍 1 条，强判据）")
logs2: list[str] = []
task2, bridge2, bb2, bus2, clock2, logs2 = build(logs=logs2)
logs2.clear()
for _ in range(50):                              # 50 拍 × 0.1 s = 5 s
    clock2.advance(0.1)
    task2.tick()
check("50 拍只打印 10 条（5 s @2 Hz）", len(logs2) == 10, f"got {len(logs2)} 条")
check("打印条数远小于拍数（50 → 10，节流比 5:1）", len(logs2) * 5 == task2.ticks,
      f"logs={len(logs2)} ticks={task2.ticks}")
check("落盘快照也是 10 次（不是每拍写）", task2.prints == 10,
      f"prints={task2.prints}")
task2.teardown()

section("③b 自定义 telemetry.period_s=1.0 → 1 Hz")
logs3: list[str] = []
task3, bridge3, bb3, bus3, clock3, logs3 = build({"telemetry": {"period_s": 1.0}},
                                                 logs=logs3)
logs3.clear()
for _ in range(10):
    clock3.advance(0.5)
    task3.tick()
check("10 拍 / 5 s @1 Hz → 5 条", len(logs3) == 5, f"got {len(logs3)} 条")
task3.teardown()

# --------------------------------------------------------------------------- #
section("④ ★ 只碰 P_TELEMETRY：不写任何其它键，也不覆盖别人的子键")
task, bridge, bb, bus, clock, sub = build()
for _ in range(20):
    clock.advance(0.5)
    task.tick()
describe = bb.describe()
check("黑盒里出现的键集合 == 测试预置的那些（遥测没新增键）",
      set(describe) == {P_POSE, P_MISSION_PHASE, P_ROUTE, P_ROUTE_INDEX, P_ROUTE_S,
                        P_STOP_POINT, P_TARGET, P_AIM, P_SAFETY, P_DRONE,
                        P_FIX_EST, P_TELEMETRY},
      f"got {sorted(describe)}")
check("P_TELEMETRY 的写入者 == TelemetryTask",
      bb.entry(P_TELEMETRY).source == "TelemetryTask",
      f"got {bb.entry(P_TELEMETRY).source}")
check("别人的子键 tracker_cmd 原样保留",
      bb.get(P_TELEMETRY)["tracker_cmd"] == {"speed": 0.15, "steer_deg": -3.5},
      f"got {bb.get(P_TELEMETRY).get('tracker_cmd')}")
check("别人的子键 mission_state 原样保留",
      bb.get(P_TELEMETRY)["mission_state"] == {"phase": "DRIVE"},
      f"got {bb.get(P_TELEMETRY).get('mission_state')}")
check("位姿等其它键的写入者仍是 test（没被遥测改过）",
      all(bb.entry(k).source == "test" for k in
          (P_POSE, P_MISSION_PHASE, P_SAFETY, P_AIM, P_DRONE, P_TARGET, P_FIX_EST)),
      f"got {[(k, bb.entry(k).source) for k in (P_POSE, P_SAFETY, P_AIM)]}")
check("没有调用 bridge 的任何写方法（遥测不做决策）",
      bridge.commands == [] and bridge.brakes == 0 and bridge.fire_gate == []
      and bridge.drone_out == [] and bridge.scan_requests == [],
      f"cmds={len(bridge.commands)} brakes={bridge.brakes} scan={bridge.scan_requests}")

# --------------------------------------------------------------------------- #
section("⑤ 兜底：汇总失败不杀线程、teardown 可重入")
logs4: list[str] = []
task, bridge, bb, bus, clock, logs4 = build(logs=logs4)

class _Boom:
    writes = 0
    def get(self, key, default=None):
        if key == P_POSE:
            raise RuntimeError("模拟黑盒读炸")
        return default

    def set(self, *a, **k):
        raise RuntimeError("不该走到这里")

task.bb = _Boom()
logs4.clear()
clock.advance(0.5)
task.tick()                                     # ★ 不抛异常
check("汇总失败被兜住（记一条日志）", len(logs4) == 1 and "汇总失败" in logs4[0],
      f"got {logs4}")
check("失败后 latest 仍为 None（不假装有数据）", task.latest is None)
task.teardown()
check("teardown 不抛异常（且能打印最后一行占位）",
      any("最后一行" in x for x in logs4), f"got {logs4}")

finish()
