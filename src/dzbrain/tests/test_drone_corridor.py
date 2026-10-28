# -*- coding: utf-8 -*-
r"""起步走廊确认 / 布局等待门（S1-CORRIDOR / 树节点 R-17）—— 纯脚本，无 pytest。

运行：
    cd dzrobot_v2 && python src/dzbrain/tests/test_drone_corridor.py
    cd dzrobot_v2 && python src/dzbrain/tests/test_drone_corridor.py --require-production-60

覆盖卡面 V1~V3（V4 是机端取证，见 _tmp_session/S1-CORRIDOR/REPORT.md）：
  V1  corridor_confirm_s 出数（判据 ≤35 s）：正例 N10 左 5 格、分帧补全、v1 整图、
      负例永不覆盖（None，不填 0）、边界贴 A 列截断。
  V2  等待门：wait_s=60 且 40 s 到布局 ⇒ 继续（不 exit 3）；0 条布局只在 60.0 s 退出；
      对照 wait_s=30 ⇒ 30.0 s 退出（证明 30→60 的必要性）。
  V3  布局非法 ⇒ 拒收 +1 且不崩（缺键 / 越界格 / 同版异内容三个负例）；
      先收好布局再收坏布局 ⇒ 保留上一份、不清空。
  另：开赛 gate 观测（预 gate 帧计数、照收不拒）、观测开关的 A/B 逐位一致。

★ 本文件**不依赖** config/map.yaml（该文件在工作树里可能不存在 —— 车端"不留地图文件"
  是设计，测试跟着走，不许拿它当夹具）。布局全部用注入帧构造，时间全部是假钟快进，
  不是无人机实测数据；实测口径见 REPLAY 脚本与卡面"注入"标注。
"""

from __future__ import annotations

import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:                                        # 车端/Windows 控制台默认不是 UTF-8
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

from dzbrain.core import geometry as geo                                        # noqa: E402
from dzbrain.core.blackboard import P_FIELD_LAYOUT, Blackboard                  # noqa: E402
from dzbrain.core.config import Config                                           # noqa: E402
from dzbrain.core.events import EventBus                                         # noqa: E402
from dzbrain.tasks.drone import (CORRIDOR_MAX_CELLS, CORRIDOR_MIN_CELLS,         # noqa: E402
                                 DroneTask, corridor_cells_of,
                                 normalize_field_layout)
from fakes import FakeBridge, FakeClock                                          # noqa: E402
from harness import check, check_raises, finish, section                         # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]              # <ws> = dzrobot_v2/
CONFIG_PATH = WS / "config" / "robot.yaml"
REQUIRE_PROD_60 = "--require-production-60" in sys.argv

#: 起点格与"左手侧 5 格"（N10 朝 M10 ⇒ 列字母递减）—— 与 card V1 判据同口径
START = "N10"
CORRIDOR = ["M10", "L10", "K10", "J10", "I10"]


# --------------------------------------------------------------------------- #
#  脚手架（不碰 ROS、不碰 config/map.yaml）
# --------------------------------------------------------------------------- #
def build(*, corridor_min_cells: int | None = None, gate_ts: float | None = None,
          log: list | None = None):
    """造 (task, bridge, bb, bus, clock)。cfg 只用内存 dict，不读 yaml（map.yaml 可能不存在）。"""
    clock = FakeClock()
    bridge = FakeBridge(clock)
    bb = Blackboard()
    bus = EventBus()
    drone_cfg: dict = {"enabled": True, "heartbeat_timeout_s": 3.0, "info_stale_s": 30.0}
    if corridor_min_cells is not None:
        drone_cfg["corridor_min_cells"] = corridor_min_cells
    if gate_ts is not None:
        drone_cfg["start_gate_ts"] = gate_ts
    cfg = Config.from_dict({"drone": drone_cfg, "logging": {"rate_limit_s": 0.0}})
    logs: list[str] = log if log is not None else []
    task = DroneTask(bridge, bb, bus, cfg, log=logs.append, clock=clock.now)
    task.setup()
    return task, bridge, bb, bus, clock, logs


def layout(*, version: int = 2, known: list | None = None, start: str = START,
           mines: list | None = None, obstacles: list | None = None,
           targets: list | None = None, bridges: list | None = None,
           ts: float = 1_700_000_000.0, target_cell: str = "", segment_id: int = 1,
           drop: tuple = ()) -> dict:
    """造一份**协议合法**的 field_layout v2 报文（缺省带 KNOWN 背包）；drop=故意删键做负例。"""
    payload = {
        "type": "field_layout", "version": version, "start": start, "end": "N5",
        "known": list(known if known is not None else [START] + CORRIDOR),
        "mines": list(mines or []), "obstacles": list(obstacles or []),
        "targets": list(targets or []), "bridges": list(bridges or ["K9"]),
        "ts": ts, "target_cell": target_cell, "segment_id": segment_id,
    }
    for key in drop:
        payload.pop(key, None)
    return payload


def feed(task, clock, n: int = 1, dt: float = 0.2) -> None:
    for _ in range(n):
        clock.advance(dt)
        task.tick()


def push(bridge, payload) -> None:
    bridge.push_drone(payload if isinstance(payload, str)
                      else json.dumps(payload, ensure_ascii=False))


# --------------------------------------------------------------------------- #
section("① corridor_cells_of：起点左手侧 5 格、不计起点、越 A 列截断")
check("N10 + 5 格 ⇒ M10/L10/K10/J10/I10",
      corridor_cells_of("N10", 5) == CORRIDOR, f"{corridor_cells_of('N10', 5)}")
check("N10 + 8 格（用户上限）⇒ 长度 8 且含 F10",
      len(corridor_cells_of("N10", 8)) == CORRIDOR_MAX_CELLS
      and corridor_cells_of("N10", 8)[-1] == "F10", f"{corridor_cells_of('N10', 8)}")
check("★ 不计起点格本身（N10 不在走廊清单里）",
      "N10" not in corridor_cells_of("N10", 8))
check("B5 + 5 格 ⇒ 只有 A5（贴边截断，不凑数）",
      corridor_cells_of("B5", 5) == ["A5"], f"{corridor_cells_of('B5', 5)}")
check("A5 + 5 格 ⇒ 空（左手侧无格，车道天生没有走廊）",
      corridor_cells_of("A5", 5) == [], f"{corridor_cells_of('A5', 5)}")
check("小写输入归一化（n10）",
      corridor_cells_of("n10", 5) == CORRIDOR)
check_raises("非法格名抛 ValueError", ValueError, corridor_cells_of, "Z9", 5)

# --------------------------------------------------------------------------- #
section("② V1 正例：走廊一次到位 ⇒ corridor_confirm_s 出数且 ≤35 s（注入帧）")
task, bridge, bb, bus, clock, logs = build()
push(bridge, {"type": "heartbeat", "seq": 1})
feed(task, clock)                                      # t += 0.2
feed(task, clock, n=58, dt=0.2)                        # 累计 ≈11.8 s
push(bridge, layout())
feed(task, clock)                                      # 12.0 s 收到布局
st = task.corridor_stats()
check("first_layout_s 出数 ≈12.0 s", st["first_layout_s"] is not None
      and abs(st["first_layout_s"] - 12.0) < 0.25, f"{st['first_layout_s']}")
check("★ V1 corridor_confirm_s 出数 ≈12.0 s", st["corridor_confirm_s"] is not None
      and abs(st["corridor_confirm_s"] - 12.0) < 0.25, f"{st['corridor_confirm_s']}")
check("★ V1 判据 ≤35 s 成立", st["corridor_confirm_s"] is not None
      and st["corridor_confirm_s"] <= 35.0, f"{st['corridor_confirm_s']}")
check("走廊口径 = N10 左 5 格", st["cells"] == CORRIDOR, f"{st['cells']}")
check("日志里有机器可判的 corridor_confirm_s=12.00",
      any("corridor_confirm_s=12.0" in m for m in logs),
      str([m for m in logs if "corridor_confirm_s" in m][:1]))
check("布局照常落黑盒（观测不改变接收）",
      isinstance(bb.get(P_FIELD_LAYOUT), dict)
      and bb.get(P_FIELD_LAYOUT)["version"] == 2 and task.layout_rejects == 0)

# ---- 已确认后不再变（后续帧不重算） ----
t_confirm = st["corridor_confirm_s"]
feed(task, clock, n=30, dt=0.2)                        # +6 s
push(bridge, layout(version=3, known=[START] + CORRIDOR + ["H10"]))
feed(task, clock)
check("★ 已确认后 corridor_confirm_s 不被后续帧改写",
      task.corridor_stats()["corridor_confirm_s"] == t_confirm,
      f"{task.corridor_stats()['corridor_confirm_s']} vs {t_confirm}")

# --------------------------------------------------------------------------- #
section("③ V1 正例：分两帧到达（先窄后补）⇒ 确认时刻取第二帧，不回填")
task, bridge, bb, bus, clock, logs = build()
push(bridge, layout(version=2, known=[START, "M10", "L10"]))
feed(task, clock, n=25, dt=0.2)                        # 5.0 s：只有 2/5 格
st = task.corridor_stats()
check("只覆盖 2/5 格 ⇒ corridor_confirm_s 仍为 None", st["corridor_confirm_s"] is None,
      f"{st['corridor_confirm_s']}")
check("日志写明未确认与缺哪几格",
      any("起步走廊未确认" in m and "K10" in m for m in logs),
      str([m for m in logs if "未确认" in m][:1]))
push(bridge, layout(version=3, known=[START] + CORRIDOR))
push(bridge, layout(version=3, known=[START] + CORRIDOR))
feed(task, clock)                                      # 下一拍即消费该帧
st2 = task.corridor_stats()
check("★ 补齐后确认时刻 ≈5.2 s（按第二帧到达时刻，不回填首帧 0.2 s）",
      st2["corridor_confirm_s"] is not None and abs(st2["corridor_confirm_s"] - 5.2) < 0.3,
      f"{st2['corridor_confirm_s']}")
check("确认时刻 > first_layout_s（两帧分离，证明按帧到达计）",
      st2["first_layout_s"] is not None
      and st2["corridor_confirm_s"] > st2["first_layout_s"] + 4.0,
      f"{st2['first_layout_s']} -> {st2['corridor_confirm_s']}")

# --------------------------------------------------------------------------- #
section("④ V1 负例：走廊永不覆盖 ⇒ corridor_confirm_s = None（不填 0）")
task, bridge, bb, bus, clock, logs = build()
push(bridge, layout(known=[START, "M10"]))
feed(task, clock, n=200, dt=0.2)                       # 40 s 一直只有 1/5
check("★ 40 s 仍 None", task.corridor_stats()["corridor_confirm_s"] is None)
task.teardown()
check("teardown 明确打 NONE（不把没量到写成 0）",
      any("确认 NONE" in m for m in logs), str([m for m in logs if "起步走廊" in m][-1:]))

# --------------------------------------------------------------------------- #
section("⑤ V1 v1 语义：无 known 键 = 整图已知 ⇒ 首布局即确认（与旧行为一致）")
task, bridge, bb, bus, clock, logs = build()
push(bridge, layout(version=1, drop=("known",)))
feed(task, clock, n=10, dt=0.2)                        # 2.0 s
st = task.corridor_stats()
check("v1 布局首帧即确认 corridor_confirm_s == first_layout_s",
      st["corridor_confirm_s"] is not None
      and st["corridor_confirm_s"] == st["first_layout_s"], f"{st}")
check("v1 布局仍是旧行为（无 known 键不改写）",
      "known" not in (bb.get(P_FIELD_LAYOUT) or {}))

# --------------------------------------------------------------------------- #
section("⑥ V1 边界：起点贴 A 列 ⇒ 走廊天生为空，不死等")
task, bridge, bb, bus, clock, logs = build()
push(bridge, layout(start="A5", known=["A5", "B5"], version=2))
feed(task, clock, n=10, dt=0.2)
st = task.corridor_stats()
check("A5 走廊为空 ⇒ 不出数（None，且不报未确认 spam）",
      st["corridor_confirm_s"] is None and st["cells"] == [], f"{st}")

# --------------------------------------------------------------------------- #
#  V2：等待门（真的调 main._wait_for_field_layout，用假钟快进，不 sleep 60 s）
# --------------------------------------------------------------------------- #
section("⑦ V2 正例：wait_s=60 且 40 s 到布局 ⇒ 继续（不 exit 3）")
from dzbrain import main as M                                                  # noqa: E402


class _Injector:
    """假钟 + 定点注入：sleep() 推进虚拟时钟，到点把布局写进黑盒（**注入，非实测**）。"""

    def __init__(self, bb, at: float, payload: dict | None = None):
        self.clock = FakeClock()
        self.bb = bb
        self.t0 = self.clock.now()
        self.at = float(at)
        self.payload = payload
        self.fired = False

    def now(self) -> float:
        return self.clock.now()

    def sleep(self, dt: float) -> None:
        self.clock.advance(dt)
        if self.payload is not None and not self.fired and self.clock.now() - self.t0 >= self.at:
            self.bb.set(P_FIELD_LAYOUT, self.payload, source="injected")
            self.fired = True


def wait_with(wait_s: float, inject_at: float | None, payload: dict | None = None):
    """真函数 + 假钟：返 (结果, 虚拟耗时)。inject_at=None ⇒ 一帧都不注入（负例）。"""
    bb = Blackboard()
    inj = _Injector(bb, inject_at if inject_at is not None else float("inf"), payload)
    cfg = Config.from_dict({"drone": {"field_layout_wait_s": wait_s}})
    out = M._wait_for_field_layout(bb, cfg, stop_flag={"stop": False}, alive=lambda: True,
                                  log=lambda m: None, sleep=inj.sleep, now=inj.now)
    return out, inj.clock.now() - inj.t0


res40, took40 = wait_with(60.0, 40.0, layout())
check("★ 40 s 到布局 ⇒ 返回布局（不 exit 3）", res40 is not None and res40.get("start") == START,
      f"{None if res40 is None else res40.get('start')}")
check("★ 返回耗时 ≈40.0 s（>35 s 且 <60 s）", 39.9 <= took40 <= 40.1, f"{took40:.2f}s")

section("⑧ V2 负例：0 条布局 ⇒ 只在 60.0 s 退出；对照 30 s 配置在 30.0 s 退出")
res_none60, took60 = wait_with(60.0, None)
check("★ 60 s 配置 + 0 条布局 ⇒ None（不规划、不出发）", res_none60 is None)
check("★ 退出时刻 ≈60.0 s（等满 60 s，不是 30 s）", 59.9 <= took60 <= 60.1, f"{took60:.2f}s")
res_none30, took30 = wait_with(30.0, None)
check("对照：30 s 配置 ⇒ 30.0 s 就退出（旧值会在 35 s 走廊确认前判死）",
      res_none30 is None and 29.9 <= took30 <= 30.1, f"{took30:.2f}s")
check("⇒ 30 s 与 60 s 的差别就是 18:50 那次实跑死因的量级（30.0 < 35）",
      took30 < 35.0 <= took60)

section("⑨ V2 正例2：60 s 配置下 55 s 迟到布局仍继续（边界内最后一秒）")
res55, took55 = wait_with(60.0, 55.0, layout())
check("55 s 到布局 ⇒ 继续", res55 is not None and 54.9 <= took55 <= 55.1, f"{took55:.2f}s")

section("⑩ V2 反例：60 s 后才到 ⇒ 已经 None（不回补）")
res_late, took_late = wait_with(60.0, 61.0, layout())
check("61 s 才注入 ⇒ 60.0 s 已判 None（迟到布局不回补）",
      res_late is None and 59.9 <= took_late <= 60.1, f"{took_late:.2f}s rc={res_late}")

# --------------------------------------------------------------------------- #
section("⑪ V3 负例①：缺必填键（mines）⇒ 拒收 +1 且不崩")
task, bridge, bb, bus, clock, logs = build()
push(bridge, layout(drop=("mines",)))
feed(task, clock, n=3)
check("拒收计数 layout_rejects == 1", task.layout_rejects == 1, f"{task.layout_rejects}")
check("未落任何布局（不是「按空表猜」）", bb.get(P_FIELD_LAYOUT) is None)
check("记了日志（缺 mines）", any("mines" in m for m in logs),
      str([m for m in logs if "非法" in m][:1]))

section("⑫ V3 负例②：越界格名（known 含 Z9）⇒ 拒收 +1，任务继续跑")
push(bridge, layout(known=[START, "Z9"]))
feed(task, clock, n=3)
check("拒收累计 2（不崩、继续计数）", task.layout_rejects == 2, f"{task.layout_rejects}")
push(bridge, {"type": "heartbeat", "seq": 9})            # 坏报文后仍能正常收
feed(task, clock)
check("坏布局后心跳照常（任务没被一条坏报文打死）", task.online is True)

section("⑬ V3 负例③：好布局之后再收坏布局 ⇒ 保留上一份、不清空")
push(bridge, layout(version=4))
feed(task, clock)
good = bb.get(P_FIELD_LAYOUT)
check("好布局已落盘（v4）", isinstance(good, dict) and good["version"] == 4)
push(bridge, layout(version=5, drop=("bridges",)))
feed(task, clock)
check("★ 坏布局不清空上一份（仍是 v4 同一对象）",
      bb.get(P_FIELD_LAYOUT) is good, f"{bb.get(P_FIELD_LAYOUT)}")
check("拒收累计 3", task.layout_rejects == 3, f"{task.layout_rejects}")

section("⑭ V3 负例④：同版异内容 ⇒ 拒收 +1（同版必须同内容）")
push(bridge, layout(version=5, known=[START] + CORRIDOR))
feed(task, clock)
push(bridge, layout(version=5, known=[START, "M10"]))     # 同 v5 但内容不同
feed(task, clock)
check("同版异内容被拒（rejects 再 +1）", task.layout_rejects == 4, f"{task.layout_rejects}")
check("当前布局仍是 v5 的第一份（未被后到的同版覆盖）",
      bb.get(P_FIELD_LAYOUT)["known"] == [START] + CORRIDOR, f"{bb.get(P_FIELD_LAYOUT)['known']}")

# --------------------------------------------------------------------------- #
section("⑮ 开赛 gate 观测：预 gate 布局照收不拒 + 只计数")
task, bridge, bb, bus, clock, logs = build(gate_ts=1_700_000_100.0)
push(bridge, layout(version=2, ts=1_700_000_000.0))       # 比 gate 早 100 s
feed(task, clock, n=3)
st = task.corridor_stats()
check("★ 预 gate 帧计数 == 1", st["layouts_pre_gate"] == 1, f"{st['layouts_pre_gate']}")
check("★ 照收不拒（布局已落盘、rejects 仍 0）",
      isinstance(bb.get(P_FIELD_LAYOUT), dict) and task.layout_rejects == 0)
check("日志点名「早于开赛 gate」", any("早于开赛 gate" in m for m in logs),
      str([m for m in logs if "gate" in m][:1]))
check("corridor_since_gate_s == -100（赛前 100 s 就采集了）",
      st["corridor_since_gate_s"] == -100.0, f"{st['corridor_since_gate_s']}")

section("⑯ 开赛 gate 对照：未配 gate（0）⇒ 不发生预 gate 判定")
task, bridge, bb, bus, clock, logs = build()
push(bridge, layout(version=2, ts=1.0))
feed(task, clock, n=3)
check("gate=0 ⇒ layouts_pre_gate == 0 且 corridor_since_gate_s is None",
      task.corridor_stats()["layouts_pre_gate"] == 0
      and task.corridor_stats()["corridor_since_gate_s"] is None,
      f"{task.corridor_stats()}")

# --------------------------------------------------------------------------- #
section("⑰ A/B：关掉走廊观测（corridor_min_cells=0）⇒ 黑盒与计数器逐位一致")
def _state_after(extra: dict):
    task, bridge, bb, bus, clock, logs = build(**extra)
    push(bridge, layout(version=2, known=[START] + CORRIDOR, ts=1_700_000_000.0))
    feed(task, clock, n=3)
    return (json.dumps(bb.get(P_FIELD_LAYOUT), sort_keys=True, ensure_ascii=False),
            task.layout_arrivals, task.layout_rejects, task.layout_updates)


s_on = _state_after({})
s_off = _state_after({"corridor_min_cells": 0})
check("★ 开关只影响观测：P_FIELD_LAYOUT / 帧计数 / 拒收 / 版本数 全部相同",
      s_on == s_off, f"on={s_on[1:]} off={s_off[1:]}")

# --------------------------------------------------------------------------- #
section("⑱ 生产链路的真实等待门（实测 main 到底等多少秒；--require-production-60 转断言）")
#   ★ 这一节是本卡最刺眼的发现：`Config.get(key, 字面量)` 在 key **不在 yaml** 时返回
#     调用方字面量，**丢掉 SCHEMA 默认值**（core/config.py:774-786）。而
#     main.py:1659 正是 `cfg.get("drone.field_layout_wait_s", 30.0) or 30.0`。
#     ⇒ 只把 SCHEMA 默认 30→60 **不会**改变生产等待时间；必须①yaml 显式写该键，
#     或②main.py 去掉字面量。两条都不在本卡写域 ⇒ 本卡只量化，不改。
prod_wait_cfg = None
prod_wait_measured = None
try:
    prod_cfg = Config.load(str(CONFIG_PATH))
    from dzbrain.core.config import SCHEMA as _SCHEMA
    spec_wait = _SCHEMA.get("drone.field_layout_wait_s")
    in_yaml = "drone.field_layout_wait_s" in set(prod_cfg.keys())
    prod_wait_cfg = prod_cfg.get("drone.field_layout_wait_s")
    print(f"  [观察] SCHEMA 默认 = {getattr(spec_wait, 'default', None)} s"
          f" ｜ yaml 里显式写了吗 = {in_yaml}"
          f" ｜ cfg.get(key) = {prod_wait_cfg}")
    check("机制探针：cfg.get(key, 字面量) 返回字面量（= SCHEMA 默认值被绕过）",
          prod_cfg.get("drone.field_layout_wait_s", 99.0) == 99.0
          and prod_cfg.get("drone.field_layout_wait_s", 30.0) == 30.0,
          f"get(key,99)={prod_cfg.get('drone.field_layout_wait_s', 99.0)}")
    # 真的让 main 等一次（假钟，零墙钟）：0 条布局 ⇒ 量到生产链路的真实等待秒数
    prod_bb = Blackboard()
    prod_inj = _Injector(prod_bb, float("inf"), None)
    _real = M._wait_for_field_layout(prod_bb, prod_cfg, stop_flag={"stop": False},
                                     alive=lambda: True, log=lambda m: None,
                                     sleep=prod_inj.sleep, now=prod_inj.now)
    prod_wait_measured = prod_inj.clock.now() - prod_inj.t0
    print(f"  [观察] 生产链路实测等待 = {prod_wait_measured:.1f} s"
          f" ｜ 用户 35 s 走廊节奏需要 ≥ 35 s")
    verdict = "PASS" if prod_wait_measured >= 35.0 else "FAIL"
    print(f"  [观察] 生产等待门能否容纳 35 s 走廊确认：{verdict}"
          + ("" if verdict == "PASS" else
             f"  ← 实测 {prod_wait_measured:.1f}s < 35s：按用户节奏仍会在走廊确认前 exit 3"))
except Exception as exc:                                 # noqa: BLE001
    print(f"  [观察] 生产配置探测失败（不影响本测试判定）：{type(exc).__name__}: {exc}")
if REQUIRE_PROD_60:
    check("--require-production-60：生产链路实测等待 ≥ 35 s", prod_wait_measured is not None
          and prod_wait_measured >= 35.0, f"{prod_wait_measured}")
else:
    print("  [提示] 加 --require-production-60 可把上面这条观察变成退出码判据")

# --------------------------------------------------------------------------- #
print("\n" + "=" * 70)
print("摘要（机器可读）：" + json.dumps({
    "corridor_cells": CORRIDOR,
    "first_layout_s_demo": 12.0,
    "corridor_confirm_s_demo": 12.0,
    "judge_corridor_le_35s": True,
    "wait60_layout_at_40s_took_s": round(took40, 2),
    "wait60_no_layout_took_s": round(took60, 2),
    "wait30_no_layout_took_s": round(took30, 2),
    "production_schema_default_s": getattr(_SCHEMA.get("drone.field_layout_wait_s"), "default", None)
    if "prod_cfg" in dir() else None,
    "production_measured_wait_s": (round(prod_wait_measured, 2)
                                   if prod_wait_measured is not None else None),
    "production_wait_ge_35s": (prod_wait_measured is not None and prod_wait_measured >= 35.0),
    "injected": True,
}, ensure_ascii=False))
finish()
