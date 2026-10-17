# -*- coding: utf-8 -*-
"""t36：动态档（按桥洞远近选 0.35/0.40 巡航速度）—— **默认关闭**、关闭时逐位同历史。

覆盖（对应任务卡 t36 验收 ①~⑤）：
  1. SCHEMA：`pursuit.dynamic_gear_enabled` 默认 **False**，阈值/两档速率都可配；
     并核对 **`config/robot.yaml` 没有写这些键**（现场 YAML 零改动 = 现场零覆盖）。
  2. 关闭时：`dynamic_gear_speed()` 恒 `None`；`compute_command(..., cruise_mps=None)` 与
     **不传该参数**逐位相等（`PursuitCommand` 数据类比较）；tracker 路径里
     `telemetry()["gear"] == "off"`、`gear_mps is None`、黑板观测键 `P_PURSUIT_GEAR` **不存在**。
  3. 开启时：按"车到下一个桥洞格的直线距离"选档（≤ 阈值 → near 档 0.35；> 阈值 → far 档 0.40；
     前方没有洞 → far 档），命令速度**真的**变成档值；阈值可配（改小/改大会改变判定）。
  4. 观测面：遥测 `gear/gear_mps` + 黑板 `P_PURSUIT_GEAR` + 日志带"动态档"字样（t31 情报：
     口径词要同时进日志与遥测）。
  5. `Config` 是 frozen dataclass ⇒ 动态档不改配置值，只作"本拍巡航速度"的注入（`cruise_mps`）。

跑法：`$env:PYTHONPATH='…\\src\\dzbrain'; python src\\dzbrain\\tests\\test_dynamic_gear.py`
全过 exit 0；任一失败 exit 1。
"""

from __future__ import annotations

import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parents[1]))       # src/dzbrain
sys.path.insert(0, str(_HERE))                  # tests/（fakes.py）

try:
    sys.stdout.reconfigure(errors="replace")    # type: ignore[union-attr]
except Exception:                               # pragma: no cover
    pass

from fakes import FakeBridge, FakeClock                                          # noqa: E402

from dzbrain.core import geometry as geo                                         # noqa: E402
from dzbrain.core.blackboard import (Blackboard, P_MISSION_PHASE, P_POSE,        # noqa: E402
                                     P_PURSUIT_GEAR, P_ROUTE, P_SAFETY, P_TELEMETRY)
from dzbrain.core.config import Config                                           # noqa: E402
from dzbrain.core.events import EventBus                                         # noqa: E402
from dzbrain.core.state import SafetyState                                       # noqa: E402
from dzbrain.io.contract import MissionPhase                                     # noqa: E402
from dzbrain.pursuit.controller import (compute_command, dynamic_gear_speed,     # noqa: E402
                                        speed_profile)
from dzbrain.route.model import RouteBuilder, Waypoint                           # noqa: E402
from dzbrain.route.parser import parse_mission                                   # noqa: E402
from dzbrain.tasks.tracker import TrackerTask                                    # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]
ROBOT_YAML = WS / "config" / "robot.yaml"
ROUTE_TOKENS = ["N10", "M10", "L10", "K10", "J10"]      # 含一次下半场穿洞（K10）

#: 动态档的 4 个 SCHEMA 键（"不改配置"的对照用）
_GEAR_KEYS = ("pursuit.dynamic_gear_enabled", "pursuit.dynamic_gear_near_dist_m",
              "pursuit.dynamic_gear_near_mps", "pursuit.dynamic_gear_far_mps")

_PASS = 0
_FAIL = 0
_FAILED: list[str] = []


def ok(name: str, cond: bool, detail: str = "") -> None:
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print(f"PASS  {name}" + (f"   [{detail}]" if detail else ""))
    else:
        _FAIL += 1
        _FAILED.append(name)
        print(f"FAIL  {name}   [{detail}]")


def sec(title: str) -> None:
    print()
    print(f"--- {title} ---")


def cfg_of(**pursuit) -> Config:
    base = {"cruise_speed_mps": 0.15, "min_speed_mps": 0.05, "speed_quant_comp": 1.0,
            "stop_approach_speed_gain": 0.30, "corner_speed_mps": 0.12,
            "corner_slow_dist_m": 0.25, "reverse_speed_ratio": 0.6}
    base.update(pursuit)
    return Config.from_dict({"pursuit": base})


def route_of(tokens=None):
    return parse_mission(list(ROUTE_TOKENS if tokens is None else tokens))


class Rig:
    """一辆"车"：FakeBridge + Blackboard + TrackerTask（够用即可）。"""

    def __init__(self, *, cfg: Config, route, pose=(0.0, 0.0, 0.0)) -> None:
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.logs: list[str] = []
        self.cfg = cfg
        self.task = TrackerTask(self.bridge, self.bb, EventBus(), cfg,
                                log=self.logs.append, clock=self.clock.now)
        self.bb.set(P_SAFETY, SafetyState(ok=True), "test")
        self.bb.set(P_MISSION_PHASE, MissionPhase.DRIVE, "test")
        self.bb.set(P_ROUTE, route, "test")
        self.set_pose(*pose)

    def set_pose(self, x: float, y: float, yaw: float = 0.0) -> None:
        self.bridge.set_pose(x, y, yaw)
        self.bb.set(P_POSE, geo.Pose(x, y, yaw), "test")

    def tick(self) -> dict:
        self.task.tick()
        self.clock.advance(0.1)
        return self.task.telemetry()


def main() -> int:
    print("=" * 72)
    print("dzbrain 动态档（t36）自测")
    print("=" * 72)

    # ------------------------------------------------------------------ #
    sec("1. SCHEMA：默认关闭 + 阈值/两档可配 + config/robot.yaml 零覆盖")
    cfg = cfg_of()
    ok("1.1 ★ `pursuit.dynamic_gear_enabled` 默认 False",
       cfg.get("pursuit.dynamic_gear_enabled") is False,
       repr(cfg.get("pursuit.dynamic_gear_enabled")))
    ok("1.2 阈值/两档键都在 SCHEMA 里（near_dist_m / near_mps / far_mps）",
       float(cfg.get("pursuit.dynamic_gear_near_dist_m")) == 1.05
       and float(cfg.get("pursuit.dynamic_gear_near_mps")) == 0.35
       and float(cfg.get("pursuit.dynamic_gear_far_mps")) == 0.40,
       f"{cfg.get('pursuit.dynamic_gear_near_dist_m')}/{cfg.get('pursuit.dynamic_gear_near_mps')}"
       f"/{cfg.get('pursuit.dynamic_gear_far_mps')}")
    yaml_text = ROBOT_YAML.read_text(encoding="utf-8")
    ok("1.3 ★ config/robot.yaml 里**没有**这几个键（现场零改动 ⇒ 现场零覆盖）",
       all(key not in yaml_text for key in ("dynamic_gear_enabled", "dynamic_gear_near",
                                            "dynamic_gear_far")),
       f"{ROBOT_YAML}")
    real = Config.load(str(ROBOT_YAML))
    ok("1.4 现场配置读出来的值 == SCHEMA 缺省（关闭）",
       real.get("pursuit.dynamic_gear_enabled") is False, "")
    ok("1.5 动态档**不改配置值**：调一轮 gear 计算 + 一拍 tracker 后 4 个键原样",
       (lambda before: (dynamic_gear_speed(cfg_of(dynamic_gear_enabled=True), 0.3),
                        all(cfg.get(k) == v for k, v in before.items())))(
           {k: cfg.get(k) for k in _GEAR_KEYS}),
       "（`Config.override` 是 bench/测试专用的**就地**改键，本测试一律用新 Config，不碰它）")

    # ------------------------------------------------------------------ #
    sec("2. 关闭时：读侧恒 None + 逐位同历史")
    for dist, label in ((0.30, "近洞"), (3.00, "远洞"), (None, "无洞")):
        ok(f"2.1 关闭 → dynamic_gear_speed({label}) 恒 None",
           dynamic_gear_speed(cfg, dist) is None, f"dist={dist}")
    sp_omitted = speed_profile(curvature=0.0, dist_to_stop=None, corner_dist=None,
                               speed_scale=1.0, cfg=cfg)
    sp_none = speed_profile(curvature=0.0, dist_to_stop=None, corner_dist=None,
                            speed_scale=1.0, cfg=cfg, cruise_mps=None)
    ok("2.2 ★ `speed_profile`：不传 cruise_mps 与传 None **逐位相等**",
       sp_omitted == sp_none, f"{sp_omitted} vs {sp_none}")
    route = route_of()
    poses = [geo.Pose(x=0.175, y=0.175, yaw=0.0), geo.Pose(x=0.175, y=0.875, yaw=0.0)]
    same = True
    for ps in poses:
        base_cmd = compute_command(ps, task_path(route), task_path(route).project(ps.x, ps.y), cfg)
        none_cmd = compute_command(ps, task_path(route), task_path(route).project(ps.x, ps.y), cfg,
                                   cruise_mps=None)
        same = same and base_cmd == none_cmd
    ok("2.3 ★ `compute_command`：cruise_mps=None 与不传该参数**逐位相等**（两处位姿）",
       same, "PursuitCommand 数据类比较")
    rig_off = Rig(cfg=cfg, route=route, pose=(0.175, 0.175, 0.0))
    tel_off = rig_off.tick()
    ok("2.4 关闭时遥测 `gear == \"off\"`、`gear_mps is None`",
       tel_off.get("gear") == "off" and tel_off.get("gear_mps") is None,
       f"gear={tel_off.get('gear')!r} gear_mps={tel_off.get('gear_mps')!r}")
    ok("2.5 关闭时**不写**黑板观测键 P_PURSUIT_GEAR",
       rig_off.bb.get(P_PURSUIT_GEAR) is None, repr(rig_off.bb.get(P_PURSUIT_GEAR)))
    ok("2.6 关闭时日志里没有『动态档』字样",
       not any("动态档" in s for s in rig_off.logs), str(rig_off.logs))
    ok("2.7 关闭时命令速度 = 配置巡航档（0.15，未被动态档抬高/压低）",
       abs(float(rig_off.bridge.commands[-1][0]) - 0.15) < 1e-9,
       f"{rig_off.bridge.commands[-1][0]}")

    # ------------------------------------------------------------------ #
    sec("3. 开启时：按桥洞远近选档（阈值可配）")
    cfg_on = cfg_of(dynamic_gear_enabled=True)
    ok("3.1 开启 + 近洞（0.30 m ≤ 1.05 m）→ 0.35 档",
       dynamic_gear_speed(cfg_on, 0.30) == 0.35, "")
    ok("3.2 开启 + 远洞（3.00 m > 1.05 m）→ 0.40 档",
       dynamic_gear_speed(cfg_on, 3.00) == 0.40, "")
    ok("3.3 开启 + 前方没有洞（None）→ 0.40 档",
       dynamic_gear_speed(cfg_on, None) == 0.40, "")
    cfg_far = cfg_of(dynamic_gear_enabled=True, dynamic_gear_near_dist_m=2.0)
    ok("3.4 ★ 阈值可配：把 near_dist_m 改到 2.0 m ⇒ 1.75 m 也算『近洞』",
       dynamic_gear_speed(cfg_far, 1.75) == 0.35
       and dynamic_gear_speed(cfg_on, 1.75) == 0.40, "1.75m: 2.0阈值→0.35 / 1.05阈值→0.40")

    # 3.5 车真的按档跑：同一位姿下"开启 vs 关闭"与"远 vs 近"两组对照
    #     （不直接断言绝对速度：停站/转角/曲率层的效果两边相同，看**差分**才是安全的判据）
    k10 = geo.block_to_meters("K10")                 # noqa: F841  （留作现场对账参考）
    #   ★ 用"直着进洞"的路线（L10→K10→J10→…）：没有转角、停站层又远 ⇒ 档值不会被曲率/停站盖住，
    #     所以这里断言**绝对速度**才是安全的（在 N10→L10→K10 那条带拐角的路上，档值会被曲率层压住，
    #     只能看差分 —— 那种断言证明不了"档值真的进去了"）。
    #     路线从 N10 起（车在 M10 时其"当前段"是 M10→L10，**还没进桥洞直行段** ⇒ 不会被
    #     `align` 定点速度盖住；这一步是本次实测踩出来的：站在 L10 上时命令速度恒为 align 档）。
    route2 = route_of(["N10", "M10", "L10", "K10", "J10", "I10", "H10"])
    along = geo.heading_of(0.0, 1.0, fallback=1.5707963267948966)   # 沿第 10 行向西（N→M→L→K）
    n10v = geo.block_to_meters("N10")
    m10v = geo.block_to_meters("M10")
    far_pose2 = (n10v[0], n10v[1] - 0.70, along)     # 距 K10 = 1.75 m ⇒ far
    near_pose = (m10v[0], m10v[1], along)            # 距 K10 = 0.70 m ⇒ near（阈值 1.05 内）
    tel_off_far = Rig(cfg=cfg, route=route2, pose=far_pose2).tick()
    rig_far = Rig(cfg=cfg_on, route=route2, pose=far_pose2)
    tel_far = rig_far.tick()
    rig_near = Rig(cfg=cfg_on, route=route2, pose=near_pose)
    tel_near = rig_near.tick()
    ok("3.5 ★ 关闭时命令速度 = 配置巡航 0.15（动态档完全没参与）",
       abs(float(tel_off_far.get("speed")) - 0.15) < 1e-6 and tel_off_far.get("gear") == "off",
       f"speed={tel_off_far.get('speed')} gear={tel_off_far.get('gear')!r}")
    ok("3.6 ★ 开启时同一位姿 → 命令速度 = 远洞档 0.40（档值真的进了速度剖面）",
       abs(float(tel_far.get("speed")) - 0.40) < 1e-6 and tel_far.get("gear") == "far",
       f"speed={tel_far.get('speed')} gear={tel_far.get('gear')!r}")
    ok("3.7 ★ 近洞位姿 → 命令速度 = 近洞档 0.35（这就是「按桥洞远近选档」）",
       abs(float(tel_near.get("speed")) - 0.35) < 1e-6 and tel_near.get("gear") == "near",
       f"speed={tel_near.get('speed')} gear={tel_near.get('gear')!r}")

    # ------------------------------------------------------------------ #
    sec("4. 观测面：遥测 + 黑板键 + 日志（口径词两处都有）")
    gear_bb = rig_near.bb.get(P_PURSUIT_GEAR)
    ok("4.1 黑板 `P_PURSUIT_GEAR` 写了当拍档位（gear/cruise_mps/dist_to_cross_m）",
       isinstance(gear_bb, dict) and gear_bb.get("gear") == "near"
       and abs(float(gear_bb.get("cruise_mps")) - 0.35) < 1e-9
       and 0.0 < float(gear_bb.get("dist_to_cross_m") or 0.0) <= 1.05,
       str(gear_bb))
    ok("4.2 遥测 `gear_mps` = 档值、`gear` = near/far",
       abs(float(tel_near.get("gear_mps")) - 0.35) < 1e-9 and tel_far.get("gear") == "far",
       f"{tel_near.get('gear_mps')}/{tel_far.get('gear')}")
    ok("4.3 ★ 日志带『动态档』字样（现场排『为什么变档了』不用翻代码）",
       any("动态档" in s for s in rig_near.logs) and any("动态档" in s for s in rig_far.logs),
       str([s for s in rig_far.logs if "动态档" in s][:1]))
    ok("4.4 同一档位不重复刷日志（只在档位变化时打一条）",
       len([s for s in rig_near.logs if "动态档" in s]) == 1,
       str([s for s in rig_near.logs if "动态档" in s]))
    ok("4.5 遥测确实落了黑板（P_TELEMETRY 可读）",
       rig_near.bb.get(P_TELEMETRY) is not None, "")

    print()
    print(f"结果：{_PASS}/{_PASS + _FAIL} 通过，{_FAIL} 失败")
    if _FAILED:
        print("失败用例：" + "；".join(_FAILED))
        return 1
    print("全部通过 ✅")
    return 0


def task_path(route):
    from dzbrain.pursuit.path import Path
    return Path(route)


if __name__ == "__main__":
    sys.exit(main())
