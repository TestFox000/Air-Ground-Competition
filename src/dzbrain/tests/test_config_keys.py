"""配置键治理守卫（2026-10-31 · lead）。

**为什么需要**：`cfg.get("字面量键")` 若是拼写错、或引用了已被删除的配置段，
`Config.get` 会静默返回 default/None —— 现场改配置不生效、联锁判据整条跳过。
本项目已实锤三例（`aim.max_shots`、`mission.shot_timeout_s`、
`path_executor.shoot_distance_*`），都是"不报错但行为不对"。

**判据**（真源 = SCHEMA ∪ 现役 robot.yaml 的扁平键集）：
  1. 代码里每个字面量键都必须在真源里；对不上 = 红（附最接近的已登记键提示）。
  2. 地图原始 dict（`map_cfg` / `cfg_map` / `map_cfg_*`）不是 robot 配置，不计。
  3. 有意读未登记键（feature flag，缺键走显式默认、零行为变化）登记在 `OPTIONAL`；
     暂时不能改的**真 bug** 登记在 `DEBT`。两者都必须带到期日，过期即红。
  4. 清单条目若已无对应代码点 = 红（防清单腐烂，修完必须删条目）。

Run: cd dzrobot_v2 && python -B -X utf8 src/dzbrain/tests/test_config_keys.py
"""
import ast
import datetime
import difflib
import pathlib
import sys

WS = pathlib.Path(__file__).resolve().parents[3]
PKG = WS / "src" / "dzbrain" / "dzbrain"
sys.path.insert(0, str(WS / "src" / "dzbrain"))

from dzbrain.core.config import SCHEMA, Config  # noqa: E402

TODAY = datetime.date.today()
EXPIRE = "2026-11-07"          # 一个收口周：要么登记进 SCHEMA/robot.yaml，要么修代码

#: 地图 dict 的接收者前缀（不是 robot 配置，`get("map")` 是合法段名）
NON_CONFIG_OWNERS = ("map_cfg", "cfg_map")

#: 有意读**未登记键**：缺键时走代码里显式写死的默认值，行为已定义。
#: 每条必须有 why + card + expires；收口动作 = 键登记进 SCHEMA/robot.yaml（或删掉读取）。
OPTIONAL = {
    "step.gate_policy": {"why": "StepLoop gate 分级开关；缺省空串=严格档",
                         "card": "S3-STEP-GATE-20261031", "expires": EXPIRE},
    "planner.corner_cut": {"why": "切角开关；缺省 False = 发车路线逐 token 不变",
                           "card": "planner 卡", "expires": EXPIRE},
    "drone.coverage_enabled": {"why": "覆盖语义开关；drone.py:626 注释明示本卡写域不含 config.py",
                               "card": "G5-2/3", "expires": EXPIRE},
    "drone.field_verify_policy": {"why": "场地核对策略；缺省 record",
                                  "card": "G5-3", "expires": EXPIRE},
    "drone.corridor_min_cells": {"why": "S1-CORRIDOR 只读观测参数；缺省常量",
                                 "card": "S1-CORRIDOR", "expires": EXPIRE},
    "drone.start_gate_ts": {"why": "S1-CORRIDOR 开赛 gate 时间戳；缺省 0",
                            "card": "S1-CORRIDOR", "expires": EXPIRE},
    "localization.reanchor_step_max_m": {"why": "F5 分拍重锚单拍步长上限；缺省 0.12",
                                         "card": "F5-REANCHOR-STEP-20261025", "expires": EXPIRE},
    "localization.reanchor_done_m": {"why": "F5 分拍重锚收敛阈值；缺省 0.02",
                                     "card": "F5-REANCHOR-STEP-20261025", "expires": EXPIRE},
    "mission.stop_deadline_s": {"why": "到站硬死线的可选覆盖键；mission.py:1570 注释明示未写入 robot.yaml",
                                "card": "MISSION-GATE-20261029", "expires": EXPIRE},
    "safety.brake_fail_alarm_after": {"why": "连续 N 拍零速发不出去即报 alarm；缺省 3",
                                      "card": "安全卡", "expires": EXPIRE},
}

#: **真 bug**：键名/配置段与真源对不上，现场改配置不会生效。到期必须修（冻结区需先登记 in_flight）。
#: 2026-10-31 `CONFIG-DEBT-20261031` 已清空：2 处拼写错（aim.max_shots / mission.shot_timeout_s）
#: + 2 组删段残留（path_executor.shoot_* / mission_blocks）全部收口；此后**任何**对不上真源的
#: 字面量键都直接判红，不再有"已知债"豁免。
DEBT: dict[str, dict] = {}


def _expr_name(node: ast.AST) -> str:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        base = _expr_name(node.value)
        return f"{base}.{node.attr}" if base else node.attr
    return ""


def _scan() -> list[tuple[str, int, str, str]]:
    hits: list[tuple[str, int, str, str]] = []
    for path in sorted(PKG.rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            if not isinstance(func, ast.Attribute) or func.attr not in ("get", "override"):
                continue
            if not node.args or not isinstance(node.args[0], ast.Constant):
                continue
            key = node.args[0].value
            if not isinstance(key, str):
                continue
            owner = _expr_name(func.value)
            low = owner.lower()
            if "cfg" not in low and "config" not in low:
                continue
            if any(low.startswith(x) or low.endswith(x) for x in NON_CONFIG_OWNERS):
                continue
            try:
                rel = path.relative_to(WS).as_posix()
            except ValueError:                      # 自测时 PKG 可被指到工作区外的临时目录
                rel = str(path)
            hits.append((rel, node.lineno, key, owner))
    return hits


def main() -> int:
    print("=" * 72)
    print("配置键治理守卫：代码字面量键 ⊆ SCHEMA ∪ robot.yaml")
    print("=" * 72)
    cfg = Config.load(str(WS / "config" / "robot.yaml"))
    known = set(cfg.keys()) | set(SCHEMA)
    hits = _scan()
    misses = [h for h in hits if h[2] not in known]
    seen = {h[2] for h in misses}
    failures: list[str] = []

    print(f"扫描 {len(hits)} 处字面量 cfg 键；真源 {len(known)} 键（SCHEMA {len(SCHEMA)}）")
    print(f"对不上真源: {len(misses)} 处 / {len(seen)} 个键")

    for rel, line, key, owner in misses:
        entry = OPTIONAL.get(key) or DEBT.get(key)
        kind = "OPTIONAL" if key in OPTIONAL else ("DEBT" if key in DEBT else "")
        if not entry:
            near = difflib.get_close_matches(key, sorted(known), n=1, cutoff=0.72)
            hint = f"；最接近的已登记键: {near[0]}" if near else ""
            failures.append(f"未登记键 {key}  [{rel}:{line} {owner}.get]{hint}")
            continue
        exp = datetime.date.fromisoformat(entry["expires"])
        if exp < TODAY:
            failures.append(f"{kind} 已过期（{entry['expires']}）: {key} [{rel}:{line}] why={entry['why']}")
        else:
            print(f"  [{kind:8}] {key}  [{rel}:{line}]  {entry['why']}  (到期 {entry['expires']})")

    # 清单腐烂：条目在、代码点没了 ⇒ 修完没删条目（同样按红处理，防清单变成垃圾场）
    for key in list(OPTIONAL) + list(DEBT):
        if key not in seen and not any(h[2] == key for h in hits):
            failures.append(f"清单条目已无对应代码点: {key}（收口后请删除条目）")
        elif key not in seen:
            failures.append(f"清单条目已在真源中: {key}（登记进 SCHEMA/robot.yaml 后请删除条目）")

    print("-" * 72)
    if failures:
        print(f"FAIL {len(failures)} 项：")
        for item in failures:
            print(f"  ✗ {item}")
        print("结果：红（配置键漂移，现场改配置可能静默不生效）")
        return 1
    print(f"结果：绿（{len(seen)} 个未登记键全部有 OPTIONAL/DEBT 登记且在有效期内）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
