---
卡号: GEOM-CLOSURE-20261028
ns: agc-1026
agent: lead
owner: lead
reviewer: -
树节点: R-21
写域: "dzrobot_v2/src/dzbrain/dzbrain/tools/sim.py + dzrobot_v2/tools/route_check.py + _tmp_session/R-21/ + 本卡"
status: in_progress
创建: 2026-10-28
---

# R-21 · P1 收尾：sim 桥洞判据 + route_check 几何真值（Lead 亲做）

## 背景（P1 交付时的两处写域外高危）
1. `sim.py` 的 `bridge_modeled` 挂在 `redundancy > 0` 上；现场把冗余置 0 后**桥洞判据整体消失**（指标 modeled=0 是「没测」），而 P1 已把墙段改成非退化真实段。
2. `tools/route_check.py` 仍用 `VEHICLE_WIDTH_M=0.135`，且 vehicle 段只读 `/dzbrain`（真源在 `/dzactuator`）⇒ 静默留旧值；`bridge_entry_walls(b, 0.0)` 不传 opening/wall，config 一改就分叉，`test_physics` §6 对拍因此红。

## 改动（只改默认值/接线，不改几何算法）
- `sim.py`：`bridge_modeled = 1 if spec.bridges else 0`（redundancy=0 也建真墙）；`body_corners`/`body_point_clearance_m`/`FieldObs.width_m`/`cfg.get("vehicle.width_m")` 四处默认 0.135→0.17；注释同步。
- `route_check.py`：`VEHICLE_WIDTH_M` 0.135→0.17（半宽 0.085）；vehicle 段改为 `/dzbrain`+`/dzactuator` 合并读取；`bridge_entry_walls` 传入 `field.bridge_opening_m/bridge_wall_m`；docstring 门槛改 0.2325 / 0.3113 / 0.095。
- 未动 `mission.py:710`：运行期读 config（=0.17），保留兜底；V1 残留见下。

## 验收（机器可判，已实测）
- `python _tmp_session/R-21/probe_bridge.py` → modeled=1；偏移 0.00/0.05/0.07 → steps=0；**0.09 → steps=21**（真墙生效）。
- `python tools/route_check.py --trajectory --map src/dzbrain/tests/fixtures/map_carloop_20261024.yaml` → **rc0 / STATIC GEOMETRY PASS**。
- `ROBOT_MAP=src/dzbrain/tests/fixtures/map_sixth_20261017.yaml python tools/route_check.py` → **rc0（仅 1 提醒）/ 雷 8-8 / 靶 2-2 / 桥洞 K5 K7**。
- `python tools/config_selfcheck.py` → error0/warn0（LAYOUT-MODE）。
- `sim_cli` → **118.12 s / 0.0300 m / 74**（与 pre 同；`field_modeled=0` 未解，桥洞指标须接图才有意义）。

## 风险 / 未验
- sim_cli 无图 ⇒ 控制台不打印桥洞指标（`_FIELD_TABLE` 只在 field_modeled>0 打印）；本卡用 fixture spec 探针验证代码，**接图后必须把 modeled/steps 纳入基线**。
- legacy route_check 依赖 `ROBOT_MAP` 指向 fixture；`config/map.yaml` 为 TRAJ-FRAME-CORE 有意删除（**禁止恢复**）。
- 未跑 test_physics/test_planner 全套：它们硬编码 `config/map.yaml`，与有意删除冲突（收口时改 fixture 口径另办）。

## 回滚
`_tmp_session/R-21/preimage/r21.diff`（sim.py + route_check.py 的完整 diff）；禁 `git checkout`（会连带他人在途改动）。

## 五项必填
- 读过的台账条目：D-039（禁补重复构造器）、D-048（P1 授权）、D-054（本次裁决）。
- 复用检索：复用 `physics.bridge_entry_walls`、`load_cfg`、既有 fixture；未新建第二套几何。
- 新增轮子及理由：无（只修既有实现）。
- 验收命令：见上。
- 回滚点：preimage/r21.diff。
