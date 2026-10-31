---
卡号: WB-7-HANDSHAKE-20261031
ns: wb-1031
agent: wb-doc
树节点: wb-7
kind: implementation
status: pending
创建: 2026-10-31 21:2x
owner: 待领
reviewer: wb-reviewer
objective: 完成两件事：① 运行面 A 区降级的握手落地（`config/robot.yaml` 加一行 `step.layout_ttl_s: 0.0`，让 `gate_policy()` 从判 hard 转为 warn），并随口径把 stepper 旧拦停断言一并改判；② tier-0 守护套件（test_sim/test_sim_guard）的陈旧钉按现 68 点路线重钉归零 + 本线文档真源收口（能力索引/复用清单/06 册）。
acceptance:
  - `python tools/config_selfcheck.py --config config/robot.yaml` 退出码 0（0 error 0 warn）
  - 启动日志出现 `policy=warn`（此前实测为 `policy=hard explicit_ttl=False`）
  - `grep -n layout_ttl_s` 三处口径一致（SCHEMA / 配置 / 运行时日志）
  - test_stepper 随新口径重钉后零红（旧拦停断言改判「保留意图、只换机制」，不许直接删断言）
  - tier-0：`test_sim.py`、`test_sim_guard.py` 退出码 0（陈旧钉按现 68 点路线/现出厂常量重钉，红因逐条写明）
  - `tools/sim_map_render.py` 的抽样口径给出处置（修 or 明确标 legacy），并同步能力索引:13 的「唯一渲染器」登记与现状一致
  - 复用登记：本线新增件（`tools/sim_world.py`〔未实现〕/`sim_rig.py`〔未实现〕/`map_synth.py`〔未实现〕/`track_sweep.py`〔未实现〕）全部进 `复用清单.md` + `能力索引.md`（tier-0/tier-1 并列关系写清，避免后来人再造第四套）
  - `python tools/wf/check.py` GREEN 且 `check_reuse` 无新增未登记
verify:
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && python tools/config_selfcheck.py --config config/robot.yaml; echo rc=$?
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && PYTHONPATH=src/dzbrain PYTHONIOENCODING=utf-8 python src/dzbrain/tests/test_stepper.py; echo rc=$?
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && PYTHONPATH=src/dzbrain PYTHONIOENCODING=utf-8 python src/dzbrain/tests/test_sim.py; echo rc=$?
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && PYTHONPATH=src/dzbrain PYTHONIOENCODING=utf-8 python src/dzbrain/tests/test_sim_guard.py; echo rc=$?
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && python tools/wf/check.py; echo rc=$?
inScope: dzrobot_v2/config/robot.yaml（仅一行握手键，已登记冻结区在途）；dzrobot_v2/src/dzbrain/tests/test_stepper.py；dzrobot_v2/src/dzbrain/tests/test_sim.py；dzrobot_v2/src/dzbrain/tests/test_sim_guard.py；dzrobot_v2/docs/{复用清单,能力索引}.md；docs/20_资料索引库/空地链路/**；docs/20_资料索引库/车端/**（仅索引行）
outOfScope: dzrobot_v2/src/dzbrain/dzbrain/**（含 main.py 的 gate_policy 实现——它已按 rt-gate 落地，本卡只补配置侧）；插件目录；`tools/sim*.py` 的实现逻辑（处置结论写文档，改动另立卡）
---

# WB-7 · 运行面握手 + tier-0 重钉 + 文档真源收口

## 读过的台账条目

D-026（不把仿真绿当实车绿）；D-055 / B 区裁决（一场一图、取消保质期 `step.layout_ttl_s→0`）；D-063（本线依据）；AGENTS §1 铁律⑤「一次一变量」——故握手与重钉分两步跑，不混在同一次改动里判因。

## 为什么这一行必须你来点头（已在册事实，非猜测）

rt-gate 交付报告风险①实测：`python -m dzbrain.main` 起动打印 `policy=hard explicit_ttl=False`。我本轮复核并**更正 rt-gate 的一句归因**：不是「`Config.get` 对缺失键不回落 SCHEMA」——`core/config.py` 的 `Config.get` 对缺失键**会**回落 SCHEMA 默认（`:790` 区）。真实机制是 `gate_policy()` 用 `_cfg_explicit()`：**要求配置里显式写了这一行才算表态**，没写就判 hard，且 `main.py:533` 注明这是刻意的（「配置没表态 ⇒ 语义不漂移」）。

⇒ 所以这不是 bug，是**一次需要人授权的握手**：A 区（互锁降级）与 B 区（一场一图）是同一次裁决的两半，握手点就是配置里那一行。`config/**` 属冻结区，我不擅自加。

## 副作用清单（必须预告，不许静默）

加这一行会让依赖旧拦停语义的断言翻红：`test_stepper` 10.x / 14.x（rt-gate 已预告）。处置原则 = 「**保留意图、只换机制**」（与 R-8 断言改写同一口径），**禁止删断言换绿**。

## tier-0 重钉为什么排在这里

本轮测试网审计实测：`test_sim` 102/113→106/113、`test_sim_guard` 100/110→106/110 在审计窗口内被并发会话持续移动（18:04/18:08/18:12/18:32 分别改过 bridge_gate/physics/link_e2e/route）。**在被改写的基线上钉数值 = 把钉子钉进沙**。等握手落地、口径固定后再钉，本卡就是那个"口径固定"的时点。

## 文档收口三处（现存与现状矛盾，须人裁定后写）

| 项 | 在册说法 | 现状实测 | 处置 |
|---|---|---|---|
| 渲染器唯一权威 | `能力索引.md:13`：`tools/sim_map_render.py` 唯一渲染器 | 实为两套；python 侧默认 `--map` 指已删图（实测 rc=1）、零测试引用、抽样 bug 仍在（`:272 i%4` 套 `:277 i%125` ⇒ 实际 500 拍=10 s，注释写 2.5 s）；插件 SVG 已自行修正 | 本卡给出处置并同步登记 |
| 仿真台唯一权威 | `能力索引.md:12` 插件四文件 | 现行确为 `dsh-dzairground-ssh`，但没挂进 bundles（verify FAIL） | 随 wb-6 enable 后补登记 |
| tier-0/tier-1 关系 | 无此概念 | 本轮结论：`sim_cli` 保留为回归基线、rig 为闭环台架 | 新增登记行，写明「不得互为替代品」 |

## 回滚点

`config/robot.yaml` 与三个测试文件改前存 `_tmp_session/wb-1031/wb7/preimage/` + sha256；逆转 = 删那一行（回到 `policy=hard`，行为与今日一致）+ 还原测试文件。禁止 `git checkout`（该文件另有他人 in_flight 登记，条目见 `frozen.json`，不动其内容）。
