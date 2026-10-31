---
卡号: WB-3-INCIDENTS-20261031
ns: wb-1031
agent: wb-plant
树节点: wb-3
kind: implementation
status: pending
创建: 2026-10-31 21:2x
owner: 待领
reviewer: wb-reviewer
objective: 把 16 条现场循迹翻车案例变成可一键重放的场景夹具，并补上「事件注入 API」（reset yaw 跳变 / odom 长时间丢样 / 单帧位姿突跳）与「无人机步进 stub」，使台架能真正复现那 10 条需要状态机×噪声同场的案例。
acceptance:
  - `fixtures/incidents/` 下 16 条案例各一份场景（路线 + 注入 + 期望症状 + 出处引用），`--scenario incident:<ID>` 可直接选
  - 每条场景的期望症状写成机器判据（计数器值 / 指标区间 / 日志事件），不是自然语言描述
  - realistic 档下 10 条「需同场」案例至少 8 条能被复现（红=复现出症状，符合预期即绿）；不可复现的每条写明为什么（缺哪个模型）
  - 事件注入 API 三个入口可用：`--inject yaw-jump=<deg>` / `--inject drop-s=<s>` / `--inject pose-step=<m>`
  - 无人机步进 stub：只产 `field_layout` 序列驱动 StepLoop→Tracker 闭环，不含天眼/侦察精度模型（用户裁决 9）
  - 每条夹具的出处必须给 `文件:行` 或文档章节，无出处的案例不许入库（防止把猜测当事实）
verify:
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && PYTHONPATH=src/dzbrain PYTHONIOENCODING=utf-8 python tools/sim_rig.py --list-incidents
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && PYTHONPATH=src/dzbrain PYTHONIOENCODING=utf-8 python tools/sim_rig.py --scenario incident:I02-K9 --arm realistic --json | tail -1
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && PYTHONPATH=src/dzbrain PYTHONIOENCODING=utf-8 python tools/sim_rig.py --scenario incident:I03-H8 --arm realistic --json | tail -1
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && PYTHONPATH=src/dzbrain PYTHONIOENCODING=utf-8 python src/dzbrain/tests/test_wb3_incidents.py
inScope: dzrobot_v2/src/dzbrain/tests/fixtures/incidents/；dzrobot_v2/tools/wb_events.py；dzrobot_v2/src/dzbrain/tests/test_wb3_incidents.py；docs/30_任务卡库/在办/wb-1031/WB-3-INCIDENTS-20261031.md（执行记录）
outOfScope: dzrobot_v2/src/dzbrain/dzbrain/**；dzrobot_v2/config/**；dzrobot_v2/tools/sim_rig.py 与 sim_world.py 的主体结构（只调用不改写；若需改，提卡给 wb-core）；dzsmartscout/**；插件目录
---

# WB-3 · 案例场景库 + 事件注入 + 无人机步进 stub

## 案例清单（16 条，逐条带出处；本卡把它们变成夹具）

| ID | 症状 | 根因 | 证据 | realistic 可复现？ |
|---|---|---|---|---|
| I01 | s 冻 7.70 而车在 16.30，追幻影，横偏 4.101 m | 窗口+弧长地板成自洽不动点，无 relock 出口 | `tasks/tracker.py:437-468` | 需 rig |
| I02 | K9：s 一步 +12.824 m，减速失效 0.12→0.35，直线冲出边界 | 全路线重投影只判「更近」，无跳跃上限 | `localization.py:74-80`；`tracker.py:98-103` | 需 rig |
| I03 | H8：s 冻 2.800、投影方向不一致 ×1695、每 12 s 跳点 | 负速度被 0.4 s 平滑窗吃掉，前冲 0.44 m | `tracker.py:124-136,524-618` | 需 rig + 平滑窗 |
| I04 | F11 折返簇每拍翻转 601 次、车几乎不动 | 自指环：命令符号→选段→is_reverse→符号 | `tracker.py:405-429` | 需 rig |
| I05 | PIVOT 无限重入，240 s 只走 4.4/34 m | 完成/超时未标记「已处理顶点」 | `tracker.py:241-247,814-875` | 需 rig |
| I06 | 兜底 12 s 强跳 E11/A6/B6 | `pivot_dist 0.25 > arrival 0.12` | `tracker.py:76-82` | 需 rig |
| I07 | 到点报错差半格/一格 | `_install_route()` 不复位 `_stall_last_s` | 症-10；`空地链路/02 §3(D2)` | 需 rig |
| I08 | 180° 画大弧 0.37–0.41 m → 漏压 B2（0.0364/阈 0.035） | 顶点切内 | `core/config.py:105`；车端/15 §8.2 | tier-0 已可测 |
| I09 | 紧凑 U 形被整段切掉，车心离顶点仅 0.240 m | 前视 0.36–0.40 ≥ 一格 0.35 | 车端/15 §8.2；`robot.yaml:166-167` | tier-0 已可测 |
| I10 | 假 drift_stop：正常切角 0.304~0.308 攒够 8 拍急停 | `drift_warn_m=0.30` 与顶点窗口打架 | `localization.py:86-93` | 需 rig + 20 Hz |
| I11 | 重锚一步 0.303 m → 突跳>0.20 → 清窗+停机 | 无单拍步长上限 | `localization.py:283-288,1246-1249` | 需事件注入 |
| I12 | 段 1 四航点横偏清一色 0.350 m | 布局 L7 / 复位 N10 / odom L7 三值各异 | `drone.py:1097-1102`；`mission.py:1923` | 需起点三方建模 |
| I13 | 吸附造误差 0.000→0.119；末点吸附致提前 FINISH ~11 cm | `arrival_m=0.12` 用于末点 | 历史/08 坑④；`localization.py:82-88` | 需 rig |
| I14 | RMS 0.124 / max 0.68，劣化 2.8× | `accel_up/down=0.0` 斜坡关 | 五链审计:19,32；`robot.yaml:171-172` | tier-0 已可测 |
| I15 | PIVOT/ALIGN 段冲出去（限速被绕过） | 分支覆盖 controller 限速，出口缺钳位 | 症-16；车端/02 §6 | 需 `P_MOTION_LIMIT` 入环 |
| I16 | 车半路停死须重启栈 | yaw 闩锁（后改 20 帧自恢复） | 症-08；车端/04 §3/§6 | 需 yaw-jump 注入 |

## 无人机步进 stub（用户裁决 9 的收窄形态）

只做三件事：① 按段吐 `field_layout`（known/unknown 逐格）驱动 StepLoop；② 可配置「提前给全图 / 逐段给 / 中途断供」三种节奏；③ 断供时验证车端行为符合 B 区裁决（不无限 WAIT_INFO）。**不做**：天眼模型、侦察覆盖率、航线优化。

## 回滚点

全为新增文件（`fixtures/incidents/`、`tools/wb_events.py`〔未实现〕、新测试）⇒ 逆转 = 删除；不动 wb-2 的 `sim_world.py`/`sim_rig.py` 主体结构。
