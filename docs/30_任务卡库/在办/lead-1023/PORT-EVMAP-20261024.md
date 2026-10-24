---
卡号: PORT-EVMAP-20261024
ns: lead-1023
agent: lead（暂挂，领卡时改派）
树节点: L-10
status: pending
创建: 2026-10-24
owner: 待指派（lead 仲裁 SHOOT-LINK 冻结冲突后放领）
写域: src/dzbrain/dzbrain/core/evidential_map.py（新增）；src/dzbrain/tests/test_evidential_map.py（新增）；src/dzbrain/dzbrain/tasks/aim.py（最小只读钩子，≤40 行）；src/dzbrain/dzbrain/tasks/drone.py（情报喂入点，可选——lead 裁成 aim-only 则不碰）；src/dzbrain/dzbrain/core/config.py（SCHEMA 登记）；config/robot.yaml（flag，默认 false）；tools/wf/frozen.json（登记）；_tmp_session/PORT-EVMAP-20261024/（过程稿）
---

# PORT-EVMAP-20261024 · 证据格地图移植（云端 evidential_map → 车端情报累积层 + aim 保险闸钩子）

> 本卡是**草稿**（C2 复核的交付物之一），未领卡、未落树。依据：EVAL 复核（_tmp_session/AGC3/c2/EVAL.md）——09 文档 §2 矩阵 7 行中 5 行仍成立、2 行行号漂移、0 行失效；云端文件 10408 B 与 10-18 逐位一致。

## 1. 背景与读过的台账条目

云端 `ag_integrate/evidential_map.py`（222 行 / 10408 B / sha256 1551CE1E…8042CF69）是 log-odds 证据格地图：正证据累积 + 画质门负证据 + 粘性否决 + 雷减半 + 半衰 + 双门限快照。本地对应能力是 frozenset 布尔集合（drone.py:571/1169）+ 片级 complete 闸（drone.py:991），无累积、无置信度、无滞后。

读过的台账：
- **D-030**（靶不翻面、圆面；红蓝门控=保险闸）——本卡把 aim 侧定位为保险闸增强的直接依据；
- **D-033**（靶速 0.3 m/s，0.70 m 两格轨道）——TrackSpot 缓行的依据；D-031 已被取代，仅沿革；
- **D-029**（火控口径、aspect 0.60~1.60 现役）；
- **D-035 / D-036 / D-037**（sim_cli --json / preflight / AGC2 三态契约——本卡离线验收沿用同款只读契约形态）；
- 09 借鉴文档 §2/§9（status=计划，P0-1 未开卡——本卡即其落地形态）；
- AGC3-20261024（母卡，L-9.2）。

## 2. 复用检索结论

- 复用清单 + 能力索引：**无证据累积/证据地图能力**；能力索引中「无人机建图/情报发布」在机端（dzsmartscout/tools/perception），车端不重算感知——本卡是车端**消费层**的累积语义，不撞车。
- 全仓 grep `logit|log_odds|EvidentialMap|证据格`：dzrobot_v2 内零命中（belief 命中全是定位信念位姿，无关）。
- 可复用件：`scout/fov.py:70` `cell_centers()`（154 格名+格心真源，行 1..11 × 列 A..N）；`io/contract.py` `ShotClass` 与 `decode_coverage`；drone.py `CoverageLedger`（已探明格账）；tests/fakes.py Rig/假桥 + 注入 clock（test_aim.py 模式）。
- 云端依赖适配：`CellRef(col,row)` → 本地格名 "A1".."N11" 字符串（fov.py 真源）；`ObjectType` → 本地三类 mines/obstacles/targets + ShotClass.BLUE/RED；`field_frame.COLS/ROWS` → geometry 的 MAP_COLS/MAP_ROWS（drone.py:329 已以 154 为分母）。

## 3. 新增轮子及理由

**新增 `core/evidential_map.py`**（纯 Python、零 ROS、时钟注入）：为什么现有不够——frozenset 布尔不能表达「见过几次 / 置信多少 / 单次清晰空视能不能清掉一个误检 / 已确认目标会不会被单次负证据闪断」；complete 闸只有片级有无，没有格级累积。移植范围（抄/改/缓）：

- **抄**：log-odds 累积 + 置信度夹挤（:52-57,:106-107）；负证据纪律=画质/complete 门（:134-135，与本地 complete 闸对接）；雷负证据减半（:67-68）；confirm/drop 双门限快照（:52-53,:192-207）；粘性否决（:58-62,:111-121）。
- **改**：格坐标与类别枚举按 §2 适配表换本地真源；参数走 Config 点号键，SCHEMA 全登记（禁止 _opt 兜底键——SHOOT-LINK 已立此先例）。
- **缓行（不进本卡）**：半衰期衰减（600 s 半衰 ≫ 一局时标，且 `drone.info_stale_s=30s` 硬闸已在；保留参数默认大值等效关闭）；TrackSpot 轨道级（D-030/D-033 后价值密度低，等 drone 情报落图后再评，二期另卡）。

aim.py 侧只接**最小只读钩子**：开火判据前查一次证据图红面信念（仅当 flag 开），命中粘性红 → 记原因码并吞许可；**不发 fire_gate、不改现有 fire_only_blue/red_early_leave 语义**（保险闸叠加，不替换）。实验形态 = feature flag `aim.evidential_gate` 默认 false（铁律 3）。

## 4. 验收命令（全离线，在 dzrobot_v2/ 下）

1. `python -B -X utf8 src\dzbrain\tests\test_evidential_map.py` → exit 0（新建纯脚本测试，模式照 test_aim.py：注入 clock、无 rclpy、GBK 安全；用例覆盖：两次高置信确认 / 单帧误检不进门 / 画质不足不产生负证据 / 雷减半 / 粘性确认后负证据与衰减不击穿 / 双门限无闪断 / 格名适配层 154 格全合法）。
2. `python -B -X utf8 src\dzbrain\tests\test_aim.py` → exit 0（122/122 不红；flag 默认关时行为逐位不变）。
3. `python -B -X utf8 src\dzbrain\tests\test_drone.py` → exit 0（若动 drone.py 喂入点则必跑，否则仍跑作回归）。
4. `python tools\wf\check.py` → exit 0（冻结区 in_flight 登记在途未过期）。
5. 根目录 `python docs/_工具/check_doc_policy.py` + `check_anchors.py` → 无新增红。
6. 不上机、不联网；实车验证另开卡等车端窗口。

## 5. 回滚点

- 新模块 + 新测试：**删除即回滚**（纯新增文件，零存量改动）；
- aim.py 钩子：flag 默认 false 即行为回滚；物理回滚 = git checkout 单文件；
- config/robot.yaml：删 flag 行即回滚；
- 冻结区登记：frozen.json 条目随卡闭环摘除；
- 全部改动进 git 历史（卡号后缀提交）。

## 6. 冻结区登记需求（写域冲突提示）

src/dzbrain/dzbrain/** 与 config/** 全在 frozen.json frozen 列表。本卡需登记 in_flight：
`core/evidential_map.py`（新增也须登记，通配覆盖）、`tasks/aim.py`、`core/config.py`、`config/robot.yaml`、（可选）`tasks/drone.py`。

**冲突**：aim.py、core/config.py、config/robot.yaml 现被 SHOOT-LINK-20261023 占用（owner lead，expires 2026-11-07）。请 lead 二选一：(a) 等 SHOOT-LINK 闭环后串行；(b) lead 合并登记并把两卡改动在同一评审里过。**不允许未登记先动手。**

## 7. 风险与边界

- D-030 后红面是保险闸场景：本卡价值主战场在 **drone 情报累积层**（差异事件 → 可累积信念），aim 钩子是顺带增强；若 lead 裁决「drone 侧也不做」，整卡可撤销，EVAL 结论仍成立。
- 云端 blob:none 克隆的 tests/test_evidential_map.py（blob 783e3e18）未物化，执行期若有网应先取它对齐用例口径；取不到就按本卡 §4-1 的用例清单自建，差异写进卡。
- 一行只做一件事：本卡不含 pursuit/ortools/blackbox 的任何移植（09 文档 P0-2/P1-x 各自另卡）。

## 8. 结论判定

（收口时填）

## 9. 文档变更（§8.8 预告）

- 新增：本卡（在办/lead-1023/PORT-EVMAP-20261024.md，待 lead 落树后移动）；
- 修改：09 借鉴文档 §9 状态表 P0-1 行「未开卡 → 已开卡 PORT-EVMAP-20261024」，并在 §2 注记 C2 复核的行号漂移（:143→:134、:90 失效、aim.py:1294→:1300）；
- 台账：新增 D-038（或下一空号）记录「移植范围收窄：情报累积层 + 保险闸钩子，半衰期/TrackSpot 缓行」的裁决；
- 归档落点（§9.5）：20_资料索引库/车端/09 篇加反向指针「来源: PORT-EVMAP-20261024」；
- 索引：gen_index.py --ns lead-1023 重扫。
