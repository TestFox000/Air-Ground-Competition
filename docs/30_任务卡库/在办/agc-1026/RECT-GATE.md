---
卡号: RECT-GATE
ns: agc-1026
agent: lead
owner: lead
reviewer: lead
树节点: R-3
写域: "dzrobot_v2/tools/config_selfcheck.py；dzrobot_v2/src/dzbrain/tests/test_config_selfcheck.py；_tmp_session/AGC-RECTIFY-20261026/gate/"
status: in_progress
创建: 2026-10-26
---

# RECT-GATE · 预检失败闭合与协同作业安全

## 目标
用户本轮目标：优先完赛，在安全和时间余量内尽量压雷、两有效最佳射击点并命中。不默认为禁火仅跑路线。共享任务 task-14，由执行者按list/get/claim流程领取，agent/owner改为真实target。

## 读过的台账条目
Lead已读D-008冻结登记、D-012唯一tour入口、D-023每局真图不纳管为默认、D-033现场靶速0.3m/s、D-044不自动commit/push。执行者须补充本卡相关条目。旧注释0.04/5.6px/s只作历史，不得重开口径。

## 复用检索结果
先读docs/README.md，用doc_search检索本主题；再读dzrobot_v2/docs/复用清单.md→能力索引.md→目录索引.md。复用night1025的工具与证据但不信其自报验收；此段由执行者补命中项。

## 新增轮子及理由
不新建规划器/仿真器/部署器；只补缺口测试，若新增永久工具必须先说明既有不足并请Lead登记。

## 写域
dzrobot_v2/tools/config_selfcheck.py；dzrobot_v2/src/dzbrain/tests/test_config_selfcheck.py；_tmp_session/AGC-RECTIFY-20261026/gate/及本卡；其余生产代码只读，现有未提交改动不得覆盖。补丁先在隔离副本；只有RECT-GATE可直接改所列非冻结文件。不碰共享登记/索引/状态板。

## 验收命令与清单
由执行者补完整cwd/env/命令及预期；环境统一python -B -X utf8，真实returncode而非tail退出码，日志UTF-8，PASS/FAIL/UNKNOWN分开。
- [ ] 缺图默认结构化非零而非traceback，空间证据UNKNOWN不能GO
- [ ] 显式config-only只证明配置，报表不冒充发车绿
- [ ] 显式夹具、坏配置、坏图、缺图和JSON契约有退出码测试
- [ ] 起栈自动发车、单写者、同run视频/bag门禁复用清单齐备
- [ ] Lead独立复跑后认可（作者不得自行勾此项）


### 18:5x–19:3x · 子会话连续中断后 Lead 接手补完 config-only

- 事实：四个子会话（raccoon 与 deepseek-account 两轮）均在**请求阶段**失败，无任何执行产出；Lead 依"反复失败可接手"补此小修，不替代其余三线结论。
- 新增 `--config-only`：只校验配置，空间证据记 UNKNOWN，`spatial_checked=false`，**退出码 3**（与 0=通过 / 1=失败 区分）。理由：静态 `map.yaml` 已按 D-023 删除，若逼操作者"为了拿绿还原旧图"，等于用假空间证据换假放行。
- 实测两条路径（cwd=`dzrobot_v2`）：
  - 默认调用（无 map）→ **RC=1**，RESULT_JSON `status=NO-GO / route_status=UNKNOWN / error=MAP_INPUT_ERROR`；
  - `--config-only` → **RC=3**，`status=CONFIG-ONLY / launch_go=false / spatial_checked=false`，结论行明写"不是发车放行"。
- 测试 **17/17 通过**（新增 config-only 两条：不冒充放行 / 坏配置仍 RC=1）。
- 当前工具 sha256=`ea9f6ed032a0715c71c3a769e32ea21a39375da7030cc2c9d78fa46d44b7535d`；证据目录 `_tmp_session/AGC-RECTIFY-20261026/gate/round3_configonly/`。
- 仍未勾：起栈自动发车 / 布局单写者 / 同 run 视频·bag 复用联测（第 4 项），以及独立 reviewer 复核。**config-only 通过 ≠ 可发车。**

## 回滚点
修改前记录sha256和逐文件副本到本卡_tmp目录；只逆转本卡差异，禁止git reset/clean/整树覆盖/提交/推送。

## 执行记录
- 17:44首批子任务与17:49单任务重试均因raccoon TRANSPORT fetch failed在请求阶段失败，未改代码。Lead按‘执行者反复失败可接手’只接此小修，不替代其余三线。
- 已改config_selfcheck.py：输入错误不再traceback；map缺失/坏yaml/非mapping/非UTF8一律非零并输出RESULT_JSON，launch_go=false、route_status=UNKNOWN；配置输入错误同理。未添加config-only/宽松跳过，不恢复静态地图。
- 复用现有测试夹具；新增test_config_selfcheck.py 8项CLI故障注入。修前7失败/rc1，修后8通过/rc0；有效fixture输出与修前逐字一致。
- 原始证据：`_tmp_session/AGC-RECTIFY-20261026/gate/verify_result.json`、同目录before.log/after.log；复跑`python -B -X utf8 _tmp_session/AGC-RECTIFY-20261026/gate/verify_gate.py`退出0。
- 工具sha256=63d1011c1ed41df013d6d9f48be2ad015d64dd514f8d424590af26beeda60257。
- **仅作者自测通过，独立review待模型入口恢复，任务不关闭、不报可发车。**
- 测试编写曾有一处字符串转义SyntaxError，已修正并重跑，不影响生产工具。


### Goal续轮1：空地图假绿补充整改（2026-10-26 18:10后）

- 原8项测试复跑虽绿，扩展审查却复现：空YAML/false/0被`safe_load(...) or {}`吞成空场地，当前路线竟输出error0/warn0/rc0；map:null/map:[]和标量/列表配置仍可traceback。未因此签独立验收。
- 只改本卡非冻结工具和测试：输入必须为非空mapping；复用route.validate的`_map_section`标准化地图；start/end及雷/障/靶/洞字段必须显式提供且类型正确，缺失不是观测为空。配置越界先输出NO-GO，路线解析错误同样结构化失败；不改默认地图、不跳过检查。
- 测试扩到15项（包含参数化子例）；修前扩展测试rc1，修后rc0；另10个补充探针全部符合非零+NO-GO+无traceback。显式正常fixture的旧输出保持不变，扁平/嵌套地图校验一致。
- 当前默认配置无map仍rc1，route_status=UNKNOWN、launch_go=false；此处NO-GO不是要求赛前全场侦察完成，真实动态段另需当趟有效已知走廊证据。
- 当前工具sha256=aca63c8e71660c1476c053b07a42943cc2cd6281d4e54388b538e6fc412407e2。对20项前置hash核查仅该工具变化，算法/配置未改。
- 新复跑：`python -B -X utf8 _tmp_session/AGC-RECTIFY-20261026/gate/capture_round1.py`；结果与原始before/after输出在[本轮证据](../../../../_tmp_session/AGC-RECTIFY-20261026/gate/round1_results/20261026_181003_814771/result.json)。补充探针结果在[review.json](../../../../_tmp_session/AGC-RECTIFY-20261026/gate/round1_review/20261026_181013_712675/review.json)。较早verify_result只对应首版，不覆盖此次修订。
- 本轮前副本：`gate/config_selfcheck.before_round1.py`、`gate/test_config_selfcheck.before_round1.py`；原始副本仍在baseline。回滚须核当前hash，只撤本卡差异。
- 以上均为Lead接手后的作者补测；**独立reviewer仍未取得，清单保持未勾，卡保持in_progress**。config-only、安全起栈与同run采集部分未完成。

### Lead 只读发现：起栈自动发车目前无预检覆盖（19:0x，待独立复核）

- 双真源分裂（**HEAD 已提交态**，非本轮在途改动；`git -C dzrobot_v2 diff -- config/robot.yaml` 无 auto_start 行）：
  - `dzrobot_v2/config/robot.yaml:337` 旧段 `path_executor.ros__parameters.auto_start: true`（delay 45.0，行 338），行内注释自证「本轮现场发车：翻 true 推车，跑完回 false」；
  - `dzrobot_v2/config/robot.yaml:704` 新段 `mission.auto_start: false`。
- 消费方分叉：旧核 `src/car_localization/car_localization/path_executor.py:441-446` 读自己那段；新核 `src/dzbrain/dzbrain/tasks/mission.py:1501` 只读 `mission.auto_start`；`start_all.sh:213` `LEGACY_PATH=1` 回退旧核 ⇒ 起栈 45s 后自行出发，绕过横幅核对窗口。
- 迁移是单向补键：`core/config.py:695-754` `_migrate_legacy` 在 `mission.auto_start` 已存在时不会反向覆盖，故两类消费者各读各的。
- 预检现状：本卡工具只在 `CHECK_KEYS`（`tools/config_selfcheck.py:47`）与打印段（124-125 行）呈现 `mission.auto_start`，**不读旧段、无 GO 判据**；`src/dzbrain/tests/test_deploy.py:180-208` 只钉「归一化 true→false、配置行恰好 2 处 false」，不钉两段原值一致，故分裂不会让它红。
- 状态：**线索已交 `gate-reviewer`（task-4）独立判定**；本轮不据此改工具、不改冻结 `config/`。倾向的最小整改（待复核同意后另起断言）：预检把两段都读出，取值不一致即非零/告警，并把 LEGACY_PATH 回退的 auto_start 纳入「起栈自动发车」判据。

### auto_start 双真源线索（19:2x Lead 只读发现；**来源更正**：非 gate-reviewer 报文，其独立复现见下节）

判定：**线索成立**，性质为「条件性真实发车 + 预检盲区」，不是默认路径必然发车；定级 **P1 安全可见性缺口**（非 P0 现网立即危险）。

独立复现值（只读，未起栈）：
- `robot.yaml:337` legacy `auto_start: true` + `338 delay 45.0`；该段 `mission_blocks` 68 点（首 5：N10 M10 M9 M8 M7），`mission_reset: N10,y`；`:704/705` mission.auto_start=false / delay 10.0。
- WORKTREE 与 `git show HEAD:config/robot.yaml` **同为 legacy=true / mission=false（split=True）**；work vs HEAD diff 27 行无 auto_start 行。
- `cfg.get('mission.auto_start')=False`、`cfg.get('path_executor.auto_start')=True`、`path_executor.auto_start_delay=45.0`，且 `cfg.validate()` 中 auto_start 相关 issue **0 条** ⇒ 分裂不产生任何 error/warn。
- 旧核消费点实为 `path_executor.py:437-440`（读值）、`850-851`（ARM 定时器）、`1460`（`_auto_start_mission` 下发 68 点）；`171` 行 `AUTO_START` 硬默认 True。新核仍是 `mission.py:1501`。
- `start_all.sh:213` → `216` 走 `path_executor_node`；默认 `219-221` 走 dzbrain ⇒ **默认栈安全，回退栈 45s 自发车**。
- 预检盲区确认：`config_selfcheck.py:37-47` CHECK_KEYS 只有 mission.auto_start、`124` 只打印，全文件无 auto_start 判据且从不读 path_executor 段；`wf/check.py` grep auto_start=0；`deploy_to_car.py --force-no-autostart` 仅 opt-in 且改车端文件；`test_deploy.py:199-208` 不钉两段一致，其 208 行「本仓两段已显式 false」注释与现网 legacy=true 不符。
- 旁证：`start_all.sh:217` echo 写「（启动后约 8~10 秒自动出发）」，与实际 false/10.0 口径相反，属误导性提示。

整改方案（复核方给，Lead 决定落地顺序）：
1) 预检增读 `path_executor.auto_start` / `auto_start_delay` 并打印；
2) 两值不一致（split）→ error 级 NO-GO；
3) 任一为 true → 默认 error（AUTOSTART_ARMED，RESULT_JSON 带两值与各自 delay），仅显式 `--allow-autostart` 降 warn；
4) 不动冻结 config，用 `--stack new|legacy|auto` 表达起栈方式；
5) 同轮修正 `test_deploy.py:208` 陈旧注释与 `start_all.sh:217` 文案。

**落地顺序决定（Lead，19:2x）**：当前被审工具 sha `aca63c8e…` 的独立 review 仍在跑，**冻结工具不并发改**；等 review 主结论落定后，按「一次只改一个变量」另起一轮 auto_start 增量，并把增量 diff 交 gate-reviewer 做增量复核。冻结 `config/` 仍不动。


### gate-reviewer 独立复核（19:0x–19:3x，子会话 1d7df6dc；Lead 已据此修工具）

**结论：窄修复接受；RECT-GATE 整卡不验收；发车 NO-GO/UNKNOWN。** 独立 57 例探针（PASS 44 / FAIL 3 / INFO 10），脚本与原始输出在 `_tmp_session/AGC-RECTIFY-20261026/review/20261026_1916_1d7df6dc/`。

被审对象为 sha `aca63c8e…`（+61/−8）。它确认：缺图/空图/坏根/非UTF-8/字段缺失或类型错/配置越界 → rc1 + 单条 RESULT_JSON + 无 traceback；合法嵌套/扁平/点号键与改前 baseline **逐字一致**；20 项前置 hash 仅本工具漂移；wf RED3 为既有删图+localization，未新增红。

它同时抓到 4 项（Lead 已逐项处理，属**改动后待增量复核**）：

| 编号 | 复核发现 | Lead 处置 |
|---|---|---|
| F1 | 嵌套类型错（`mission_blocks: 5` → 迁移成 `route.tokens=5`）在 `core/config.py:859` 抛 TypeError，无 RESULT_JSON | `main()` 包 `_run()` 顶层兜底：任何未预料异常 → `INTERNAL_ERROR` + RESULT_JSON + rc1；测试用最小配置复现该 TypeError |
| F2 | `mines: []` 静默关闭"雷 8/8"，rc0 ✅；`bridges: []` 还与车端 `drone.py:242 allow_empty=False` 冲突 | `bridges` 空表判 `MAP_INPUT_ERROR`；`mines/obstacles/targets` 空表不判错但打 `[显式空集合]` 行并计入 warn，结论不再印 ✅ |
| F3 | `pursuit: []` / `route: "oops"` 整段回落默认值，只 warn1、rc0"✅通过" | 措辞改：warn>0 时结论印 **⚠ 有条件通过（禁止只看退出码）**；默认值证明问题仍留待更大改动 |
| F4 | 等价性测试在 baseline 缺失时静默 skip → "OK(skipped)" 不再验证 | 先断言契约（rc0 / 含 ✅通过 / 无 traceback），再可选比 baseline |

另两条它明确挂账、本轮**未修**：F3 的"关键键来自配置而非默认"仍需专门断言；单写者 / 同 run 视频·bag·裁判计数仓内无采集器，保持 **UNKNOWN**，不能作发车依据。

它要求的归属更正已做：原"gate-reviewer 独立复核结论"小节改为 **Lead 只读发现**（数值它已独立复现成立）。

当前状态：工具与测试已改到 **20 项全绿**，但 sha 已不是它审过的 `aca63c8e…`；增量复核**未取得**（该子会话已结束、无法续话），故本卡**仍不勾验收、不发车**。

### 19:4x 证据身份校正与裁决（按独立复核 round3）

- 卡内早先写的 `ea9f6ed0`/17 项是中间快照，已作废。**最终身份**：工具 `83e23f94cab03203816bd288e75632a8edfb57b8f331d9a1dbd6cec892f1ab1b`（209 行），测试 `d3d277862bffdafd4d09857c7c7bcf9149d6d5430ecf438e7a8f437f06326ec9`（204 行/20 项）；20 项 OK rc0，另有 57 例与 15 例两轮独立探针。跨快照不得引用"绿"——期间曾因把 path_executor 写在 /dzbrain 下出现 1 失败，19:30 修正后才是上述 sha 的绿。
- **空表策略裁决（Lead）**：比赛图不硬编码"雷 8/靶≥1"，因为每局真图不同，硬编码就是把某一局当真理。规则改为：空集合必须**显式声明**并计入 warn、结论降为「有条件通过」；`bridges:[]` 保持硬错（与车端 `drone.py` 布局协议 `allow_empty=False` 一致，空桥洞会让车永远过不去）。现场判据另设：**当趟布局若 known 已达可观覆盖而雷/靶仍为 0，必须由人工对照真实场地确认后才允许发车**，这条进放行单 G4，不靠预检自动放行。
- 独立复核对 `83e23f94`+`d3d27786` 的"输入失败结构化 NO-GO / config-only 不冒充发车 / 合法输入不回归"范围**接受**；对整卡**不验收**（本轮已记）——第 4 项起栈 auto_start、布局单写者、同 run 视频/bag 仍未闭合。
- 复核指出的来源问题已处理：本卡该节原先被写成"gate-reviewer 独立复核结论"，实际是 Lead 只读发现，非 reviewer 发出。

## 未验证
本轮不部署、不起栈、不发车、不出光、不飞行。不为求绿放宽安全阈值/取消闩锁/伪造靶。

## 文档变更
新增本卡；树和索引由Lead单写。

## 经验总结
待复核后提炼，来源/结论回指项目资料库。
