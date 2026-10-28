---
卡号: RECT-REVIEW
ns: agc-1026
agent: gate-reviewer
owner: gate-reviewer
reviewer: lead
树节点: R-6
写域: "_tmp_session/AGC-RECTIFY-20261026/review/及本卡"
status: in_progress
创建: 2026-10-26
---

# RECT-REVIEW · Lead作者预检改动的独立复核

## 目标
task-16：用DeepSeek Account/deepseek-flash/max独立复跑，不得修改作者生产工具/测试来迁就结果。预检可接受不等于可发车；当前任务目标仍为完赛优先、预算内压雷、两有效最佳站与命中。

## 读过的台账条目
D-008冻结区、D-020首段可靠GO（非全图先齐）、D-023无固定旧场图真源、D-033靶速0.3m/s、D-044不自动提交/推送。本轮另核 D-008 相关 path_executor.auto_start 双真源（见执行记录 §7）。

## 复用检索结果
先读 docs/README.md → 本 ns 树；复用现行 tools/config_selfcheck.py、其新测试与显式 fixture（src/dzbrain/tests/fixtures/map_sixth_20261017.yaml 等）、route.validate._map_section；不重造整套门禁。基线改用 git show HEAD:tools/config_selfcheck.py 的字节（sha 02adafcd…fe9319，与作者 baseline 相同），不采信作者自报。

## 新增轮子及理由
只新增复核探针三支（review/round1/probe.py、probe_residual.py、probe_typeerror.py）与故障数据，均为只读调用 + 写本目录，不是永久预检/部署入口。

## 验收命令与清单
真实 cwd=D:\User\Desktop\比赛存档\v1；env=PYTHONUTF8=1 / PYTHONIOENCODING=utf-8 / PYTHONDONTWRITEBYTECODE=1；Python 3.12；真实 returncode 见 review/round1/results.json。
- [x] 全文审作者 diff 和精确源码 hash（含开工后漂移登记，见 §1）
- [x] 独立正常 fixture 与边界/畸形/缺失输入复跑，PASS/FAIL/UNKNOWN 分开（60 例 + 8 例 + 3 例：PASS 51 / FAIL 0 / OBSERVE 9，另 11 例观察/复核）
- [x] 结构化失败/退出码及 quiet 不假绿；输入缺失不当空世界（I01..I36、C01..C05、quiet 三组对照）
- [x] 指出作用域和缺证：静态单测不证明实车 GO（见 §9）
- [ ] Lead 独立认可（作者不得自勾，保持未勾）

## 写域/回滚点
只写本卡及 _tmp_session/AGC-RECTIFY-20261026/review/。生产工具/测试/config 零修改（写入前核 git status：被审两文件仍只有作者改动，reviewer 未触碰）。回滚：删除 review/round1/fixtures/ 即可（脚本重建）；baseline_ref/ 与 results.json 保留作证据。

## 执行记录

### 0. 身份与路由
gate-reviewer · DeepSeek Account / deepseek-flash / max（用户 2026-10-26 18:34 指令）。共享任务 task-4 已 list/get/claim（revision 1→2，owner=gate-reviewer）。

### 1. 被审对象与一处必须记录的漂移
开工时被审：tools/config_selfcheck.py sha aca63c8e…d407e2（6420 B）、测试 ecf6b575…6299e76（7138 B）。
复跑时发现两者已在 19:29:21 / 19:30:42 被改写为 83e23f94…f1ab1b（9251 B）与 d3d27786…526ec9（10545 B）。
- 递归按 sha256 搜 _tmp_session：aca63c8e 无任何副本（hits: []）⇒ 对那一版的逐字节复核只能判 UNKNOWN。
- 本轮证据全部锚定现行 83e23f94 + d3d27786，并在 results.json 记录当次 sha 与 mtime；作者内部复核注释（F1/F2/F4）出现在新版测试里，说明这是复核后的修订版。
- 流程缺陷（C4）：冻结应先留 preimage；没有 preimage 时，“你审的那一版”无法复现。

### 2. 复跑命令（可复制）
cd D:\User\Desktop\比赛存档\v1
python -B -X utf8 _tmp_session/AGC-RECTIFY-20261026/review/round1/probe.py
python -B -X utf8 _tmp_session/AGC-RECTIFY-20261026/review/round1/probe_residual.py
python -B -X utf8 _tmp_session/AGC-RECTIFY-20261026/review/round1/probe_typeerror.py

产出：round1/results.json（60 例 + 21 条基线逐字节对比 + 作者测试复跑 + 默认调用）、round1/residual_false_green.json、round1/typeerror_probe.json。证据索引 review/round1/README.md。

### 3. 硬证据 A：有效输入输出不变
HEAD 字节（02adafcd…，用 __file__ 锚定 exec 运行）vs 现行，13 例：12 例 rc/stdout/stderr 逐字节相同；唯一差异 V03 是改进——点号格式图 HEAD 打印 图名 = None，现行打印真实图名。quiet 有效输入同样逐字节相同。

### 4. 硬证据 B：残余假绿反例（能骗过现判据）
- E01：mines/obstacles/targets 全 []、bridges 非空 → rc=0，warn 3，“有条件通过”。退出码 0 = 自动化放行“空世界”。
- E02/E03/E04：单独空 mines / obstacles / targets → rc=0，warn 1；路线块仍印“✅ 路线校验通过”。
- E06：8 颗雷全填 A1（重复格）→ rc=0，0 warn，✅ 通过。内容非法无判据，计数仍 8/8。
- O05：配置根写错（/dzbrain→robot，0 个已注册键）→ rc=0，warn 83；mission.auto_start 显示成 True。静默回落默认 + legacy 迁移。
- 对照：bridges: [] 单独出现已 rc=1 + MAP_INPUT_ERROR（E05）⇒ “空集合判错”只落在 bridges 一个字段。

### 5. 修复确实补到的洞（HEAD → 现行）
- 地图缺 mines 字段（I17）：HEAD rc=0 ✅ 通过（假绿）→ 现行 rc=1 NO-GO、MAP_INPUT_ERROR。
- 四集合全 []（O01）：HEAD rc=0 ✅ 通过（假绿）→ 现行 rc=1（因 bridges 空）。
- 根为列表（I11）：HEAD traceback、无 RESULT_JSON → 现行 rc=1、1 条 RESULT_JSON、无 traceback。
- 配置越界（I31）：HEAD rc=1 但先打整份报告 → 现行早退 CONFIG_VALIDATION_ERROR + RESULT_JSON。
- mission_blocks: 5（G01）：同类路径 HEAD traceback → 现行 rc=1、RESULT_JSON=INTERNAL_ERROR、无 traceback。

### 6. --config-only 契约（C01..C05）
rc=3 + status=CONFIG-ONLY + launch_go=false + spatial_checked=false + route_status=UNKNOWN；不读地图也能出（C02 用不存在的 map 仍 rc3）；坏配置照样 rc1（C03）；quiet 同契约（C04）；错根配置 rc3 且把 warn 计数写进 JSON（C05）。设计上正确：配置可用 ≠ 发车放行。

### 7. 条目 C（起栈自动发车）独立复核：线索成立
详见 review/auto-start/evidence.md。要点：
- config/robot.yaml:320 旧段 path_executor.auto_start: true @337、delay 45.0 @338、mission_blocks 68 点、mission_reset: N10,y；/dzbrain 段 @704 mission.auto_start: false、delay 10.0。WORKTREE 与 HEAD 都 split=true，非在途改动。
- 旧核 path_executor.py:437-440 读自己那段，850-851 挂定时器 → 1460 自动下发 68 点；AUTO_START 硬默认 @171 = True。新核 mission.py:1501 只读 mission 段。
- start_all.sh:213 LEGACY_PATH=1 → 216 旧核；默认 219-221 新核 ⇒ 默认栈当前不发车，回退栈 45 s 自行出发。
- 预检只打印 mission.auto_start（37-47/124），不读旧段：实测 O06 打印 False + ✅ 通过（rc=0），而旧段是 true。test_deploy.py:199-208 不钉两段一致；start_all.sh:217 文案“8~10 秒自动出发”与实际口径相反。
- 判定：清单第 4 项「起栈自动发车」未闭合，P1 安全可见性缺口。若把 split 直接判 NO-GO，现网默认档会立刻变红（真分裂），建议先对齐 config / 先 warn 一周期再升 error。

### 8. 结论：有条件接受（不接受“可发车”语义）
现行 83e23f94 + d3d27786 相对 HEAD 是实质改进，失败闭合、结构化 NO-GO、有效输入输出不变三项证据成立；但有条件：
- C1（必须）空集合退出码统一：mines/obstacles/targets == [] 必须 error，或至少 rc≠0 + launch_go=false。现在 rc=0（E01/E02）。理由：工具 docstring 承诺“退出码可直接用在部署前门禁”，自动化只读 rc。
- C2（必须）配置根/键识别：0 个已注册键（错根）必须 error 级；并读 path_executor.auto_start 两段、分裂即显式判据 + RESULT_JSON 字段（条目 C）。
- C3（应修）失败分类：mission_blocks: 5 同类类型错落成 INTERNAL_ERROR（G01），而 mission_blocks: 0 是 CONFIG_VALIDATION_ERROR（G03）——分类不一致，闸门日志难判“输入错 vs 工具 bug”。
- C4（流程）冻结留 preimage：aca63c8e 无副本导致逐字节复核 UNKNOWN；下一轮增量必须留旧字节 + sha 清单，增量复核对象唯一。

### 9. 这张工具到底证明到哪一步
证明：预检工具自身对坏输入是失败闭合的（缺图/坏 YAML/非 mapping/类型错/配置越界/route 解析错 → rc=1 + 结构化 RESULT_JSON + 无 traceback；quiet 不吞；有效输入输出不变；--config-only 不会冒充放行）。
不证明：实车 GO、场地真值、路线物理可走、定位/自瞄/云台可用、部署后行为；地图内容合法性也只到“字段存在且类型对”，不校验真值（E06 重复雷仍绿）。另：清单第 4 项其余两部分（单写者、同 run 视频/bag 采集失败路径）本轮无实现可审，判 UNKNOWN；不做设备动作。

### 10. 未证项
1) aca63c8e 逐字节复核（副本已不存在）；
2) 实车/物理/场地真值（禁设备动作，未部署）；
3) 单写者机制、同 run 视频/bag 采集失败路径（仓库内 grep 无对应工具/判据）；
4) LEGACY_PATH 运行期取值（只能读脚本分支，不能判现场启动方式）；
5) 地图“内容真值”（静态检查天然不可判，需真图/现场证据）。

## 文档变更
本卡由 pending 转为执行记录（agent/owner=gate-reviewer，status=in_progress，等待 Lead 最终认可与增量复核）；review/ 新增证据索引 README.md、round1/README.md、auto-start/evidence.md。树、共享状态、台账、索引未改（Lead 单写）。

## 经验总结
- 现象：预检工具在“缺集合字段/空集合/错根配置”上曾以 rc0 放行；工具被改写但无 preimage，导致复核对象漂移。
- 机制：判据只看“字段存在且类型对”，未区分“显式空 = 主张”与“缺失 = 未知”；失败闭合只覆盖解析层，没覆盖语义层；mission.auto_start 与 path_executor.auto_start 双真源、迁移单向，使预检显示与真实执行器取值相反。
- 位置：tools/config_selfcheck.py:_map_evidence（空集合只拦 bridges）、verdict 分支（warn ⇒ rc0）、core/config.py:695-754（单向迁移）、path_executor.py:437-440/850-851。
- 处置：C1/C2 升 error 或改 rc；C3 收紧分类；C4 冻结留 preimage；条目 C 按最小整改加两段读取与 AUTOSTART_ARMED。
- 证据：_tmp_session/AGC-RECTIFY-20261026/review/round1/results.json、residual_false_green.json、typeerror_probe.json、auto-start/evidence.md。
- 落点：本卡 + Lead 的 RECT-GATE 增量；机制类结论待 Lead 决定是否入决策台账/D-xxx（reviewer 不写台账）。
