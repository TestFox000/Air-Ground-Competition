---
卡号: M2-ARCHIVE
ns: agc-1026
agent: 6719d8a0-37ff-45a1-af0b-f24157aa831b
owner: 6719d8a0-37ff-45a1-af0b-f24157aa831b
reviewer: lead
树节点: R-19
写域: "src/dzbrain/dzbrain/main.py + 可选 core/run_archive.py + 唯一命名新测试 + _tmp_session/rt-gate/ + 本卡"
status: in_progress
创建: 2026-10-27
---

# M2 · 运行归档（新能力，须登记复用清单）

> 落点裁决（Lead 2026-10-27，用户「找个合适的文件夹」）：**C/artifacts/runs/<UTC ts>/**。dzrobot_v2/.gitignore 已有 artifacts/ 忽略规则，归档不入库、不污染两棵树的脏文件计数；车端部署后为 ~/dzrobot/artifacts/runs/。

## 触发
本次运行结束：完赛 / 手动停 / 故障退出 / SIGTERM 都算。

## 产出（每场一个目录）
map.json / route.json / pose.csv / log.txt / trace.jsonl / summary.json
summary.json 必含：时长 / 距离 / 越界次数 / unknown 格数 / warn 计数。

## 硬约束
只写不读 —— 下一场启动不得读上一场任何文件。

## 验收
- [ ] SIGTERM **未证**（Windows 无 POSIX 语义：os.kill=SIGTERM→TerminateProcess，处理器不跑）；只证「body 收停机原因 → finally 必出包」机制层，真机待一次实跑
- [x] summary 六键齐：run_id/duration_s/distance_m/out_of_bounds/unknown_cells/warn_counts
- [x] 上一场目录存在但新场不读（同名再起 RUNX-1；目录新建）

## 五项必填
- 读过的台账条目：D-044（提交/推送等用户令）、D-046（候选验收纠偏）、D-047（离线边界）；本卡关联下条 Lead 裁决。
- 复用检索：复用现有 route/physics、route/planner、core/config、main.py StepLoop、drone.field_layout 协议，不新建第二套同类工具。
- 新增轮子及理由：无（仅在本卡写域内改现有实现）；如需新模块，理由写在本节。
- 验收命令：见「验收」逐条，全部机器可判。
- 回滚点：见「回滚」。

## 执行报告

**结论：新能力 `core/run_archive.py` 落地——一场一目录、只写不读、六文件、summary 六键；停机/故障路径都出包。**

### 改了什么
- 新建 `src/dzbrain/dzbrain/core/run_archive.py`（299 行）：`RunArchive` +
  `install_log_tee` + `RUN_FILES`/`SUMMARY_KEYS`/`POSE_HEADER`。
  - 落点 `artifacts/runs/<UTC ts>/`（`.gitignore` 已忽略；车端部署后即 `~/dzrobot/artifacts/runs/`）；
  - 六文件（每场运行生成〔未入库〕）：`map.json` / `route.json` / `pose.csv` / `log.txt` /
    `trace.jsonl` / `summary.json`；
  - `summary.json` 六键：**run_id / duration_s / distance_m / out_of_bounds / unknown_cells / warn_counts**
    （另附 reason/finished_at/files，供复盘读）；
  - 原子写（`.tmp` + `os.replace`）、`finalize` 幂等、`mkdir(exist_ok=False)` 保证"绝不复用上一场目录"；
  - 里程由位姿采样折线累计；越界拍数、unknown 格数、warn 计数由 `StepLoop`/`GateLedger` 注入。
- `main.py`：`run_live` 拆成 **wrapper + `_run_live_body`**，wrapper 负责"建目录 → tee 日志 → 跑 →
  `finally` 必 `finalize`"，因此**所有**退出路径（完赛 / 手动停 / 故障 / SIGTERM / 等布局超时早期 return）
  都会出包；归档不可用时只打一行 warn、不影响跑车。

### 验收
```
M2.1 六文件齐：{'map.json':True,'route.json':True,'pose.csv':True,'log.txt':True,'trace.jsonl':True,'summary.json':True}
M2.2 summary 六键齐 = run_id / duration_s / distance_m / out_of_bounds / unknown_cells / warn_counts
M2.3 里程 0.7 m（0.3 + 0.4 折线）        M2.4 out_of_bounds=1 / unknown_cells=5
M2.5 warn_counts={'layout_stale': 1}     M2.6 停机路径 rc=0、reason=stop/SIGTERM、六文件齐
负例 M2.7 body 抛异常 ⇒ 仍出包且异常照旧上抛；M2.8 finalize 幂等（不产生第二个目录）
M1.6a run_archive.py 内无"读既有 runs/"的代码（只写不读）
```
`python src/dzbrain/tests/test_agc_rectify_rt_gate.py` → **69/69**（本卡 M2.1~M2.8）。

### 风险 / 下一步
1. **SIGTERM 只证到机制层**：Windows 上 `os.kill(pid,SIGTERM)` 是 TerminateProcess，处理器不跑；
   本轮用"body 收到停机原因 ⇒ finally 必出包"钉住，真 POSIX 实机 SIGTERM 仍**未验证**，不得当已验证。
2. **登记未完成**：`tools/wf/check.py` 现 **RED(1)** = `新增未登记: src/dzbrain/dzbrain/core/run_archive.py`。
   登记落 `dzrobot_v2/docs/复用清单.md`（Lead/RECT-GATE 写域）。建议条目：
   `- \`src/dzbrain/dzbrain/core/run_archive.py\`：运行归档（M2-ARCHIVE / AGC-RECTIFY）；一场一目录、只写不读、六文件 + summary 六键。`
3. 归档目录只在 `run_live` 建；`--dry-run`/`--plan-only` 不建。单测直接调 `run_live` 会在
   `artifacts/runs/` 留临时目录（git 忽略，可随时删）。
4. 回滚点 `_tmp_session/rt-gate/preimage/`；证据 `_tmp_session/rt-gate/REPORT.md`。
## Lead 复核（2026-10-28 01:1x · §11 独立复跑）

- 独立复跑：`test_agc_rectify_rt_gate.py` **69/69 PASS（exit0）**；`config_selfcheck` error0/warn0；`tools/wf/check.py` **GREEN**（复用登记已补）；`sim_cli` **118.12 s / 0.0300 m / 74**（与改前逐项相同）；`test_stepper` **117/118**（唯一红 12.11 = 基线存量，ttl=0 落地后未再翻红）。
- 配置落地：`config/robot.yaml` 已加 `step.layout_ttl_s: 0.0` ⇒ `gate_policy=warn`（实测），A 区降级在车上生效。
- 见 D-055；未 commit / 未部署 / 未连车。

