---
卡号: M1-BRAINMAP
ns: agc-1026
agent: 6719d8a0-37ff-45a1-af0b-f24157aa831b
owner: 6719d8a0-37ff-45a1-af0b-f24157aa831b
reviewer: lead
树节点: R-16
写域: "src/dzbrain/dzbrain/main.py + 可选新建 core/run_archive.py + 唯一命名新测试 + _tmp_session/rt-gate/ + 本卡"
status: in_progress
创建: 2026-10-27
---

# M1 · 脑内地图生命周期（核心卡）

## 三件事
1. 一场一图：取消保质期（ttl=0），改逐格 known/unknown；unknown 不触发 WAIT_INFO(layout_stale)。
2. 空白起步：每次启赛建本地运行目录（C/artifacts/runs/<UTC ts>/，该目录已在 .gitignore 忽略），写「只有固定层」的空图；变量由无人机增量注入。
3. 先走边补：不等整图完整；unknown 当可通行但记 unknown_traversed / unknown_cells；同格不换版。

## 验收
- [x] V1 unknown=5 → 继续 + `unknown_cells=5`
- [x] V2 unknown=1 → 不 exit
- [x] V3 同图再读 `map_version 1→1`、`replans 1→1`
- [x] V4 同名再起必 RUNX-1；新进程目录新建、不读上一场

## 五项必填
- 读过的台账条目：D-044（提交/推送等用户令）、D-046（候选验收纠偏）、D-047（离线边界）；本卡关联下条 Lead 裁决。
- 复用检索：复用现有 route/physics、route/planner、core/config、main.py StepLoop、drone.field_layout 协议，不新建第二套同类工具。
- 新增轮子及理由：无（仅在本卡写域内改现有实现）；如需新模块，理由写在本节。
- 验收命令：见「验收」逐条，全部机器可判。
- 回滚点：见「回滚」。

## 执行报告

**结论：一场一图 / unknown 当可通行 / 先走边补 / 同格不换版 / 新进程新目录，本卡 8 项判据全绿（含 2 负例）。**

### 改了什么
- **脑内地图**（`StepLoop._sync_brain`）：布局 `known` **只做增量并集**（无人机只增不减），
  `known` 集合变化才 `map_version += 1`；`_cell_state` 逐格 known/unknown；`unknown_cells = 全图 − known`
  只作日志/归档，**不再是拦停判据**。
- **unknown 当可通行**（`plan_segment_from_layout(degrade=True)`）：`blocked_extra=[]`（不再把未知格当
  不可通行），`bad`（路线穿未知格）由"自检拒收"改为 `GATE[warn] key=layout_incomplete why=route_through_unknown`；
  hard 口径逐字节保留（still 拒发）。
- **unknown_traversed**：`_unknown_traversal` 在 warn 口径只 `ℹ️ INFO key=unknown_traversed block=… hits=…
  map_version=… unknown_cells=…`，不再 SAFE_STOP；hard 口径仍 3 拍 SAFE_STOP。
- **取消保质期**：`step.layout_ttl_s` 缺省 0（SCHEMA 单源），warn 口径下 stale 只记不拦（见 S3 卡）。
- **空白起步**：`run_live` 启动即建 `artifacts/runs/<UTC ts>/`（`.gitignore` 已忽略），写
  `map.json`〔未入库：每场运行生成〕= **只有固定层**（网格尺寸 / 隔板列行列；mines/obstacles/targets/bridges 全空），
  变量由无人机增量注入、本场不回写（见 M2 卡）。
- **不读上一场**：`core/run_archive.py` 无任何读取既有 `runs/` 的代码路径；同名 `run_id` 再起必
  新建 `<id>-N` 目录（`mkdir(exist_ok=False)`）。

### 验收
```
M1.1a unknown_cells == 5（全图 157 格，known=152）      M1.1b 相位 DRIVING（继续跑）
M1.2  1 个 unknown ⇒ 不 exit、不 SAFE_STOP
M1.3a ℹ️ INFO key=unknown_traversed block=A1 hits=1 map_version=1 unknown_cells=5
M1.4a 同一份图再读一次：map_version 1 → 1（不换版）
M1.4b replans 1→1、段请求 1→1（不重算、不解锁）
M1.5a RUNX vs RUNX-1（新进程必新目录）        M1.5b 新目录 map.json 的 mines == []
M1.6a run_archive.py 无读 runs/ 的代码        M1.6b 第二场收尾后第一场 summary 哈希不变
负例 M1.7 hard 口径走过未知格 3 拍仍 SAFE_STOP；
     M1.8 同一份图两口径：degrade 穿 M10 出段 / hard 拒发（未知格/无解）
```
`python src/dzbrain/tests/test_agc_rectify_rt_gate.py` → **69/69**（本卡 M1.1~M1.8）。

### 风险 / 下一步
- 生产要真正进入"一场一图"口径，仍需配置显式表态（`step.layout_ttl_s: 0.0`）；
  否则 `gate_policy=hard`，未知格仍会 3 拍判停（见 G1 卡风险 1）。
- 空白图目前只落**固定层**；隔板/桥洞的"固定义"未与 `config/map.yaml` 交叉核对（该文件当前在工作树里
  被删/待裁决，属他人写域），下一轮应在有图后比一次固定层是否一致。
- 回滚点 `_tmp_session/rt-gate/preimage/`；证据 `_tmp_session/rt-gate/REPORT.md`。
## Lead 复核（2026-10-28 01:1x · §11 独立复跑）

- 独立复跑：`test_agc_rectify_rt_gate.py` **69/69 PASS（exit0）**；`config_selfcheck` error0/warn0；`tools/wf/check.py` **GREEN**（复用登记已补）；`sim_cli` **118.12 s / 0.0300 m / 74**（与改前逐项相同）；`test_stepper` **117/118**（唯一红 12.11 = 基线存量，ttl=0 落地后未再翻红）。
- 配置落地：`config/robot.yaml` 已加 `step.layout_ttl_s: 0.0` ⇒ `gate_policy=warn`（实测），A 区降级在车上生效。
- 见 D-055；未 commit / 未部署 / 未连车。

