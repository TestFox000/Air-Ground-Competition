---
来源: 交接_20261024 §4；manifest.json；用户 2026-10-24 接手指令
取证: 实（车在线，可只读拉取）+ 档
更新: 2026-10-24
status: in_progress
ns: agc-1024
agent: e-integ
树节点: AGC-4
写域: ~~_tmp_session/agc-1024/e1/~~；本卡；只读车端，不改车端文件
摘要: 对本地HEAD/工作树/两份覆盖层/车端src+install 做四方差异，检测 dzrobot2 与 dzrobot 分列，产出可审计对账表与恢复范围。
---

# AGC-E1-RECONCILE-20261024 · 四方版本对账

owner: e-integ；reviewer: lead（独立评审）。

## 1 依据与复用（五必填）

- 读过的台账：D-007/D-019（人工部署、唯一 deploy_to_car 入口）；现行推进计划 §6 E1。
- 复用检索：`tools/deploy_to_car.py probe`、`deploy_car.py` 只读拉取对账、`deploy-verified-hashes.json`、`prep-deployment.json`、R3 窄部署工具（193 项）；**不重写部署器、不手工 scp**。
- 新增轮子：无（对账脚本如需，仅过程目录一次性）。
- 验收命令：见 §3。
- 回滚点：只读阶段车端零写入。

## 2 当前问题

本地 HEAD、工作树、两份覆盖层（prep-candidate / reverse-fix）、车端 src 与 install 五方存在差异（已证：planner 车端为旧接口窄移植；Mission 等于本地 HEAD；Aim 未替换；C++ 保留 lead/EKF/锁存；serial_reader src 与 install 不同）。检测进程来自 `/home/duzhong/dzrobot2/install/det_node`，**必须与 dzrobot 分列**。

## 3 验收

- 表：每个运行模块 → 源码路径 / 配置 / 模型 / install 哈希（完整 SHA256，不用前缀）。
- 未解释差异数 = 0，或逐条显式挂起（写理由与影响）。
- 明确列出“必须保留的已运行补丁”：R3 位置/航向 reset、lead/EKF 进控制环、停止锁存、倒车符号与真实反向确认、权限心跳/离站撤销。
- 顺带产出：det_node 实际来源、类别数、模型指纹（供 P1 判定“漏检 vs 未入画”），及 dzrobot2 是否还有第二写者。

## 4 交付

1. 四方（+检测进程）对账表 + 差异清单（可审计、带命令）。
2. 归并建议：哪些改动应回唯一源码、哪些显式挂起。
3. 窄部署范围与回滚点草案（**不执行部署**；执行需用户确认具体 manifest）。


## 5 执行记录（2026-10-24 17:18–17:42，车在线，全程只读零写入）

**产物**（过程目录 `_tmp_session/agc-1024/e1/`）：
- `RECONCILE_TABLE.md` —— 六方对账表（15 行模块 + 6 条二进制 + 3 个模型），未解释差异 0、显式挂起 3（U1 车端模型未入库 / U2 现场配置未入库 / U3 colcon 前缀链机制）
- `NARROW_DEPLOY_MANIFEST.draft.md` —— 窄部署草案（只改 det_node 启动方式，零文件部署，可秒回滚）
- 原始证据：`car_probe.sh…12.sh` + `car_probe.out…12.out`（只读探测）、`car/`（12 份车端文件，落地哈希与车端逐条一致）、`norm_matrix.py/.out`、`patch_markers.py/.out`、`planner_char.py/.out`

**关键结论**

1. **det_node 三行单列**：实跑 = dzrobot2 3 字段版（9b449ba2，10-20 21:51，源 d14d92df，模型 59aaad60 9.9MB）；车上有未用的 8 字段版（1d0ae071，10-24 09:06，源 a81d0d9b = 本地 HEAD，模型 8b17ec17 8.1MB）；`build/det_node/det_node` d6259195 与 install 仅差 RUNPATH、BuildID 同 ⇒ 09:06 构建即 8 字段版。**运行证据**：`prep-processes.log` pid 602768 = `dzrobot2/install/det_node/…/det_node -p draw:=true`，父进程 `ros2 run det_node`。P1 前提成立。类别数 = 2（`rknn_pt/include/common.h:21`：cls 0=蓝靶/1=红靶），发布 `cls+1` ⇒ 1=蓝/2=红/0=无目标；故 3 字段版必然过不了「≥5 字段」健壮性守卫。
2. **为何解析到 dzrobot2**：`~/.bashrc` 122–129 行两次 source dzrobot2 且从不 source dzrobot/install；实测两套都 source 时 dzrobot 前缀落在 AMENT 索引 56–58（dzrobot2 在 49）⇒ 首匹配 dzrobot2；只 source dzrobot 时索引 8 ⇒ dzrobot 胜出。
3. **两处口径更正**：① mission「等于本地 HEAD」应为「等于本地工作树」（车端 d9d53cb0 与工作树 8a47e341 仅 EOL/行尾空白之差；HEAD 60fd1ac2 语义不同）；② planner 车端**已含** `check_shoot_stops`（5 处），缺的是工作树的 TRACK-PLAN-B1 任务优先规划器（10 个新 def，+428/−21），不是"旧接口窄移植"。
4. **必须保留的已运行补丁 6 条**：R3 reset/两段（serial_reader revfix）、倒车符号+真实反向确认（tracker revfix，原始里程判据）、lead/EKF 进控制环（cpp prep）、停止锁存（cpp prep）、权限心跳/离站撤销（aim+cpp+mission 三处齐备）、以及尚未生效的 8 字段 det_node。
5. **第二写者确认**：dzrobot2/install 61 包 vs dzrobot/install 22 包，**9 个同名包重叠**（car_localization、det_node、dzactuator、dzjudgment、dzrknn_dataprocess、fake_amcl_node、openslam_gmapping、robot_localization、rplidar_ros）且二进制全不同；dzrobot2 无 dzbrain ⇒ mission/tracker/planner 单一来源。dzrobot2 的 dzrknn_dataprocess.cpp 10-21 11:28 被改过但未重建（二进制仍 10-20）⇒ 潜在第三分叉。
6. **现在无 det_node 进程**：17:18 车 up 9 min，ps 无视觉进程；`.ros/log` 最新 10-24 10:19，无 15:xx launch.log ⇒ 15:0x 的 det_node 是人工 `ros2 run`，不是 start_all.sh；`4_det.log` 末次 10-23 21:40（那次是 8 字段版）。 〔未入库：车端 robot_logs〕
7. **模型三份互不相同**：59aaad60(9.9MB, dzrobot2) / 8b17ec17(8.1MB, 车端在用，**不在仓库**) / 12a3b548(8.5MB, 仓库+dzrobot/models)。

**复跑**：`cd _tmp_session/agc-1024/e1 && ssh duzhong@192.168.0.4 'bash -s' < car_probe.sh`；矩阵在 v1 根 `python _tmp_session/agc-1024/e1/norm_matrix.py`。
**回滚点**：只读阶段，车端零写入，无需回滚。

**未做**：未改 `tools/deploy_to_car.py`（本轮无必要）；未部署、未重启、未发车、未出光、未飞。

## 10 文档变更

新增本卡；修改本 ns 任务树节点状态。归档：无。
本次另在过程目录新增 `_tmp_session/agc-1024/e1/RECONCILE_TABLE.md` 与 `NARROW_DEPLOY_MANIFEST.draft.md`（过程稿，按 AGENTS §8.1 不进 `docs/`；结论已在本卡）。

---

## 11 AGC-21 car_snapshot 归档（t10 · 2026-10-24 17:51 采集，只读）

**目的**：关闭 §上方 U1 的「车上是哪几个字节」——把车端检测栈的全部输入字节落进 `artifacts/`，使「模型 → 结果」可复现，且不依赖 `_tmp_session` 存活。

**归档位置**：`artifacts/agc-1024/car_snapshot/`（6 文件 / 29.78 MB）

| 文件 | 字节 | SHA256(前16) | 车上路径 |
|---|---|---|---|
| dzrobot2__det_node | 3,855,872 | 9b449ba21b6c629e | dzrobot2/install/det_node/lib/det_node/det_node |
| dzrobot__det_node | 776,112 | 1d0ae071732db7fa | dzrobot/install/det_node/lib/det_node/det_node |
| dzrobot__light_det.rknn | 8,125,813 | 8b17ec17bfed399d | dzrobot/install/det_node/share/det_node/model/light_det.rknn |
| dzrobot__light_det.rknn.bak_20261023_1915 | 8,487,541 | 12a3b5487e0099f9 | 同目录 .bak_20261023_1915 |
| dzrobot2__light_det.rknn | 9,899,901 | 59aaad606a641d14 | dzrobot2/install/det_node/share/det_node/model/light_det.rknn |
| dzrobot__config_robot.yaml | 76,923 | 49128a01ab95d9a1 | dzrobot/config/robot.yaml |

**对账**：6/6 与 RECONCILE_TABLE §1/§2 行哈希一致，0 处不符；本地 SHA256 == 车端 `sha256sum`（`MANIFEST.json: car_side_hash_verified=true`）。对账明细已写入 `_tmp_session/agc-1024/e1/RECONCILE_TABLE.md` §7。

**U1 关闭依据**：8b17ec17 的来源链 = 10-23 19:12:35 产出 `car_light_det_20261023_1912.rknn`（`_tmp_session/VISION-MODEL-20261023/models/`，另有 `_tmp_session/LEAD-VISION-20261023/target_pose_v2_20261016.rknn` 同哈希不同名）→ 10-23 19:15 换装上车并留 `.bak`（`.bak` = 12a3b548 = 仓库 tracked `dzrobot_v2/models/light_det.v2.3.2.rknn`）。⇒ U1 降级为「来源已定、字节已归档、仅未纳入 version control」。

**新发现的坑**：`dzrobot2/install/det_node/share/det_node/model/` 下有两个**同为 9,899,901 B** 的模型——`light_det.rknn`=59aaad60（实跑加载）与 `light_det2.rknn`=5698ce0e（同尺寸不同字节，未加载）。按尺寸/文件名排查会把两者当同一份。

**复跑**：`cd artifacts/agc-1024/car_snapshot && sha256sum -c MANIFEST.sha256`（6/6 OK，rc=0）；重采集 `python -B -X utf8 _tmp_session/agc-1024/e1/gen_snapshot_manifest.py`（需车在线）。

**边界**：车端仅 `sha256sum`/`stat`/`ls`，本地仅 `scp` 拉取；未写车、未重启、未部署、未发车。

**需 Lead 裁决（未执行）**：① 29.78 MB 的 git 策略——`artifacts/` 未被 `.gitignore` 忽略（现有 5 个 tracked 文件），选项：只提交清单 / 把 `*.rknn` 忽略但提交 MANIFEST / LFS；② 8b17ec17 是 8 字段栈现役模型，建议 M2 前登记进 `dzrobot_v2/models/`（仓库变更，等排期）。

**文档变更（本条追加）**：新增 `artifacts/agc-1024/car_snapshot/README.md`、`MANIFEST.sha256`、`MANIFEST.json`（artifacts 属 §8.6 豁免区，不入 docs 索引）；修改本卡与 `_tmp_session/agc-1024/e1/RECONCILE_TABLE.md` §7。

---

## 12 AGC-20 独立评审：DL1 合入候选（t9 · 2026-10-24 18:09）

**对象（pin 到字节，变了即评审作废）**：`dzudp_adapter.py` `64c93b3b…`(43,443 B) / `dzudp_core.py` `cd1167f1…`(14,235 B)（评审结束时复算未变）。

**结论：退回（needs_revision）**。①②④⑤ 全过且有证据；③「禁止同 topic 双写」的运行期自检**在真实启动路径上无效**，2 个 high + 1 个 medium 已用脚本复现。完整报告见 `_tmp_session/agc-1024/e1/REVIEW_DL1.md`。

| 项 | 结论 | 关键证据 |
|---|---|---|
| ① PIN 与「只同步不改」 | ✅ | 快照段 == 教员原件**逐字节**（11,821 B / `9ff259a3…`）；变异 M1 改 1 字节、M2 换原件、M3 加第二个分界行、M4 偷改模块常量 —— **四种全部判红**（B0 基线 11 例全绿） |
| ② 不补 coverage/field_layout/不伪造置信度 | ✅ | 真 352 B 13 类帧实跑：报文只有 `target_zone`+`field_verify`，禁止键 0 命中；`absent_in_binary` 显式列出；`bridge_confidence=1.0` 是合成常数、仅在有桥洞时出现且进 `records["synthetic"]` |
| ③ 双写自检 | ❌ 退回 | F1：`acquire()` 全仓只有测试调用，`--selfcheck` 只读 claim ⇒ 第二个实例能过自检（A8 复现：两次 preflight 都 `ok`）；F2：`alive and age<=600` 才红 ⇒ 存活进程的旧 claim 被判「可接管」（A9 复现：alive=True age=3600s → `ok`）；F3：`len(uniq)>1` 在「只有外来者 1 个 publisher」时判 ok（A4） |
| ④ 测试真断言 | ✅ | 90 + 51 = **141 例全绿**，assert 251+157 次，`print(` **0** 次，**0 skip** |
| ⑤ 不调 sniff/bridge | ✅ | 源码 `nav/waypoint` 0 次；真 route 帧 `messages=[]`；`load_parse_core` 断言 `PUBLISH_ENABLED is False`；炸弹测试把 `bridge`/`sniff_loop` 换成炸弹仍跑通 |

**修法（交 dl-drone，单写者不变）**：F1 → `main()` 增加 run 模式做 `acquire()` + 定时刷新 `ts` + `finally release()`（或明确降级为 opt-in 并把互斥交给 supervisor）；F2 → `alive` 为真即 red，`age` 只用于清理死 pid 的残留；F3 → 「`uniq` 里不含本进程节点名」判 red（或先建 publisher 再探测）。

**维护性提醒（非缺陷）**：分界行字节序列在文件里出现 2 次（一次是 `PINNED_MARKER` 赋值本身），测试靠 `MARKER + b"\n"` 才命中真的那条；后人若"简化"成裸 `find(MARKER)` 会取到 11,881 B 的错正文，建议在 `snapshot_payload()` 加一行注释。另：`dzudp_core.py` 头部的复跑命令实跑 90 例而非 105 例（105 是整目录），建议改写。

**边界**：未改被测三文件；变异只在 `_tmp_session/agc-1024/e1/review_mut/**` 副本上做。复跑 `python -B -X utf8 _tmp_session/agc-1024/e1/review_adversarial.py`（B0/M1–M4/A1–A9，末行给 FAILS 清单）。

---

## L 坑库（按症状查 · 本卡产出）

| 症状 | 机制 | 位置 | 怎么避 |
|---|---|---|---|
| 把「同尺寸模型」当成同一份 | `dzrobot2/install/det_node/share/det_node/model/` 下 **`light_det.rknn`=59aaad60** 与 **`light_det2.rknn`=5698ce0e** 都是 **9,899,901 B**，字节不同；实跑加载的是 `light_det.rknn` | 车 `/home/duzhong/dzrobot2/.../model/` | **只认 sha256**，不认文件名/字节数；`ls -l` 分不出 |
| 用「差不多大」判断模型版本 | 8 字段栈模型 `8b17ec17`=8,125,813 B 与旧 `12a3b548`=8,487,541 B 只差 36 万字节，肉眼/字节数都像 | `artifacts/agc-1024/car_snapshot/MANIFEST.sha256` | 同上：`sha256sum` 一条命令定性 |
| 以为 `dzrobot/install` 就是生效前缀 | `~/.bashrc` 两次 source `dzrobot2`、从不 source `dzrobot/install`；两套都 source 时 dzrobot 前缀落在 AMENT 索引 56–58、dzrobot2 在 49 ⇒ **首匹配 dzrobot2** | 车 `~/.bashrc` | 8 字段版必须**绝对路径直启**，`ros2 run det_node` 会解析到 dzrobot2（见 §6 manifest） |

**复评（2026-10-24 18:2x · commit e58aa4a9）→ PASS**：dl-drone 修完 F1–F5。`review_adversarial.py` 复跑 `RESULT ADVERSARIAL CLEAN`；B0 `Ran 11 OK`、M1–M4 四变异全红、A1–A9 全绿（A4/A8/A9 三个原 bug 现在都判红）。套件：`test_dzudp_*` 98 OK、`link/tests` 整目录 **203 OK**、`dl1/tests` 51 OK，断言 280 次、print 0。代码级核对：`--run` 真写 claim + 心跳（tmp+os.replace 原子替换）+ finally release；`if alive: red`；`own_node_names` 判「不属于本进程的 publisher」；F4 把 `PINNED_MARKER` 拆成两段隐式拼接让 marker 字节只出现 1 次（比加注释更彻底）；F5 三条命令标了实际例数。
残留观察（不改 PASS）：①`--run` 只占位不监听，跳过它直接起 DL2 监听则无保护（已写进 docstring 三步部署）；②`--node-name` 默认只含 `_dzudp_selfcheck`，系统在线时重跑 selfcheck 需显式传真实节点名，否则假红；③`_hold_claim` 不检查 `refresh()` 返回 None。

---

## 13 AGC-22 M1 离线路线回归（t11 · 2026-10-24 18:3x–18:5x，纯离线）

### 先说要紧的：一条**一级风险**（机器门禁全过，人工判据不过）

车端 `planner.fallback_to_tokens: true`，默认 `mission_blocks` 是**50 token 老路线（含 `F5*`）**。这条路线在**第七张图**上：

| 门禁 | 结果 |
|---|---|
| `tools/route_check.py`（车端 map b5629d05） | **0 error / 2 warn**（急转弯 17 处、倒车+转向 1 组），总里程 17.85 m |
| `validate_route`（与起栈门禁同一套：`Config.load` → `parse_mission` → `validate_route`，车端 map） | **0 error / 0 warn** |

⇒ **没有任何机器门禁会拦住它**。而 `CAR-PREFLIGHT-20261024` §59 判过「旧 50 点路线足迹无侵入告警，但 **F5\* 射击站非法**，不能回退它」——因为**第七图上 F5 是雷格**（map.yaml mines 含 F5）。

后果：规划器一旦失败（`PlannerError`/超时/坏图），`main` 会按 `fallback_to_tokens` **静默回退**到这条带病路线并出发，正是"带病出车"。

**建议（都不改代码，二选一，请 Lead 裁决）**：M1 现场把 `planner.fallback_to_tokens` 显式设 **false**（失败即停 IDLE，不带病出车）；或把 50 token 里 `F5` 的星号去掉。我**没有改**任何 config。

### Q1 路线从哪来（判据，不是猜）

- 车端 `robot.yaml` 现场值（归档件）：`planner.auto_plan_enabled: true`（:760）、`planner.fallback_to_tokens: true`（:767）、`route.tokens`(=`path_executor.mission_blocks`) = **50 token**。
- 起栈门禁：`main.py:142 load_and_check` → `:191-193` `parse_mission(cfg.get("route.tokens"))` + `validate_route(route, map_cfg, cfg)`，任一 error ⇒ **拒绝启动**。
- 执行分支：`auto_plan_enabled=true` 时执行**规划结果**；只有规划失败才按 `fallback_to_tokens` 回退 `mission_blocks`（`core/config.py:245-246` 两个 Spec 的语义就是这个）。
- 本轮实跑（`_tmp_session/carloop-1024/run-intent.json`）用的是**进程私有配置**：`config_delivery: process-private memfd; no remote config file`，`remote_config` 只留痕 ⇒ 车端**没有**这次 54 token 的配置文件，路线完全由私有参数注入。

### Q2 车端 50 token vs 文档 54 点：逐点对齐

**唯一差异**（前后完全一致）：在 `C3` 之后插入 `C4 C5* R:C4 R:C3`（+4 token），并把 `F5*` 降为 `F5`。50 + 4 = 54。

| 版本 | token | 段数 | 里程 | 停站 |
|---|---|---|---|---|
| A 车端默认 `mission_blocks` | 50 | 49 | **17.85 m** | `F9*` `F5*` |
| B 实跑 `run-intent.json` | 54 | 53 | **19.25 m** | `F9*` `C5*` |

⇒ **19.25 m 属 54 token 那版**，与交接快照一致；「54 点」与「53 段」不矛盾：两个 `R:` 相对点（`R:H9 R:H10` / `R:C4 R:C3`）各自展开成段，段数 = 点数 − 1。

**双站判据（route_check 靶覆盖行）**：
- A：`D7<-#12(0.99m)  E7<-#12(0.78m)` ⇒ **两个靶都由同一个停站服务**；`F5*` 对 D7/E7 的距离与 `F9*` **完全相同**（各自 0.990 / 0.783 m）⇒ 是**等距冗余站**，不是"第二最佳射击点"。
- B：`D7<-#12(0.78m)  E7<-#32(0.78m)` 反查为 `F9*→E7 0.78 m`、`C5*→D7 0.78 m` ⇒ **两站各打一面、不重叠**，符合双站口径。

### Q3 兜底路线合法性（两个门禁都跑了）

见上表：50 token 在第七图上 route_check **0 error**、validate_route **0 error/0 warn**；54 token 同样 0 error（route_check 2 warn：急转弯 16、倒车+转向 2 组）。`R:H9/R:H10/R:C4/R:C3` 相对点、`K10/K4` 桥洞（桥洞保护点覆盖 ✅）、`F9*` 停站距离均在体系内正常展开与判过。

⇒ 结论落在 **「路线没算错，是门禁查不到『停站落在雷格』这类语义违规」**。

### virtual_car（`tools/sim_pursuit.py`，54 token，车端 cfg）

`completed=true`、`path_length=19.25 m`、`route_points=54`、`steps=5383`、`final_error=0.0596 m`、越界 0 拍、锚定 24 次、横向 MAX 0.4324 m（D3→E3 90° 角）、倒车段 4 段共 21.92 s、耗时 **107.66 s**。

**逐点累计里程**（`by_segment.csv`，53 段）：`F9` 停站 **4.55 m**（seg#11 G9→F9）、`C5` 停站 **11.90 m**（seg#31 C4→C5，180° 角）、`F5` 14.35 m、终点 `N5` **19.25 m**（seg#52）。

**两个必须记账的边界**：
1. **速度口径不同**：sim 用的是 `robot.yaml` 的 pursuit 速度，不是 M1 的 0.10 m/s。0.10 m/s 下**纯行驶** 19.25/0.10 = **192.5 s**，已超 180 s 要求（与交接"不能宣称达 180 s"一致）；107.66 s 不能当 M1 时间预算。
2. **停站时长没有离线预言机**：`sim.py` 是纯循迹仿真，不建模任务层停站（本次 `field_modeled=0.0`，输出里没有停站时长）。取停站时长只能：mission 层单测（`test_dual_shoot.py`）或实跑遥测（run2：F9 停 6.1 s / 倒车 53 拍）。**不建议为此新写任务仿真器**（AGENTS 禁重造仿真器）。

### 禁止跳点（审计）

- 跳点都带**显式原因**，不是静默：`mission.py:3268` `_skipped_at[idx] = "停滞兜底 {t}s / 累计转 ..."`、`:3910` `_skipped_at[i] = detail`。
- 但有一条默认开启的**任务级跳站**：`mission.skip_second_stop_after_hit` 默认 **True**（"第一射击停站发数达标 ⇒ 跳过后续射击停站"）。车端 yaml 只在 :797 的注释里提到、**未显式置值** ⇒ **M2 若按默认值跑，第二次停站不会发生**，双站验收会被它挡住。M1 禁火不触发这条。

### 复跑

```bash
cd D:/User/Desktop/比赛存档/v1/dzrobot_v2
python -B -X utf8 tools/route_check.py --config ../_tmp_session/agc-1024/e1/m1/car_cfg/robot.yaml            # A: 50 token
python -B -X utf8 tools/route_check.py --config ../_tmp_session/agc-1024/e1/m1/car_cfg/robot.yaml --route "$(cat ../_tmp_session/agc-1024/e1/m1/route54.txt)"   # B: 54 token
python -B -X utf8 ../_tmp_session/agc-1024/e1/m1_validate.py      # validate_route（起栈同一套）两条路线
python -B -X utf8 tools/sim_pursuit.py --config ../_tmp_session/agc-1024/e1/m1/car_cfg/robot.yaml --route ../_tmp_session/agc-1024/e1/m1/route54.txt --out-dir ../_tmp_session/agc-1024/e1/m1/sim54 --json
```

### 边界

未改 `planner.py` / `mission.py` / 任何 config；未部署；未实车。所有中间件在 `_tmp_session/agc-1024/e1/m1/**`（含车端 cfg 副本、两条路线文件、sim 的 traj/by_segment CSV）。

---

## 14 AGC-24 M1 实验配置副本（t13 · 2026-10-24 19:0x，只改一行，未推车）

### 产物

| 项 | 值 |
|---|---|
| 副本路径 | `_tmp_session/agc-1024/e1/m1_cfg/robot_cfg_m1.yaml` |
| 来源 | 车端 `/home/duzhong/dzrobot/config/robot.yaml` 只读 scp 拉取（源 sha256 `49128a01ab95d9a1…`，76,923 B） |
| 副本 sha256 | `76889ff6401c0096e558942db0215d214a6b1c4b893277c0d69e6a2319204c7d` |
| 与源差异 | **正好 1 行**（见下 diff） |

```diff
@@ -764,7 +764,7 @@
       map_path: "config/map.yaml"     # 解析顺序：绝对路径 → 相对配置文件目录 → 相对工作区
                                       # ★ 2026-10-11 t3:换图就是**覆盖这个文件**(第四张图识图版),
                                       #   所有消费方(route_check/planner/sim/config_selfcheck)按此名查找
-      fallback_to_tokens: true        # 规划失败（PlannerError/超时/坏图）→ warn 后回退 route.tokens，仍出发
+      fallback_to_tokens: false        # 规划失败（PlannerError/超时/坏图）→ warn 后回退 route.tokens，仍出发
       allow_reverse: false            # ★真车底盘不支持倒车 → 规划器走无倒车模式（0 段 R:，折返交 PIVOT）
       max_seconds: 2.0                # 规划时间预算；超出抛 PlannerError → 走上面的回退分支
       pivot_penalty: 5.0              # ★ 折返惩罚/处；与 mission_blocks 生成参数一致（--pivot-penalty 5），
```

注释行**故意保持原文**（未改措辞）——它的下一条 `allow_reverse/max_seconds` 都是规划失败相关，改注释会让人以为动过别的键。

### 语义级校验：不只 diff 行数，而是**逐键比**

用 `Config.load()` 把「车端原件」与「副本」都展平成配置字典再比（这比 diff 更强：注释/空白/键序都不影响，只有真正的键值会露头）：

```
orig keys=264  mine keys=264
planner.fallback_to_tokens: orig=True mine=False
配置字典差异键数 = 1
   planner.fallback_to_tokens               True -> False
RESULT ONLY-FALLBACK-CHANGED
```
关键键回归（必须原样）：`planner.auto_plan_enabled=True`、`planner.map_path=config/map.yaml`、`planner.allow_reverse=False`、`planner.max_seconds=2.0`、`planner.pivot_penalty=5.0`、`planner.footprint_gate=warn`、`route.tokens` n=50、`mission.skip_second_stop_after_hit=True`（M1 禁火不涉及；**M2 前必须显式置 false**）。

### 可加载性（仓内 config_selfcheck，副本 + 车端 map）

```
路线: 50 点 | 射击停站 [F9*, F5*] | 倒车 [R:H9, R:H10]
[配置校验] 0 条非 info 问题
[路线校验] 0 error / 0 warn
结论：error 0 / warn 0  ✅ 通过
```

### M1 启动命令（车端零文件改动，利用 start_all.sh 已有的 `ROBOT_CONFIG` 开关）

现场事实（只读取证）：`/home/duzhong/dzrobot/start_all.sh`
- `:23  CONFIG="${ROBOT_CONFIG:-$WS/config/robot.yaml}"` ← **原生支持换配置**
- `:147 echo "配置中心: $CONFIG"` ← **横幅**，启动日志第一屏就写着用的是哪份配置
- `:220 python3 -u -m dzbrain.main --config "$CONFIG" --map "$WS/config/map.yaml" --force`

GO 时三步（**本次均未执行**）：

```bash
# ① 投递副本（只往 /tmp 写，不碰车端 config/）
ssh duzhong@192.168.0.4 "mkdir -p /tmp/agc1024_m1"
scp _tmp_session/agc-1024/e1/m1_cfg/robot_cfg_m1.yaml duzhong@192.168.0.4:/tmp/agc1024_m1/robot_cfg_m1.yaml
# ② 车上校验字节一致（必须输出 76889ff6…）
ssh duzhong@192.168.0.4 "sha256sum /tmp/agc1024_m1/robot_cfg_m1.yaml"
# ③ 带配置起栈（横幅会打印「配置中心: /tmp/agc1024_m1/robot_cfg_m1.yaml」）
ssh duzhong@192.168.0.4 "ROBOT_CONFIG=/tmp/agc1024_m1/robot_cfg_m1.yaml bash /home/duzhong/dzrobot/start_all.sh restart"
```

### 回滚命令（一条，回到车端默认配置）

```bash
ssh duzhong@192.168.0.4 "bash /home/duzhong/dzrobot/start_all.sh restart; rm -rf /tmp/agc1024_m1"
```

车端 `config/robot.yaml` / `config/map.yaml` **从未被改**（`49128a01` / `b5629d05` 原样），所以回滚不需要恢复任何文件——只要不带 `ROBOT_CONFIG` 重启即可。

### 两个必须说明的口径

1. **`--force` 是 start_all.sh 既有行为**（`:220`，2026-10-11 用户令：起栈提速，自检卸载）。所以 M1 的保护**不是**靠启动门禁（它被 `--force` 跳过），而是靠这份配置里的 `fallback_to_tokens=false`：规划失败 → `PlannerError` → 停在 IDLE，不再回退到 F5* 那条带病路线。
2. **备选投递方式**：上一轮实跑用的是 memfd（`_tmp_session/carloop-1024/reverse-fix/run2-launch-stdin.py`：把 yaml 写进 `os.memfd_create` 再把 fd 当 `--config` 传），车上不留文件。若要求"零落盘"，改用那条骨架；本轮推荐 `/tmp` 形式，因为**车上一句 `sha256sum` 就能核对字节**，审计成本更低。

### 边界

未推车、未改车端任何文件、未起栈、未发车。副本与脚本全部在 `_tmp_session/agc-1024/e1/m1_cfg/**`（`_tmp_session` 已在 .gitignore，不入库；可复现靠上面的 sha256 与 diff）。
