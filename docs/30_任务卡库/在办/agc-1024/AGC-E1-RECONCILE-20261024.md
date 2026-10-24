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
