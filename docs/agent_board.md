# 7-Agent 协同管理看板

> **经理（项目经理）维护**。本看板是 7 个 Agent 的唯一任务来源和进度追踪台。
> **所有 Agent 开工前必读**：第 1 节（名册）→ 第 2 节（你的任务）→ 第 4 节（汇报规范）。

---

## 1. Agent 名册

| 代号 | 名称 | 核心职责 |
|---|---|---|
| **Agent-1** | 车机协同工作任务协同 | 车机接口、链路联调、任务拆分与依赖协调 |
| **Agent-2** | 改进创意 | 识别瓶颈、提出创新改进方案、评估可行性 |
| **Agent-3** | 无人车 | 车端全栈：ag_integrate、朝向对齐、路径执行、禁火保险、实车验证 |
| **Agent-4** | 无人机工作 | 机载端全栈：dzuav、相机、检测、格号识别、PnP、UDP广播、模型训练 |
| **Agent-5** | 赛程控制 | 比赛流程、时间分配、赛前检查、应急方案 |
| **Agent-6** | 成果汇报 | 整理成果、训练指标、验证证据、项目文档 |
| **Agent-7** | 改进建议 | 复盘问题、提出优化建议、跟踪改进闭环 |

---

## 2. 任务分配表（核心）

> **状态说明**：🔵 待领 ｜ 🟡 进行中 ｜ ✅ 完成 ｜ ⛔ 受阻
> **每个 Agent 只做自己代号开头的任务。完成后必须按第 4 节规范汇报。**

### Agent-1 车机协同

| 任务ID | 任务 | 状态 | 截止 | 完成时间 | 进度 |
|---|---|---|---|---|---|
| A1-1 | 车机接口契约梳理 | ✅ | 10-09 | 10-09 20:00 | docs/interface_contract.md 已完成 |
| A1-2 | 车机链路联调计划 | ✅ | 10-09 | 10-09 20:00 | 见接口契约§5 + 赛前检查C节 |
| A1-3 | 路径规划与命令格式对齐 | ✅ | 10-09 | 10-09 20:00 | Held-Karp+Dijkstra 已就绪 |

### Agent-2 改进创意

| 任务ID | 任务 | 状态 | 截止 | 完成时间 | 进度 |
|---|---|---|---|---|---|
| A2-1 | 识别完赛最大瓶颈 | ✅ | 10-09 | 10-09 20:00 | docs/finish_line_analysis.md |
| A2-2 | 提出绕过瓶颈的备选方案 | ✅ | 10-09 | 10-09 20:00 | 每个瓶颈均有降级方案 |

### Agent-3 无人车

| 任务ID | 任务 | 状态 | 截止 | 完成时间 | 进度 |
|---|---|---|---|---|---|
| A3-1 | align 朝向对齐模式代码 | ✅ | 10-09 | 10-10 08:00 | 代码已提交，状态机+闭环转向+/cmd_vel，22测试通过 |
| A3-2 | **实车验证：车能走直线** | 🟡 待实车 | 硬件就绪后1天 | — | 软件就绪，等实车 |
| A3-3 | path_executor 不占 /cmd_vel 确认 | 🟡 待实车 | 硬件就绪后1天 | — | align 模式前提 |
| A3-4 | dry_run 联调→关闭 dry_run | ⛔ | 依赖A3-2 | — | 联调无误后实发 |

### Agent-4 无人机工作

| 任务ID | 任务 | 状态 | 截止 | 完成时间 | 进度 |
|---|---|---|---|---|---|
| A4-1 | **7cls 蒸馏训练收敛** | ✅ | 10-11 16:05 | 10-11 16:05 | epoch 75/75 完成，best mAP50=0.7879(ep56)。评估：官方74张 mine mAP50=0.7972(厂家0.1248)，7cls val 全类 mAP50=0.7771，mine mAP50-95=0.8380。已发布 hbb_detector_7cls_distill_20261011.onnx，current_hbb 指针已更新。注意：car类mAP50-95=0.0000（车自报位置，可接受） |
| A4-2 | **格号识别 v4c 训练** | ✅ | 10-10 | 10-10 08:00 | v4c模型已发布 cell_classifier_v4c_20261009.onnx |
| A4-3 | Orin 编译 dzuav | 🔵 待启动 | Orin上线后1h | — | 需Orin在线 |
| A4-4 | 相机出图验证 | ⛔ | 依赖A4-3 | — | |
| A4-5 | 格号识别离线验证 | ✅ | 10-10 | 10-10 20:05 | 32 val帧：识别率89.0%(1046/1175)，lattice几何校验准确率93.4%(894/957)，跨批次泛化良好 |
| A4-6 | PnP 定位验证 | ⛔ | 依赖A4-3,A4-5 | — | A4-5已完成，待Orin相机帧 |
| A4-7 | UDP 广播验证 | ⛔ | 依赖A4-6 | — | |
| A4-8 | 0x4002 车位姿帧广播 | ✅ | 10-09 | 10-09 | 代码完成，待Orin编译运行 |

### Agent-5 赛程控制

| 任务ID | 任务 | 状态 | 截止 | 完成时间 | 进度 |
|---|---|---|---|---|---|
| A5-1 | 赛前检查清单 | ✅ | 10-09 | 10-09 20:00 | docs/preflight_checklist.md |
| A5-2 | 比赛流程时间线 | ✅ | 10-09 | 10-09 20:00 | 8分钟总时长，7阶段 |
| A5-3 | 应急方案 | ✅ | 10-09 | 10-09 20:00 | 4类应急+降级方案 |

### Agent-6 成果汇报

| 任务ID | 任务 | 状态 | 截止 | 完成时间 | 进度 |
|---|---|---|---|---|---|
| A6-1 | 训练指标曲线整理 | ✅ | 10-09 | 10-09 20:00 | project_summary.md |
| A6-2 | 验证证据汇总 | ✅ | 10-09 | 10-09 20:00 | 7大里程碑 |
| A6-3 | 项目文档更新 | ✅ | 10-09 | 10-09 20:00 | 新增5份文档 |

### Agent-7 改进建议

| 任务ID | 任务 | 状态 | 截止 | 完成时间 | 进度 |
|---|---|---|---|---|---|
| A7-1 | 复盘已知问题清单 | ✅ | 10-09 | 10-09 20:00 | 5个已踩坑+教训 |
| A7-2 | 提出改进建议并跟踪 | ✅ | 10-09 | 10-09 20:00 | 5项优先改进按ROI排序 |

---

## 3. 完赛最小闭环（M0）总览

| # | 任务 | 负责 | 状态 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | Agent-3 | 🟡 软件就绪待实车 |
| M0-2 | 朝向对齐实车验证 | Agent-3 | 🟡 软件就绪待实车 |
| M0-3 | 无人机出情报(UDP) | Agent-4 | 🔵 待Orin上线（v4c离线验证已过，7cls模型已发布；本地端到端管线验证通过：ONNX契约[1,3,1280,1280]→[1,11,33600]+CPU推理431ms，frame_000124检测37雷+3格号，render_field_matrix全链路跑通） |
| M0-4 | 车机UDP链路通 | Agent-1 | ✅ 模拟验证通过 |
| M0-5 | 路径规划可执行 | Agent-1 | ✅ 模拟验证通过 |
| M0-6 | 车按路径行驶 | Agent-3 | ⛔ 依赖M0-1,M0-2 |
| M0-7 | 赛前检查清单 | Agent-5 | ✅ 已完成 |

---

## 4. 进度汇报规范（所有 Agent 必须遵守）

### 4.1 汇报时机
- **任务开始时**：在看板对应任务行把状态改为 🟡，填写"完成时间"列为"进行中"
- **任务完成时**：把状态改为 ✅，填写实际完成时间，在"进度"列简述产出
- **任务受阻时**：把状态改为 ⛔，在"进度"列写明缺什么、等谁

### 4.2 汇报格式
每个任务行必须包含：
```
状态 | 截止时间 | 完成时间(YYYY-MM-DD HH:MM) | 进度简述(产出物路径或结论)
```

### 4.3 信箱消息格式
需要其他 Agent 配合时，在 docs/AI信箱.md 发消息：
```
## [发→收] YYYY-MM-DD HH:MM · 状态
结论 + 需要对方做什么
```

### 4.4 提交规范
```
<类型>(<Agent代号>): <一句话>
例：feat(A3): align模式实车验证通过，零残差
```

---

## 5. 协同规则

1. **一个任务同时只一个 Agent 做**，看到 🟡 别重复动手
2. **改他人模块前在信箱说一声**
3. **配置文件按归属分治**，动对方的段先说
4. **经理每小时巡检一次**，更新看板状态

---

## 6. 当前瓶颈与风险

| # | 风险 | 负责 | 缓解 |
|---|---|---|---|
| R1 | Orin 不在线 | Agent-4 | 确认WiFi连接；同时推进训练 |
| R2 | 车仍画圆 | Agent-3 | align已就绪，待实车 |
| R3 | 7cls训练中断 | Agent-4 | ✅ 训练已于15:38完成epoch 75并发布 hbb_detector_7cls_distill_20261011.onnx；本地ONNX冒烟测试通过。建议禁用Modern Standby防止再次中断 |
| R7 | 5090连接不稳定 | Agent-4 | 5090实例持续不可达（connect.bjb2.seetacloud.com:37113 与 106.38.204.136:52238 均 TcpTestSucceeded=False）。seed157/v4d/target_merged 训练状态未知。两个 watch_5090_v4d_seed.py 后台运行中（PID 33984 + 37436），连通即自动监控+下载。需用户在 AutoDL 控制台检查实例状态 |
| R4 | 0x4002帧未广播 | Agent-4 | 代码完成，待Orin编译 |
| R6 | E盘（项目存储） | 经理 | ✅ 已恢复（7cls续训成功读取E:\ml_datasets），持续监控 |

---

*最后更新：2026-10-12 01:04 经理（每小时巡检）*

## 7. 本轮巡检非硬件推进（2026-10-12 01:04）

### ✅ 全量数据集标签完整性复核（非硬件推进）
- **seed_157**：363 框全部 w>0 h>0（0 无效标签），类别分布 class0=36/1=63/2=76/3=55/4=102/5=31，与 data.yaml 一致
- **target_merged**：1768 框全部 w>0 h>0（0 无效标签），类别分布 class0=211/1=142/2=475/3=83/4=559/5=298
- **v4d**：`cellv4d_upload_v2.zip`（218.4MB，25236 条目）结构正确：cell_classifier/ + cell_labels/ + gen_cell_synth.py
- 三个数据集均已就绪，5090 恢复后可立即部署训练，不会再出现 seed_157 h=0 类致命问题

### ✅ 代码base健康
- `pytest tests/` 128 passed in 18.99s（无回归）

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 两个 `watch_5090_v4d_seed.py` 后台运行中（PID 33984 + 37436），连通即自动监控+下载
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

---

## 8. 上轮巡检非硬件推进（2026-10-12 00:05）

### ✅ 全量就绪确认：数据集+部署脚本+模型指针+测试全绿
- **seed_157**：363 框全部 w>0 h>0（0 无效标签），已部署 5090（训练状态待连回确认）
- **v4d**：`cellv4d_upload_v2.zip`（218.4MB，正确 cell_labels/ 结构）就绪，deploy 脚本 V4D_ZIP 已指向 v2
- **target_merged**：652 帧（555 train + 97 val，6类），`target_merged_upload.zip`（631.9MB）就绪，deploy 脚本已验证（imgsz=960, batch=32, epochs=150）
- **模型指针**：`current_hbb_detector` → `hbb_detector_7cls_distill_20261011.onnx`，`current_cell_classifier` → `cell_classifier_v4c_ft_20261010.onnx`
- **代码base健康**：`pytest tests/` 125 passed, 1 skipped in 16.53s（无回归，跳过项为 onnx 库未安装）
- 5090 恢复后可一键部署 v4d + target_merged 训练，watch 脚本自动下载产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157 训练状态未知，v4d/target_merged 部署待重连
- 两个 `watch_5090_v4d_seed.py` 后台运行中（PID 33984 + 37436），连通即自动监控+下载
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

---

## 8. 上轮巡检非硬件推进（2026-10-11 23:02）

### ✅ 数据集与部署脚本全量就绪确认
- **seed_157**：134 train + 23 val，363 框全部 w>0 h>0（0 无效标签），已部署 5090（训练状态待连回确认）
- **v4d**：`cellv4d_upload_v2.zip`（218.4MB，25236 文件，正确 cell_labels/ 结构）就绪，deploy 脚本 V4D_ZIP 已指向 v2
- **target_merged**：652 帧（555 train + 97 val，6类），`target_merged_upload.zip`（631.9MB）就绪，deploy 脚本已验证（imgsz=960, batch=32, epochs=150）
- 5090 恢复后可一键部署 v4d + target_merged 训练

### ✅ 代码base健康
- `pytest tests/` 128 passed in 19.00s（无回归）

### ✅ 模型发布指针确认
- `current_hbb_detector` → `hbb_detector_7cls_distill_20261011.onnx`（38.44MB，契约 [1,3,1280,1280]→[1,11,33600]）
- `current_cell_classifier` → `cell_classifier_v4c_ft_20261010.onnx`

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` NoValidConnectionsError
- seed157 训练状态未知，v4d/target_merged 部署待重连
- 两个 `watch_5090_v4d_seed.py` 后台运行中（PID 33984 + 37436），连通即自动监控+下载

### 📝 7cls 训练：epoch 75/75 已完成（无变化）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/357（无变化）

---

## 8. 上轮巡检非硬件推进（2026-10-11 22:01）

### ✅ 修复 deploy_5090_v4d_seed.py 的 V4D_ZIP 路径
- 原 `V4D_ZIP` 指向 `cellv4d_upload.zip`（113.6MB，旧版结构错误：缺 cell_labels/ + v4c 图片扁平化）
- 已改为指向 `cellv4d_upload_v2.zip`（218.4MB，25236 文件，正确 cell_labels/ 目录 + merged_v4d.csv）
- 5090 恢复后，`deploy_5090_v4d_seed.py --skip-seed` 将上传正确的 v4d 包并启动训练

### ✅ 代码base健康
- `pytest tests/` 125 passed, 1 skipped in 18.65s（无回归，跳过项为 onnx 库未安装）

### ✅ 数据集就绪确认
- seed_157：363 框（train 261 + val 102），已在 5090 启动训练（状态待连回确认）
- target_merged：1768 框（555 train + 97 val，6类），部署脚本 `deploy_target_merged.py` 已就绪，待 5090 恢复后部署

### ⚠️ 5090 实例不可达
- `connect.bjb2.seetacloud.com:37113` NoValidConnectionsError（3次重试均失败）
- seed157 训练状态未知，v4d 上传待重连
- 两个 `watch_5090_v4d_seed.py` 后台运行中（PID 33984 + 37436），连通即自动监控训练+下载产物

### 📝 7cls 训练：epoch 75/75 已完成（无变化）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

---

## 8. 上轮巡检非硬件推进（2026-10-11 21:40）

### ✅ 5090 实例恢复在线，部署 seed157 + v4d 训练
- 5090 实例（connect.bjb2.seetacloud.com:37113）恢复可达，GPU: RTX 5090 32GB
- **seed157 标靶训练**：已启动，yolo11s.pt 从 /root/ 复制就位（绕过 GitHub 慢下载），AMP 检查中
- **v4d 格号识别训练**：原上传 zip 结构错误（aerial_auto/ 缺失 + v4c 图片扁平化），已重新打包 `cellv4d_upload_v2.zip`（229MB，25236 文件，含正确 cell_labels/ 目录结构 + 预构建 merged_v4d.csv）
- v4d 上传因 5090 网络中断失败（144/229MB），待重连后续传
- watch_5090_v4d_seed.py 双实例运行中，自动重连并监控训练完成

### ✅ seed_157 标签完整性验证
- 全部 363 框（train 261 + val 102）w>0 且 h>0，无无效标签
- 类别分布与 data.yaml 一致，训练就绪

### ✅ 代码base健康
- pytest tests/ 128 passed in 25.55s（无回归）
- 7cls 蒸馏模型已发布，本地端到端验证通过（无变化）

### 📝 manual_200 标注：157/200（无变化）

### ⚠️ 5090 连接不稳定
- SSH/SFTP 频繁中断，已尝试 3 次重连均失败
- seed157 训练已在服务器启动，即使断开也会继续运行
- v4d 上传待重连后重试

---

## 8. 上轮巡检非硬件推进（2026-10-11 20:11）

### 🔴 关键修复：seed_157 标靶种子数据集标签全部无效（h=0）
- **问题**：seed_157/labels 中所有 363 个标注框的 height=0.000000，YOLO 训练会完全失败
- **根因**：旧版转换脚本仅使用矩形前 2 个点（top-left + top-right，y 相同），导致 y2-y1=0
- **修复**：`convert_anylabeling_json.py` 已正确使用全部 4 个点取 min/max
- **增强**：新增 `--preset seed` 参数，支持 seed_157 的 6 类映射（blue bucket=0, car=1, green bucket=2, plane=3, red bucket=4, target=5）
- **验证**：重新生成 363 框，全部 w>0 且 h>0，类别分布与 data.yaml 一致
- **影响**：5090 恢复后 seed157 训练可正常启动，无需人工干预

### 部署就绪确认
- `deploy_5090_v4d_seed.py --wait` 后台运行中，5090 连通即自动打包+上传+训练
- `watch_5090_v4d_seed.py` 后台运行中，训练完成后自动下载产物
- seed_157 标签修复后，pack_seed() 会自动检测到文件更新并重新打包 zip
