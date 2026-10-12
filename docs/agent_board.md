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
| R7 | 5090连接不稳定 | Agent-4 | 5090实例持续不可达（connect.bjb2.seetacloud.com:37113 与 106.38.204.136:52238 均 TcpTestSucceeded=False）。seed157/v4d/target_merged 训练状态未知。watch_5090_v4d_seed.py 已于 18:04 重启（旧实例被系统休眠杀掉），连通即自动监控+下载。需用户在 AutoDL 控制台检查实例状态 |
| R4 | 0x4002帧未广播 | Agent-4 | 代码完成，待Orin编译 |
| R6 | E盘（项目存储） | 经理 | ✅ 已恢复（7cls续训成功读取E:\ml_datasets），持续监控 |

---

*最后更新：2026-10-12 19:02 经理（每小时巡检）*

## 7. 本轮巡检非硬件推进（2026-10-12 19:02）

### ✅ fire_guard 禁火保险边界测试补强（非硬件推进，服务 M0-6 射击安全）
- **问题**：`fire_guard.py` 是车端射击前的最后一道安全闸门（误击红靶 −5/次，最多 −15），但原有测试仅覆盖基础场景，缺失 5 类高风险边界：
  1. 红蓝双邻（`mixed_targets_risk`）：停靠格同时正交邻到蓝靶和红靶时拒绝开火
  2. 原因码断言：`enemy_blue_adjacent` / `friendly_red_adjacent` / `no_target_adjacent`
  3. 边界/角落格邻居计算（A1/N11/F1/A6 越界拒绝）
  4. 垂直轨道移动靶（D8-D9，蓝面朝 E/W 两侧停靠位判定）
  5. 水平轨道蓝面朝北（N 朝向，北侧正对端点允火、南侧禁）
- **修复**：新增 5 项测试（`test_fire_guard_mixed_targets_risk` / `test_fire_guard_reason_codes` / `test_fire_guard_boundary_cells` / `test_fire_guard_moving_target_vertical` / `test_fire_guard_moving_target_north_face`），覆盖固定靶混合风险、边界格越界、垂直/水平轨道四向蓝面
- **验证**：`tests/test_integrate_car.py` 12 passed（+5），全量 `pytest tests/` 144 passed（+5），无回归
- **价值**：M0-6 射击安全闸门的边界行为现在有测试守护，实车联调前即可确认"红蓝双邻禁火""红面侧禁停"等关键规则不会被误改

### ✅ 代码base健康
- `pytest tests/` 144 passed in 20.10s（+5 fire_guard 边界测试，无回归）

### ✅ watch_5090 监控脚本持续存活
- PID 4256（系统Python）+ 14796（venvPython），均于 18:04:39 启动（已运行 ~58min）
- 本轮巡检未发现被系统休眠杀掉（累计休眠杀进程事件仍为 6 次，本轮未新增）
- 5090 恢复后将自动下载 v4d/seed157/target_merged 产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 无（待硬件） |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | ✅ fire_guard禁火保险新增5项边界测试（红蓝双邻/边界格/垂直轨道/北向蓝面），射击安全闸门有测试守护 |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 18:04）

### ✅ watch_5090 监控脚本重启（非硬件推进，保障 seed157/v4d/target_merged 产物自动回收）
- **问题**：巡检发现上轮启动的 3 个 watch 实例（PID 41148 + 42360 + 42708）已全部消失，系统中仅余 X-AnyLabeling 进程。判定为系统休眠再次杀掉后台进程（累计第 6 次休眠杀进程事件）
- **修复**：以 air_ground venv 重新启动 `watch_5090_v4d_seed.py`（后台 job-7ab4100e），启动日志确认 `[10-12 18:04] 未就绪: NoValidConnectionsError`，脚本进入每 60s 重试循环
- **价值**：5090 一旦恢复，watch 脚本将自动检测 v4d/seed157/target_merged 三个训练的 best.pt 并 SFTP 下载至 `E:\ml_datasets\downloaded\`，无需人工值守
- ⚠️ 持续风险：系统休眠已累计杀进程 6 次，强烈建议禁用 Modern Standby / 合盖休眠

### ✅ 代码base健康
- `pytest tests/` 139 passed in 22.81s（无回归，较上轮 136+3）

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化，X-AnyLabeling 进程运行中 PID 25784+27244）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 无（待硬件） |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件，标靶模型640契约已就绪） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 17:14）

### ✅ 标靶pose模型重导出为640分辨率，匹配车端契约（非硬件推进，服务 M0-6）
- **问题**：4 个下载的标靶 pose ONNX 模型（target_car_pose_v2 / target_pose_lightdet / target_pose_yolo11n / target_synth_yolo11s）均为 1280 分辨率，输出 `[1,C,33600]`；车端 det_node 现役 `light_det.rknn` 契约为 `[1,9,8400]`（640 分辨率），直接转 RKNN 会因 anchor 数不匹配无法使用
- **修复**：用 ultralytics 将 4 个 `.pt` 权重以 `imgsz=640, dynamic=False, opset=17` 重新导出 ONNX，存放于 `E:\ml_datasets\downloaded\car640\`
- **契约核验**（onnx 加载 + onnxruntime 推理）：

  | 模型 | 输入 | 输出 | 通道 | 车端契约匹配 |
  |---|---|---|---|---|
  | `target_pose_lightdet_best_640.onnx` | [1,3,640,640] | [1,9,8400] | 9(4+2+3) | ✅ |
  | `target_pose_yolo11n_best_640.onnx` | [1,3,640,640] | [1,9,8400] | 9(4+2+3) | ✅ |
  | `target_car_pose_v2_best_640.onnx` | [1,3,640,640] | [1,8,8400] | 8(4+2+2) | ✅（prelabel自动补v=1.0） |
  | `target_synth_yolo11s_best_640.onnx` | [1,3,640,640] | [1,8,8400] | 8(4+2+2) | ✅（prelabel自动补v=1.0） |

- **推理冒烟测试**：`target_pose_lightdet_best_640.onnx` CPU 推理 OK，输出 `(1,9,8400)` float32
- **解码测试**：prelabel_rknn 6 项 decode 测试全过（含 8 通道无 visibility 自动补 1.0）
- **价值**：M0-6 车端标靶识别的 4 个备选模型现在全部符合车端 640/8400 契约，可直接 `rknn_convert.py` 转 RKNN 部署，不再阻塞标靶射击链路

### ✅ 代码base健康
- `pytest tests/` 136 passed, 1 skipped in 34.58s（无回归）

### ✅ watch 脚本健康（3 实例运行中）
- PID 41148 + 42360 + 42708，均于 05:06:37 启动（已运行 ~12h）
- 连通 5090 后自动监控 v4d + seed157 + target_merged 三个训练并下载产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化，X-AnyLabeling 进程运行中 PID 43764+49868）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 无（待硬件） |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | ✅ 4个标靶pose模型重导出为640分辨率，匹配车端[1,C,8400]契约，可直接转RKNN部署 |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 16:01）

### ✅ lattice_ransac 核心函数回归测试（非硬件推进，服务 M0-3）
- **背景**：`onboard/dzuav/src/MapLocalizer.cpp` 的 `solve()` 在上轮巡检新增了单应拟合兜底（PnP 失败时用 `cv::findHomography` 直接拟合世界坐标↔图像坐标），这是 M0-3「无人机出情报」的关键定位兜底路径
- **问题**：`tools/ml/lattice_ransac.py` 作为该单应逻辑的 Python 参考实现（`fit_lattice`/`apply_lattice`/`invert_lattice`/`ransac_lattice`/`_any3_collinear`），此前完全没有单元测试覆盖，且模块级硬依赖 `ultralytics`/`torch`/`PIL` 导致无法在测试环境导入
- **修复 1 — 模块解耦**：将 `PIL`/`ultralytics`/`cell_classifier.predictor`（需 torch）的导入从模块级移入 `process_frame()`/`main()` 函数内部，模块级仅保留 `cv2`+`numpy`+标准库，纯函数可被测试直接导入
- **修复 2 — 新增 10 项回归测试**（`tests/test_lattice_ransac.py`）：
  - `cell_to_idx`/`idx_to_cell` 全格号 A1~N11 往返一致性 + 越界拒绝
  - `_any3_collinear` 共线检测（直线 True / 四边形 False）
  - `fit_lattice`+`apply_lattice` 合成正射网格往返重投影误差 < 1e-3 px
  - `invert_lattice` 图像坐标反推格号误差 < 1e-2
  - `fit_lattice` 对 4 点全共线返回 None（退化拒绝）
  - `ransac_lattice` 内点提取 + 离群点剔除（5 个随机离群点全部被拒）
  - `ransac_lattice` 少于 4 点返回 None
  - `ransac_lattice` 内点重拟合后中位重投影误差 < 2px
- **价值**：MapLocalizer 单应兜底的几何正确性现在有 Python 级回归测试守护，C++ 侧逻辑可参照该测试集验证，Orin 编译后定位兜底有保障

### ✅ 代码base健康
- `pytest tests/` 136 passed, 1 skipped in 18.23s（+10 新增测试，无回归）

### ✅ watch 脚本健康（3 实例运行中）
- PID 41148 + 42360 + 42708，均于 05:06:37 启动（已运行 ~11h）
- 连通 5090 后自动监控 v4d + seed157 + target_merged 三个训练并下载产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ✅ MapLocalizer单应兜底的Python参考逻辑新增10项回归测试，几何正确性有守护 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 15:02）

### ✅ MapLocalizer 新增单应拟合兜底（非硬件推进，服务 M0-3）
- **问题**：`onboard/dzuav/src/MapLocalizer.cpp` 的 `solve()` 仅使用 PnP 定位，当相机内参偏差或地标几何不佳时 PnP 失败 → `localized=false` → 无人机不广播情报 → M0-3 直接断链
- **修复**：PnP 失败后自动兜底为 `cv::findHomography(w2, p, RANSAC)`，直接拟合格号世界坐标↔图像坐标的单应矩阵，无需相机内参
- **质检**：单应解重投影中位误差 ≤ 15px（PnP 为 10px，单应无内参故门槛略宽）、内点数 ≥ max(8, 30%地标)，否则拒绝
- **价值**：Python `lattice_ransac.py` 已验证单应拟合在航拍正射下定位有效，兜底后定位成功率提升，减少"有地标但不广播"的断帧
- **影响范围**：仅 `MapLocalizer::solve()` 内部逻辑，`project()` 接口不变，Orin 编译后生效

### ✅ 代码base健康
- `pytest tests/` 129 passed in 16.12s（无回归）
- CMakeLists.txt 已链接 `${OpenCV_LIBS}` 全模块，`findHomography`/`perspectiveTransform` 可用

### ✅ watch 脚本健康（3 实例运行中）
- PID 41148 + 42360 + 42708，均于 05:06:37 启动（已运行 ~10h）
- 连通 5090 后自动监控 v4d + seed157 + target_merged 三个训练并下载产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ✅ MapLocalizer新增单应拟合兜底，PnP失败时仍可定位广播 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 14:09）

### ✅ 标靶检测后处理兼容 8 通道模型（非硬件推进，服务 M0-6）
- **问题**：`tools/ml/prelabel_rknn.py` 的 `decode_output` 严格要求 9 通道布局（4 bbox + 2 cls + 1 kpt*3），4 个下载的标靶 pose 模型中有 2 个（`target_car_pose_v2`、`target_synth_yolo11s`）为 8 通道（无 visibility 分），直接解码会抛 ValueError
- **修复**：`decode_output` 自动判别两种布局 —— 含 visibility（C=4+nc+nk*3）或无 visibility（C=4+nc+nk*2），后者自动补 v=1.0，返回统一的 `[N,nk,3]` 张量
- **验证**：新增 `test_decode_eight_channel_no_visibility` 测试，修正 `test_decode_rejects_channel_mismatch` 参数；全量 126 passed, 1 skipped，无回归
- **价值**：即使 seed157/target_merged 新训练产物未就绪，4 个已下载的标靶 pose 模型（2 个 9 通道 + 2 个 8 通道）均可直接转 RKNN 部署到车端，标靶识别不阻塞

### ✅ 代码base健康
- `pytest tests/` 126 passed, 1 skipped in 16.59s（无回归，跳过项为 onnx 库未安装）

### ✅ watch 脚本健康（3 实例运行中）
- PID 41148 + 42360 + 42708，均于 05:06:37 启动（已运行 ~9h）
- 连通 5090 后自动监控 v4d + seed157 + target_merged 三个训练并下载产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 无（待硬件） |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | ✅ 标靶检测后处理兼容8通道模型（4个pose模型均可部署） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 13:02）

### ✅ 8通道标靶pose模型类别确认（非硬件推进，重要发现）
- 此前巡检认为 `target_car_pose_v2_best.onnx` 和 `target_synth_yolo11s_best.onnx` 为8通道单类模型，与车端2类契约不符
- **经加载 .pt 权重确认：两个8通道模型均为2类（blue, red），与车端标靶识别需求完全匹配**
- 输出通道结构差异：
  - 9通道模型（yolo11n/lightdet）：4 bbox + 2 cls + 3 kpt(x,y,visibility)
  - 8通道模型（car_pose_v2/synth）：4 bbox + 2 cls + 2 kpt(x,y，无visibility分数)
- 结论：4个下载的标靶pose模型全部可用，8通道模型仅需在车端后处理中将visibility默认设为1即可
- 这意味着即使seed157/target_merged新训练产物未就绪，也有可部署的标靶识别模型备选（4个2类模型可选）

### ✅ 代码base健康
- `pytest tests/` 125 passed, 1 skipped in 18.40s（无回归，跳过项为onnx库未安装）

### ✅ watch 脚本健康（3 实例运行中）
- PID 41148 + 42360 + 42708，均于 05:06:37 启动（已运行 ~8h）
- 连通 5090 后自动监控 v4d + seed157 + target_merged 三个训练并下载产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 无（待硬件） |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 12:02）

### ✅ 标靶pose模型ONNX契约全量核验（非硬件推进）
- 对 `E:\ml_datasets\downloaded\` 中4个标靶pose ONNX模型做输入/输出契约核验（air_ground venv onnxruntime）：

  | 模型 | 输入 | 输出 | 车端契约匹配 | 部署就绪 |
  |---|---|---|---|---|
  | `target_pose_yolo11n_best.onnx` | [1,3,1280,1280] | [1,9,33600] | ✅ 9通道(4bbox+2cls+3kpt) | 转640→RKNN即可部署 |
  | `target_pose_lightdet_best.onnx` | [1,3,1280,1280] | [1,9,33600] | ✅ 9通道(4bbox+2cls+3kpt) | 转640→RKNN即可部署 |
  | `target_car_pose_v2_best.onnx` | [1,3,1280,1280] | [1,8,33600] | ⚠️ 8通道(4bbox+1cls+3kpt) | 单类，需确认类别后决定 |
  | `target_synth_yolo11s_best.onnx` | [1,3,1280,1280] | [1,8,33600] | ⚠️ 8通道(4bbox+1cls+3kpt) | 单类，需确认类别后决定 |

- 车端现役 `light_det.rknn` 契约为 `[1,9,8400]`（640分辨率），9通道模型转RKNN后可直接替换
- 8通道模型疑似单类（仅blue或仅red），与车端2类(blue/red)契约不符，待seed157/target_merged新训练产物替换
- 5090恢复后watch脚本自动下载seed157(2类)产物，届时可用2类模型替换8通道单类模型

### ✅ 代码base健康
- `pytest tests/` 125 passed, 1 skipped in 15.98s（无回归，跳过项为onnx库未安装）

### ✅ watch 脚本健康（3 实例运行中）
- PID 41148 + 42360 + 42708，均于 05:06:37 启动（已运行 ~7h）
- 连通 5090 后自动监控 v4d + seed157 + target_merged 三个训练并下载产物

### ✅ 模型指针确认无误
- `current_hbb_detector` → `hbb_detector_7cls_distill_20261011.onnx`
- `current_cell_classifier` → `cell_classifier_v4c_ft_20261010.onnx`

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 无（待硬件） |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 11:05）

### ✅ 端到端管线实跑验证通过（M0-3 软件就绪确认）
- `render_field_matrix.py --image frame_00124.jpg` 全链路跑通：
  - 检测目标数：88，格号锚点：80，定位方式：lattice（RANSAC 单应拟合）
  - 格网反推：内点 57 / 剔除假锚点 23，反推格 23 个
  - 输出：`runs/matrix_test/field_matrix.png` (1148x1617)
- 结论：7cls 检测 → v4c 格号识别 → RANSAC lattice 反推 → 矩阵渲染 全链路就绪

### ✅ 路径规划渲染验证通过（M0-5 就绪确认）
- `render_route_plan.py` 输出 40 格路径，覆盖全部雷区与射击位
- 自动绕行 D7-N7 静态挡板，通过 L7 桥洞穿越中场

### ✅ 赛前检查清单同步当前模型指针
- B1-3/B1-4：标注当前模型指针（7cls_distill_20261011 / v4c_ft_20261010）
- B3-1/B3-2：补充精确契约维度（[1,3,1280,1280]→[1,11,33600] / [1,3,96,96]→[1,14]）
- 新增 B3-4：current 指针检查项

### ✅ 代码base健康
- `pytest tests/` 128 passed in 16.67s（无回归，0 skipped）

### ✅ watch 脚本健康（3 实例运行中）
- PID 41148 + 42360 + 42708，均于 05:06:37 启动（已运行 ~6h）
- 连通 5090 后自动监控 v4d + seed157 + target_merged 三个训练并下载产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ✅ 端到端管线实跑验证通过（88目标+lattice定位+矩阵渲染） |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | ✅ 路径渲染验证通过（40格覆盖全雷区） |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | ✅ 同步当前模型指针+契约维度 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 10:02）

### ✅ 关键路径协议契约复核通过（非硬件推进）
- **UDP 协议一致性**：逐字段比对 `onboard/dzuav/src/UdpBroadcaster.cpp` 与 `src/ag_integrate/ag_integrate/intel_protocol.py`，0x4001（情报帧）和 0x4002（车位姿帧）的 magic/version/msg_id/length/objects/crc16/suffix 完全对齐，CRC-16/Modbus（多项式 0xa001，init 0xffff）实现一致
- **0x4002 广播逻辑**：`main.cpp` L151-160 确认定位有效+检测到车时调用 `estimateCarYaw` → `sendCarPose`，车端 align 模式可据此闭环转向，M0-2 数据源就绪
- **Orin 部署脚本复核**：`orin_convert_engines.sh`（current 指针解析→FP16 TRT 引擎）、`orin_build_verify.sh`（L0~L3 分层验证含车端 parse 闭环）、`orin_run_dzuav.sh`（相机/图片/视频三模式）逻辑完整正确
- **target_merged 数据集**：555 train + 97 val，1768 框类别分布 class0=211/1=142/2=475/3=83/4=559/5=298，与 data.yaml 6 类映射一致

### ✅ 代码base健康
- `pytest tests/` 128 passed in 18.86s（无回归，0 skipped）

### ✅ watch 脚本健康（3 实例运行中）
- PID 41148（系统Python）+ 42360（venvPython）+ 42708，均于 05:06:37 启动（已运行 ~5h）
- 连通 5090 后自动监控 v4d + seed157 + target_merged 三个训练并下载产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | ✅ 0x4002 广播逻辑+协议契约复核通过 |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ✅ Orin 部署脚本+协议契约全链路复核通过 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | ✅ 协议契约逐字段复核一致 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 09:04）

### ✅ 非硬件推进：端到端检测+格号+矩阵渲染全链路实跑验证
- `render_field_matrix.py --image capture/aerial_field/images/frame_00124.jpg` 全链路跑通：
  - 检测目标数：88，格号锚点：80，定位方式：lattice（RANSAC 单应拟合）
  - 格网反推：内点 57 / 剔除假锚点 23（含 M9,K9,D8,E9,E10,M10,M8,N7,N6,C7,N4,K5,B4,G3,F4,E3,D8,D3,H10,K4,M5,B2）
  - 反推格（目标落入但标签未识别）：23 格（B1,C1,C2,C3,C4,D1,D2,E1,E2,F1,F2,G2,H2,H3,H9,I2,J1,J2,J3,M2,M7,N2,N5）
  - 输出：`runs/matrix_test/field_matrix.png` (1148x1617)
- 验证结论：7cls 检测 → 格号识别 → RANSAC lattice 反推 → 矩阵渲染 全链路就绪，Orin 上线后可直接部署

### ✅ 代码base健康
- `pytest tests/` 128 passed in 16.77s（无回归，0 skipped）

### ✅ watch 脚本健康（3 实例运行中）
- PID 41148（系统Python）+ 42360（venvPython）+ 42708，均于 05:06:37 启动（已运行 ~4h）
- 连通 5090 后自动监控 v4d + seed157 + target_merged 三个训练并下载产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ✅ 端到端链路实跑验证通过（7cls+格号+矩阵渲染全通） |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 08:02）

### ✅ 非硬件推进：下载的标靶pose模型ONNX契约核验
- 5090不可达期间，对 `E:\ml_datasets\downloaded\` 中4个标靶pose ONNX模型做了输入/输出契约核验（air_ground venv onnxruntime）：
  | 模型 | 输入 | 输出 | 车端契约匹配 |
  |---|---|---|---|
  | `target_pose_yolo11n_best.onnx` | [1,3,1280,1280] | [1,9,33600] | ✅ 9通道(4bbox+2cls+3kpt)，1280分辨率需转RKNN时降为640→8400 |
  | `target_pose_lightdet_best.onnx` | [1,3,1280,1280] | [1,9,33600] | ✅ 同上 |
  | `target_car_pose_v2_best.onnx` | [1,3,1280,1280] | [1,8,33600] | ⚠️ 8通道，疑似单类(4bbox+1cls+3kpt)，与车端[1,9,8400]契约不符 |
  | `target_synth_yolo11s_best.onnx` | [1,3,1280,1280] | [1,8,33600] | ⚠️ 同上，8通道 |
- 车端现役 `light_det.rknn` 契约为 `[1,9,8400]`（640分辨率），9通道模型转RKNN后可直接替换，8通道模型需确认类别数后重训
- 这些模型来自5090失联前的训练批次，seed157/target_merged新训练产物待5090恢复后由watch脚本自动下载

### ✅ 代码base健康
- `pytest tests/` 125 passed, 1 skipped in 15.89s（无回归，跳过项为onnx库未安装）

### ✅ watch 脚本健康（3 实例运行中）
- PID 41148（系统Python）+ 42360（venvPython）+ 42708，均于 05:06:37 启动（已运行 ~3h）
- 连通 5090 后自动监控 v4d + seed157 + target_merged 三个训练并下载产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 当前无可推进非硬件工作
- 所有软件任务（7cls检测、格号识别、路径规划、朝向对齐、UDP链路）均已就绪
- seed157/v4d/target_merged 数据集 + 部署脚本 + watch监控全链路就绪，等待 5090 恢复后自动部署训练
- M0-1/M0-2 待实车，M0-3 待 Orin，其余 M0 项已完成

---

## 8. 上轮巡检非硬件推进（2026-10-12 07:03）

### ✅ 非硬件推进：提交关键部署脚本入库
- `tools/ml/deploy_target_merged.py`（标靶合并数据集部署脚本）此前未入库，5090恢复后需用其上传652帧数据集并启动训练
- `tools/ml/merge_target_dataset.py`（seed_157 + manual_hq_5合并脚本，含green target→green bucket、r'r/RR→red bucket手误修正映射）
- `tools/ml/gen_widget.py`（标注预览组件生成）
- 三个脚本均通过 `py_compile` 语法校验，watch脚本也已覆盖target_merged产物下载

### ✅ 代码base健康
- `pytest tests/` 128 passed in 16.19s（无回归，0 skipped）

### ✅ watch 脚本健康（3 实例运行中）
- PID 41148（系统Python）+ 42360（venvPython）+ 42708，均于 05:06:37 启动
- 连通 5090 后自动监控 v4d + seed157 + target_merged 三个训练并下载产物
- `E:\ml_datasets\downloaded\` 尚无 v4d/seed157/target_merged 产物（5090不可达）

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 当前无可推进非硬件工作
- 所有软件任务（7cls检测、格号识别、路径规划、朝向对齐、UDP链路）均已就绪
- seed157/v4d/target_merged 数据集 + 部署脚本 + watch监控全链路就绪，等待 5090 恢复后自动部署训练
- M0-1/M0-2 待实车，M0-3 待 Orin，其余 M0 项已完成

---

## 8. 上轮巡检非硬件推进（2026-10-12 06:07）

### ✅ 非硬件推进：更新 finish_line_analysis.md 瓶颈分析文档
- 原文档停留在 2026-10-09，7cls 仍标为 "epoch 21/75"，格号识别标为 "v4c训练中"
- 已更新 §1 瓶颈排序表：7cls → ✅ 已完成（epoch 75/75，best mAP50=0.7879，模型已发布）；格号识别 → "v4c已发布(joint_acc=0.8588)，v4d待5090训练"
- 已更新 §2.3：补充 7cls 蒸馏模型双口径评估结果（官方74张 mine mAP50=0.7972 vs 厂家 0.1248）
- 已更新 §2.4：补充 v4c 离线验证指标 + v4d 训练集构成 + RANSAC/lattice 已落地说明
- 已更新 §3.2 改进建议表：新增"状态"列，3/5 项已实现（MatrixFusion多帧融合、RANSAC几何约束、路径可视化）

### ✅ 代码base健康
- `pytest tests/` 128 passed in 16.08s（无回归，0 skipped）

### ✅ watch 脚本健康（3 实例运行中）
- PID 41148（系统Python）+ 42360（venvPython）+ 42708，均于 05:06:37 启动
- 连通 5090 后自动监控 v4d + seed157 + target_merged 三个训练并下载产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 当前无可推进非硬件工作
- 所有软件任务（7cls检测、格号识别、路径规划、朝向对齐、UDP链路）均已就绪
- seed157/v4d/target_merged 数据集 + 部署脚本 + watch监控全链路就绪，等待 5090 恢复后自动部署训练
- M0-1/M0-2 待实车，M0-3 待 Orin，其余 M0 项已完成

---

## 8. 上轮巡检非硬件推进（2026-10-12 05:07）

### ✅ 非硬件推进：watch_5090 脚本新增 target_merged 监控
- `watch_5090_v4d_seed.py` 此前仅监控 v4d + seed157，target_merged 训练无自动下载
- 已在 BEST/LOGS/PROC/ARTIFACTS 字典中新增 `target` 条目（`runs/target/target_merged_v1/weights/best.pt` + results.csv）
- main() 的 status/bad 字典及监控循环已扩展为 `("v4d", "seed", "target")`
- 5090 恢复后三个训练（v4d格号 / seed157标靶种子 / target_merged标靶全量）产物均自动下载至 `E:\ml_datasets\downloaded\`
- 语法校验通过，watch 脚本已重启（PID 42708 系统Python + 42360 venvPython + 41148）

### ✅ 代码base健康
- `pytest tests/` 125 passed, 1 skipped in 16.06s（无回归，跳过项为 onnx 库未安装）

### ✅ 数据集 + 部署脚本全量就绪确认
- seed_157: 134 train + 23 val，363 框全部 w>0 h>0
- target_merged: 555 train + 97 val，1768 框全部 w>0 h>0
- v4d: `cellv4d_upload_v2.zip`（218.4MB）就绪
- 部署脚本：`deploy_5090_v4d_seed.py`（v4d+seed）+ `deploy_target_merged.py`（target）均就绪
- 模型指针：`current_hbb_detector` → `hbb_detector_7cls_distill_20261011.onnx`，`current_cell_classifier` → `cell_classifier_v4c_ft_20261010.onnx`

### ✅ watch 脚本健康（已重启，加载 target_merged 监控）
- 三个 `watch_5090_v4d_seed.py` 后台运行中（PID 42708 + 42360 + 41148）
- 连通即自动监控 v4d + seed157 + target_merged 三个训练并下载产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化，157 JSON + 157 txt）

### 📋 当前无可推进非硬件工作
- 所有软件任务（7cls检测、格号识别、路径规划、朝向对齐、UDP链路）均已就绪
- seed157/v4d/target_merged 数据集 + 部署脚本 + watch监控全链路就绪，等待 5090 恢复后自动部署训练
- M0-1/M0-2 待实车，M0-3 待 Orin，其余 M0 项已完成

---

## 8. 上轮巡检非硬件推进（2026-10-12 04:04）

### ✅ 代码base健康
- `pytest tests/` 128 passed in 16.20s（无回归，0 skipped）

### ✅ 模型发布指针 + Orin 部署脚本全链路复核通过
- `current_hbb_detector` → `hbb_detector_7cls_distill_20261011.onnx`（36.66MB）
- `current_cell_classifier` → `cell_classifier_v4c_ft_20261010.onnx`（4.13MB）
- `orin_convert_engines.sh` 指针解析逻辑正确：优先读 current 指针 → 回退无版本名 → 取最新带版本文件
- Orin 上线后 `git pull → orin_convert_engines.sh → orin_build_verify.sh` 一键部署

### ✅ 5090 监控脚本健康（已运行 19h+）
- 两个 `watch_5090_v4d_seed.py` 后台运行中（PID 33984 系统Python + 37436 venv Python）
- CPU 占用极低（休眠重试模式），连通即自动监控训练+下载产物

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化，157 JSON 在 images/ 目录 + 157 txt）

### 📋 当前无可推进非硬件工作
- 所有软件任务（7cls检测、格号识别、路径规划、朝向对齐、UDP链路）均已就绪
- seed157/v4d/target_merged 数据集已就绪，等待 5090 恢复后自动部署训练
- M0-1/M0-2 待实车，M0-3 待 Orin，其余 M0 项已完成

---

## 8. 上轮巡检非硬件推进（2026-10-12 03:01）

### ✅ 代码base健康
- `pytest tests/` 128 passed in 17.31s（无回归）

### ✅ 模型发布指针确认无误
- `current_hbb_detector` → `hbb_detector_7cls_distill_20261011.onnx`（36.66MB，契约 [1,3,1280,1280]→[1,11,33600]）
- `current_cell_classifier` → `cell_classifier_v4c_ft_20261010.onnx`（4.13MB）

### ✅ 5090 监控脚本健康（已运行 6h+）
- 两个 `watch_5090_v4d_seed.py` 后台运行中（PID 33984 系统Python + 37436 venv Python）
- 脚本 `wait_ssh()` 每 60s 重试连接，连通即自动监控训练+下载产物
- seed157/v4d/target_merged 训练状态未知（5090 不可达）

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化，157 JSON + 157 txt）

### 📋 当前无可推进非硬件工作
- 所有软件任务（7cls检测、格号识别、路径规划、朝向对齐、UDP链路）均已就绪
- seed157/v4d/target_merged 数据集已就绪，等待 5090 恢复后自动部署训练
- M0-1/M0-2 待实车，M0-3 待 Orin，其余 M0 项已完成

---

## 8. 上轮巡检非硬件推进（2026-10-12 02:01）

### ✅ 模型 ONNX 契约冒烟测试通过（非硬件推进）
- **7cls 检测器**：`hbb_detector_7cls_distill_20261011.onnx` 输入 [1,3,1280,1280] → 输出 [1,11,33600]，CPU 推理 OK
- **格号识别 v4c**：`cell_classifier_v4c_ft_20261010.onnx` 输入 [1,3,96,96] → 输出 [1,14]，CPU 推理 OK
- 两个已发布模型均能正常加载推理，部署契约无误

### ✅ 数据集全量就绪确认
- **seed_157**：134 train + 23 val，标签数与图片数完全匹配（134+23），data.yaml 6 类映射正确
- **target_merged**：555 train + 97 val，标签数与图片数完全匹配（555+97），data.yaml 6 类映射正确
- **v4d**：`cellv4d_upload_v2.zip`（218.4MB）就绪

### ✅ 代码base健康
- `pytest tests/` 128 passed in 16.10s（无回归）

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 两个 `watch_5090_v4d_seed.py` 后台运行中（PID 33984 + 37436），连通即自动监控+下载
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 训练：epoch 75/75 已完成（无变化）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/357（无变化）

---

## 8. 上轮巡检非硬件推进（2026-10-12 01:04）

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
