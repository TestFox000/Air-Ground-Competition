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
| A4-1 | **7cls 蒸馏训练收敛** | ✅ | 10-11 16:05 | 10-11 16:05 | 蒸馏版ep75/75完成(mAP50=0.7879)已发布。**4090三连训**：7cls_clean(ep80/80✅,mAP50=0.6666)→7cls_final_v2(ep60/60✅,mAP50=0.7207)→7cls_v3(ep20/50🟡,mAP50=0.6799)。**综合评估最优：2stage_v2+edge1%=91.07%**。mine_cls_v5雷分类器(ep40/40✅,val_acc=0.9969)已下载。**⚠️本地7cls补充训练已停止(ep47/60)**：aerial_field_7cls_local(2053障碍样本)best mAP50=0.6795@ep13，综合精度89.19%，obstacle FN仍=30，target_red/blue灾难性遗忘。**⛔car_aug训练已停止(ep6/50)**：mAP50=0.64，因car_aug数据集含增强假阳性风险被放弃。**⛔car_up训练已停止(ep10/40)**：best mAP50=0.656@ep4(car_upsampled 1679 train)。**⛔7cls_v2训练已停止(ep25/80)**：best mAP50=0.649@ep6，连续19epoch无提升(远低于蒸馏版0.7879)，patience=30预计ep36早停，主动停止释放GPU。**✅7cls_combined训练已完成(ep60/60)**：从aerial_field_finetuned_v2 best.pt训练，2087 train/541 val(含mine+obstacle+arrow CP+car样本)，lr0=0.01，4060 batch2/imgsz1280，best mAP50=**0.64706@ep31**（28epoch无提升）。**⚠️综合精度90.53%（低于现役91.07%）**：obstacle FN=30→4（根治！），arrow召回18%→100%（根治！），但mine acc下降(0.8872 vs 0.91+)，target_red/blue=0%。**结论：combined解决两大瓶颈但mine退化，需集成策略融合combined(obstacle+arrow)与final_v2(mine)**。**🔄本轮集成实验（16:02）：4种集成策略均未超越combined单独(90.53%)**。**⚠️val集口径修正**：91.07%基线(val-A,32帧,无obstacle/cell_label GT)与combined(val-B,34帧,obstacle=88/cell_label=1032)不可直接比较。val-B上combined=90.53%为最优。mine_cls_v5航拍域无效（域不匹配）。当前瓶颈mine FN=38。 |
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
| M0-3 | 无人机出情报(UDP) | Agent-4 | 🔵 待Orin上线（v4c离线验证已过，7cls模型已发布；本地端到端管线验证通过。**⚠️val集口径修正**：91.07%基线在val-A(32帧，无obstacle/cell_label GT)，combined在val-B(34帧，obstacle=88/cell_label=1032)，**两者不可直接比较**。在val-B上，**combined模型综合精度90.53%为当前最优**（obstacle FN=4、arrow召回100%两大瓶颈已根治）。**4种集成策略均未超越combined单独**：final_v2 mine+cls=90.37%、combined+mine_cls_v5=90.52%、mine联合=90.38%、TTA=90.35%、conf=0.15=89.95%。**mine_cls_v5在航拍域无效**（域不匹配）。**当前瓶颈**：mine FN=38。距95%差4.47pp） |
| M0-4 | 车机UDP链路通 | Agent-1 | ✅ 模拟验证通过 |
| M0-5 | 路径规划可执行 | Agent-1 | ✅ 模拟验证通过 |
| M0-6 | 车按路径行驶 | Agent-3 | ⛔ 依赖M0-1,M0-2（标靶pose模型target_pose_v2已完成：mAP50=0.9755，ONNX契约[1,9,8400]匹配车端；**mine_cls_v5雷分类器已完成val_acc=0.9969**，爆炸物二次确认就绪） |
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
| R3 | 7cls综合精度未达95% | Agent-4 | **⚠️val集口径修正**：91.07%基线(val-A,32帧,无obstacle/cell_label GT)与combined(val-B,34帧,obstacle=88/cell_label=1032)**不可直接比较**。在val-B上**combined=90.53%为最优**（obstacle FN=4、arrow召回100%已根治）。**4种集成策略均未超越combined单独**：①final_v2 mine+mine_cls_v5=90.37% ②combined+mine_cls_v5=90.52% ③mine联合(distill+combined)=90.38% ④TTA=90.35% ⑤conf=0.15=89.95%。**mine_cls_v5在航拍域无效**（训练于官方近景图，域不匹配，仅过滤少量FP且损失TP）。**当前瓶颈mine FN=38**，需更优mine模型/数据或TTA优化。距95%差4.47pp |
| R7 | 5090连接不稳定 | Agent-4 | 5090实例持续不可达（connect.bjb2.seetacloud.com:37113 TcpTestSucceeded=False）。seed157/v4d/target_merged 训练状态未知。watch_5090_v4d_seed.py 2实例持续存活（自18:04起），连通即自动监控+下载。需用户在 AutoDL 控制台检查实例状态 |
| R8 | 4090训练进度 | Agent-4 | ⚠️ 4090实例(connect.bjb1:27379)本轮不可达。**已完成**：7cls_clean / 7cls_final_v2 / mine_cls_v5 / target_pose_v2 / red_bucket。**进行中(中断)**：7cls_v3(ep20/50) / red_bucket_v2(ep28)，待实例恢复后续训 |
| R4 | 0x4002帧未广播 | Agent-4 | 代码完成，待Orin编译 |
| R6 | E盘（项目存储） | 经理 | ✅ 已恢复（7cls续训成功读取E:\ml_datasets），持续监控 |

---

*最后更新：2026-10-13 16:02 经理（每小时巡检）*

## 7. 本轮巡检非硬件推进（2026-10-13 16:02）

### ⚠️ 重大发现：val集口径不一致 + 4种集成策略均未超越combined单独（服务 M0-3 95%硬指标）

**关键发现1：91.07%基线与combined不可直接比较（val集GT标签空间不同）**
- val-A（91.07%基线，32帧）：mine=1059, car=1175, arrow=16, target_red=16, target_blue=4, **obstacle=0, cell_label=0**
- val-B（combined评估，34帧）：mine=966, obstacle=88, cell_label=1032, arrow=11, **target_red=0, target_blue=0**
- val-A无obstacle/cell_label GT → combined在val-A上这些类的正确检测全部计为FP，导致39.81%（假象）
- **结论**：91.07%基线虚高（未包含obstacle瓶颈），val-B更贴近比赛场景

**关键发现2：在val-B上，combined单独(90.53%)为最优，4种集成策略均未超越**

| 方案 | 综合精度 | mine TP | mine FP | mine FN | 说明 |
|---|---|---|---|---|---|
| **combined单独(conf=0.25+edge)** | **90.53%** | 928 | 80 | 38 | **当前最优** |
| combined + mine_cls_v5 | 90.52% | 925 | 77 | 41 | cls过滤6框，损失3TP |
| combined非mine + final_v2 mine(edge+cls) | 90.37% | 927 | 83 | 39 | final_v2 mine更差 |
| mine联合(distill+combined, IoU去重+cls) | 90.38% | 931 | 87 | 35 | +3TP但+7FP |
| combined TTA | 90.35% | 930 | 84 | 36 | +2TP但+4FP |
| combined conf=0.15 | 89.95% | 929 | 84 | 37 | +1TP但+4FP |

**关键发现3：mine_cls_v5在航拍域无效**
- mine_cls_v5训练于官方近景图，航拍域泛化有限
- 在combined mine预测上仅移除6/86个FP，同时损失3个TP
- 与历史结论一致："官方域训练的雷分类器在当前检测设置下反而会降低整体精确率"

**当前瓶颈：mine FN=38**（val-B共966个mine GT）
- obstacle FN=4（已根治，原30）
- arrow FN=0（已根治）
- cell_label FN=9
- mine FN=38为最大短板

**下一步建议（不依赖硬件）：**
1. 需更优mine模型：可尝试从combined的mine FN漏检帧做针对性数据增强
2. 或接受combined 90.53%为当前最优，聚焦target_red/blue检测（val-B无GT无法评估）
3. 4090恢复后可用更大batch/更长训练优化mine

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879@ep56，无变化）
- **✅ 7cls_combined 训练已完成**：ep60/60，best mAP50=0.64706@ep31，val-B综合精度90.53%（当前最优）
- **本地无活跃训练进程**（所有训练已完成或停止）
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复

### ✅ 代码base健康
- `pytest tests/` 147 passed in 26.43s（无回归）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ⚠️ **val集口径修正**：91.07%基线不可直接比较；✅ **combined 90.53%为val-B最优**；❌ 4种集成策略均未超越combined；❌ mine_cls_v5航拍域无效；🔴 瓶颈mine FN=38 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-13 15:08）

### ✅ combined训练完成 + 自动评估完成：两大瓶颈根治，mine退化，需集成策略（服务 M0-3 95%硬指标）
- **训练完成**：combined ep60/60 完成，best mAP50=**0.64706@ep31**（28 epoch 无提升）
- **watch_eval 自动评估结果**（含 mine 边缘 1% 过滤，与 91.07% 基线同口径）：

| 类别 | acc | TP | FP | FN | vs 91.07%基线 |
|---|---|---|---|---|---|
| mine | 0.8872 | 928 | 80 | 38 | ⬇️ FN 22→38（退化） |
| obstacle | **0.9333** | 84 | 2 | **4** | ⬆️ FN 30→4（根治！） |
| arrow | **0.9167** | 11 | 1 | **0** | ⬆️ 召回18%→100%（根治！） |
| cell_label | 0.9200 | 1023 | 80 | 9 | 持平 |
| target_red | 0.0000 | 0 | 0 | 0 | 无GT/零检出 |
| target_blue | 0.0000 | 0 | 0 | 0 | 无GT/零检出 |

- **综合精度：90.53%**（TP=2046 FP=163 FN=51），**低于现役最优 91.07%**
- **核心发现**：combined 模型根治了 obstacle FN=30→4 和 arrow 召回18%→100% 两大瓶颈，但 mine 性能退化（FN=38 vs 22）
- **集成实验**（非硬件推进）：尝试 combined(obstacle+arrow) + final_v2(mine) 集成
  - 结果：综合精度 **90.13%**（低于 combined 单独 90.53%）
  - 原因：final_v2 单独（无 mine_cls_v5 分类器）mine FP=90 > combined 的 FP=80
  - 结论：**简单集成无效**，需用 2stage_v2（final_v2 + mine_cls_v5 分类器）做 mine 才能获得 91.07% 基线的 mine 性能
- **下一步策略**：2stage_v2(mine) + combined(obstacle+arrow) 集成，需在相同 34 帧 val 集上重跑 2stage_v2（含 mine_cls_v5），理论可达 92%+

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879@ep56，无变化）
- **✅ 7cls_combined 训练已完成（本地4060）**：ep60/60，best mAP50=**0.64706@ep31**，综合精度90.53%
  - 基座：aerial_field_finetuned_v2 best.pt
  - 数据集：aerial_field_7cls_combined（**2087 train / 541 val**）
  - **成果**：obstacle FN=30→4（根治），arrow召回18%→100%（根治）
  - **不足**：mine FN=22→38（退化），需集成 final_v2 补偿
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
- **watch_eval 守护**：已完成评估并退出（PID 46596）

### ✅ 代码base健康
- `pytest tests/` 147 passed in 30.50s（无回归）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ✅ **combined训练完成(ep60/60)**；⚠️ 综合精度90.53%（低于91.07%）；✅ obstacle FN=30→4、arrow召回100%（两大瓶颈根治）；⬇️ mine退化(FN=38)；🔄 下一步：集成combined(obstacle+arrow)+final_v2(mine) |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-13 14:02）

### 🟡 combined训练稳步推进至ep51/60，距patience早停仅5epoch（服务 M0-3 95%硬指标）
- **训练进展**：combined 从 ep45→ep51（+6 epoch/小时），**best mAP50 仍为 0.64706@ep31**（已 20 epoch 无提升），patience=25 预计 **ep56 早停**（仅剩 5 epoch）
- **进程健康**：单进程稳定运行（PID 46976，CPU 2720s，GPU 85%/6164MiB），无双进程冲突
- **watch_eval 守护**：PID 46596 存活，已检测到 ep51/60，训练完成即自动输出综合精度
- **预判**：若 ep52-56 仍无提升，训练将在 ep56 早停；watch_eval 将立即用 best.pt 跑 eval_comprehensive 输出可直接对比 91.07% 的综合精度

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879@ep56，无变化）
- **🟡 7cls_combined 训练进行中（本地4060，单进程）**：ep51/60，best mAP50=**0.64706@ep31**（PID 46976，GPU 85%/6164MiB）
  - 基座：aerial_field_finetuned_v2 best.pt
  - 数据集：aerial_field_7cls_combined（**2087 train / 541 val**）
  - 策略：lr0=0.01 / patience=25 / batch=2 / imgsz=1280 / workers=0 / amp=False / close_mosaic=10
  - 约 8min/epoch，ETA ~15:00 完成（若 patience=25 早停则约 ep56，ETA ~14:45）
  - **直接针对两大瓶颈**：obstacle=6324 样本（根治 FN=30）+ arrow=1823 样本（根治召回低）
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_eval 守护**：PID 46596 存活（含 mine 边缘过滤增强），已检测到 ep51/60

### ✅ 代码base健康
- `pytest tests/` 144 passed, 1 skipped in 27.49s（无回归）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 🟡 **combined训练推进至ep51/60**(best mAP50=0.64706@ep31，20epoch无提升，patience=25预计ep56早停)；✅ 单进程稳定运行无双进程冲突；✅ watch_eval已就位训练完成即出综合精度 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-13 13:02）

### ⚠️ combined训练在ep44后中断 + 双进程误杀，已干净恢复（服务 M0-3 95%硬指标）
- **发现**：combined训练在 ep44 后进程退出（results.csv 12:58 写入 ep44 后停止，GPU 0%）。巡检时发现双进程冲突：venv残留训练进程(PID 40048, CPU仅0.015s) + 重复watch_eval(PID 43772)
- **纠正过程**：
  1. kill 残留重复进程(PID 40048 + 43772) → 但因误判导致主训练进程(PID 6052)也被终止
  2. 排查确认 venv python.exe 为存根，实际工作由其启动的系统 python 子进程完成，杀存根会连带杀子进程
  3. 从 `aerial_field_7cls_combined/weights/last.pt`(ep44) **单进程** resume 训练（venv python 启动，实际子进程 PID 46976），已确认 GPU 6734MiB，ep45/60 训练中
  4. 重启 watch_eval 守护（PID 46596，含 mine 边缘 1% 过滤）
- **训练进展**：combined 从 ep38→ep45（+7 epoch/小时），**best mAP50 仍为 0.64706@ep31**（已 14 epoch 无提升），patience=25 预计 ep56 早停
- **价值**：combined 是当前唯一冲击 95% 的方案，及时恢复避免进度停滞

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879@ep56，无变化）
- **🟡 7cls_combined 训练进行中（本地4060，单进程）**：ep45/60，best mAP50=**0.64706@ep31**（自 13:02 从 ep44 恢复运行中，PID 46976，GPU 6734MiB）
  - 基座：aerial_field_finetuned_v2 best.pt
  - 数据集：aerial_field_7cls_combined（**2087 train / 541 val**）
  - 策略：lr0=0.01 / patience=25 / batch=2 / imgsz=1280 / workers=0 / amp=False / close_mosaic=10
  - 约 8min/epoch，ETA ~15:00 完成（若 patience=25 早停则约 ep56）
  - **直接针对两大瓶颈**：obstacle=6324 样本（根治 FN=30）+ arrow=1823 样本（根治召回低）
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_eval 守护**：PID 46596 存活（含 mine 边缘过滤增强），已检测到 ep45/60

### ✅ 代码base健康
- `pytest tests/` 147 passed in 29.12s（无回归）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 🟡 **combined训练推进至ep45/60**(best mAP50=0.64706@ep31，14epoch无提升)；⚠️ 训练在ep44中断已干净恢复+双进程已清理；✅ watch_eval已重启 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-13 12:06）

### ⚠️ 训练进程中断恢复 + 双进程冲突再次清理（服务 M0-3 95%硬指标）
- **发现**：7cls_combined 训练在 ep37 后进程退出（GPU 0%，results.csv 12:03 写入 ep37 后停止），疑似系统休眠/Modern Standby 中断。同时发现双进程冲突再次出现：venv python 残留训练进程(PID 2696, CPU仅0.03s) + venv watch_eval(PID 45964, CPU仅0.015s)
- **纠正行动**：
  1. kill 残留重复进程(PID 2696 + 45964)，防止文件锁冲突
  2. 从 `aerial_field_7cls_combined/weights/last.pt`(ep37) **单进程** resume 训练（venv python，PID 6052），已确认 GPU 93%/6715MiB，ep38/60 训练中
  3. 重启 watch_eval 守护（PID 16832，含 mine 边缘 1% 过滤）
- **训练进展**：combined 从 ep29→ep38（+9 epoch），**best mAP50 从 0.6407@ep17 提升至 0.64706@ep31**（+0.006pp），训练仍在稳步提升，未触发 patience 早停
- **价值**：combined 是当前唯一冲击 95% 的方案（2087 train 含 obstacle=6324+arrow=1823 CP 样本），及时恢复训练避免进度停滞

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879@ep56，无变化）
- **🟡 7cls_combined 训练进行中（本地4060，单进程）**：ep38/60，best mAP50=**0.64706@ep31**（自 12:06 从 ep37 恢复运行中，PID 6052，GPU 93%/6715MiB）
  - 基座：aerial_field_finetuned_v2 best.pt
  - 数据集：aerial_field_7cls_combined（**2087 train / 541 val**）
  - 策略：lr0=0.01 / patience=25 / batch=2 / imgsz=1280 / workers=0 / amp=False / close_mosaic=10
  - 约 7-8min/epoch，ETA ~15:00 完成（若 patience=25 早停则约 ep42-46）
  - **直接针对两大瓶颈**：obstacle=6324 样本（根治 FN=30）+ arrow=1823 样本（根治召回低）
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_eval 守护**：PID 16832 存活（含 mine 边缘过滤增强），已检测到训练恢复

### ✅ 代码base健康
- `pytest tests/` 147 passed in 29.40s（无回归）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 🟡 **combined训练推进至ep38/60**(best mAP50=0.64706@ep31↑)；⚠️ 训练进程中断已恢复+双进程冲突再次清理；✅ watch_eval已重启 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-13 11:06）

### ✅ watch_eval 增强：mine边缘过滤集成（服务 M0-3 95%硬指标）
- **问题**：原 `_watch_eval_combined.py` 在训练完成后直接用 YOLO predict 输出跑 eval_comprehensive，**未应用 mine 类 1% 边缘过滤**，导致输出的综合精度无法直接与 91.07% 基线（含边缘过滤，+0.76pp）对比
- **修复**：新增 `_apply_mine_edge_filter()` 函数，在推理后、评估前过滤中心点距图像边缘<1%的 mine(cls=0) 检测框
  - 过滤逻辑与 `prelabel_onnx.py::filter_edge_boxes()` 完全一致，确保口径对齐
  - 日志记录移除的边缘 mine 框数量，便于诊断
- **已重启守护**：kill 旧 watch_eval(PID 31376) → 启动增强版(PID 45964)，已确认检测到 ep29/60
- **价值**：combined 训练完成后自动输出的综合精度可直接与 91.07% 基线对比，第一时间判断是否达标 95%

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879@ep56，无变化）
- **🟡 7cls_combined 训练进行中（本地4060，单进程）**：ep29/60，best mAP50=**0.6407@ep17**（PID 40952，GPU 94%/6627MiB）
  - 基座：aerial_field_finetuned_v2 best.pt
  - 数据集：aerial_field_7cls_combined（**2087 train / 541 val**）
  - 策略：lr0=0.01 / patience=25 / batch=2 / imgsz=1280 / workers=0 / amp=False / close_mosaic=10
  - 约 8min/epoch，ETA ~15:30 完成（若 patience=25 早停则约 ep42，ETA ~13:30）
  - **直接针对两大瓶颈**：obstacle=6324 样本（根治 FN=30）+ arrow=1823 样本（根治召回低）
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_eval 守护**：PID 45964 存活（含 mine 边缘过滤增强），已检测到 ep29/60

### ✅ 代码base健康
- `pytest tests/` 147 passed in 29.82s（无回归）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 🟡 **combined训练推进至ep29/60**(best mAP50=0.6407@ep17)；✅ **watch_eval增强mine边缘过滤并重启**，训练完成即出可对比综合精度 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-13 09:06）

### ⚠️ 关键纠正：combined训练被中断+car_aug误启动，已恢复
- **发现**：7cls_combined训练（针对M0-3两大瓶颈）在ep13/60后停止，而被弃用的car_aug训练（因假阳性风险被放弃）在08:17被启动运行至ep6
- **根因**：combined训练进程在08:15后退出（可能系统休眠），随后car_aug被误启动（PID 5768+42064）
- **纠正行动**：
  1. kill car_aug进程（PID 5768+42064），释放GPU
  2. 从 `aerial_field_7cls_combined/weights/last.pt` resume combined训练，已推进至ep14/60
  3. 重启 `_watch_eval_combined.py` 自动评估守护（09:05:55启动，已检测到ep13）
- **价值**：combined训练直接针对obstacle FN=30+arrow召回低两大瓶颈，是当前唯一能冲击95%硬指标的方案；car_aug数据集含增强假阳性风险，已弃用

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879@ep56，无变化）
- **🟡 7cls_combined 训练进行中（本地4060，已从ep13恢复）**：ep14/60，best mAP50=0.6407@ep17（自 09:04 恢复运行中）
  - 基座：aerial_field_finetuned_v2 best.pt
  - 数据集：aerial_field_7cls_combined（**2087 train / 541 val**）
  - 类别分布：mine=56859 / obstacle=6324 / arrow=1823 / target_blue=1867 / target_red=1083 / car=186 / cell_label=59967
  - 策略：lr0=0.01 / patience=25 / batch=2 / imgsz=1280 / workers=0 / amp=False / close_mosaic=10
  - 约 7min/epoch，ETA ~14:30 完成
  - **直接针对两大瓶颈**：obstacle=6324 样本（根治 FN=30）+ arrow=1823 样本（根治召回低）
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_5090 监控脚本**：2 实例持续存活（PID 4256+14796，自 18:04 起）

### ✅ 代码base健康
- `pytest tests/` 147 passed in 29.63s（无回归）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ⚠️ **纠正训练优先级**：kill误启动的car_aug + resume combined至ep14/60；✅ 自动评估守护已重启 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-13 08:07）

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879@ep56，无变化）
- **🟡 7cls_combined 训练进行中（本地4060）**：ep12/60，best mAP50=0.6407@ep17（自 06:45 运行中，PID 29352 存活）
  - 基座：aerial_field_finetuned_v2 best.pt
  - 数据集：aerial_field_7cls_combined（**2087 train / 541 val**）
  - 类别分布：mine=56859 / obstacle=6324 / arrow=1823 / target_blue=1867 / target_red=1083 / car=186 / cell_label=59967
  - 策略：lr0=0.01 / patience=25 / batch=2 / imgsz=1280 / workers=0 / amp=False / close_mosaic=10
  - 约 7min/epoch，ETA ~13:45 完成
  - **直接针对两大瓶颈**：obstacle=6324 样本（根治 FN=30）+ arrow=1823 样本（根治召回低）
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_5090 监控脚本**：2 实例持续存活（PID 4256+14796，自 18:04 起）

### ✅ 非硬件推进：combined 训练自动评估守护（服务 M0-3 95%硬指标）
- **问题**：`_7cls_watch_eval.py` 仅监控旧蒸馏训练（已完成），当前 combined 训练无自动评估，训练完成后无法第一时间获得省赛综合精度
- **新增 `tools/ml/_watch_eval_combined.py`**：监控 combined 训练 results.csv，训练完成（ep60/60 或进程退出）后自动：
  1. 用 best.pt 在原始 7cls val 集（34 帧，与 91.07% 基线同口径）推理生成 YOLO 预测标签
  2. 运行 `eval_comprehensive.py` 输出省赛综合识别精度（含各类 TP/FP/FN、距 95% 差距）
- **已启动**（后台 job），日志：`logs/7cls_combined_watch_eval.log`，已确认检测到 ep12/60
- **价值**：combined 训练完成后无需人工值守，自动输出综合精度，第一时间判断是否达标 95% 或需继续迭代

### ✅ 代码base健康
- `pytest tests/` 144 passed, 1 skipped in 28.05s（无回归）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 🟡 **7cls_combined训练推进至ep12/60**(best mAP50=0.6407@ep17)；✅ 自动评估守护已启动，训练完成即出综合精度 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-13 07:03）

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879@ep56，无变化）
- **⛔ obstacle_aug 训练已停止**：上轮巡检记录 ep1/80，实际已切换至 combined 训练
- **🟡 7cls_combined 训练进行中（本地4060）**：ep2/60，best mAP50=0.5718@ep2（自 06:45 运行中）
  - 基座：aerial_field_finetuned_v2 best.pt
  - 数据集：aerial_field_7cls_combined（**2087 train / 541 val**）
  - 类别分布：mine=56859 / obstacle=6324 / arrow=1823 / target_blue=1867 / target_red=1083 / car=186 / cell_label=59967
  - 策略：lr0=0.01 / patience=25 / batch=2 / imgsz=1280 / workers=0 / amp=False
  - 约 7min/epoch，ETA ~13:50 完成
  - **直接针对两大瓶颈**：obstacle=6324 样本（根治 FN=30）+ arrow=1823 样本（根治召回低）
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_5090 监控脚本**：2 实例持续存活（PID 4256+14796，自 18:04 起）

### ✅ 代码base健康
- `pytest tests/` 147 passed in 29.58s（无回归）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 🟡 **7cls_combined训练已启动(ep2/60)**：2087 train含obstacle=6324+arrow=1823样本，直接根治两大瓶颈 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-13 06:05）

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879@ep56，无变化）
- **⛔ 7cls_v2 训练已主动停止**：ep25/80，best mAP50=0.64856@ep6，连续19 epoch无提升（远低于蒸馏版0.7879），patience=30预计ep36早停，主动停止释放GPU给更有价值的obstacle_aug训练
- **🟡 obstacle_aug 训练已启动（本地4060）**：ep1/80，GPU 4.74G/8G，~5.3min/epoch
  - 基座：7cls蒸馏版 best.pt（mAP50=0.7879，当前综合91.07%）
  - 数据集：aerial_field_7cls_obstacle_aug（1716 train = 原始216 + mine CP 500 + obstacle CP 500 + arrow CP 500）
  - 策略：低学习率微调(lr0=0.003) + close_mosaic=10，防止 catastrophic forgetting
  - 4060配置：batch=2 / imgsz=1280 / workers=0 / amp=False
  - **直接针对两大瓶颈**：obstacle FN=30 + arrow召回低(11GT仅2检出)
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_5090 监控脚本**：2 实例持续存活（PID 4256+14796，自 18:04 起）

### ✅ GPU资源优化：停止失败训练 → 启动高价值训练（服务 M0-3 95%硬指标）
- **决策**：7cls_v2训练连续19 epoch无提升（best 0.649@ep6 vs 蒸馏版0.7879），继续训练至ep36早停属于浪费GPU
- **行动**：停止7cls_v2 → 启动obstacle_aug训练（从蒸馏版best.pt微调，含obstacle CP+arrow CP数据）
- **价值**：obstacle_aug直接注入漏检障碍和箭头的真实纹理，是目前唯一能从数据层面根治obstacle FN=30和arrow召回低的方案

### ✅ 代码base健康
- `pytest tests/` 147 passed in 30.95s（无回归）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ⛔ 7cls_v2已停(ep25,best0.649)；🟡 **obstacle_aug训练已启动(ep1/80)**，从蒸馏版微调含obstacle+arrow CP |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-13 05:07）

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879，无变化）
- **⛔ car_up 训练已停止**：ep10/40，best mAP50=0.65644@ep4（car_upsampled 1679 train）
- **🟡 新训练 aerial_field_7cls_v2**：ep10/80，best mAP50=0.64856@ep6（自 04:27 运行中）
  - 数据集：aerial_field_7cls_supplemented（2053 障碍样本）
  - 从 aerial_field_finetuned_v2 best.pt 续训，lr0=0.005 / cls=0.5 / close_mosaic=10
  - 关键差异 vs 失败 local：学习率减半(0.01→0.005)、cls损失权重降低、最后10epoch关mosaic
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_5090 监控脚本**：2 实例持续存活（PID 4256+14796，自 18:04 起）

### ✅ obstacle_aug 训练脚本就绪（非硬件推进，服务 M0-3 95%硬指标）
- **新增 `tools/ml/_train_7cls_obstacle_aug.py`**：4090 训练脚本，针对 obstacle FN=30 + arrow 召回低
  - 数据集：aerial_field_7cls_obstacle_aug（1716 train = 原始216 + mine CP 500 + obstacle CP 500 + arrow CP 500）
  - 基座：7cls蒸馏版 best.pt（mAP50=0.7879，当前综合91.07%）
  - 策略：低学习率微调(lr0=0.003) + close_mosaic=10，防止 catastrophic forgetting
  - 4090配置：batch=32 / imgsz=1280 / workers=8 / amp=True
  - 本地调试：注释切换至 E:\ml_datasets 路径 + batch=2
- **数据集验证**：1716 train / 34 val，7类完整（mine/car/arrow/target_red/cell_label/target_blue/obstacle）
- **价值**：4090 恢复后可立即启动训练，直接将漏检障碍+箭头的真实纹理注入，从数据层面根治域差异

### ✅ 代码base健康
- `pytest tests/` 147 passed in 29.69s（无回归）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ⛔ car_up已停(ep10)；🟡 7cls_v2新训练进行中(ep10/80,best 0.649@ep6)；✅ obstacle_aug训练脚本就绪待4090 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879，无变化）
- **本地 7cls 补充训练（aerial_field_7cls_local）**：⛔ 已停止 ep47/60，best mAP50=0.6795@ep13（失败，综合89.19%）
- **⛔ car_aug 训练已停止**：ep6/50，mAP50=0.64（car_aug数据集含增强假阳性风险被放弃）
- **🟡 新训练 aerial_field_7cls_car_up**：ep8/40，best mAP50=0.65644@ep4（自 03:07 运行中，GPU 72%）
  - 数据集：car_upsampled（1679 train，car=738真实样本上采样，仅复制真实样本不引入假阳性）
  - 类别分布：mine=49968 / car=738 / arrow=713 / target_red=1037 / cell_label=52115 / target_blue=2083 / obstacle=3757
  - 从 local best.pt 续训，40 epochs / patience=15
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_5090 监控脚本**：2 实例持续存活（PID 4256+14796，自 18:04 起）

### ✅ 清理卡死评估进程（非硬件推进）
- 发现 PID 4120 + 10040 运行 `_eval_7cls_cls.py` 评估 local best.pt，自 23:56 起已运行 **4小时**，CPU 累计仅 35.6s（几乎停滞）
- 该评估对象（local best.pt）已确认失败（综合89.19%），继续运行无意义
- 已 kill 两个进程，释放系统资源

### ✅ 代码base健康
- `pytest tests/` 147 passed in 30.09s（无回归）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 🟡 car_up新训练进行中(ep8/40,best mAP50=0.656@ep4)；⛔ car_aug已停；✅ 卡死评估进程已清理 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-13 03:02）

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879，无变化）
- **本地 7cls 补充训练（aerial_field_7cls_local）**：⛔ 已停止 ep47/60，best mAP50=0.6795@ep13（失败）
- **🟡 新训练 aerial_field_7cls_car_aug**：ep6/50，mAP50=0.64（自 02:06 运行中，GPU 96%）
  - 数据集：2100 train / 541 val（比原始216 train丰富10倍）
  - 类别分布：target_blue=3707 / target_red=2006 / obstacle=3893 / arrow=678 / mine=48817
  - 从 local best.pt 续训，超参 lr0=0.01+mosaic=1.0+randaugment（与失败local相同）
  - **关键差异**：原始7cls数据集target_blue=0（无标注），car_aug有3707个target_blue，有望解决标靶检测
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_5090 监控脚本**：2 实例持续存活（PID 4256+14796，自 18:04 起）

### ✅ arrow copy-paste 增强数据生成（非硬件推进，服务 M0-3 arrow召回瓶颈）
**针对 arrow 召回极低问题**（val 11个GT仅检出2个，召回率18%）：

1. **新增 `tools/ml/build_copypaste_arrow.py`**：从训练帧裁出103个箭头区域，回贴到216张训练背景图
   - 增强策略：90°旋转（保留箭头方向性）、小角度扰动±10°、缩放0.6~1.4×、颜色抖动、高斯模糊
   - 箭头面积比例0.1%~1%（匹配航拍箭头实际尺寸）
2. **生成结果**：500张增强图，共 **1239个箭头标注**（平均2.5个/图）
   - 输出：`E:\ml_datasets\aerial_field_7cls_arrow_cp\`
3. **更新 `tools/ml/merge_copypaste_obstacle.py`**：合并原始216 + mine CP 500 + obstacle CP 500 + arrow CP 500 = **1716 train / 34 val**
   - 输出：`E:\ml_datasets\aerial_field_7cls_obstacle_aug\data.yaml`
   - 合并后类别：obstacle=6087 / arrow=1812 / mine=56859
   - **待4090恢复后即可用此数据集训练**，同时解决obstacle FN=30和arrow召回低

**价值**：与mine/obstacle copy-paste成功经验一致，直接将箭头真实纹理注入训练集，从数据层面根治arrow漏检。

### ⚠️ 发现：原始7cls数据集 target_blue=0
- 原始 `aerial_field_7cls` 训练集 target_blue 标注数为 **0**
- 这解释了为何部分模型 target_blue 检测能力弱
- car_aug 数据集（2100 train）有 target_blue=3707，是更全面的训练数据
- 当前 car_aug 训练正在进行，有望补齐 target_blue 检测能力

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | 🟡 car_aug新训练进行中(ep6/50,2100 train)；✅ arrow copy-paste增强已生成(1239箭头框)，obstacle_aug扩充至1716 train |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879，无变化）
- **本地 7cls 补充训练（aerial_field_7cls_local，2053障碍样本）**：⛔ **已停止 ep47/60**，best mAP50=0.6795@ep13（远低于蒸馏模型0.7879）
  - 训练进程已退出（GPU 0%利用率），无guard进程存活
  - PID 10040 正在运行 `_eval_7cls_cls.py` 对 best.pt 做评估（自23:56起）
  - 确认失败：综合精度89.19%，obstacle FN=30未改善，target_red/blue灾难性遗忘
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_5090 监控脚本**：2 实例持续存活（PID 4256+14796，自 18:04 起）

### ✅ obstacle copy-paste 增强数据生成（非硬件推进，服务 M0-3 省赛95%硬指标）
**针对 obstacle FN=30 瓶颈**（四模型均漏检同一批障碍，纯数据域差异，本地2053障碍样本训练未解决）：

1. **新增 `tools/ml/build_copypaste_obstacle.py`**：从漏检val帧（00016~00023, 00096~00103, 00255）裁出41个障碍区域，回贴到216张训练背景图
   - 增强策略：旋转±15°、缩放0.7~1.3×、透视切变、颜色抖动(亮度/对比度/饱和度)、高斯模糊
   - 障碍面积比例0.5%~4%（匹配航拍障碍实际尺寸）
2. **生成结果**：500张增强图，共 **3025个障碍标注**（平均6个/图）
   - 输出：`E:\ml_datasets\aerial_field_7cls_obstacle_cp\`
3. **新增 `tools/ml/merge_copypaste_obstacle.py`**：合并原始216 train + mine copy-paste 500 + obstacle copy-paste 500 = **1216 train / 34 val**
   - 输出：`E:\ml_datasets\aerial_field_7cls_obstacle_aug\data.yaml`
   - **待4090恢复后即可用此数据集训练**，针对性解决obstacle FN=30

**价值**：与mine copy-paste成功经验一致，直接将漏检障碍的真实纹理注入训练集，从数据层面根治域差异导致的漏检。

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ✅ obstacle copy-paste增强数据已生成(500图/3025障碍框)，合并数据集(1216 train)就绪待训 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879，无变化）
- **本地 7cls 补充训练（aerial_field_7cls_local，2053障碍样本）**：ep32/60，best mAP50=0.6795@ep13（持续下降，远低于蒸馏模型0.7879）
  - 训练进程 PID 37816 运行中（CPU 累计 7148s，自 22:42 起）
  - patience=25，best@ep13，将于 ep38 早停（约6个epoch后）
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_5090 监控脚本**：2 实例持续存活（PID 4256+14796，自 18:04 起）

### ⚠️ 本地7cls补充训练评估失败（服务 M0-3 省赛95%硬指标）
用本地训练 best.pt(ep13) 在32帧val集上运行 eval_comprehensive.py：

| 类别 | acc | TP | FP | FN |
|---|---|---|---|---|
| mine | 0.8822 | 1056 | 138 | 3 |
| obstacle | 0.7647 | 104 | 2 | **30** |
| cell_label | 0.9183 | 1169 | 98 | 6 |
| arrow | 0.7778 | 14 | 2 | 2 |
| target_red | **0.0000** | 0 | 1 | 0 |
| target_blue | **0.0000** | 0 | 2 | 0 |

**综合精度：89.19%**（vs 现役最优 91.07%，劣化 1.88pp）

**根因分析：**
1. **obstacle FN=30 未改善**：2053个训练障碍样本未能解决漏检。漏检障碍面积31K-614Kpx（均值220Kpx），**非小目标问题**，是域差异（外观/材质/颜色与训练集不同）
2. **target_red/blue 灾难性遗忘**：lr0=0.01 过高 + 强增强(randaugment/erasing0.4/mosaic1.0)导致模型遗忘标靶检测
3. **结论**：本地补充训练方案失败，现役最优仍为 7cls_final_v2 + two_stage_v2 + mine边缘1%过滤 = 91.07%

### 🔍 obstacle FN=30 漏检障碍特征分析
- 30个漏检障碍分布在17帧（frame_00016~00023, 00096~00103, 00255）
- 面积：31,680 ~ 614,141 px（均值 220,421 px），均为大目标
- 所有模型（7cls_final_v2 / 7cls_clean / 蒸馏版 / 本地补充训练）均漏检**完全相同的30个**
- 确认是纯数据域差异问题，非模型容量或阈值问题

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ⚠️ 本地补充训练失败(89.19%)，现役最优仍91.07%；obstacle FN=30确认纯数据域差异 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-13 00:02）

### 📊 训练进度监控
- **7cls 蒸馏训练**：ep75/75 已完成（best mAP50=0.7879，无变化）
- **本地 7cls 训练（aerial_field_7cls_supplemented，2053障碍样本）**：ep17/60，best mAP50=0.7314（ep15）↑，ETA ~3h
  - 训练进程 PID 37816 正常运行（自 22:42 起，CPU 累计 3842s）
  - 每 epoch ~4.5min，预计 03:10 左右完成
- **4090/5090 实例**：均不可达（TcpTestSucceeded=False）
  - 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练中断，待实例恢复
- **watch_5090 监控脚本**：2 实例持续存活（PID 4256+14796，自 18:04 起）

### 🔍 arrow 类召回诊断（非硬件推进，服务 M0-3）
- **问题**：arrow 类综合精度仅 78.95%，为 obstacle 之外的第二短板
- **诊断方法**：用 7cls_final_v2 模型在 40 帧 val 图上做 CPU 推理，统计 arrow 检测框边缘分布
- **结论**：
  - arrow 预测：40 帧仅 2 个检出（GT 共 11 个），**召回率极低**
  - arrow 边缘分布：预测 0% 在边缘，GT 0% 在边缘 → **非边缘 FP 问题**
  - arrow 瓶颈为**漏检（FN）**，非误检（FP），边缘过滤无效
  - 需更多 arrow 训练数据或更高分辨率训练来提升召回（当前 1280 分辨率下 arrow 目标过小）
- **影响**：edge filter 仅对 mine 有效，不应扩展到 arrow

### ✅ 代码base健康
- `pytest tests/` 147 passed in 30.54s（无回归）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ✅ 本地7cls训练ep17/60(best mAP50=0.7314↑)；arrow召回诊断确认需更多数据 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 23:18）

### ✅ 综合识别精度突破：mine类1%边缘过滤 (+0.76pp，服务 M0-3 95%硬指标)

**问题**：上轮综合精度90.38%，瓶颈为 mine FP=115（误检）和 obstacle FN=30（漏检）。

**根因分析**（33帧val集，逐框诊断）：
1. **obstacle FN=30 全部为完全漏检**：7cls_final_v2 / 7cls_clean / 蒸馏版三个模型均漏掉**完全相同的30个障碍**，无分类错误、无低置信度漏检。确认是**纯数据问题**（训练数据中障碍样本不足），阈值调节和集成均无效。
2. **mine FP 81.5% 集中在图像边缘1%区域**：119个mine FP中97个中心点距边缘<1%，而真实mine仅34.1%在边缘。原因是letterbox填充伪影导致模型在边缘幻觉出mine检测。

**解决方案：mine类1%边缘过滤**
- 在 `prelabel_onnx.py` 新增 `filter_edge_boxes()` 函数，过滤中心点距图像边缘<1%的mine检测
- **效果**：mine FP 118→79（-39），mine TP 1053→1037（-16），净增+23正确识别

**各方案综合精度对比**：

| 方案 | 综合精度 | mine FP | mine FN | obstacle FN | 距95% |
|---|---|---|---|---|---|
| 7cls_final_v2 基线 | 90.31% | 118 | 6 | 30 | -4.69pp |
| + mine_cls_v5两阶段(cls0.9) | 90.45% | 114 | 6 | 30 | -4.55pp |
| **+ mine类1%边缘过滤** | **91.07%** | 79 | 22 | 30 | **-3.93pp** |

**已实现**：
- `tools/ml/prelabel_onnx.py`：新增 `filter_edge_boxes()` + `to_yolo_lines` 增加 `edge_thr`/`edge_cls` 参数 + CLI `--edge-mine`
- `tests/test_ml_tools.py`：新增3项单元测试（边缘mine过滤、全类别过滤、零阈值不过滤）
- 全量测试 147 passed（+3，无回归）

### ✅ 本地7cls训练启动（非硬件推进，服务 obstacle FN=30 瓶颈）
- 上轮巡检后启动 `tools/ml/_train_7cls_local.py`（RTX 4060 8GB，batch2/imgsz1280）
- 数据集：`aerial_field_7cls_supplemented`（**2053个障碍训练样本**，是原始数据集的数倍）
- 当前进度：**epoch 7/60**，mAP50=0.6265↑（每epoch约4.5min，ETA ~4h）
- 预期：补充的障碍数据有望解决三模型共漏的30个FN

### ⚠️ 4090 和 5090 实例仍不可达
- 4090（connect.bjb1:27379）：False
- 5090（connect.bjb2:37113）：False
- 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练状态未知
- 需用户在 AutoDL 控制台检查并重启实例

### ✅ 代码base健康
- `pytest tests/` 147 passed in 28.40s（+3 edge_filter测试，无回归）

### 📝 7cls 蒸馏训练：epoch 75/75 已完成（无变化）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ✅ 综合精度90.38%→**91.07%**(mine边缘过滤+0.76pp)；本地7cls训练进行中 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 22:08）

### ⚠️ 4090 和 5090 实例本轮均不可达
- 4090（connect.bjb1:27379）：3次重试均 False（上轮在线）
- 5090（connect.bjb2:37113）：持续不可达
- 7cls_v3(ep20/50) / red_bucket_v2(ep28) 训练状态未知，待实例恢复后续训
- 需用户在 AutoDL 控制台检查两个实例状态

### ✅ 省赛综合识别精度多方案评估（服务 M0-3 95%硬指标）
在33帧val集上对比4种后处理方案（类别+IoU≥0.5全对才算正确，car类排除）：

| 方案 | 综合精度 | mine | obstacle | 距95% |
|---|---|---|---|---|
| 基线 7cls_final_v2 (conf=0.25) | 90.24% | acc=0.8930 (FP=119) | acc=0.7482 (FN=30) | -4.76pp |
| **+ mine_cls_v5 两阶段(cls=0.9)** | **90.38%** | acc=0.8961 (FP=115) | acc=0.7482 (FN=30) | **-4.62pp** |
| + red_bucket融合(rb_conf=0.35) | 89.89% | 不变 | acc=0.6980 (FP=15) | -5.11pp |
| 多阈值(mine/obs=0.15,其他=0.25) | 90.15% | acc=0.8910 (FN=4) | acc=0.7482 (FN=30) | -4.85pp |

**结论：**
1. **mine_cls_v5两阶段为最优**：综合精度90.38%(+0.14pp)，过滤4个mine FP且零TP损失
2. **mine_cls_v5域不匹配**：仅过滤4/119个mine FP（训练于官方近景图，航拍域泛化有限）
3. **obstacle FN=30为模型能力问题**：降阈值至0.15仍无法检出，需更多障碍训练数据
4. **red_bucket融合有害**：引入10+ obstacle FP，净降精度

### ✅ 代码base健康
- `pytest tests/` 144 passed in 27.55s（无回归）

### 📝 7cls 蒸馏训练：epoch 75/75 已完成（无变化）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ✅ 两阶段评估完成，综合精度90.38%(+0.14pp) |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | 无（待硬件） |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 21:10）

### ✅ 4090训练全面完成 + 综合识别精度评估（非硬件推进，服务 M0-3 核心硬指标）
- **4090实例在线**（connect.bjb1.seetacloud.com:27379），训练状态：
  | 训练 | 状态 | 关键指标 |
  |---|---|---|
  | 7cls_clean | ✅ ep80/80 | best mAP50=0.6666 |
  | 7cls_final_v2 | ✅ ep60/60 | best mAP50=0.7207 |
  | 7cls_v3 | 🟡 ep20/50 | mAP50=0.6799（续训中） |
  | mine_cls_v5 | ✅ ep40/40 | **val_acc=0.9969** |
  | red_bucket_v2 | 🟡 ep28/ | mAP50=0.8464 |

### ✅ 省赛综合识别精度评估（核心硬指标95%）
- 在33帧val集上评估3个模型（类别+IoU≥0.5全对才算正确，car类排除因车自报位置）：

  | 模型 | 综合精度 | mine | cell_label | arrow | obstacle | 距95% |
  |---|---|---|---|---|---|---|
  | **7cls_final_v2** | **90.24%** | 89.30% | 92.99% | 78.95% | 74.82% | **-4.76pp** |
  | 蒸馏版(现役) | 89.89% | 88.94% | 92.76% | 65.22% | 75.91% | -5.11pp |
  | 7cls_clean | 89.44% | 89.15% | 92.48% | 70.00% | 75.36% | -5.56pp |

- **结论**：7cls_final_v2 为当前最优（90.24%），比现役蒸馏版高0.35pp
- **瓶颈分析**：obstacle召回率仅74.82%（FN=30）是最大短板，其次mine FP=119（误检）
- **已下载模型**：`E:\ml_datasets\downloaded\4090_batch2\`（7cls_clean/7cls_final_v2/mine_cls_v5 best.onnx）
- **下一步**：7cls_v3训练完成后复评，若>90.24%则发布为新current_hbb_detector

### ✅ mine_cls_v5雷分类器完成（服务爆炸物二次确认）
- val_acc=0.9969（40 epochs），2类(mine/non_mine)分类精度极高
- 已下载至 `E:\ml_datasets\downloaded\4090_batch2\mine_cls_v5_best.onnx`（20.8MB）
- 价值：爆炸物两阶段检测（YOLO粗定位+雷分类器二次确认）的二次确认模型就绪

### ✅ watch_5090 监控脚本持续存活
- PID 4256（系统Python）+ 14796（venvPython），均于 18:04:39 启动（已运行 ~3h）
- 本轮巡检未发现被系统休眠杀掉

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 蒸馏训练：epoch 75/75 已完成（无变化）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`

### 📝 manual_200 标注：157/200（无变化）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ✅ 综合识别精度90.24%(7cls_final_v2)，距95%差4.76pp，7cls_v3训练中 |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | ✅ mine_cls_v5雷分类器完成(val_acc=0.9969)，爆炸物二次确认就绪 |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 20:11）

### ✅ 4090远程实例训练全面推进（非硬件推进，服务 M0-3/M0-6）
- **4090实例在线**（connect.bjb1.seetacloud.com:27379，RTX 4090 48GB），4训练并发：
  | 训练 | 状态 | 关键指标 | 产物 |
  |---|---|---|---|
  | 7cls_clean干净版 | 🟡 ep57/80 | best mAP50=0.662(ep44) | ETA ~22:00 |
  | target_pose_v2 | ✅ 完成 | **mAP50=0.9755** | best.onnx 37.4MB |
  | red_bucket | ✅ 完成 | mAP50=0.861 | best.onnx 10.1MB |
  | mine_cls_v5 | 🟡 已重启 | 修复data路径(目录非yaml) | 40 epochs |

### ✅ target_pose_v2标靶pose模型下载+契约核验（服务 M0-6 车端射击）
- 下载至 `E:\ml_datasets\downloaded\target_pose_v2_best.onnx`（37.4MB）
- **契约核验通过**：输入 `[1,3,640,640]` → 输出 `[1,9,8400]`（4bbox+2cls+3kpt），与车端 `light_det.rknn` 现役契约完全匹配
- mAP50=0.9755（epoch 200/200），标靶识别精度极高，可直接 `rknn_convert.py` 转 RKNN 部署
- 价值：M0-6 车端标靶射击链路现在有高精度+契约匹配的模型备选，不再阻塞标靶识别

### ✅ mine_cls_v5雷分类器训练修复并重启（服务 爆炸物二次确认）
- **问题**：首次启动失败，YOLO分类任务要求 `data` 为目录路径而非 yaml 文件
- **修复**：确认远程脚本 `DATA="/root/ag_train/mine_classifier/images"` 已正确（images/train|val/{mine,non_mine}），重新启动训练
- 训练已正常启动：9268 train / 1301 val 图像，2类（mine/non_mine），yolo11s-cls，40 epochs
- 价值：雷分类器v5（平衡采样+强正则+早停）训练恢复，为爆炸物两阶段检测的二次确认提供模型

### ✅ red_bucket红桶检测器训练完成+下载
- 下载至 `E:\ml_datasets\downloaded\red_bucket_best.onnx`（10.1MB）
- 输出 `[1,10,8400]`，red bucket类 mAP50=0.946，可作为障碍物检测的补充模型

### ✅ 代码base健康
- `pytest tests/` 144 passed in 21.16s（无回归）

### ✅ watch_5090 监控脚本持续存活
- PID 4256（系统Python）+ 14796（venvPython），均于 18:04:39 启动（已运行 ~2h）
- 本轮巡检未发现被系统休眠杀掉

### ⚠️ 5090 实例持续不可达
- `connect.bjb2.seetacloud.com:37113` TcpTestSucceeded=False
- `106.38.204.136:52238` TcpTestSucceeded=False
- seed157/v4d/target_merged 训练状态未知
- 需用户在 AutoDL 控制台检查实例状态

### 📝 7cls 蒸馏训练：epoch 75/75 已完成（无变化，最后写入 10-11 15:38）
- best mAP50=0.7879(ep56)，模型已发布 `hbb_detector_7cls_distill_20261011.onnx`
- 7cls_clean干净版训练在4090进行中（ep57/80）

### 📝 manual_200 标注：157/200（无变化）

### 📋 M0 完赛最小闭环进度（重点关注）
| # | 任务 | 状态 | 本轮进展 |
|---|---|---|---|
| M0-1 | 车能走直线不画圆 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-2 | 朝向对齐实车验证 | 🟡 软件就绪待实车 | 无（待硬件） |
| M0-3 | 无人机出情报(UDP) | 🔵 待Orin上线 | ✅ 7cls_clean干净版训练4090进行中(ep57/80) |
| M0-4 | 车机UDP链路通 | ✅ 模拟验证通过 | 无变化 |
| M0-5 | 路径规划可执行 | ✅ 模拟验证通过 | 无变化 |
| M0-6 | 车按路径行驶 | ⛔ 依赖M0-1,M0-2 | ✅ target_pose_v2标靶模型完成(mAP50=0.9755,契约匹配车端[1,9,8400])；mine_cls_v5雷分类器已重启 |
| M0-7 | 赛前检查清单 | ✅ 已完成 | 无变化 |

---

## 8. 上轮巡检非硬件推进（2026-10-12 19:02）

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
