---
来源: ext_yb_tree_20261022.tar.gz :: yb_tree/reference/members/wtt/entity_train/README_驯化指南.md:1-223〔未入库〕
取证: 档（外部成员文档；库内此前命中 0，本轮首次提炼）
更新: 2026-10-24
status: 现行
取代: 无
---

# 外部资料 · 02 成员wtt_实体训练（代码「事物」分类器）

## TL;DR（给 agent 看的）

- **这是什么**：外部成员「万涛涛（wtt）」无人机侧交付里，给代码打「事物标签」的训练管线：让模型认识本仓库代码里的水桶、爆炸物、敌打击区、我方打击区、桥洞、绿线六类事物，能判断「这段代码在干什么」。
- **一句话结论**：两条路线——A 判别式分类器（CodeBERT 冻结编码器 + 轻量分类头，已跑通 324 条样本）与 B LoRA 驯化代码大模型（Qwen2.5-Coder，生成式）；建议**先 A 后 B**，A 数据要求低、见效快。
- **关键数字**：8 类标签（含 other 负类）；8 GB 显存下 A 路线 2~4 GB 轻松、B 路线 QLoRA 可行；路线 A 训练十几分钟、B 1~3 小时；数据 500~2000 条指令对、留 10% 考试集。
- **最大坑**：**只喂正样本** → 模型见啥都硬分一个事物；**标签靠规则而非语义**（hit 在游戏代码是「打击」、别的库是「命中缓存」）→ 训练集要覆盖不同语义的同词。
- **与本项目关系**：与 dzuav C++ 分类器互补，作快速迭代工具；吸收决策表第 9 条已决定**进树（reference）**，但**暂不进 CI**（第 11 条：单种子运行时长未测）。

## 1. 这一篇是什么

- 来源是 tar ext_yb_tree_20261022.tar.gz 内 yb_tree/reference/members/wtt/entity_train/README_驯化指南.md（222 行，11340 B）。
- 硬件前提：**RTX 4060 Laptop 8 GB 显存**，文中所有参数按 8 GB 给出。
- 取证分级：**档**（外部文档；本篇未复跑其训练）。

## 2. 两条路线

| 维度 | A 判别式分类器 | B LoRA 驯化代码大模型 |
|---|---|---|
| 能力 | 给一段代码 → 判它属于哪个事物 | 问答/解释/定位/改代码（生成式） |
| 模型 | CodeBERT(125M) / Qwen-Coder-0.5B | Qwen2.5-Coder-1.5B/3B-Instruct |
| 8 GB 显存 | 轻松（2~4 GB） | QLoRA 可行 |
| 数据量 | 数百~数千条片段+标签 | 500~2000 条高质量指令对 |
| 训练耗时 | 十几分钟 | 1~3 小时 |
| 本项目文件 | entity_labels.py、build_dataset.py、train_classifier.py、predict.py | 见 §4（LLaMA-Factory） |

**建议顺序：先 A 后 B** —— A 先把「识别事物」跑通，B 在其标签体系与数据基础上再升级成「能对话」。（README_驯化指南.md:15-27〔未入库〕）

## 3. 标签体系（8 类，含负类）

| 标签 | 事物 | 代码判别特征 | 代表代码 |
|---|---|---|---|
| bucket | 水桶/散落障碍物 | BUCKET、BUCKETS_N、OBSTACLE_D_CM | BUCKETS_N = 6 |
| explosive | 爆炸物/地雷 | EXPLOSIVE(S)_N、role=="explosive"、MINE_ZONE | explosive_order(...) |
| enemy_strike | 敌打击区 | ENEMY_N、stop_face、hit、蓝面 | strike_cell(...) |
| friend_strike | 我方打击区 | FRIEND、无蓝面 | kind = FRIEND |
| strike_zone | 打击区通用/敌我判别 | TARGET(S_N)、strike_* | target_rc() |
| bridge | 桥洞 | BRIDGE_CELLS、BRIDGE_* | is_bridge(rc) |
| green_line | 绿线禁入 | GREEN_LINE、LINE | line_cells(...) |
| other | 无关代码（**负类，必不可少**） | 无任何特征 | def man(a,b): ... |

要点：
- **负类决定模型的「否定能力」**——没有 other，模型会把所有代码硬归到某个事物。
- 标签允许层级/多标签（enemy_strike 是 strike_zone 的子类），先单标签跑通再升级。
- 自动打标只是**初标**，训练前人工抽查 50~100 条改错，收益远大于加数据。（README_驯化指南.md:29-50〔未入库〕）

## 4. 路线 A 已跑通的产出与用法

- 抽取标注样本（纯标准库）：python build_dataset.py --show 2 → dataset.jsonl，**产出 324 条**。
- 训练：python train_classifier.py --head logreg（默认 AI-ModelScope/codebert-base，失败自动走 HF 镜像）。
- 推理：predict.py --text / --file / --func。
- 训练产物在 output/：config.json + head.joblib + results.txt；**训练只保存分类头**，编码器本体（CodeBERT 约 500 MB）导入时按 config.json 的模型名自动重载（第一次）。
- 一行 import：EntityClassifier("output")，自动用 GPU（无则回落 CPU）。
- 路线 B 的 LoRA 产物是「增量权重」，不能直接 AutoModel.from_pretrained，须先 llamafactory-cli export 合并导出。

（README_驯化指南.md:51-138〔未入库〕）

## 5. 数据质量三条铁律（路线 B 尤其）

1. **每条都要有「为什么」**（不只是标签，要有解释）。
2. **混入负例与刁钻问题**（「这段代码和水桶有关吗？」→ 无关）。
3. **留 10% 做考试集**，考试集在训练中绝对不碰。

验收三看：考试集（改写过的问题，防背题）+ 幻觉检查（问「地雷直径多少？」——README 说障碍物 26cm、地雷无直径，模型瞎编数字就是没驯化好）。（README_驯化指南.md:163-198〔未入库〕）

## 6. 常见坑（6 条，都在这类项目实际发生）

| # | 坑 | 后果 |
|---|---|---|
| 1 | 只喂正样本 | 没负类 → 对任何代码硬分一个事物 |
| 2 | 类别严重不平衡 | 小类（bridge/green_line）被吞 → 加权/过采样 |
| 3 | 标签靠规则而非语义 | hit 在不同库语义不同 → 训练集要覆盖同词异义 |
| 4 | 测试集泄漏 | 同段代码既在训练又在测试 → 准确率虚高 |
| 5 | 过拟合 | 几百条时别追训练集 100%，看验证集；小数据用小学习率 |
| 6 | 生成式模型背答案 | 换问法就崩 → 考试集要改写问题 |

（README_驯化指南.md:200-208〔未入库〕）

## 7. 与本项目的关系

| 项 | 内容 | 出处 |
|---|---|---|
| 吸收决策 | entity_train 训练管线 + dataset.jsonl **进树（reference）** | 吸收决策表第 9 条（吸收决策与待验证清单-20261018.md:49〔未入库〕） |
| 是否进 CI | **暂不进 CI**（记录为待验证 V2）：单种子运行时长未测，直接进 CI 有超时风险 | 同表第 11 条（:51〔未入库〕） |
| 互补对象 | 与 dzuav C++ 分类器互补，作快速迭代工具 | 同表第 9 条（:49〔未入库〕） |

## 相关文件

- 本册：[00_索引.md](00_索引.md)
- 吸收决策来源：[../历史/02_并入与改名_20261018.md](../历史/02_并入与改名_20261018.md)
- 决策台账：[../赛事与治理/04_决策台账摘编.md](../赛事与治理/04_决策台账摘编.md)
- 原文：ext_yb_tree_20261022.tar.gz 内 yb_tree/reference/members/wtt/entity_train/
