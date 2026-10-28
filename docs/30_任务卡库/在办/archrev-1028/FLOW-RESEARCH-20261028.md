---
卡号: FLOW-RESEARCH-20261028
ns: archrev-1028
agent: /root/flow_research
树节点: AR-3
owner: /root/flow_research
reviewer: /root/test_research
kind: review
status: in_progress
创建: 2026-10-28
更新: 2026-10-28
摘要: 只读核查 agent 工作流、隔离机制、证据管理和改造优先级。
---

# FLOW-RESEARCH-20261028 · agent 工作流研究

## 读过的台账条目
D-008 冻结区与 D-044 提交/推送由用户决定；不擅自简化或修改现行规则。

## 复用检索结果
复用资料库与卡库、现有工作流门禁；不引入第二套管理工具。

## 新增轮子及理由
无。

## 写域
无代码写域；只读研究，结果由 Lead 追加本卡。不触碰他人更改。

## 验收清单
- [x] 针对本项目给出工作流建议，区分规则与拟议改造。
- [x] 论断带证据，避免拿旧状态当当前状态。

验收命令：独立运行 python tools/wf/check.py（cwd 为 dzrobot_v2）；文档收口由 Lead 统一安排独立 reviewer 复跑。

## 回滚点
仅撤回本 ns 新文档，不回滚生产。

## 文档变更
仅本卡，研究结果交主卡。

## 研究交付
独立执行 wf/check.py exit 0 / GREEN；只读核对现行 E3 RUN-EVIDENCE、最近状态板、两仓基线与既有插件。结论：优先落实证据链和首失败边界；不另建仪表盘，不增加派工层级。建议五步现场循环及证据由 Lead 汇入同 ns 主卡 ARCH-RESEARCH-20261028。本卡研究已交付，正式收口仍受全库既有红阻塞。
