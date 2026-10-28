---
卡号: TEST-RESEARCH-20261028
ns: archrev-1028
agent: /root/test_research
树节点: AR-2
owner: /root/test_research
reviewer: /root/flow_research
kind: verification
status: in_progress
创建: 2026-10-28
更新: 2026-10-28
摘要: 只读核查测试、仿真、回放和部署证据链，结果交主卡。
---

# TEST-RESEARCH-20261028 · 测试链路研究

## 读过的台账条目
D-007、D-019、D-023、D-025：不连接设备，不部署，不把合规门禁当功能验证。

## 复用检索结果
沿 docs/README.md 入口阅读资料；复用 sim_cli、现有测试和 workflow 守卫，不新增测试平台。

## 新增轮子及理由
无。

## 写域
无代码写域；只读研究，结果由 Lead 追加本卡。不触碰他人更改。

## 验收清单
- [x] 核对当前测试/仿真实现与入口，给出文件行号。
- [x] 区分已证实缺口与改造建议。

验收命令：独立运行 python tools/wf/check.py（cwd 为 dzrobot_v2）；文档收口由 Lead 统一安排独立 reviewer 复跑。

## 回滚点
仅撤回本 ns 新文档，不回滚生产。

## 文档变更
仅本卡，研究结果交主卡。

## 研究交付
独立执行 wf/check.py exit 0 / GREEN。核实 sim_cli/bench/现场片段回放的覆盖区别、归档接线和部署哈希对账；只读复现 trace 的 event 重名实参错误。详细证据由 Lead 汇入同 ns 主卡 ARCH-RESEARCH-20261028。本卡研究已交付，正式收口仍受全库既有红阻塞，未宣称实车通过。
