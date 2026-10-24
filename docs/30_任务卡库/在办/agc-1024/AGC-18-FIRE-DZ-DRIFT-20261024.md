---
来源: p-aim P1B 报告 §风险；Lead 2026-10-24 17:5x 立卡
取证: 车端 robot.yaml:179 现场值 + MANIFEST + 已有 mvlog
更新: 2026-10-24
status: in_progress
ns: agc-1024
agent: p-aim
树节点: AGC-18
写域: 本卡；`_tmp_session/agc-1024/agc18/**`（新）
摘要: fire_dz_far_px 由 MANIFEST 记录的 25 漂到现行 18.0，且 yaml:149 注释写 14 —— 三个数互不一致；只取证不改值，M2 现场 A/B 后再定。
---

# AGC-18-FIRE-DZ-DRIFT-20261024 · fire_dz_far_px 漂移取证

owner: p-aim；reviewer: Lead。

## 1 依据与复用（五必填）

- 读过的台账：P1B 卡 §风险/回滚；MANIFEST 10:39 记录。
- 复用检索：已有 mvlog 采样与 fire_rec 记账；不新建采集管线。
- 新增轮子：无（只做离线回放与时间线取证）。
- 验收命令：回放脚本 exit 0 + 逐项数字。
- 回滚点：无生产改动。

## 2 当前问题

同一参数三处不一致：MANIFEST(10:39)=25、车端 robot.yaml:179=18.0、yaml:149 注释="须 < fire_dz_far_px=14"。谁在何时改的、影响多大都不知道。

## 3 唯一假设与验收

- 假设：18.0 是某次实验的临时收紧，未走台账。
- 必测：①时间线（git log / MANIFEST / robot_logs）②用已有 mvlog 量化 18 vs 25 的远距开火窗口差异（提前/推迟多少帧、漏打/多打几帧）③M2 现场 A/B 方案。
- 边界：**不改车上 config**；与 P1B 无关，不并案。

## 5 执行记录

（待写）

## 10 文档变更

新增本卡；结论若出需提炼进 `20_资料索引库/视觉与云台/`。
