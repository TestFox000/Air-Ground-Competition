---
卡号: AIM-BASELINE-20261023
owner: P（云台PID/打靶线）
reviewer: Lead + R
kind: work
status: in_progress
创建: 2026-10-23 18:45
来源: 决策台账 D-033（靶速 0.3 m/s 现行）、D-030（靶形态/0.70 m 轨道）、TRACK-FIELD-D11-20261022（876 帧诊断）、_tmp_session/AG-TEAM-20261023/P/PLAN.md
取证: 实/档/未
更新: 2026-10-23
取代: 无
摘要: D11 静止 1.41 m、禁火，45 s 两段（静止 20 s + 人手动 0.30 m/s 横扫 25 s），取得首个有效跟踪基线；不测开门/实发。
ns: aim-1023
agent: aim-lead（身份待核实）
树节点: A-1
写域: docs/30_任务卡库/在办/aim-1023/, _tmp_session/aim-p0/
---

# AIM-BASELINE-20261023 · 云台首个有效跟踪基线（F-P 第一格）

## TL;DR
- **一句话**：在 D11 格心、站距 1.41 m、**禁火**下录 45 s（静止 20 s + 手动 0.30 m/s 横扫 25 s），回答「检测连续性与跟踪误差是否够格」，为后续 PID 单变量整定提供基线。
- **不做**：开门、实发、命中（另卡）；不改代码/config；不动 1 m/s 旧口径（D-033 已改判 0.3）。
- **依据**：D-033 明写 0.3 m/s 下提前量 7~11 px < 25 px 框、云台需速仅 17~28% ⇒ 真瓶颈是**观测连续性**，不是增益。

## 1 五必填
| 项 | 内容 |
|---|---|
| 读过的台账 | D-033（0.3 m/s 现行，取代 D-031）；D-030（1:1 圆面/0.70 m 轨道/不翻面）；D-027（阿克曼，与本卡无关） |
| 复用检索 | 复用 `_tmp_session/aimtune/{aim_rec.py,aim_stat.py,grab.py}`，不新造采集；脚本 `_tmp_session/AG-TEAM-20261023/P/run_baseline_45.sh` |
| 新增轮子及理由 | 仅新增一个 45 s 编排脚本（串预检/两段录制/统计），因 aimtune 三工具无编排；已 `bash -n` 通过 |
| 验收命令 | 车端 `bash run_baseline_45.sh`（交互 3 问）；本地 `python` 复算 stat；见 §4 |
| 回滚点 | 本卡不改任何生产文件；现场只读录制；若发过 `/nav/scan_request`，云台回 IDLE 即停 |

## 2 质量契约
- objective：取得一份可复算的 45 s 跟踪基线（静止+扫动），并给出五层验收结论。
- acceptance：① 预检全过（进程/发布者/相机/禁火/无桶/TRACK）；② 静止段 present≥95%、dx σ≤2 px、|dx| 中位≤10 px、in14≥80%、饱和 0、fb 间隔≤50 ms、TRACK≥95%；③ 扫动段 present≥85%、中位≤25 px、P90≤50 px、in14≥20%、in26≥60%、饱和≤10%、TRACK≥95%；④ 全程实发 0；⑤ 数据落盘并可复算。
- verify：`bash run_baseline_45.sh` + `aim_stat.py`；Lead 用同一 jsonl 复算。
- inScope：`_tmp_session/AG-TEAM-20261023/P/**`、车端 `/tmp/P_baseline*`（仅读取回传）。
- outOfScope：`src/**`、`config/**`、`tools/deploy_to_car.py`、任何发车/运动指令。

## 3 现场步骤（Lead 排班，人执行摆位/扫靶）
1. 停掉 dzrobot2 的 camera/det（当前 18:40 起仍在跑；相机唯一）。
2. 车摆 D11 格心、车头 +x（朝靶）；靶 D7/E7 蓝面朝南；车心→板心 1.41 m。
3. Lead 起 `dzrobot` 的 camera+det+gimbal(+chassis 供 `/camemo_feedback`)，**不跑 dzbrain**。
4. 跑 `run_baseline_45.sh`：预检 → 静止 20 s → 手动 0.30 m/s（单程 2.33 s，26 BPM）4 往返 25 s → 统计。
5. Lead 拉回 `/tmp/P_baseline*` 到 `P/data/`，复算判据。

## 4 判据与失败归因
- 判据见 §2 acceptance；脚本逐条打印 PASS/FAIL，最后一行 `RESULT PASS|FAIL`。
- 归因顺序（不跳级）：预检失败→检测漏检（present 低）→出画（P90 巨大且贴边）→饱和（贴 800）→控制增益（其余）→反馈缺失（fb 间隔/Pub）→状态机。
- 失败后的唯一变量：**保持 L 档不动**，先判归因；只有归因落到控制增益且饱和≤10% 时，下一轮才动 `pid_kp 2.5→4.0`。

## 5 风险
- 两套栈混用：脚本 `readlink` 拦 dzrobot2；桶误检：预检抓帧人工确认；`/camemo_feedback` Pub=0：预检拦；`--once` 掉消息：连发 5 次。
- L 档未 frozen 登记（R-07）：本卡只测不发布；登记与回滚归 Lead/E。

## 6 证据落盘
- `_tmp_session/AG-TEAM-20261023/P/data/{P_static.jsonl,P_sweep.jsonl,P-static-stat.json,P-sweep-stat.json,P_pre.png}`
- 车端原件保留 `/tmp/P_baseline*` 至收车后 24 h。

## 7 结论（待填）
| 项 | 判定 | 证据 |
|---|---|---|
| 预检 | 待跑 | — |
| 静止段 | 待跑 | — |
| 扫动段 | 待跑 | — |
| 首个有效基线 | 待跑 | — |

## 8 文档变更（§10）
- 新增：本卡 `docs/30_任务卡库/在办/aim-1023/AIM-BASELINE-20261023.md`；`docs/目录索引.md/.json`（gen_index 重生成）。
- 修改：无。归档：无。
