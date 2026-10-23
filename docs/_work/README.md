---
status: 过程
更新: 2026-10-23
---

# docs/_work —— 过程稿只读区（不是文库）

这里放 10-21 文档收口期间产生的**过程稿**：时间线片段、清理脚本与清单、决策候选表、事故现场记录。它被 `doc_policy.json` 的 `exempt_prefixes` 豁免，**不计入"资料库篇数"、不进 `目录索引` 的正文口径**（`docs/20_资料索引库/README.md` 的 42 篇不含本目录）。

## 三条纪律

1. **只出不进**：新的过程稿写 `_tmp_session/<卡号>/`；本目录不再新增。
2. **被正式文档引用的不许搬**（搬了链接就断），要在原文件里写清"被谁引用"。
3. **提炼进库后留档不删除**：结论进 `docs/20_资料索引库/` 或卡库，本目录保留原始数据供复核。

## 现状（2026-10-23 19:35 清理后）

| 类别 | 文件 | 处置 |
|---|---|---|
| 时间线片段（锚定保留） | `timeline_merged.md`、`timeline_doc-car/drone/ops/cards/prompts.md` | `00_项目总览/时间线.md` 逐段引用，**不许搬** |
| 档期素材（锚定保留） | `TRACK-DAY-20261022/`（48 MB / 45 个车端脚本） | 见下：已移出 |
| 清理工具数据（被 `docs/_工具/*.py` 读） | `inventory.csv`、`cleanup_ledger.csv`、`ds_*.json`、`salvage_report.json` | 保留，脚本默认路径指向此处 |
| 决策候选 / 审计记录 | `decision_list.csv`、`plan_库外清理.md`、`index_audit.md`、`rebuild_8_lost.md` | 保留；结论已在卡与历史篇 |
| 事故现场 | `incident_20261022_外部删除.md`、`archive_manifest.csv`、`pack_report.txt` | 保留（历史不计入篇数） |

## 2026-10-23 移出记录

- `TRACK-DAY-20261022/`（48 MB / 45 文件，10-22 车端作业脚本与产物）→ `_archive/20261023/docs_work/TRACK-DAY-20261022/`
  理由：无任何现行文档引用该**目录**（`交接_20261023` 与 `30/README` 里出现的同名条目指的是**卡** `在办/TRACK-DAY-20261022.md`，不是这个目录）。体积占 `_work` 的 98%，移入归档区保留可查，本目录 49 MB → 535 KB。
- 根目录 `_work_tmp_d11_session.sh`、`_work_tmp_start_aim_d11.sh` → `_archive/20261023/d11_scripts/`（10-22 D11 现场脚本）。
- 根目录空目录 `_recover_tmp/`（0 文件）→ 已删除。

> 注：`dzrobot_v2/docs/目录索引.json` 仍列有上述两个根目录脚本与 `_recover_tmp`，需其 owner 下次跑 `tools/wf/repo_index.py --write` 时自然刷掉；本目录无权代跑（会扫进别的会话的未提交改动）。
