---
title: 库外清理结果与待裁清单
类型: 报告
更新: 2026-10-22
状态: 现行
owner: Lead
---

# 库外清理结果（2026-10-22）

> 计划书：[plan_库外清理.md](plan_库外清理.md) · 逐条裁决表：[decision_list.csv](decision_list.csv) · 删除账本：[cleanup_ledger.csv](cleanup_ledger.csv)

## 1. 数字

| 项 | 清理前 | 清理后 |
|---|---:|---:|
| 库外 `.md` | 1056 | **213** |
| 其中：代码仓自带（原位） | — | 152 + 50 + 9 ≈ 211（含并发新增 2 篇） |
| 库内 `.md`（四库） | 192 | **198** |
| 归档快照 | 0 | **7 个 tar / 1468 MB** |

处置明细：

| 动作 | 数量 | 证据 |
|---|---:|---|
| A 类删除（库内同哈希） | 134 | 134/134 副本复验同哈希 |
| B 类删除（库外重复的非首份） | 363 | 259 篇保留份在盘上同哈希；其余涉及 101 个唯一哈希，**99 个在 tar 内复验命中**，2 个属已知丢失/旧版项（见 incident） |
| 打包进 tar（文档 + 过程数据） | 341 篇 md | 7 个 tar 共 15,438 个成员，`verify_tars2.py` 7/7 OK |
| 个人域提炼后删除 | 5 | 事实入 `06_本机值守与守护器.md`、`历史/03_*` |
| 外部进程删除后回填 | 153 | 152 精确 + 1 篇旧版（见 incident） |
| 打包期丢失（不可恢复） | 8 篇 md + 1 个嵌套 tgz | 见 incident §2 |

## 2. 快照包清单（`docs/_archive/snapshots/`）

| 包 | 内容 | 大小 |
|---|---|---:|
| `ext_gitee_air_ground_20261022.tar.gz` | 他人 gitee 仓快照 | 431 MB |
| `process_TRACK-FIELD-R3_20261022.tar.gz` | R3 现场作业全量 | 320 MB |
| `process_drafts_20261022.tar.gz` | `_tmp_session` 过程稿（抢救重打） | 252 MB |
| `ext_yb_tree_20261022.tar.gz` | 外部仓库快照 | 208 MB |
| `ext_gitee_main_tree_20261022.tar.gz` | 外部仓库快照 | 207 MB |
| `dzrobot_v2_tmp_session_20261022.tar.gz` | 车端仓过程稿 | 35 MB |
| `ext_carsnap_tree_20261022.tar.gz` | 车端文档快照树 | 16 MB |

## 3. 门禁

- `python docs/_工具/gen_index.py` → 70 files
- `python dzrobot_v2/tools/wf/check_links.py` → 80 files, **0 broken**
- `python tools/wf/check.py` → **GREEN**（唯一 WARN 是 `tracker.py` 冻结登记失效，属旧账）

## 4. 要你决定的（详见 decision_list.csv）

1. **D08 丢失 8 篇过程稿**：`agent_overview_20261020`×4、`aim_audit`、`audit_20261018`、`cleanup_20261018`、`d13_verify` 各 1 —— 是否要按现有库内容重建摘要？
2. **D09 `link_status.md`**：只有旧版（3194 B），原 3267 B 版本随外部删除丢失。接受旧版，还是找现场重新导出？
3. **D10 `_archive` 114 篇**：留作追溯还是真删（清单可保留）。
4. **D11 靶速口径**：**已定（10-22）** 靶速 = 1.0 m/s，0.04 仅注释残留、不参与计算（决策台账 D-031，`prompt_靶速口径确认.md` 已封）；改正注释另开 `TRACK-CFG-TARGETSPEED-20261022` 待排期。
5. **D03 快照保留期**：1.47 GB，是否按季度清。
6. **D07 事件**：17:33 与 17:50 两批删除的发起者未查明（见 [incident_20261022_外部删除.md](incident_20261022_外部删除.md)）；建议加每日 git 快照。

## 5. 复跑命令

```powershell
python docs\_工具\cleanup_inventory.py     # 重建全量清单与三分类
python docs\_工具\cleanup_audit_ab.py       # A/B 删除对账
python docs\_工具\verify_b_in_tar.py        # B 保留份在 tar 内验证
python docs\_工具\verify_tars2.py           # 7 个 tar 完整性
```
## 教训：去重保留份怎么选（2026-10-22 事故后补）

_tmp_session/aim_audit/体检报告_20261018_1741.md 与 yb_tree/.../车上实测_20261018/ 下同名同哈希两份，去重时保留了 _tmp_session 那份、删了 yb_tree 那份；随后 _tmp_session 那份被外部进程删除，而 yb_tree 已在快照包内 —— 保留份选错，导致一篇唯一内容归零。

**规则**：去重留一份时，优先留「在 tracked 目录或已被 tar 收纳」的那份，不要留 gitignored、无备份的 _tmp_session 副本。判断依据：git ls-files 能否列出、docs/_工具/verify_b_in_tar.py 能否在包内找到。

## 附：归档重组（2026-10-22 晚 · 用户拍板后执行）

- 用户决定：`link_status.md` 接受旧版；旧文留存；快照全留可瘦身；全部留存件集中到 `docs/history_cache/` 并建索引。
- 执行：`docs/_archive/` → `docs/history_cache/`，分三类（旧文档 118 个 / 仓库快照 4 包 / 过程稿快照 3 包），各建 `00_索引.md` + `00_清单.csv`；总 1.33 GB（原 1.47 GB），剔 `tmp_pack` 残包 143.8 MB。
- 门禁：`check_links.py` 81 文件 0 断链；`check.py` GREEN（仅 tracker.py 登记失效 WARN，旧账）。
- 索引：`docs/README.md` 增「历史留存」入口；`gen_index.py` / `check_links.py` 跳过 `history_cache`（历史原文内部回链属预期失效）。
