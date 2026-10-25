---
ns: doc-1024
agent: doc-archivist-ext
树节点: DOC-2
写域: docs/20_资料索引库/外部资料/（6 篇 + 00_索引.md）
更新: 2026-10-24
status: in_progress
---

# DOC-P1A-REFINE-EXT-20261024 · 外部资料 6 篇

## 1 五必填

| 项 | 内容 |
|---|---|
| 读过的台账条目 | D-027（车体几何冲突）、D-029（PIVOT 形态）、D-031（靶速封死 1.0 m/s）、D-033（靶速改判 0.3 m/s）；AGENTS §8.2（存入三步）、§8.3（禁止项） |
| 复用检索结果 | 车端/09_外部实现借鉴_air_ground_20261018.md 已收 hjy 部分 → 本卡只补它没收的（wtt、yb 精度/终点线/接口契约、官方 SDK），不重复写 |
| 新增轮子及理由 | 无新增脚本。复用 doc_search.py 检索、gen_index.py --ns 登记 |
| 验收命令 | python docs/_工具/doc_search.py "TargetAimer" → 命中 外部资料/01；python docs/_工具/check_doc_policy.py → exit 0 |
| 回滚点 | 6 篇 + 索引都是新文件，删除即回滚；不碰 车端/09 |

## 2 任务

产出 docs/20_资料索引库/外部资料/ 6 篇 + 00_索引.md：

1. 01_成员交接_hjy_自瞄设计.md —— TargetAimer 三层拆解、α/β 判据、380 ms 延迟推导、own_gimbal=false 的实证缺口
2. 02_成员wtt_实体训练.md —— 库内当前命中 0，从 ext_yb_tree 提炼
3. 03_yb_精度对照_0x4002.md
4. 04_yb_终点线分析.md
5. 05_yb接口契约_vs_本地协议v2.md —— 并补一行指向 空地链路/01 做对照
6. 06_官方SDK与手册索引.md —— 把 车端/10 与 赛事与治理/07 的官方部分收拢成索引

每篇必须：先通读原文 → 写一句话结论 + 关键数字 + 来源 路径:行 → 标 取证: 实/档/未。

## 3 执行记录（追加）

- 通读原文：ext_yb_tree_20261022.tar.gz 内 hu-jinyu 交接文档（1062 行）+ dzmission-doc 7 份 + wtt entity_train 驯化指南 + 0x4002 精度方案 + finish_line_analysis + interface_contract。
- 产出 6 篇 + 00_索引.md（行数：01=90 / 02=101 / 03=79 / 04=115 / 05=87 / 06=93 / 索引=68）。front matter status 全为 现行。
- 01 不重复写车端/09 已收的 hjy 部分，只补自瞄细节（三层拆解 / αβγ 判据 / 380 ms / own_gimbal=false）。
- 05 补一行指向 空地链路/01 做本地对照；06 收拢 车端/10 与 赛事与治理/07 官方部分成索引。
- 门禁：gen_index --only 外部资料/ exit 0；check_doc_policy exit 0（R3=0；R6 已随本册同步 49→60）；check_anchors 本册新增红 0；check_links 188 文件 broken 0。
- 复核：`python docs/_工具/doc_search.py "TargetAimer"` 首位命中 外部资料/01（分 10）。
- 过程修订：首稿 01-05 曾标 status: 历史，按 captain 指示改为 现行；临时解压的 _tmp_session/t2/src 已删除（原文仅存 tar 内，引用标〔未入库〕）。

## 4 结论判定

## 10 文档变更

新增：docs/20_资料索引库/外部资料/ 6 篇 + 00_索引.md
修改：docs/20_资料索引库/README.md 篇数口径行（49→60，R6 守卫要求）；外部资料 7 文件登记进 目录索引.json（gen_index --only）。车端/09 与 赛事与治理/07 未改，只在本册反向指针。
归档：无
