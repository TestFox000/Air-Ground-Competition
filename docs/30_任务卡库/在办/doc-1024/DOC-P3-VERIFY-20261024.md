---
ns: doc-1024
agent: lead
树节点: DOC-7
写域: docs/20_资料索引库/README.md, docs/00_项目总览/编年/00_总纲.md, docs/目录索引.md
更新: 2026-10-24
status: pending
---

# DOC-P3-VERIFY-20261024 · 6 条门禁 + R6 篇数 + 主题矩阵

## 1 五必填

| 项 | 内容 |
|---|---|
| 读过的台账条目 | D-023（回归口径）、D-032（git_save）；AGENTS §4（收口门禁）、§8.4（三层索引）、§8.6（守卫顺序）、§8.9（指标） |
| 复用检索结果 | 复用 6 条现有守卫，不新建任何校验脚本 |
| 新增轮子及理由 | 无 |
| 验收命令 | 6 条门禁全部 exit 0；python docs/_工具/check_doc_policy.py 红线数 = 0；README 篇数 = ls 实扫篇数 |
| 回滚点 | 只改 README / 总纲 / 目录索引三个文件，git checkout 即可 |

## 2 任务

1. 跑完整门禁链并把退出码写进 §3：
   gen_index.py --ns doc-1024 → check_doc_policy.py → check_anchors.py → check_numbers.py → dzrobot_v2/tools/wf/check.py → check_links.py
2. 更新 docs/20_资料索引库/README.md：
   - §1 表格加 外部资料 6 篇、运维与排障 4 篇两行
   - §1 表格 赛事与治理 11 → 11（本卡不改）
   - 篇数口径行：48 → 59
   - §4 症状索引加一行指针到 运维与排障/01
3. 写主题矩阵（README §3 之后新增一节）：硬件/物料、参数真源、故障排障、外部实现、对外交付 五行横切索引
4. 编年/00_总纲.md 收口（若 DOC-4 未写完则本卡标 ⏸）
5. 指标写进 docs/TEAM_STATUS.md：库内篇数 / 库外受管篇数 / 红线数 / 过期候选数 / 断链数

## 3 执行记录（追加）

## 4 结论判定

## 10 文档变更

新增：无
修改：docs/20_资料索引库/README.md（篇数 + 两行 + 主题矩阵 + 指针）、docs/目录索引.md（gen_index 重生成）、docs/TEAM_STATUS.md（指标）
归档：无
