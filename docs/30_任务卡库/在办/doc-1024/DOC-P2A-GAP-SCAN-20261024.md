---
ns: doc-1024
agent: doc-indexer
树节点: DOC-5
写域: artifacts/wordcloud/, docs/history_cache/99_未消化清单.md
更新: 2026-10-24
status: pending
---

# DOC-P2A-GAP-SCAN-20261024 · 热词缺口清单

## 1 五必填

| 项 | 内容 |
|---|---|
| 读过的台账条目 | 无新增裁决。AGENTS §8.9（月度审计第 2 项：抽查豁免区）、§8.10（乱拉屎） |
| 复用检索结果 | 复用 doc_search.py 做「词 → 是否已有条目」反查；复用 index_tar.py 的 tar 枚举（DOC-1） |
| 新增轮子及理由 | artifacts/wordcloud/_gap.py —— 现有工具没有分词/词频能力；且**只落 csv + png 到 artifacts，不进库**（§8.10 判词云当正文为乱拉屎） |
| 验收命令 | ls artifacts/wordcloud/ 有 csv 与 png；csv 含「库内是否已有条目」列；缺口行 ≥ 5 |
| 回滚点 | artifacts/ 是豁免区，删除即回滚 |

## 2 任务

1. 对 489 篇做分词/词频（零依赖 n-gram 或 jieba）
2. 每个热词反查 doc_search.py 在库内是否已有对应条目
3. 产出 artifacts/wordcloud/热词_2026W43.csv（词 / 频次 / 出现在哪几篇 / 库内是否已有条目）
4. 产出 热词_2026W43.png（**只给对外展示**）
5. 把 99_未消化清单.md 从骨架定稿（补上「待提炼」清单）
6. 缺口行写进 §3，作为 DOC-P1A/P1B 的补充输入（不阻塞它们）

## 3 执行记录（追加）

## 4 结论判定

## 10 文档变更

新增：artifacts/wordcloud/_gap.py、artifacts/wordcloud/热词_2026W43.csv、artifacts/wordcloud/热词_2026W43.png
修改：docs/history_cache/99_未消化清单.md（定稿）
归档：无
