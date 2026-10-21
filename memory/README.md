# 大肥鱼的本地长期记忆（dzmem）

单文件 SQLite，零依赖。四个表：`facts`（事实/结论）、`events`（时间线）、`preferences`（偏好）、`people`（人）。

## 常用命令

```powershell
cd memory
python dzmem.py add "T3 审计结论 NO-GO" --topic t3 --tag audit
python dzmem.py event "22:53 仿真台 v2 完成" --kind milestone
python dzmem.py diary "今晚把云端的证据格地图搬回来了…"
python dzmem.py pref "报告风格" "中文、结论先行、表格 + 证据"
python dzmem.py person "用户" --relation "伙伴" --notes "喜欢大肥鱼"
python dzmem.py search 仿真台
python dzmem.py today                 # 近 1 天摘要
python dzmem.py digest --days 2 --out digest.md
python dzmem.py export backup.json    # 全量备份
python dzmem.py stats
```

## 自测

```powershell
cd memory
python -m unittest discover -s tests -p "test_dzmem.py" -v   # 5 条
```

## 数据

- `dzmem.db`：SQLite 单文件（可直接拷走/备份）
- `seed.py`：把项目现状 + 今夜时间线写进去（幂等：先清空再写）
- `digest_20261018.md`：今晚生成的摘要样例

## 以后怎么用

每次开新会话/新一天，先跑 `python dzmem.py today`，把最近的事实、事件、偏好读回来——
这样跨会话也不会把项目状态、你的偏好、答应过的事忘掉。