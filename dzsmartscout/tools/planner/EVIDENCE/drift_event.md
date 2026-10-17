# 漂移事件记录（t60 · 哈希门禁第一次真实触发）

**这不是失败，是门禁按设计工作的一次实例。** 记录在此，供复盘"并发写入者"场景。

| 项 | 值 |
|---|---|
| 发现时间 | 2026-10-15 20:46（本机时钟） |
| 发现方式 | `python -m unittest discover -s dzsmartscout/tools/planner/tests` → `test_vendor_matches_source_now` **FAIL** |
| 报错原文 | `AssertionError: ['config/robot.yaml'] is not false : 真源已变（或产物过期），请重跑 sync_planner_from_source.py：['config/robot.yaml']` |
| 漂移文件 | `dzrobot_v2/config/robot.yaml` |
| 漂移性质 | **纯注释**（`Compare-Object` 命中 5 行，全部是 `#` 注释：t57 的落盘更正说明 —— F3 射击点 0.7215 m 口径、"54 → 44 点"、`path_executor.shoot_distance_*` 与 `route.shoot_distance_*` 两套语义的提示） |
| 体量变化 | 60 279 B → 60 872 B；sha256 `7b46c962a5eb4d31…` → `58298dc2a1af65bc870d…` |
| 我的处置 | 1) 先存漂移明细；2) 重跑 `sync_planner_from_source.py` 吸收新快照；3) **重跑全部验证**（对拍 + route_check + 整线干跑 + 单测） |
| 处置后结果 | 路线 token **未变**（当时口径 40 token，`...F3*...`），双侧仍 IDENTICAL，`route_check` 仍 0 error，单测 9/9 OK |

> ⚠️ 后续**收口轮**（队长实锤）发现另一处更严重的口径问题：当时的对拍**没传 `--engine`**，
> 跑的是 `tools/plan_route.py` 的默认 `tour`；而实车走 `planner.engine=tracker-safe`
> ⇒ 正确口径是 **44 token**。详见 `plan_tokens.txt` 与 `../README.md` §3。

## 为什么值得记一笔

* 本卡纪律是"**无人机侧规划器副本只能由脚本生成 + 哈希门禁**"。
  本次是它第一次在真实并发写入（队友 t57 正在改 `robot.yaml`）下触发 ——
  如果没有门禁，`vendor/` 会静静停在 20:42 的旧 cookie 上，而"跑的是哪一版"就再也说不清了。
* 结论：**并发写入者存在时，`--verify-only` 必须在每次用之前跑一次**（README §2-②）。
* 顺带说明：注释级改动不影响规划输出（本次实测 token 逐字不变），但**门禁不区分"注释/代码"**——
  这是**故意**的：判"这次改动要不要重同步"不该由脚本猜。
