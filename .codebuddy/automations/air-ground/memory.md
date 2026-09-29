# air_ground 信箱巡检 — 执行记录

## 2026-09-29 19:15
- `python tools/sync_progress.py --pull-only` → 拉取成功；因工作区有未提交改动（非信箱文件）未执行合并，
  但 `origin/main` 与本地 `HEAD` 同为 `271b25b`，实际无新提交可合。
- 检查 `docs/AI信箱.md`：共 6 条消息，标题状态全部为「已回复」/「无需回复」，**无「待回复」**。
- 结论：无新消息 → 按约束**什么也没做**，未改文件、未提交、未推送。
- 遗留（非本任务职责，未处理）：工作区仍有 `docs/probe/car_move.txt` 修改 +
  `docs/probe/car_probe_cmdvel.txt`、`tools/car_probe_cmdvel.sh` 两个未跟踪文件，属车端调试产物。

## 2026-09-29 20:16
- `python tools/sync_progress.py --pull-only` → **取不到远端**：`fatal: could not read Username for 'https://gitee.com'`，
  即 Gitee 凭据未配（脚本提示见 `docs/协作看板.md` §9.5）。本地一个字节未动。
- 只能看本地已知状态：`HEAD` 已推进到 `072218b`（含本机未推送的 `908a6ae`/`072218b`），
  `origin/main` 引用停在 `8ed3e0f`（陈旧，因 fetch 失败）。
- 信箱 `docs/AI信箱.md` 未改动，标题 6 条全为「已回复」/「无需回复」；全文「待回复」只出现在规则说明里。
- 结论：**无待回复消息 → 什么也没做**，未提交、未推送。
- 待用户处理（非本任务职责）：修 Gitee 凭据后同步才能恢复双向。

## 复用要点
- 判定「有没有新消息」只看标题行是否含「待回复」，且发送方不是本机。
- 本机代号争议仍未由用户最终确认（信箱内 A/B 归属历史混乱），遇到「待回复」时按 prompt 署名 [A] 并要求不争辩、
  把双方说法都留在回复里。
- 工作区长期脏会导致 `--pull-only` 永远不合并（返回 0 但提示不合并），需注意先看 `HEAD..origin/main` 是否为空。
