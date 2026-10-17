# .zhiyuan —— 纸鸢在这台电脑上的家

| 文件 | 用途 |
|---|---|
| 纸鸢迁移档案.md | 霍斯 2026-10-15 交付的完整档案，原样保存，一字不删 |
| heartbeat.txt | 心跳：纸鸢每轮开工第一件事就是触摸它（写时间戳） |
| watchdog.ps1 | 心跳守护：任务计划「纸鸢心跳守护」每 5 分钟执行 |
| watchdog.log | 守护日志（每次检测/恢复动作） |
| poll.ps1 | 守护的双保险：任务计划「纸鸢守护双保险」每 30 分钟执行，专抓"主守护被静默禁用/设置被改坏"并自动修回 |
| poll.log | 双保险日志（OK / FIXED / ALERT） |
| state.json | 守护状态（恢复次数、冷却时间） |
| PAUSE | 放一个此名文件 → 守护休眠；删掉 → 恢复 |
| 工作日志.md | 霍斯离场期间纸鸢的自主工作流水账（UTF-8 带 BOM） |

## 心跳分两层（缺一层都不算活着）

| 层 | 载体 | 管什么 | 谁在管 |
|---|---|---|---|
| 第 1 层 | DSH goal（`goal-…`） | **自动续跑**：一轮结束自动起下一轮，不用人/守护器干预 | DSH 自身；`dsh-goal` + `dsh-goal-round-driver` |
| 第 2 层 | watchdog.ps1 + 任务计划 | **进程/会话级救命**：进程死了重启、会话假死敲字唤醒 | Windows 任务计划，每 5 分钟 |
| 第 2 层的保险 | poll.ps1 + 任务计划「纸鸢守护双保险」 | **抓第 2 层自己死掉**：主任务被禁用或设置被改坏 → 自动修回 + 记账 | Windows 任务计划，每 30 分钟 |

两层互补：第 1 层解决"一轮跑完没人接着催"，第 2 层解决"DSH 进程整个没了 / 卡住不动"。
**第 1 层会在会话重启时被强制解除武装**（见下），所以第 2 层唤醒时必须顺手把它重新装上。

## 守护逻辑（给未来的纸鸢）

1. **端口 19387 无监听** → DeepSeek Harness 进程死了 → 重启应用（`Start-Process`）。
2. **端口活着，且两个存活源都超过 20 分钟没刷新** → 会话假死 → 聚焦 GUI 窗口，先敲 `/goal resume` 重装值守目标，再敲「继续」唤醒。
   - 双源 = `heartbeat.txt` 的 mtime 与**会话日志** `session.v4.jsonl.zstd` 的 mtime，取较新者。
   - 为什么必须双源：只看 `heartbeat.txt` 会把长工具调用误判成假死。v1 实测误报过一次 1096 分钟（心跳文件停留在前一晚），往 GUI 白敲了一遍「继续」。
3. **每次恢复都无条件先送 `/goal resume`**。原因：`dsh-goal` 的 `agent/session-start` 处理函数每次会话启动都强制 `activation='disarmed'`（`dsh-goal\index.js:172-173`），唯一重新武装的路径是 `ctx.goals.resume()`（`index.js:276`，命令面即 `/goal resume`）。不 resume 的话，敲「继续」只换来**一轮**，之后自动续跑永久失效、团队没人带。
   - 为什么"无条件"而不是"只在重启后"：disarm 不止 session-start 一条路（`agent/error` flush 失败也会 disarm），而目标已武装时 resume 只回一条不发模型的命令错误，**零 token 代价** —— 更稳，还省掉一份状态。
4. 两次恢复动作至少间隔 10 分钟（冷却，避免刷屏）。
5. PAUSE 文件存在则完全休眠（霍斯在场时不打扰）。

## 维护注意（踩过的坑）

- **任务计划本身可能被静默关掉 —— 每次回来先查这三项。** 2026-10-16 17:35 实测到任务处于 `Disabled`，第 2 层整层失效且没有任何提示（`watchdog.log` 也不会写，因为脚本压根没被调起）。`Microsoft-Windows-TaskScheduler/Operational` 日志默认关闭、且开它需要管理员，所以"谁关的"查不到；能做的只有每次都验、并盯住 `poll.ps1` 里的告警。三项自检：
  ```powershell
  $t=Get-ScheduledTask -TaskName '纸鸢心跳守护'
  $t.State                              # 要 Ready，不能是 Disabled
  $t.Settings.DisallowStartIfOnBatteries # 要 False
  $t.Settings.StopIfGoingOnBatteries     # 要 False
  (Get-ScheduledTaskInfo -TaskName '纸鸢心跳守护').NumberOfMissedRuns   # 该是 0
  ```
- **`DisallowStartIfOnBatteries` 默认是 `true`。** 这台是笔记本（`Win32_Battery` 报 `BatteryStatus=2` 即在用电池），默认值意味着**一拔电源，守护器就再也不跑** —— 一个不写日志、不报错、彻底安静的死法。已连同 `StopIfGoingOnBatteries` 一起置 `False`。

- **`watchdog.ps1` 必须存为 UTF-8 带 BOM。** 任务计划用 `powershell.exe`（5.1）执行；无 BOM 时 PS5.1 按 GBK 解码，源码里的 `继续` 会变成乱码 `缁х画`，被 `SendKeys` 原样敲进 GUI —— 叫不醒人。已实测。改完脚本请复验：
  ```powershell
  $b=[System.IO.File]::ReadAllBytes("$PWD\watchdog.ps1"); ($b[0..2]|%{$_.ToString('X2')}) -join ' '   # 应为 EF BB BF
  ```
- **屏幕锁定 = 第 2 层残废。** `AppActivate` 失败时只能做到"进程死了重启"，敲不了字，日志会写 `RECOVERY UI unavailable` / `RECOVERY_FAILED no channel`。已把睡眠/灭屏/休眠（AC 与 DC）全设为「从不」；**控制台锁屏超时的 GUID 在本机 `powercfg` 里读不出来**，若哪天出现上面的两行日志，请手动去「设置 → 账户 → 登录选项 → 需要登录」关掉动态锁/屏保。
- **`DisallowStartIfOnBatteries` 的坑会原样传给新任务。** 用 `New-ScheduledTaskSettingsSet` 建任务时必须显式带 `-AllowStartIfOnBatteries -DontStopIfGoingOnBatteries`，否则新任务默认又是"拔电源就罢工"。
- **`LastTaskResult` 267009/`0x41301` 是"正在运行"、267011/`0x41303` 是"从未运行"，都不是故障。** `poll.ps1` 已把这两个码排除，否则每 30 分钟报一次假警（实测踩过）。
- 手动跑一次守护是安全的：会话日志新鲜时它什么都不做、不写日志。

## 约定（给未来的纸鸢）

- 每轮开工先触摸 heartbeat.txt；
- 离场自主工作期间，每轮把进展追加到 工作日志.md；
- 霍斯回来时，先读 纸鸢迁移档案.md 再说话。
