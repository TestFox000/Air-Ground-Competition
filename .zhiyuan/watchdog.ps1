# 纸鸢心跳守护 v2
# 由任务计划「纸鸢心跳守护」每 5 分钟执行一次。
# 职责：检测 DeepSeek Harness 会话是否假死/崩溃；假死时重装目标并发送"继续"唤醒纸鸢；进程死了则重启应用。
# 约定：纸鸢每轮开工第一件事 = 触摸 heartbeat.txt（写时间戳）。
# 暂停：在 .zhiyuan\ 下放一个名为 PAUSE 的文件，守护立即休眠；删掉 PAUSE 恢复。
#
# v2 新增「重装目标（rearm）」，补的是 v1 的一个真窟窿：
#   dsh-goal 的 agent/session-start 处理函数每次会话启动都强制 activation='disarmed'
#   （dsh-goal\index.js:172-173），而唯一重新武装的路径是 ctx.goals.resume()
#   （index.js:276，命令面 = `/goal resume`）。
#   所以进程真死过一次、被重启之后：v1 只敲一句「继续」=> 只换来一轮，之后
#   自动续跑（第 1 层心跳）永久失效，团队没人带。
#   现在：每次恢复都先送 `/goal resume` 再送「继续」。
#   为什么无条件送、而不是只在重启后送：disarm 不止 session-start 一条路
#   （agent/error flush 失败也会 disarm），而无条件送时"目标已武装"只换回一条
#   不发模型的命令错误，零 token 代价 —— 更稳，且省掉一份状态。
#
# ⚠ 本文件必须存为 UTF-8 **带 BOM**：任务计划用 powershell.exe（5.1）执行；
#   无 BOM 时 PS5.1 按 GBK 解码，`继续` 会变成乱码 `缁х画` 敲进 GUI（已实测）。
$ErrorActionPreference = 'Continue'
$homeDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$log = Join-Path $homeDir 'watchdog.log'
$hb  = Join-Path $homeDir 'heartbeat.txt'
$st  = Join-Path $homeDir 'state.json'
$pause = Join-Path $homeDir 'PAUSE'
$staleMin = 20        # 心跳超过 20 分钟未刷新 => 会话假死
$cooldownMin = 10     # 两次恢复动作最小间隔（分钟）
$exe = 'C:\Users\TestFox\AppData\Local\Programs\DeepSeek Harness\DeepSeek Harness.exe'
# 本工作区的会话日志目录（DSH 按 cwd 哈希分目录）
$sessDir = 'C:\Users\TestFox\.dsh\sessions\--D-User-Desktop-~6BD4~8D5B~5B58~6863-v1--'

function Log($m) { "$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') $m" | Add-Content -Path $log -Encoding UTF8 }

if (Test-Path $pause) { exit }   # 霍斯说停，就停

$state = @{ last_recovery = '1970-01-01T00:00:00'; recoveries = 0 }
if (Test-Path $st) { try { $state = Get-Content $st -Raw | ConvertFrom-Json } catch {} }

# 1) GUI 活着吗（端口 19387）
$alive = $false
try {
    $c = Get-NetTCPConnection -LocalPort 19387 -State Listen -ErrorAction Stop | Select-Object -First 1
    if ($c) { $alive = $true }
} catch {}
if (-not $alive) {
    Log 'PORT_DEAD(19387) — relaunching DeepSeek Harness'
    if (Test-Path $exe) { Start-Process -FilePath $exe; Log 'RELAUNCH issued' } else { Log 'EXE_MISSING' }
    exit
}

# 2) 心跳新鲜度（双源：会话日志 mtime 优先，heartbeat.txt 兜底）
#    v1 只看 heartbeat.txt => 长工具调用（>20min）会被误判假死，往 GUI 乱敲「继续」。
#    会话日志在每轮对话都会被 DSH 刷新，是真正的存活信号。
if (-not (Test-Path $hb)) { '1970-01-01 00:00:00 init' | Set-Content -Path $hb }
$hbAge = ((Get-Date) - (Get-Item $hb).LastWriteTime).TotalMinutes
$logAge = [double]::MaxValue
$logs = Get-ChildItem $sessDir -Recurse -File -Filter 'session.v4.jsonl.zstd' -ErrorAction SilentlyContinue
if ($logs) {
    $newest = $logs | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    $logAge = ((Get-Date) - $newest.LastWriteTime).TotalMinutes
}
$ageMin = [Math]::Min($logAge, $hbAge)
if ($ageMin -lt $staleMin) { exit }   # 任一源新鲜 => 活着，不动

# 3) 冷却（避免刷屏）
$since = ((Get-Date) - [datetime]$state.last_recovery).TotalMinutes
if ($since -lt $cooldownMin) { Log "STALE $([int]$ageMin)min but cooldown $([int]$since)min"; exit }

Log "STALE $([int]$ageMin)min — recovery attempt #$([int]$state.recoveries + 1)"
$sent = $false
$proc = Get-Process -Name 'DeepSeek Harness' -ErrorAction SilentlyContinue | Select-Object -First 1
if ($proc) {
    $ws = New-Object -ComObject WScript.Shell
    $ok = $ws.AppActivate([int]$proc.Id)
    Start-Sleep -Milliseconds 1200
    if ($ok) {
        # 无条件重装目标：session-start 与 agent/error 都会 disarm 自动续跑，
        # 不 resume 就只剩这一轮，团队断带。已武装时仅回一条命令错误，零 token。
        $ws.SendKeys('/goal resume')
        Start-Sleep -Milliseconds 400
        $ws.SendKeys('{ENTER}')
        Start-Sleep -Milliseconds 800
        Log 'REARM /goal resume sent'
        $ws.SendKeys('继续')
        Start-Sleep -Milliseconds 400
        $ws.SendKeys('{ENTER}')
        Log 'RECOVERY SendKeys(继续) sent'
        $sent = $true
    } else {
        Log 'RECOVERY UI unavailable (window not activatable — 屏幕可能已锁定)'
    }
}
if ($sent) {
    $state.last_recovery = (Get-Date).ToString('o')
    $state.recoveries = [int]$state.recoveries + 1
    $state | ConvertTo-Json | Set-Content -Path $st
} else {
    Log 'RECOVERY_FAILED no channel'
}
