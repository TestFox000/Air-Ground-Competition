# 纸鸢守护双保险 v1
# 由任务计划「纸鸢守护双保险」每 30 分钟执行。
# 存在的唯一理由：主守护（每 5 分钟的 watchdog.ps1）在 2026-10-16 被静默置为 Disabled，
# 第 2 层整层失效却毫无提示 —— 脚本压根没被调起，watchdog.log 自然也不写一行。
# 所以这里只做单点失效巡检：发现主任务被禁/设置异常 -> 自动修回 + 记账。
# ⚠ 同样必须存为 UTF-8 带 BOM（任务计划用 powershell.exe 5.1 执行，无 BOM 会变乱码）。
$dir = Split-Path -Parent $MyInvocation.MyCommand.Path
$plog = Join-Path $dir 'poll.log'
$main = '纸鸢心跳守护'
function Log($m) { "$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') $m" | Add-Content -Path $plog -Encoding UTF8 }

if (Test-Path (Join-Path $dir 'PAUSE')) { exit }   # 与主守护同一个暂停开关

$issues = @()
$state = 'MISSING'
$t = Get-ScheduledTask -TaskName $main -ErrorAction SilentlyContinue
if (-not $t) {
    $issues += '主守护任务不存在'
} else {
    $state = $t.State
    $bad = @()
    if ($t.State -eq 'Disabled' -or -not $t.Settings.Enabled) { $bad += 'Disabled' }
    if ($t.Settings.DisallowStartIfOnBatteries) { $bad += 'DisallowStartIfOnBatteries' }
    if ($t.Settings.StopIfGoingOnBatteries) { $bad += 'StopIfGoingOnBatteries' }
    if ($bad.Count) {
        $issues += "主守护设置异常: $($bad -join ',')"
        try {
            $t.Settings.Enabled = $true
            $t.Settings.DisallowStartIfOnBatteries = $false
            $t.Settings.StopIfGoingOnBatteries = $false
            Set-ScheduledTask -InputObject $t | Out-Null
            $after = (Get-ScheduledTask -TaskName $main).State
            $state = $after
            if ($after -eq 'Disabled') {
                $issues += '自动修复未生效'
            } else {
                # 修好了就不算故障：从 issues 摘掉，只留 FIXED 记账，退出码保持 0。
                # 否则 LastTaskResult 长期非 0，日后审计会把"已自愈"误读成"一直坏着"。
                $issues = $issues | Where-Object { $_ -ne "主守护设置异常: $($bad -join ',')" }
                Log "FIXED 主守护已修回（原异常: $($bad -join ',')）"
            }
        } catch { $issues += "自动修复失败: $($_.Exception.Message)" }
    }
    $info = Get-ScheduledTaskInfo -TaskName $main -ErrorAction SilentlyContinue
    # 267009/0x41301 = 正在运行；267011/0x41303 = 从未运行。都不是故障，别误报（实测踩过）。
    if ($info -and $info.LastTaskResult -ne 0 -and $info.LastTaskResult -notin 267009, 267011) {
        $issues += "主守护上次退出码 $($info.LastTaskResult)"
    }
}

# 心跳停滞时长：>6h 说明确实没人/没在跑（主守护每 5 分钟本该让日志推进）
$hb = Join-Path $dir 'heartbeat.txt'
$hbH = if (Test-Path $hb) { [int]((Get-Date) - (Get-Item $hb).LastWriteTime).TotalHours } else { -1 }

if ($issues.Count) {
    Log "ALERT $($issues -join '；') ｜ 心跳停滞 ${hbH}h"
    exit 1
}
Log "OK 主守护 $state ｜ 上次退出码 0 ｜ 心跳停滞 ${hbH}h"
exit 0
