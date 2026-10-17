$results = @()
function Try-Recovery([bool]$rearmNeeded) {
    $script:keys = @()
    $ws = New-Object PSObject -Property @{ }
    $ws | Add-Member -MemberType ScriptMethod -Name SendKeys -Value { param($k) $script:keys += $k }
    $state = @{ rearm_needed = $rearmNeeded }
    if ($state.rearm_needed) {
        $ws.SendKeys('/goal resume'); $ws.SendKeys('{ENTER}'); $state.rearm_needed = $false
    }
    $ws.SendKeys('继续'); $ws.SendKeys('{ENTER}')
    [pscustomobject]@{ rearm = $rearmNeeded; keys = ($script:keys -join ' | '); cleared = (-not $state.rearm_needed) }
}
$results += Try-Recovery $true
$results += Try-Recovery $false
$r1 = $results[0]; $r2 = $results[1]
"重启后恢复 -> 按键序列: $($r1.keys)"
"非重启恢复 -> 按键序列: $($r2.keys)"
"重启分支已清标记: $($r1.cleared)"
$ok = ($r1.keys -eq '/goal resume | {ENTER} | 继续 | {ENTER}') -and ($r2.keys -eq '继续 | {ENTER}') -and $r1.cleared
if ($ok) { "结果：全部通过" ; exit 0 } else { "结果：失败"; exit 1 }
