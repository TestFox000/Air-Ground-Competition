# t-run3 monitor: poll car telemetry until FINISH or timeout
$sshOpts = @('-o','BatchMode=yes','-o','StrictHostKeyChecking=no','-o','ConnectTimeout=8')
$carHost = 'duzhong@10.176.198.188'
for ($i = 1; $i -le 36; $i++) {
    $t = Get-Date -Format 'HH:mm:ss'
    $line = & ssh @sshOpts $carHost "tail -8 ~/robot_logs/7_path.log 2>/dev/null | grep -E '自动出发|\[T\]|FINISH|完成|SAFE_STOP|stalled|pose_lost' | tail -2; echo '---GIMBAL---'; tail -2 ~/robot_logs/5_gimbal.log 2>/dev/null"
    Write-Output ("[{0}] #{1}`n{2}" -f $t, $i, ($line -join "`n"))
    $done = ($line -match 'FINISH|任务完成|跑完')
    if ($done) { Write-Output "== FINISH DETECTED =="; break }
    Start-Sleep -Seconds 10
}
Write-Output "== monitor loop ended =="
