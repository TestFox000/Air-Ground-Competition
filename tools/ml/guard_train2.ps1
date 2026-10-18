# Improved training guard - uses Start-Process (no pipe blocking)
# Auto-resumes from last.pt when train process dies.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File tools/ml/guard_train2.ps1
#
# Auto-start on boot: shortcut in shell:startup pointing to this command.

$ErrorActionPreference = "Continue"
$python = "C:\Users\29212\Desktop\ultralytics-main\.venv\Scripts\python.exe"
$workDir = "C:\Users\29212\CodeBuddy\20260927120106\air_ground"
$logFile = Join-Path $workDir "train_guard.log"
$trainLog = Join-Path $workDir "train_stdout.log"

function Log($msg) {
    $ts = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    $line = "[$ts] $msg"
    Add-Content -Path $logFile -Value $line
    Write-Host $line
}

function Get-TrainProc {
    Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -like "*train_detect.py*" }
}

Log "guard2 started (PID=$PID)"

while ($true) {
    $proc = Get-TrainProc
    if ($proc) {
        Start-Sleep -Seconds 30
        continue
    }

    # Train not running - (re)start it
    $lastPt = Get-ChildItem (Join-Path $workDir "runs") -Recurse -Filter "last.pt" -ErrorAction SilentlyContinue |
              Sort-Object LastWriteTime -Descending | Select-Object -First 1

    Set-Location $workDir
    if ($lastPt) {
        Log "resuming from $($lastPt.FullName) (updated $($lastPt.LastWriteTime))"
        $args = @("tools\ml\train_detect.py", "--resume")
    } else {
        Log "no last.pt found, starting fresh"
        $args = @("tools\ml\train_detect.py", "--data", "capture\datasets\aerial_field\data.yaml", "--batch", "8")
    }

    $p = Start-Process -FilePath $python -ArgumentList $args `
        -WorkingDirectory $workDir `
        -RedirectStandardOutput $trainLog `
        -RedirectStandardError (Join-Path $workDir "train_stderr.log") `
        -NoNewWindow -PassThru

    Log "started train PID=$($p.Id)"

    # Wait for process to exit (poll every 30s)
    while (-not $p.HasExited) {
        Start-Sleep -Seconds 30
    }
    Log "train PID=$($p.Id) exited with code $($p.ExitCode)"
    Start-Sleep -Seconds 10
}
