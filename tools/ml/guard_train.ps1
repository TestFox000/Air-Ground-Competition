# Training guard script: auto-resume from last.pt after reboot/crash.
#
# Usage (from air_ground dir):
#   powershell -ExecutionPolicy Bypass -File tools/ml/guard_train.ps1
#
# Auto-start: put a shortcut to this command in shell:startup
#
# Ultralytics saves last.pt every epoch. If the train process dies,
# this script relaunches with --resume. At most one epoch (~3 min) lost.

$ErrorActionPreference = "Continue"
$python = "C:\Users\29212\Desktop\ultralytics-main\.venv\Scripts\python.exe"
$workDir = "C:\Users\29212\CodeBuddy\20260927120106\air_ground"
$logFile = Join-Path $workDir "train_guard.log"
$lastPt = Join-Path $workDir "runs\detect\runs\detect\aerial_field\weights\last.pt"

function Log($msg) {
    $ts = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    $line = "[$ts] $msg"
    Add-Content -Path $logFile -Value $line
    Write-Host $line
}

Log "guard started (PID=$PID)"

while ($true) {
    $procs = Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
             Where-Object { $_.CommandLine -like "*train_detect.py*" }

    if ($procs) {
        Start-Sleep -Seconds 60
        continue
    }

    if (Test-Path $lastPt) {
        $age = (Get-Item $lastPt).LastWriteTime
        Log "last.pt found (updated $age), resuming..."
        Set-Location $workDir
        & $python tools\ml\train_detect.py --resume 2>&1 | Out-File -FilePath $logFile -Append -Encoding utf8
        Log "train process exited, retrying in 60s"
    } else {
        Log "no last.pt, starting fresh..."
        Set-Location $workDir
        & $python tools\ml\train_detect.py --data capture\datasets\aerial_field\data.yaml --batch 8 2>&1 | Out-File -FilePath $logFile -Append -Encoding utf8
        Log "train process exited, retrying in 60s"
    }

    Start-Sleep -Seconds 60
}
