# dzcam.ps1 - one-command camera / VNC helper for the drone (Windows side)
#
# Usage:
#   .\dzcam.ps1 ip                        find a reachable drone IP
#   .\dzcam.ps1 status                    preview + recording state
#   .\dzcam.ps1 preview on|off|status     live window on VNC :1
#   .\dzcam.ps1 record start|stop|status  record to ~/Videos/recordings
#   .\dzcam.ps1 shot [out.png]            screenshot of VNC desktop -> PC
#   .\dzcam.ps1 snap [out.jpg]            single camera frame -> PC (camera must be free)
#
# Options: -Ip 192.168.x.x   (skip discovery)
# Notes:
#   * dzsmart.local may resolve to a STALE IP; prefer -Ip or "ip" discovery.
#   * preview and record both need /dev/video0 exclusively.
#   * only writes on the drone: /tmp/xwd2png.py (uploaded once), /tmp/dzcam_*.png
param(
    [Parameter(Mandatory=$true)][string]$Action,
    [string]$Arg1 = "",
    [string]$Ip = ""
)

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$SSHOPT = @("-o","BatchMode=yes","-o","ConnectTimeout=8")
$XAUTH = "/home/duzhong/.Xauthority"

function Find-Drone {
    if ($Ip) { return $Ip }
    $cands = @()
    try {
        $arp = arp -a 2>$null | Select-String -Pattern "(\d+\.\d+\.\d+\.\d+)\s+([0-9a-f\-]{17})" -AllMatches
        foreach ($m in $arp.Matches) { $cands += $m.Groups[1].Value }
    } catch {}
    $cands = $cands | Where-Object { $_ -notlike "*.255" -and $_ -notlike "*.1" } | Select-Object -Unique
    foreach ($c in $cands) {
        $r = ssh @SSHOPT "duzhong@$c" "hostname" 2>$null
        if ($r -and $r.Trim() -eq "dzsmart") { return $c }
    }
    throw "drone not found on this network. Pass -Ip <addr> (see: arp -a)"
}

function Invoke-Remote([string]$cmd) {
    $r = ssh @SSHOPT "duzhong@$script:droneIp" $cmd
    return ($r -join "`n")
}

function Send-Script([string]$file, [string]$argsLine) {
    $body = Get-Content -Raw (Join-Path $here $file)
    $r = $body | ssh @SSHOPT "duzhong@$script:droneIp" "bash -s -- $argsLine"
    return ($r -join "`n")
}

$script:droneIp = Find-Drone
Write-Host "[drone] $script:droneIp" -ForegroundColor DarkGray

switch ($Action.ToLower()) {
    "ip" { Write-Output $script:droneIp }

    "status" { Write-Output (Send-Script "cam_preview_vnc.sh" "--status") }

    "preview" {
        switch ($Arg1) {
            "on"     { Write-Output (Send-Script "cam_preview_vnc.sh" "") }
            "off"    { Write-Output (Send-Script "cam_preview_vnc.sh" "--stop") }
            "status" { Write-Output (Send-Script "cam_preview_vnc.sh" "--status") }
            default  { throw "usage: dzcam.ps1 preview on|off|status" }
        }
    }

    "record" {
        switch ($Arg1) {
            "start"  { Write-Output (Send-Script "cam_record.sh" "start") }
            "stop"   { Write-Output (Send-Script "cam_record.sh" "stop") }
            "status" { Write-Output (Send-Script "cam_record.sh" "status") }
            default  { throw "usage: dzcam.ps1 record start|stop|status" }
        }
    }

    "shot" {
        $out = if ($Arg1) { $Arg1 } else { Join-Path $here ("shot_{0}.png" -f (Get-Date -Format "yyyyMMdd_HHmmss")) }
        $stamp = "dzcam_{0}" -f (Get-Date -Format "HHmmss")
        $xwd = "/tmp/$stamp.xwd"; $png = "/tmp/$stamp.png"
        $has = Invoke-Remote "test -f /tmp/xwd2png.py && echo yes || echo no"
        if ($has.Trim() -ne "yes") {
            scp @SSHOPT (Join-Path $here "xwd2png.py") "duzhong@$($script:droneIp):/tmp/xwd2png.py" | Out-Null
        }
        $r = Invoke-Remote "DISPLAY=:1 XAUTHORITY=$XAUTH xwd -root -silent -out $xwd && python3 /tmp/xwd2png.py $xwd $png 1600 && ls -l $png"
        if ($LASTEXITCODE -ne 0) { throw "screenshot failed: $r" }
        scp @SSHOPT "duzhong@$($script:droneIp):$png" $out | Out-Null
        Write-Output "saved: $out"
    }

    "snap" {
        $out = if ($Arg1) { $Arg1 } else { Join-Path $here ("snap_{0}.jpg" -f (Get-Date -Format "yyyyMMdd_HHmmss")) }
        $stamp = "dzcam_{0}" -f (Get-Date -Format "HHmmss")
        $png = "/tmp/$stamp.png"
        $r = Invoke-Remote "DISPLAY=:1 XAUTHORITY=$XAUTH timeout 20 gst-launch-1.0 -q v4l2src device=/dev/video0 num-buffers=1 ! image/jpeg,width=1920,height=1080,framerate=30/1 ! jpegdec ! videoconvert ! pngenc ! filesink location=$png && ls -l $png"
        if ($LASTEXITCODE -ne 0) { throw "snap failed (camera busy? stop preview/record first): $r" }
        scp @SSHOPT "duzhong@$($script:droneIp):$png" $out | Out-Null
        Write-Output "saved: $out"
    }

    default { throw "unknown action: $Action" }
}
