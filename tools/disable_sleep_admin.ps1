# 需以管理员身份运行：禁用 Modern Standby + 网络适配器防睡眠 + 执行请求覆盖
# 运行后需重启生效
$ErrorActionPreference = 'Continue'
$sid='0a2861a1-4371-423a-8e3b-257c5fb75310'

Write-Host "=== 1. 禁用 Modern Standby (S0 连接待机) ===" -ForegroundColor Cyan
$powerKey='HKLM:\SYSTEM\CurrentControlSet\Control\Power'
Set-ItemProperty -Path $powerKey -Name 'PlatformAoAcOverride' -Value 0 -Type DWord -Force
Set-ItemProperty -Path $powerKey -Name 'CsEnabled' -Value 0 -Type DWord -Force
Write-Host "  PlatformAoAcOverride=0, CsEnabled=0 已写入" -ForegroundColor Green

Write-Host "=== 2. 禁用网络适配器电源管理 (允许计算机关闭此设备) ===" -ForegroundColor Cyan
Get-NetAdapter -Physical | ForEach-Object {
    $nic = $_
    try {
        $pnp = Get-PnpDevice -InstanceId $nic.InstanceID -ErrorAction Stop
        $power = Get-PnpDeviceProperty -InstanceId $nic.InstanceID -KeyName 'DEVPKEY_Device_PowerData' -ErrorAction SilentlyContinue
        # 用注册表禁用选择性挂起
        $regPath = "HKLM:\SYSTEM\CurrentControlSet\Control\Class\{4d36e972-e325-11ce-bfc1-08002be10318}\0000"
        if (Test-Path $regPath) {
            Set-ItemProperty -Path $regPath -Name 'SelectiveSuspend' -Value 0 -Type DWord -Force -ErrorAction SilentlyContinue
        }
        # WMI 方式禁用电源管理
        $nicWmi = Get-WmiObject -Class MSPower_DeviceEnable -Namespace root\wmi -ErrorAction SilentlyContinue | Where-Object { $_.InstanceName -like "*$($nic.InterfaceDescription)*" }
        if ($nicWmi) { $nicWmi.Enable = $false; $nicWmi.Put() | Out-Null }
        Write-Host "  $($nic.Name): 已尝试禁用睡眠关闭" -ForegroundColor Green
    } catch {
        Write-Host "  $($nic.Name): $($_.Exception.Message)" -ForegroundColor Yellow
    }
}

Write-Host "=== 3. 电源策略收尾 (硬盘/PCIe/屏保) ===" -ForegroundColor Cyan
powercfg /setacvalueindex $sid SUB_DISK DISKIDLE 0
powercfg /setdcvalueindex $sid SUB_DISK DISKIDLE 0
powercfg /setacvalueindex $sid SUB_PCIEXPRESS ASPM 0
powercfg /setdcvalueindex $sid SUB_PCIEXPRESS ASPM 0
powercfg /setactive $sid
Set-ItemProperty -Path 'HKCU:\Control Panel\Desktop' -Name ScreenSaveActive -Value 0 -Type String
Set-ItemProperty -Path 'HKCU:\Control Panel\Desktop' -Name ScreenSaveTimeOut -Value 0 -Type String
Write-Host "  硬盘=永不, PCIe=关闭, 屏保=禁用" -ForegroundColor Green

Write-Host "=== 4. 为 python/训练进程添加执行请求覆盖 (强制阻止睡眠) ===" -ForegroundColor Cyan
powercfg /requestsoverride PROCESS python.exe SYSTEM DISPLAY 2>&1 | Out-Null
powercfg /requestsoverride PROCESS python3.exe SYSTEM DISPLAY 2>&1 | Out-Null
Write-Host "  python.exe 已设为阻止系统睡眠和关闭显示" -ForegroundColor Green

Write-Host "`n=== 完成！请重启电脑使 Modern Standby 禁用生效 ===" -ForegroundColor Yellow
Write-Host "重启后运行: powercfg /a  确认不再显示 S0 低电量待机" -ForegroundColor Yellow
Read-Host "按回车退出"
