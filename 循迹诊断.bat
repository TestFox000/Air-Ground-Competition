@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
title 循迹诊断 · air_ground

REM ============================================================
REM  循迹"信念 vs 真实"对照诊断 · 一键启动
REM  赛前准备只需一次：打印 docs\track_diag\markers\ 里的标记并贴好
REM  手册：docs\track_diag现场手册.md
REM ============================================================

REM ---------- 0. 依赖自检 ----------
python -c "import cv2, numpy, paramiko" 2>nul
if errorlevel 1 (
    echo.
    echo [错误] 缺 Python 依赖：cv2 / numpy / paramiko
    echo        请先执行：python -m pip install opencv-contrib-python numpy paramiko
    echo.
    pause
    exit /b 1
)

REM ---------- 1. 读取保存的小车 IP（不保存密码，安全约定） ----------
set ENV_FILE=tools\track_diag.local.env
if exist "%ENV_FILE%" (
    for /f "usebackq tokens=1,* delims==" %%a in ("%ENV_FILE%") do set "%%a=%%b"
)

REM 角点配置存在则自动带上
set CORNER_ARG=
if exist "tools\track_diag_field.json" set CORNER_ARG=--corner-config tools/track_diag_field.json

:MENU
set CHOICE=
echo.
echo ==================================================
echo   循迹诊断 · 信念 vs 真实
echo ==================================================
if defined CAR_HOST (echo   小车: %CAR_HOST%) else (echo   小车: 未设置)
echo --------------------------------------------------
echo   [1] 开始诊断（相机 + 小车，赛前选这个）
echo   [2] 只开相机自检（调机位 / 贴标记时用）
echo   [3] 只连小车自检（验 SSH / 话题）
echo   [4] 修改小车 IP
echo   [5] 打开现场手册
echo   [0] 退出
echo --------------------------------------------------
set /p CHOICE=选择 [1]:
if "%CHOICE%"=="" set CHOICE=1
if "%CHOICE%"=="1" goto RUN_FULL
if "%CHOICE%"=="2" goto RUN_CAM
if "%CHOICE%"=="3" goto RUN_CAR
if "%CHOICE%"=="4" goto SET_HOST
if "%CHOICE%"=="5" goto OPEN_DOC
if "%CHOICE%"=="0" exit /b 0
goto MENU

REM ---------- 找车：先测保存的 IP，不通则按车上 MAC 扫 ARP ----------
:ENSURE_HOST
if not defined CAR_HOST goto FIND_BY_MAC
powershell -NoProfile -Command "if (Test-NetConnection -ComputerName '%CAR_HOST%' -Port 22 -InformationLevel Quiet -WarningAction SilentlyContinue) {exit 0} else {exit 1}"
if not errorlevel 1 (
    echo [通过] 小车 %CAR_HOST% 可达
    goto :EOF
)
echo [提示] 保存的小车 IP %CAR_HOST% 不通，尝试按 MAC 重新定位……
:FIND_BY_MAC
REM 车上无线网卡 MAC 固定（28:2d:06:23:d0:6e），换网段也能认出来
set "FOUND_IP="
for /f "tokens=1" %%i in ('arp -a ^| findstr /i "28-2d-06-23-d0-6e"') do set "FOUND_IP=%%i"
if defined FOUND_IP (
    echo [通过] 按 MAC 找到小车：%FOUND_IP%
    set "CAR_HOST=%FOUND_IP%"
    echo CAR_HOST=%FOUND_IP%>"%ENV_FILE%"
    goto :EOF
)
set /p CAR_HOST=未找到小车，请手动输入小车 IP（留空回车返回菜单）:
if not defined CAR_HOST goto MENU
if "%CAR_HOST%"=="" goto MENU
echo CAR_HOST=%CAR_HOST%>"%ENV_FILE%"
goto :EOF

:SET_HOST
set /p CAR_HOST=输入小车 IP:
if not "%CAR_HOST%"=="" echo CAR_HOST=%CAR_HOST%>"%ENV_FILE%"
goto MENU

:OPEN_DOC
start "" "docs\track_diag现场手册.md"
goto MENU

REM ---------- 密码：仅本会话有效，不落盘 ----------
:ENSURE_PASS
if defined AG_CAR_PASS goto :EOF
echo.
for /f "delims=" %%p in ('powershell -NoProfile -Command "$p = Read-Host -AsSecureString '车端 SSH 密码（用户 duzhong，输入不回显）'; [Runtime.InteropServices.Marshal]::PtrToStringAuto([Runtime.InteropServices.Marshal]::SecureStringToBSTR($p))"') do set "AG_CAR_PASS=%%p"
goto :EOF

:RUN_FULL
call :ENSURE_HOST
if not defined CAR_HOST goto MENU
call :ENSURE_PASS
echo.
echo 启动诊断（Ctrl+C 或关窗结束；蓝箭头=信念 绿箭头=真值）……
python tools/track_diag.py run --car-host %CAR_HOST% %CORNER_ARG% --show
echo.
pause
goto MENU

:RUN_CAM
echo.
echo 相机自检模式（未接小车）。把 4 个场角标记和车顶标记放进画面，
echo 看到检测框即正常；按 Ctrl+C 或关窗结束。
python tools/track_diag.py run --no-car %CORNER_ARG% --show
echo.
pause
goto MENU

:RUN_CAR
call :ENSURE_HOST
if not defined CAR_HOST goto MENU
call :ENSURE_PASS
echo.
echo 小车自检模式（只看信念流，不开相机）……
python tools/track_diag.py run --car-host %CAR_HOST% --no-camera
echo.
pause
goto MENU
