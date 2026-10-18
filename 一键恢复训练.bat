@echo off
chcp 65001 >nul
echo ============================================
echo    一键恢复训练 - 航拍场地 11 类检测模型
echo ============================================
echo.
echo 前置条件：
echo   1. 已安装 Python 3.12
echo   2. 原始视频已放到本目录，命名为 video.mp4
echo.
echo 即将自动完成：环境搭建 -^> 抽帧 -^> 切分 -^> 训练
echo.
pause

powershell -ExecutionPolicy Bypass -File "%~dp0setup_and_train.ps1"

echo.
echo ============================================
echo  训练已启动，日志见 train_guard.log
echo  关闭此窗口不会停止训练（守护进程独立运行）
echo ============================================
pause
