@echo off
REM Dumb launcher on purpose: cmd.exe mis-decodes UTF-8 Chinese batch text
REM (chunked reads under chcp 65001) and CRLF/LF trips goto parsing.
REM All UI text lives in tools/uav_wizard_menu.py (Chinese, encoding-safe).
cd /d "%~dp0"
where python >nul 2>nul || (echo [ERROR] python not found in PATH & pause & exit /b 1)
python tools\uav_wizard_menu.py
if errorlevel 1 pause
