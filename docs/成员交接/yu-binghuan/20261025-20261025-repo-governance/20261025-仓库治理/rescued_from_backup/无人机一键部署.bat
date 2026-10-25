@echo off
REM 无人机一键链: deploy all drone-side code (except flight controller) -> build/verify -> bringup -> health -> smoke
REM Usage:  set AG_ORIN_PASS=******  &&  无人机一键部署.bat
REM         set AG_ORIN_PASS=******  &&  无人机一键部署.bat --dry-run
python "%~dp0tools\orin_chain.py" %*
