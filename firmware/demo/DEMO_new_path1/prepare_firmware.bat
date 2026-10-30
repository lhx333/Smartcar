@echo off
setlocal
set "PLANNER=%~dp0..\..\..\path_planner2"
python -B "%PLANNER%\prepare_firmware.py" --firmware "%~dp0."
exit /b %errorlevel%
