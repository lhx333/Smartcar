@echo off
setlocal
set "PLANNER=%~dp0..\..\..\..\..\path_planner"
if exist "D:\Python1\python.exe" (
  "D:\Python1\python.exe" -B "%PLANNER%\prepare_firmware.py" --firmware "%~dp0."
) else (
  python -B "%PLANNER%\prepare_firmware.py" --firmware "%~dp0."
)
exit /b %errorlevel%
