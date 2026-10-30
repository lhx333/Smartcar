@echo off
chcp 65001 >nul
cd /d "%~dp0"
python -B prepare_firmware.py
if errorlevel 1 goto failed
start "" "out_robust\path.svg"
echo DEMO_new_path1 headers updated. Rebuild its Keil project before downloading.
pause
exit /b 0
:failed
echo Planning rejected. Existing outputs may belong to an older run.
pause
exit /b 1
