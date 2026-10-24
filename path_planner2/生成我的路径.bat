@echo off
chcp 65001 >nul
cd /d "%~dp0"
python -B path_planner.py --input barrels_input.csv --out out_robust
if errorlevel 1 goto failed
start "" "out_robust\path.svg"
echo Read ROBUSTNESS.md. Firmware needs robust_trajectory.h and firmware_patch.
pause
exit /b 0
:failed
echo Planning rejected. Existing outputs may belong to an older run.
pause
exit /b 1
