@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo Reading barrels_input.csv ...
echo.
python -B path_planner.py --input barrels_input.csv --out out_my
if errorlevel 1 goto failed
echo.
echo Done. Opening out_my\path.svg ...
start "" "out_my\path.svg"
echo.
echo C array: out_my\path_array.txt
echo Copy it into navigation.c when you are ready.
pause
exit /b 0

:failed
echo.
echo Planning failed. Check the messages above and barrels_input.csv.
pause
exit /b 1
