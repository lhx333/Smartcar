@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo Creating Python virtual environment...
  py -m venv .venv
  if errorlevel 1 (echo Python 3 is required.& pause & exit /b 1)
  echo Installing dependencies...
  ".venv\Scripts\python.exe" -m pip install -r requirements.txt
  if errorlevel 1 (echo Dependency installation failed.& pause & exit /b 1)
)
".venv\Scripts\python.exe" main.py
if errorlevel 1 pause
endlocal
