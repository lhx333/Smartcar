@echo off
setlocal
cd /d "%~dp0"
set "PYTHON310=C:\Users\pc\AppData\Roaming\uv\python\cpython-3.10-windows-x86_64-none\python.exe"
if not exist ".venv\Scripts\python.exe" (
  if not exist "%PYTHON310%" (
    echo Python 3.10 was not found at:
    echo %PYTHON310%
    pause
    exit /b 1
  )
  echo Creating Python virtual environment...
  "%PYTHON310%" -m venv .venv
  if errorlevel 1 (echo Could not create the Python 3.10 environment.& pause & exit /b 1)
  echo Installing dependencies...
  ".venv\Scripts\python.exe" -m pip install -r requirements.txt
  if errorlevel 1 (echo Dependency installation failed.& pause & exit /b 1)
)
".venv\Scripts\python.exe" main.py
if errorlevel 1 pause
endlocal
