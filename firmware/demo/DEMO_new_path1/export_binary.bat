@echo off
setlocal
set "FROMELF=fromelf.exe"
where fromelf.exe >nul 2>&1
if not errorlevel 1 goto run
for /f "usebackq delims=" %%I in (`powershell -NoProfile -Command "$p=(Get-Process -Name UV4 -ErrorAction SilentlyContinue)[0].Path; if ($p) { Join-Path (Split-Path (Split-Path $p)) 'ARM\ARMCLANG\bin\fromelf.exe' }"`) do set "FROMELF=%%I"
:run
"%FROMELF%" --bincombined --output "%~dp0mdk\Objects\rt1064_new_path1.bin" "%~dp0mdk\Objects\rt1064_new_path1.axf"
exit /b %errorlevel%
