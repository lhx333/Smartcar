@echo off
setlocal
set "FROMELF=%~dp0..\..\keil\ARM\ARMCLANG\bin\fromelf.exe"
if not exist "%FROMELF%" set "FROMELF=fromelf.exe"
"%FROMELF%" --bincombined --output "%~dp0mdk\Objects\rt1064_component.bin" "%~dp0mdk\Objects\rt1064_component.axf"
exit /b %errorlevel%
