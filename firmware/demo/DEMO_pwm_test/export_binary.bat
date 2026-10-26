@echo off
setlocal
set "FROMELF=fromelf.exe"
if exist "%~dp0..\..\..\..\..\keil\ARM\ARMCLANG\bin\fromelf.exe" set "FROMELF=%~dp0..\..\..\..\..\keil\ARM\ARMCLANG\bin\fromelf.exe"
if exist "%~dp0..\..\..\..\keil\ARM\ARMCLANG\bin\fromelf.exe" set "FROMELF=%~dp0..\..\..\..\keil\ARM\ARMCLANG\bin\fromelf.exe"
"%FROMELF%" --bincombined --output "%~dp0mdk\Objects\rt1064_pwm_test.bin" "%~dp0mdk\Objects\rt1064_pwm_test.axf"
exit /b %errorlevel%
