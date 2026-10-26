$ErrorActionPreference = 'Stop'
$compiler = (Get-Command gcc -ErrorAction Stop).Source
$inc = Join-Path $PSScriptRoot '..\user\inc'
$pulseBinary = Join-Path $PSScriptRoot 'pwm_pulse.exe'
& $compiler -std=c99 -Wall -Wextra -Werror -I (Join-Path $PSScriptRoot 'runtime_stubs') -I $inc (Join-Path $PSScriptRoot 'pwm_pulse.c') -lm -o $pulseBinary
if ($LASTEXITCODE -ne 0) { throw 'PWM pulse regression compilation failed' }
& $pulseBinary
if ($LASTEXITCODE -ne 0) { throw 'PWM pulse regression failed' }
$actuatorBinary = Join-Path $PSScriptRoot 'actuator_pwm.exe'
& $compiler -std=c99 -Wall -Wextra -Werror -I (Join-Path $PSScriptRoot 'stubs') -I $inc (Join-Path $PSScriptRoot 'actuator_pwm.c') (Join-Path $PSScriptRoot '..\user\src\speed_control.c') (Join-Path $PSScriptRoot '..\user\src\servo_control.c') -lm -o $actuatorBinary
if ($LASTEXITCODE -ne 0) { throw 'Actuator regression compilation failed' }
& $actuatorBinary
if ($LASTEXITCODE -ne 0) { throw 'Actuator regression failed' }
