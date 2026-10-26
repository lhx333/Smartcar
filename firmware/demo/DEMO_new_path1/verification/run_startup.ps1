$ErrorActionPreference = 'Stop'
$compiler = (Get-Command gcc -ErrorAction Stop).Source
$startupBinary = Join-Path $PSScriptRoot 'startup_flow.exe'
& $compiler -std=c99 -Wall -Wextra -Werror `
    -I (Join-Path $PSScriptRoot 'runtime_stubs') `
    -I (Join-Path $PSScriptRoot '..\user\inc') `
    (Join-Path $PSScriptRoot 'startup_flow.c') -lm -o $startupBinary
if ($LASTEXITCODE -ne 0) { throw 'Startup regression compilation failed' }
& $startupBinary
if ($LASTEXITCODE -ne 0) { throw 'Startup regression failed' }
