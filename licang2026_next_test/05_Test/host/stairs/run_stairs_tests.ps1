param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$buildDir = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$ccArgs = @()
if ([IO.Path]::GetFileNameWithoutExtension($Compiler) -eq 'zig') { $ccArgs += 'cc' }
$ccArgs += @('-std=c11', '-Wall', '-Wextra', '-Werror')
foreach ($relative in @('05_Test/host/stairs/fakes', 'lhy/01_App', 'lhy/02_Service',
    'lhy/04_Bsp', 'lhy/99_Utils', 'lhy/03_Middleware/map', '02_Service/chassis_bridge',
    'Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2')) {
    $ccArgs += @('-I', (Join-Path $projectRoot $relative))
}
$binary = Join-Path $buildDir 'test_stairs.exe'
& $Compiler @ccArgs (Join-Path $PSScriptRoot 'test_stairs.c') -o $binary
if ($LASTEXITCODE -ne 0) { throw 'Stairs host compilation failed' }
& $binary
if ($LASTEXITCODE -ne 0) { throw 'Stairs regression failed' }
