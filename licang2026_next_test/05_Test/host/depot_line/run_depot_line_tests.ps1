param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$buildDir = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$ccArgs = @()
if ([IO.Path]::GetFileNameWithoutExtension($Compiler) -eq 'zig') { $ccArgs += 'cc' }
$ccArgs += @('-std=c11', '-O2', '-UNDEBUG', '-Wall', '-Wextra', '-Werror', '-ffunction-sections', '-fdata-sections')
foreach ($relative in @('05_Test/host/stairs/fakes', 'lhy/01_App', 'lhy/02_Service',
    'lhy/04_Bsp', 'lhy/04_Bsp/zdt_motor', 'lhy/04_Bsp/hwt101', 'lhy/99_Utils',
    'lhy/03_Middleware/map', '02_Service/chassis_bridge',
    'Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2')) {
    $ccArgs += @('-I', (Join-Path $projectRoot $relative))
}
$binary = Join-Path $buildDir 'test_depot_line.exe'
& $Compiler @ccArgs (Join-Path $PSScriptRoot 'test_depot_line.c') '-Wl,--gc-sections' -o $binary
if ($LASTEXITCODE -ne 0) { throw 'Depot line host compilation failed' }
& $binary
if ($LASTEXITCODE -ne 0) { throw 'Depot line regression failed' }
