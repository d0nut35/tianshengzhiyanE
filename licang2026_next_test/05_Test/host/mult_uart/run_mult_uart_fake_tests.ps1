$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$outputDir = Join-Path $PSScriptRoot ".build"
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

$coreDir = Join-Path $projectRoot "03_Middleware\mult_uart"
$serviceDir = Join-Path $projectRoot "02_Service\mult_uart"
$deviceDir = Join-Path $projectRoot "01_App\mult_uart_device"
$targetTestDir = Join-Path $projectRoot "05_Test\mult_uart"

gcc -std=c11 -Wall -Wextra -Werror `
    -I $coreDir (Join-Path $coreDir "mult_uart_core.c") `
    (Join-Path $PSScriptRoot "test_mult_uart_core.c") `
    -o (Join-Path $outputDir "test_mult_uart_core.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_mult_uart_core.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

gcc -std=c11 -Wall -Wextra -Werror `
    -I $coreDir -I $serviceDir `
    (Join-Path $coreDir "mult_uart_core.c") `
    (Join-Path $serviceDir "mult_uart_service.c") `
    (Join-Path $PSScriptRoot "test_mult_uart_service.c") `
    -o (Join-Path $outputDir "test_mult_uart_service.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_mult_uart_service.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

gcc -std=c11 -Wall -Wextra -Werror `
    -I $coreDir -I $serviceDir -I $deviceDir `
    (Join-Path $deviceDir "mult_uart_device.c") `
    (Join-Path $PSScriptRoot "test_mult_uart_device.c") `
    -o (Join-Path $outputDir "test_mult_uart_device.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_mult_uart_device.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

gcc -std=c11 -Wall -Wextra -Werror `
    -I $targetTestDir `
    (Join-Path $targetTestDir "mult_uart_test_protocol.c") `
    (Join-Path $PSScriptRoot "test_mult_uart_test_protocol.c") `
    -o (Join-Path $outputDir "test_mult_uart_test_protocol.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_mult_uart_test_protocol.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
