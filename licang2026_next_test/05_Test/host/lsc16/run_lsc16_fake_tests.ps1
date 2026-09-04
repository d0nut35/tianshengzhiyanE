$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$outputDir = Join-Path $PSScriptRoot ".build"
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

$coreDir = Join-Path $projectRoot "03_Middleware\lsc16"
$serviceDir = Join-Path $projectRoot "02_Service\lsc16"
$coreSource = Join-Path $coreDir "lsc16_core.c"
$serviceSource = Join-Path $serviceDir "lsc16_service.c"
$testDir = Join-Path $projectRoot "05_Test\lsc16"
$testCommonSource = Join-Path $testDir "lsc16_test_common.c"

gcc -std=c11 -Wall -Wextra -Werror `
    -I $coreDir $coreSource `
    (Join-Path $PSScriptRoot "test_lsc16_core.c") `
    -o (Join-Path $outputDir "test_lsc16_core.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_lsc16_core.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

gcc -std=c11 -Wall -Wextra -Werror `
    -I $coreDir -I $serviceDir $coreSource $serviceSource `
    (Join-Path $PSScriptRoot "test_lsc16_service.c") `
    -o (Join-Path $outputDir "test_lsc16_service.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_lsc16_service.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

gcc -std=c11 -Wall -Wextra -Werror `
    -I $coreDir -I $testDir $testCommonSource `
    (Join-Path $PSScriptRoot "test_lsc16_test_common.c") `
    -o (Join-Path $outputDir "test_lsc16_test_common.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_lsc16_test_common.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
