$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$outputDir = Join-Path $PSScriptRoot ".build"
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

$coreInclude = Join-Path $projectRoot "04_Bsp\ic_card"
$serviceInclude = Join-Path $projectRoot "02_Service\ic_card"
$appInclude = Join-Path $projectRoot "01_App\ic_card_device"
$coreSource = Join-Path $coreInclude "ic_bsp.c"
$serviceSource = Join-Path $serviceInclude "ic_card_service.c"
$ruleSource = Join-Path $appInclude "ic_ball_rule_2026.c"
$deviceSource = Join-Path $appInclude "ic_card_device.c"

gcc -std=c11 -Wall -Wextra -Werror -DLICANG_RELEASE_MINIMAL=1 `
    -I $coreInclude $coreSource `
    (Join-Path $PSScriptRoot "test_ic_card_core.c") `
    -o (Join-Path $outputDir "test_ic_card_core.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_ic_card_core.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

gcc -std=c11 -Wall -Wextra -Werror -DLICANG_RELEASE_MINIMAL=1 `
    -I $coreInclude -I $appInclude $ruleSource `
    (Join-Path $PSScriptRoot "test_ic_ball_rule_2026.c") `
    -o (Join-Path $outputDir "test_ic_ball_rule_2026.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_ic_ball_rule_2026.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

gcc -std=c11 -Wall -Wextra -Werror -DLICANG_RELEASE_MINIMAL=1 `
    -I $coreInclude -I $serviceInclude $coreSource $serviceSource `
    (Join-Path $PSScriptRoot "test_ic_card_service.c") `
    -o (Join-Path $outputDir "test_ic_card_service.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_ic_card_service.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

gcc -std=c11 -Wall -Wextra -Werror -DLICANG_RELEASE_MINIMAL=1 `
    -I $coreInclude -I $serviceInclude -I $appInclude `
    $coreSource $ruleSource $deviceSource `
    (Join-Path $PSScriptRoot "test_ic_card_device_transport.c") `
    -o (Join-Path $outputDir "test_ic_card_device_transport.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_ic_card_device_transport.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
