$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$outputDir = Join-Path $PSScriptRoot ".build"
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

$coreInclude = Join-Path $projectRoot "04_Bsp\ic_card"
$serviceInclude = Join-Path $projectRoot "02_Service\ic_card"
$fakeInclude = Join-Path $PSScriptRoot "fakes"
$coreSource = Join-Path $coreInclude "ic_bsp.c"
$serviceSource = Join-Path $serviceInclude "ic_card_service.c"
$muxStub = Join-Path $fakeInclude "mux_service_stub.c"

gcc -std=c11 -Wall -Wextra -Werror -DLICANG_RELEASE_MINIMAL=1 `
    -I $coreInclude $coreSource `
    (Join-Path $PSScriptRoot "test_ic_card_core.c") `
    -o (Join-Path $outputDir "test_ic_card_core.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_ic_card_core.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

gcc -std=c11 -Wall -Wextra -Werror -DLICANG_RELEASE_MINIMAL=1 `
    -I $fakeInclude -I $coreInclude -I $serviceInclude `
    $coreSource $serviceSource $muxStub `
    (Join-Path $PSScriptRoot "test_ic_ball_rule_2026.c") `
    -o (Join-Path $outputDir "test_ic_ball_rule_2026.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_ic_ball_rule_2026.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

gcc -std=c11 -Wall -Wextra -Werror -DLICANG_RELEASE_MINIMAL=1 `
    -I $fakeInclude -I $coreInclude -I $serviceInclude `
    $coreSource $serviceSource $muxStub `
    (Join-Path $PSScriptRoot "test_ic_card_service.c") `
    -o (Join-Path $outputDir "test_ic_card_service.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_ic_card_service.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

gcc -std=c11 -Wall -Wextra -Werror -DLICANG_RELEASE_MINIMAL=1 `
    -I $fakeInclude -I $coreInclude -I $serviceInclude `
    $coreSource $serviceSource `
    (Join-Path $PSScriptRoot "test_ic_card_device_transport.c") `
    -o (Join-Path $outputDir "test_ic_card_device_transport.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $outputDir "test_ic_card_device_transport.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
