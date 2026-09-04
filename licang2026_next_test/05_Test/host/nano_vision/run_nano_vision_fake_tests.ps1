$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$outputDir = Join-Path $PSScriptRoot ".build"
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

$coreInclude = Join-Path $projectRoot "03_Middleware\nano_vision"
$coreSource = Join-Path $coreInclude "nano_vision_core.c"
$testSource = Join-Path $PSScriptRoot "test_nano_vision_core.c"
$testBinary = Join-Path $outputDir "test_nano_vision_core.exe"

gcc -std=c11 -Wall -Wextra -Werror `
    -I $coreInclude $coreSource $testSource -o $testBinary
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $testBinary
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
