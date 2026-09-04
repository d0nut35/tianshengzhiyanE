$ErrorActionPreference = "Stop"

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..\..\..")
$coreDir = Join-Path $projectRoot "03_Middleware\ball_manifest"
$outputDir = Join-Path $PSScriptRoot "build"
$testBinary = Join-Path $outputDir "test_ball_manifest_core.exe"

New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
gcc -std=c11 -Wall -Wextra -Werror `
    -I $coreDir `
    (Join-Path $coreDir "ball_manifest_core.c") `
    (Join-Path $PSScriptRoot "test_ball_manifest_core.c") `
    -o $testBinary
if ($LASTEXITCODE -ne 0) { throw "ball_manifest compile failed" }

& $testBinary
if ($LASTEXITCODE -ne 0) { throw "ball_manifest tests failed" }
