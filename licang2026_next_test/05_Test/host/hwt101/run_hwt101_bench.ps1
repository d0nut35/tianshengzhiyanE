param([string]$BaselineDir)
$ErrorActionPreference = 'Stop'
$projectDir = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..')).Path
$moduleDir = Join-Path $projectDir 'lhy\04_Bsp\hwt101'
$buildDir = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

if (-not $BaselineDir) {
    $BaselineDir = Join-Path $projectDir 'tmp\hwt101_opt_before\hwt101'
}
if (-not (Test-Path -LiteralPath (Join-Path $BaselineDir 'hwt101.c'))) {
    throw 'Pass -BaselineDir with the saved pre-change driver directory.'
}

foreach ($variant in @('before', 'after')) {
    $sourceDir = if ($variant -eq 'before') { $BaselineDir } else { $moduleDir }
    $sources = @((Join-Path $sourceDir 'hwt101_adaption.c'))
    if ($variant -eq 'before') {
        $sources += Join-Path $sourceDir 'hwt101.c'
    }
    $benchExe = Join-Path $buildDir ('bench_' + $variant + '.exe')
    & gcc -std=c11 -Wall -Wextra -Werror -O2 `
        -I (Join-Path $PSScriptRoot 'fake') -I $sourceDir `
        (Join-Path $PSScriptRoot 'bench_hwt101.c') @sources -lm -o $benchExe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    Write-Output ($variant + ' (same GCC -O2, 9 x 1000000 calls):')
    & $benchExe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
