$ErrorActionPreference = 'Stop'
$projectDir = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..')).Path
$moduleDir = Join-Path $projectDir 'lhy\04_Bsp\hwt101'
$buildDir = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

# Instrument a generated copy only; production has no test switch or callback.
$source = [IO.File]::ReadAllText((Join-Path $moduleDir 'hwt101_adaption.c'))
$source = $source.Replace("`r`n", "`n")
if (($source.Split('    size_t pos = 0U;').Length -ne 2) -or
    ($source.Split('        frame = &data[pos];').Length -ne 2)) {
    throw 'Parser instrumentation anchors changed; review the host test.'
}
$source = $source.Replace('    size_t pos = 0U;',
    "    test_parse_enter();`n    size_t pos = 0U;")
$source = $source.Replace('        frame = &data[pos];',
    "        test_scan_step();`n        frame = &data[pos];")
[IO.File]::WriteAllText((Join-Path $buildDir 'driver_under_test.c'),
    $source, [Text.UTF8Encoding]::new($false))

foreach ($optimization in @('-O0', '-O2')) {
    $testExe = Join-Path $buildDir ('test_hwt101' + $optimization + '.exe')
    & gcc -std=c11 -Wall -Wextra -Werror $optimization `
        -I (Join-Path $PSScriptRoot 'fake') -I $moduleDir -I $buildDir `
        (Join-Path $PSScriptRoot 'test_hwt101.c') -lm -o $testExe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $testExe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
