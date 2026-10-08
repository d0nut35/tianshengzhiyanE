$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$buildDir = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$source = [IO.File]::ReadAllText((Join-Path $projectRoot '01_App/mission/mission_app.c'))
$match = [regex]::Match($source, 'static bool mission_advance_slot\([^;]*?\)\s*\{')
if (!$match.Success) { throw 'Missing production slot function' }
$depth = 1
$end = $source.IndexOf('{', $match.Index) + 1
while ($depth -gt 0 -and $end -lt $source.Length) {
    if ($source[$end] -eq '{') { $depth++ }
    if ($source[$end] -eq '}') { $depth-- }
    $end++
}
[IO.File]::WriteAllText((Join-Path $buildDir 'slot_under_test.inc'), $source.Substring($match.Index,$end-$match.Index), [Text.UTF8Encoding]::new($false))
foreach ($trace in @(0,1)) {
    $binary = Join-Path $buildDir "test_slot_$trace.exe"
    gcc -std=c11 -Wall -Wextra -Werror "-DTEST_TRACE=$trace" -I $buildDir -I (Join-Path $projectRoot '01_App/mission') -I (Join-Path $projectRoot '03_Middleware/zdt_turntable') (Join-Path $PSScriptRoot 'test_slot.c') -o $binary
    if ($LASTEXITCODE -ne 0) { throw 'Slot compile failed' }
    & $binary
    if ($LASTEXITCODE -ne 0) { throw 'Slot regression failed' }
}
