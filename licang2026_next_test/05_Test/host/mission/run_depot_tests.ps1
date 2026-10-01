$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$buildDir = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$source = [IO.File]::ReadAllText((Join-Path $projectRoot '01_App/mission/mission_app.c'))
# 直接抽取正式源码函数，不复制状态转换实现；设备/时钟由测试替身提供。
$names = @('mission_depot_ccw_steps', 'mission_depot_prepare', 'mission_depot_start_digit', 'mission_depot_digit_done',
           'mission_depot_next_ball', 'mission_depot_arm_done', 'mission_check_timeout')
$functions = foreach ($name in $names) {
    $match = [regex]::Match($source, "static (?:void|uint8_t) $name\([^;]*?\)\s*\{")
    if (!$match.Success) { throw "Missing production function: $name" }
    $opening = $source.IndexOf('{', $match.Index)
    $depth = 1
    $end = $opening + 1
    while ($depth -gt 0 -and $end -lt $source.Length) {
        if ($source[$end] -eq '{') { $depth++ }
        if ($source[$end] -eq '}') { $depth-- }
        $end++
    }
    $source.Substring($match.Index, $end - $match.Index)
}
[IO.File]::WriteAllText((Join-Path $buildDir 'depot_under_test.inc'), ($functions -join "`n"), [Text.UTF8Encoding]::new($false))
$core = Join-Path $projectRoot '03_Middleware/ball_manifest'
foreach ($minimal in @(0, 1)) {
    $binary = Join-Path $buildDir "test_depot_$minimal.exe"
    gcc -std=c11 -Wall -Wextra -Werror "-DLICANG_RELEASE_MINIMAL=$minimal" `
        -I $buildDir -I $core -I (Join-Path $projectRoot '01_App/mission') `
        (Join-Path $PSScriptRoot 'test_depot.c') (Join-Path $core 'ball_manifest_core.c') -o $binary
    if ($LASTEXITCODE -ne 0) { throw 'Mission depot test compile failed' }
    & $binary
    if ($LASTEXITCODE -ne 0) { throw 'Mission depot tests failed' }
}
# 同时执行无线生产函数，核对1基槽转换、跨列当前位置及动作/故障收敛。
$wirelessFunctions = foreach ($name in @('mission_depot_ccw_steps', 'mission_test_run_depot_balls')) {
    $match = [regex]::Match($source, "static (?:uint8_t|bool) $name\([^;]*?\)\s*\{")
    if (!$match.Success) { throw "Missing production function: $name" }
    $opening = $source.IndexOf('{', $match.Index)
    $depth = 1
    $end = $opening + 1
    while ($depth -gt 0 -and $end -lt $source.Length) {
        if ($source[$end] -eq '{') { $depth++ }
        if ($source[$end] -eq '}') { $depth-- }
        $end++
    }
    $source.Substring($match.Index, $end - $match.Index)
}
[IO.File]::WriteAllText((Join-Path $buildDir 'depot_wireless_under_test.inc'), ($wirelessFunctions -join "`n"), [Text.UTF8Encoding]::new($false))
$wirelessBinary = Join-Path $buildDir 'test_depot_wireless.exe'
gcc -std=c11 -Wall -Wextra -Werror -I $buildDir -I $core -I (Join-Path $projectRoot '01_App/mission') (Join-Path $PSScriptRoot 'test_depot_wireless.c') -o $wirelessBinary
if ($LASTEXITCODE -ne 0) { throw 'Wireless depot test compile failed' }
& $wirelessBinary
if ($LASTEXITCODE -ne 0) { throw 'Wireless depot regression failed' }
