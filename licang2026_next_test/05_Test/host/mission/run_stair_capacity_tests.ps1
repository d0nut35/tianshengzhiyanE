$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$build = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$source = [IO.File]::ReadAllText((Join-Path $root '01_App/mission/mission_app.c'))
# 同时验证真实ROUTE STAIRS入口确实初始化测试档案，不能仅验证Core特例。
if ($source -notmatch 'if \(g_wireless_test.target == MISSION_TEST_STAGE_STAIRS\)\s*\{\s*ball_manifest_init_stair_test\(&ctx->manifest, MISSION_STAIR_TEST_BALL_COUNT\);') {
    throw 'ROUTE STAIRS manifest initialization missing'
}
$functions = foreach ($name in @('mission_record_ball', 'mission_handle_storage')) {
    $match = [regex]::Match($source, "static (?:bool|void) $name\([^;]*?\)\s*\{")
    if (!$match.Success) { throw "Missing function $name" }
    $end = $source.IndexOf('{', $match.Index) + 1
    $depth = 1
    while ($depth -gt 0 -and $end -lt $source.Length) {
        if ($source[$end] -eq '{') { $depth++ }
        if ($source[$end] -eq '}') { $depth-- }
        $end++
    }
    $source.Substring($match.Index, $end - $match.Index)
}
[IO.File]::WriteAllText((Join-Path $build 'stair_capacity_under_test.inc'), ($functions -join "`n"), [Text.UTF8Encoding]::new($false))
$core = Join-Path $root '03_Middleware/ball_manifest'
foreach ($mode in @(0, 1)) {
    foreach ($minimal in @(0, 1)) {
        $binary = Join-Path $build "test_stair_capacity_${mode}_$minimal.exe"
        gcc -std=c11 -Wall -Wextra -Werror "-DTEST_MODE=$mode" "-DLICANG_RELEASE_MINIMAL=$minimal" `
            -I $build -I $core -I (Join-Path $root '01_App/mission') `
            (Join-Path $PSScriptRoot 'test_stair_capacity.c') (Join-Path $core 'ball_manifest_core.c') -o $binary
        if ($LASTEXITCODE -ne 0) { throw 'Stair capacity compile failed' }
        & $binary
        if ($LASTEXITCODE -ne 0) { throw 'Stair capacity regression failed' }
    }
}
