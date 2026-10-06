$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$build = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$source = [IO.File]::ReadAllText((Join-Path $root '01_App/mission/mission_app.c'))
$functions = foreach ($name in @('mission_start_arm', 'mission_read_ball', 'mission_store_ball', 'mission_platform_prepare_storage', 'mission_finish_platform', 'mission_handle_storage')) {
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
[IO.File]::WriteAllText((Join-Path $build 'platform_under_test.inc'), ($functions -join "`n"), [Text.UTF8Encoding]::new($false))
foreach ($mode in @(0, 1)) {
    $binary = Join-Path $build "test_platform_$mode.exe"
    gcc -std=c11 -Wall -Wextra -Werror "-DTEST_MODE=$mode" -I $build -I (Join-Path $root '01_App/mission') (Join-Path $PSScriptRoot 'test_platform.c') -o $binary
    if ($LASTEXITCODE -ne 0) { throw 'Platform test compile failed' }
    & $binary
    if ($LASTEXITCODE -ne 0) { throw 'Platform regression failed' }
}
