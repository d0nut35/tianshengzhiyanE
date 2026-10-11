$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$build = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$source = [IO.File]::ReadAllText((Join-Path $root '01_App/mission/mission_app.c'))
$functions = foreach ($name in @('mission_test_pause_ms', 'mission_test_pause', 'mission_test_depot_path_command')) {
    $match = [regex]::Match($source, "static bool $name\([^;]*?\)\s*\{")
    if (!$match.Success) { throw "Missing production function $name" }
    $end = $source.IndexOf('{', $match.Index) + 1
    $depth = 1
    while ($depth -gt 0 -and $end -lt $source.Length) {
        if ($source[$end] -eq '{') { $depth++ }
        if ($source[$end] -eq '}') { $depth-- }
        $end++
    }
    $source.Substring($match.Index, $end - $match.Index)
}
[IO.File]::WriteAllText((Join-Path $build 'depot_path_under_test.inc'), ($functions -join "`n"), [Text.UTF8Encoding]::new($false))
$binary = Join-Path $build 'test_depot_path.exe'
gcc -std=c11 -Wall -Wextra -Werror -I $build -I (Join-Path $root '01_App/mission') (Join-Path $PSScriptRoot 'test_depot_path.c') -o $binary
if ($LASTEXITCODE -ne 0) { throw 'Depot path compile failed' }
& $binary
if ($LASTEXITCODE -ne 0) { throw 'Depot path tests failed' }
