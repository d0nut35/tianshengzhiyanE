$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$buildDir = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$source = [IO.File]::ReadAllText((Join-Path $projectRoot '01_App/mission/mission_app.c'))
$functions = foreach ($name in @('mission_model_process', 'mission_try_ready')) {
    $match = [regex]::Match($source, "static void $name\([^;]*?\)\s*\{")
    if (!$match.Success) { throw "Missing function: $name" }
    $depth = 1
    $end = $source.IndexOf('{', $match.Index) + 1
    while ($depth -gt 0 -and $end -lt $source.Length) {
        if ($source[$end] -eq '{') { $depth++ }
        if ($source[$end] -eq '}') { $depth-- }
        $end++
    }
    $source.Substring($match.Index, $end - $match.Index)
}
[IO.File]::WriteAllText((Join-Path $buildDir 'model_gate_under_test.inc'), ($functions -join "`n"), [Text.UTF8Encoding]::new($false))
$core = Join-Path $projectRoot '03_Middleware/nano_vision'
$binary = Join-Path $buildDir 'test_model_gate.exe'
gcc -std=c11 -Wall -Wextra -Werror -I $buildDir -I $core -I (Join-Path $projectRoot '01_App/mission') (Join-Path $PSScriptRoot 'test_model_gate.c') (Join-Path $core 'nano_vision_core.c') -o $binary
if ($LASTEXITCODE -ne 0) { throw 'Model gate compile failed' }
& $binary
if ($LASTEXITCODE -ne 0) { throw 'Model gate test failed' }
