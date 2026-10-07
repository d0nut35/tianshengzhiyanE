$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$buildDir = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$source = [IO.File]::ReadAllText((Join-Path $projectRoot '01_App/mission/mission_app.c'))
function Get-ProductionFunction([string]$name) {
    $match = [regex]::Match($source, "static (?:void|bool) $name\([^;]*?\)\s*\{")
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
$functions = foreach ($name in @('mission_handle_block_result', 'mission_test_block_move', 'mission_test_run_block_digits')) { Get-ProductionFunction $name }
[IO.File]::WriteAllText((Join-Path $buildDir 'block_digit_under_test.inc'), ($functions -join "`n"), [Text.UTF8Encoding]::new($false))
[IO.File]::WriteAllText((Join-Path $buildDir 'block_delay_under_test.inc'), (Get-ProductionFunction 'mission_test_block_delay'), [Text.UTF8Encoding]::new($false))
$core = Join-Path $projectRoot '03_Middleware/nano_vision'
$binary = Join-Path $buildDir 'test_block_digit.exe'
gcc -std=c11 -Wall -Wextra -Werror -I $buildDir -I $core -I (Join-Path $projectRoot '01_App/mission') (Join-Path $PSScriptRoot 'test_block_digit.c') (Join-Path $core 'nano_vision_core.c') -o $binary
if ($LASTEXITCODE -ne 0) { throw 'Block digit compile failed' }
& $binary
if ($LASTEXITCODE -ne 0) { throw 'Block digit test failed' }
