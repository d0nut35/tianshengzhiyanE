$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$build = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$source = [IO.File]::ReadAllText((Join-Path $root '01_App/mission/mission_app.c'))
$match = [regex]::Match($source, 'static bool mission_test_disc_path_command\([^;]*?\)\s*\{')
if (!$match.Success) { throw 'Missing production disc path command function' }
$end = $source.IndexOf('{', $match.Index) + 1
$depth = 1
while ($depth -gt 0 -and $end -lt $source.Length) {
    if ($source[$end] -eq '{') { $depth++ }
    if ($source[$end] -eq '}') { $depth-- }
    $end++
}
[IO.File]::WriteAllText((Join-Path $build 'disc_path_under_test.inc'), $source.Substring($match.Index, $end - $match.Index), [Text.UTF8Encoding]::new($false))
$binary = Join-Path $build 'test_disc_path.exe'
gcc -std=c11 -Wall -Wextra -Werror -I $build -I (Join-Path $root '01_App/mission') (Join-Path $PSScriptRoot 'test_disc_path.c') -o $binary
if ($LASTEXITCODE -ne 0) { throw 'Disc path test compile failed' }
& $binary
if ($LASTEXITCODE -ne 0) { throw 'Disc path regression failed' }
