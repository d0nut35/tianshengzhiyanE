param([string]$SourceRevision = '')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$buildDir = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
if ($SourceRevision) {
    $repo = (Resolve-Path (Join-Path $projectRoot '..')).Path
    $source = (git -c "safe.directory=$($repo.Replace('\','/'))" -C $repo show "${SourceRevision}:licang2026_next_test/01_App/mission/mission_app.c") -join "`n"
    if ($LASTEXITCODE -ne 0) { throw 'Cannot read baseline source' }
} else {
    $source = [IO.File]::ReadAllText((Join-Path $projectRoot '01_App/mission/mission_app.c'))
}
# 测试实际函数，复现“回调已到、线程尚未消费”这一调度窗口。
$functions = foreach ($name in @('mission_start_vision', 'mission_vision_process')) {
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
[IO.File]::WriteAllText((Join-Path $buildDir 'vision_under_test.inc'), ($functions -join "`n"), [Text.UTF8Encoding]::new($false))
$core = Join-Path $projectRoot '03_Middleware/nano_vision'
$binary = Join-Path $buildDir 'test_vision_handoff.exe'
gcc -std=c11 -Wall -Wextra -Werror -I $buildDir -I $core `
    -I (Join-Path $projectRoot '01_App/mission') `
    (Join-Path $PSScriptRoot 'test_vision_handoff.c') (Join-Path $core 'nano_vision_core.c') -o $binary
if ($LASTEXITCODE -ne 0) { throw 'Vision handoff compile failed' }
& $binary
if ($LASTEXITCODE -ne 0) { throw 'Vision handoff regression failed' }
