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
$functions = foreach ($name in @('mission_start_vision', 'mission_vision_process', 'mission_stair_grasp_group', 'mission_start_stair_layer')) {
    $match = [regex]::Match($source, "static (?:void|bool|uint8_t) $name\([^;]*?\)\s*\{")
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
# 使用生产源码的停车回报分支，验证ACK/PAUSE两种真实先后顺序。
$pauseStart = $source.IndexOf('    if ((event->type == CHASSIS_CMD_STAIR_PAUSE)')
$pauseEnd = $source.IndexOf('    /* 7) 底盘确认恢复', $pauseStart)
if ($pauseStart -lt 0 -or $pauseEnd -lt 0) { throw 'Missing stair pause branch' }
$functions += "static void handle_pause(mission_context_t *ctx, const chassis_mission_event_t *event) { uint8_t grasp_group;`n" + $source.Substring($pauseStart, $pauseEnd - $pauseStart) + "}`n"
$diagnosticSource = [IO.File]::ReadAllText((Join-Path $projectRoot '01_App/mission/mission_app.c'))
$diagStart = $diagnosticSource.IndexOf('/* 阶梯诊断只在正式日志开关打开时存在')
$diagEnd = $diagnosticSource.IndexOf('static mission_context_t g_mission;', $diagStart)
if ($diagStart -lt 0 -or $diagEnd -lt 0) { throw 'Missing stair diagnostics' }
[IO.File]::WriteAllText((Join-Path $buildDir 'stair_diag_under_test.inc'), $diagnosticSource.Substring($diagStart, $diagEnd - $diagStart), [Text.UTF8Encoding]::new($false))
[IO.File]::WriteAllText((Join-Path $buildDir 'vision_under_test.inc'), ($functions -join "`n"), [Text.UTF8Encoding]::new($false))
$core = Join-Path $projectRoot '03_Middleware/nano_vision'
foreach ($trace in @(0, 1)) {
$binary = Join-Path $buildDir "test_vision_handoff_$trace.exe"
gcc -std=c11 -Wall -Wextra -Werror "-DTEST_TRACE=$trace" -I $buildDir -I $core `
    -I (Join-Path $projectRoot '01_App/mission') `
    (Join-Path $PSScriptRoot 'test_vision_handoff.c') (Join-Path $core 'nano_vision_core.c') -o $binary
if ($LASTEXITCODE -ne 0) { throw 'Vision handoff compile failed' }
& $binary
if ($LASTEXITCODE -ne 0) { throw 'Vision handoff regression failed' }
}
