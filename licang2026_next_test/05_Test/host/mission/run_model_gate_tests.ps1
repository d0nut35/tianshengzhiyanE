$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$buildDir = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$source = [IO.File]::ReadAllText((Join-Path $projectRoot '01_App/mission/mission_app.c'))
$functions = foreach ($name in @('mission_model_process', 'mission_try_ready', 'mission_prepare_zdt')) {
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
[IO.File]::WriteAllText((Join-Path $buildDir 'model_gate_under_test.inc'), ($functions -join "`n"), [Text.UTF8Encoding]::new($false))
# 抽取实际事务邮箱消费和MODEL_STATE分支，覆盖起点通信故障，不复制判定实现。
$replyStart = $source.IndexOf('    /* 0) 先处理等待当前串口事务结束')
$replyEnd = $source.IndexOf('    if ((ctx->vision.phase == MISSION_VISION_STARTING)', $replyStart)
if ($replyStart -lt 0 -or $replyEnd -lt 0) { throw 'Missing model reply branch' }
$reply = "static void model_reply(mission_context_t *ctx) { nano_vision_status_t status;`n" + $source.Substring($replyStart,$replyEnd - $replyStart) + "}`n"
[IO.File]::WriteAllText((Join-Path $buildDir 'model_reply_under_test.inc'), $reply, [Text.UTF8Encoding]::new($false))
$core = Join-Path $projectRoot '03_Middleware/nano_vision'
foreach ($trace in @(0,1)) {
    $binary = Join-Path $buildDir "test_model_gate_$trace.exe"
    gcc -std=c11 -Wall -Wextra -Werror "-DTEST_TRACE=$trace" -I $buildDir -I $core -I (Join-Path $projectRoot '01_App/mission') -I (Join-Path $projectRoot '03_Middleware/zdt_turntable') (Join-Path $PSScriptRoot 'test_model_gate.c') (Join-Path $core 'nano_vision_core.c') -o $binary
    if ($LASTEXITCODE -ne 0) { throw 'Model gate compile failed' }
    & $binary
    if ($LASTEXITCODE -ne 0) { throw 'Model gate test failed' }
}
