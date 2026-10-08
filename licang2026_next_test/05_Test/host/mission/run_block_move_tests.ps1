$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$build = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$source = [IO.File]::ReadAllText((Join-Path $root '01_App/mission/mission_app.c'))
function Get-Function([string]$name) {
    $match = [regex]::Match($source, "static (?:void|bool|uint8_t|uint16_t) $name\([^;]*?\)\s*\{")
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
$names = @('mission_arm_tx_done', 'mission_arm_report', 'mission_start_arm',
    'mission_next_request_id', 'mission_fail', 'mission_handle_command',
    'mission_reset_vision', 'mission_start_vision', 'mission_stop_vision',
    'mission_handle_block_result', 'mission_vision_process', 'mission_check_timeout',
    'mission_block_place_point', 'mission_block_scan_point_id', 'mission_block_run_arm', 'mission_block_start_layer', 'mission_block_next_layer',
    'mission_block_position_done', 'mission_block_move', 'mission_block_scan_point',
    'mission_block_arm_done', 'mission_block_begin', 'mission_block_digit_done')
$functions = @($names | ForEach-Object { Get-Function $_ })
$declarations = $functions | ForEach-Object { $_.Substring(0, $_.IndexOf('{')).Trim() + ';' }
[IO.File]::WriteAllText((Join-Path $build 'block_move_declarations.inc'), ($declarations -join "`n"), [Text.UTF8Encoding]::new($false))
# 使用实际请求编号门禁及积木分支，排除与本夹具无关的圆盘/阶梯分支。
$chassis = Get-Function 'mission_handle_chassis'
$start = $chassis.IndexOf('    /* 1) 正式流程只接收')
$end = $chassis.IndexOf('    /* 3) 圆盘到位', $start)
$gate = $chassis.Substring($start, $end - $start)
$start = $chassis.IndexOf('#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED', $end)
$end = $chassis.IndexOf('    if ((ctx->state == MISSION_STATE_WAIT_PLATFORM)', $start)
$functions += "static void block_handle_chassis(mission_context_t *ctx, const chassis_mission_event_t *event) {`n" + $gate + $chassis.Substring($start, $end - $start) + "}`n"
$arm = Get-Function 'mission_handle_arm'
$start = $arm.IndexOf('    if ((ctx->state == MISSION_STATE_BLOCK_PREPARE)')
$end = $arm.IndexOf('#if !MISSION_CHASSIS_ROUTE_TEST_ENABLED', $start)
$functions += "static void block_handle_arm(mission_context_t *ctx, bool success) {`n" + $arm.Substring($start, $end - $start) + "}`n"
# ACK成功/失败与STOP回报仍执行生产事务消费代码，不把write提交当成成功。
$vision = Get-Function 'mission_handle_vision'
$start = $vision.IndexOf('    /* 0) 先处理等待当前串口事务')
$end = $vision.IndexOf('    /* 3) READY必须', $start)
$functions += "static void block_complete_io(mission_context_t *ctx) { nano_vision_status_t status;`n" + $vision.Substring($start, $end - $start) + "}`n"
$functions += "#if TEST_MODE`n" + (Get-Function 'mission_test_run_block_move') + "`n#endif"
[IO.File]::WriteAllText((Join-Path $build 'block_move_under_test.inc'), ($functions -join "`n"), [Text.UTF8Encoding]::new($false))
$protocol = [IO.File]::ReadAllText((Join-Path $root '02_Service/chassis_bridge/chassis_mission_link.h'))
$start = $protocol.IndexOf('typedef uint8_t mission_command_type_t;')
$end = $protocol.IndexOf('/** Mission写入', $start)
$commands = $protocol.Substring($start, $end - $start)
$start = $protocol.IndexOf('typedef struct {', $end)
$end = $protocol.IndexOf('} chassis_mission_event_t;', $start) + '} chassis_mission_event_t;'.Length
[IO.File]::WriteAllText((Join-Path $build 'block_move_protocol.inc'), ($commands + $protocol.Substring($start, $end - $start)), [Text.UTF8Encoding]::new($false))
foreach ($mode in @(0, 1)) {
    foreach ($trace in @(0, 1)) {
        $binary = Join-Path $build "test_block_move_${mode}_$trace.exe"
        gcc -std=c11 -Wall -Wextra -Werror "-DTEST_MODE=$mode" "-DTEST_TRACE=$trace" `
            -I $build -I (Join-Path $root '01_App/mission') `
            -I (Join-Path $root '03_Middleware/nano_vision') `
            -I (Join-Path $root '03_Middleware/ball_manifest') `
            -I (Join-Path $root '03_Middleware/lsc16') `
            (Join-Path $PSScriptRoot 'test_block_move.c') `
            (Join-Path $root '03_Middleware/nano_vision/nano_vision_core.c') `
            (Join-Path $root '03_Middleware/ball_manifest/ball_manifest_core.c') -o $binary
        if ($LASTEXITCODE -ne 0) { throw 'Block move compile failed' }
        & $binary
        if ($LASTEXITCODE -ne 0) { throw 'Block move regression failed' }
    }
}
