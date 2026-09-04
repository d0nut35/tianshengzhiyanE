$ErrorActionPreference = "Stop"

& (Join-Path $PSScriptRoot "mult_uart\run_mult_uart_fake_tests.ps1")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $PSScriptRoot "lsc16\run_lsc16_fake_tests.ps1")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $PSScriptRoot "ic_card\run_ic_card_fake_tests.ps1")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& (Join-Path $PSScriptRoot "zdt_turntable\run_zdt_turntable_fake_tests.ps1")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& (Join-Path $PSScriptRoot "nano_vision\run_nano_vision_fake_tests.ps1")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& (Join-Path $PSScriptRoot "ball_manifest\run_ball_manifest_fake_tests.ps1")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Output "All host fake tests passed."
