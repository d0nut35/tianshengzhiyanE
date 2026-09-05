$ErrorActionPreference = 'Stop'
$root = (Resolve-Path "$PSScriptRoot\..\..\..").Path
$build = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$exe = Join-Path $build 'test_zdt_turntable_core.exe'
& gcc -std=c11 -Wall -Wextra -Werror -DTURN_BSP_HAL_ENABLE=0 `
    -I (Join-Path $root '04_Bsp\zdt_turntable') `
    (Join-Path $root '04_Bsp\zdt_turntable\turn_bsp.c') `
    (Join-Path $PSScriptRoot 'test_zdt_turntable_core.c') `
    -o $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$serviceExe = Join-Path $build 'test_zdt_turntable_service.exe'
& gcc -std=c11 -Wall -Wextra -Werror -DTURN_BSP_HAL_ENABLE=0 `
    -I (Join-Path $PSScriptRoot 'fakes') `
    -I (Join-Path $root '04_Bsp\zdt_turntable') `
    -I (Join-Path $root '02_Service\zdt_turntable') `
    (Join-Path $root '04_Bsp\zdt_turntable\turn_bsp.c') `
    (Join-Path $root '02_Service\zdt_turntable\zdt_turntable_service.c') `
    (Join-Path $PSScriptRoot 'test_zdt_turntable_service.c') `
    -o $serviceExe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $serviceExe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$deviceExe = Join-Path $build 'test_zdt_turntable_device.exe'
& gcc -std=c11 -Wall -Wextra -Werror -DTURN_BSP_HAL_ENABLE=0 `
    -I (Join-Path $root '04_Bsp\zdt_turntable') `
    -I (Join-Path $root '02_Service\zdt_turntable') `
    -I (Join-Path $root '01_App\zdt_turntable_device') `
    (Join-Path $root '04_Bsp\zdt_turntable\turn_bsp.c') `
    (Join-Path $root '01_App\zdt_turntable_device\zdt_turntable_device.c') `
    (Join-Path $PSScriptRoot 'test_zdt_turntable_device.c') `
    -o $deviceExe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $deviceExe
exit $LASTEXITCODE
