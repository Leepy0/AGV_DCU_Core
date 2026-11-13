# Host unit tests: powershell -File tests/run_host.ps1
$ErrorActionPreference = 'Stop'
Set-Location (Split-Path -Parent $PSScriptRoot)
New-Item -ItemType Directory -Force build/t | Out-Null

$src = Get-ChildItem src/*.c | ForEach-Object { $_.FullName }
$flags = '-std=c99', '-Wall', '-DAGV_HOST_BUILD', '-Iinclude', '-Itests'
$failed = 0

& gcc -std=c99 -DAGV_HOST_BUILD -Iinclude tests/test_dsl.c -o build/t/test_dsl.exe
if ($LASTEXITCODE -ne 0) { $failed++ } else { & build/t/test_dsl.exe; if ($LASTEXITCODE -ne 0) { $failed++ } }
# Compile-error tests: each NEG_* in test_dsl.c must fail to compile
foreach ($n in 'AMP_INPUT', 'NO_DIRECTION', 'PARAM_ZERO', 'MOD_ZERO', 'HIGH_ANY', 'TURN_SPEED', 'ACS1_RESET', 'PIN_TWICE', 'PIN_ANY', 'PIN_ONE',
              'SEQ_TAPE', 'SEQ_OPT', 'SEQ_OLD5', 'SEQ_SPEED', 'MMIN', 'TAG_NOWAIT', 'TAG_KIND2', 'TAG_OBS2', 'TAG_WORD', 'TAG_OLD6', 'SEQSTART_OPT', 'SEQSTART_ORDER', 'SEQ_KIND2', 'MOVE_LIMIT', 'MOVE_OPT') {
    & cmd /c "gcc -std=c99 -DAGV_HOST_BUILD -Iinclude -DNEG_$n -fsyntax-only tests/test_dsl.c >nul 2>nul"
    if ($LASTEXITCODE -eq 0) { Write-Output "NEG_$n compiled"; $failed++ }
}

# Both builds: with the tuning t3 dev tests (default) and without (AGV_DEV_TESTS=0, site images)
foreach ($dv in 1, 0) {
    $dev = "-DAGV_DEV_TESTS=$dv"
    $sfx = if ($dv -eq 0) { '_dev0' } else { '' }
    if ($dv -eq 0) { Write-Output '-- AGV_DEV_TESTS=0' }
    foreach ($t in 'frame', 'acs', 'param', 'seq', 'motor', 'stop', 'steer', 'guide') {
        & gcc @flags $dev "tests/test_$t.c" tests/fakes.c @src -o "build/t/test_$t$sfx.exe" -lm
        if ($LASTEXITCODE -ne 0) { Write-Output "BUILD FAIL $t$sfx"; $failed++; continue }
        & "build/t/test_$t$sfx.exe"
        if ($LASTEXITCODE -ne 0) { Write-Output "FAIL $t$sfx"; $failed++ }
    }
    # Assembly test: real upper modules on the host HAL, without fakes
    $sys = @($src) + (Get-ChildItem src/sys/*.c, hal/host/*.c | ForEach-Object { $_.FullName })
    & gcc -std=gnu99 -Wall -DAGV_HOST_BUILD $dev -Iinclude -Ihal/host tests/test_sys.c @sys -o "build/t/test_system$sfx.exe" -lm
    if ($LASTEXITCODE -ne 0) { Write-Output "BUILD FAIL sys$sfx"; $failed++ }
    else {
        & "build/t/test_system$sfx.exe"
        if ($LASTEXITCODE -ne 0) { Write-Output "FAIL sys$sfx"; $failed++ }
    }
    # Closed-loop curve simulation on the same full stack
    & gcc -std=gnu99 -Wall -DAGV_HOST_BUILD $dev -Iinclude -Ihal/host tests/test_curve.c @sys -o "build/t/test_curve$sfx.exe" -lm
    if ($LASTEXITCODE -ne 0) { Write-Output "BUILD FAIL curve$sfx"; $failed++ }
    else {
        & "build/t/test_curve$sfx.exe"
        if ($LASTEXITCODE -ne 0) { Write-Output "FAIL curve$sfx"; $failed++ }
    }
    # Packed tag table re-entrancy (user function calls tagact_run again)
    & gcc -std=gnu99 -Wall -DAGV_HOST_BUILD $dev -Iinclude -Ihal/host tests/test_tagpk.c @sys -o "build/t/test_tagpk$sfx.exe" -lm
    if ($LASTEXITCODE -ne 0) { Write-Output "BUILD FAIL tagpk$sfx"; $failed++ }
    else {
        & "build/t/test_tagpk$sfx.exe"
        if ($LASTEXITCODE -ne 0) { Write-Output "FAIL tagpk$sfx"; $failed++ }
    }
}
if ($failed -ne 0) { Write-Output "FAILED: $failed"; exit 1 }
Write-Output 'ALL PASSED'
