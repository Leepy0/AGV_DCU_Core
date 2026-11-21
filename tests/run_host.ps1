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
              'SEQ_TAPE', 'SEQ_OPT', 'SEQ_OLD5', 'SEQ_SPEED', 'MMIN', 'TAG_NOWAIT', 'TAG_KIND2', 'TAG_OBS2', 'TAG_WORD', 'TAG_OLD6', 'SEQSTART_OPT', 'SEQSTART_ORDER', 'SEQ_KIND2', 'MOVE_LIMIT', 'MOVE_OPT', 'MOVE_MAX',
              'STOP_NEG', 'STOP_MAX', 'STOP_WORD', 'SEQSTART_MAX', 'SEQ_AUTO16', 'SEQ_FB16', 'TAG_AUTO16', 'TAG_FB16', 'TAG_DEG47', 'TAG_DEG320','BELT_OPT', 'BELT_DIR', 'BELT_UNTIL_NODIR',
              'TAGNUM_PLAIN', 'TAGNUM_N', 'MOD_N', 'MOD_M0', 'SEND_N', 'SEND_MOD', 'SEND_TAG', 'SEND_BIG', 'PARAM_NAME', 'RANGE_ORDER', 'RANGE_MAX',
              'SEQ_SEND_N', 'FS_TICKS', 'FS_TICK0', 'FS_CM', 'FS_IN', 'FS_TAPE') {
    & cmd /c "gcc -std=c99 -DAGV_HOST_BUILD -Iinclude -DNEG_$n -fsyntax-only tests/test_dsl.c >nul 2>nul"
    if ($LASTEXITCODE -eq 0) { Write-Output "NEG_$n compiled"; $failed++ }
}
# DSL unification (v0.58.0, tests/dsl): new forms (AND, AGV_SPEED, ACS, WAIT_TO PARAM_TIME, TAG_IF) must give the same bytes as the old forms,
# and each -DNEG_* case must fail to compile (calling the build twice: with and without the NEG guard)
foreach ($n in 'and_seq', 'and_tag', 'tagif', 'speed_seq', 'speed_tag', 'acs_seq', 'acs_tag', 'wait_to', 'checks_seq', 'checks_tag', 'mix_seq', 'mix_tag', 'man_and', 'site_words_seq', 'site_words_tag', 'paren_gcc', 'and_neg_seq', 'and_neg_tag', 'and_neg_site', 'hold_seq', 'hold_tag', 'man_set') {
    & cmd /c "gcc -std=c99 -DAGV_HOST_BUILD -Iinclude -Itests/dsl tests/dsl/$n.c -o build/t/dsl_$n.exe 2>nul"
    if ($LASTEXITCODE -ne 0) { Write-Output "BUILD FAIL dsl_$n"; $failed++ }
    else { & "build/t/dsl_$n.exe" | Out-Null; if ($LASTEXITCODE -ne 0) { Write-Output "FAIL dsl_$n"; $failed++ } }
}
$dslNeg = [ordered]@{
    'and_neg_seq' = 'ANY_MERGE1 ANY_MERGE2 ANY_MERGE3 SEQ_2STATE SEQ_STATE_ANY2 ANY3 STATE3 POLARITY ANYDUP UNKNOWN EMPTY AMP SEVEN STATION_ARITY STATION_BAD_K STATE_ANY2B NOT_INPUT NOT_NOT PARAM_MIN_PLACE WAITSTABLE_STATE WAITSTABLE_ANY2 ERRHOLD_STATE FLICKER_ANY2 FLICKER_2STATE FLICKER_PERIOD STATIONK ALL_PARAM_ANDSTATE'
    'and_neg_tag' = 'TAG_3STATE TAG_ANY2 TAG_CELLS TAG_RANGE_ANY2 TAG_RANGE_CELLS TAG_AUTOSTART TAG_X1 TAG_X2 TAG_RANGE_X1'
    'and_neg_site' = 'SITE_NUM SITE_PAREN SITE_IDENT SITE_ID'
    'checks_seq' = 'WHEN_ANY2 IN_ANY2 IN_ANY2B WAITTO_ANY2 SELTO0 SELTO_BIG TAGNUM_WHEN_IF TAGNUM_WAIT_IF TAGNUM_WAIT_IF_IN_TO TAGNUM_AND HANDSHAKE_UNMATCHED FLICKER_UNMATCHED'
    'checks_tag' = 'AUTOSTART AUTOSTART300 CALL ACT CELLS5 CELLS_TURN CELLS_IF CELLS_RANGE SEQSTEP_OUT SEQSTEP_DELAY SEQSTEP_WAIT SEQSTEP_LABEL SEQSTEP_GOTO ANY2_IN ANY2_IN2 SEQSTEP_OUTIF SEQSTEP_HANDSHAKE SEQSTEP_WHEN AND_ANY2 AND_3STATE'
    'acs_seq' = 'ST1_PLUS1 ST0_PLUS2 ST0_MINUS1 MOD BIG NEG STK BADWORD AGVMINUS TAGNUM CONST'
    'acs_tag' = '256 MOD NEG STN129 RESET_ST0 BADWORD AGVMINUS256'
    'speed_seq' = '0 3 256 WORD NEG LOWER EMPTY'
    'speed_tag' = '0 3 256 WORD NEG LOWER EMPTY'
    'man_and' = 'MAN_STATE MAN_STATE2 MAN_KEY2 MAN_ANY2'
    'paren_gcc' = 'ACS_PAREN SPEED_PAREN'
    'wait_to' = 'UNMATCHED BIG NEG STATE_PARAM ANY2_PARAM UNIT PARAM0 FND PLACE PTM_ALONE PCOUNT UNMATCHED_AND'
    'hold_seq' = 'TAG_ACCUM TAG_ACCUM_TO TAG_WHEN_STABLE TAG_WHEN_ACCUM TAG_WHEN_OUT ACCUM_STATE ACCUM_ANY2 ACCUM_BIG ACCUM_NEG HOLD_MIN HOLD_PLACE HOLD_PCOUNT HOLD_FND HOLD_STATE_PARAM WHENHOLD_STATE WHENHOLD_ANY2 WHENHOLD_BIG WHENHOLD_MIN WHENOUT_STATE STATE_SLOT STATE_ORDER STATE_BIG STATE_NEG STATE_TWO STATE_ARITY STATE_STABLE STATE_MAN'
    'hold_tag' = 'TAG_STATE3 TAG_STATE_SLOT TAG_DISPLAY_BIG'
    'man_set' = 'SET_16 SET_NEG SET_PLAIN SET_PF0 SET_PF64 SET_EMPTY SET_17 SET_KEY SET_64 ANY_LOWER FL_0 FL_64 FL_TWICE FL_SET FL_MAN FL_TAG FL_TAGIF FL_TAGRANGE FL_TAGRANGEIF FL_TAGP FL_SEQMAP TAG_64 MAP_64'
}
foreach ($f in $dslNeg.Keys) {
    foreach ($n in $dslNeg[$f] -split ' ') {
        & cmd /c "gcc -std=c99 -DAGV_HOST_BUILD -Iinclude -Itests/dsl -DNEG -DNEG_$n -fsyntax-only tests/dsl/$f.c >nul 2>nul"
        if ($LASTEXITCODE -eq 0) { Write-Output "dsl $f NEG_$n compiled"; $failed++ }
    }
}
foreach ($n in 'MAN_SETCLR', 'MAN_BELT_NODIR', 'MAN_PROFILE_0', 'MAN_PROFILE_RANGE', 'MAN_START_PROFILE_0', 'MAN_START_PROFILE_RANGE', 'MAN_START_ID', 'MAN_LOAD_EMPTY', 'MAN_LOAD_NAME') {
    & cmd /c "gcc -std=gnu99 -DAGV_HOST_BUILD -Iinclude -Ihal/host -DNEG_$n -fsyntax-only tests/test_manual.c >nul 2>nul"
    if ($LASTEXITCODE -eq 0) { Write-Output "NEG_$n compiled"; $failed++ }
}

# Both builds: with the tuning t3 dev tests (default) and without (AGV_DEV_TESTS=0, site images)
# t1 steering gain search: judge and candidate order (gain.c only)
& gcc -std=c99 -Wall -DAGV_HOST_BUILD -Iinclude tests/test_gain.c src/gain.c -o build/t/test_gain.exe -lm
if ($LASTEXITCODE -ne 0) { Write-Output 'BUILD FAIL gain'; $failed++ } else { & build/t/test_gain.exe; if ($LASTEXITCODE -ne 0) { Write-Output 'FAIL gain'; $failed++ } }
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
    # Packed tag table re-entrancy (user function calls tagact_run again); manual table and profile load conditions
    foreach ($t in 'tagpk', 'manual') {
        & gcc -std=gnu99 -Wall -DAGV_HOST_BUILD $dev -Iinclude -Ihal/host "tests/test_$t.c" @sys -o "build/t/test_$t$sfx.exe" -lm
        if ($LASTEXITCODE -ne 0) { Write-Output "BUILD FAIL $t$sfx"; $failed++ }
        else {
            & "build/t/test_$t$sfx.exe"
            if ($LASTEXITCODE -ne 0) { Write-Output "FAIL $t$sfx"; $failed++ }
        }
    }
}
if ($failed -ne 0) { Write-Output "FAILED: $failed"; exit 1 }
Write-Output 'ALL PASSED'
