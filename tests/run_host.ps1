# Host unit tests: powershell -File tests/run_host.ps1
$ErrorActionPreference = 'Stop'
Set-Location (Split-Path -Parent $PSScriptRoot)
New-Item -ItemType Directory -Force build/t | Out-Null

$src = Get-ChildItem src/*.c | ForEach-Object { $_.FullName }
$flags = '-std=c99', '-Wall', '-DAGV_HOST_BUILD', '-Iinclude', '-Itests'
$failed = 0

& gcc -std=c99 -DAGV_HOST_BUILD -Iinclude tests/test_dsl.c -o build/t/test_dsl.exe
if ($LASTEXITCODE -ne 0) { $failed++ } else { & build/t/test_dsl.exe; if ($LASTEXITCODE -ne 0) { $failed++ } }

foreach ($t in 'frame', 'acs', 'param', 'seq', 'motor', 'stop', 'steer', 'guide') {
    & gcc @flags "tests/test_$t.c" tests/fakes.c @src -o "build/t/test_$t.exe" -lm
    if ($LASTEXITCODE -ne 0) { Write-Output "BUILD FAIL $t"; $failed++; continue }
    & "build/t/test_$t.exe"
    if ($LASTEXITCODE -ne 0) { Write-Output "FAIL $t"; $failed++ }
}
# Assembly test: real upper modules on the host HAL, without fakes
$sys = @($src) + (Get-ChildItem src/sys/*.c, hal/host/*.c | ForEach-Object { $_.FullName })
& gcc -std=gnu99 -Wall -DAGV_HOST_BUILD -Iinclude -Ihal/host tests/test_sys.c @sys -o build/t/test_system.exe -lm
if ($LASTEXITCODE -ne 0) { Write-Output 'BUILD FAIL sys'; $failed++ }
else {
    & build/t/test_system.exe
    if ($LASTEXITCODE -ne 0) { Write-Output 'FAIL sys'; $failed++ }
}
if ($failed -ne 0) { Write-Output "FAILED: $failed"; exit 1 }
Write-Output 'ALL PASSED'
