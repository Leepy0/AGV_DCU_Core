# Site build against the core library: powershell -File tools/build_site.ps1 -Site <site folder> -Out <path without .out>
#   [-ObjDir <folder for site objects>] [-SiteDef NAME=1,NAME2=0] [-Lib <core library>]
# Runs inside the distribution bundle (no core sources): include/, cmd/, lib/ next to this tools/ folder.
# Same compile options, link order and linker command file as tools/build_target.ps1 with -CoreLib.
# The library follows the site: '#define AGV_DEV_TESTS 0' in site.h picks lib/core_dev0.lib, otherwise lib/core.lib.
# A site compiled with headers of another core version fails to link (agv/version.h renames site_register).
param([Parameter(Mandatory = $true)][string]$Site, [Parameter(Mandatory = $true)][string]$Out,
      [string]$ObjDir = '', [string]$SiteDef = '', [string]$Lib = '',
      [string]$Cgt = 'C:\ti\ti-cgt-c2000_18.12.2.LTS\ti-cgt-c2000_18.12.2.LTS')
$ErrorActionPreference = 'Stop'
$Site = (Resolve-Path $Site).Path
$Out = [IO.Path]::GetFullPath($Out)
if ($ObjDir -eq '') { $ObjDir = "$Out.obj" }
$ObjDir = [IO.Path]::GetFullPath($ObjDir)
if ($Lib -ne '') { $Lib = (Resolve-Path $Lib).Path }
Set-Location (Split-Path -Parent $PSScriptRoot)

$cl = "$Cgt\bin\cl2000.exe"
$flags = '-v28', '-ml', '-mt', '--c99', '--float_support=softlib', '-O2', '--opt_for_speed=2', '-g',
         '--diag_warning=225', '--display_error_number', "-I$Cgt\include", '-Iinclude'

# The bundle records the compiler that built the library
if (Test-Path VERSION) {
    $want = ((Get-Content VERSION) -split '\s+' | Where-Object { $_ -like 'v*.LTS' -or $_ -like 'v*.STS' } | Select-Object -Last 1)
    $have = ([regex]::Match((& cmd /c "`"$cl`" -version 2>&1" | Select-Object -First 1), 'v[0-9][0-9.]*\.(LTS|STS)')).Value
    if ($want -and ($have -ne $want)) { Write-Warning "cl2000 $have, but the core library was built with $want" }
}

$devTests = 1
$m = [regex]::Match([IO.File]::ReadAllText((Join-Path $Site 'site.h')), '(?m)^\s*#\s*define\s+AGV_DEV_TESTS\s+(\d+)')
if ($m.Success) { $devTests = [int]$m.Groups[1].Value }
$devFlag = @()
if ($devTests -eq 0) { $devFlag = @('-DAGV_DEV_TESTS=0') }
if ($Lib -eq '') { $Lib = (Resolve-Path $(if ($devTests -eq 0) { 'lib/core_dev0.lib' } else { 'lib/core.lib' })).Path }
New-Item -ItemType Directory -Force $ObjDir, (Split-Path -Parent $Out) | Out-Null

$failed = 0
$siteObjs = @()
$extra = @("-I$Site") + $devFlag + @($SiteDef -split ',' | Where-Object { $_ } | ForEach-Object { "-D$_" })
foreach ($f in @(Get-ChildItem "$Site/*.c")) {
    $o = Join-Path $ObjDir ($f.BaseName + '.obj')
    & $cl @flags @extra "--output_file=$o" -c $f.FullName | Out-Host
    if ($LASTEXITCODE -ne 0) {
        if (Test-Path $o) { Remove-Item -Force $o }
        $failed++
    }
    $siteObjs += $o
}
if ($failed -ne 0) { Write-Output "COMPILE FAILED: $failed"; exit 1 }

$link = @($siteObjs) + @('cmd/F2808_app_direct.cmd', "--library=$Lib")
& $cl -v28 -ml -mt --float_support=softlib -z "--output_file=$Out.out" "--map_file=$Out.map" `
    --stack_size=0x400 --heap_size=0 --rom_model --warn_sections --reread_libs --entry_point=code_start `
    "-i$Cgt\lib" @link "--library=rts2800_ml.lib"
if ($LASTEXITCODE -ne 0) { Write-Output 'LINK FAILED'; exit 1 }
Write-Output "linked $Out.out"
