# Target build: powershell -File tools/build_target.ps1 [-Site examples/empty_site] [-Out build/c28/agv_empty]
param([string]$Site = 'examples/empty_site', [string]$Out = 'build/c28/agv_empty')
$ErrorActionPreference = 'Stop'
Set-Location (Split-Path -Parent $PSScriptRoot)

$cgt = 'C:\ti\ti-cgt-c2000_18.12.2.LTS\ti-cgt-c2000_18.12.2.LTS'
$cl = "$cgt\bin\cl2000.exe"
$flags = '-v28', '-ml', '-mt', '--c99', '--float_support=softlib', '-O2', '--opt_for_speed=2', '-g',
         '--diag_warning=225', '--display_error_number', "-I$cgt\include", '-Iinclude', '-Ihal/f2808', "-I$Site"
$objdir = 'build/c28'
New-Item -ItemType Directory -Force $objdir | Out-Null

$src = @(Get-ChildItem src/*.c, src/sys/*.c, hal/f2808/*.c, hal/f2808/*.asm, "$Site/*.c")
$objs = @()
$failed = 0
foreach ($f in $src) {
    $o = Join-Path $objdir ($f.BaseName + '.obj')
    & $cl @flags "--output_file=$o" -c $f.FullName
    if ($LASTEXITCODE -ne 0) { $failed++ }
    $objs += $o
}
if ($failed -ne 0) { Write-Output "COMPILE FAILED: $failed"; exit 1 }

& $cl -v28 -ml -mt --float_support=softlib -z "--output_file=$Out.out" "--map_file=$Out.map" `
    --stack_size=0x400 --heap_size=0 --rom_model --warn_sections --reread_libs --entry_point=code_start `
    "-i$cgt\lib" @objs cmd/F2808_app_direct.cmd "--library=rts2800_ml.lib"
if ($LASTEXITCODE -ne 0) { Write-Output 'LINK FAILED'; exit 1 }
Write-Output "linked $Out.out"
