# Target build: powershell -File tools/build_target.ps1 [-Site examples/empty_site] [-Out build/c28/agv_empty]
#   [-ObjDir build/c28] [-CoreCache] [-CoreLib <path>] [-CoreOnly]
# Core objects go to <ObjDir>/core, site objects to <ObjDir>/site.
# -CoreCache reuses existing core objects. Use it only when the core tree cannot change for that ObjDir
#   (for example an app keys ObjDir by the hash of its vendored core). Site files are always compiled.
# -CoreLib archives the core objects into that library and links the site objects against it
#   (the same way an IDE project that links the core library does). -CoreOnly stops after the library.
param([string]$Site = 'examples/empty_site', [string]$Out = 'build/c28/agv_empty',
      [string]$ObjDir = 'build/c28', [switch]$CoreCache, [string]$CoreLib = '', [switch]$CoreOnly)
$ErrorActionPreference = 'Stop'
Set-Location (Split-Path -Parent $PSScriptRoot)

$cgt = 'C:\ti\ti-cgt-c2000_18.12.2.LTS\ti-cgt-c2000_18.12.2.LTS'
$cl = "$cgt\bin\cl2000.exe"
$flags = '-v28', '-ml', '-mt', '--c99', '--float_support=softlib', '-O2', '--opt_for_speed=2', '-g',
         '--diag_warning=225', '--display_error_number', "-I$cgt\include", '-Iinclude', '-Ihal/f2808'
$coreDir = Join-Path $ObjDir 'core'
$siteDir = Join-Path $ObjDir 'site'
New-Item -ItemType Directory -Force $coreDir, $siteDir | Out-Null

$failed = 0
function Compile($f, [string]$dir, [string[]]$extra) {
    $o = Join-Path $dir ($f.BaseName + '.obj')
    & $cl @flags @extra "--output_file=$o" -c $f.FullName | Out-Host
    if ($LASTEXITCODE -ne 0) {
        if (Test-Path $o) { Remove-Item -Force $o }
        $script:failed++
    }
    return $o
}

# The core does not include site headers, so its objects do not depend on the site
$coreObjs = @()
foreach ($f in @(Get-ChildItem src/*.c, src/sys/*.c, hal/f2808/*.c, hal/f2808/*.asm)) {
    $o = Join-Path $coreDir ($f.BaseName + '.obj')
    if ($CoreCache -and (Test-Path $o)) { $coreObjs += $o; continue }
    $coreObjs += Compile $f $coreDir @()
}
if ($failed -ne 0) { Write-Output "COMPILE FAILED: $failed"; exit 1 }

if ($CoreLib -ne '') {
    $libNew = -not (Test-Path $CoreLib)
    if (-not $libNew) {
        $libTime = (Get-Item $CoreLib).LastWriteTimeUtc
        $libNew = @($coreObjs | Where-Object { (Get-Item $_).LastWriteTimeUtc -gt $libTime }).Count -gt 0
    }
    if ($libNew) {
        if (Test-Path $CoreLib) { Remove-Item -Force $CoreLib }
        New-Item -ItemType Directory -Force (Split-Path -Parent $CoreLib) | Out-Null
        # ar2000 can wait on stdin; give it an empty one
        $null | & "$cgt\bin\ar2000.exe" -rq $CoreLib @coreObjs | Out-Host
        if ($LASTEXITCODE -ne 0) { Write-Output 'ARCHIVE FAILED'; exit 1 }
    }
    if ($CoreOnly) { Write-Output "core library $CoreLib"; exit 0 }
}

$siteObjs = @()
foreach ($f in @(Get-ChildItem "$Site/*.c")) {
    $siteObjs += Compile $f $siteDir @("-I$Site")
}
if ($failed -ne 0) { Write-Output "COMPILE FAILED: $failed"; exit 1 }

if ($CoreLib -ne '') { $link = @($siteObjs) + @('cmd/F2808_app_direct.cmd', "--library=$CoreLib") }
else { $link = @($coreObjs) + @($siteObjs) + @('cmd/F2808_app_direct.cmd') }
& $cl -v28 -ml -mt --float_support=softlib -z "--output_file=$Out.out" "--map_file=$Out.map" `
    --stack_size=0x400 --heap_size=0 --rom_model --warn_sections --reread_libs --entry_point=code_start `
    "-i$cgt\lib" @link "--library=rts2800_ml.lib"
if ($LASTEXITCODE -ne 0) { Write-Output 'LINK FAILED'; exit 1 }
Write-Output "linked $Out.out"
