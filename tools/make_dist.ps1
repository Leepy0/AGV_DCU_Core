# Distribution bundle: powershell -ExecutionPolicy Bypass -File tools/make_dist.ps1 -Ver v0.55.0 [-NoPush] [-Trailer <commit trailer>]
# Checks (clean tree, HEAD = tag Ver, agv/version.h name = Ver, host tests pass), builds the two core libraries with
# tools/build_target.ps1 -CoreOnly (lib/core.lib with the dev tests, lib/core_dev0.lib without), assembles the bundle,
# commits it to the orphan branch 'dist' through a git worktree (the master working tree is not touched), tags it
# dist-<Ver> and pushes the branch and the tag. If dist-<Ver> already exists locally it only pushes (resume after -NoPush).
# Bundle: VERSION, MANIFEST.sha256, README.md, include/agv/*.h, lib/, cmd/*.cmd, tools/build_site.ps1.
param([Parameter(Mandatory = $true)][string]$Ver, [switch]$NoPush, [string]$Trailer = '')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$tag = "dist-$Ver"

function Run([string]$what) {      # git and other native commands: stop on a non-zero exit code
    if ($LASTEXITCODE -ne 0) { throw "$what failed ($LASTEXITCODE)" }
}
function Push-Dist {
    & git push origin dist; Run 'git push dist'
    & git push origin $tag; Run "git push $tag"
    Write-Output "pushed dist, $tag"
}

if (@(& git tag --list $tag).Count -ne 0) {
    Write-Output "$tag exists"
    if (-not $NoPush) { Push-Dist }
    exit 0
}

# 1. Checks
if (@(& git status --porcelain).Count -ne 0) { throw 'working tree is not clean' }
$head = (& git rev-parse HEAD).Trim(); Run 'git rev-parse HEAD'
$want = (& git rev-parse "$Ver^{commit}").Trim(); Run "tag $Ver"
if ($head -ne $want) { throw "HEAD is not $Ver" }
$h = [IO.File]::ReadAllText((Join-Path $root 'include/agv/version.h'))
$m = [regex]::Match($h, '(?m)^\s*#\s*define\s+AGV_CORE_VER\s+(\S+)')
if (-not $m.Success -or ($m.Groups[1].Value -ne ($Ver -replace '\.', '_'))) { throw "agv/version.h AGV_CORE_VER is not $($Ver -replace '\.', '_')" }
$t = @(& cmd /c "powershell -ExecutionPolicy Bypass -File tests\run_host.ps1 2>&1")
if ("$($t | Select-Object -Last 1)" -ne 'ALL PASSED') { $t | Select-Object -Last 10; throw 'host tests failed' }
Write-Output 'checks ok'

# 2. Libraries (same options as build_target.ps1 -CoreOnly)
$stage = Join-Path $root "build\dist\$Ver"
$obj = Join-Path $root 'build\dist_obj'
$dev0 = Join-Path $root 'build\dist_dev0'
foreach ($d in $stage, $obj, $dev0) { if (Test-Path $d) { Remove-Item -Recurse -Force $d } }
New-Item -ItemType Directory -Force $stage, $dev0 | Out-Null
[IO.File]::WriteAllText((Join-Path $dev0 'site.h'), "#define AGV_DEV_TESTS 0`n")
foreach ($a in @("-ObjDir `"$obj`" -CoreLib `"$stage\lib\core.lib`" -CoreOnly",
                 "-Site `"$dev0`" -ObjDir `"$obj`" -CoreLib `"$stage\lib\core.lib`" -CoreOnly")) {
    $o = @(& cmd /c "powershell -ExecutionPolicy Bypass -File tools\build_target.ps1 $a 2>&1")
    if ($LASTEXITCODE -ne 0) { $o | Select-Object -Last 10; throw 'library build failed' }
}
if (-not (Test-Path "$stage\lib\core_dev0.lib")) { throw 'lib/core_dev0.lib missing' }

# 3. Bundle
$cl = 'C:\ti\ti-cgt-c2000_18.12.2.LTS\ti-cgt-c2000_18.12.2.LTS\bin\cl2000.exe'
$cver = ([regex]::Match((& cmd /c "`"$cl`" -version 2>&1" | Select-Object -First 1), 'v[0-9][0-9.]*\.(LTS|STS)')).Value
$short = (& git rev-parse --short HEAD).Trim()
New-Item -ItemType Directory -Force "$stage\include\agv", "$stage\cmd", "$stage\tools" | Out-Null
Copy-Item include\agv\*.h "$stage\include\agv"
Copy-Item cmd\*.cmd "$stage\cmd"
Copy-Item tools\build_site.ps1 "$stage\tools"
Copy-Item tools\dist_README.md "$stage\README.md"
[IO.File]::WriteAllText("$stage\VERSION", "$Ver $short cl2000 $cver`n")

# MANIFEST: "sha256  path" per file, path order. Text files hash with LF line ends, .lib as is.
# The app checks the bundle with the same rule (its tools/core_manifest.ps1)
$latin = [Text.Encoding]::GetEncoding(28591)
$sha = [Security.Cryptography.SHA256]::Create()
$lines = @()
$base = (Resolve-Path $stage).Path.TrimEnd('\') + '\'
foreach ($f in Get-ChildItem -Recurse -File $stage) {
    $rel = $f.FullName.Substring($base.Length).Replace('\', '/')
    $b = [IO.File]::ReadAllBytes($f.FullName)
    if ($f.Extension.ToLower() -ne '.lib') { $b = $latin.GetBytes($latin.GetString($b).Replace("`r`n", "`n")) }
    $lines += (-join ($sha.ComputeHash($b) | ForEach-Object { $_.ToString('x2') })) + '  ' + $rel
}
$keys = [string[]]($lines | ForEach-Object { $_.Substring(66) })
$arr = [string[]]$lines
[Array]::Sort($keys, $arr, [StringComparer]::Ordinal)
[IO.File]::WriteAllText("$stage\MANIFEST.sha256", ($arr -join "`n") + "`n")
Write-Output "bundle $stage"

# 4. Commit to the orphan branch 'dist' in a worktree, then tag
$wt = Join-Path $root 'build\dist_wt'
if (Test-Path $wt) { & git worktree remove --force $wt; Run 'git worktree remove' }
& git worktree prune
if (@(& git branch --list dist).Count -eq 0) {
    & cmd /c "git fetch -q origin dist:dist 2>nul"              # fails when there is no remote dist yet
}
if (@(& git branch --list dist).Count -ne 0) {
    & git worktree add $wt dist; Run 'git worktree add'
}
else {
    & git worktree add --detach $wt; Run 'git worktree add'
    & git -C $wt checkout -q --orphan dist; Run 'git checkout --orphan'
}
& git -C $wt rm -r -q -f --ignore-unmatch -- .; Run 'git rm'
Get-ChildItem -Force $wt | Where-Object { $_.Name -ne '.git' } | Remove-Item -Recurse -Force
Copy-Item -Recurse "$stage\*" $wt
& git -C $wt add -f -- VERSION MANIFEST.sha256 README.md include lib cmd tools; Run 'git add'
$msg = @('-m', "dist $Ver (source $short)")
if ($Trailer -ne '') { $msg += @('-m', $Trailer) }
& git -C $wt commit -q @msg; Run 'git commit'
& git -C $wt tag -a $tag -m "core $Ver bundle (source $short)"; Run 'git tag'
$dc = (& git -C $wt rev-parse --short HEAD).Trim()
& git worktree remove $wt; Run 'git worktree remove'
Write-Output "dist $dc, tag $tag"
if (-not $NoPush) { Push-Dist }
