# cl2000 으로 tests/dsl 을 시험한다(gcc 호스트 시험 run_host.ps1 과 따로, C28x 컴파일러가 있을 때만): powershell -File tests/dsl/cl2000_check.ps1
#  1 새 꼴·옛 꼴 표가 cl2000 이 만든 어셈블리 .bits 까지 같은가  2 -DNEG_* 가 cl2000 에서도 컴파일 오류인가
# 임시 파일은 build/dsl_cl2000/ 에 둔다
param([string]$Cgt = 'C:\ti\ti-cgt-c2000_18.12.2.LTS\ti-cgt-c2000_18.12.2.LTS')
$ErrorActionPreference = 'Continue'
$core = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$cl = "$Cgt\bin\cl2000.exe"
$d = Join-Path $core 'build\dsl_cl2000'
New-Item -ItemType Directory -Force $d | Out-Null
Set-Location $d
$fail = 0
$pairs = @{
 'and_seq' = @(,@('s_new_steps','s_old_steps'));
 'and_tag' = @(,@('t_new','t_old'));
 'tagif' = @(,@('t_new','t_old'));
 'speed_seq' = @(@('s_new_steps','s_old_steps'),@('s_mmin_steps','s_mmin_old_steps'));
 'speed_tag' = @(,@('t_new','t_old'));
 'acs_seq' = @(,@('s_new_steps','s_old_steps'));
 'acs_tag' = @(@('t_new','t_old'),@('t_one_new','t_one_old'));
 'wait_to' = @(,@('s_new_steps','s_old_steps'));
 'checks_seq' = @(,@('s_any2_steps','s_ref_steps'));
 'mix_seq' = @(,@('s_new_steps','s_old_steps'));
 'mix_tag' = @(,@('t_new','t_old'));
 'man_and' = @(,@('m_new','m_old'));
 'site_words_seq' = @(,@('s_new_steps','s_ref_steps'));
 'site_words_tag' = @(,@('t_new','t_ref'));
 'hold_seq' = @(,@('s_new_steps','s_ref_steps'));
 'hold_tag' = @(,@('t_new','t_ref'));
 'man_set' = @(,@('m_new','m_old'));
}
foreach ($n in $pairs.Keys | Sort-Object) {
  $src = "$core\tests\dsl\$n.c"
  $o = & $cl -v28 -ml -mt --abi=coffabi --c99 --compile_only --display_error_number -k "-I$core\include" "-I$core\tests\dsl" --output_file="$d\$n.obj" $src 2>&1 | ForEach-Object { "$_" }
  $errs = @($o | Where-Object { $_ -match 'error' })
  if ($errs.Count -gt 0) { "$n : cl2000 ERROR"; $fail++; $errs | Select-Object -First 3; continue }
  $asm = "$d\$n.asm"
  if (-not (Test-Path $asm)) { $asm = "$core\tests\dsl\$n.asm" }
  $text = [IO.File]::ReadAllText($asm)
  foreach ($p in $pairs[$n]) {
    function Grab($sym) {
      $m = [regex]::Match($text, "(?m)^_$sym`:\r?\n((?:(?!\.sect|^[A-Za-z_]).*\r?\n)*)")
      if (-not $m.Success) { return $null }
      return @($m.Groups[1].Value -split "\r?\n" | Where-Object { $_ -match '^\s*\.bits' } | ForEach-Object { ($_ -replace ';.*','').Trim() })
    }
    $a = Grab $p[0]; $b = Grab $p[1]
    if (-not $a -or -not $b) { "$n : asm label missing $($p[0]) $($p[1])"; $fail++; continue }
    if (($a -join "`n") -eq ($b -join "`n")) { "$n : cl2000 SAME $($p[0]) = $($p[1]) ($($a.Count) .bits)" } else { "$n : cl2000 DIFF $($p[0]) vs $($p[1])"; $fail++ }
  }
}

$neg = [ordered]@{
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
 'wait_to' = 'UNMATCHED BIG NEG STATE_PARAM ANY2_PARAM UNIT PARAM0 FND PLACE PTM_ALONE PCOUNT UNMATCHED_AND'
 'hold_seq' = 'TAG_ACCUM TAG_ACCUM_TO TAG_WHEN_STABLE TAG_WHEN_ACCUM TAG_WHEN_OUT ACCUM_STATE ACCUM_ANY2 ACCUM_BIG ACCUM_NEG HOLD_MIN HOLD_PLACE HOLD_PCOUNT HOLD_FND HOLD_STATE_PARAM WHENHOLD_STATE WHENHOLD_ANY2 WHENHOLD_BIG WHENHOLD_MIN WHENOUT_STATE STATE_SLOT STATE_ORDER STATE_BIG STATE_NEG STATE_TWO STATE_ARITY STATE_STABLE STATE_MAN'
 'hold_tag' = 'TAG_STATE3 TAG_STATE_SLOT TAG_DISPLAY_BIG'
 'man_set' = 'SET_16 SET_NEG SET_PLAIN SET_PF0 SET_PF64 SET_EMPTY SET_17 SET_KEY SET_64'
}
$ok = 0
foreach ($f in $neg.Keys) {
  foreach ($n in $neg[$f] -split ' ') {
    $o = & $cl -v28 -ml -mt --abi=coffabi --c99 --compile_only --display_error_number -DNEG "-DNEG_$n" "-I$core\include" "-I$core\tests\dsl" --output_file="$d\neg_tmp.obj" "$core\tests\dsl\$f.c" 2>&1 | ForEach-Object { "$_" }
    $err = @($o | Where-Object { $_ -match '\berror\b' -and $_ -notmatch 'last line' })
    if ($err.Count -eq 0) { "cl2000 COMPILED: $f NEG_$n"; $fail++ } else { $ok++ }
  }
}
"cl2000 negative ok=$ok"

if ($fail -ne 0) { "cl2000 실패 $fail"; exit 1 }
"cl2000 통과"
