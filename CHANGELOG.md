# 변경 이력

## 미릴리스 (2026-11-08)
- 지정 시험(TEMP) 단계 3 타력 시험: 스윕 뒤 OPT 를 되돌리고 4회(방향 번갈아) 잰다. 1·2회 속도 단계 2, 3·4회 최고 속도.
  가속 → 안정 0.5 s → 두 DAC 0 고정(순서 7, 조향 꺼짐)으로 원시 rpm 을 매 틱 잰다. 최저 속도·3 s·감속 끝점·가이드 이탈에서 풀고
  지령을 실제 속도(최저 속도 상한)에서 이어 최저 속도로 선을 찾아 돈다. 타력 시작 한계는 1000 rpm/s 어림 거리로 둔다.
  결과 `agv_coast`(v0, 90→30·90→60·60→30% 감속도, 최저 속도까지 시간·거리, 20 ms 곡선 64칸)도 `test_keep` 에 둔다.
  `agv_all.stage` 3, 실패 코드 3000 + 회 × 10 + 순서. STOP·E-STOP·오류·패널 키는 타력 중 바로 고정을 푼다(`ident_coast_key`).
  주행 중 DAC 고정은 진행 방향을 그대로 둔다.
- 지정 시험(TEMP): 튜닝 대기 중 SHIFT 로 고른다(TUNE → IDEN → RPM → TOP → ALL → TEMP). 0회(선 찾기, L)만 한 뒤
  통합 시험 단계 2 와 같은 OPT 스윕을 TEMP 목록(기본 {0,3,19,26,0,3,19,26}, `ident_temp_set`)으로 하고 단계 2 끝에서 멈춘다.
  식별 1~6회·최고 속도 없음. 결과 `agv_all`(stage 2). FND `T`+단계·회·순서, `TMOK`/`F`+코드.
- 옆 계단: 응답을 정속 평균에서 잰다(목표 = 평균 + 4칸). 치우침이 있으면 계단이 줄어 t63·지연이 0 으로 나오던 문제. `lat_err_mm` = 창 끝 평균 - (평균 + 4칸).
- 통합 시험: 튜닝 대기 중 SHIFT 로 고른다(TUNE → IDEN → RPM → TOP → ALL). START 한 번으로 식별 시험 0~6회,
  OPT 스윕(목록의 OPT 마다 왕복 2회, 식별 3·4회 속도, SL 정지·속도 계단 없음), 최고 속도 시험 1·2회(단계 1 의 L)를 잇는다.
  스윕 OPT 는 `param_set_ram` 으로 RAM 에만 넣고 회마다 조향 상태를 지운다. 끝·실패·끊김 모두 OPT 를 시험 전 값으로 돌린다.
  결과 `agv_all`(스윕 칸 `sw[]`)·`agv_ident`·`agv_top`. 실패 코드 = 단계 × 1000 + 회 × 10 + 순서. FND `A`+단계·회·순서, `ALOK`/`F`+코드.
  스윕 목록은 `ident_sweep_set` 으로 바꾼다.
- `param_set_ram`: `param_set` 과 같고 EEPROM 에 쓰지 않는다.
- 시험 결과(`agv_ident`·`agv_top`·`agv_all`·`agv_rpmt`)를 codestart 가 지우지 않는 RAMH0 구역 `test_keep` 에 둔다.
  CCS 연결 때 워치독 리셋이 나도 남는다. 부팅 때 `ident_keep_boot` 가 표시 워드로 전원 투입을 가려 지우고, 진행 중이던 결과는 state 3, fail -1.
- OTA 링커 파일 RAMH0 시작을 0x3FA000 으로 고쳤다(0x3F8000 은 L0·L1 거울).
- 바닥 최고 속도 시험: 튜닝 대기 중 SHIFT 로 고른다(TUNE → IDEN → RPM → TOP). 식별 시험 주행로에서 0회로 L 을 잰 뒤
  1·2회(방향 번갈아)에 지령을 `TOP_CMD_RPM` 까지 P.acc 램프로 올린다. 지령이 실제보다 많이 앞서면 실제 속도로 끌어내려 조향 몫을 남긴다.
  토크 상한·지령 지연 포화 중 0.3 s 가속도가 100 rpm/s 아래면 0.5 s 정속으로 최고 rpm 을 재고, 남은 거리로 감속해 선에서 돈다.
  결과 `agv_top`. FND `TPOK`, UP·DOWN 으로 좌·우 최고 rpm. 시험 중 스파이크 컷을 `TOP_SPIKE_RPM` 으로 올린다(`motor_spike_set`).
- 최대 rpm 시험: 튜닝 대기 중 SHIFT 로 고른다(TUNE → IDEN → RPM). 바퀴를 띄우고 START 하면 주행하지 않고
  두 바퀴에 DAC 1000·2000·3000·4000·4095 를 3 s 씩 고정해 낸다. `MOTOR_RPM_SPIKE` 컷 전 원시 rpm 으로 단계별 rpm,
  리플, 90% 오름 시간을 `agv_rpmt` 에 남긴다. STOP·E-STOP·오류·패널 키는 바로 DAC 0. FND `RPOK`, UP·DOWN 으로 좌·우 최대 rpm.
- `motor_dac_force`, `motor_dac_release`, `motor_raw_rpm`.
- 식별 시험: 튜닝 대기 중 SHIFT 로 고르는 자동 직진 시험. 결과 구조체 `agv_ident`. 조향 옆 계단 지령 `p_test`.
  교차선 두 개 사이를 혼자 오간다: 0회에 선 간격 L 을 재고, 1~6회는 속도 세 단계를 두 번씩(방향이 번갈아) 잰다.
  짝수 회는 SL 정지 뒤, 홀수 회는 서지 않고 감속해 최저 속도로 다음 선까지 가서 정렬 180도 회전. 정속·옆 계단 창은 남은 거리로 정하고, 1 s 가 안 되면 그 단계 속도를 4/5 로 낮춰 다시 잰다. 회별 결과는 `lv[6]`.
- `P_OPT_FLAGS` b4: 조향 이득을 휠 간격에 비례 (600 mm 기준). 범위 0~31.
- 2단계 HAL: `hal/f2808` (PLL, 워치독, 플래시 대기, PIE, Timer0, GPIO, 버스, SPI DAC, eCAP, SCI, Microwire EEPROM, ADC, codestart), `hal/host`. TI 헤더 없음.
- 3단계 조립: `src/sys` (메인 스케줄러, `run`, `comm`, `guide_poll`, `tagact`, `ui`, `storage`, `io`, `link`, `log`), 빈 예제 사이트, `tools/build_target.ps1`, `tests/test_sys.c`.
- SCI 를 16단 FIFO 로 바꿨다 (원본은 FIFO 꺼짐). 수신 오버런과 오류를 실제로 복구한다.
- 기존 모듈 추가: `seq_pick_profile`, `seq_call_once`, `param_limit`, `param_store_set`, `acs_cross_release`, `acs_cross_clear`, ACT 슬롯 부족·재전송 포기 로그.
- 1단계 코어 구현: `motor`(rpm 피드백, 속도 PI, S커브, 주행 단계, 테이프 회전 정렬), `stop`(MotorStop), `steer`, `branch`, `guide`, `frame`, `acs`, `seq`, `param`과 호스트 단위 시험.
- `guide`: `'a'`·`'h'` 해석, 바이트 파서, LED 덩어리 추적, 필터(기본 꺼짐), 센서 전환 초기화, 두절(255) 판정. `P_GUIDE_CURVE_DETECT` = 2 로 위치 소스 선택.
- 고친 결함: `curve_now` 반전, 테이프 회전 Step 2, `nGuide_Risk_Stop_Flag` 잔류, `'h'` 자리표 (`P_LEGACY_QUIRKS` b0~b3 으로 구동작 재현), 재전송 CT²·선두 대기, `set[40]` 곱 넘침, `DA_EST_cnt` 잔류, 스테이션 큐 상한, 부팅 직후 교차로 판정.
- 설계 문서 초안.
- 공개 헤더 초안: `types.h`, `seq.h`, `tagact.h`, `io.h`, `mgmt.h`.
- 시퀀스 표를 YAML에서 C 매크로(`seq_dsl.h`)로 변경. 생성기 제거. 시간 단위는 1/100초.
- 조건 `IN(ON/OFF)`, 가지 수 제한 없는 `SELECT`/`WHEN`, 라벨, `BELT_ON(n, CW/CCW)`와 사이트 배선 마스크 추가.
- 주행 중 병렬 동작 ACT와 태그 액션 `A_ACT`, 태그 상태 조건 `IF_STATION` 추가.
- 논리 I/O ID 재배치: 표에서 쓰는 신호 0~31, 앰프 신호 32 이상.
- `tests/test_dsl.c`: gcc 실행, cl2000 컴파일, 컴파일 오류 시험.
- OTA 앱 영역 링커 명령 파일 초안 `cmd/F2808_app_ota.cmd` (`AGV_Wifi_Bridge/dsp/app/F2808_app_example.cmd` 사본, 실기 검증 전 잠정).
