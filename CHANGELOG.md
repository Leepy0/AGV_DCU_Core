# 변경 이력

## 미릴리스 (2026-11-07)
- 식별 시험: 튜닝 대기 중 SHIFT 로 고르는 자동 직진 시험. 결과 구조체 `agv_ident`. 조향 옆 계단 지령 `p_test`.
  교차선 두 개 사이를 혼자 오간다: 0회에 선 간격 L 을 재고, 1~6회는 속도 세 단계를 두 번씩(방향이 번갈아) 재고 SL 정지,
  최저 속도로 다음 선까지 가서 정렬 180도 회전. 정속·옆 계단 창은 남은 거리로 정하고, 1 s 가 안 되면 그 단계 속도를 4/5 로 낮춰 다시 잰다. 회별 결과는 `lv[6]`.
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
