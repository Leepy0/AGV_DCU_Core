# 변경 이력

## 미릴리스 (2026-11-07)
- 설계 문서 초안.
- 공개 헤더 초안: `types.h`, `seq.h`, `tagact.h`, `io.h`, `mgmt.h`.
- 시퀀스 표를 YAML에서 C 매크로(`seq_dsl.h`)로 변경. 생성기 제거. 시간 단위는 1/100초.
- 조건 `IN(ON/OFF)`, 가지 수 제한 없는 `SELECT`/`WHEN`, 라벨, `BELT_ON(n, CW/CCW)`와 사이트 배선 마스크 추가.
- 주행 중 병렬 동작 ACT와 태그 액션 `A_ACT`, 태그 상태 조건 `IF_STATION` 추가.
- 논리 I/O ID 재배치: 표에서 쓰는 신호 0~31, 앰프 신호 32 이상.
- `tests/test_dsl.c`: gcc 실행, cl2000 컴파일, 컴파일 오류 시험.
- OTA 앱 영역 링커 명령 파일 초안 `cmd/F2808_app_ota.cmd` (`AGV_Wifi_Bridge/dsp/app/F2808_app_example.cmd` 사본, 실기 검증 전 잠정).
