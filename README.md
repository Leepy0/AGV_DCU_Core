# AGV_DCU_Core

AGV 주행 제어기 DCU(TI TMS320F2808) 펌웨어의 **코어 라이브러리**. 주행·조향·정지 제어, 가이드센서, ACS 통신, 파라미터·EEPROM, I/O, 시퀀스 인터프리터, 관리 프로토콜, FOTA 연계를 담는다. 사이트별 동작(시퀀스·태그·I/O 배선)은 담지 않는다. 그것은 앱 저장소(사내)에서 이 라이브러리를 벤더링해 작성한다.

현재 상태: **하드웨어 무관 순수 로직과 호스트 단위 시험**. HAL, 런 조정, 태그 액션, 키·표시, 메인 스케줄러는 아직 없다.

## 문서

| 문서 | 내용 |
|---|---|
| [01_아키텍처](docs/01_아키텍처.md) | 계층, 실행 모델(ISR 최소화 + 10 ms 메인 스케줄러), 모듈 경계, 시간·메모리 예산, 통신 링크 추상화, FOTA·브릿지 호환, 버전, 저장소·릴리스 |
| [02_네이밍_표준](docs/02_네이밍_표준.md) | 이름 규칙, 시퀀스 요청 ID, 논리 I/O, 파라미터·오류 코드 대응표, 레거시 호환 헤더 |
| [03_시퀀스_DSL](docs/03_시퀀스_DSL.md) | 런 프로세스·태그 파일(약 21만 줄) 처리 방안: C 매크로 표, 분기, 벨트 배선, 주행 중 병렬 동작 ACT, 태그 규칙 표, 변환 절차 |
| [04_통신_계층](docs/04_통신_계층.md) | 프레임 코덱, 슬롯 스케줄러 재설계, 지그비 효율화, WiFi 브릿지 링크, 관리 프레임 |
| [05_게이트웨이_기능](docs/05_게이트웨이_기능.md) | 파라미터 조정·I/O 모니터/제어·조그·변위 이동·목적지 지령·태그 시험의 DCU 측 규격과 안전 규칙 |
| [07_주행_조향_성능](docs/07_주행_조향_성능.md) | 동일 출력 증명 방법, 유지할 상수, 개선 후보와 격리 스위치 |
| [08_추가_제안_목록](docs/08_추가_제안_목록.md) | 요구 외 제안 38건, 우선순위 |

요구사항 재검토, 기능 인벤토리, 분석 보고서는 사이트 정보를 포함하므로 앱 저장소에 있다.

## 디렉터리

```
include/agv/   공개 헤더 (types.h, drive.h, param.h, motor.h, stop.h, steer.h, branch.h, guide.h, frame.h, acs.h, seq.h, seq_dsl.h, tagact.h, io.h, mgmt.h)
src/           모듈 구현
hal/f2808/     레지스터 HAL (예정)
hal/host/      PC 시뮬레이션 HAL (예정)
cmd/           OTA 앱 영역 링커 명령 파일 (브릿지 레포 예제 기준, 실기 검증 전 잠정)
tools/         라이브러리 빌드 스크립트, 표 출력 도구 (예정)
tests/         test_dsl.c (표 매크로 검사), 호스트 단위 시험. 실행: powershell -File tests/run_host.ps1
docs/          설계 문서
```

## 빌드 (예정)

- 타깃: TI C2000 CGT 18.12.2.LTS, `cl2000 -v28 -ml -mt --c99 --float_support=softlib`, `ar2000 -r`로 `lib/agv_dcu_core.lib`.
- 호스트: gcc, `-DAGV_HOST_BUILD`, `tests/` 실행.
- 릴리스: `agv_dcu_core-<ver>.zip` = `lib/` + `include/` + `cmd/` + `CHANGELOG.md`. 앱 저장소는 이 zip을 `third_party/agv_dcu_core/<ver>/`에 풀어 커밋한다.

## 공개 헤더 안정성

`include/agv/*.h`가 공개 API다. `legacy_compat.h`는 전환 기간 전용이며 호환을 보장하지 않는다.
