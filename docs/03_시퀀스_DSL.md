# 런 프로세스·태그 처리 방안: 시퀀스 표, ACT, 태그 규칙

사이트 동작은 C 파일에 매크로로 쓴 표로 정의한다. 한 줄이 스텝 하나다. 매크로 정의는 `include/agv/seq_dsl.h`, 검증은 `tests/test_dsl.c`(gcc 실행, cl2000 컴파일)에 있다.

**모든 시간 인자는 1/100초 단위 숫자다. `300` = 3초.** 앱 코드의 10 ms 틱과 같다.

## 1. 문제

레거시 앱 계층은 AGV 타입마다 `#if` 블록으로 나뉜 거대한 파일 두 개다. 각 타입 안에서 `if (플래그 == ON) { switch (gAction_Step) { case n: 출력; if (입력 && 카운터) { 다음 } } }` 틀이 반복된다. 동작 하나를 바꾸려면 큰 파일에서 블록을 찾아 들어가야 하고, 비슷한 블록이 많아 실수가 복제된다.

틀이 정형이므로 출력·입력대기·디바운스·지연·깜박임·보고·벨트·완료의 원시 동작과 소수의 관용구로 대부분 표현된다. 호기별 분기는 프로파일로, 나머지는 사용자 정의 함수로 분리한다. 표로 안 되는 수동 조작 코드는 프로파일별 함수로 옮긴다.

## 2. 방안 비교

| 방안 | 내용 | 장점 | 단점 |
|---|---|---|---|
| A. 양식 유지 | 활성 타입 블록을 그대로 새 코어 위에 컴파일 | 변환 위험 0. 빠름 | 13만 줄 구조 그대로. `cnt_n` 공유 버그 그대로 |
| B. YAML 표 + 생성기 | YAML에서 C를 생성 | 비개발자 편집 가능 | 들여쓰기 규칙이 C보다 어렵고, 오류를 생성기를 돌려야 안다. **탈락** |
| C. C 매크로 표 + 사용자 정의 함수 (채택) | 표를 C 파일에 매크로로 작성. 표로 안 되는 블록은 `CALL`로 C 함수 호출 | 익숙한 C 파일. 오타·인자 오류를 컴파일러가 잡음. 생성기 불필요. 코어가 표를 읽으므로 건식 실행·문서 출력 가능 | 양식 전환 기간에 두 양식 공존 |

먼저 시험할 타입부터 변환하고, 나머지 타입은 레거시 아카이브에 두었다가 필요할 때 같은 절차로 변환한다.

## 3. 시퀀스 표

### 3.1 모델

- **시퀀스**: 정지 완료 뒤 도는 로딩·언로딩 동작. 요청 ID(`SEQ_LOAD` 등) 하나에 표 하나. 한 번에 하나만 활성이다.
- **스텝**: 매 10 ms 활성 스텝 하나를 평가한다. 출력·보고처럼 시간이 걸리지 않는 스텝은 같은 스캔에 이어서 실행한다(최대 8개).
- **타이머**: 스텝 진입 때 0에서 시작하는 스텝 전용 카운터다. `cnt_1` 공유 문제가 없다.
- **라벨**: 번호 대신 이름으로 분기한다. 파일 첫머리 `SEQ_LABELS(...)`에 선언하고 `L( 이름 )`으로 붙인다. 이름은 파일 안에서 유일해야 한다. `L`은 실행 시간이 없고 스텝 한 칸을 쓴다. 코어는 분기할 때 표에서 라벨을 찾으므로 별도 RAM이 필요 없다.

### 3.2 스텝

| 구분 | 매크로 | 뜻 | 레거시 |
|---|---|---|---|
| 출력 | `OUT_ON( a, b )` / `OUT_OFF( a, b )` | 출력 켜기·끄기, 최대 16개 | `Out = nON;` |
| | `OUT_PULSE( o, 켬, 끔, 횟수 )` | 펄스 반복 | `if (cnt>0 && cnt<50) …` |
| | `OUT_FLICKER_UNTIL_IN( o, 간격, 조건 )` | **접점 통신.** 출력을 점멸하다가 입력이 들어오면 끄고 다음 | `Out = g1sec_Pulse_Flag; if (In == nON) …` |
| | `OUT_FLICKER_UNTIL_IN_TO( o, 간격, 조건, 제한, 라벨 )` | 위 + 시간 제한 | |
| | `HANDSHAKE( o, 응답입력, 제한, 라벨 )` | 출력 켬, 응답 켜짐 대기, 출력 끔, 응답 꺼짐 대기 | 설비 연동 4\~6 `case` |
| 벨트 | `BELT_ON( n, CW )` / `BELT_ON( n, CCW )` / `BELT_OFF( n )` | 3.5절 | `*_Belt_On_Off_Process` |
| 대기 | `WAIT( 조건 )` | 조건 참까지 대기 | `if (In == nON) step = n;` |
| | `WAIT_TO( 조건, 제한, 라벨 )` | 제한 넘으면 라벨로 | `cnt++; if (cnt > N) …` |
| | `WAIT_STABLE( 조건, 안정 )` | 조건이 연속으로 `안정`을 초과해 참이면 통과 | `if (In) { cnt++; if (cnt > N) … } else cnt = 0;` |
| | `WAIT_STABLE_TO( 조건, 안정, 제한, 라벨 )` | 위 + 제한. `UNMATCHED(제한)`이면 조건이 거짓인 시간만 누적 | `else { cnt_2++; … }` |
| | `WAIT_OUT( 조건 )` | 출력 상태 대기 | `if (Output_Spare.bit.X == nOFF)` |
| | `DELAY( 시간 )` | 시간 대기 | `cnt++; if (cnt > N)` |
| | `WAIT_IF( x )` / `WAIT_IF_TO( x, 제한, 라벨 )` | 확장 조건 대기 (3.3.1) | `if (AGV_Backward == ON)` |
| | `WAIT_IF_IN( x, 조건 )` / `WAIT_IF_IN_TO( x, 조건, 제한, 라벨 )` | 확장 조건 AND 입력 조건 | `if (Flag_Cross_In_On == OFF && In_3 == nON)` |
| | `WAIT_ANY2( 조건, ANY2_ON(c, d) )` / `WAIT_ANY2_TO( 조건, 묶음2, 제한, 라벨 )` | 두 번째 "하나라도 ON" 묶음 (3.3) | `if ((In_1 \|\| In_2) && (In_3 \|\| In_4))` |
| | `DELAY_PARAM( P_x, 단위 )` | 파라미터 x 단위 대기 (3.3.2) | `cnt++; if (cnt > set[X] * 6000)` |
| | `WAIT_TO_PARAM( 조건, P_x, 단위, 라벨 )` | 제한 시간이 파라미터인 `WAIT_TO` | |
| 분기 | `SELECT_BEGIN … WHEN … SELECT_END` | 3.4절 | `if … else if` |
| | `GOTO( 라벨 )` | 이동 | `step = n;` |
| | `ERROR_HOLD( 코드, 조건, 라벨 )` | 오류 표시 유지, 조건 참이면 라벨로 | `case 290` |
| ACS | `ACS_SEND( LOADING, 87 )` / `ACS_SEND( NEXT, STATION(1) )` / `ACS( LOADING, AGV_ID )` | 보고. `ACS_SEND` 번호는 상수 0\~65535 또는 `STATION(k)`(스테이션 큐[k], `ACS( 종류, STATION(k) )` 와 같은 스텝). 종류는 레거시 함수 이름 그대로 `LOADING` `UNLOADING` `NEXT` `ARRIVE` `FINISH` `PROCESSING` `STANDBY` `CALL`(태그 표는 `DSL_INFO_<이름>`). 번호 출처 `STATION0`, `STATION0_PLUS1`, `STATION(k)`(큐[k]), `AGV_ID` 또는 상수 | `DA_ACS_Loading(…)` |
| | `ACS_WAIT_ACK( 제한, 라벨 )` | ACK 대기 | `Scia_Send_AGV2PC_Message == ACK_NONE_MSG` |
| | `ACS_STATION( 칸, 값, 길이 )` / `STATION_RESET` | 스테이션 큐 | `ACS_Station_Call_Num[1] = 0;` |
| 주행 | `AGV_SPEED( HIGH·LOW·LLOW·MIN )` / `AGV_SPEED_MMIN( v )` | 재출발 속도. v 는 m/min 4\~255 | `gDriving_speed = …` |
| | `AGV_FORBACK( FWD·BACK·TOGGLE )` | 방향 전환 예약 | `AGV_ForBack_Action(FB)` |
| | `AGV_TURN( CW, 90, fb, auto[, 속도[, 옵션…]] )` | 회전 요청 + 완료 대기(3.2.2) | `AGV_Turn_Action(…)` |
| | `AGV_TURN( CW, 90, fb, auto, 0, NOWAIT )` | 회전 요청, 다음 스캔에 다음 스텝. 회전 중 뒤 스텝이 돈다 | `AGV_Turn_Action(…); step++;` |
| | `AGV_MOVE( mm )` | 그 자리에서 mm 앞에 위치 정지하고 설 때까지 대기(3.2.1) | 없음(v0.41.0) |
| | `OBS_ON` / `OBS_OFF` | 장애물 센서 사용 | `Flag_OBS_Sen_Use` |
| 상태 | `SET_STATE( id, 값 )` / `WAIT_STATE( id, 값 )` | 사이트 상태 변수 | `Gate1_Out`, `FL_check` 류 |
| 병렬 | `ACT_START( id )` / `WAIT_ACT( id )` | 4절 | |
| 확장 | `CALL( 인덱스 )` | 사용자 정의 함수 | 표로 안 되는 블록 |
| 종료 | `END_RESTART` / `END_STOP` / `END_ERROR( 코드 )` | 재출발 예약 / 그냥 종료 / 오류 종료 | `case 11` / 플래그 OFF / `action_err = n` |

#### 3.2.1 AGV_MOVE( cm ) (v0.41.0, v0.49.0 까지 이름 `MOVE_MM`, v0.50.0 부터 cm)

서 있는 자리에서 출발해 cm(1\~3276, 밖이면 컴파일 오류. 표의 거리는 모두 `SL` 과 같은 cm 이고 스텝에는 mm 로 넣는다) 앞에 위치 정지하고, 설 때까지 시퀀스가 기다린 뒤 다음 스텝으로 간다. 회전(`AGV_TURN`)처럼 시퀀스를 살려 둔 채 주행한다.

```c
SEQ_BEGIN( shift_unload )
    CALL( CALL_LEFT_UNLOAD )      /* 왼쪽 컨베이어 언로딩 */
    AGV_MOVE( 10 )                /* 10 cm 앞으로 */
    CALL( CALL_RIGHT_UNLOAD )     /* 오른쪽 컨베이어 언로딩 */
    END_RESTART
SEQ_END
```

- 방향: 지금 전후진 그대로. 앞에 `AGV_FORBACK( BACK )` 을 두면 후진이다.
- 속도: 거리에 맞춰 자동. 0 → v → 0 이 거리 안에 드는 가장 빠른 v 를 `SPN`\~`LLOW` 로 묶는다(`tagact_move_mm` 주석에 식). 제동점은 위치 정지 제어가 정한다(`STOP_REASON_MOVE`, 제동점까지 순항). 정지 허용 오차는 다른 위치 정지와 같다.
- 이동 중 태그: 동작·정지·속도 모두 무시한다. ACK 는 보낸다(안 보내면 재전송이 이동 뒤 늦게 처리될 수 있다). 교차로 IN·OUT 태그는 버리지 않고 보고(IN 보고·허가 대기, OUT 보고 큐)한 뒤 서고 오류 290(v0.50.0. 시퀀스 이동은 교차로를 건너는 용도가 아니다). IN 의 허가 대기는 오류 동안 남아 리셋 키로 풀고 START 하면 교차로 대기 거리에서 허가를 기다린다.
- 장애물·일시정지: 서고, 풀리면 남은 거리를 이어 간다(목표 지점 그대로).
- 도착(v0.50.0): 목표에 도착해 서야(`MOVE_DONE`) 다음 스텝이다. 도착 전에 다른 원인(명령 정지, ACS 교통 정지 등)으로 서면 실패한 접근 오류 277 로 서고 시퀀스가 끝난다(리셋 필요. 남은 거리·출발 계기를 다시 정할 수 없어 이어 가지 않는다). 회전·정지 중이면 이동을 걸지 않고 다음 스캔에 다시 건다(회전 `NOWAIT` 뒤 이동). 거리가 펄스 범위(32767)를 넘으면 오류 290.
- 선 잃음: 기존 가이드 오류로 서고 시퀀스는 중단된다. STOP·E-STOP 도 중단.
- 이동·교차선 일 중에는 자동 속도 변경·START 고속이 지령을 덮지 않는다(v0.50.0).
- 출발 정렬 단계의 조향 상한(× 0.5)이 이동 내내 걸린다. 속도는 이미 `LLOW` 이하라 거리에는 영향이 없다.
- 짧은 거리: 따로 최소 거리를 두지 않는다. 끝 꼬리(바닥 150 rpm 에서 0, 약 2 펄스)보다 짧으면 `SPN` 으로 출발해 꼬리만큼 간다(모의 5 mm → 7.7 mm).
- 모의(`test_curve move`, 바퀴 지연 0.1 s, `LOS` 25): 5·20·50·100·300·1000 mm 에서 지나침 2.7·3.1·3.6·4.0·5.6·6.3 mm, 최고 1.6\~12.5 m/min, 0.7\~6.2 s. 지나침은 속도에 비례한다(바퀴 지연 몫).
- ACT 에서는 금지(`seq_bind` 거부).

#### 3.2.2 AGV_TURN 과 옵션 표기 (v0.50.0)

방식 옵션은 인자 맨 뒤에 적는다. 안 적으면 일반 동작이다. 같은 뜻이면 태그 표와 시퀀스 표에서 같은 이름이다. 정해 두지 않은 옵션·조합·같은 옵션 두 번은 컴파일 오류다.

`AGV_TURN( 방향, 각도, 회전 뒤 방향, 자동 출발[, 속도[, 옵션…]] )`. 인자 수는 4(속도 생략 = 0), 5(속도), 6·7(속도 + 옵션 1·2개). 옵션을 쓰려면 속도 칸(0 = 자동)을 적는다.

| 옵션 | 시퀀스 표 | 태그 표 | 인코딩 |
|---|---|---|---|
| (없음) | 회전하고 끝날 때까지 기다림 (`SK_TURN`) | 회전 (`TA_TURN`) | |
| `NOWAIT` | 회전을 걸고 다음 스캔에 다음 스텝 (`SK_TURN_NOWAIT`) | 컴파일 오류 | 스텝 종류 |
| `ALIGN` | 선 자리에서 정렬 회전, 정착·보정까지 끝나 서면 다음 스텝 (`SK_TURN_ALIGN`, v0.50.0. 3.2.4) | 주행로 정렬 회전 (`TA_TURN_ALIGN`) | 종류 |
| `TAPE` | 컴파일 오류(시퀀스 중 교차선 회전은 하지 않는다) | 교차 테이프를 찾은 뒤 정렬 회전 (`TA_TURN_TAPE`) | 종류 |
| `OBS` | 회전 중 장애물 감지 | 같다 | 셋째(태그는 둘째) 칸 b8 = 1 |

- 종류 옵션(`NOWAIT`·`ALIGN`·`TAPE`)은 하나까지, `OBS` 는 함께 쓸 수 있다. 차례는 자유다(`…, 0, ALIGN, OBS` = `…, 0, OBS, ALIGN`).
- `OBS` 는 회전 중 장애물 감지(`D.turn_obs_use`, 값 1 만 본다)다. 주행 중 장애물 센서 사용 `OBS_ON`/`OBS_OFF` 와 다른 것이다. v0.49.0 까지는 다섯째 위치 인자(장애물 0\~7)였다.
- 회전 뒤 방향 b0\~3 | 자동 출발 b4\~7 | 장애물 b8(b9\~10 은 0) | 속도 b11\~15. 태그 둘째 칸과 시퀀스 셋째 칸이 같은 배치다.
- 방향(`CW`·`CCW`)은 `AGV_TURN` 이 바로 붙인다(사이트 `legacy_const.h` 의 `CW`·`CCW` 매크로가 펼쳐지지 않는다).

옛 이름 → 새 이름(v0.50.0, 데이터는 같다):

| 옛 이름 | 새 이름 |
|---|---|
| 시퀀스 `SPEED( level )` / `SPEED_MMIN( v )` | `AGV_SPEED( level )` / `AGV_SPEED_MMIN( v )` |
| 시퀀스 `FORBACK( dir )` | `AGV_FORBACK( dir )` |
| 시퀀스 `TURN( d, deg, fb, auto, 장애물 )` | `AGV_TURN( d, deg, fb, auto, 0 )` (장애물 1 이면 `…, 0, OBS`) |
| 시퀀스 `TURN_NOWAIT( d, deg, fb, auto, 장애물 )` | `AGV_TURN( d, deg, fb, auto, 0, NOWAIT )` |
| 시퀀스 `MOVE_MM( mm )` | `AGV_MOVE( cm )`(v0.50.0 부터 cm: 옛 값 ÷ 10) |
| 태그 `AGV_TURN( d, deg, fb, auto, 장애물[, 속도] )` | `AGV_TURN( d, deg, fb, auto[, 속도[, OBS]] )` |
| 태그 `AGV_TURN_ALIGN( … )` / `AGV_TURN_TAPE( … )` | `AGV_TURN( …, 속도, ALIGN )` / `AGV_TURN( …, 속도, TAPE )` |
| 태그 `AGV_TAPE_STOP( seq )` | `SEQ_START( TAPE, seq )`(이 동작만 옵션이 앞) |
| 태그 `AGV_TAPE_STOP( 0 )`(시퀀스 없음) | `AGV_STOP( TAPE )` |
| (없음) | `AGV_STOP( cm )`, `AGV_STOP( TAPE_GD )`, `SEQ_START( cm, x )`, `SEQ_START( TAPE_GD, x )`(5.2) |
| 태그 `( TA_SPEED \| n )` | `AGV_SPEED_MMIN( n )` |

> **옛 표를 옮길 때 `AGV_TURN` 의 다섯째 칸(장애물)을 빼거나 `, 0, OBS` 로 바꾼다.** 옛 5인자 `AGV_TURN( …, 자동 출발, 장애물 )` 은 새 형식에서 (…, 자동 출발, 속도)로 읽힌다. 장애물이 0 이면 같지만 1 이면 회전 속도 1 m/min(최저 속도로 묶임)이 되고 장애물 감지는 꺼진다. 컴파일러가 막지 못한다. 옛 6인자(…, 장애물, 속도)는 컴파일 오류다.

회전 표기의 범위(컴파일 오류): 회전 뒤 방향·자동 출발 0\~15(넘치면 옆 칸으로 샌다), 속도 0\~31, 태그 표의 각도는 90 의 배수(칸이 90 단위다. 45 는 0 이 된다).

#### 3.2.3 교차선까지 이동 `AGV_MOVE( 한계 cm, TAPE )`·`AGV_MOVE( 한계 cm, TAPE_GD )` (v0.50.0)

서 있는 자리에서 지금 전후진 방향으로 출발해 다음 교차선을 보면 서고, 설 때까지 기다린 뒤 다음 스텝으로 간다(`SK_MOVE_TAPE`, `tagact_move_tape`).

- 교차선 인정·서는 방식·찾는 속도는 태그 교차선 정지와 같은 한 곳(`run.c` 교차선 절, 5.3)을 쓴다. `TAPE` 는 분류 3(최저 속도 `SPN` 으로 찾아 본 자리에서 바로 정지), `TAPE_GD` 는 분류 4(교차선 회전과 같은 찾기 속도, 교차선 뒤 `GD` 구간으로 회전 중심이 교차선 위에 선다).
- 출발 때 센서가 이미 교차선 위면 그 선은 건너뛴다: 출발 뒤 5틱 뒤, 교차선 위가 아닌 응답을 이어서 두 번 본 뒤부터 센다.
- 한계(1\~3276 cm, 밖이면 컴파일 오류. 32비트로 센다)를 넘겨 가면 서고 오류 258(교차선 정지가 못 찾을 때와 같다). 다음 스텝(출력)은 돌지 않고 시퀀스가 끝난다.
- 도착(교차선 인정 뒤 선 것)이어야 다음 스텝이다. 다른 원인으로 서면 277(3.2.1과 같다). 이동 중 태그·교차로 태그·장애물·STOP 은 3.2.1과 같다. 이동을 시작할 때 남은 교차선 요청(태그 쪽)을 지운다.
- 거부(달리는 중·회전·정지 중)는 걸지 않고 다음 스캔에 다시 건다. ACT 에서는 금지.
- 모의(`test_curve mtape`, 교차선 폭 20 mm, `SPN` 3 m/min): 100\~2000 mm 앞 교차선에서 센서 가운데가 교차선 가운데보다 4.1\~4.3 mm 앞에 선다(선 가장자리를 넘어 2스캔). 출발 때 선 위 → 건너뛰고 한계 1500 에서 258. 한계 1000·교차선 2000 → 1006 mm 에서 258. 300 mm 에서 장애물 1 s → 서고 이어 가 같은 자리에 선다. 이동 중 STOP → 시퀀스 끝, 뒤 출력 안 켜짐. 후진(모형 차는 방향과 상관없이 앞으로만 가 코드 경로만 본다).

#### 3.2.4 시퀀스 정렬 회전 `AGV_TURN( …, 속도, ALIGN )` (v0.50.0)

서 있는 자리에서 태그·튜닝과 같은 정렬 회전(`tagact_turn_align_stopped`)을 건다. 목표각 근처에서 주행로를 찾아 가운데에 맞추고, 정착·보정까지 끝나 차가 선 위에 서면(`tape_align` 0, 회전 끝) 다음 스텝으로 간다.

- 오류: 목표각 근처에서 선을 못 찾는 등(258) 회전이 오류로 끝나면 다음 스텝으로 가지 않는다. 같은 틱 뒤 `run_after_motor` 가 시퀀스를 끝낸다. 일반 회전 스텝도 오류가 선 틱에는 다음 스텝으로 가지 않는다.
- 오류 없이 실패로 끝남: 정렬이 가운데에 서지 못하면(`tape_fix_end` ≠ 0: 선을 잡은 뒤 범위 초과, 보정 횟수 끝, 보정 각 초과, 선 놓침, 다가온 쪽 모름) 258(`system_err`)로 서고 시퀀스가 끝난다.
- 거부: 정렬 회전을 걸 수 없으면(`TFB` 십의 자리 0, 회전·정지 거리가 남아 있음) 290(`stop_err`)으로 서고 시퀀스가 끝난다. 일반 회전 스텝의 거부는 지금처럼 다음 스캔에 지나간다.
- 시퀀스 회전·이동 스텝의 오류는 리셋까지 남는 오류다(차가 서고 표시·ACS 오류 보고, 시퀀스·출력 정리).
- ACT 에서는 금지(`seq_bind` 거부).
- 모의(`test_curve salign`, 교차선 위 CW 90도, 슬립 0.8·1.0·1.25): 선 기준 각 -0.51·-0.34·-0.26 도, 보정 0, 태그 경로와 같다(시퀀스가 1틱 늦게 시작). 교차선이 없으면 258, 시퀀스 끝남, 뒤 출력 안 켜짐.

### 3.3 조건

한 조건 안에 켜짐과 꺼짐을 같이 쓴다. 신호는 그룹당 최대 16개, `IN` 인자는 최대 6개다.

```c
IN( ON( SEN_1, SEN_2 ) )                                /* 둘 다 켜짐 */

IN( OFF( SEN_1, SEN_2 ) )                               /* 둘 다 꺼짐 */

IN( OFF( BOX_SEN_1, BOX_SEN_2, BOX_SEN_3 ),             /* 1·2·3 꺼짐 그리고 4·5·6 켜짐 */
    ON ( BOX_SEN_4, BOX_SEN_5, BOX_SEN_6 ) )

ANY_ON( SEN_1, SEN_2, SEN_3, SEN_4 )                       /* 하나라도 켜짐 */

IN( ANY_ON( SEN_1, SEN_2 ), OFF( SEN_5 ) )                 /* 1·2 중 하나라도 켜짐 그리고 5 꺼짐 */
```

- `ANY_ON`은 나열한 신호 중 하나라도 켜지면 참이다. `IN`으로 `ON`·`OFF`와 묶는다.
- 금지: 한 조건 안에서 같은 핀을 반대 뜻으로 겹쳐 쓰지 않는다. `IN( ON(x), OFF(x) )`, `IN( ON(SENH_1), ON(SEN_1) )`(같은 핀의 두 극성 이름), `ON( SENH_1, SEN_1 )`, `IN( ANY_ON(SEN_1, SEN_2), ON(SENH_1) )`·`IN( ANY_ON(SEN_1, SEN_2), OFF(SEN_1) )`(`ANY_ON` 묶음의 핀을 다른 인자에 또 씀)는 컴파일 오류다(v0.48.0). 예전에는 마스크 양쪽 절반에 같은 비트가 서서 `ANY_ON`으로 읽히거나 한쪽이 묻혔다. 같은 핀을 같은 뜻으로 두 번 쓰는 것(`IN( ON(x), ON(x) )`, `IN( OFF(SENH_1), ON(SEN_1) )`)은 된다.
- `ON(x)`는 x 가 활성(자재 감지, 설비 신호 들어옴), `OFF(x)`는 비활성이다. 핀 레벨은 사이트 `io_map.h`의 이름이 정한다. 레거시가 `== nON`으로 견준 신호는 그대로 논리 ID, `== ON`으로 견준 신호(같은 핀을 차종마다 반대 극성으로 쓰는 경우)는 `#define SENH_1 IN_ACTIVE_HIGH( IN_EXT_1 )`처럼 이름에 표시한다. `ON()`·`OFF()`가 컴파일 때 반대쪽 마스크로 보내므로 표 데이터·해석기는 같다. 표시한 이름은 `ON`·`OFF`에만 쓴다(`ANY_ON`·`ANY2_ON`·출력·`LOAD_SENSORS`는 컴파일 오류, C 코드의 `io_in`에는 논리 ID 를 쓴다).
- 조건에 쓸 수 있는 신호는 논리 ID 0\~31이다. 앰프 과부하 입력처럼 그 밖의 ID는 컴파일 오류다.
- 조건은 그룹끼리 AND다. 같은 신호들의 OR은 `ANY_ON`. 서로 다른 조건의 OR은 `WHEN`을 나눈다.
- `ANY_ON` 묶음은 조건 하나에 하나다. 묶음이 둘이면 `WHEN_ANY2`·`WAIT_ANY2`·`WAIT_ANY2_TO`의 둘째 인자 `ANY2_ON(...)`. 스텝의 `out` 칸을 쓴다. 이 셋에서만 된다. 확장 조건과 같이 쓸 수 없다.

```c
WAIT_ANY2( IN( ANY_ON( SEN_1, SEN_2 ), OFF( SEN_5 ) ),  ANY2_ON( SEN_3, SEN_4 ) )   /* (1||2) && (3||4) && !5 */
```

#### 3.3.1 확장 조건

입력이 아닌 조건. `X_*`는 "코드, a, b" 세 값으로 펼쳐진다. 시퀀스와 태그 규칙(5절)이 같은 이름을 쓴다.

| 매크로 | 참 조건 | 레거시 |
|---|---|---|
| `X_FORWARD` / `X_BACKWARD` | 주행 방향 | `AGV_Backward == OFF/ON` |
| `X_CROSS_IN` / `X_CROSS_OUT` | 교차로 대기 또는 유지 중 | `Flag_Cross_In_Flag \|\| Flag_Cross_In_On` |
| `X_CROSS_IN_ON` / `X_CROSS_OUT_ON` | 유지(허가 대기 정지) 중만 | `Flag_Cross_In_On == ON` |
| `X_ERR_EQ( n )` | `action_err == n` | `action_err == BATTERY_LOW_VOLTAGE` |
| `X_PARAM_EQ( P_x, v )` / `X_PARAM_GE` / `X_PARAM_LE` | 파라미터 `==`·`>=`·`<=` v | `set[X] == N` |
| `X_ID( lo, hi )` | lo <= `P_AGV_ID` <= hi | `set[AGV_ID] == N` |
| `X_KEY( UP )` | 지금 눌린 키 값 == `KEY_UP`. `ui_key()`만 읽고 키를 소비하지 않는다. `UP DOWN SET RESET MODE SHIFT` | `Read_Key_buffer == UP_KEY` |
| `X_FORBACK_MODE` / `X_LOW_BATTERY` | 방향 전환 모드 > 0 / 저전압 | |
| `X_STATION( k, lo, hi )` / `X_STATION_RUNNING( k, lo, hi )` | lo <= 큐[k] <= hi (/ + 정지 전) | `ACS_Station_Call_Num[k] == N` |
| `X_STATION_MOD( k, n, m )` / `X_STATION_DIV( k, n, m )` | 큐[k] `%` n == m / 큐[k] `/` n == m. n = 0 은 컴파일 오류 | `ACS_Station_Call_Num[k] % N == M` |
| `X_TAG_MINUS_EQ_STATION( k, n )` / `X_TAG_EQ_STATION( k )` | 태그 - n == 큐[k] / 태그 == 큐[k]. **태그 표 전용** | `Tag_Num - N == ACS_Station_Call_Num[k]` |
| `X_NOT( x )` | 부정 | `== OFF` |

```c
WAIT_IF( X_NOT( X_CROSS_IN ) )                                   /* 교차로 대기·유지가 풀릴 때까지 */
WAIT_IF_IN_TO( X_BACKWARD, IN( ON(SEN_4) ), 500, ERR )           /* 후진이고 SEN_4 켜짐, 5초 제한 */

SELECT_BEGIN
    WHEN_IF_IN( X_ID(12, 12),         IN( ON(UP_LIMIT_B) ), STOP )  /* 호기 12 만 리밋 입력이 다르다 */
    WHEN_IF_IN( X_NOT( X_ID(12, 12) ), IN( ON(UP_LIMIT) ),   STOP )
SELECT_END
```

- 스텝: `WHEN_IF( x, 라벨 )`, `WHEN_IF_IN( x, 조건, 라벨 )`, `WAIT_IF*`(위 표). 확장 조건은 스텝 하나에 하나, 입력 조건과 AND.
- 저장: 스텝의 `xc`(C28x 정렬 빈칸이라 크기 그대로 12워드)에 코드, `arg0`·`arg1`에 a·b. 그래서 `WAIT_STABLE`·`WAIT_TO_PARAM`에는 못 쓴다.
- 호기별 다른 입력 번호는 프로파일을 나누지 않고 `WHEN_ID( lo, hi, 라벨 )`(= `WHEN_IF( X_ID(lo, hi), 라벨 )`)로 한 표에서 가른다.

#### 3.3.2 파라미터 시간

`DELAY_PARAM( P_x, 단위 )`, `WAIT_TO_PARAM( 조건, P_x, 단위, 라벨 )`. 단위 `TICK`(10 ms), `DSEC`(0.1초), `SEC`, `MIN`. 시간 = 값 x 단위이고 `DELAY`처럼 그 시간을 넘으면 통과다.

- 값은 스캔마다 다시 읽는다. 음수는 0. `WAIT_TO_PARAM`은 값 0 이하면 제한 없음.
- 단위 안 틱과 지난 단위 수를 따로 세서 16비트 틱(655초)을 넘어도 된다. 99분 = 594000틱.
- 파라미터 번호는 1 \~ `P_COUNT`-1, 밖이면 컴파일 오류.

### 3.4 분기

레거시 `case`는 조건에 따라 다른 `gAction_Step`으로 간다.

`SELECT`의 가지 수에는 제한이 없다.

```c
SELECT_BEGIN

    WHEN(  IN( ON (TYPE_SEN_A),
               OFF(TYPE_SEN_B, TYPE_SEN_C) ),    LOAD_TYPE_A  )

    WHEN(  IN( ON (TYPE_SEN_B),
               OFF(TYPE_SEN_A, TYPE_SEN_C) ),    LOAD_TYPE_B  )

    WHEN(  IN( ON (TYPE_SEN_C),
               OFF(TYPE_SEN_A, TYPE_SEN_B) ),    LOAD_TYPE_C  )

    WHEN(  IN( ON (TYPE_SEN_A, TYPE_SEN_B) ),    ERR_SENSOR   )

SELECT_END
```

| 레거시 형태 | 표 |
|---|---|
| `if (A) step=1; else if (B) step=2;` (둘 다 아니면 대기) | `WHEN` 두 줄. 위에서 아래로 평가해 첫 번째 참인 가지로 간다. 전부 거짓이면 대기 |
| `else step=3;` (대기 없이 분기) | 마지막에 `OTHERWISE( 라벨 )` |
| 대기에 시간 제한 | `SELECT_END` 대신 `SELECT_END_TO( 3000, ERR_READY )` |
| 안정 시간 + 불일치 누적 타임아웃 | `WAIT_STABLE_TO( 조건, 300, UNMATCHED(2000), ERR_READY )` |
| `case 290: action_err = 290; if (START_SW == nON) step = 1;` | `ERROR_HOLD( 290, IN( ON(START_SW) ), 복귀라벨 )` |
| 앞 스텝의 결과를 뒤에서 사용 | 가지마다 `ACS_STATION`·`SET_STATE` 뒤 `GOTO( 합류라벨 )`. 뒤에서 값을 보려면 `WHEN_STATE( id, 값, 라벨 )` |

`SELECT_BEGIN`·`SELECT_END`는 읽기 위한 표시이며 스텝을 만들지 않는다. 코어는 연속한 `WHEN`을 하나의 묶음으로 본다. 가지 하나가 스텝 한 칸(12워드)을 쓴다.

### 3.5 벨트

방향은 `CW`·`CCW`로 쓰고, 방향별로 어느 출력을 몇 개 켜는지는 사이트 `io_map.h`에서 정한다. 시퀀스는 배선을 모른다.

```c
/* site/<name>/io_map.h */

/* 레거시 표준 배선: CW 는 Power 1개, CCW 는 Power + FB 2개 */
#define BELT1_CW     OUTS( CV1_POWER )
#define BELT1_CCW    OUTS( CV1_POWER, CV1_FB )

/* 방향별 출력 1개씩 */
#define BELT2_CW     OUTS( CV2_CW )
#define BELT2_CCW    OUTS( CV2_CCW )

/* 한 방향만 있는 벨트 */
#define BELT3_CW     OUTS( CV3_RUN )
#define BELT3_CCW    NO_OUT
```

```c
BELT_ON ( 2, CCW )     /* CCW 출력을 켜고, CCW 에 없는 나머지 출력은 끈다 */
BELT_ON ( 2, CW )
BELT_OFF( 2 )          /* 그 벨트 출력 전부 OFF */
```

- 레거시 `First_Belt_On_Off_Process(InOut, OnOff)`는 `(1,1)`이면 Power·FB, `(0,1)`이면 Power만 켠다. 이 함수를 쓰던 코드는 위 표준 배선으로 옮기고, `InOut`을 사이트별로 CW·CCW에 대응시킨다. 레거시 주석의 방향 표기와 실제 회전 방향은 현장에서 확인한다.
- 없는 방향을 켜면 컴파일 오류다.
- 벨트 표나 코드 없이 출력 마스크로 펼쳐지므로 `SK_OUT` 스텝 하나로 끝난다.

### 3.6 표 형식과 크기

`seq_step_t`는 C28x에서 12워드다(`tests/test_dsl.c`가 cl2000 컴파일 시 확인). 확장 조건 칸 `xc`는 정렬 빈칸에 들어가 크기가 늘지 않았다. 스텝 530개면 약 6,400워드에 라벨 칸이 더해진다. 기존 코드 크기와의 비교는 변환 뒤 `.map`으로 한다.

조건은 32비트 마스크라서 논리 ID를 0\~31에 배치했다(`io.h`): 입력은 EXT 16 + 보드 16, 출력은 EXT 16 + 램프·브레이크 등 9. 앰프 신호는 32 이상이다. 시퀀스가 앰프를 직접 건드리지 못하는 것이 이 구조의 의도이기도 하다.

### 3.7 인터프리터

```
seq_run():                                  run 모듈이 디스패치 조건 아래에서 호출
    활성 시퀀스가 없으면 요청 중 우선순위가 가장 높은 ID를 활성화
    최대 8회 반복:
        즉시 스텝(라벨, 출력, ACS, GOTO …)  실행하고 다음 스텝 계속
        대기 스텝                           timer++, 조건 평가, 통과면 다음 스텝 계속, 아니면 끝
        WHEN 묶음                           위에서 평가, 참이면 라벨로, 전부 거짓이면 끝
        CALL                                반환 0 끝, 1 다음, 2 오류 종료
        END_*                               요청 해제, 재출발 예약(END_RESTART), 비활성
```

- `Auto_Run_Process`의 디스패치 조건은 `run` 모듈이 맡는다. 회전 중에도 호출되는 현행 동작은 유지해 `AGV_TURN` 스텝이 동작한다.
- 시퀀스 중단(STOP, E-STOP, `W` 리셋)은 `seq_abort_all()`이다.
- 표 파일은 짧은 이름(`ON`, `IN`, `L`)을 쓰므로 `legacy_compat.h`와 함께 포함하지 않는다.

### 3.8 사용자 정의 함수

```c
/* site/<name>/custom.c */
int fork_manual_step( seq_ctx_t *c )     /* 반환 0 대기, 1 완료, 2 실패 */
{
    switch ( c->sub ) {
    case 0:
        io_out( LIFT_UP_OUT, 1 );
        c->sub = 1;
        return 0;

    case 1:
        if ( io_in( LIFT_UP_LIMIT ) ) { io_out( LIFT_UP_OUT, 0 ); return 1; }
        return ( c->timer > 3000 ) ? 2 : 0;

    default:
        return 2;
    }
}
```

- 코어 상태 접근은 공개 API로만 한다.
- `DA_Manual_Act`의 "대기 중 수동 조작"은 공통 조건을 `run_is_idle()`로 빼고 프로파일별 `site_manual()`로 옮긴다.

## 4. ACT: 주행 중에 도는 병렬 동작

시퀀스는 정지 완료 뒤에만 돈다. 주행 중 태그를 읽고 일정 시간 액추에이터를 움직이는 동작은 **ACT**로 쓴다. 문법은 시퀀스와 같고, 주행을 멈추지 않으며, 시퀀스와 동시에 돈다.

### 4.1 레거시 구조

태그가 `Gate1_Out = ON`만 세우면 `DA_Manual_Act`가 주행 여부와 무관하게 매 10 ms `cnt_1`을 세어 출력을 만든다. 문제는 세 가지다.

- `cnt_1`을 Transfer와 공유한다.
- 출력 패턴을 `if ((cnt_1 > 0) && (cnt_1 < 50))` 줄로 손으로 써서 카운터를 잘못 비교하는 실수가 나온다.
- `Gate1_Out`의 뜻이 호기마다 다르다.

### 4.2 작성

```c
/* 주행 중 설비 신호 3회: 0.5초 켜고 0.5초 끄기 */
ACT_BEGIN( lift_signal_3x )
    OUT_PULSE( LIFT_SIGNAL, 50, 50, 3 )
ACT_END

/* 주행 중 벨트를 5초 돌리고 정지 */
ACT_BEGIN( belt_run_5s )
    BELT_ON ( 1, CW )
    DELAY   ( 500 )
    BELT_OFF( 1 )
ACT_END

/* 주행 중 리프트 상승. 상한 도달까지, 10초 넘으면 주행 정지 */
SEQ_LABELS( LIFT_FAIL )

ACT_BEGIN( lift_up_while_run )
    OUT_ON ( LIFT_UP_OUT )
    WAIT_TO( IN( ON(LIFT_UP_LIMIT) ),  1000,  LIFT_FAIL )
    OUT_OFF( LIFT_UP_OUT )
    END_STOP

    L( LIFT_FAIL )
    OUT_OFF( LIFT_UP_OUT )
    END_ERROR( 290 )
ACT_END
```

```c
/* 태그 표: AGV_STOP 이 없으므로 주행을 계속한다 */
TAG( 41, PF_LIFT, ACT_START( ACT_BELT_5S ) )

/* 상태 조건: 스테이션 큐[1] 이 101~116 일 때만 */
TAG_IF( 36, PF_LIFT, IF_STATION( 1, 101, 116 ), ACT_START( ACT_LIFT_SIGNAL ) )
```

마지막 스텝이 끝나면 ACT는 자동으로 끝난다. 다음에 스텝이 더 있는 지점에서 끝내려면 `END_STOP`을 쓴다.

### 4.3 규칙

| 항목 | 동작 |
|---|---|
| 시작 | 태그 `ACT_START(id)`, 시퀀스 `ACT_START(id)`, 관리 명령 |
| 실행 | 매 10 ms, 주행·정지와 무관. 슬롯 4개, 슬롯별 타이머 |
| 같은 ACT 재요청 | 무시. 레거시 `Gate1_Out = ON` 재대입과 같다 |
| 슬롯 부족 | 시작하지 않고 로그에 남긴다 |
| 장애물·교차로 일시정지 | 계속 돈다. 시간도 계속 흐른다 |
| STOP, E-STOP, `W` 리셋, `system_err`, `stop_err` | 즉시 중단하고 **그 ACT가 켠 출력만** 끈다 |
| 실패 | `END_ERROR(code)`는 `stop_err = code`로 주행을 멈춘다. 시퀀스의 `END_ERROR`는 `action_err`다 |
| 금지 스텝 | `AGV_SPEED`, `AGV_SPEED_MMIN`, `AGV_FORBACK`, `AGV_TURN`(옵션 모두), `AGV_MOVE`, `ERROR_HOLD`, `END_RESTART`. 주행 상태를 건드리므로 `seq_bind()`가 부팅 때 거부한다 |
| 시퀀스와 같은 출력 | 금지. 겹치면 같은 스캔에서 나중에 실행되는 ACT가 이긴다. 정지 태그로 시퀀스에 들어갈 때 ACT가 남아 있을 수 있으면 첫 스텝에 `WAIT_ACT( id )` |
| 정지 태그와 같이 사용 | `TAG( n, PF, AGV_STOP, ACT_START(id) )`. 정지와 동시에 시작하며 서로 기다리지 않는다 |

레거시와 달라지는 점: 레거시는 `stop_err`가 나도 `Gate` 펄스가 계속되고 STOP에서만 지운다. ACT는 오류에서도 멈춘다. 안전 쪽 변경이다.

시간 기준이므로 감속이나 장애물 정지가 끼면 동작이 끝나는 바닥 위치가 달라진다. 위치가 중요한 동작은 끝 위치에 태그를 하나 더 두고 그 태그로 끈다.

태그 없이 주행 내내 감시하는 동작은 ACT가 아니라 `site_manual()`에 둔다.

## 5. 태그 규칙 표

```c
#include "agv/tag_dsl.h"   /* 태그 표 동작 이름 */

TAG_TABLE_BEGIN( site_tags )

    /* 공통 */
    TAG( 1,  ANY_PROFILE,  AGV_STOP, AGV_AUTOSTART_PARAM )
    TAG( 2,  ANY_PROFILE,  AGV_SPEED( LLOW ) )
    TAG( 3,  ANY_PROFILE,  AGV_SPEED( LOW ) )
    TAG( 5,  ANY_PROFILE,  AGV_SPEED( HIGH ) )
    TAG( 21, ANY_PROFILE,  AGV_TURN( CW, 90, 0, 1 ) )
    TAG( 23, ANY_PROFILE,  AGV_TURN( CW, 90, 0, 1, 0, TAPE ) )       /* 교차 테이프를 찾은 뒤 정렬 회전 */
    TAG( 24, ANY_PROFILE,  SEQ_START( TAPE, LOAD ) )                 /* 교차 테이프에서 서고 시퀀스 */
    TAG( 25, ANY_PROFILE,  AGV_STOP( 30 ) )                          /* 태그 자리에서 30 cm 가서 정지(SL 과 같은 cm) */
    TAG( 26, ANY_PROFILE,  AGV_STOP( TAPE_GD ), ACS_SEND( STANDBY, 3 ) )   /* 회전 중심을 교차선에 맞춰 서고 대기 보고 */

    /* 적재 태그 4: 프로파일별. 위에서 아래로 첫 번째로 맞는 규칙 하나만 실행 */
    TAG   ( 4, PF_MATERIAL,  AGV_STOP, SEQ_START( LOAD ) )
    TAG   ( 4, PF_FRAME,     AGV_STOP, SEQ_START( LOAD_FINISH ) )
    TAG_IF( 4, PF_TUB,       IF_FORBACK_MODE,  AGV_STOP, SEQ_START( LOAD ) )
    TAG   ( 4, ANY_PROFILE,  AGV_STOP )

TAG_TABLE_END
```

- `tagact_run(tag)`는 활성 프로파일과 번호가 맞는 첫 규칙을 찾아 액션을 순서대로 실행한다. 규칙이 없으면 아무것도 하지 않는다.
- 액션은 규칙당 4칸이다. `AGV_TURN`은 2칸을 쓴다. 넘치면 `CALL( 인덱스 )`.
- 회전 `AGV_TURN( 방향, 각도, 회전 뒤 방향, 자동 출발[, 속도[, 옵션…]] )`(3.2.2). 옵션 `ALIGN` 주행로 정렬, `TAPE` 교차 테이프를 찾은 뒤 정렬(둘 중 하나), `OBS` 회전 중 장애물 감지. 속도는 m/min 0\~31이고 0이나 생략은 자동(적재 `TNS`, 공차 `ETN`). 값이 있으면 적재·공차와 상관없이 그 회전 한 번만 그 속도다. 회전 최저 속도보다 낮으면 최저 속도, `TNS` 한계(25)보다 높으면 한계로 묶는다. 회전 끝 감속·정렬·회전 정지 계획도 그 값을 상한으로 쓰고, 회전이 끝나거나 STOP·오류로 끊기면 자동으로 돌아간다. 예: `AGV_TURN( CW, 90, 0, 1, 15 )` 15 m/min, `AGV_TURN( CW, 90, 0, 1, 0, ALIGN, OBS )`. 둘째 칸 = 회전 뒤 방향 b0\~3 | 자동 출발 b4\~7 | 장애물 b8 | 속도 b11\~15.
- 정지는 4분류다(5.2): `AGV_STOP`(`SL`), `AGV_STOP( cm )`, `AGV_STOP( TAPE )`, `AGV_STOP( TAPE_GD )`. 정지 뒤 시퀀스는 `SEQ_START( [정지 방식,] 시퀀스 )`(이 동작만 옵션이 앞이다. 예전 `AGV_TAPE_STOP` 은 `SEQ_START( TAPE, 시퀀스 )`). 숫자 속도는 `AGV_SPEED_MMIN( m/min )`(4\~255).
- 보고·대기·스테이션은 태그 표 파일에서 `agv/tag_dsl.h`를 포함하고 시퀀스 표와 같은 이름으로 쓴다. `ACS_SEND( 종류, 번호 )`(0\~255, 2칸), 번호 자리에 `STATION(k)`(스테이션 큐[k], 레거시 `ACS_Station_Call_Num[k]`) 또는 `AGV_ID_MINUS(n)`(호기 번호 - n), `ACS( 종류, STATION0 )`(1칸, 정보 값이 8비트뿐이라 `STANDBY_RESET`은 컴파일 오류. 대기 리셋은 2칸 `ACS_SEND`), `STATION_RESET`. 대기 `ACS_SEND( STANDBY, n )`은 레거시 `DA_ACS_Standby( n, NONE )` + 자동 재전송 켬(정지, 번호 > 0 이면 자동 재전송 늘 켬. 번호 0 이하(호기 - n 이 0 이하 등)는 한 번만 보낸다), `ACS_SEND( STANDBY_RESET, n )`은 `DA_ACS_Standby( n, RESET )`(먼저 스테이션 큐를 지운다)이다. 예: `TAG( 30, PF_X, AGV_STOP, ACS_SEND( STANDBY, 1 ), STATION_RESET )`. 둘째 칸 = Info | `TA_ACS_STANDBY_RESET`(0x100) | `TA_ACS_AGV_MINUS`(0x200) | `TA_ACS_STATION`(0x400). 시퀀스 표의 `ACS_SEND( STANDBY, … )`는 보고만 한다. `STATION[k]`(대괄호)는 쓰지 않는다: 문자열 첨자 같은 우회는 C 정수 상수식이 아니라 컴파일 때 범위 검사(`sizeof` 배열)와 같이 못 쓰고 표준 밖 확장에 기댄다. 숫자 칸 `( TA_ACS | DSL_INFO_… )`, `( TA_ACS_NUM | n ), DSL_INFO_…`도 쓸 수 있다. 예전 `A_` 이름(`A_STOP` 등)은 v0.47.0 에서 뺐다(대응표는 CHANGELOG v0.47.0).
- 조건은 `TAG_IF`의 셋째 인자 하나. 확장 조건 둘과 입력 조건 하나를 AND 한다.

| 매크로 | 뜻 |
|---|---|
| `IF_X( x )` / `IF_X2( x1, x2 )` | 확장 조건(3.3.1) 하나 / 둘 |
| `IF_IN( 조건 )` | 입력 조건. 시퀀스와 같은 `IN`·`ON`·`OFF`·`ANY_ON` |
| `IF_X_IN( x, 조건 )` / `IF_X2_IN( x1, x2, 조건 )` | 확장 조건 + 입력 조건 |
| `IF_FORBACK_MODE`, `IF_LOW_BATTERY`, `IF_FORWARD`, `IF_BACKWARD` | `IF_X( X_… )` 줄임 |
| `IF_STATION( 칸, 하한, 상한 )`, `IF_STATION_RUNNING( … )` | 큐 범위 (/ + 정지 전) |
| `IF_ID( lo, hi )` | 프로파일 안 일부 호기 |
| `IF_STATION_MOD( k, n, m )`, `IF_STATION_DIV( k, n, m )` | 큐[k] % n == m, 큐[k] / n == m |
| `IF_TAG_MINUS_EQ_STATION( k, n )`, `IF_TAG_EQ_STATION( k )` | 태그 - n == 큐[k], 태그 == 큐[k] |

```c
TAG_IF( 6,  PF_TUB,  IF_IN( ON(IN_EXT_4) ),   SEQ_START( UNLOAD ), AGV_FORBACK( TOGGLE ) )   /* 입력 켜짐일 때만 */
TAG   ( 6,  PF_TUB,  AGV_STOP )                                                   /* 이 줄이 없으면 뒤 공통 규칙 */
TAG_IF( 51, PF_LIFT, IF_X2( X_ID(85, 85), X_STATION(1, 42, 42) ),  AGV_TURN( CCW, 90, 0, 1, 0 ) )
TAG_IF( 52, PF_LIFT, IF_X_IN( X_ID(81, 84), IN( OFF(SEN_1) ) ),    AGV_STOP )
TAG_IF( 45, PF_LIFT, IF_TAG_MINUS_EQ_STATION( 1, 40 ),             AGV_STOP )       /* 큐[1] == 5 */
```

- 조건이 거짓이면 다음 규칙을 본다. "조건이 아니면 아무것도 안 함"이면 뒤에 액션 없는 규칙 `TAG( n, PF, 0 )`을 둔다.
- 태그 번호 조건은 규칙의 `tag`를 쓴다. 맞은 규칙의 `tag`가 곧 지금 태그다. 1xx·2xx 별칭을 `tagact_run( 번호 % 100 )`으로 다시 찾는 사이트는 변환된 번호로 평가된다(레거시 `Tag_Num = j`와 같음). 별칭 규칙 자체에 쓰면 1xx·2xx 번호다.
- `X_KEY`·`X_CROSS_*`도 쓸 수 있다. 표로 안 되는 동작은 `CALL`.
- 규칙 1개는 16워드다(예전 9워드). 새 칸은 `act` 뒤에 있어 `{ tag, profile, cond, a, b, { act } }` 직접 초기화와 `TAG(...)`는 그대로 된다.
- 압축 표(생성기용, v0.33.0): 규칙이 많아 플래시가 모자라면 확장 조건·입력 조건·동작을 중복 없는 표(`TAG_X`·`TAG_IN`·`TAG_ACTS`)로 빼고 규칙은 표 번호 3워드(`TAGP(태그, 프로파일, 확장 조건 1, 확장 조건 2, 입력, 동작)`)로 둔다. 번호 0 = 없음, 표 번호 0\~255, 태그 0\~1023, 프로파일 0\~63. `TAG_PACKED`로 묶어 `tagact_bind_packed`로 넘긴다. 맞춰 보는 규칙은 16워드 규칙으로 펼쳐 같은 길로 돈다. 손으로 쓰는 표는 16워드 그대로 쓴다.

### 5.1 프로파일

`P_AGV_TYPE`과 `P_AGV_ID`로 프로파일을 고른다. 레거시의 `set[AGV_ID]` 범위 분기와 그룹 매크로가 전부 이 표로 옮겨진다.

```c
PROFILE_TABLE_BEGIN( site_profiles )
    PROFILE( PF_TUB,       100, 1,  7  )
    PROFILE( PF_MATERIAL,  100, 8,  8  )
    PROFILE( PF_LIFT,      100, 81, 93 )
PROFILE_TABLE_END
```

### 5.2 태그 정지 4분류 (v0.50.0)

| 분류 | 정지 | 정지 뒤 시퀀스 | 서는 자리 | 쓰임 |
|---|---|---|---|---|
| 1 | `AGV_STOP` | `SEQ_START( x )` | 태그를 읽은 자리에서 `SL` 만큼 가서 | 대기·교통 정지. 지금 동작 |
| 2 | `AGV_STOP( cm )` | `SEQ_START( cm, x )`(2칸) | 태그를 읽은 자리에서 cm 가서. `SL` 과 같은 cm, 0\~255, 0 = 바로 | 태그마다 거리를 달리할 때 |
| 3 | `AGV_STOP( TAPE )` | `SEQ_START( TAPE, x )` | 교차선을 찾아 센서가 선 위인 자리에서 바로 | 적재·하역 자리처럼 정밀도가 필요한 정지 |
| 4 | `AGV_STOP( TAPE_GD )` | `SEQ_START( TAPE_GD, x )` | 교차선을 찾아 `GD` 더 가서(회전 중심이 선 위) | 회전 자리. 교차선 회전 `AGV_TURN( …, TAPE )` 과 같은 루틴 |

고르는 기준:

- 태그 위치로 충분하면 1(모든 태그 같은 `SL`) 또는 2(태그마다 거리). 2 의 거리는 1 과 같은 기준점(태그를 읽은 자리)에서 재고, 정지 제어도 같다. 거리가 그 속도의 제동 거리보다 짧으면 `SL` 이 짧을 때와 같다.
- 장비와 맞추는 정밀 위치는 바닥에 교차선을 붙이고 3. 그 자리에서 돌거나 회전 중심을 선에 맞춰야 하면 4.
- 3·4 는 태그 뒤 `SL` 의 2배 안에 교차선이 있어야 한다. 없으면 서고 가이드 센서 오류(258). 찾는 속도는 3 이 최저 속도(`SPN`), 4 가 교차선 회전과 같은 계산값이다.
- 거리는 `SL` 과 같은 cm 를 숫자로 적는다(예: `AGV_STOP( 30 )` = 30 cm). 0\~255 밖이거나 정하지 않은 낱말은 컴파일 오류. 인코딩: 분류 2 `TA_STOP_CM`(인자 cm), 3 `TA_TAPE_STOP`, 4 `TA_TAPE_STOP_GD`(인자 = 시퀀스, 0 = 없음). 분류 1·3·4(+시퀀스)는 1칸, 2 + 시퀀스는 2칸이다.

같은 규칙 안의 겹침:

- 규칙에 분류 2·3·4 가 있으면 규칙 안 순서와 무관하게 그 정지가 이긴다. 같은 규칙의 `AGV_STOP`·`SEQ_START( x )`·`AGV_AUTOSTART…`·`ACS_SEND( STANDBY… )` 는 태그 자리 `SL` 정지를 걸지 않는다. 대기 보고는 태그를 읽은 때 보고·자동 재전송만 한다. 분류 3·4 규칙의 `SEQ_START( x )` 는 교차선 정지 뒤 시퀀스가 된다(교차선 칸에 시퀀스가 없을 때).
- 자동 출발: 분류 2 규칙은 태그 때 재출발 예약을 건다(분류 1 과 같다). 분류 3·4 규칙은 태그 때 걸지 않고 보관했다가(`D.tape_autostart`) 교차선 정지를 실제로 건 때 건다. 교차선 전에 다른 원인(교통 정지 등)으로 섰을 때 스스로 다시 떠나 그 정지를 어기지 않게 한다.
- 분류 2 와 3·4 가 같이 있으면 3·4 가 이긴다. 교차선 정지 둘은 금지(아래).
- 다른 정지가 진행 중일 때: 분류 2 는 진행 중인 정지 거리를 바꾸지 않는다(분류 1 과 같다). 분류 3·4 요청은 받고, 주행 쪽이 다른 정지 중에는 찾기를 보류했다가 다시 달리면 잇는다(5.3).
- 번호 변환 별칭(사용자 함수가 `tagact_run` 을 다시 부름)에서는 안쪽 규칙이 정한 정지 방식이 남는다(바깥 호출이 `SL` 로 덮지 않는다).

금지 조합(뜻이 모호하다. 압축 표 생성기가 규칙 번호·태그와 함께 오류로 잡는다. 손으로 쓰는 펼친 표는 검사하지 않는다):

- 회전(일반·`ALIGN`·`TAPE`)과 분류 2·3·4 정지
- 회전 둘(뒤 회전은 거부된다)
- 교차선 정지(분류 3·4) 둘(뒤의 것만 남아 앞 칸의 시퀀스가 사라진다)
- 분류 3·4 규칙에 시퀀스 둘(교차선 칸의 시퀀스 + `SEQ_START( y )`. 하나는 버려진다)
- 분류 3·4 와 `AGV_SPEED…`(찾는 동안 찾기 속도가 덮는다)

### 5.3 교차선 요청의 일생, 인정, 교차선 뒤 `GD` 구간 (v0.50.0)

교차선 요청은 교차선 회전(`AGV_TURN( …, TAPE )`)과 분류 3·4 정지(정지 뒤 시퀀스 포함)다.

거는 조건(태그 쪽):

- 자동 주행 중(`RUN_DRIVE` 로 달리는 중, 정지 감속 포함)에 읽은 태그만 요청이 된다. 조그·수동으로 움직이며 읽은 태그, 서 있는 차가 (같은 자리 태그를) 다시 읽은 것은 요청이 되지 않는다.
- 교차선 회전 요청은 일반 회전이 거부되는 상황(회전 모드 0, 회전 중, 거리 정지 진행 중, 서 있음)과, 180도 이상인데 회전 뒤 방향이 지금 방향과 반대인 조합(회전을 걸 수 없다)에서는 걸지 않는다. 명령 정지(ACS 교통 정지 등) 감속 중에는 건다(레거시는 이때 `SL` 정지를 다시 걸고 돈다).
- 분류 3·4 요청은 자동 주행 중이면 다른 정지가 진행 중이어도 받는다.

보류(주행 쪽): 요청이 걸려 있어도 다른 정지(거리 정지·명령 정지)가 진행 중이거나 주행 상태가 아니면 속도 지령·인정을 건드리지 않고 요청을 둔다. 다시 달리면 찾기를 잇는다. 찾기 한계(`SL` 의 2배)는 요청을 건 자리부터 실제로 간 거리로 센다. 장애물 일시정지는 일시정지 기구가 맡고 풀리면 잇는다.

지우는 때: 처리 완료(인정 뒤 정지를 건 때), 한계 초과(서고 258), 그리고 사람의 STOP, 비상 정지, 오류(범퍼 포함), 자동 → 조그 전환, ACS 작업 취소('C' 2·3, 'W' 2·3)다(`run_tape_req_clear`, `run_seq_init` 이 부른다). **다시 출발(START, ACS 출발, 자동 출발)로는 지우지 않는다.** 레거시는 출발(`Initial_Start` → `Auto_Loading_Init`)에서도 지웠다. 레거시의 일반 회전은 태그 자리에서 곧 서고 돌아 "요청이 걸린 채 다시 출발"하는 상태가 없었다. 교차선 회전에서는 회전 태그와 교차선 사이에서 ACS 교통 정지가 걸리면 레거시처럼 지울 경우 차가 돌지 않고 직진하므로 일부러 다르게 했다. 다른 정지의 감속 중에 교차선을 지나쳐 버리면 다시 출발한 뒤 한계에서 258 로 선다(조용히 직진하지 않는다).

찾는 동안의 태그: 교차선 회전 요청이 걸린 동안(찾는 중·보류 중)에는 회전 중과 같이 새 태그를 ACK 없이 버린다(서 있는 차가 같은 자리 태그를 다시 읽는 경우 포함). 교차로 IN·OUT 태그는 보고(IN 보고·허가 대기, OUT 보고 큐)한 뒤 서고 오류 290 이다. 분류 3·4 정지를 찾는 동안의 태그는 지금처럼 처리한다.

인정(주행 쪽, `run_tape_rx`): 새 가이드 응답마다 본다. 한쪽 끝 칸부터 주행 테이프 덩어리 바로 옆까지 빈 칸 없이 켜졌고, 그 칸이 wide(32칸) 8칸·16칸 센서 4칸 이상이며, 끝 칸이 켜진 뒤 20 mm 안에 다 찼으면(또는 전부 켜짐) 교차선 위 응답이다. 이런 응답이 이어서 두 번이면 인정한다(아닌 응답에 0, 새 응답이 없는 틱은 세지 않는다). 주행 테이프가 켠 칸은 세지 않고, 양쪽(+)·한쪽(T·L) 모두 인정한다. 비스듬한 분기선은 주행 테이프 옆부터 끝까지를 채우지 못하고, 합류선은 끝 칸이 켜지고 한참 뒤에 차서 인정하지 않는다. 16칸 센서는 한쪽이 4\~6칸뿐이라 넓은(50 mm) 분기·합류선에서 오인정이 남고(모의 912건 중 87건), 50 mm 주행선의 T 모양을 놓칠 수 있다(540건 중 112건). wide 는 모의 오인정 0.

교차선 뒤 `GD` 구간(분류 4·교차선 회전·`AGV_MOVE( 한계, TAPE_GD )`): 인정한 틱부터 설 때까지 조향 보정을 멈춘다(두 바퀴 같은 지령, 적분 지움). 정지 거리 = `GD` + 교차선 반폭 − 응답 늦음 − 인정까지 간 거리. 반폭은 먼저 10 mm 로 걸고 뒤 가장자리(아닌 응답 둘)를 보면 잰 값으로 고친다. 60 mm 안에 못 보면 30 mm 로 고친다. 응답 늦음은 18 ms × 속도로 **모의로 맞춘 값**이라 실차에서 `run_tape_last.width_mm`(잰 교차선 폭)가 테이프 폭 + 약 2 mm 인지로 확인한다. 선이 교차선에서 끝나도(T·L) 이 구간에서는 선 없음 오류를 내지 않는다. 모의 회전 중심 오차(16800건): 표준편차 15.4 → 3.7 mm, 평균 −14.4 → −2.1 mm. 분류 3 은 방향 0°에서 예전과 같은 자리에 선다.

기록: `run_tape_last`(`kind` 3·4·−1(교차선 회전), `ticks`, `run_mm` 인정부터 선 자리까지 간 거리, `width_mm` 잰 교차선 폭(−1 = 못 봄·분류 3), `n`)이 설 때마다 바뀐다. 교차선 정지는 태그 위치 정지(`STOP_REASON_TAG_DOCKING`, 교차선 회전 앞은 `BEFORE_TURN`, 시퀀스 이동은 `MOVE`)라 `stop_last` 에도 남는다. 분류 3 은 목표 0 이라 넘침 = 인정 뒤 간 거리다. 분류 4 의 `stop_last.target_mm` 은 처음 건 거리이고 넘침은 반폭으로 고친 목표 기준이다. 간 거리는 `run_tape_last.run_mm` 로 본다.

배포 점검:

- 분류 3·4 와 교차선 회전 태그는 태그 뒤 `SL` 의 2배 안에 교차선(교차하는 주행선 포함)이 있어야 한다. 없으면 서고 258.
- 16칸 센서 차종은 넓은 분기·합류선이 있는 곳에서 오인정, 넓은 주행선의 T 모양에서 놓침이 남는다. 적용 전 현장 선 배치를 확인한다.
- 시퀀스 이동 중 ACS 교통 정지가 걸리면 277 로 서고 리셋이 필요하다. 시퀀스 이동 경로에 교차로를 두지 않는다(교차로 태그를 만나면 290).

## 6. 도구

생성기는 없다. 표는 C 파일이 원본이다.

| 도구 | 입력 → 출력 | 상태 |
|---|---|---|
| `tests/test_dsl.c` | 매크로 → 표 값 검사, 컴파일 오류 시험 | 있음 |
| `tools/seqdump` | 사이트 표를 호스트로 빌드해 스텝 표 문서, 태그 표, RFID 매니저 태그 목록, 게이트웨이 이름 표 출력 | 예정 |
| `tools/legacy2table.py` | 레거시 Transfer → 표 C 초안 | 예정 |

문서를 표에서 뽑으므로 C 소스를 정규식으로 해석하지 않는다. 컴파일된 표를 그대로 읽는다.

## 7. 변환 절차 (레거시 → 표)

1. **분할**: 레거시 파일을 타입별로 전처리해 앱 저장소 `legacy/by_type/`에 둔다.
2. **자동 변환**: `legacy2table.py`가 `case`를 패턴 매칭해 C 표 초안을 만든다. 매칭 실패 블록은 `CALL` 자리표시와 원본 주석으로 남긴다.
3. **검토**: 입력·출력 이름을 사이트 별칭으로 바꾼다. 미연결 출력은 `io_map.h`에서 `IO_NONE`으로 두면 무시된다.
4. **동등성 시험**: 호스트에서 원본 Transfer 함수와 표를 같은 입력 시퀀스로 돌려 출력·플래그·스텝 궤적을 비교한다(`tests/legacy_ref/`). 통과한 시퀀스만 변환 완료로 표시한다.
5. **사용자 정의 함수 작성**: `DA_Manual_Act` ID별 블록, 포크 수동 시퀀스 등.
6. **ACT 분리**: `Gate*_Out`이 주행 중 출력 패턴을 만드는 경로를 ACT로 옮긴다.
7. **빌드·크기 확인**: `.map`으로 코드 크기 비교.
8. **실차**: 시험 차량 1대에서 `SQ`·`TT`로 정지 상태 검증 후 주행.

나머지 타입은 1단계까지만 하고, 그 사이트의 펌웨어 갱신이 필요할 때 2\~8단계를 수행한다. 변환 상태는 앱 저장소 `legacy/STATUS.md`로 관리한다.

## 8. 변환 예 (가상의 적재 시퀀스)

레거시 양식의 스텝 6개를 표로 옮긴 예다. 실제 사이트 변환 예는 앱 저장소 `docs/레거시_분석과_변환예.md`에 있다.

```c
case 0:
    if (In_4 == nON && In_5 == nON) { Belt2(1,1); step = 1; }
    else if (In_4 == nOFF && In_5 == nOFF) { step = 2; }
    break;
case 1:
    if (In_4 == nOFF && In_5 == nOFF) { cnt_1++; if (cnt_1 > 300) { Belt2(0,0); step = 2; } }
    else { cnt_1 = 0; cnt_2++; if (cnt_2 > 2000) { Belt2(0,0); step = 290; } }
    break;
case 290:
    action_err = 290;
    if (START_SW == nON) step = 1;
    break;
case 2:
    Out_Req = nON;
    if (In_11 == nON) { Belt1(0,1); step = 3; }
    break;
case 3:
    if (In_1 == nON && In_2 == nON) { cnt_1++; if (cnt_1 > 300) Belt1(0,0); } else cnt_1 = 0;
    if (Belt1_Power == nOFF) { Out_Req = nOFF; if (In_11 == nOFF) step = 11; }
    break;
case 11:
    Flag_Wait_Start = ON; step = 0; Load_Flag = OFF; cnt_1 = cnt_2 = 0;
    break;
```

```c
SEQ_LABELS( EMPTY_CV2, WAIT_EMPTY, ERR_READY, REQUEST )

SEQ_BEGIN( example_LOAD )

    /* 0. CV#2 에 자재가 있으면 먼저 비우고, 없으면 바로 요청으로 */
    SELECT_BEGIN
        WHEN(  IN( ON (CV2_SEN_A, CV2_SEN_B) ),   EMPTY_CV2  )
        WHEN(  IN( OFF(CV2_SEN_A, CV2_SEN_B) ),   REQUEST    )
    SELECT_END

    /* 1. 3초 안정이면 완료, 불일치 누적 20초면 오류 */
    L( EMPTY_CV2 )
        BELT_ON( 2, CCW )

    L( WAIT_EMPTY )
        WAIT_STABLE_TO(  IN( OFF(CV2_SEN_A, CV2_SEN_B) ),
                         300,
                         UNMATCHED( 2000 ),   ERR_READY )
        BELT_OFF( 2 )
        GOTO( REQUEST )

    /* 290. 오류 표시 후 START 스위치로 재시도 */
    L( ERR_READY )
        BELT_OFF( 2 )
        ERROR_HOLD( 290,   IN( ON(START_SW) ),   WAIT_EMPTY )

    /* 2. 설비에 요청, 응답 오면 CV#1 투입 */
    L( REQUEST )
        OUT_ON( REQ_OUT )
        WAIT(   IN( ON(ACK_IN) ) )
        BELT_ON( 1, CW )

    /* 3. 센서가 3초 안정되면 정지, 요청 해제, 응답 해제 대기 */
        WAIT_STABLE(  IN( ON(CV1_SEN_A, CV1_SEN_B) ),   300 )
        BELT_OFF( 1 )
        OUT_OFF( REQ_OUT )
        WAIT(   IN( OFF(ACK_IN) ) )

    /* 11. 재출발 */
        END_RESTART

SEQ_END
```

레거시와 다른 점 두 가지는 의도된 수정이다.

- 레거시는 290에서 복귀해도 `cnt_2`를 지우지 않아 두 번째 시도의 타임아웃이 곧바로 걸린다. 표는 스텝에 다시 들어갈 때 타이머를 0으로 한다.
- 스텝 3의 두 조건은 뒤 조건이 앞 조건의 결과(벨트 정지)로만 참이 되므로 순차 스텝으로 풀었다.

## 9. 사내 일상 수정 흐름 (리팩토링 뒤)

| 하고 싶은 것 | 고치는 파일 | 절차 |
|---|---|---|
| 적재 시퀀스 타임아웃 변경 | `seq_table.c` | 값 수정 → 빌드 → `SQ`로 정지 시험 |
| 주행 중 동작 변경 | `seq_table.c`의 ACT | 값 수정 → 빌드 → `AC`로 정지 시험 |
| 새 호기 추가(기존 차종) | `site.c`의 프로파일 표 | 행 추가 → 빌드 |
| 새 차종 | `site/EXAMPLE` 복사 후 표 작성 | 호스트 시험 → `SQ`/`TT` |
| 태그 번호 동작 변경 | `tag_table.c` | 규칙 수정 → 빌드 → `TT` 건식 실행으로 확인 |
| 배선 변경 | `io_map.h` | 별칭·벨트 마스크 수정 → 빌드 |
| 표로 안 되는 동작 | `custom.c` | `CALL` 함수 작성. 코어 공개 API만 사용 |
| 코어 수정 | `AGV_DCU_Core` | 사내 수정 금지. 이슈·PR로 코어 릴리스에 반영 후 벤더링 갱신 |

사용자가 일상적으로 만지는 것은 사이트 폴더의 C 파일뿐이다. 코어는 라이브러리로 들어온다.
