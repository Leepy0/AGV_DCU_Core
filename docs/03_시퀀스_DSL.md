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
| ACS | `ACS( L, STATION0 )` / `ACS_N( L, 87 )` | 보고. 번호 출처 `STATION0`, `STATION0_PLUS1`, `STATION(k)`(큐[k]), `AGV_ID` 또는 상수 | `DA_ACS_Loading(…)` |
| | `ACS_WAIT_ACK( 제한, 라벨 )` | ACK 대기 | `Scia_Send_AGV2PC_Message == ACK_NONE_MSG` |
| | `ACS_STATION( 칸, 값, 길이 )` / `STATION_RESET` | 스테이션 큐 | `ACS_Station_Call_Num[1] = 0;` |
| 주행 | `SPEED( HIGH·LOW·LLOW·MIN )` / `SPEED_MMIN( v )` | 재출발 속도 | `gDriving_speed = …` |
| | `FORBACK( FWD·BACK·TOGGLE )` | 방향 전환 예약 | `AGV_ForBack_Action(FB)` |
| | `TURN( CW, 90, fb, auto, obs )` | 회전 요청 + 완료 대기 | `AGV_Turn_Action(…)` |
| | `TURN_NOWAIT( CW, 90, fb, auto, obs )` | 회전 요청, 다음 스캔에 다음 스텝. 회전 중 뒤 스텝이 돈다 | `AGV_Turn_Action(…); step++;` |
| | `OBS_ON` / `OBS_OFF` | 장애물 센서 사용 | `Flag_OBS_Sen_Use` |
| 상태 | `SET_STATE( id, 값 )` / `WAIT_STATE( id, 값 )` | 사이트 상태 변수 | `Gate1_Out`, `FL_check` 류 |
| 병렬 | `ACT_START( id )` / `WAIT_ACT( id )` | 4절 | |
| 확장 | `CALL( 인덱스 )` | 사용자 정의 함수 | 표로 안 되는 블록 |
| 종료 | `END_RESTART` / `END_STOP` / `END_ERROR( 코드 )` | 재출발 예약 / 그냥 종료 / 오류 종료 | `case 11` / 플래그 OFF / `action_err = n` |

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

- `ANY_ON`은 나열한 신호 중 하나라도 켜지면 참이다. `IN`으로 `ON`·`OFF`와 묶는다. `ON(x)`와 `OFF(x)`를 같이 쓰면 컴파일 오류 없이 `ANY_ON(x)`로 읽히니 주의한다.
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

- `Auto_Run_Process`의 디스패치 조건은 `run` 모듈이 맡는다. 회전 중에도 호출되는 현행 동작은 유지해 `TURN` 스텝이 동작한다.
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
/* 태그 표: A_STOP 이 없으므로 주행을 계속한다 */
TAG( 41, PF_LIFT, A_ACT( ACT_BELT_5S ) )

/* 상태 조건: 스테이션 큐[1] 이 101~116 일 때만 */
TAG_IF( 36, PF_LIFT, IF_STATION( 1, 101, 116 ), A_ACT( ACT_LIFT_SIGNAL ) )
```

마지막 스텝이 끝나면 ACT는 자동으로 끝난다. 다음에 스텝이 더 있는 지점에서 끝내려면 `END_STOP`을 쓴다.

### 4.3 규칙

| 항목 | 동작 |
|---|---|
| 시작 | 태그 `A_ACT(id)`, 시퀀스 `ACT_START(id)`, 관리 명령 |
| 실행 | 매 10 ms, 주행·정지와 무관. 슬롯 4개, 슬롯별 타이머 |
| 같은 ACT 재요청 | 무시. 레거시 `Gate1_Out = ON` 재대입과 같다 |
| 슬롯 부족 | 시작하지 않고 로그에 남긴다 |
| 장애물·교차로 일시정지 | 계속 돈다. 시간도 계속 흐른다 |
| STOP, E-STOP, `W` 리셋, `system_err`, `stop_err` | 즉시 중단하고 **그 ACT가 켠 출력만** 끈다 |
| 실패 | `END_ERROR(code)`는 `stop_err = code`로 주행을 멈춘다. 시퀀스의 `END_ERROR`는 `action_err`다 |
| 금지 스텝 | `SPEED`, `SPEED_MMIN`, `FORBACK`, `TURN`, `TURN_NOWAIT`, `ERROR_HOLD`, `END_RESTART`. 주행 상태를 건드리므로 `seq_bind()`가 부팅 때 거부한다 |
| 시퀀스와 같은 출력 | 금지. 겹치면 같은 스캔에서 나중에 실행되는 ACT가 이긴다. 정지 태그로 시퀀스에 들어갈 때 ACT가 남아 있을 수 있으면 첫 스텝에 `WAIT_ACT( id )` |
| 정지 태그와 같이 사용 | `TAG( n, PF, A_STOP, A_ACT(id) )`. 정지와 동시에 시작하며 서로 기다리지 않는다 |

레거시와 달라지는 점: 레거시는 `stop_err`가 나도 `Gate` 펄스가 계속되고 STOP에서만 지운다. ACT는 오류에서도 멈춘다. 안전 쪽 변경이다.

시간 기준이므로 감속이나 장애물 정지가 끼면 동작이 끝나는 바닥 위치가 달라진다. 위치가 중요한 동작은 끝 위치에 태그를 하나 더 두고 그 태그로 끈다.

태그 없이 주행 내내 감시하는 동작은 ACT가 아니라 `site_manual()`에 둔다.

## 5. 태그 규칙 표

```c
TAG_TABLE_BEGIN( site_tags )

    /* 공통 */
    TAG( 1,  ANY_PROFILE,  A_STOP, A_AUTOSTART_PARAM )
    TAG( 2,  ANY_PROFILE,  A_SPEED_LLOW )
    TAG( 3,  ANY_PROFILE,  A_SPEED_LOW )
    TAG( 5,  ANY_PROFILE,  A_SPEED_HIGH )
    TAG( 21, ANY_PROFILE,  A_TURN( CW, 90, 0, 1, 0 ) )

    /* 적재 태그 4: 프로파일별. 위에서 아래로 첫 번째로 맞는 규칙 하나만 실행 */
    TAG   ( 4, PF_MATERIAL,  A_STOP, A_SEQ( SEQ_LOAD ) )
    TAG   ( 4, PF_FRAME,     A_STOP, A_SEQ( SEQ_LOAD_FINISH ) )
    TAG_IF( 4, PF_TUB,       IF_FORBACK_MODE,  A_STOP, A_SEQ( SEQ_LOAD ) )
    TAG   ( 4, ANY_PROFILE,  A_STOP )

TAG_TABLE_END
```

- `tagact_run(tag)`는 활성 프로파일과 번호가 맞는 첫 규칙을 찾아 액션을 순서대로 실행한다. 규칙이 없으면 아무것도 하지 않는다.
- 액션은 규칙당 4칸이다. `A_TURN`은 2칸을 쓴다. 넘치면 `A_CALL( 인덱스 )`.
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
TAG_IF( 6,  PF_TUB,  IF_IN( ON(IN_EXT_4) ),   A_SEQ( SEQ_UNLOAD ), A_FORBACK( TOGGLE ) )   /* 입력 켜짐일 때만 */
TAG   ( 6,  PF_TUB,  A_STOP )                                                   /* 이 줄이 없으면 뒤 공통 규칙 */
TAG_IF( 51, PF_LIFT, IF_X2( X_ID(85, 85), X_STATION(1, 42, 42) ),  A_TURN( CCW, 90, 0, 1, 0 ) )
TAG_IF( 52, PF_LIFT, IF_X_IN( X_ID(81, 84), IN( OFF(SEN_1) ) ),    A_STOP )
TAG_IF( 45, PF_LIFT, IF_TAG_MINUS_EQ_STATION( 1, 40 ),             A_STOP )       /* 큐[1] == 5 */
```

- 조건이 거짓이면 다음 규칙을 본다. "조건이 아니면 아무것도 안 함"이면 뒤에 액션 없는 규칙 `TAG( n, PF, 0 )`을 둔다.
- 태그 번호 조건은 규칙의 `tag`를 쓴다. 맞은 규칙의 `tag`가 곧 지금 태그다. 1xx·2xx 별칭을 `tagact_run( 번호 % 100 )`으로 다시 찾는 사이트는 변환된 번호로 평가된다(레거시 `Tag_Num = j`와 같음). 별칭 규칙 자체에 쓰면 1xx·2xx 번호다.
- `X_KEY`·`X_CROSS_*`도 쓸 수 있다. 표로 안 되는 동작은 `A_CALL`.
- 규칙 1개는 16워드다(예전 9워드). 새 칸은 `act` 뒤에 있어 `{ tag, profile, cond, a, b, { act } }` 직접 초기화와 `TAG(...)`는 그대로 된다.

### 5.1 프로파일

`P_AGV_TYPE`과 `P_AGV_ID`로 프로파일을 고른다. 레거시의 `set[AGV_ID]` 범위 분기와 그룹 매크로가 전부 이 표로 옮겨진다.

```c
PROFILE_TABLE_BEGIN( site_profiles )
    PROFILE( PF_TUB,       100, 1,  7  )
    PROFILE( PF_MATERIAL,  100, 8,  8  )
    PROFILE( PF_LIFT,      100, 81, 93 )
PROFILE_TABLE_END
```

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
