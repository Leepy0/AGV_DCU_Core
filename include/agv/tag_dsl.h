/*
 * tag_dsl.h - 태그 표 동작 이름. seq_dsl.h 를 포함한다. 태그 표 파일에서만 포함한다.
 * 시퀀스 표와 뜻이 같은 동작은 같은 이름이다(ACS·ACS_SEND·STATION_RESET·OBS_ON·OBS_OFF·OBS_OFF_AUTO·CALL·ACT_START).
 * 이 헤더 뒤로 그 이름은 시퀀스 스텝이 아니라 태그 동작 칸을 만든다. 차가 움직이는 동작은 AGV_ 로 시작한다.
 *
 * 방식 옵션은 인자 맨 뒤에 적는다. 안 적으면 일반 동작이다. 정해 두지 않은 옵션·조합은 컴파일 오류다.
 * 태그 정지 4분류(정지 뒤 시퀀스도 같다). 한 규칙에 분류 2·3·4 가 있으면 그 정지가 이긴다(순서와 무관):
 * 같은 규칙의 AGV_STOP·SEQ_START( 시퀀스 )·AGV_AUTOSTART·ACS_SEND( STANDBY… ) 는 태그 자리 SL 정지를 걸지 않는다.
 *   AGV_STOP                          1: 태그 자리에서 SL 만큼 가서 선다
 *   AGV_STOP( cm )                    2: 태그 자리에서 cm 가서 선다. 0~255, 0 = 바로. 숫자로 적는다(표의 거리는 모두 cm, SL 과 같다)
 *   AGV_STOP( TAPE )                  3: 교차선을 찾아 본 자리에서 바로 선다
 *   AGV_STOP( TAPE_GD )               4: 교차선을 찾아 GD 더 가서 선다(회전 중심이 교차선 위)
 *   AGV_STOP( FL|cm, FROM_SENSOR( 입력, 유지틱 ) )   센서가 켜진 자리부터 거리만큼 가서 선다(아래). 교차선 요청과 한 규칙에 못 쓴다
 *   SEQ_START( 시퀀스 )                1 + 시퀀스(SEQ_ 뺀 이름: LOAD, UNLOAD, LOAD_1 …)
 *   SEQ_START( cm, 시퀀스 )            2 + 시퀀스 (2칸)
 *   SEQ_START( TAPE, 시퀀스 )          3 + 시퀀스
 *   SEQ_START( TAPE_GD, 시퀀스 )       4 + 시퀀스. 분류 3·4 규칙의 SEQ_START( 시퀀스 ) 도 교차선 정지 뒤 시작한다
 *   AGV_AUTOSTART( 초 ) / AGV_AUTOSTART_PARAM   서고 그 초(또는 AS) 뒤 다시 출발
 *   AGV_SPEED( HIGH | LOW | LLOW | MIN | m/min ) 주행 속도: 단계 낱말 또는 m/min 4~255(숫자·식). AGV_SPEED_MMIN 은 숫자만 받는 별칭
 *   AGV_FORBACK( FWD | BACK | TOGGLE )          전후진
 *   AGV_TURN( 방향, 각도, 회전 뒤 방향, 자동 출발[, 속도[, 옵션…]] )   제자리 회전. 옵션 ALIGN 주행로 정렬,
 *                                     TAPE 교차 테이프를 찾은 뒤 정렬(둘 중 하나까지), OBS 회전 중 장애물 감지
 *   AGV_BRANCH( LEFT | RIGHT )                  분기
 *   ACT_START( ACT ID )                        주행을 멈추지 않고 병렬 동작
 *   OBS_ON / OBS_OFF                             장애물 센서 사용
 *   OBS_OFF_AUTO                                 장애물 센서 끔. 일을 통째로 지우면(STOP·비상 정지·오류·ACS 작업 취소·수동 전환) 다시 켠다.
 *                                    다시 출발·방향 전환에서는 켜지 않는다. OBS_OFF 는 그때도 꺼진 채다(레거시)
 *   CALL( 인덱스 )                              사용자 함수
 *   ACS( 종류, 번호 )                 보고. 번호 칸(2칸 형식): 숫자 0~255, STATION(k)(스테이션 큐[k], 레거시 ACS_Station_Call_Num[k]),
 *                                     STATION(k) ± n(n -128~127), TAG_NUM ± n(지금 태그 번호 ± n, n -128~127), AGV_ID(= AGV_ID_MINUS(0)),
 *                                     AGV_ID_MINUS(n)(호기 번호 - n, n 0~255). ACS_SEND 는 같은 형식의 옛 이름(AGV_ID 낱말 말고)
 *   ACS( 종류, STATION0 )             보고, 번호 = 스테이션 큐[0]  (1칸 형식. STANDBY_RESET 은 컴파일 오류, 2칸 형식으로)
 *   AGV_WAIT_START / AGV_WAIT_CANCEL    출발 대기 걸기·풀기
 *   ACS( STANDBY, 번호 )              대기 보고: 정지(분류 2·3·4 규칙에서는 그 정지, 정지가 진행 중에 읽은 태그면 정지 없이), 번호 > 0 이면
 *                                     자동 재전송 늘 켬(레거시 DA_ACS_Standby( n, NONE ) + 자동 재전송 켬). 다른 보고는 ACK 까지 재전송만 한다
 *   ACS( STANDBY_RESET, 번호 )        위와 같고 먼저 스테이션 큐를 지운다(레거시 DA_ACS_Standby( n, RESET ))
 *   STATION_RESET                    스테이션 큐를 지운다
 *   NO_ACTION                        아무 동작 없음(0). 뒤 규칙으로 넘어가지 않게 막는 용도
 * 종류는 seq_dsl.h DSL_INFO_* 이름(LOADING UNLOADING NEXT ARRIVE FINISH PROCESSING STANDBY CALL IDLE DISPLAY).
 * 시퀀스 표의 ACS( STANDBY, … ) 도 보고 + 자동 재전송을 하지만 정지는 걸지 않는다(시퀀스는 이미 선 뒤에 돈다).
 *
 * 정지가 진행 중(감속·정지 진행 중, D.flag_stopping)에 읽은 다른 태그의 단순 정지(AGV_STOP·AGV_STOP( cm )·AGV_AUTOSTART, 대기 보고의 정지 부분)만
 * 모든 태그에서 건너뛴다(v0.58.0, tagact_run). 회전·교차선·SEQ_START·센서 정지·보고·출력·속도·전후진·ACT·CALL 은 그대로 실행한다.
 * P_LEGACY_QUIRKS b11(QUIRK_TAG_WHILE_STOPPING)을 켜면 이 무시를 끈다(예전 동작).
 * 태그 표 파일은 시퀀스 스텝 이름(WHEN·WAIT·L·GOTO·OUT_ON·DELAY·HANDSHAKE …)을 쓸 수 없다(아래 SEQ_STEP_NOT_ALLOWED_IN_TAG_TABLE 오류).
 */
#ifndef AGV_TAG_DSL_H
#define AGV_TAG_DSL_H

#include "agv/seq_dsl.h"

#undef ACS
#undef ACS_SEND
#undef STATION_RESET
#undef OBS_ON
#undef OBS_OFF
#undef OBS_OFF_AUTO
#undef CALL
#undef ACT_START

#undef AGV_SPEED
#undef AGV_SPEED_MMIN
#undef AGV_FORBACK
#undef AGV_TURN
#undef AGV_MOVE

/* AGV_STOP 은 괄호 없이 쓰면 이 상수(분류 1), 괄호를 쓰면 아래 매크로(분류 2·3·4) */
enum { AGV_STOP = TA_STOP };
/* 낱말 가리기: x 가 TAPE·TAPE_GD 면 DSL_xxx_TAPE… 가 "~, 값" 으로 펼쳐져 둘째 값이 뽑히고, 숫자면 dflt 가 뽑힌다(DSL_ARG2 는 agv/dsl_and.h) */
#define DSL_STOPCM_OK(cm)        (0 * sizeof(char[(((cm) >= 0) && ((cm) <= 255)) ? 1 : -1]))
#define DSL_STOPCM(cm)           (TA_STOP_CM | (uint16_t)((cm) + DSL_STOPCM_OK(cm)))
#define DSL_STOPW_TAPE           ~, TA_TAPE_STOP
#define DSL_STOPW_TAPE_GD        ~, TA_TAPE_STOP_GD
#define DSL_STOPW(x, dflt)       DSL_ARG2(DSL_STOPW_##x, dflt)
#define AGV_STOP(...)            DSL_CAT(DSL_AGVSTOP, DSL_NARG(__VA_ARGS__))(__VA_ARGS__)
#define DSL_AGVSTOP1(x)          DSL_STOPW(x, DSL_STOPCM(x))
/* AGV_STOP( 거리, FROM_SENSOR( 입력, 유지틱 ) ): 서지 않고 기다리다 입력(io_map 이름, 켜짐 = 감지)이 유지틱(10 ms, 1~511) 동안
 * 켜지면 처음 켜진 자리 + 거리에서 선다(이미 넘었으면 바로). 거리는 FL(정지 거리 파라미터) 또는 cm 0~255. 꺼짐이 5틱 이하면 이어
 * 센다. 요청 뒤 5 m 안에 감지가 시작되지 않으면 서고 오류 270. 이 동작 뒤에 적은 동작(최대 2칸)은 확정 때 실행한다. 2칸 */
#define DSL_FSD_FL               ~, 0
#define DSL_FSF_FL               ~, TA_FS_FL
#define DSL_AGVSTOP2(d, fs)      (TA_STOP_SENSOR | DSL_ARG2(DSL_FSD_##d, (uint16_t)((d) + DSL_STOPCM_OK(d)))), \
                                 ((fs) | DSL_ARG2(DSL_FSF_##d, 0))
#define DSL_FS_OK(in, t)         (0 * sizeof(char[(((in) >= 0) && ((in) < 64) && ((t) >= 1) && ((t) < 512)) ? 1 : -1]))
#define FROM_SENSOR(in, t)       ((uint16_t)((in) | ((t) << TA_FS_TICKS_SHIFT)) + DSL_FS_OK(in, t))
/* 태그 동작 칸의 인자는 8비트(0~255). 넘으면 윗비트가 동작 코드에 묻혀 조용히 다른 동작이 되므로 컴파일 오류 */
#define DSL_U8_OK(n)             (0 * sizeof(char[(((n) >= 0) && ((n) < 256)) ? 1 : -1]))
#define AGV_AUTOSTART(sec)       (TA_STOP_AUTOSTART | ((sec) + DSL_U8_OK(sec)))
#define AGV_AUTOSTART_PARAM      TA_STOP_AUTOSTART
#define DSL_TSPD_HIGH            0
#define DSL_TSPD_LOW             1
#define DSL_TSPD_LLOW            2
#define DSL_TSPD_MIN             3
/* AGV_SPEED( HIGH | LOW | LLOW | MIN | m/min ): 낱말 또는 m/min 4~255(숫자·식). 낱말은 붙여서 고르므로 사이트가 HIGH 같은 이름의
 * 매크로를 정의하면 깨진다. AGV_SPEED_MMIN 은 숫자만 받는 별칭 */
#define DSL_TSPW_HIGH            ~, DSL_TSPD_HIGH
#define DSL_TSPW_LOW             ~, DSL_TSPD_LOW
#define DSL_TSPW_LLOW            ~, DSL_TSPD_LLOW
#define DSL_TSPW_MIN             ~, DSL_TSPD_MIN
#define AGV_SPEED(x)             (TA_SPEED | DSL_ARG2(DSL_TSPW_##x, ((x) + DSL_MMIN_OK(x))))   /* 낱말은 펼치기 전에 붙인다. 첫 글자가 "(" 인 식은 못 받는다 */
#define AGV_SPEED_MMIN(v)        (TA_SPEED | ((v) + DSL_MMIN_OK(v)))      /* m/min 4~255 */
#define AGV_FORBACK(dir)         (TA_FORBACK | DIR_##dir)
/* AGV_TURN( 방향, 각도, 회전 뒤 방향, 자동 출발[, 속도[, 옵션…]] ). 옵션을 쓰려면 속도 칸을 적는다.
 * 속도 m/min 0~31, 0 이나 생략 = 자동(TNS·ETN). 지정하면 적재·공차와 상관없이 그 회전 한 번만 그 속도
 * (회전 최저 속도~TNS 한계로 묶는다). 옵션: ALIGN 주행로 정렬, TAPE 교차선을 찾은 뒤 정렬 회전(둘 중 하나까지),
 * OBS 회전 중 장애물 감지(둘째 칸 b8 = 1. 주행 중 장애물 센서 사용 OBS_ON/OBS_OFF 와 다른 것). NOWAIT 는 태그에서 컴파일 오류.
 * 둘째 칸: fb b0~3 | 자동 출발 b4~7 | 장애물 b8(b9~10 은 0) | 속도 b11~15 */
#define DSL_TOPT_K_ALIGN         1
#define DSL_TOPT_K_TAPE          2
#define DSL_TOPT_K_OBS           0
#define DSL_TOPT_O_ALIGN         0
#define DSL_TOPT_O_TAPE          0
#define DSL_TOPT_O_OBS           1
#define DSL_TKIND(k)             (((k) == 0) ? TA_TURN : (((k) == 1) ? TA_TURN_ALIGN : TA_TURN_TAPE))
/* 각도는 5 의 배수 0~315(칸은 5도 단위 6비트, v0.54.0. 예전 90 단위), 회전 뒤 방향·자동 출발은 0~15(넘치면 옆 칸으로 샌다) */
#define DSL_TURN_OK(deg, fb, au, v, k1, k2, o) \
    (0 * sizeof(char[(((v) >= 0) && ((v) < 32) && (((k1) == 0) || ((k2) == 0)) && ((o) <= 1) \
                      && ((deg) >= 0) && (((deg) % 5) == 0) && (((deg) / 5) < 64) \
                      && ((fb) >= 0) && ((fb) < 16) && ((au) >= 0) && ((au) < 16)) ? 1 : -1]))
#define DSL_TURN_W(d, deg, fb, au, v, k1, k2, o) \
    (DSL_TKIND((k1) + (k2)) | (d) | (((deg) / 5) << 2)), \
    ((fb) | ((au) << 4) | ((o) << 8) | (((uint16_t)(v) << TA_TURN_SPEED_SHIFT) + DSL_TURN_OK(deg, fb, au, v, k1, k2, o)))
#define DSL_TURN3(d, deg, fb, au)              DSL_TURN_W(d, deg, fb, au, 0, 0, 0, 0)
#define DSL_TURN4(d, deg, fb, au, v)           DSL_TURN_W(d, deg, fb, au, v, 0, 0, 0)
#define DSL_TURN5(d, deg, fb, au, v, a)        DSL_TURN_W(d, deg, fb, au, v, DSL_TOPT_K_##a, 0, DSL_TOPT_O_##a)
#define DSL_TURN6(d, deg, fb, au, v, a, b)     DSL_TURN_W(d, deg, fb, au, v, DSL_TOPT_K_##a, DSL_TOPT_K_##b, \
                                                          DSL_TOPT_O_##a + DSL_TOPT_O_##b)
/* 방향은 이 매크로에서 바로 붙인다(사이트 legacy_const.h 의 CW·CCW 매크로가 펼쳐지지 않게) */
#define AGV_TURN(dir, ...)       DSL_CAT(DSL_TURN, DSL_NARG(__VA_ARGS__))(TURN_##dir, __VA_ARGS__)
#define AGV_BRANCH(dir)          (TA_BRANCH | BRANCH_##dir)
/* SEQ_START( [정지 방식,] 시퀀스 ): 서고 시퀀스. 정지 방식은 AGV_STOP 의 괄호 안과 같다(cm·TAPE·TAPE_GD).
 * 분류 3·4 는 정지 거리 2배 안에 교차선이 없으면 오류. 이 동작만 옵션이 앞이다(사용자 결정) */
#define DSL_SEQSTART1(seq)       (TA_SEQ | SEQ_##seq)
#define DSL_SEQSTART2(opt, seq)  DSL_ARG2(DSL_SSW_##opt, DSL_SEQSTART_CM)(opt, seq)
#define DSL_SSW_TAPE             ~, DSL_SEQSTART_TAPE
#define DSL_SSW_TAPE_GD          ~, DSL_SEQSTART_TAPE_GD
#define DSL_SEQSTART_CM(cm, seq)       DSL_STOPCM(cm), (TA_SEQ | SEQ_##seq)
#define DSL_SEQSTART_TAPE(opt, seq)    (TA_TAPE_STOP | SEQ_##seq)
#define DSL_SEQSTART_TAPE_GD(opt, seq) (TA_TAPE_STOP_GD | SEQ_##seq)
#define SEQ_START(...)           DSL_CAT(DSL_SEQSTART, DSL_NARG(__VA_ARGS__))(__VA_ARGS__)
#define ACT_START(id)            (TA_ACT | ((id) + DSL_U8_OK(id)))
#define OBS_ON                   (TA_OBS | 1)
#define OBS_OFF                  (TA_OBS | 0)
#define OBS_OFF_AUTO             (TA_OBS | 2)
#define CALL(idx)                (TA_CALL | ((idx) + DSL_U8_OK(idx)))
/* 아무 동작 없음. 규칙을 맞게 해 뒤 규칙으로 넘어가지 않게 막는 용도(동작 칸 0). 사이트가 같은 토큰열로 정의해 쓰던 것을 옮겼다 */
#define NO_ACTION  0

#define DSL_INFO_STANDBY_RESET   (DSL_INFO_STANDBY | TA_ACS_STANDBY_RESET)
/* 번호 칸: 숫자 0~255, AGV_ID_MINUS(n)(n 0~255), STATION(k) ± n, TAG_NUM ± n(지금 태그 번호, n -128~127) */
#define DSL_AGV_MINUS_MARK       0x10000L
#define AGV_ID_MINUS(n)          (DSL_AGV_MINUS_MARK + (n))
#define DSL_TAG_NUM_MARK         0x40000L
#define TAG_NUM                  DSL_TAG_NUM_MARK
#define DSL_IS_AGVMINUS(n)       (((n) >= DSL_AGV_MINUS_MARK) && ((n) <= DSL_AGV_MINUS_MARK + 255L))
#define DSL_IS_TAGNUM(n)         (((n) >= DSL_TAG_NUM_MARK - 128L) && ((n) <= DSL_TAG_NUM_MARK + 127L))
#define DSL_TSEND_OK(n)          (0 * sizeof(char[((((n) >= 0) && ((n) <= 255)) || DSL_IS_AGVMINUS(n) || DSL_IS_TAGNUM(n) \
                                                   || (DSL_IS_STATION(n) && (DSL_ST_M(n) == 0) && (DSL_ST_N(n) >= -128L) && (DSL_ST_N(n) <= 127L))) ? 1 : -1]))
#define DSL_TSEND_ARG(n)         ((DSL_IS_STATION(n) ? DSL_ST_N(n) : (DSL_IS_TAGNUM(n) ? ((n) - DSL_TAG_NUM_MARK) : (long)(n))) & 0xFFL)
#define DSL_TSEND_FLAG(n)        (DSL_IS_STATION(n) ? (TA_ACS_STATION | ((uint16_t)DSL_ST_K(n) << TA_ACS_K_SHIFT)) \
                                  : (DSL_IS_TAGNUM(n) ? TA_ACS_TAG_NUM : (DSL_IS_AGVMINUS(n) ? TA_ACS_AGV_MINUS : 0)))
#define ACS_SEND(info, num)      (TA_ACS_NUM | (uint16_t)(DSL_TSEND_ARG(num) + DSL_TSEND_OK(num))), (uint16_t)(DSL_INFO_##info | DSL_TSEND_FLAG(num))
/* 출발 대기 걸기·풀기(run_wait_start·run_wait_cancel, 레거시 Flag_Wait_Start = ON·OFF) */
#define AGV_WAIT_START           (TA_WAIT | 1)
#define AGV_WAIT_CANCEL          (TA_WAIT | 0)
/* ACS( 종류, 번호 ): 번호 칸이 괄호로 시작하거나(STATION(k), AGV_ID_MINUS(n) …) 낱말이 아닌 식(숫자, TAG_NUM ± n)이면 2칸 형식(ACS_SEND),
 * 낱말 STATION0 이면 1칸 형식, 낱말 AGV_ID 이면 AGV_ID_MINUS(0) 의 2칸 형식. 낱말은 붙여서 고르므로 사이트가 STATION0·AGV_ID 같은
 * 이름의 매크로를 정의하면 깨진다. ACS( 종류, STATION(0) ) 는 2칸 형식이다(1칸 STATION0 과 같은 뜻, 칸을 하나 더 쓴다) */
#define ACS(info, x)             DSL_ARG2(DSL_TACSW_##x, DSL_TACSN)(info, x)
#define DSL_TACSW_STATION0       ~, DSL_TACSV_STATION0
#define DSL_TACSW_AGV_ID         ~, DSL_TACSV_AGV_ID
#define DSL_TACSV_STATION0(info, x)  DSL_TACS_STATION0(info)
#define DSL_TACSV_AGV_ID(info, x)    ACS_SEND(info, AGV_ID_MINUS(0))
#define DSL_TACSN(info, x)       ACS_SEND(info, x)
/* 1칸 형식은 정보 값이 인자 8비트뿐이다. 0xFF 를 넘으면(STANDBY_RESET 등) 윗비트가 동작 코드에 묻히므로 컴파일 오류 */
#define DSL_TACS_OK(v)           (0 * sizeof(char[((v) <= 0xFF) ? 1 : -1]))
#define DSL_TACS_STATION0(info)  (TA_ACS | (DSL_INFO_##info + DSL_TACS_OK(DSL_INFO_##info)))
#define STATION_RESET            TA_STATION_RESET

/* 태그 표에서 시퀀스 스텝 이름을 쓰면 읽을 수 있는 이름의 컴파일 오류로 막는다(예: 태그 동작 칸에 WAIT 를 쓰면 스텝 한 줄이 칸에 들어가
 * 조용히 틀리거나 엉뚱한 오류가 난다). 이 헤더를 포함한 파일에서는 시퀀스 표를 쓸 수 없다(사이트 태그 표 파일에는 시퀀스가 없다) */
#define DSL_NOT_IN_TAG SEQ_STEP_NOT_ALLOWED_IN_TAG_TABLE
#undef L
#define L(...) DSL_NOT_IN_TAG
#undef GOTO
#define GOTO(...) DSL_NOT_IN_TAG
#undef WHEN
#define WHEN(...) DSL_NOT_IN_TAG
#undef WHEN_STATE
#define WHEN_STATE(...) DSL_NOT_IN_TAG
#undef WHEN_IF
#define WHEN_IF(...) DSL_NOT_IN_TAG
#undef WHEN_IF_IN
#define WHEN_IF_IN(...) DSL_NOT_IN_TAG
#undef WHEN_ANY2
#define WHEN_ANY2(...) DSL_NOT_IN_TAG
#undef WHEN_ID
#define WHEN_ID(...) DSL_NOT_IN_TAG
#undef WHEN_STABLE
#define WHEN_STABLE(...) DSL_NOT_IN_TAG
#undef WHEN_ACCUM
#define WHEN_ACCUM(...) DSL_NOT_IN_TAG
#undef WHEN_OUT
#define WHEN_OUT(...) DSL_NOT_IN_TAG
#undef OTHERWISE
#define OTHERWISE(...) DSL_NOT_IN_TAG
#undef SELECT_END_TO
#define SELECT_END_TO(...) DSL_NOT_IN_TAG
#undef OUT_ON
#define OUT_ON(...) DSL_NOT_IN_TAG
#undef OUT_OFF
#define OUT_OFF(...) DSL_NOT_IN_TAG
#undef OUT_PULSE
#define OUT_PULSE(...) DSL_NOT_IN_TAG
#undef OUT_FLICKER_UNTIL_IN
#define OUT_FLICKER_UNTIL_IN(...) DSL_NOT_IN_TAG
#undef OUT_FLICKER_UNTIL_IN_TO
#define OUT_FLICKER_UNTIL_IN_TO(...) DSL_NOT_IN_TAG
#undef HANDSHAKE
#define HANDSHAKE(...) DSL_NOT_IN_TAG
#undef BELT_ON
#define BELT_ON(...) DSL_NOT_IN_TAG
#undef BELT_OFF
#define BELT_OFF(...) DSL_NOT_IN_TAG
#undef WAIT_BELTS
#define WAIT_BELTS(...) DSL_NOT_IN_TAG
#undef WAIT
#define WAIT(...) DSL_NOT_IN_TAG
#undef WAIT_TO
#define WAIT_TO(...) DSL_NOT_IN_TAG
#undef WAIT_STABLE
#define WAIT_STABLE(...) DSL_NOT_IN_TAG
#undef WAIT_STABLE_TO
#define WAIT_STABLE_TO(...) DSL_NOT_IN_TAG
#undef WAIT_ACCUM
#define WAIT_ACCUM(...) DSL_NOT_IN_TAG
#undef WAIT_ACCUM_TO
#define WAIT_ACCUM_TO(...) DSL_NOT_IN_TAG
#undef WAIT_OUT
#define WAIT_OUT(...) DSL_NOT_IN_TAG
#undef DELAY
#define DELAY(...) DSL_NOT_IN_TAG
#undef WAIT_IF
#define WAIT_IF(...) DSL_NOT_IN_TAG
#undef WAIT_IF_TO
#define WAIT_IF_TO(...) DSL_NOT_IN_TAG
#undef WAIT_IF_IN
#define WAIT_IF_IN(...) DSL_NOT_IN_TAG
#undef WAIT_IF_IN_TO
#define WAIT_IF_IN_TO(...) DSL_NOT_IN_TAG
#undef WAIT_ANY2
#define WAIT_ANY2(...) DSL_NOT_IN_TAG
#undef WAIT_ANY2_TO
#define WAIT_ANY2_TO(...) DSL_NOT_IN_TAG
#undef DELAY_PARAM
#define DELAY_PARAM(...) DSL_NOT_IN_TAG
#undef WAIT_TO_PARAM
#define WAIT_TO_PARAM(...) DSL_NOT_IN_TAG
#undef WAIT_PARAM_TO
#define WAIT_PARAM_TO(...) DSL_NOT_IN_TAG
#undef ACS_WAIT_ACK
#define ACS_WAIT_ACK(...) DSL_NOT_IN_TAG
#undef ACS_STATION
#define ACS_STATION(...) DSL_NOT_IN_TAG
/* 태그 동작 SET_STATE( 칸, 값 )(v0.63.0, 2칸): 시퀀스 SET_STATE 와 같은 사이트 상태 칸 0~15, 값 0~65535(밖은 컴파일 오류) */
#undef SET_STATE
#define SET_STATE(k, v) (TA_STATE | (uint16_t)((k) + DSL_STATE_OK(k, v, v))), (uint16_t)(v)
#undef WAIT_STATE
#define WAIT_STATE(...) DSL_NOT_IN_TAG
#undef WAIT_ACT
#define WAIT_ACT(...) DSL_NOT_IN_TAG
#undef ERROR_HOLD
#define ERROR_HOLD(...) DSL_NOT_IN_TAG
#undef END_ERROR
#define END_ERROR(...) DSL_NOT_IN_TAG
#undef END_STOP
#define END_STOP DSL_NOT_IN_TAG
#undef END_RESTART
#define END_RESTART DSL_NOT_IN_TAG

#endif /* AGV_TAG_DSL_H */
