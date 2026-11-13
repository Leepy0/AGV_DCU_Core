/*
 * tag_dsl.h - 태그 표 동작 이름. seq_dsl.h 를 포함한다. 태그 표 파일에서만 포함한다.
 * 시퀀스 표와 뜻이 같은 동작은 같은 이름이다(ACS·ACS_SEND·STATION_RESET·OBS_ON·OBS_OFF·CALL·ACT_START).
 * 이 헤더 뒤로 그 이름은 시퀀스 스텝이 아니라 태그 동작 칸을 만든다. 차가 움직이는 동작은 AGV_ 로 시작한다.
 *
 * 방식 옵션은 인자 맨 뒤에 적는다. 안 적으면 일반 동작이다. 정해 두지 않은 옵션·조합은 컴파일 오류다.
 * 태그 정지 4분류(정지 뒤 시퀀스도 같다). 한 규칙에 분류 2·3·4 가 있으면 그 정지가 이긴다(순서와 무관):
 * 같은 규칙의 AGV_STOP·SEQ_START( 시퀀스 )·AGV_AUTOSTART·ACS_SEND( STANDBY… ) 는 태그 자리 SL 정지를 걸지 않는다.
 *   AGV_STOP                          1: 태그 자리에서 SL 만큼 가서 선다
 *   AGV_STOP( cm )                    2: 태그 자리에서 cm 가서 선다. SL 과 같은 cm, 0~255, 0 = 바로. 숫자로 적는다
 *                                     (시퀀스 표 AGV_MOVE 는 mm 다. 단위가 다르다)
 *   AGV_STOP( TAPE )                  3: 교차선을 찾아 본 자리에서 바로 선다
 *   AGV_STOP( TAPE_GD )               4: 교차선을 찾아 GD 더 가서 선다(회전 중심이 교차선 위)
 *   SEQ_START( 시퀀스 )                1 + 시퀀스(SEQ_ 뺀 이름: LOAD, UNLOAD, LOAD_1 …)
 *   SEQ_START( cm, 시퀀스 )            2 + 시퀀스 (2칸)
 *   SEQ_START( TAPE, 시퀀스 )          3 + 시퀀스
 *   SEQ_START( TAPE_GD, 시퀀스 )       4 + 시퀀스. 분류 3·4 규칙의 SEQ_START( 시퀀스 ) 도 교차선 정지 뒤 시작한다
 *   AGV_AUTOSTART( 초 ) / AGV_AUTOSTART_PARAM   서고 그 초(또는 AS) 뒤 다시 출발
 *   AGV_SPEED( HIGH | LOW | LLOW | MIN )        주행 속도 단계
 *   AGV_SPEED_MMIN( m/min )                     주행 속도 m/min(4~255)
 *   AGV_FORBACK( FWD | BACK | TOGGLE )          전후진
 *   AGV_TURN( 방향, 각도, 회전 뒤 방향, 자동 출발[, 속도[, 옵션…]] )   제자리 회전. 옵션 ALIGN 주행로 정렬,
 *                                     TAPE 교차 테이프를 찾은 뒤 정렬(둘 중 하나까지), OBS 회전 중 장애물 감지
 *   AGV_BRANCH( LEFT | RIGHT )                  분기
 *   ACT_START( ACT ID )                        주행을 멈추지 않고 병렬 동작
 *   OBS_ON / OBS_OFF                             장애물 센서 사용
 *   CALL( 인덱스 )                               사용자 함수
 *   ACS_SEND( 종류, 번호 )            보고, 번호 0~255  (2칸)
 *   ACS_SEND( 종류, STATION(k) )      보고, 번호 = 스테이션 큐[k] (레거시 ACS_Station_Call_Num[k])  (2칸)
 *   ACS_SEND( 종류, AGV_ID_MINUS(n) ) 보고, 번호 = 호기 번호 - n, n 0~255  (2칸)
 *   ACS_SEND( STANDBY, 번호 )         대기 보고: 정지(분류 2·3·4 규칙에서는 그 정지), 번호 > 0 이면 자동 재전송 늘 켬(레거시 DA_ACS_Standby( n, NONE ) + 자동 재전송 켬)
 *   ACS_SEND( STANDBY_RESET, 번호 )   위와 같고 먼저 스테이션 큐를 지운다(레거시 DA_ACS_Standby( n, RESET ))
 *   ACS( 종류, STATION0 )            보고, 번호 = 스테이션 큐[0]  (1칸. STANDBY_RESET 은 컴파일 오류, ACS_SEND 로)
 *   STATION_RESET                    스테이션 큐를 지운다
 * 종류는 seq_dsl.h DSL_INFO_* 이름(LOADING UNLOADING NEXT ARRIVE FINISH PROCESSING STANDBY CALL).
 * 시퀀스 표의 ACS_SEND( STANDBY, … ) 는 보고만 한다(정지·자동 재전송 없음).
 */
#ifndef AGV_TAG_DSL_H
#define AGV_TAG_DSL_H

#include "agv/seq_dsl.h"

#undef ACS
#undef ACS_SEND
#undef STATION_RESET
#undef OBS_ON
#undef OBS_OFF
#undef CALL
#undef ACT_START

#undef AGV_SPEED
#undef AGV_SPEED_MMIN
#undef AGV_FORBACK
#undef AGV_TURN
#undef AGV_MOVE

/* AGV_STOP 은 괄호 없이 쓰면 이 상수(분류 1), 괄호를 쓰면 아래 매크로(분류 2·3·4) */
enum { AGV_STOP = TA_STOP };
/* 낱말 가리기: x 가 TAPE·TAPE_GD 면 DSL_xxx_TAPE… 가 "~, 값" 으로 펼쳐져 둘째 값이 뽑히고, 숫자면 dflt 가 뽑힌다 */
#define DSL_ARG2_(a, b, ...)     b
#define DSL_ARG2(...)            DSL_ARG2_(__VA_ARGS__, ~)
#define DSL_STOPCM_OK(cm)        (0 * sizeof(char[(((cm) >= 0) && ((cm) <= 255)) ? 1 : -1]))
#define DSL_STOPCM(cm)           (TA_STOP_CM | (uint16_t)((cm) + DSL_STOPCM_OK(cm)))
#define DSL_STOPW_TAPE           ~, TA_TAPE_STOP
#define DSL_STOPW_TAPE_GD        ~, TA_TAPE_STOP_GD
#define DSL_STOPW(x, dflt)       DSL_ARG2(DSL_STOPW_##x, dflt)
#define AGV_STOP(x)              DSL_STOPW(x, DSL_STOPCM(x))
#define AGV_AUTOSTART(sec)     (TA_STOP_AUTOSTART | (sec))
#define AGV_AUTOSTART_PARAM      TA_STOP_AUTOSTART
#define DSL_TSPD_HIGH            0
#define DSL_TSPD_LOW             1
#define DSL_TSPD_LLOW            2
#define DSL_TSPD_MIN             3
#define AGV_SPEED(level)         (TA_SPEED | DSL_TSPD_##level)
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
/* 각도는 90 의 배수(칸은 90 단위 6비트), 회전 뒤 방향·자동 출발은 0~15(넘치면 옆 칸으로 샌다) */
#define DSL_TURN_OK(deg, fb, au, v, k1, k2, o) \
    (0 * sizeof(char[(((v) >= 0) && ((v) < 32) && (((k1) == 0) || ((k2) == 0)) && ((o) <= 1) \
                      && ((deg) >= 0) && (((deg) % 90) == 0) && (((deg) / 90) < 64) \
                      && ((fb) >= 0) && ((fb) < 16) && ((au) >= 0) && ((au) < 16)) ? 1 : -1]))
#define DSL_TURN_W(d, deg, fb, au, v, k1, k2, o) \
    (DSL_TKIND((k1) + (k2)) | (d) | (((deg) / 90) << 2)), \
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
#define ACT_START(id)            (TA_ACT | (id))
#define OBS_ON                   (TA_OBS | 1)
#define OBS_OFF                  (TA_OBS | 0)
#define CALL(idx)                (TA_CALL | (idx))

#define DSL_INFO_STANDBY_RESET   (DSL_INFO_STANDBY | TA_ACS_STANDBY_RESET)
#define DSL_AGV_MINUS_MARK       0x10000L
#define AGV_ID_MINUS(n)          (DSL_AGV_MINUS_MARK + (n))
#define DSL_TSEND_OK(n)          (0 * sizeof(char[(((n) >= 0) && (((n) & 0xFFFFL) < 256)) ? 1 : -1]))
#define DSL_TSEND_FLAG(n)        (DSL_IS_STATION(n) ? TA_ACS_STATION : (((n) >= DSL_AGV_MINUS_MARK) ? TA_ACS_AGV_MINUS : 0))
#define ACS_SEND(info, num)      (TA_ACS_NUM | (uint16_t)(((num) & 0xFF) + DSL_TSEND_OK(num))), (DSL_INFO_##info | DSL_TSEND_FLAG(num))
#define ACS(info, src)           DSL_TACS_##src(info)
/* 1칸 형식은 정보 값이 인자 8비트뿐이다. 0xFF 를 넘으면(STANDBY_RESET 등) 윗비트가 동작 코드에 묻히므로 컴파일 오류 */
#define DSL_TACS_OK(v)           (0 * sizeof(char[((v) <= 0xFF) ? 1 : -1]))
#define DSL_TACS_STATION0(info)  (TA_ACS | (DSL_INFO_##info + DSL_TACS_OK(DSL_INFO_##info)))
#define STATION_RESET            TA_STATION_RESET

#endif /* AGV_TAG_DSL_H */
