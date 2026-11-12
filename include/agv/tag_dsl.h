/*
 * tag_dsl.h - 태그 표 동작 이름. seq_dsl.h 를 포함한다. 태그 표 파일에서만 포함한다.
 * 시퀀스 표와 뜻이 같은 동작은 같은 이름이다(ACS·ACS_SEND·STATION_RESET·OBS_ON·OBS_OFF·CALL·ACT_START).
 * 이 헤더 뒤로 그 이름은 시퀀스 스텝이 아니라 태그 동작 칸을 만든다. 차가 움직이는 동작은 AGV_ 로 시작한다.
 *
 *   AGV_STOP                          선다(정지 거리 SL)
 *   AGV_AUTOSTART( 초 ) / AGV_AUTOSTART_PARAM   서고 그 초(또는 AS) 뒤 다시 출발
 *   AGV_SPEED( HIGH | LOW | LLOW | MIN )        주행 속도 단계
 *   AGV_FORBACK( FWD | BACK | TOGGLE )          전후진
 *   AGV_TURN( 방향, 각도, 회전 뒤 방향, 자동 출발, 장애물 )   제자리 회전. _ALIGN 주행로 정렬, _TAPE 교차 테이프 뒤
 *   AGV_BRANCH( LEFT | RIGHT )                  분기
 *   AGV_TAPE_STOP( 시퀀스 )                     교차 테이프에서 서고 시퀀스(SEQ_ 뺀 이름)
 *   SEQ_START( 시퀀스 )                          서고 시퀀스 시작(SEQ_ 뺀 이름: LOAD, UNLOAD, LOAD_1 …)
 *   ACT_START( ACT ID )                          주행을 멈추지 않고 병렬 동작
 *   OBS_ON / OBS_OFF                             장애물 센서 사용
 *   CALL( 인덱스 )                               사용자 함수
 *   ACS_SEND( 종류, 번호 )            보고, 번호 0~255  (2칸)
 *   ACS_SEND( 종류, STATION(k) )      보고, 번호 = 스테이션 큐[k] (레거시 ACS_Station_Call_Num[k])  (2칸)
 *   ACS_SEND( 종류, AGV_ID_MINUS(n) ) 보고, 번호 = 호기 번호 - n, n 0~255  (2칸)
 *   ACS_SEND( STANDBY, 번호 )         대기 보고: 정지, 자동 재전송 늘 켬(레거시 DA_ACS_Standby( n, NONE ) + standby_auto_sending)
 *   ACS_SEND( STANDBY_RESET, 번호 )   위와 같고 먼저 스테이션 큐를 지운다(레거시 DA_ACS_Standby( n, RESET ))
 *   ACS( 종류, STATION0 )            보고, 번호 = 스테이션 큐[0]  (1칸)
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

#define AGV_STOP                 TA_STOP
#define AGV_AUTOSTART(sec)       (TA_STOP_AUTOSTART | (sec))
#define AGV_AUTOSTART_PARAM      TA_STOP_AUTOSTART
#define DSL_TSPD_HIGH            0
#define DSL_TSPD_LOW             1
#define DSL_TSPD_LLOW            2
#define DSL_TSPD_MIN             3
#define AGV_SPEED(level)         (TA_SPEED | DSL_TSPD_##level)
#define AGV_FORBACK(dir)         (TA_FORBACK | DIR_##dir)
#define AGV_TURN(dir, deg, fb, autostart, obs) \
    (TA_TURN | TURN_##dir | (((deg) / 90) << 2)), ((fb) | ((autostart) << 4) | ((obs) << 8))
#define AGV_TURN_ALIGN(dir, deg, fb, autostart, obs) \
    (TA_TURN_ALIGN | TURN_##dir | (((deg) / 90) << 2)), ((fb) | ((autostart) << 4) | ((obs) << 8))
#define AGV_TURN_TAPE(dir, deg, fb, autostart, obs) \
    (TA_TURN_TAPE | TURN_##dir | (((deg) / 90) << 2)), ((fb) | ((autostart) << 4) | ((obs) << 8))
#define AGV_BRANCH(dir)          (TA_BRANCH | BRANCH_##dir)
#define AGV_TAPE_STOP(seq)       (TA_TAPE_STOP | SEQ_##seq)
#define SEQ_START(seq)           (TA_SEQ | SEQ_##seq)
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
#define DSL_TACS_STATION0(info)  (TA_ACS | DSL_INFO_##info)
#define STATION_RESET            TA_STATION_RESET

#endif /* AGV_TAG_DSL_H */
