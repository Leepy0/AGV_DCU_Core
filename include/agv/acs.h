/*
 * acs.h - ACS 세션: 슬롯 40칸, 슬롯별 재전송, 교차로, 스테이션 큐. 설계: docs/04_통신_계층.md 3절
 *
 * P_ACS_RETRY_LEGACY 가 0 이면 ACS 를 쓰지 않는 장비다. ACS 슬롯과 응답을 만들지 않는다.
 * 태그 리더 보드로 가는 'E' 프레임 ACK 는 계속 보낸다.
 */
#ifndef AGV_ACS_H
#define AGV_ACS_H

#include "agv/types.h"
#include "agv/param.h"

#define ACS_SLOT_COUNT      40
#define ACS_STATION_MAX     24
#define ACS_MAX_TRY         99

/* 슬롯 번호 (원본 ACS_Send_* 인덱스) */
enum {
    ACS_SLOT_CALL_ACK = 0, ACS_SLOT_LOADING = 2, ACS_SLOT_UNLOADING = 3, ACS_SLOT_NEXT = 4,
    ACS_SLOT_ERR = 5, ACS_SLOT_STANDBY = 6, ACS_SLOT_STANDBY_CLEAR = 9, ACS_SLOT_START = 10,
    ACS_SLOT_ARRIVE = 11, ACS_SLOT_ERR_CLEAR = 16, ACS_SLOT_STOP = 17, ACS_SLOT_DISPLAY = 20,
    ACS_SLOT_CALL = 21, ACS_SLOT_FINISH = 22, ACS_SLOT_PROCESSING = 23, ACS_SLOT_IDLE_X = 24
};

enum { ACS_CROSS_IN = 0, ACS_CROSS_OUT = 1 };

/* OUT 보고 큐. 가장 오래된 것부터 ACK 까지 보낸다 */
#define ACS_OUT_MAX         4
#define CROSS_OUT_MAX_MM    2000L   /* 가장 오래된 OUT 태그 뒤 이만큼 ACK 없이 달리면 서서 기다린다 */
/* 1 s 넘게 미확인이면 큐를 EEPROM 에 둔다(체크섬 밖). 120~123 구역 번호, 124 머리('O'<<8 | 개수) */
#define ACS_OUT_EE_FIRST    120
#define ACS_OUT_EE_HEAD     124

typedef struct { uint16_t tx, retry, give_up; } acs_stat_t;

void     acs_reset(void);
void     acs_schedule(void);                   /* 10 ms. 호출당 프레임 최대 1개 */

/* 보고. 슬롯을 만들고 다음 acs_schedule 에서 처음 보낸다 */
void     acs_report(uint16_t info, int16_t num);        /* 'L' 'U' 'N' 'A' 'F' 'P' 'B' 'C' ... */
void     acs_report_slot(uint16_t slot, int16_t num);   /* 같은 Info 가 여럿인 슬롯용 */
void     acs_call_ack(uint16_t rx_msgid);               /* 'N' 수신 응답. 1회 송신 */
void     acs_slot_clear(uint16_t slot);
int      acs_send_ack(uint16_t rx_msgid, uint16_t c6, uint16_t c7, uint16_t c9);   /* ACS 로 즉시 1회 */
int      acs_send_tag_ack(uint16_t rx_msgid, uint16_t c6, uint16_t c7, uint16_t c9);   /* 태그 리더 보드로. ACS 사용 여부와 무관 */

/* ACS 의 'A' 프레임. 같은 Info 중 가장 오래된 슬롯 해제.
 * 'O' 는 MsgID 가 지금 보내는 OUT 의 것일 때만 그 OUT 을 지운다. MsgID '0'(구 ACS)은 가장 오래된 OUT */
void     acs_on_ack(uint16_t info, uint16_t msgid);
uint16_t acs_pending(void);                    /* 미확인 슬롯 수 */
const acs_stat_t *acs_stat(void);

uint16_t acs_pan(void);                        /* 최근 PAN 문자 */
void     acs_set_pan(uint16_t pan);            /* 수신 [2] 문자 */

/* 교차로 */
void     acs_cross_in(int16_t id);
void     acs_cross_out(int16_t id);            /* OUT 보고를 큐에 넣는다. 서지 않는다. 같은 구역이 있으면 넣지 않는다 */
int16_t  acs_cross_num(uint16_t which);        /* OUT 은 큐 맨 앞 */
uint16_t acs_cross_acked(uint16_t which);
uint16_t acs_cross_retry(uint16_t which);
uint16_t acs_cross_out_count(void);            /* 미확인 OUT 수 */
int16_t  acs_cross_out_due(void);              /* 가장 오래된 OUT 태그 뒤 CROSS_OUT_MAX_MM 를 달렸다 */
/* IN: ACS 허가 또는 초기화로 재전송과 대기를 끝낸다. OUT: 대기만 끝낸다. 보고 큐는 ACK 까지 남는다 */
void     acs_cross_release(uint16_t which);
void     acs_cross_clear(void);
/* 대기(cross_*_flag) 중에는 목표(cross_pulses)까지 가서 선다 (stop.c 교차로 위치 정지).
 * 정지가 끝나면 유지(cross_*_on)로 넘어가 풀릴 때까지 서 있는다.
 * IN 대기: 'I' 태그 뒤 P_CROSS_IN_CM, ACS 허가로 풀린다.
 * OUT 대기: 'I' 태그 때 OUT 이 남아 있으면 IN 과 같은 목표, 또는 CROSS_OUT_MAX_MM 초과 때 그 자리 제동 거리.
 * OUT 큐가 비면 풀린다 */
void     acs_cross_hold(void);                 /* 대기를 유지로 넘긴다 */
/* 부팅: EEPROM 의 OUT 큐를 읽어 재전송을 잇는다. 이 저장 계층으로 큐를 기록한다 */
void     acs_out_boot(const param_store_t *store);

/* 스테이션 큐. push 반환 1 추가, 0 중복·0번·가득 참 */
int      acs_station_push(int16_t n);
void     acs_station_reset(void);
void     acs_station_set(uint16_t idx, int16_t value, uint16_t count);
int16_t  acs_station_get(uint16_t idx);
uint16_t acs_station_count(void);

/* 10 ms. 오류 지속 시간에 따라 'X'·'E' 를 보고하고 해소되면 'C' 를 보고한다 */
void     acs_error_tick(void);

#endif /* AGV_ACS_H */
