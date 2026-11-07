/*
 * acs.h - ACS 세션: 슬롯 40칸, 슬롯별 재전송, 교차로, 스테이션 큐. 설계: docs/04_통신_계층.md 3절
 *
 * P_ACS_RETRY_LEGACY 가 0 이면 ACS 를 쓰지 않는 장비다. 슬롯을 만들지 않고 송신하지 않는다.
 */
#ifndef AGV_ACS_H
#define AGV_ACS_H

#include "agv/types.h"

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

typedef struct { uint16_t tx, retry, give_up; } acs_stat_t;

void     acs_reset(void);
void     acs_schedule(void);                   /* 10 ms. 호출당 프레임 최대 1개 */

/* 보고. 슬롯을 만들고 다음 acs_schedule 에서 처음 보낸다 */
void     acs_report(uint16_t info, int16_t num);        /* 'L' 'U' 'N' 'A' 'F' 'P' 'B' 'C' ... */
void     acs_report_slot(uint16_t slot, int16_t num);   /* 같은 Info 가 여럿인 슬롯용 */
void     acs_call_ack(uint16_t rx_msgid);               /* 'N' 수신 응답. 1회 송신 */
void     acs_slot_clear(uint16_t slot);
int      acs_send_ack(uint16_t rx_msgid, uint16_t c6, uint16_t c7, uint16_t c9);   /* 즉시 1회 */

void     acs_on_ack(uint16_t info);            /* ACS 의 'A' 프레임. 같은 Info 중 가장 오래된 슬롯 해제 */
uint16_t acs_pending(void);                    /* 미확인 슬롯 수 */
const acs_stat_t *acs_stat(void);

uint16_t acs_pan(void);                        /* 최근 PAN 문자 */
void     acs_set_pan(uint16_t pan);            /* 수신 [2] 문자 */

/* 교차로 */
void     acs_cross_in(int16_t id);
void     acs_cross_out(int16_t id);
int16_t  acs_cross_num(uint16_t which);

/* 스테이션 큐. push 반환 1 추가, 0 중복·0번·가득 참 */
int      acs_station_push(int16_t n);
void     acs_station_reset(void);
void     acs_station_set(uint16_t idx, int16_t value, uint16_t count);
int16_t  acs_station_get(uint16_t idx);
uint16_t acs_station_count(void);

/* 10 ms. 오류 지속 시간에 따라 'X'·'E' 를 보고하고 해소되면 'C' 를 보고한다 */
void     acs_error_tick(void);

#endif /* AGV_ACS_H */
