/* comm.h - 수신 프레임 처리. 설계: docs/04_통신_계층.md 7절 */
#ifndef AGV_COMM_H
#define AGV_COMM_H
#include "agv/types.h"

#define COMM_QUEUE_LEN    4
#define COMM_TAG_HISTORY  5
#define RFID_RETRY_TICKS  100   /* 재전송 태그를 기다리는 시간 1 s */

void     comm_reset(void);
void     comm_rx_byte(uint16_t byte);    /* SCI-A RX ISR */
void     comm_poll(void);                /* 10 ms */
uint16_t comm_rx_flash(void);            /* 수신 표시 점. 2 ms 호출마다 줄어든다 */

int16_t  comm_tag_index(uint16_t n);     /* 태그 이력. n = 0 이 최근 */
int16_t  comm_tag_num(uint16_t n);
void     comm_tag_restore(uint16_t n, int16_t index, int16_t num);
uint16_t comm_recent_index(void);
int16_t  comm_recent_num(void);
void     comm_recent_clear(void);        /* 최근 태그·중복 판정 초기화 */
#endif
