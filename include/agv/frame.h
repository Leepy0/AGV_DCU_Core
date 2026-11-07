/*
 * frame.h - ACS 프레임 코덱. 링크와 무관하다. 설계: docs/04_통신_계층.md 2절
 *
 * 한 프레임: STX, 본문, BCC, ETX. 길이는 20 또는 12 바이트.
 * C28x 에서 char 가 16비트이므로 바이트도 uint16_t 로 다룬다.
 */
#ifndef AGV_FRAME_H
#define AGV_FRAME_H

#include "agv/types.h"

#define FRAME_STX       0x02
#define FRAME_ETX       0x03
#define FRAME_LEN_LONG  20
#define FRAME_LEN_SHORT 12

typedef enum {
    FRAME_EV_NONE = 0,   /* 더 받아야 한다 */
    FRAME_EV_OK,         /* 유효 프레임. frame_rx() 로 읽는다 */
    FRAME_EV_RFID_BAD    /* BCC 만 틀린 'T' 프레임. RFID 재시도 후보로 frame_rx() 를 쓴다 */
} frame_ev_t;

typedef struct {
    uint16_t len;                /* STX~ETX 바이트 수 */
    uint16_t b[FRAME_LEN_LONG];  /* [0] STX, [1] 종류, [2] PAN, [3] MsgID, [4:5] AGV ID */
} frame_t;

typedef struct {
    uint16_t ok, len_err, bcc_err, type_err, pan_err, msgid_err, id_mismatch, field_err, aborted;
} frame_stat_t;

void            frame_reset(void);
frame_ev_t      frame_feed(uint16_t byte);
const frame_t  *frame_rx(void);                     /* 다음 frame_feed 전까지 유효 */
const frame_stat_t *frame_stat(void);

uint16_t frame_checksum(const uint16_t *p, uint16_t n);   /* p[0..n-1] 합의 하위 4비트 → ASCII */
uint16_t frame_msgid_next(void);                          /* 송신 MsgID '1'~'9','A'~'F' 순환 */

/* 20바이트 고정. pan·msgid 는 ASCII. 빈자리는 '0' */
void frame_build(uint16_t *buf, uint16_t type, uint16_t pan, uint16_t msgid,
                 uint16_t info, int16_t num, int16_t bat);
/* ACK: 'A' pan msgid ID ID c6 c7 'A' c9 '0'×8 */
void frame_build_ack(uint16_t *buf, uint16_t pan, uint16_t msgid, uint16_t c6, uint16_t c7, uint16_t c9);

#endif /* AGV_FRAME_H */
