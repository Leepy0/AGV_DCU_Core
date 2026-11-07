/* log.h - 링 버퍼 로그. 정비 때 관리 프로토콜로 회수한다 */
#ifndef AGV_LOG_H
#define AGV_LOG_H
#include "agv/types.h"

enum {
    LOG_OVERRUN = 1,     /* a = 밀린 틱 */
    LOG_ACT_NO_SLOT,     /* a = ACT ID */
    LOG_ACS_GIVE_UP,     /* a = Info, b = 번호 */
    LOG_SCI_ERROR,       /* a = 포트, b = 횟수 */
    LOG_EE_FAIL,         /* a = 주소 */
    LOG_FRAME_DROP       /* 프레임 큐가 찼다 */
};

typedef struct { uint16_t code; int16_t a, b; uint16_t seq; } log_rec_t;

void     log_event(uint16_t code, int16_t a, int16_t b);
uint16_t log_count(void);                                   /* 지금까지 쌓인 수 */
int      log_read(uint16_t back, log_rec_t *out);           /* back = 0 이 가장 최근. 없으면 0 */
#endif
