/* mgmt.h - 관리 프로토콜('M' 프레임). 설계: docs/05_게이트웨이_기능.md */
#ifndef AGV_MGMT_H
#define AGV_MGMT_H
#include "agv/types.h"
typedef enum {
    MGMT_NG_RUNNING = 1, MGMT_NG_MODE = 2, MGMT_NG_TOKEN = 3, MGMT_NG_LIMIT = 4,
    MGMT_NG_LINK = 5, MGMT_NG_SAFETY = 6, MGMT_NG_ARG = 7, MGMT_NG_OTHER = 9
} mgmt_ng_t;
void mgmt_handle(const uint16_t *frame, uint16_t len);   /* comm_poll 이 'M' 프레임마다 호출 */
void mgmt_poll(void);                                     /* 10 ms: 데드맨, 주기 상태 프레임, 강제 출력 해제 */
#endif
