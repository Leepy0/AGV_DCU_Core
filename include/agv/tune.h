/* tune.h - 회전 튜닝 모드 */
#ifndef AGV_TUNE_H
#define AGV_TUNE_H
#include "agv/types.h"

enum { TUNE_OFF = 0, TUNE_WAIT, TUNE_GO_1, TUNE_TURN, TUNE_RESTART, TUNE_GO_2, TUNE_DONE, TUNE_SAVED };

void     tune_reset(void);
void     tune_arm(void);          /* 전원 투입 때 켠다. 끝내려면 전원을 다시 켠다 */
int      tune_active(void);
uint16_t tune_state(void);
int16_t  tune_result(void);       /* 계산한 휠 간격 값 */
void     tune_run(void);          /* 10 ms */
int      tune_key(uint16_t key);  /* 처리했으면 1 */
#endif
