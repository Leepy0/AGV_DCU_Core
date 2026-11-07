/*
 * steer.h - 가이드 조향 PI + 칼만 변화율 항. 20 ms 마다 호출한다.
 * 현장 튜닝값이므로 상수와 수식을 바꾸지 않는다. docs/07_주행_조향_성능.md 3절
 */
#ifndef AGV_STEER_H
#define AGV_STEER_H

#include "agv/types.h"

void    steer_reset(void);
void    steer_run(int16_t tuning_type);    /* DA_Gain_Guide_Direction. tuning_type = nTuningType */

#endif /* AGV_STEER_H */
