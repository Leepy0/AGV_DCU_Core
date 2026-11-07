/* rpmtest.h - 최대 rpm 시험. 튜닝 모드에서 SHIFT 로 고르고 START.
 * 바퀴를 바닥에서 띄우고 한다. 속도 PI 를 거치지 않고 두 바퀴에 같은 DAC 를 단계별로 고정해 낸다.
 * 단계마다 3 s. 끝 1 s 의 원시 rpm(MOTOR_RPM_SPIKE 컷 전)으로 정상 rpm 을 잰다.
 * STOP·E-STOP·오류·패널 키는 바로 DAC 0 으로 끊는다.
 * 결과는 agv_rpmt 에 남는다. 단계별 rpm 이 DAC 에 비례하면 앰프가 속도 지령을 받는 것이고,
 * 낮은 DAC 에서도 같은 rpm 까지 오르면 토크 지령이다 */
#ifndef AGV_RPMTEST_H
#define AGV_RPMTEST_H
#include "agv/types.h"

#define RPMT_LEVELS 5

typedef struct {
    int16_t dac;          /* 낸 DAC 값 */
    int16_t rpm[2];       /* 끝 1 s 평균 rpm (모터축). 좌, 우 */
    int16_t ripple[2];    /* 끝 1 s 최대 - 최소 */
    int16_t t90_ms[2];    /* 앞 단계 rpm 에서 이번 rpm 까지 90% 오른 시간. -1 = 1.28 s 안에 못 닿음 */
    int16_t spikes[2];    /* 끝 1 s 에 버린 튄 값 수 */
    float   mpm;          /* 두 바퀴 평균을 m/min 으로 (22·23번 기준) */
} rpmt_level_t;

typedef struct {
    int16_t state;        /* 0 없음, 1 진행, 2 끝, 3 실패 */
    int16_t fail;         /* 원인 × 10 + 단계. 1 STOP·키, 2 E-STOP·오류, 3 바퀴가 안 돈다 */
    int16_t level;        /* 진행 단계 1~5 */
    int16_t max_rpm[2];   /* 단계 평균 중 최대. 좌, 우 */
    float   max_mpm;
    rpmt_level_t lv[RPMT_LEVELS];
} rpmt_result_t;

extern rpmt_result_t agv_rpmt;

void     rpmt_start(void);
int      rpmt_run(void);             /* 10 ms. 0 진행, 1 끝, -1 실패 */
void     rpmt_abort(int16_t reason); /* 패널 키 등. 다음 rpmt_run 이 -1 을 낸다 */
uint16_t rpmt_level(void);
#endif
