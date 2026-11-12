/*
 * steer.h - 가이드 조향 PI + 칼만 변화율 항. 20 ms 마다 호출한다.
 * 현장 튜닝값이므로 상수와 수식을 바꾸지 않는다. docs/07_주행_조향_성능.md 3절
 */
#ifndef AGV_STEER_H
#define AGV_STEER_H

#include "agv/types.h"

void    steer_reset(void);
void    steer_run(int16_t tuning_type);    /* DA_Gain_Guide_Direction. tuning_type = nTuningType */
/* 개발 시험(t3.9 바퀴 지연 측정, AGV_DEV_TESTS)만 정의한다. 0 이 아니면 RUN_DRIVE 조향 출력 대신 왼쪽 +값·오른쪽 −값 */
extern int16_t steer_test_rpm;
/* 개발 시험(AGV_DEV_TESTS)만 정의한다. 조향 선택: 0 = 37차 조향 그대로, 1 = 각도·곡률 추정 조향(분기·예전 조향 비트 중에는 37차 쪽) */
extern int16_t steer_alg_test;
/* 개발 시험만. 반응형 곡선 속도: 0 = 지금 그대로, 1 = 주행 목표 = min(지령, steer_curve_vlim()), 위험 감속은 편차가 센서 반폭 80% 넘을 때만 */
extern int16_t speed_curve_test;
/* 개발 시험만(TST, steer.c STEER_I_FLIP_* 설명). 곡선 끝 적분 처리: 0 예전(무장 + 4단위 아래 감쇠), 1 부호 기준, 2 부호 기준 + 빠른 접근, 3 없음 */
extern int16_t steer_exit_test;
/* 개발 시험만. 1 = 속도 지령이 바뀌면 적분에 |v| 비(한 틱 0.9~1.1)를 곱한다 */
extern int16_t steer_iv_test;
/* 개발 시험만(TST, steer.c STEER_ENTRY_RISE 설명). 곡선 진입 적분 가속 배수: 0 끔, 4·8 */
extern int16_t steer_entry_test;
/* 개발 시험만. 1 = 큰 편차 보조 비례(4단위 넘는 몫의 기울기 2배, 10 ms 에 10 rpm 씩 따라간다) */
extern int16_t steer_px_test;
int16_t steer_curve_vlim(void);    /* 곡률·편차로 본 속도 상한 rpm. 주행 틱마다 한 번 부른다 */

#endif /* AGV_STEER_H */
