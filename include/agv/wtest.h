/* wtest.h - 바퀴 지연 측정(튜닝 t3.9 WT). 개발 시험이라 AGV_DEV_TESTS 0 이미지에는 없다.
 * 직선 테이프 위에서 START 한 번. 평소 조향으로 LOS 까지 가속하고 편차 ±WT_STEADY_DEV 단위 안 정속이 WT_STEADY_TICKS 이어지면
 * 조향 출력을 시험이 직접 준다(steer_test_rpm, 가이드 조향 끔): 왼쪽 +A·오른쪽 −A 를 T, 반대를 2T, 다시 T(짝 펄스.
 * 끝나면 방향·옆 위치가 거의 제자리). 평균 속도 지령은 그대로. 10 ms 마다 좌·우 원시 rpm(D.rpm)을 rpm[][] 에 담는다.
 * 측정이 끝나면 평소 조향으로 돌아와 편차 ±WT_BACK_DEV 단위 안에서 바로 정지(tagact_stop).
 * 측정 중 선을 잃거나 편차가 센서 반폭의 60% 를 넘으면 즉시 조향 복귀 후 정지, 결과 없음.
 * 계산(바퀴·계단마다): 계단 직전 실측 y0 → 계단 뒤 지령 c 의 10%·63% 를 처음 넘는 때 t10·t63(표본 사이 직선 보간).
 * 1차 지연 + 순수 지연 L 이면 t10 = L + 0.105τ, t63 = L + τ 이므로 τ = (t63 − t10) / 0.895, L = t10 − 0.105τ.
 * 창(T·2T·T) 안에 63% 를 못 넘은 계단은 뺀다(τ 가 T 에 가까우면 2T 계단만 남는다). 결과는 쓴 계단의 평균.
 * 결과·버퍼는 보존 영역 밖(cmd 의 gscan_ram, t3.8 결과 뒤)이다. 부팅 때 지운다 */
#ifndef AGV_WTEST_H
#define AGV_WTEST_H
#include "agv/tune.h"

#define WT_A_RPM        200     /* 바퀴 하나의 지령 계단 rpm(모터축) */
#define WT_T_TICKS      30      /* T = 0.3 s */
#define WT_PRE          5       /* 첫 계단 앞 표본(y0 평균) */
#define WT_N            (WT_PRE + 4 * WT_T_TICKS + 2)
#define WT_STEPS        3

typedef struct {
    int16_t state;              /* 0 안 함, 1 진행, 2 끝, 3 없음 */
    int16_t phase;              /* 1 가속·정속 대기, 2 측정, 3 복귀, 4 정지 */
    int16_t fail;               /* 없음 까닭: 1 측정 중 선 잃음·편차 넘침, 2 정속 못 잡음, 3 쓸 계단 없음, 4 주행이 끊김 */
    int16_t wtl, wtr;           /* FND TL·TR: 1차 지연 τ ms(쓴 계단 평균). TUNE_NO_DATA = 없음 */
    int16_t wdl, wdr;           /* FND DL·DR: 순수 지연 ms */
    int16_t n;                  /* 담은 표본 수 */
    int16_t used[2];            /* 바퀴별 쓴 계단 수 */
    int16_t tau[2][WT_STEPS];   /* 바퀴·계단별 τ ms. TUNE_NO_DATA = 창 안에 63% 못 감 */
    int16_t dead[2][WT_STEPS];  /* 바퀴·계단별 순수 지연 ms */
    int16_t cmd[2][WT_STEPS];   /* 계단 뒤 바퀴 지령 rpm */
    int16_t dev_max;            /* 측정 중 편차 절대값 최대(단위) */
    int16_t dev_end;            /* 정지를 건 때 편차(단위) */
    int16_t v_rpm;              /* 측정 때 평균 지령 rpm */
    int16_t ff;                 /* FND FF: 측정 직전 정속 0.5 s 의 FF(파라미터 FF) 추천 = 두 바퀴 토크 지령 합 ÷ 지령 rpm 합 × 1000.
                                 * tq_cmd 는 PI + 지금 FF × w_cmd 라 FF 를 켠 채 재도 (지금 FF + PI 몫) 이 된다. TUNE_NO_DATA = 없음 */
    int16_t rpm[2][WT_N];       /* 원시 rpm. [WT_PRE + 1] 이 첫 계단 시각(그 틱 지령 전 실측) */
} wt_result_t;

#if AGV_DEV_TESTS
extern wt_result_t agv_wt;

void wtest_reset(void);         /* 부팅 */
void wtest_start(void);         /* 주행 시작(flag_run) 뒤 tune 이 부른다 */
int  wtest_run(void);           /* tune_run(10 ms, motor_run 앞). 1 = 아직 진행 */
void wtest_calc(void);          /* rpm[][]·cmd[][] 로 결과 계산(호스트 시험도 쓴다) */
#endif

#endif /* AGV_WTEST_H */
