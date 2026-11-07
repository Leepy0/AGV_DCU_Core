/* ident.h - 식별 시험. 튜닝 모드에서 SHIFT 로 고르고 START.
 * 긴 직선에서 속도 세 단계로 직진하며 조향·구동 응답을 재고, 마지막에 SL 정지를 잰다.
 * 결과는 agv_ident 에 남는다. CCS Expressions 에 agv_ident 를 넣고 펼쳐 읽는다.
 * 값이 -1 이면 그 응답이 창 안에 목표에 닿지 않은 것이다 */
#ifndef AGV_IDENT_H
#define AGV_IDENT_H
#include "agv/types.h"

#define IDENT_LEVELS 3

typedef struct {
    float   v_mpm;           /* 실제 평균 속도 m/min */
    float   dev_mean_mm;     /* 가이드 편차 평균. 0 에서 멀면 치우침 */
    float   dev_rms_mm;      /* 평균을 뺀 흔들림 RMS */
    float   dev_pp_mm;       /* 최대 - 최소 */
    float   osc_ms;          /* 흔들림 주기. 0 = 주기 없음 */
    float   steer_rms_rpm;   /* 조향 출력 RMS */
    float   rpm_l, rpm_r;    /* 평균 rpm */
    float   tq_l, tq_r;      /* 평균 토크 지령 */
    float   ripple_l, ripple_r;   /* rpm 표준편차 */
    float   ramp_lag_mpm;    /* 가속 중 지령 - 실제 속도 최대 */
    float   lat_delay_ms;    /* 옆 계단(4칸) 10% 도달 */
    float   lat_t63_ms;      /* 63% 도달 */
    float   lat_rise_ms;     /* 10 → 90% */
    float   lat_over_pct;    /* 넘침 */
    float   lat_settle_ms;   /* ±10% 안에 든 뒤 안 나간 시각 */
    float   lat_err_mm;      /* 창 끝 0.5 s 평균 - 목표 */
    int16_t risk;            /* 위험 감속 횟수 */
} ident_level_t;

typedef struct {
    int16_t state;           /* 1 진행, 2 끝, 3 실패 */
    int16_t fail;            /* 실패한 곳: 단계 × 10 + 순서. 0 = 없음 */
    int16_t cfg_high, cfg_kp, cfg_ki, cfg_opt, cfg_ff;   /* 시험 때 파라미터 */
    float   total_m;         /* 시험에 쓴 거리 */
    ident_level_t lv[IDENT_LEVELS];
    float   spd_delay_ms[2]; /* 1단계에서 속도 계단(+3 m/min). [0] 좌, [1] 우 */
    float   spd_t63_ms[2];
    float   spd_over_pct[2];
    float   spd_down_t63_ms[2];   /* 계단 내림 63%. 제동 토크가 없어 마찰로 준다 */
    float   ff_suggest;      /* 정속 토크로 맞춘 FF(52번) 추천값 */
    float   tq_offset;       /* 0 rpm 으로 늘인 토크. 마찰 몫 */
    float   stop_mm;         /* SL 정지 명령부터 멈출 때까지 거리 */
    float   stop_sl_mm;      /* 그때 SL */
    float   stop_ms;
    int16_t stop_floor_rpm;
} ident_result_t;

extern ident_result_t agv_ident;

void     ident_start(void);
int      ident_run(void);          /* 10 ms. 0 진행, 1 끝, -1 실패 */
void     ident_abort(void);
uint16_t ident_level(void);        /* 1 부터 */
uint16_t ident_phase(void);
#endif
