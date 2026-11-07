/* ident.h - 식별 시험. 튜닝 모드에서 SHIFT 로 고르고 START.
 * 교차선 X, Y 사이(L ≈ 5 m)를 혼자 오가며 잰다. 선마다 정렬 180도 회전.
 *  0회: 최저 속도로 선을 찾아 돌고, 다음 선까지 재서 L 을 구한다.
 *  1~6회: 속도 단계 (회 - 1) / 2 (최고 속도의 1/3, 2/3, 1)로 직진하며 조향·구동 응답을 잰다.
 *         짝수 회는 SL 정지한 뒤, 홀수 회는 서지 않고 감속해 최저 속도로 다음 선까지 가서 돈다.
 *         6회가 끝나면 선 위에서 멈춰 기다린다.
 * 결과는 agv_ident 에 쌓인다. CCS Expressions 에 agv_ident 를 넣고 펼쳐 읽는다.
 * 값이 -1 이면 그 응답이 창 안에 목표에 닿지 않은 것이다 */
#ifndef AGV_IDENT_H
#define AGV_IDENT_H
#include "agv/types.h"

#define IDENT_LEVELS 3
#define IDENT_PASSES 6

typedef struct {             /* 측정 한 회 */
    float   v_mpm;           /* 실제 평균 속도 m/min. 0 = 정속 측정 전에 끊김 */
    float   dev_mean_mm;     /* 가이드 편차 평균. 0 에서 멀면 치우침 */
    float   dev_rms_mm;      /* 평균을 뺀 흔들림 RMS */
    float   dev_pp_mm;       /* 최대 - 최소 */
    float   osc_ms;          /* 흔들림 주기. 0 = 주기 없음 */
    float   steer_rms_rpm;   /* 조향 출력 RMS */
    float   rpm_l, rpm_r;    /* 평균 rpm */
    float   tq_l, tq_r;      /* 평균 토크 지령 */
    float   ripple_l, ripple_r;   /* rpm 표준편차 */
    float   ramp_lag_mpm;    /* 가속 중 지령 - 실제 속도 최대 */
    float   dec_lag_mpm;     /* 감속 중 실제 - 지령 속도 최대 (홀수 회) */
    float   lat_delay_ms;    /* 옆 계단(4칸) 10% 도달 */
    float   lat_t63_ms;      /* 63% 도달 */
    float   lat_rise_ms;     /* 10 → 90% */
    float   lat_over_pct;    /* 넘침 */
    float   lat_settle_ms;   /* ±10% 안에 든 뒤 안 나간 시각 */
    float   lat_err_mm;      /* 창 끝 평균 - 목표 */
    float   run_m;           /* 선에서 측정 끝(SL 정지·감속 시작)까지 거리 */
    float   stop_mm;         /* SL 정지 명령부터 멈출 때까지 거리 (짝수 회만) */
    float   stop_ms;
    float   hold_ms;         /* 정속 측정 창. 남은 거리로 정한다 */
    float   lat_ms;          /* 옆 계단 창 */
    int16_t risk;            /* 위험 감속 횟수 */
    int16_t cut;             /* 비트 합. 1 = 거리가 모자라 측정 중에 SL 정지했다, 2 = 직선이 짧아 속도를 낮췄다 */
} ident_level_t;

typedef struct {
    int16_t state;           /* 1 진행, 2 끝, 3 실패 */
    int16_t fail;            /* 실패한 곳: 회 × 10 + 순서. 0 = 없음 */
    int16_t pass, phase;     /* 진행 중인 회(0~6)와 순서(1~9) */
    int16_t cfg_high, cfg_kp, cfg_ki, cfg_opt, cfg_ff;   /* 시작 때 파라미터 */
    float   track_mm;        /* 잰 교차선 간격 L */
    float   total_m;         /* 시험 전체 직진 거리 */
    ident_level_t lv[IDENT_PASSES];
    float   spd_delay_ms[2]; /* 1회에서 속도 계단(+3 m/min). [0] 좌, [1] 우 */
    float   spd_t63_ms[2];
    float   spd_over_pct[2];
    float   spd_down_t63_ms[2];   /* 계단 내림 63%. 제동 토크가 없어 마찰로 준다 */
    float   ff_suggest;      /* 정속 토크로 맞춘 FF(52번) 추천값. 6회가 끝나야 나온다 */
    float   tq_offset;       /* 0 rpm 으로 늘인 토크. 마찰 몫 */
    float   stop_sl_mm;      /* 마지막 정지 때 SL */
    int16_t stop_floor_rpm;
} ident_result_t;

extern ident_result_t agv_ident;

void     ident_start(void);        /* 결과를 지우고 0회 선 찾기부터 */
int      ident_run(void);          /* 10 ms. 0 진행, 1 끝, -1 실패 */
void     ident_abort(void);
uint16_t ident_pass(void);
uint16_t ident_phase(void);
int      ident_want_h(void);       /* 1 = 선 찾기 중. 'h' 로 교차선을 본다 */
#endif
