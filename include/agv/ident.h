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
    float   lat_delay_ms;    /* 옆 계단 10% 도달. 계단 = 정속 평균에서 4칸 (목표 = 평균 + 4칸) */
    float   lat_t63_ms;      /* 63% 도달 */
    float   lat_rise_ms;     /* 10 → 90% */
    float   lat_over_pct;    /* 넘침 */
    float   lat_settle_ms;   /* ±10% 안에 든 뒤 안 나간 시각 */
    float   lat_err_mm;      /* 창 끝 평균 - (정속 평균 + 4칸), mm */
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
/* ---- 바닥 최고 속도 시험 (튜닝 모드 TOP) ----
 * 식별 시험과 같은 주행로·0회(L 측정)를 쓴다. 1·2회는 방향을 바꿔 가며 선에서 출발해
 * 지령을 TOP_CMD_RPM 까지 P.acc 램프로 올린다. 조향은 그대로 켠다.
 * 지령이 실제보다 300 rpm 넘게 앞서면 지령을 실제 속도로 끌어내려 다시 올린다(조향 권한 유지).
 * 포화(토크 지령이 상한 근처이거나 지령보다 늦음) 중에 0.3 s 실제 가속도가 100 rpm/s 아래면 최고 속도로 본다.
 * 그때 지령을 실제 속도에 고정해 0.5 s 정속으로 rpm 을 잰 뒤 감속해 최저 속도로 선에 들어가 돈다.
 * 감속 지점은 남은 거리 = L - GD - 100 mm - 감속 거리. 포화 전에 닿으면 sat = 0 (직선 부족).
 * 시험 중에는 측정 rpm 스파이크 컷을 TOP_SPIKE_RPM 으로 올린다. 2회 회전 뒤 선 위에서 기다린다.
 * 실패 코드는 식별 시험과 같은 회 × 10 + 순서 */
#define TOP_PASSES     2
#define TOP_CMD_RPM    4095
#define TOP_SPIKE_RPM  6000

typedef struct {
    int16_t top_rpm[2];      /* 포화 뒤 0.5 s 평균 rpm. 좌, 우. sat = 0 이면 감속 직전 0.3 s 평균 */
    int16_t peak_rpm[2];     /* 그 회 원시 rpm 최대 (튄 값 제외) */
    int16_t tq[2];           /* 포화 뒤 평균 토크 지령 */
    int16_t sat;             /* 1 = 포화까지 감, 0 = 직선이 모자라 가속 중에 감속 */
    int16_t sat_why;         /* 1 토크 상한, 2 지령 대비 지연, 3 지령 상한 TOP_CMD_RPM 에 닿음 */
    float   reach_mm;        /* 선에서 포화까지 거리. sat = 0 이면 0 */
    float   acc_end_rpm_s;   /* 포화(또는 감속) 직전 0.3 s 실제 가속도 */
    float   top_mpm;         /* top_rpm 좌우 평균을 m/min */
    float   run_m;           /* 선에서 감속 시작까지 */
    float   dec_lag_mpm;     /* 감속 중 실제 - 지령 최대 */
} top_pass_t;

typedef struct {
    int16_t state;           /* 1 진행, 2 끝, 3 실패 */
    int16_t fail;
    int16_t pass, phase;
    int16_t cfg_acc, cfg_dcc;
    float   track_mm;
    int16_t max_rpm[2];      /* sat = 1 인 회의 top_rpm 최대. 없으면 peak 최대 */
    float   max_mpm;
    top_pass_t p[TOP_PASSES];
} top_result_t;

extern top_result_t agv_top;

void ident_start_top(void);  /* 이후 ident_run, ident_pass, ident_phase, ident_abort, ident_want_h 를 같이 쓴다 */
int  ident_top_mode(void);   /* 1 = 지금 돌고 있는(또는 마지막) 시험이 최고 속도 시험 */

/* ---- 통합 시험 (튜닝 모드 ALL) ----
 * START 한 번으로 단계 1 → 2 → 3 을 잇는다. 사람 없이 끝까지 가서 선 위에서 기다린다.
 *  단계 1: 식별 시험 0~6회 그대로 (지금 OPT). 결과는 agv_ident.
 *  단계 2: OPT 스윕. 목록의 OPT 마다 2회(왕복 한 번). OPT 는 RAM 에만 바꾸고 회 시작 전에 조향 상태를 지운다.
 *          속도는 식별 3·4회와 같은 단계(최고 속도의 2/3, 단계 1 에서 낮춘 상한 그대로).
 *          회마다 가속 → 안정 → 정속 측정 → 옆 계단 → 감속하며 선 찾기 → 회전. SL 정지와 속도 계단은 없다.
 *          창이 안 들어가면 식별 시험처럼 속도를 4/5 로 낮춰 같은 회를 다시 한다. 결과는 agv_all.sw[k].lv[0..1].
 *  단계 3: 최고 속도 시험 1·2회. 0회는 다시 하지 않고 단계 1 의 L 을 쓴다. 결과는 agv_top.
 * 끝, 실패, 끊김 어느 쪽이든 OPT 를 시험 전 값으로 되돌린다. EEPROM 은 쓰지 않는다.
 * 실패 코드 = 단계 × 1000 + 회 × 10 + 순서. 회는 단계 1·3 은 0~6, 단계 2 는 스윕 칸 × 2 + (0 또는 1) + 1 */
#define SWEEP_MAX     16
#define SWEEP_PASSES  2

typedef struct {
    int16_t       opt;               /* 이 칸의 OPT(51번) 값 */
    int16_t       done;              /* 끝낸 회 수 0~2 */
    ident_level_t lv[SWEEP_PASSES];  /* 식별 시험과 같은 칸. 속도 계단·SL 정지 칸은 0 */
} sweep_t;

typedef struct {
    int16_t state;           /* 1 진행, 2 끝, 3 실패 */
    int16_t fail;            /* 단계 × 1000 + 회 × 10 + 순서. 0 = 없음 */
    int16_t stage;           /* 1 식별, 2 OPT 스윕, 3 최고 속도. 지정 시험은 3 타력, 4 정지 정확도 */
    int16_t pass, phase;     /* 단계 2 의 회(스윕 칸 × 2 + 0·1 + 1)와 순서. 단계 1·3 은 agv_ident·agv_top 쪽을 본다 */
    int16_t sweep_n;         /* 스윕 칸 수 */
    int16_t sweep_k;         /* 지금(또는 마지막) 스윕 칸 */
    int16_t opt_saved;       /* 시험 전 OPT. 끝나면 이 값으로 돌린다 */
    float   track_mm;        /* 단계 1 의 L */
    float   total_m;         /* 시험 전체 직진 거리 */
    float   elapsed_s;       /* START 부터 끝까지 */
    sweep_t sw[SWEEP_MAX];
} all_result_t;

extern all_result_t agv_all;

/* 스윕 목록. 기본은 OPT 1·2·8·16 비트의 16 조합 {0,1,2,3,8,9,10,11,16,17,18,19,24,25,26,27}.
 * 사이트가 바꿀 때 부팅 때 한 번 부른다. n 은 SWEEP_MAX 에서 자른다. 0..31 밖의 값은 건너뛴다 */
void ident_sweep_set(const int16_t *opts, uint16_t n);
void ident_start_all(void);  /* 이후 ident_run, ident_pass, ident_phase, ident_abort, ident_want_h 를 같이 쓴다 */
int  ident_all_mode(void);   /* 1 = 지금 돌고 있는(또는 마지막) 시험이 통합 시험이나 지정 시험 */
uint16_t ident_stage(void);  /* 통합 시험 단계 1~3, 지정 시험 1~4. 통합 시험이 아니면 0 */

/* ---- 지정 시험 (튜닝 모드 TEMP) ----
 * 통합 시험을 줄인 것. 단계 1 은 0회만(선 찾기, L 측정, 회전 두 번). 식별 1~6회와 최고 속도 시험은 없다.
 * 0회 두 번째 회전이 끝나면 바로 단계 2. 통합 시험 단계 2 와 같이 선에서 출발해 TEMP 목록의 OPT 마다 2회 잰다
 * (속도 단계 2, 같은 회 순서, OPT 는 RAM 에만, 회마다 조향 상태를 지움, 창이 안 들어가면 4/5 로 다시).
 * 마지막 스윕 회전 뒤 단계 3 타력 시험. OPT 를 시험 전 값으로 돌린 뒤 COAST_PASSES 회(방향 번갈아), 선에서 출발한다.
 *  1·2회는 속도 단계 2(스윕에서 낮춘 상한 그대로), 3·4회는 최고 속도. 1 가속 → 2 안정 0.5 s(조향 켬)
 *  → 7 타력: 두 DAC 를 0 으로 고정(조향 꺼짐)하고 매 틱 원시 rpm 을 잰다. 좌우 평균이 최저 속도 이하가 되거나
 *  3 s 가 지나거나 감속 끝점(L - GD - 100 mm)에 닿거나 가이드가 크게 벗어나면 풀고, 지령을 실제 속도(최저 속도 상한)에서
 *  최저 속도로 이어 8 선 찾기 → 9 회전. 순서 7(SL 정지)은 지정 시험에 없어 타력에 쓴다.
 *  타력 시작은 늦어도 L - GD - 100 mm - (v² - 최저²) / (2 × 1000 rpm/s). 안정 창이 안 들어가면 4/5 로 다시(cut 2).
 * 결과는 agv_all 과 agv_coast. agv_ident 에는 0회 값(L 등)만 남는다(state 2, pass 0).
 * 단계 3·4 의 agv_all.pass·phase 는 그 단계 회와 순서. 마지막 타력 회전 뒤 단계 4(아래 agv_stopt).
 * 끝, 실패, 끊김 모두 OPT 를 되돌린다. STOP·E-STOP·오류·패널 키는 타력 중이면 바로 고정을 푼다. 실패 코드는 통합 시험과 같다 */
/* TEMP 목록. 기본 {0,3,19,26,0,3,19,26}. n 은 SWEEP_MAX 에서 자른다. 0..31 밖의 값은 건너뛴다 */
void ident_temp_set(const int16_t *opts, uint16_t n);
void ident_start_temp(void); /* 이후 ident_run, ident_pass, ident_phase, ident_abort, ident_want_h 를 같이 쓴다 */
int  ident_temp_mode(void);  /* 1 = 지금 돌고 있는(또는 마지막) 시험이 지정 시험 */
int  ident_coast_key(void);  /* 패널 키. 타력 중이면 고정을 풀고, 단계 4 정지 중이면 대기를 풀고 실패시켜 1 */

#define COAST_PASSES 4
#define COAST_CURVE  64            /* 20 ms 간격 평균 rpm */

typedef struct {
    int16_t v0_rpm[2];             /* 고정 직전 0.1 s 평균 rpm. 좌, 우 */
    int16_t dec_rpm_s;             /* v0 의 90% → 30% 구간 평균 감속도. 좌우 평균 rpm/s. 닿지 않으면 -1 */
    int16_t dec_hi_rpm_s;          /* 90% → 60% */
    int16_t dec_lo_rpm_s;          /* 60% → 30% */
    int16_t t_min_ms;              /* 고정부터 최저 속도까지. 못 닿고 풀었으면 -1 */
    int16_t dist_mm;               /* 고정부터 풀 때까지 거리 (엔코더) */
    int16_t n;                     /* curve 칸 수 */
    int16_t curve[COAST_CURVE];    /* 고정부터 20 ms 마다 좌우 평균 원시 rpm */
    float   v0_mpm, dec_mps2;      /* dec_rpm_s 를 m/s². 닿지 않으면 -1 */
    int16_t cut;                   /* 비트 합. 1 감속 끝점에서 풀었다, 2 속도를 낮췄다, 4 가이드가 벗어나 풀었다 */
} coast_pass_t;                    /* float 는 짝수 칸(72)에 온다 */

typedef struct {
    int16_t state;                 /* 1 진행, 2 끝, 3 실패 */
    int16_t fail;                  /* 회 × 10 + 순서 */
    int16_t pass, phase;
    coast_pass_t p[COAST_PASSES];
} coast_result_t;

extern coast_result_t agv_coast;

/* ---- 지정 시험 단계 4: 정지 정확도 ----
 * 마지막 타력 회전 뒤 STOPT_PASSES 회(방향 번갈아), 선에서 출발한다. 사용자 OPT, 조향 켬.
 * 회 1·2 는 최고 속도의 1/3, 3·4 는 2/3, 5·6 은 최고 속도(앞 단계에서 낮춘 상한 그대로).
 * 홀수 회 목표 = CID(35번, 0 이면 STOPT_CID_MM), 짝수 회 = STOPT_SHORT_MM.
 * 1 가속 → 2 안정 0.5 s → 7 정지: 그 자리에서 교차로 대기 목표(cross_pulses)를 직접 건다(ACS 를 거치지 않는다).
 *  주행 단계가 제동점까지 달리고 교차로 위치 정지(사유 5)가 선다. 조용해지면(stop_last 기록) 대기를 풀고
 *  8 최저 속도로 선 찾기 → 9 회전. 제동 토크가 없어 목표가 가까우면 지나친다. 그 거리를 잰다.
 * 트리거는 늦어도 L - GD - 100 mm - max(목표, v² / (2 × 1000 rpm/s)). 안 들어가면 4/5 로 다시(cut 2).
 * STOP·E-STOP·오류·패널 키·가이드 이탈(10칸)·정지 중 교차선은 실패 = 4000 + 회 × 10 + 순서. 30 s 안에 안 서도 실패.
 * 마지막 회전 뒤 agv_all.state = 2, stage 4 */
#define STOPT_PASSES   6
#define STOPT_SHORT_MM 200
#define STOPT_CID_MM   600         /* 35번이 0 일 때 */

typedef struct {
    int16_t v0_rpm;                /* 트리거 직전 0.1 s 좌우 평균 rpm */
    int16_t rate_limited;          /* 1 = 계획 감속도가 dcc 를 넘었다 */
    float   v0_mpm;
    float   target_mm;             /* 건 목표 */
    float   travel_mm;             /* 트리거부터 선 자리까지 (엔코더) */
    float   overshoot_mm;          /* stop_last 값. + 지나침, - 못 미침. ≈ travel - target */
    float   required_decel;        /* 계획 감속도 최대 rpm/s. dcc × 100 과 견준다 */
    float   stop_ms;               /* 트리거부터 조용해질 때까지 */
    int16_t cut;                   /* 2 = 속도를 낮췄다 */
    int16_t reason;                /* stop_last.reason. 5 여야 한다 */
} stopt_pass_t;

typedef struct {
    int16_t state;                 /* 1 진행, 2 끝, 3 실패 */
    int16_t fail;                  /* 회 × 10 + 순서 */
    int16_t pass, phase;
    stopt_pass_t p[STOPT_PASSES];
} stopt_result_t;

extern stopt_result_t agv_stopt;

/* ---- 바닥 rpm 시험 (튜닝 모드 RPM) ----
 * 0회는 식별 시험과 같다(선 찾기, L 측정, 회전 두 번). 1~5회는 rpmtest.c 가 회 안을 재고,
 * 회가 끝나면 여기서 지령을 실제 속도에서 이어 1번 최고 속도의 1/2 로 먼 선까지 간 뒤 최저 속도로 선을 찾아 돈다.
 * 5회 회전 뒤 선 위에서 기다린다. 결과는 agv_rpmt. 실패 = 회 × 10 + 단계(8 이동·선 찾기, 9 회전).
 * 개루프 중 실패는 DAC 고정을 충격 없이 풀고 보통 정지한다 */
void  ident_start_rpm(void); /* 이후 ident_run, ident_pass, ident_phase, ident_abort, ident_want_h 를 같이 쓴다 */
int   ident_rpm_mode(void);  /* 1 = 지금 돌고 있는(또는 마지막) 시험이 바닥 rpm 시험 */
int   ident_rpm_key(void);   /* 패널 키. 바닥 rpm 시험 중이면 실패시키고 1 */
float ident_decel_mm(int16_t from);       /* from rpm 에서 최저 속도까지 주행 램프 감속 거리 mm */
void  ident_speed(int16_t v, int ramp);   /* 주행 지령. ramp 0 = 바로 */

/* ---- 리셋에 남는 결과 ----
 * agv_ident·agv_top·agv_all·agv_coast·agv_rpmt·agv_stopt 는 codestart 가 지우지 않는 RAM 구역 test_keep 에 둔다.
 * test_keep_add(agv_stopt)·test_keep_rpm(agv_rpmt)은 링커가 뒤에 붙여 앞 주소를 옮기지 않는다.
 * CCS 연결로 워치독 리셋이 나도 남는다. 부팅 때 ident_keep_boot 를 한 번 부른다.
 * 표시가 틀리면(전원 투입) 모두 지우고 표시를 쓴다. 진행 중(state 1)이던 결과는 state 3, fail 이 0 이면 -1 */
extern uint16_t test_keep_mark;
void ident_keep_boot(void);
#endif
