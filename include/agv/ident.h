/* ident.h - 식별 시험. 튜닝 모드 t3.1 에서 START.
 * 교차선 X, Y 사이(L ≈ 5 m)를 혼자 오가며 잰다.
 *  0회: 최저 속도로 선을 찾아 돌고, 다음 선까지 재서 L 을 구한다(입력 L 이 있으면 LOS 로 가다 끝만 최저 속도).
 *  1~6회: 속도 단계 (회 - 1) / 2 (최고 속도의 1/3, 2/3, 1)로 직진하며 조향·구동 응답을 잰다.
 *         짝수 회는 SL 정지, 홀수 회는 최저 속도까지 감속한 뒤 선다.
 * 회 사이(모든 튜닝 시험 공통): 0회 뒤 직선 위 위치를 엔코더로 잇고 교차선마다 다시 맞춘다(X = 0, Y = L).
 *  측정이 끝나면 다음 회에 필요한 앞 거리를 셈해, 서서 뒤(온 쪽 선)가 그만큼이면 그 자리 직선 테이프 위에서 정렬
 *  180도 회전하고, 모자라면 LOS(2번)로 더 가서 돈다. 선이 필요한 회(TOP, 개루프 rpm, 직선보다 긴 회)만 선까지 LOS 로
 *  가고 센서가 선 APPROACH_MM 앞에서 최저 속도가 된다. 짝 회의 첫 회(제작·점검은 모든 회)는 앞이 넉넉하면 돌지 않고 잇는다. 끝은 돌지 않고 선다.
 *  직선 가운데서 교차선을 보거나 선이 위치 계산과 LINE_TOL_MM 넘게 어긋나면 서서 실패(순서 8).
 *  직선 위 회전은 테이프를 다시 잡아야 끝난다. 못 잡으면 258 로 서고 실패(순서 9). X–Y 테이프는 깨끗한 직선이어야 한다.
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
/* ---- 바닥 최고 속도 시험 (튜닝 모드 t3.3 TOP) ----
 * 식별 시험과 같은 주행로·0회(L 측정)를 쓴다. 1·2회는 방향을 바꿔 가며 선에서 출발해
 * 지령을 TOP_CMD_RPM 까지 P.acc 램프로 올린다. 조향은 그대로 켠다.
 * 지령이 실제보다 300 rpm 넘게 앞서면 지령을 실제 속도로 끌어내려 다시 올린다(조향 권한 유지).
 * 포화(토크 지령이 상한 근처이거나 지령보다 늦음) 중에 0.3 s 실제 가속도가 100 rpm/s 아래면 최고 속도로 본다.
 * 그때 지령을 실제 속도에 고정해 0.5 s 정속으로 rpm 을 잰 뒤 감속해 최저 속도로 선에 들어가 돈다.
 * 감속 지점은 남은 거리 = L - GD - 100 mm - 감속 거리. 포화 전에 닿으면 sat = 0 (직선 부족).
 * 시험 중에는 측정 rpm 스파이크 컷을 TOP_SPIKE_RPM 으로 올린다. 2회 뒤 돌지 않고 선 앞에서 기다린다.
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

/* ---- 통합 시험 (튜닝 모드 t3.4 ALL) ----
 * START 한 번으로 단계 1 → 2 → 3 을 잇는다. 사람 없이 끝까지 가서 기다린다.
 *  단계 1: 식별 시험 0~6회 그대로 (지금 51·39번). 결과는 agv_ident.
 *  단계 2: 조향 스윕. 목록 칸(51번 OPT, 39번 레거시 비트, 38번 위치 소스)마다 2회(왕복 한 번).
 *          세 값은 RAM 에만 바꾸고 회 시작 전에 조향 상태를 지운다.
 *          속도는 식별 3·4회와 같은 단계(최고 속도의 2/3, 단계 1 에서 낮춘 상한 그대로).
 *          회마다 가속 → 안정 → 정속 측정 → 옆 계단 → 감속하며 선 찾기 → 회전. SL 정지와 속도 계단은 없다.
 *          창이 안 들어가면 식별 시험처럼 속도를 4/5 로 낮춰 같은 회를 다시 한다. 결과는 agv_all.sw[k].lv[0..1].
 *  단계 3: 최고 속도 시험 1·2회. 0회는 다시 하지 않고 단계 1 의 L 을 쓴다. 결과는 agv_top.
 * 끝, 실패, 끊김 어느 쪽이든 51·39·38번을 시험 전 값으로 되돌린다. EEPROM 은 쓰지 않는다.
 * 실패 코드 = 단계 × 1000 + 회 × 10 + 순서. 회는 단계 1·3 은 0~6, 단계 2 는 스윕 칸 × 2 + (0 또는 1) + 1 */
#define SWEEP_MAX     16
#define SWEEP_PASSES  2

/* 스윕 칸: 51번 OPT, 39번 레거시 비트, 38번 위치 소스(GUIDE_SRC_*, SWEEP_SRC_NOW = 시작 때 값) */
typedef struct { int16_t opt, quirk, src; } sweep_cand_t;
#define SWEEP_SRC_NOW (-1)

typedef struct {
    int16_t       opt;               /* 이 칸의 OPT(51번) 값 */
    int16_t       quirk;             /* 이 칸의 레거시 비트(39번) 값 */
    int16_t       src;               /* 이 칸의 위치 소스(38번) 값. 0 'a', 2 'h' 만 */
    int16_t       done;              /* 끝낸 회 수 0~2 */
    ident_level_t lv[SWEEP_PASSES];  /* 식별 시험과 같은 칸. 속도 계단·SL 정지 칸은 0 */
} sweep_t;

typedef struct {
    int16_t state;           /* 1 진행, 2 끝, 3 실패 */
    int16_t fail;            /* 단계 × 1000 + 회 × 10 + 순서. 0 = 없음 */
    int16_t stage;           /* 1 식별, 2 조향 스윕, 3 최고 속도. 지정 시험은 3 을 건너뛰고 4 정지 정확도 */
    int16_t pass, phase;     /* 단계 2 의 회(스윕 칸 × 2 + 0·1 + 1)와 순서. 단계 1·3 은 agv_ident·agv_top 쪽을 본다 */
    int16_t sweep_n;         /* 스윕 칸 수 */
    int16_t sweep_k;         /* 지금(또는 마지막) 스윕 칸 */
    int16_t opt_saved;       /* 시험 전 51번. 끝나면 이 값으로 돌린다 */
    int16_t quirk_saved;     /* 시험 전 39번 */
    int16_t src_saved;       /* 시험 전 38번 */
    int16_t hs_mpm;          /* 고속 스윕 속도 m/min. 0 = 고속 스윕 아님 */
    float   track_mm;        /* 단계 1 의 L */
    float   total_m;         /* 시험 전체 직진 거리 */
    float   elapsed_s;       /* START 부터 끝까지 */
    float   need_mm;         /* 고속 스윕: 이 속도에 필요한 최소 L. 0 = 고속 스윕 아님 */
    sweep_t sw[SWEEP_MAX];
} all_result_t;

extern all_result_t agv_all;

/* 스윕 목록. 기본은 조향 스위치 넷(QUIRK_STEER_20MS, QUIRK_DAMP_SIGN_OLD, QUIRK_NO_WB_GAIN, OPT_KF_POSITION)의
 * 16 조합. 예전 OPT 값 0,1,2,3,8,9,10,11,16,17,18,19,24,25,26,27 의 조향과 같은 차례다. 위치 소스는 시작 때 값.
 * 사이트가 바꿀 때 부팅 때 한 번 부른다. n 은 SWEEP_MAX 에서 자른다. 51·39·38번 한계 밖의 칸은 건너뛴다 */
void ident_sweep_set(const sweep_cand_t *c, uint16_t n);
void ident_start_all(void);  /* 이후 ident_run, ident_pass, ident_phase, ident_abort, ident_want_h 를 같이 쓴다 */
int  ident_all_mode(void);   /* 1 = 지금 돌고 있는(또는 마지막) 시험이 통합 시험이나 지정 시험 */
uint16_t ident_stage(void);  /* 통합 시험 단계 1~3, 지정 시험 1·2·4(3 은 비어 있다). 통합 시험이 아니면 0 */

/* ---- 지정 시험 (튜닝 모드 t3.5 TEMP) ----
 * 통합 시험을 줄인 것. 단계 1 은 0회만(선 찾기, L 측정, 회전 두 번). 식별 1~6회와 최고 속도 시험은 없다.
 * 0회 두 번째 회전이 끝나면 바로 단계 2. 통합 시험 단계 2 와 같이 선에서 출발해 TEMP 목록 칸마다 2회 잰다
 * (속도 단계 2, 같은 회 순서, 51·39·38번은 RAM 에만, 회마다 조향 상태를 지움, 창이 안 들어가면 4/5 로 다시).
 * 마지막 스윕 회전 뒤 스윕 칸이 RAM 에 건 조향 스위치·가이드 소스를 시험 전 값으로 돌리고 바로 단계 4(아래 agv_stopt)로 간다. 단계 3 은 비워 두고 건너뛴다.
 * 결과는 agv_all 과 agv_stopt. agv_ident 에는 0회 값(L 등)만 남는다(state 2, pass 0).
 * 단계 4 의 agv_all.pass·phase 는 그 단계 회와 순서.
 * 끝, 실패, 끊김 모두 그 조향 스위치·가이드 소스를 되돌린다. 실패 코드는 통합 시험과 같다 */
/* TEMP 목록. 기본은 예전 OPT 0, 3, 19, 26 의 조향을 두 번 돈다(위치 소스는 시작 때 값). n 은 SWEEP_MAX 에서 자른다.
 * 한계 밖의 칸은 건너뛴다 */
void ident_temp_set(const sweep_cand_t *c, uint16_t n);
void ident_start_temp(void); /* 이후 ident_run, ident_pass, ident_phase, ident_abort, ident_want_h 를 같이 쓴다 */
int  ident_temp_mode(void);  /* 1 = 지금 돌고 있는(또는 마지막) 시험이 지정 시험 */
int  ident_coast_key(void);  /* 패널 키. 가감속 찾기 감속이나 정지 정확도 정지 중이면 대기를 풀고 실패시켜 1 */

/* 연속 시험 가감속 찾기(단계 4)의 진행 칸. 결과는 agv_build */
typedef struct {
    int16_t state;                 /* 1 진행, 2 끝, 3 실패 */
    int16_t fail;                  /* 회 × 10 + 순서 */
    int16_t pass, phase;
} coast_result_t;

extern coast_result_t agv_coast;

/* ---- 지정 시험 단계 4: 정지 정확도 ----
 * 마지막 스윕 회전 뒤 STOPT_PASSES 회(방향 번갈아), 선에서 출발한다. 사용자 51·39·38번, 조향 켬.
 * 회 1·2 는 최고 속도의 1/3, 3·4 는 2/3, 5·6 은 최고 속도(앞 단계에서 낮춘 상한 그대로).
 * 홀수 회 목표 = CID(35번, 0 이면 STOPT_CID_MM), 짝수 회 = STOPT_SHORT_MM.
 * 1 가속 → 2 안정 0.5 s → 7 정지: 그 자리에서 교차로 대기 목표(cross_pulses)를 직접 건다(ACS 를 거치지 않는다).
 *  주행 단계가 제동점까지 달리고 교차로 위치 정지(사유 5)가 선다. 조용해지면(stop_last 기록) 대기를 풀고
 *  8 최저 속도로 선 찾기 → 9 회전. 제동 토크가 없어 목표가 가까우면 지나친다. 그 거리를 잰다.
 * 트리거는 늦어도 L - GD - 100 mm - max(목표, v 에서 최저 속도까지 주행 램프 감속 거리). 안 들어가면 4/5 로 다시(cut 2).
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

/* ---- 바닥 rpm 시험 (튜닝 모드 t3.2 RPM) ----
 * 0회는 식별 시험과 같다(선 찾기, L 측정, 회전 두 번). 1~5회는 rpmtest.c 가 회 안을 재고,
 * 회가 끝나면 여기서 지령을 실제 속도에서 이어 다음 회 자리로 간다(위 회 사이 규칙, LOS). 개루프 1~3회는 선에서 출발한다.
 * 5회 뒤 직선 위에서 기다린다. 결과는 agv_rpmt. 실패 = 회 × 10 + 단계(8 이동·선 찾기, 9 회전).
 * 개루프 중 실패는 DAC 고정을 충격 없이 풀고 보통 정지한다 */
void  ident_start_rpm(void); /* 이후 ident_run, ident_pass, ident_phase, ident_abort, ident_want_h 를 같이 쓴다 */
int   ident_rpm_mode(void);  /* 1 = 지금 돌고 있는(또는 마지막) 시험이 바닥 rpm 시험 */
int   ident_rpm_key(void);   /* 패널 키. 바닥 rpm 시험 중이면 실패시키고 1 */
float ident_decel_mm(int16_t from);       /* from rpm 에서 최저 속도까지 주행 램프 감속 거리 mm */
float ident_ahead_mm(void);               /* 0회 뒤: 회전 중심에서 진행 방향 앞 교차선까지 mm(위치 모델) */
void  ident_track_hint(int16_t mm);       /* 입력 L(tune). 0회 둘째 직선을 LOS 로 간다. 0 = 없음 */
void  ident_speed(int16_t v, int ramp);   /* 주행 지령. ramp 0 = 바로 */

/* ---- 제작·점검 연속 시험 (튜닝 모드 t1·t2) ----
 * START 한 번으로 아래 단계 중 mask 에 든 것을 차례로 한다. 단계 사이는 선 위에서 돌고 이어 간다(0회는 한 번만).
 *  1 기구: 0회(선 찾기, 회전, L 측정, 회전). 둘째 직선 끝에서 입력 L 로 휠 지름을 고친다(BLD_DIA, ±10% 넘으면 실패 1008).
 *          둘째 회전(0회 회전, 최저 속도)으로 제자리 회전 보정비(BLD_WB, 24번은 안 바꾼다), 선 찾기 중 'a'·'h' 짝으로 가이드 부호·배율, 둘째 직선 좌우 펄스 비를 잰다.
 *          BLD_GA 면 둘째 회전 대신 선을 보자마자 서서 미끄러진 거리(ov_mm)를 재고 끝낸다.
 *  2 rpm: 바닥 rpm 시험 4회(안정 최저 rpm)만. 점검(BLD_CHECK, t2.4)은 1회(앰프 형식)도 한다. 제작의 앰프 형식은 0(모름). 28번 추천 = 올림(max(low_rpm × 1.2, 정지 바닥 350) ÷ 1 m/min 당 rpm),
 *         4 미만은 4. low_rpm 이 0(300 rpm 도 불안정)이면 추천 없음. 28번은 RAM 최저 속도를 바로 안 바꾼다(다시 켜면 적용).
 *  3 최고 속도: TOP 1·2회. 1번 추천 = min(내림(max_mpm × 0.85), 상한). 뒤 단계는 이 값(RAM)으로 달린다.
 *          제작은 1회로 추천이 상한에 닿으면(top_capped) 2회를 건너뛴다.
 *  4 가감속 찾기: 최고 속도 회마다 가속 램프로 ACC 후보 하나, 0.5 s 정속 뒤 최저 속도까지 감속 램프로 DCC 후보 하나를 본다.
 *          ACC·DCC 찾기가 둘 다 끝나면(많아야 BLD_COAST_PASSES 회) 끝. 찾을 것이 없으면 달리지 않는다.
 *          DCC 찾기는 ACC 찾기를 뒤집은 것이다(아래 ACC). 시작 = 지금 DCC, 상한 = AU(acc_cap, DCC 한계 20~60 으로 자름).
 *          판정 창은 지령이 회 속도의 80% → 30% 를 지나는 동안(저속 감속 제한 최저 속도 × 2 의 1.1배 위만).
 *          실제 감속도가 후보의 90% 이상이고 좌우 rpm 이 어긋나지 않아야 깨끗하다. 토크 지령은 0 아래로 안 가 제동 포화는 보지 않는다.
 *          후보는 그 회 감속 동안만 DCC RAM 에 넣고 끝나면 시작 값으로 돌린다. 단계 끝에 DCC 추천(min(깨끗한 가장 큰 값, 상한))을
 *          RAM 에 넣어 뒤 정지 단계가 그 값으로 간다. 회는 앞이 넉넉하면 같은 방향으로 잇는다.
 *  5 정지: 최고 속도에서 교차로 위치 정지. 제작은 목표 20 cm 부터, 다음 목표 = 올림(목표 + 지나침 - BLD_OVER_OK_MM, 10 cm),
 *          적어도 +10 cm, 많아야 60 cm. 지나침이 BLD_OVER_OK_MM 이하인 첫 목표에서 멈춘다(35번 추천).
 *          점검(BLD_CHECK)은 지금 35번(0 이면 60 cm)으로 한 번.
 *  ACC: 단계 3 부터 최고 속도·가감속 찾기·편차 회의 출발 램프를 후보 ACC(12번, RAM)로 달려 실제 가속이 따라오는지 본다(이분 찾기,
 *          첫 후보 = acc_cap). 정지 단계는 시작 때 ACC. 12번 추천 = min(깨끗한 가장 큰 값, acc_cap). 끝나면 12번 RAM 을 되돌린다.
 *  6 편차: 지금 설정(51·39·38번 그대로)으로 직진 BLD_DEV_PASSES 회. 통합 시험 단계 2 의 한 회와 같다(최고 속도의 2/3,
 *          앞 단계에서 RAM 에 넣은 1번 추천 기준). 정속 편차 RMS × 10 이 se_x10(기준값 125 아래 바이트, t2 판정)이다.
 *  후진 점검(t1, TFB 일의 자리 > 0): 편차 단계 전진 회 뒤 그 자리에서 회전 대신 방향 전환(run_dir_change, 0도 회전과 같다)을 하고
 *          같은 회를 후진으로 한 번 더 한다. 결과는 agv_all.sw[0].lv[1](전진은 lv[0] 그대로), 판정은 agv_build.bk_*.
 *          뒤가 모자라면(최저 속도 0.5 m + 편차 회 길이) 안 한다. 출발 뒤 0.5 m 는 최저 속도(SPN)로 가며 편차가 ±10칸(wide ±20칸)에
 *          닿거나 0.1 s 마다 5번 이어 커지거나(4칸 이상) 선을 잃으면 바로 서고 발산(3). 발산은 t1 실패가 아니다.
 *          끝나면(정상·발산 모두) 그 자리에서 전진으로 되돌리고 단계를 끝낸다. 끊기면 서고 나서 시작 방향으로 돌린다.
 * 바로 저장(BLD_DIA 22번, BLD_WB 회전 보정비 KEEP 54, BLD_GUIDE 'h' 부호·배율, BLD_MIN 28번)은 그 값을 잰 단계가 끝날 때 한다.
 * 1·13번은 RAM 에서만 바꾸고 끝·실패·끊김 때 시험 전 값으로 돌린다. 결과는 agv_build 와 각 단계 결과.
 * 단계 번호는 agv_all.stage, 실패 코드 = 단계 × 1000 + 회 × 10 + 순서 */
#define BLD_MECH   0x0001u
#define BLD_RPM    0x0002u
#define BLD_TOP    0x0004u
#define BLD_COAST  0x0008u
#define BLD_STOPT  0x0010u
#define BLD_STEER  0x0020u
#define BLD_DIA    0x0040u
#define BLD_WB     0x0080u   /* 제자리 회전 보정비(tune_turn_scale). 이름은 예전 휠 간격 측정 그대로 */
#define BLD_GUIDE  0x0100u
#define BLD_MIN    0x0200u
#define BLD_GA     0x0400u
#define BLD_CHECK  0x0800u
#define BLD_STAGES 6
#define BLD_COAST_PASSES 8   /* 가감속 찾기 회 상한 */
#define BLD_OVER_OK_MM 10
#define BLD_DEV_PASSES 1

typedef struct {
    int16_t state;           /* 1 진행, 2 끝, 3 실패 */
    int16_t fail;            /* 단계 × 1000 + 회 × 10 + 순서 */
    int16_t mask;
    int16_t track_in_mm;     /* 입력 L. 0 = 지름 보정 안 함 */
    int16_t cap_mpm;         /* 1번 추천 상한 */
    int16_t cfg_high, cfg_dcc, cfg_dia, cfg_wb;   /* 시작 때 1·13·22·24번 */
    int16_t dia_new;         /* 고친 22번. 0 = 안 함 */
    int16_t turn_x1000;      /* 잰 제자리 회전 보정비 × 1000(둘째 회전 펄스 ÷ 휠 간격 180도 펄스). 0 = 안 함 */
    int16_t lr_permille;     /* 둘째 직선 우/좌 펄스 비 - 1, 천분율 */
    int16_t top_rpm;         /* agv_top.max_rpm 좌우 평균. 0 = 없음 */
    int16_t over_mm;         /* 정지 단계 마지막 회 지나친 거리 */
    int16_t ov_mm;           /* BLD_GA: 선을 본 뒤 미끄러진 거리 */
    int16_t amp;             /* agv_rpmt.amp. 1회를 안 하는 제작(t1)은 0 */
    int16_t min_rec, high_rec, dcc_rec, cid_rec;   /* 28·1·13·35번 추천. -1 = 없음 */
    int16_t guide_k100;      /* 기구 단계 끝 'a' = k × 'h' 의 k × 100. TUNE_NO_DATA = 측정 부족 */
    int16_t se_x10;          /* 편차 단계 정속 편차 RMS × 10. TUNE_NO_DATA = 없음 */
    int16_t floor_rpm, bat_dv;   /* tune.c 가 끝에 채운다: 정지 바닥 rpm, 48번 */
    int16_t acc_cap;         /* ACC·DCC 상한(입력 AU). tune.c 가 시작 때 넣는다 */
    int16_t acc_max;         /* 가속을 깨끗이 따라간 가장 큰 ACC(최고 속도·가감속 찾기·편차 회의 램프로 찾음). -1 = 없음 */
    int16_t acc_rec;         /* 12번 추천 = min(acc_max, acc_cap). -1 = 없음 */
    int16_t acc_slip;        /* 1 = 어느 후보 램프에서 좌우 rpm 이 어긋났다(미끄럼) */
    int16_t dcc_max;         /* 감속을 깨끗이 따라간 가장 큰 DCC(가감속 찾기 회의 감속 램프로 찾음). -1 = 없음 */
    int16_t dcc_slip;        /* 1 = 어느 후보 감속 램프에서 좌우 rpm 이 어긋났다(미끄럼) */
    float   track_mm;        /* 0회에 잰 L (지름 보정 전) */
    int16_t bk_dev_x10;      /* 후진 점검 정속 편차 평균(칸 × 10, 진행 방향 기준). 발산이면 선 때 편차. TUNE_NO_DATA = 안 함 */
    int16_t bk_judge;        /* 후진 점검 0 안 함, 1 정상, 2 치우침 큼(전진 평균과 3칸 넘게 다름), 3 발산·선 잃음 */
} build_result_t;

extern build_result_t agv_build;

void ident_start_build(uint16_t mask, int16_t track_in_mm, int16_t cap_mpm);
int  ident_build_mode(void); /* 1 = 지금 돌고 있는(또는 마지막) 시험이 연속 시험 */

/* ---- 고속 스윕 (튜닝 모드 t3.6 HS) ----
 * 지정 시험처럼 0회(선 찾기, L 측정, 회전 두 번)만 한 뒤 바로 단계 2 조향 스윕을 하고 끝에 직선 위에서 기다린다.
 * 다른 점: 속도는 입력 mpm(1번 이하) 그대로, 후보 표(ident.c s_hs_cand: 51·39·38번)를 HS_REPS 번 차례로 돈다
 * (칸 k 와 k + HS_CANDS 가 같은 후보, 칸마다 왕복 2회). 기준은 'h' 만 쓰는 새 기본이다.
 * 51·39·38번은 RAM 에만, 끝·실패·끊김 때 되돌린다. 위치 소스는 회전을 마치고 선 위에 선 뒤 회 시작 때 바뀐다.
 * 0회가 끝나면 잰 L 이 need_mm(ident_hs_need_mm) 보다 짧은지 본다. 짧으면 달리지 않고 실패 HS_FAIL_SHORT
 * (단계 2, 회 0, 순서 0). need = GD + 100 mm + 가속 거리 + (안정 0.5 s + 정속·옆 계단 창 1 s 씩) ÷ 0.95 + 감속 거리.
 * 가속·감속 거리는 주행 램프(12·13번)를 그대로 돌려 본다. 결과 agv_all(hs_mpm, need_mm, sw[k].opt·quirk) */
#define HS_CANDS       4
#define HS_REPS        1           /* 2 로 바꾸면 후보마다 4회, 16회 */
#define HS_FAIL_SHORT  2000

void     ident_start_hs(int16_t mpm);
int      ident_hs_mode(void);           /* 1 = 지금 돌고 있는(또는 마지막) 시험이 고속 스윕 */
float    ident_hs_need_mm(int16_t mpm); /* mpm 에서 측정 창이 들어가는 최소 L mm */
int16_t  ident_hs_rms10(uint16_t c);    /* 후보 c 의 정속 편차 RMS 평균 × 10. -1 = 측정 없음 */
uint16_t ident_hs_best(void);           /* 평균이 가장 작은 후보. HS_CANDS = 없음 */

/* ---- 리셋에 남는 결과 ----
 * agv_ident·agv_top·agv_all·agv_coast·agv_rpmt·agv_stopt·agv_build 는 codestart 가 지우지 않는 RAM 구역 test_keep 에 둔다.
 * test_keep_add(agv_stopt)·test_keep_rpm(agv_rpmt)·test_keep_bld(agv_build)는 링커가 뒤에 붙여 앞 주소를 옮기지 않는다.
 * CCS 연결로 워치독 리셋이 나도 남는다. 부팅 때 ident_keep_boot 를 한 번 부른다.
 * 표시가 틀리면(전원 투입) 모두 지우고 표시를 쓴다. 진행 중(state 1)이던 결과는 state 3, fail 이 0 이면 -1 */
extern uint16_t test_keep_mark;
void ident_keep_boot(void);
#endif
