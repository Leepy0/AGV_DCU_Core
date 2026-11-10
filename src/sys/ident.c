/* ident.c - 식별 시험. 교차선 X, Y 사이를 오가며 잰다. 순서 번호(FND, 실패 코드):
 *  1 가속, 2 안정 0.5 s, 3 정속 측정, 4·5 속도 계단 올림·내림(1회만), 6 옆 계단, 7 SL 정지,
 *  8 회 사이 이동(직선 위 회전 자리까지 LOS, 또는 선까지 LOS 뒤 최저 속도), 9 정렬 180도 회전 뒤 재출발
 * 0회는 8 → 9 → 8(L 측정) → 9. 1~6회는 앞 회가 정한 자리(직선 위 또는 선)에서 출발한다. 거리는 위치 모델의 앞 거리로 셈한다.
 *  짝수 회(2, 4, 6): 1 → 7 SL 정지 → 8 → 9. 센서가 앞 선 300 mm 앞에서 서도록 SL 정지 시작점을 둔다.
 *  홀수 회(1, 3, 5): 1 → 8. 감속해 최저 속도가 되면 선다(앞 선 100 mm 앞까지).
 * 정속·옆 계단 창은 남은 거리로 정한다.
 * 옆 계단은 조향 지령(p_test)을 4칸 옮겨 폐루프 응답을 본다. 치우침은 계단 뒤에도 남으니 정속 평균에서 4칸을 잰다.
 * 속도 계단은 램프 없이 지령을 바꾼다.
 * 실패 코드 = 회 × 10 + 순서. 08 은 0회에 선을 못 찾았거나 L < 2 m (track_mm 가 0 이 아니면 후자)
 * 최고 속도 시험(TOP)은 같은 0회·선 찾기·회전을 쓰고 1·2회만 다르다: 1 가속, 3 최고 속도 정속, 8, 9.
 * 결과는 agv_top 에 쌓는다.
 * 통합 시험(ALL)은 단계마다 s_r 을 그 단계 결과로 돌린다. 단계 2 는 agv_all 칸을 쓴다.
 * 단계 2 는 식별 시험의 홀수 회와 같은 길(속도 단계 2, SL 정지·속도 계단 없음)이고 결과만 스윕 칸으로 간다.
 * 지정 시험(TEMP)은 통합 시험에 s_temp 를 켠 것이다. 0회 뒤 바로 단계 2(TEMP 목록), 단계 3 은 타력 시험이다.
 * 고속 스윕(HS)은 지정 시험에 s_hs(속도 rpm)를 켠 것이다. 0회 뒤 L 을 보고 단계 2 만 입력 속도로 하고 끝낸다.
 * 타력은 1 가속, 2 안정, 7 타력(DAC 0 고정), 8, 9. 7 은 지정 시험에 SL 정지가 없어 다시 쓴다. 결과는 agv_coast
 * 단계 4 정지 정확도는 타력과 같은 1, 2 뒤 7 교차로 위치 정지, 8, 9. 결과는 agv_stopt
 * 바닥 rpm 시험(RPM)은 같은 0회 뒤 1~5회. 회 안(순서 = 단계 1~7)은 rpmtest.c 가 하고, 끝나면 실제 속도에서 이어
 * 다음 회 자리로 간다(8). 측정 중 교차선은 위치 모델상 선 가까이면 실패가 아니라 그 회를 끊고
 * 그 선에서 GD 더 가서 돈다. 결과는 agv_rpmt
 * 연속 시험(튜닝 t1·t2)은 위 시험들을 단계로 잇는다(ident.h). 단계마다 s_r 과 s_top·s_rpm 을 그 단계로 돌린다 */
#include <math.h>
#include <string.h>
#include "agv/ident.h"
#include "agv/acs.h"
#include "agv/drive.h"
#include "agv/guide.h"
#include "agv/motor.h"
#include "agv/param.h"
#include "agv/rpmtest.h"
#include "agv/run.h"
#include "agv/steer.h"
#include "agv/stop.h"
#include "agv/tagact.h"
#include "agv/tune.h"

#define D agv_drive
#define P param_dv

#define TICK_MS          10.0f
#define SETTLE_TICKS     50
#define WIN_TICKS        200      /* 정속·옆 계단 창 최대 */
#define WIN_MIN          100      /* 이보다 짧으면 주행로가 짧은 것 */
#define SPD_TICKS        200
#define END_TICKS        50       /* 창 끝 평균 */
#define RAMP_LIMIT       2000     /* 가속이 20 s 안에 안 끝나면 실패 */
#define STOP_LIMIT       3000
#define LAT_STEP_BITS    4
#define SPD_STEP_MPM     3.0f
#define GUIDE_MM         6.89f
#define BAND             0.1f     /* 정착 판정 ±10% */
#define FIND_LIMIT_MM    20000.0f /* 0회 선 찾기 */
#define TRACK_MIN_MM     2000.0f
#define APPROACH_MM      300.0f   /* 선 찾기: 센서가 선 앞 이만큼에서 최저 속도가 된다. 그 앞은 LOS(2번) */
#define LINE_TOL_MM      300.0f   /* 위치 모델 오차 한계. 선이 이보다 일찍 보이거나 이만큼 지나도 안 보이면 실패 */
#define HINT_TOL         0.1f     /* 0회 둘째 직선: 알려진 L 의 오차 한계(지름 보정 한계와 같다) */
#define NEED_LINE        1.0e9f   /* 다음 회는 선에서 출발한다 */
#define CID_FIRST_MM     200.0f   /* 연속 시험(제작) 정지 단계 첫 목표 */
#define CID_MAX_MM       600.0f   /* 그 상한 */
#define MARGIN_MM        300.0f   /* 센서가 앞 선 앞에 남길 거리 */
#define DEC_MARGIN_MM    100.0f   /* 감속 회: 최저 속도가 된 뒤 선까지 남길 거리 */
#define DEC_LAG_TICKS    15       /* 감속 회: 실제 속도가 지령을 늦게 따라가는 몫 */
#define PLAN_SLACK       0.95f    /* 창 계획 여유. 창이 끝나기 전에 budget 에 닿지 않게 */
#define TOP_RING         30       /* 최고 속도 시험: 0.3 s 원시 rpm 고리 */
#define TOP_AVG          10       /* 가속도는 고리 앞뒤 10 틱 평균 차 / 0.2 s */
#define TOP_HOLD_TICKS   50
#define TOP_SAT_TICKS    5
#define TOP_TQ_SAT       ((int16_t)(MOTOR_TORQUE_LIMIT * 0.97f))
#define TOP_LAG_SAT      200      /* 지령 - 실제 rpm 이 이보다 크면 지연 포화 */
#define TOP_LAG_PULL     300      /* 이보다 크면 지령을 실제 속도로 끌어내린다 */
#define TOP_FLAT_RPM_S   100.0f
#define TOP_DEC_EVERY    4        /* 감속 거리는 4 틱마다 4 틱 뒤 지령 상한으로 미리 잰다. CPU 를 아낀다 */

#define SWEEP_LEVEL      1u       /* 단계 2 속도 = 식별 3·4회 단계 */
#define COAST_V0_TICKS   10       /* 타력: 고정 직전 평균 */
#define COAST_TICKS      300      /* 3 s 안에 최저 속도가 안 되면 풀고 다음으로 */
#define COAST_EST_RPM_T  10.0f    /* 타력 거리 어림 감속도 1000 rpm/s 를 틱으로 */
#define COAST_GUIDE_OFF  10       /* 조향이 꺼져 있으니 가이드가 이만큼 벗어나면 푼다(위험 감속 문턱) */
#define COAST_BRAKE_MPS2 2.0f    /* 타력 감속이 이보다 크면 굴러서 선 것이 아니라 앰프가 제동했다 */
#define COAST_CUT_BRAKE  8        /* cut 비트: 제동으로 보여 평균에서 뺐다 */
#define ACC_WIN_LO       30       /* ACC 판정 창: 지령이 목표의 30% → 80% 사이(저속 가속 제한 llow 위만) */
#define ACC_WIN_HI       80
#define ACC_FOLLOW_PCT   90       /* 창 안 실제 가속도가 지령 가속도의 이만큼 이상이어야 깨끗하다 */
#define ACC_TQ_OK        0.97f    /* 지령이 목표의 60% 아래일 때 토크 지령이 상한의 이만큼에 닿으면 깨끗하지 않다.
                                     속도 지령형 앰프는 높은 속도에서 지령 자체가 상한 가까이 가므로 낮은 쪽만 본다 */
#define ACC_TQ_PCT       60
#define ACC_SLIP_RPM     100      /* 가속 중 좌우 rpm 차가 이보다 크고 */
#define ACC_SLIP_PCT     15       /* 평균의 이만큼보다 크게 */
#define ACC_SLIP_TICKS   5        /* 이만큼 이어지면 미끄럼 */
#define KEEP_MARK        ((uint16_t)(0x7E57u ^ (uint16_t)(sizeof(agv_ident) + sizeof(agv_top) + sizeof(agv_all) \
                                                           + sizeof(agv_coast) + sizeof(agv_rpmt) + sizeof(agv_stopt) \
                                                           + sizeof(agv_build))))

enum { PH_RAMP = 1, PH_SETTLE, PH_HOLD, PH_SPD_UP, PH_SPD_DOWN, PH_LAT, PH_STOP, PH_SEEK, PH_TURN };
/* 순서 8 안의 이동: 선까지(LOS 뒤 마지막 APPROACH_MM 만 최저 속도), 직선 위 회전 자리까지 LOS, 서기 기다림 */
enum { MV_LINE = 0, MV_POS, MV_HALT };
/* 다음 회의 종류 */
enum { NX_END = 0, NX_LINE, NX_IDENT, NX_SWEEP, NX_COAST, NX_STOPT, NX_RPM };
#define PH_COAST PH_STOP          /* 지정 시험 단계 3 */
#define PH_STOPT PH_STOP          /* 지정 시험 단계 4 위치 정지 */

/* codestart 가 지우지 않는 RAM. 초기화식을 두면 .cinit 이 덮으니 두지 않는다 */
#ifdef __TMS320C2000__
#pragma DATA_SECTION(agv_ident, "test_keep")
#pragma DATA_SECTION(agv_top, "test_keep")
#pragma DATA_SECTION(agv_all, "test_keep")
#pragma DATA_SECTION(agv_coast, "test_keep")
#pragma DATA_SECTION(test_keep_mark, "test_keep")
#pragma DATA_SECTION(agv_stopt, "test_keep_add")   /* 링커가 test_keep 뒤에 둔다. 앞 주소가 그대로다 */
#pragma DATA_SECTION(agv_build, "test_keep_bld")   /* test_keep_rpm 뒤 */
#endif
ident_result_t agv_ident;
top_result_t   agv_top;
all_result_t   agv_all;
coast_result_t agv_coast;
stopt_result_t agv_stopt;
build_result_t agv_build;
uint16_t       test_keep_mark;

/* 예전 OPT 값 o(b0 10 ms, b1 감쇠 부호, b3 칼만 위치, b4 휠 간격)와 같은 조향 칸. 위치 소스는 시작 때 값 */
#define OLD_OPT(o) { (int16_t)((o) & OPT_KF_POSITION), \
                     (int16_t)((((o) & 1) ? 0 : QUIRK_STEER_20MS) | (((o) & 2) ? 0 : QUIRK_DAMP_SIGN_OLD) \
                               | (((o) & 16) ? 0 : QUIRK_NO_WB_GAIN)), SWEEP_SRC_NOW }
static const sweep_cand_t s_sweep_def[SWEEP_MAX] = {
    OLD_OPT(0), OLD_OPT(1), OLD_OPT(2), OLD_OPT(3), OLD_OPT(8), OLD_OPT(9), OLD_OPT(10), OLD_OPT(11),
    OLD_OPT(16), OLD_OPT(17), OLD_OPT(18), OLD_OPT(19), OLD_OPT(24), OLD_OPT(25), OLD_OPT(26), OLD_OPT(27)
};
static sweep_cand_t s_sweep[SWEEP_MAX];
static uint16_t s_sweep_n = 0xFFFFu;   /* 0xFFFF = 기본 목록 */
static const sweep_cand_t s_temp_def[8] = {
    OLD_OPT(0), OLD_OPT(3), OLD_OPT(19), OLD_OPT(26), OLD_OPT(0), OLD_OPT(3), OLD_OPT(19), OLD_OPT(26)
};
static sweep_cand_t s_temp_l[SWEEP_MAX];
static uint16_t s_temp_n = 0xFFFFu;
/* 고속 스윕 후보 {51번, 39번, 38번}. 줄을 바꾸면 후보가 바뀐다(HS_CANDS 줄) */
static const sweep_cand_t s_hs_cand[HS_CANDS] = {
    { 0,               0,                  GUIDE_SRC_LED_ONLY },   /* 1 새 기본, 'h' 만 (기준) */
    { 0,               0,                  GUIDE_SRC_POSITION },   /* 2 새 기본, 'a' */
    { OPT_KF_POSITION, 0,                  GUIDE_SRC_LED_ONLY },   /* 3 새 기본 + 칼만 위치, 'h' 만 */
    { 0,               QUIRK_STEER_LEGACY, GUIDE_SRC_POSITION }    /* 4 예전 OPT 0, 'a' (예전 현장 동작) */
};

/* 지금 시험의 진행 칸. 식별 시험이면 agv_ident, 최고 속도 시험이면 agv_top, 통합 시험 단계 2 는 agv_all */
static struct { int16_t *state, *fail, *pass, *phase; float *track; } s_r = {
    &agv_ident.state, &agv_ident.fail, &agv_ident.pass, &agv_ident.phase, &agv_ident.track_mm
};
static int16_t s_top, s_all, s_temp, s_rpm;
static int16_t s_hs;              /* 고속 스윕 속도 rpm. 0 = 아님 */
static uint16_t s_build;          /* 연속 시험 mask. 0 = 아님 */
static struct {
    uint32_t l0, r0, line;        /* 둘째 직선 시작 펄스, BLD_GA 선을 본 펄스 */
    int16_t  ga;                  /* 1 = BLD_GA 정지 중 */
    float    cid;                 /* 정지 단계(제작) 이번 목표 mm */
    int16_t  prof_high, prof_dcc; /* 정지 단계를 공차 프로필로 달리기 전 1·13번. 0 = 안 바꿈 */
} s_bld;

/* 직선 위 회전 중심 위치 mm. 0회에 L 을 잰 뒤 엔코더와 진행 방향으로 잇고, 교차선을 볼 때마다 다시 맞춘다.
 * X = 0, Y = len. 회전마다 dir 이 뒤집힌다 */
static struct {
    float    at, len, turn_at;    /* at: ref 펄스 때 위치. turn_at: 지금 회전이 끝나면 설 위치 */
    uint32_t ref;
    int16_t  dir;                 /* +1 X → Y, -1 Y → X */
    int16_t  ok;                  /* 1 = L 을 쟀다 */
} s_pos;
static float   s_len_seen;        /* 이번 부팅에 잰 L. 0 = 없음 */
static int16_t s_hint;            /* 입력 L(tune). 0 = 없음 */

static struct {
    int16_t ri, rn, last[2], n_tq, n_lag;
    float   dec;
} s_tp;

/* 연속 시험 ACC 찾기. 최고 속도 단계부터 최고 속도·타력·편차 회의 가속 램프를 후보 ACC 로 달려 깨끗한지 본다
 * (따로 달리지 않는다). 정지 단계는 정지 정확도를 흐리지 않게 시작 때 ACC 로 달린다.
 * lo = 깨끗하다고 아는 가장 큰 값(처음은 지금 12번), hi = 입력 상한. 첫 후보는 hi, 뒤는 가운데(이분) */
static struct {
    int16_t a0, lo, hi, c;        /* c: 지금 후보. 0 = 찾지 않는다 */
    int16_t t, t1, win, bad, slip_n;
    float   r1;
} s_acc;

static struct {                  /* 타력(정지 정확도) 한 회 */
    int16_t  last[2];
    int32_t  v0[2];
    uint32_t start;
    float    prev, tc[3];        /* tc: 90·60·30% 에 닿은 틱(보간). -1 = 아직 */
    uint16_t n;                  /* 단계 4: 트리거 때 stop_last.n */
} s_cs;

typedef struct {
    float   y0, step, peak, end_sum;
    int16_t t, t10, t63, t90, t_out, window, end;
} step_t;

static struct {
    int16_t  t, want, risk_prev, hold_n, lat_n, restart, turn_seen, legs;
    int16_t  rstop;                   /* 바닥 rpm 시험: 비뚤어져 끊은 회. 보통 정지 뒤 다음 자리로 */
    int16_t  mv, turn;                /* 순서 8 이동(MV_*). turn 1 = 서서 180도 회전, 0 = 같은 방향으로 잇거나 끝 */
    float    go_dec, need, exp;       /* need: 설 자리의 뒤 거리. exp: mv_ref 에서 선까지 회전 중심 거리, 0 = 모름 */
    uint32_t mv_ref;
    int16_t  cap[IDENT_LEVELS], retry;   /* 직선이 짧으면 그 단계 속도를 낮춰 같은 회를 다시 한다 */
    uint32_t start, begin, ticks;     /* ticks: 통합 시험 경과 */
    float    budget;
    step_t   lat, spd[2];
    union { float y[WIN_TICKS]; int16_t ring[2][TOP_RING]; } u;   /* ring: 최고 속도 시험 0.3 s 원시 rpm */
    float    steer_ss, rpm_s[2], rpm_ss[2], tq_s[2];
} s_id;

#define PASS ((uint16_t)*s_r.pass)

/* 통합 시험 단계 2 (OPT 스윕) 중. 연속 시험은 단계 6 */
static int sweep(void) { return s_all && (agv_all.stage == (s_build ? 6 : 2)); }

/* 지정 시험 단계 3 (타력)·4 (정지 정확도) 중. 가속·안정·선 찾기·회전이 같다. 연속 시험은 단계 4·5 */
static int coast(void)
{
    return s_build ? ((agv_all.stage == 4) || (agv_all.stage == 5)) : (s_temp && (agv_all.stage >= 3));
}
static int stopt(void) { return s_build ? (agv_all.stage == 5) : (s_temp && (agv_all.stage == 4)); }

/* 속도 단계 0~2. 1, 2회 → 0, 3, 4회 → 1, 5, 6회 → 2. 스윕은 1, 타력은 1, 1, 2, 2, 정지 정확도는 식별과 같다 */
static uint16_t lvl(void)
{
    if (s_build && coast()) { return IDENT_LEVELS - 1u; }   /* 연속 시험 타력·정지는 최고 속도 */
    if (stopt()) { return (PASS - 1u) / 2u; }
    if (coast()) { return (PASS + 1u) / 2u; }
    return sweep() ? SWEEP_LEVEL : (PASS - 1u) / 2u;
}

/* 이번 회의 결과 칸 */
static ident_level_t *cur_lv(void)
{
    uint16_t p = (PASS > 0u) ? PASS - 1u : 0u;
    if (sweep()) { return &agv_all.sw[p / SWEEP_PASSES].lv[p % SWEEP_PASSES]; }
    return &agv_ident.lv[p];
}

/* 속도 계단은 식별 1회만 */
static int spd_pass(void) { return (PASS == 1u) && !sweep(); }

static uint32_t travel(void) { return run_pulses(D.backward ? 1u : 0u); }
static float mm_since(uint32_t from) { return (float)(travel() - from) / P.one_length_pulse; }

static float pos_mm(void)    { return s_pos.at + (float)s_pos.dir * mm_since(s_pos.ref); }
static float ahead_mm(void)  { return (s_pos.dir > 0) ? s_pos.len - pos_mm() : pos_mm(); }   /* 앞 선까지 */
static float behind_mm(void) { return s_pos.len - ahead_mm(); }                              /* 뒤 선까지 */
float ident_ahead_mm(void)   { return ahead_mm(); }
void  ident_track_hint(int16_t mm) { s_hint = mm; }

/* SL 정지 거리 mm */
static float sl_mm(void) { return (float)(D.backward ? P.stop_len : P.fwd_stop_len) / P.one_length_pulse; }

/* 진행 방향 기준 가이드 위치(칸). 옆 계단을 넣으면 이 값이 p_test 로 간다 */
static float lateral(void) { return D.backward ? -D.kf_pos : D.kf_pos; }

static float ms(int16_t t) { return (t < 0) ? -1.0f : (float)t * TICK_MS; }

static void step_begin(step_t *s, float y0, float step, int16_t window)
{
    s->y0 = y0;
    s->step = step;
    s->peak = 0.0f;
    s->end_sum = 0.0f;
    s->t = 0;
    s->t10 = s->t63 = s->t90 = s->t_out = -1;
    s->window = window;
    s->end = (window / 2 < END_TICKS) ? (int16_t)(window / 2) : END_TICKS;
}

static void step_sample(step_t *s, float y)
{
    float r = (s->step != 0.0f) ? (y - s->y0) / s->step : 0.0f;

    if ((s->t10 < 0) && (r >= 0.1f))   { s->t10 = s->t; }
    if ((s->t63 < 0) && (r >= 0.632f)) { s->t63 = s->t; }
    if ((s->t90 < 0) && (r >= 0.9f))   { s->t90 = s->t; }
    if (r > s->peak) { s->peak = r; }
    if (fabsf(r - 1.0f) > BAND) { s->t_out = s->t; }
    if (s->t >= s->window - s->end) { s->end_sum += r; }
    s->t++;
}

static float over_pct(const step_t *s) { return (s->t90 < 0) ? -1.0f : (s->peak - 1.0f) * 100.0f; }
static float settle_ms(const step_t *s) { return (s->t90 < 0) ? -1.0f : ms((int16_t)(s->t_out + 1)); }

/* 램프로 가거나(ramp=1) 지령을 바로 바꾼다 */
static void set_speed(int16_t v, int ramp)
{
    s_id.want = v;
    if (D.speed_cmd == v) { return; }
    D.speed_cmd = v;
    if (ramp)                           { D.step = DRIVE_CHANGE_SPEED; }
    else if (D.step == DRIVE_CHANGE_SPEED) { D.step = DRIVE_CRUISE; }
}

void ident_speed(int16_t v, int ramp) { set_speed(v, ramp); }

/* 1, 2회 → 1/3, 3, 4회 → 2/3, 5, 6회 → 최고 속도. 고속 스윕은 입력 속도 */
static int16_t level_rpm(void)
{
    int16_t v = s_hs ? s_hs : (int16_t)(((int32_t)P.target_speed * (int32_t)(lvl() + 1u)) / IDENT_LEVELS);
    return (v < P.min_speed) ? P.min_speed : v;
}

/* 단계 level 의 회 속도. 짧은 직선 때문에 낮춘 상한을 넣는다 */
static int16_t rpm_at(uint16_t level)
{
    int16_t v = s_hs ? s_hs : (int16_t)(((int32_t)P.target_speed * (int32_t)(level + 1u)) / IDENT_LEVELS);
    if (v < P.min_speed) { v = P.min_speed; }
    return (v > s_id.cap[level]) ? s_id.cap[level] : v;
}

static int16_t pass_rpm(void) { return rpm_at(lvl()); }

static void acc_reset(void);
static void acc_ramp(void);
static void acc_judge(void);

static void enter(uint16_t phase)
{
    *s_r.phase = (int16_t)phase;
    s_id.t = 0;
    if (phase == PH_RAMP) { acc_reset(); }
    if (coast()) {               /* 단계 3 은 agv_all 에도 회·순서를 보인다 */
        agv_all.pass = *s_r.pass;
        agv_all.phase = *s_r.phase;
    }
}

/* 앞 선을 최저 속도로 찾는다(선 자리를 모른다) */
static void seek_begin(void)
{
    tune_cross_clear();
    s_id.mv = MV_LINE;
    s_id.exp = 0.0f;
    enter(PH_SEEK);
}

static void leave(int still);

static void start_common(void)
{
    uint16_t i;

    *s_r.state = 1;
    s_id.restart = 0;
    s_id.legs = 0;
    s_id.retry = 0;
    for (i = 0; i < IDENT_LEVELS; i++) { s_id.cap[i] = 0x7FFF; }
    s_id.risk_prev = D.risk_stop;
    s_id.begin = travel();
    s_id.rstop = 0;
    s_pos.dir = -1;                  /* 처음 본 선이 X(0) 다 */
    s_pos.at = s_pos.len = 0.0f;
    s_pos.ref = s_id.begin;
    s_pos.ok = 0;
    D.p_test = 0;
    seek_begin();
}

static void point(int16_t *state, int16_t *fail, int16_t *pass, int16_t *phase, float *track)
{
    s_r.state = state;
    s_r.fail = fail;
    s_r.pass = pass;
    s_r.phase = phase;
    s_r.track = track;
}

static void point_top(void)
{
    s_top = 1;
    point(&agv_top.state, &agv_top.fail, &agv_top.pass, &agv_top.phase, &agv_top.track_mm);
    memset(&agv_top, 0, sizeof(agv_top));
    agv_top.cfg_acc = P.acc;
    agv_top.cfg_dcc = P.dcc;
}

void ident_start(void)
{
    s_build = 0;
    s_hs = 0;
    s_top = 0;
    s_all = 0;
    s_temp = 0;
    s_rpm = 0;
    point(&agv_ident.state, &agv_ident.fail, &agv_ident.pass, &agv_ident.phase, &agv_ident.track_mm);
    memset(&agv_ident, 0, sizeof(agv_ident));
    agv_ident.cfg_high = param_get(P_SPEED_HIGH);
    agv_ident.cfg_kp = param_get(P_STEER_KP);
    agv_ident.cfg_ki = param_get(P_STEER_KI);
    agv_ident.cfg_opt = param_get(P_OPT_FLAGS);
    agv_ident.cfg_ff = param_get(P_SPEED_FF);
    start_common();
}

void ident_start_top(void)
{
    s_build = 0;
    s_hs = 0;
    s_all = 0;
    s_temp = 0;
    s_rpm = 0;
    point_top();
    start_common();
}

int ident_top_mode(void) { return s_top; }

static void rpm_point(void)
{
    s_rpm = 1;
    point(&agv_rpmt.state, &agv_rpmt.fail, &agv_rpmt.pass, &agv_rpmt.phase, &agv_rpmt.track_mm);
    memset(&agv_rpmt, 0, sizeof(agv_rpmt));
    agv_rpmt.cfg_high = param_get(P_SPEED_HIGH);
    agv_rpmt.lim_rpm = (int16_t)(((int32_t)P.target_speed * 2L) / 3L);
    agv_rpmt.steer_k = RPMT_STEER_K;
}

void ident_start_rpm(void)
{
    s_build = 0;
    s_hs = 0;
    s_top = 0;
    s_all = 0;
    s_temp = 0;
    rpm_point();
    start_common();
}

int ident_rpm_mode(void) { return s_rpm; }

static void list_set(sweep_cand_t *dst, uint16_t *dn, const sweep_cand_t *c, uint16_t n)
{
    uint16_t i;
    int16_t olo, oup, qlo, qup, slo, sup;

    param_limit(P_OPT_FLAGS, &olo, &oup);
    param_limit(P_LEGACY_QUIRKS, &qlo, &qup);
    param_limit(P_GUIDE_CURVE_DETECT, &slo, &sup);
    *dn = 0;
    for (i = 0; (i < n) && (*dn < SWEEP_MAX); i++) {
        if ((c[i].opt >= olo) && (c[i].opt <= oup) && (c[i].quirk >= qlo) && (c[i].quirk <= qup)
            && ((c[i].src == SWEEP_SRC_NOW) || ((c[i].src >= slo) && (c[i].src <= sup)))) { dst[(*dn)++] = c[i]; }
    }
}

void ident_sweep_set(const sweep_cand_t *c, uint16_t n) { list_set(s_sweep, &s_sweep_n, c, n); }
void ident_temp_set(const sweep_cand_t *c, uint16_t n)  { list_set(s_temp_l, &s_temp_n, c, n); }

/* 시험 전 51·39·38번으로 돌린다 */
static void opt_restore(void)
{
    (void)param_set_ram(P_OPT_FLAGS, agv_all.opt_saved);
    (void)param_set_ram(P_LEGACY_QUIRKS, agv_all.quirk_saved);
    (void)param_set_ram(P_GUIDE_CURVE_DETECT, agv_all.src_saved);
}

static void all_begin(const sweep_cand_t *list, uint16_t n)
{
    uint16_t k;

    memset(&agv_all, 0, sizeof(agv_all));
    agv_all.state = 1;
    agv_all.stage = 1;
    agv_all.opt_saved = param_get(P_OPT_FLAGS);
    agv_all.quirk_saved = param_get(P_LEGACY_QUIRKS);
    agv_all.src_saved = param_get(P_GUIDE_CURVE_DETECT);
    agv_all.sweep_n = (int16_t)n;
    for (k = 0; k < n; k++) {
        agv_all.sw[k].opt = list[k].opt;
        agv_all.sw[k].quirk = list[k].quirk;
        agv_all.sw[k].src = (list[k].src == SWEEP_SRC_NOW) ? agv_all.src_saved : list[k].src;
    }
    ident_start();
    s_all = 1;
    s_id.ticks = 0;
}

void ident_start_all(void)
{
    if (s_sweep_n == 0xFFFFu) { all_begin(s_sweep_def, SWEEP_MAX); }
    else                      { all_begin(s_sweep, s_sweep_n); }
}

void ident_start_temp(void)
{
    if (s_temp_n == 0xFFFFu) { all_begin(s_temp_def, 8u); }
    else                     { all_begin(s_temp_l, s_temp_n); }
    memset(&agv_coast, 0, sizeof(agv_coast));
    memset(&agv_stopt, 0, sizeof(agv_stopt));
    s_temp = 1;
}

int      ident_temp_mode(void) { return s_temp && !s_hs; }
int      ident_all_mode(void) { return s_all; }
uint16_t ident_stage(void)    { return s_all ? (uint16_t)agv_all.stage : 0u; }

/* 단계 1 끝(식별)이나 단계 2 끝(스윕). 다음 단계로 s_r 을 돌린다. 차는 선 위에 서 있다 */
static void all_next(void)
{
    s_id.retry = 0;
    if (agv_all.stage == 1) {
        agv_all.track_mm = agv_ident.track_mm;
        agv_all.stage = 2;
        point(&agv_all.state, &agv_all.fail, &agv_all.pass, &agv_all.phase, &agv_all.track_mm);
        if ((agv_all.sweep_n > 0) || s_temp) { return; }
    }
    /* 최고 속도 시험은 사용자 51·39번으로 한다. 0회 없이 단계 1 의 L 로 1회부터 */
    opt_restore();
    steer_reset();
    agv_all.stage = 3;
    point_top();
    agv_top.track_mm = agv_all.track_mm;
    agv_top.state = 1;
}

/* 지정 시험 단계 2 끝. 타력은 사용자 51·39번으로 한다. 차는 선 위에 서 있다 */
static void coast_next(void)
{
    s_id.retry = 0;
    opt_restore();
    steer_reset();
    agv_all.stage = 3;
    point(&agv_coast.state, &agv_coast.fail, &agv_coast.pass, &agv_coast.phase, &agv_all.track_mm);
    agv_coast.state = 1;
}

/* 지정 시험 단계 3 끝. 차는 선 위에 서 있다 */
static void stopt_next(void)
{
    s_id.retry = 0;
    agv_all.stage = 4;
    point(&agv_stopt.state, &agv_stopt.fail, &agv_stopt.pass, &agv_stopt.phase, &agv_all.track_mm);
    agv_stopt.state = 1;
}

static int16_t clamp_p(uint16_t idx, int32_t v)
{
    int16_t lo, up;
    param_limit(idx, &lo, &up);
    return (v < lo) ? lo : ((v > up) ? up : (int16_t)v);
}

/* 연속 시험이 RAM 에서 바꾼 1·13번을 돌린다 */
static void build_restore(int16_t state)
{
    (void)param_set_ram(P_SPEED_HIGH, agv_build.cfg_high);
    (void)param_set_ram(P_DCC, agv_build.cfg_dcc);
    if (s_acc.a0 > 0) { (void)param_set_ram(P_ACC, s_acc.a0); }   /* ACC 찾기 후보 */
    s_acc.a0 = -1;
    agv_build.state = state;
}

void ident_abort(void)
{
    int16_t code = (int16_t)(*s_r.pass * 10 + *s_r.phase);
    int all_run = s_all && (agv_all.state == 1);

    motor_dac_release();         /* 타력 고정. 아니면 아무 일 없다 */
    D.p_test = 0;
    motor_spike_set(0);
    if (*s_r.state == 1) {
        *s_r.fail = code;
        *s_r.state = 3;
    }
    if (all_run) {
        agv_all.fail = (int16_t)(agv_all.stage * 1000 + code);
        agv_all.state = 3;
        opt_restore();
        if (stopt()) { acs_cross_release(ACS_CROSS_IN); }   /* 단계 4 가 건 대기 */
    }
    if (s_build && (agv_build.state == 1)) {
        agv_build.fail = agv_all.fail;
        build_restore(3);
    }
    s_bld.ga = 0;
}

uint16_t ident_pass(void)  { return (uint16_t)*s_r.pass; }
uint16_t ident_phase(void) { return (uint16_t)*s_r.phase; }

static void keep_cut(int16_t *state, int16_t *fail)
{
    if (*state != 1) { return; }
    *state = 3;
    if (*fail == 0) { *fail = -1; }
}

void ident_keep_boot(void)
{
    s_len_seen = 0.0f;
    if (test_keep_mark != KEEP_MARK) {
        memset(&agv_ident, 0, sizeof(agv_ident));
        memset(&agv_top, 0, sizeof(agv_top));
        memset(&agv_all, 0, sizeof(agv_all));
        memset(&agv_coast, 0, sizeof(agv_coast));
        memset(&agv_rpmt, 0, sizeof(agv_rpmt));
        memset(&agv_stopt, 0, sizeof(agv_stopt));
        memset(&agv_build, 0, sizeof(agv_build));
        test_keep_mark = KEEP_MARK;
        return;
    }
    /* 시험 도중 리셋 */
    keep_cut(&agv_ident.state, &agv_ident.fail);
    keep_cut(&agv_top.state, &agv_top.fail);
    keep_cut(&agv_all.state, &agv_all.fail);
    keep_cut(&agv_coast.state, &agv_coast.fail);
    keep_cut(&agv_rpmt.state, &agv_rpmt.fail);
    keep_cut(&agv_stopt.state, &agv_stopt.fail);
    keep_cut(&agv_build.state, &agv_build.fail);
}

int ident_want_h(void)
{
    return (*s_r.state == 1) && (*s_r.phase == PH_SEEK) && D.flag_run && !D.flag_stopping;
}

static void hold_clear(void)
{
    uint16_t i;
    s_id.steer_ss = 0.0f;
    for (i = 0; i < 2u; i++) { s_id.rpm_s[i] = s_id.rpm_ss[i] = s_id.tq_s[i] = 0.0f; }
}

static void hold_sample(void)
{
    uint16_t i;
    s_id.u.y[s_id.t] = lateral();
    s_id.steer_ss += D.p_pi_out * D.p_pi_out;
    for (i = 0; i < 2u; i++) {
        float r = (float)D.rpm[i];
        s_id.rpm_s[i] += r;
        s_id.rpm_ss[i] += r * r;
        s_id.tq_s[i] += (float)D.tq_cmd[i];
    }
}

static float std_of(float s, float ss, float n)
{
    float v = ss / n - (s / n) * (s / n);
    return (v > 0.0f) ? sqrtf(v) : 0.0f;
}

/* 평균을 중심으로 히스테리시스를 둔 교차 수로 흔들림 주기를 잰다 */
static void hold_finish(ident_level_t *lv)
{
    const int16_t len = s_id.hold_n;
    const float n = (float)len;
    float sum = 0.0f, ss = 0.0f, lo, hi, mean, rms, hys;
    int16_t i, side = 0, cross = 0, first = -1, last = -1;

    lo = hi = s_id.u.y[0];
    for (i = 0; i < len; i++) {
        float y = s_id.u.y[i];
        sum += y;
        if (y < lo) { lo = y; }
        if (y > hi) { hi = y; }
    }
    mean = sum / n;
    for (i = 0; i < len; i++) { ss += (s_id.u.y[i] - mean) * (s_id.u.y[i] - mean); }
    rms = sqrtf(ss / n);
    hys = (rms * 0.5f > 0.3f) ? rms * 0.5f : 0.3f;
    for (i = 0; i < len; i++) {
        float d = s_id.u.y[i] - mean;
        int16_t now = (d > hys) ? 1 : ((d < -hys) ? -1 : side);
        if ((side != 0) && (now != side)) {
            cross++;
            if (first < 0) { first = i; }
            last = i;
        }
        side = now;
    }
    lv->dev_mean_mm = mean * GUIDE_MM;
    lv->dev_rms_mm = rms * GUIDE_MM;
    lv->dev_pp_mm = (hi - lo) * GUIDE_MM;
    lv->osc_ms = (cross >= 3) ? (2.0f * (float)(last - first) * TICK_MS / (float)(cross - 1)) : 0.0f;
    lv->steer_rms_rpm = sqrtf(s_id.steer_ss / n);
    lv->rpm_l = s_id.rpm_s[0] / n;
    lv->rpm_r = s_id.rpm_s[1] / n;
    lv->tq_l = s_id.tq_s[0] / n;
    lv->tq_r = s_id.tq_s[1] / n;
    lv->ripple_l = std_of(s_id.rpm_s[0], s_id.rpm_ss[0], n);
    lv->ripple_r = std_of(s_id.rpm_s[1], s_id.rpm_ss[1], n);
    lv->v_mpm = ((lv->rpm_l + lv->rpm_r) * 0.5f) / P.one_meter_rpm;
}

static void lat_finish(ident_level_t *lv)
{
    const step_t *s = &s_id.lat;
    lv->lat_delay_ms = ms(s->t10);
    lv->lat_t63_ms = ms(s->t63);
    lv->lat_rise_ms = ((s->t10 < 0) || (s->t90 < 0)) ? -1.0f : ms((int16_t)(s->t90 - s->t10));
    lv->lat_over_pct = over_pct(s);
    lv->lat_settle_ms = settle_ms(s);
    lv->lat_err_mm = (s->end_sum / (float)s->end - 1.0f) * s->step * GUIDE_MM;
}

/* 정속 토크 = 마찰 + 기울기 × rpm. 기울기 × 1000 이 52번 단위다. 정속 측정 전에 끊긴 회는 뺀다 */
static void fit_ff(void)
{
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f, n = 0.0f, den;
    uint16_t k;

    for (k = 0; k < IDENT_PASSES; k++) {
        const ident_level_t *lv = &agv_ident.lv[k];
        float x = (lv->rpm_l + lv->rpm_r) * 0.5f;
        float y = (lv->tq_l + lv->tq_r) * 0.5f;
        if (lv->v_mpm == 0.0f) { continue; }
        sx += x; sy += y; sxx += x * x; sxy += x * y; n += 1.0f;
    }
    den = n * sxx - sx * sx;
    if (den <= 0.0f) { return; }
    agv_ident.ff_suggest = ((n * sxy - sx * sy) / den) * 1000.0f;
    agv_ident.tq_offset = (sy - (agv_ident.ff_suggest / 1000.0f) * sx) / n;
}

/* 지금 속도에서 최저 속도까지 감속하는 거리. 주행 램프를 그대로 돌려 본다 */
static float decel_mm(int16_t from)
{
    const float k = 1000.0f / 6000.0f / P.one_meter_rpm;                  /* rpm·틱 → mm */
    int16_t v = from, a = 0, n = 0;
    float sum = (float)from * (float)DEC_LAG_TICKS;

    while ((v > P.min_speed) && (n < 3000)) {
        (void)motor_scv_core(P.min_speed, &v, P.acc, P.dcc, P.llow_speed, P.min_speed, P.min_speed, 1, &a);
        sum += (float)v;
        n++;
    }
    return sum * k;
}

float ident_decel_mm(int16_t from) { return decel_mm(from); }

static int sl_pass(void) { return ((PASS & 1u) == 0u) && !sweep(); }

/* 출발 자리의 앞 거리로 셈한다. SL 정지나 감속은 늦어도 budget 에서 시작해야 센서가 앞 선 앞에 선다.
 * 스윕은 칸이 바뀔 때 51·39번을 RAM 에 넣고, 회마다 조향 상태를 지운다 */
static void pass_begin(void)
{
    if (s_id.retry) {
        s_id.retry = 0;
        memset(cur_lv(), 0, sizeof(ident_level_t));
        cur_lv()->cut = 2;
    }
    else {
        (*s_r.pass)++;
        if (sweep() && (((PASS - 1u) % SWEEP_PASSES) == 0u)) {
            agv_all.sweep_k = (int16_t)((PASS - 1u) / SWEEP_PASSES);
            (void)param_set_ram(P_OPT_FLAGS, agv_all.sw[agv_all.sweep_k].opt);
            (void)param_set_ram(P_LEGACY_QUIRKS, agv_all.sw[agv_all.sweep_k].quirk);
            (void)param_set_ram(P_GUIDE_CURVE_DETECT, agv_all.sw[agv_all.sweep_k].src);   /* 다음 요청부터 'A'/'H' 가 바뀐다 */
        }
    }
    if (sweep()) { steer_reset(); }
    if (pass_rpm() < level_rpm()) { cur_lv()->cut |= 2; }   /* 낮춘 속도로 잰다 */
    if (sl_pass()) { s_id.budget = ahead_mm() - P.guide_dist_mm - MARGIN_MM - sl_mm(); }
    else           { s_id.budget = ahead_mm() - P.guide_dist_mm - DEC_MARGIN_MM - decel_mm(pass_rpm()); }
    D.p_test = 0;
    enter(PH_RAMP);
}

/* 남은 거리로 정속·옆 계단 창을 나눈다. 0 = 창이 너무 짧다 */
static int plan(ident_level_t *lv)
{
    const float k = 1000.0f / 6000.0f;                                       /* m/min → mm/틱 */
    float v = (float)s_id.want / P.one_meter_rpm * k;
    float extra = spd_pass() ? SPD_STEP_MPM * k * (float)SPD_TICKS : 0.0f;  /* 속도 계단 동안 더 가는 거리 */
    float rem = (s_id.budget - mm_since(s_id.begin) - extra) / v * PLAN_SLACK
              - (float)(SETTLE_TICKS + (spd_pass() ? 2 * SPD_TICKS : 0));
    float hold = (rem * 0.5f < (float)WIN_TICKS) ? rem * 0.5f : (float)WIN_TICKS;
    float lat = (rem - hold < (float)WIN_TICKS) ? rem - hold : (float)WIN_TICKS;

    if ((hold < (float)WIN_MIN) || (lat < (float)WIN_MIN)) { return 0; }
    s_id.hold_n = (int16_t)hold;
    s_id.lat_n = (int16_t)lat;
    lv->hold_ms = ms(s_id.hold_n);
    lv->lat_ms = ms(s_id.lat_n);
    return 1;
}

static void lat_begin(const ident_level_t *lv)
{
    D.p_test = LAT_STEP_BITS;
    step_begin(&s_id.lat, lv->dev_mean_mm / GUIDE_MM, (float)LAT_STEP_BITS, s_id.lat_n);   /* 목표 = 평균 + 4칸 */
    enter(PH_LAT);
}

/* 측정 끝. 짝수 회는 SL 정지, 홀수 회는 최저 속도로 감속하며 다음 자리로 간다(leave) */
static void sl_stop(void)
{
    D.p_test = 0;
    if (!sl_pass()) {
        cur_lv()->run_m = mm_since(s_id.begin) / 1000.0f;
        set_speed(P.min_speed, 1);
        leave(0);
        return;
    }
    agv_ident.stop_sl_mm = sl_mm();
    s_id.start = travel();
    tagact_stop_length();
    enter(PH_STOP);
}

/* 바닥 rpm 시험: DAC 고정을 풀고 지령을 실제 속도에서 잇는다. 적분에 낸 DAC 를 넣어 토크가 튀지 않게 한다 */
static void rpm_release(void)
{
    int16_t act = (int16_t)(((int32_t)D.rpm_avg[0] + D.rpm_avg[1]) / 2);
    uint16_t i;

    if (!motor_dac_forced()) { return; }
    motor_dac_release();
    for (i = 0; i < 2u; i++) {
        D.iref_it[i] = (float)D.tq_cmd[i] - P.speed_ff * (float)act;
        D.iref_fb[i] = D.w_pi_out[i] = D.iref_it[i];
    }
    steer_reset();
    D.vin_out = act;
    D.scv_accel = 0;
    D.speed_cmd = act;
    D.step = DRIVE_CHANGE_SPEED;
}

static int fail_now(void)
{
    if (s_rpm) { rpm_release(); }   /* 보통 정지가 실제 속도에서 내린다 */
    ident_abort();
    if (D.flag_run) { tagact_stop(); }
    D.tape_slow = 0;
    return -1;
}

/* 창이 안 들어간다. 속도를 4/5 로 낮춰 같은 회를 다시 한다. 최저 속도에서도 안 되면 실패 */
static int slow_down(void)
{
    int16_t c = (int16_t)(((int32_t)s_id.want * 4) / 5);
    if ((PASS == 0u) || (s_id.want <= P.min_speed)) { return fail_now(); }
    s_id.cap[lvl()] = (c < P.min_speed) ? P.min_speed : c;
    s_id.retry = 1;
    if (coast()) {
        set_speed(P.min_speed, 1);
        leave(0);
    }
    else { sl_stop(); }
    return 0;
}

/* ---- 최고 속도 시험 1·2회 ---- */

/* 회전 중심이 선 위에서 출발한다. 감속 거리를 뺀 남은 거리가 0 이 되면 감속한다 */
static void top_begin(void)
{
    agv_top.pass++;
    memset(&s_tp, 0, sizeof(s_tp));
    s_id.budget = ahead_mm() - P.guide_dist_mm - DEC_MARGIN_MM;
    motor_spike_set(TOP_SPIKE_RPM);
    D.p_test = 0;
    enter(PH_RAMP);
}

static int16_t ring_at(uint16_t i, int16_t k)    /* k = 0 가장 오래된 값 */
{
    int16_t j = (int16_t)(s_tp.ri + k);
    return s_id.u.ring[i][(j >= TOP_RING) ? j - TOP_RING : j];
}

/* 고리 앞뒤 10 틱 평균 차로 본 실제 가속도 rpm/s. 고리가 덜 찼으면 0 */
static float ring_acc(void)
{
    int32_t d = 0;
    int16_t k;

    if (s_tp.rn < TOP_RING) { return 0.0f; }
    for (k = 0; k < TOP_AVG; k++) {
        d += (int32_t)ring_at(0, (int16_t)(TOP_RING - TOP_AVG + k)) + ring_at(1, (int16_t)(TOP_RING - TOP_AVG + k))
           - ring_at(0, k) - ring_at(1, k);
    }
    return (float)d * (1000.0f / (2.0f * (float)TOP_AVG * (float)(TOP_RING - TOP_AVG) * TICK_MS));   /* 좌우 합 → 평균, 20 틱 → s */
}

static void top_hold_end(top_pass_t *tp)
{
    uint16_t i;
    float n = (float)s_id.t;

    if (s_id.t <= 0) { return; }     /* 정속 창 첫 틱에 감속. 포화 때 값을 남긴다 */
    for (i = 0; i < 2u; i++) {
        tp->top_rpm[i] = (int16_t)(s_id.rpm_s[i] / n);
        tp->tq[i] = (int16_t)(s_id.tq_s[i] / n);
    }
}

/* 감속해 최저 속도로 선을 찾는다. 지령이 실제보다 앞서 있으면 실제 속도에서 내린다 */
static int top_decel(top_pass_t *tp, int16_t act)
{
    uint16_t i, k;

    if (s_build) { acc_judge(); }
    if (*s_r.phase == PH_HOLD) { top_hold_end(tp); }
    else {                          /* 포화 전에 직선이 끝났다 */
        tp->sat = 0;
        tp->acc_end_rpm_s = ring_acc();
        for (i = 0; i < 2u; i++) {
            int32_t sum = 0;
            for (k = 0; k < (uint16_t)s_tp.rn; k++) { sum += s_id.u.ring[i][k]; }
            tp->top_rpm[i] = (s_tp.rn > 0) ? (int16_t)(sum / s_tp.rn) : 0;
        }
    }
    tp->top_mpm = ((float)tp->top_rpm[0] + (float)tp->top_rpm[1]) * 0.5f / P.one_meter_rpm;
    if (D.vin_out > act) { D.vin_out = act; }
    D.scv_accel = 0;                /* decel_mm 처럼 가속도 0 에서 내린다 */
    tp->run_m = mm_since(s_id.begin) / 1000.0f;
    set_speed(P.min_speed, 1);
    leave(0);
    return 0;
}

static int top_run(void)
{
    top_pass_t *tp = &agv_top.p[PASS - 1u];
    int16_t spike = motor_spike(), act, lag, why;
    uint16_t i;
    float acc;

    for (i = 0; i < 2u; i++) {      /* 튄 값은 앞 값으로 둔다 */
        int16_t r = motor_raw_rpm(i);
        if (r <= spike) { s_tp.last[i] = r; }
        if (s_tp.last[i] > tp->peak_rpm[i]) { tp->peak_rpm[i] = s_tp.last[i]; }
        s_id.u.ring[i][s_tp.ri] = s_tp.last[i];
    }
    s_tp.ri = (int16_t)((s_tp.ri + 1 >= TOP_RING) ? 0 : s_tp.ri + 1);
    if (s_tp.rn < TOP_RING) { s_tp.rn++; }
    act = (int16_t)((s_tp.last[0] + s_tp.last[1]) / 2);

    if ((s_id.t & (TOP_DEC_EVERY - 1)) == 0) {
        s_tp.dec = decel_mm((int16_t)(D.vin_out + ((*s_r.phase == PH_RAMP) ? TOP_DEC_EVERY * P.acc : 0)));
    }
    if (mm_since(s_id.begin) + s_tp.dec >= s_id.budget) { return top_decel(tp, act); }

    if (*s_r.phase == PH_HOLD) {
        for (i = 0; i < 2u; i++) {
            s_id.rpm_s[i] += (float)s_tp.last[i];
            s_id.tq_s[i] += (float)D.tq_cmd[i];
        }
        if (++s_id.t >= TOP_HOLD_TICKS) { return top_decel(tp, act); }
        return 0;
    }

    /* PH_RAMP. 지령이 너무 앞서면 실제 속도로 끌어내려 조향 몫을 남긴다 */
    set_speed(TOP_CMD_RPM, 1);
    if (s_build) { acc_ramp(); }
    lag = (int16_t)(D.vin_out - act);
    if (((D.step == DRIVE_CHANGE_SPEED) || (D.step == DRIVE_CRUISE)) && (lag > TOP_LAG_PULL)) {
        D.vin_out = act;
        D.scv_accel = 0;
        D.step = DRIVE_CHANGE_SPEED;
        lag = 0;
    }
    s_tp.n_tq = (int16_t)(((D.tq_cmd[0] >= TOP_TQ_SAT) || (D.tq_cmd[1] >= TOP_TQ_SAT)) ? s_tp.n_tq + 1 : 0);
    s_tp.n_lag = (int16_t)((lag > TOP_LAG_SAT) ? s_tp.n_lag + 1 : 0);
    if (s_tp.n_tq > TOP_SAT_TICKS)  { s_tp.n_tq = TOP_SAT_TICKS; }
    if (s_tp.n_lag > TOP_SAT_TICKS) { s_tp.n_lag = TOP_SAT_TICKS; }
    why = (s_tp.n_tq >= TOP_SAT_TICKS) ? 1 : ((s_tp.n_lag >= TOP_SAT_TICKS) ? 2 : 0);
    acc = ring_acc();
    if ((D.vin_out >= TOP_CMD_RPM) && ((int32_t)act * 100L >= (int32_t)TOP_CMD_RPM * 97L)) { why = 3; }
    else if ((why == 0) || (s_tp.rn < TOP_RING) || (acc >= TOP_FLAT_RPM_S)) {
        if (++s_id.t > RAMP_LIMIT) { return fail_now(); }
        return 0;
    }
    /* 최고 속도. 지령을 실제 속도에 고정하고 정속으로 잰다 */
    if (s_build) { acc_judge(); }
    tp->sat = 1;
    tp->sat_why = why;
    tp->reach_mm = mm_since(s_id.begin);
    tp->acc_end_rpm_s = acc;
    for (i = 0; i < 2u; i++) { tp->top_rpm[i] = s_tp.last[i]; }
    D.vin_out = act;
    D.scv_accel = 0;
    set_speed(act, 0);
    hold_clear();
    enter(PH_HOLD);
    return 0;
}

/* sat = 1 인 회의 top_rpm 최대. 없으면 peak 최대 */
static void top_finish(void)
{
    int16_t any = 0;
    uint16_t i, k;

    motor_spike_set(0);
    for (k = 0; k < TOP_PASSES; k++) { any |= agv_top.p[k].sat; }
    for (i = 0; i < 2u; i++) {
        int16_t m = 0;
        for (k = 0; k < TOP_PASSES; k++) {
            const top_pass_t *tp = &agv_top.p[k];
            int16_t v = any ? (tp->sat ? tp->top_rpm[i] : 0) : tp->peak_rpm[i];
            if (v > m) { m = v; }
        }
        agv_top.max_rpm[i] = m;
    }
    agv_top.max_mpm = ((float)agv_top.max_rpm[0] + (float)agv_top.max_rpm[1]) * 0.5f / P.one_meter_rpm;
}

/* ---- 지정 시험 단계 3: 타력 ---- */

/* v 에서 lo rpm 까지 타력 거리 어림 mm. 감속도 1000 rpm/s */
static float coast_mm(float v, float lo)
{
    return (v * v - lo * lo) / (2.0f * COAST_EST_RPM_T) * (1000.0f / 6000.0f / P.one_meter_rpm);
}

/* 회전 중심이 선 위에서 출발한다. 타력은 늦어도 budget 에서 시작해야 감속 끝점 전에 최저 속도가 된다 */
static void coast_begin(void)
{
    coast_pass_t *cp;
    float v;

    if (s_id.retry) { s_id.retry = 0; }
    else            { agv_coast.pass++; }
    cp = &agv_coast.p[PASS - 1u];
    memset(cp, 0, sizeof(*cp));
    memset(&s_cs, 0, sizeof(s_cs));
    if (pass_rpm() < level_rpm()) { cp->cut = 2; }   /* 낮춘 속도로 잰다 */
    v = (float)pass_rpm();
    s_id.budget = ahead_mm() - P.guide_dist_mm - DEC_MARGIN_MM - coast_mm(v, (float)P.min_speed);
    D.p_test = 0;
    enter(PH_RAMP);
}

/* ---- 지정 시험 단계 4: 정지 정확도 ---- */

/* p 회 목표 mm. 제작은 찾는 중인 목표(s_bld.cid), 점검·홀수 회는 35번(0 이면 STOPT_CID_MM), 짝수 회는 STOPT_SHORT_MM */
static float stopt_target(uint16_t p)
{
    int16_t cid = param_get(P_CROSS_IN_CM);
    if (s_build && !(s_build & BLD_CHECK)) { return s_bld.cid; }
    if ((p & 1u) || s_build) { return (cid > 0) ? (float)cid * 10.0f : (float)STOPT_CID_MM; }
    return (float)STOPT_SHORT_MM;
}

/* 선 자리는 목표나 타력 거리 중 먼 쪽까지로 본다(제동 토크가 없다) */
static void stopt_begin(void)
{
    stopt_pass_t *sp;
    float d, c;

    if (s_id.retry) { s_id.retry = 0; }
    else            { agv_stopt.pass++; }
    sp = &agv_stopt.p[PASS - 1u];
    memset(sp, 0, sizeof(*sp));
    memset(&s_cs, 0, sizeof(s_cs));
    if (pass_rpm() < level_rpm()) { sp->cut = 2; }
    d = stopt_target(PASS);
    sp->target_mm = d;
    c = coast_mm((float)pass_rpm(), 0.0f);
    s_id.budget = ahead_mm() - P.guide_dist_mm - DEC_MARGIN_MM - ((c > d) ? c : d);
    D.p_test = 0;
    enter(PH_RAMP);
}

/* 안정 끝. 이 자리에서 교차로 대기 목표를 건다. 제동점과 제동은 stop.c 교차로 위치 정지가 정한다 */
static int stopt_trigger(void)
{
    stopt_pass_t *sp = &agv_stopt.p[PASS - 1u];
    float n = sp->target_mm * P.one_length_pulse + 0.5f;       /* acs.c 와 같은 반올림 */

    sp->v0_rpm = (int16_t)((s_cs.v0[0] + s_cs.v0[1]) / (2L * COAST_V0_TICKS));
    sp->v0_mpm = (float)sp->v0_rpm / P.one_meter_rpm;
    s_cs.start = travel();
    s_cs.n = stop_last.n;
    D.cross_pulses = (n > 32767.0f) ? (int16_t)32767 : (int16_t)n;
    D.cross_in_flag = 1;
    enter(PH_STOPT);
    return 0;
}

/* 조용해질 때까지(stop_last 가 새로 쓰일 때까지) 기다린다. 그 뒤 대기를 풀고 최저 속도로 선을 찾는다 */
static int stopt_wait(void)
{
    stopt_pass_t *sp = &agv_stopt.p[PASS - 1u];

    s_id.t++;
    if ((D.guide_pos >= COAST_GUIDE_OFF) || (D.guide_pos <= -COAST_GUIDE_OFF)) { return fail_now(); }
    if (stop_last.n == s_cs.n) { return (s_id.t > STOP_LIMIT) ? fail_now() : 0; }
    sp->travel_mm = mm_since(s_cs.start);
    sp->overshoot_mm = stop_last.overshoot_mm;
    sp->required_decel = stop_last.required_decel;
    sp->rate_limited = stop_last.rate_limited;
    sp->reason = stop_last.reason;
    sp->stop_ms = ms(s_id.t);
    set_speed(P.min_speed, 1);
    acs_cross_release(ACS_CROSS_IN);     /* 다음 틱 stop.c 가 0 에서 주행 램프로 넘긴다 */
    leave(0);
    return 0;
}

/* tc[a] → tc[b] 구간 감속도 rpm/s. 닿지 않았으면 -1 */
static int16_t coast_dec(float v0, float frac, uint16_t a, uint16_t b)
{
    float dt = s_cs.tc[b] - s_cs.tc[a];
    if ((s_cs.tc[a] < 0.0f) || (s_cs.tc[b] < 0.0f) || (dt <= 0.0f)) { return -1; }
    return (int16_t)(v0 * frac / (dt * (TICK_MS / 1000.0f)));
}

/* 고정을 풀고 제어를 잇는다. 지령은 실제 속도(최저 속도 상한)에서 최저 속도로 간다 */
static int coast_end(coast_pass_t *cp, int16_t act, int reached)
{
    float v0 = ((float)cp->v0_rpm[0] + (float)cp->v0_rpm[1]) * 0.5f;
    uint16_t i;

    motor_dac_release();
    cp->t_min_ms = reached ? (int16_t)ms(s_id.t) : -1;
    cp->dist_mm = (int16_t)mm_since(s_cs.start);
    cp->dec_rpm_s = coast_dec(v0, 0.6f, 0u, 2u);
    cp->dec_hi_rpm_s = coast_dec(v0, 0.3f, 0u, 1u);
    cp->dec_lo_rpm_s = coast_dec(v0, 0.3f, 1u, 2u);
    cp->v0_mpm = v0 / P.one_meter_rpm;
    cp->dec_mps2 = (cp->dec_rpm_s < 0) ? -1.0f : (float)cp->dec_rpm_s / P.one_meter_rpm / 60.0f;
    if (cp->dec_mps2 > COAST_BRAKE_MPS2) { cp->cut |= COAST_CUT_BRAKE; }
    for (i = 0; i < 2u; i++) { D.w_pi_out[i] = D.iref_it[i] = D.iref_fb[i] = 0.0f; }   /* 타력 전 정속 적분은 버린다 */
    steer_reset();
    D.vin_out = (act < P.min_speed) ? act : P.min_speed;
    D.scv_accel = 0;
    set_speed(P.min_speed, 1);
    D.step = DRIVE_CHANGE_SPEED;
    leave(0);
    return 0;
}

/* ACC 찾기 결과를 내고 12번 RAM 을 다음 후보(끝나면 시작 값)로 둔다 */
static void acc_set(void)
{
    agv_build.acc_max = s_acc.lo;
    agv_build.acc_rec = ((agv_build.acc_cap > 0) && (agv_build.acc_cap < s_acc.lo)) ? agv_build.acc_cap : s_acc.lo;
    s_acc.c = (s_acc.hi > s_acc.lo) ? ((s_acc.c == 0) ? s_acc.hi : (int16_t)((s_acc.lo + s_acc.hi + 1) / 2)) : 0;
    s_acc.t = s_acc.win = s_acc.bad = s_acc.slip_n = 0;
}

/* 타력 단계 들어갈 때. 상한이 지금 값 이하면 찾지 않는다(추천 = 상한) */
static void acc_begin(void)
{
    int16_t cap = agv_build.acc_cap;

    s_acc.a0 = P.acc;
    s_acc.lo = (cap > 0) && (cap < s_acc.a0) ? cap : s_acc.a0;
    s_acc.hi = (cap > s_acc.lo) ? cap : s_acc.lo;
    s_acc.c = 0;
    acc_set();
}

/* 가속 램프 매 틱(후보가 있을 때). 지령이 목표의 30 → 80% 를 지나는 동안 실제 가속도, 토크 지령, 좌우 rpm 어긋남을 본다.
 * 목표는 최고 속도 회면 1번, 아니면 그 회 속도. 저속 가속 제한(llow 아래 ACC 5) 구간은 빼고 본다 */
static void acc_ramp(void)
{
    int32_t a = D.rpm[0], b = D.rpm[1], d = (a > b) ? a - b : b - a;
    int32_t ref = s_top ? P.target_speed : s_id.want;
    int32_t pct = (ref > 0) ? ((int32_t)D.vin_out * 100L) / ref : 0;
    int32_t lo = ((ref * ACC_WIN_LO) / 100L > (int32_t)P.llow_speed * 11L / 10L) ? (ref * ACC_WIN_LO) / 100L
               : (int32_t)P.llow_speed * 11L / 10L;

    if ((s_acc.c == 0) || stopt() || (lo * 100L >= ref * ACC_WIN_HI)) { return; }
    s_acc.t++;
    if ((s_acc.win == 0) && (D.vin_out >= lo)) {
        s_acc.win = 1;
        s_acc.t1 = s_acc.t;
        s_acc.r1 = (float)(a + b) * 0.5f;
    }
    if (s_acc.win != 1) { return; }
    if ((pct <= ACC_TQ_PCT)
        && (((float)D.tq_cmd[0] >= MOTOR_TORQUE_LIMIT * ACC_TQ_OK) || ((float)D.tq_cmd[1] >= MOTOR_TORQUE_LIMIT * ACC_TQ_OK))) {
        s_acc.bad = 1;
    }
    s_acc.slip_n = (int16_t)(((d > ACC_SLIP_RPM) && (d * 200L > (a + b) * ACC_SLIP_PCT)) ? s_acc.slip_n + 1 : 0);
    if (s_acc.slip_n >= ACC_SLIP_TICKS) {
        s_acc.bad = 1;
        agv_build.acc_slip = 1;
    }
    if (pct >= ACC_WIN_HI) {        /* 창 끝: 실제 가속도(rpm/틱)가 후보를 따라왔나 */
        float r = ((float)(a + b) * 0.5f - s_acc.r1) / (float)(s_acc.t - s_acc.t1);
        if (r * 100.0f < (float)(s_acc.c * ACC_FOLLOW_PCT)) { s_acc.bad = 1; }
        s_acc.win = 2;
    }
}

/* 램프 시작(enter PH_RAMP): 창을 지우고(끊긴 램프는 판정하지 않는다) 이 회 ACC 를 RAM 에 넣는다 */
static void acc_reset(void)
{
    s_acc.t = s_acc.win = s_acc.bad = s_acc.slip_n = 0;
    if (s_build && (s_acc.a0 > 0)) { (void)param_set_ram(P_ACC, ((s_acc.c > 0) && !stopt()) ? s_acc.c : s_acc.a0); }
}

/* 램프 끝. 창을 다 봤으면 판정해 다음 후보로 */
static void acc_judge(void)
{
    if ((s_acc.c == 0) || (s_acc.win != 2)) { return; }
    if (s_acc.bad) { s_acc.hi = (int16_t)(s_acc.c - 1); }
    else           { s_acc.lo = s_acc.c; }
    acc_set();
}

static int coast_run(int stopped)
{
    static const float frac[3] = { 0.9f, 0.6f, 0.3f };
    coast_pass_t *cp;
    uint16_t ph = (uint16_t)*s_r.phase;
    int16_t spike = motor_spike(), act;
    uint16_t i;
    float m, v0;

    if (stopped || D.flag_stopping || D.stop_sw || D.estop || (D.system_err != 0)) { return fail_now(); }
    for (i = 0; i < 2u; i++) {      /* 튄 값은 앞 값으로 둔다 */
        int16_t r = motor_raw_rpm(i);
        if (r <= spike) { s_cs.last[i] = r; }
    }
    act = (int16_t)(((int32_t)s_cs.last[0] + s_cs.last[1]) / 2);

    if (ph == PH_RAMP) {
        set_speed(pass_rpm(), 1);
        if (s_build) { acc_ramp(); }
        if (mm_since(s_id.begin) > s_id.budget) { return slow_down(); }
        if ((D.step == DRIVE_CRUISE) && (D.vin_out == s_id.want)) {
            float v = (float)s_id.want / P.one_meter_rpm * (1000.0f / 6000.0f);   /* mm/틱 */
            if (s_build) { acc_judge(); }
            if (mm_since(s_id.begin) + v * (float)SETTLE_TICKS > s_id.budget) { return slow_down(); }
            enter(PH_SETTLE);
        }
        else if (++s_id.t > RAMP_LIMIT) { return fail_now(); }
        return 0;
    }
    if (ph == PH_SETTLE) {
        if (s_id.t >= SETTLE_TICKS - COAST_V0_TICKS) {
            for (i = 0; i < 2u; i++) { s_cs.v0[i] += s_cs.last[i]; }
        }
        if (++s_id.t < SETTLE_TICKS) { return 0; }
        if (stopt()) { return stopt_trigger(); }
        cp = &agv_coast.p[PASS - 1u];
        for (i = 0; i < 2u; i++) { cp->v0_rpm[i] = (int16_t)(s_cs.v0[i] / COAST_V0_TICKS); }
        s_cs.prev = ((float)cp->v0_rpm[0] + (float)cp->v0_rpm[1]) * 0.5f;
        for (i = 0; i < 3u; i++) { s_cs.tc[i] = -1.0f; }
        s_cs.start = travel();
        motor_dac_force(0, 0);
        enter(PH_COAST);
        return 0;
    }

    if (stopt()) { return stopt_wait(); }
    /* PH_COAST. s_id.t = 고정부터 틱 */
    cp = &agv_coast.p[PASS - 1u];
    m = ((float)s_cs.last[0] + (float)s_cs.last[1]) * 0.5f;
    v0 = ((float)cp->v0_rpm[0] + (float)cp->v0_rpm[1]) * 0.5f;
    s_id.t++;
    if (((s_id.t & 1) == 0) && (cp->n < COAST_CURVE)) { cp->curve[cp->n++] = (int16_t)m; }
    for (i = 0; i < 3u; i++) {      /* 앞 틱은 문턱 위였다. 두 틱 사이를 잇는다 */
        float th = v0 * frac[i];
        if ((s_cs.tc[i] < 0.0f) && (m <= th)) { s_cs.tc[i] = (float)(s_id.t - 1) + (s_cs.prev - th) / (s_cs.prev - m); }
    }
    s_cs.prev = m;
    if (m <= (float)P.min_speed) { return coast_end(cp, act, 1); }
    if ((D.guide_pos >= COAST_GUIDE_OFF) || (D.guide_pos <= -COAST_GUIDE_OFF)) {
        cp->cut |= 4;
        return coast_end(cp, act, 0);
    }
    if (ahead_mm() <= P.guide_dist_mm + DEC_MARGIN_MM) {
        cp->cut |= 1;
        return coast_end(cp, act, 0);
    }
    if (s_id.t >= COAST_TICKS) { return coast_end(cp, act, 0); }   /* 실패가 아니다. 기록하고 다음 회 */
    return 0;
}

/* 패널 키. 타력 중이면 바로 고정을 푼다. 단계 4 정지 중이면 대기를 풀고 실패 */
int ident_coast_key(void)
{
    if (!coast() || (*s_r.state != 1) || (*s_r.phase != PH_COAST)) { return 0; }
    (void)fail_now();
    return 1;
}

/* ---- 바닥 rpm 시험 1~5회 ---- */

/* 앞 선까지 거리는 rpmtest 가 ident_ahead_mm 으로 본다 */
static void rpm_begin(void)
{
    D.p_test = 0;
    rpmt_pass_begin();
}

static int rpm_line(void);

static int rpm_run(int stopped)
{
    int r;

    if (stopped || D.flag_stopping || D.stop_sw || D.estop || (D.system_err != 0)) { return fail_now(); }
    if (D.guide_all_on) { return rpm_line(); }
    /* 개루프 중 편차는 rpmtest 가 5칸에서 끊고 세운다 */
    if (!motor_dac_forced() && ((D.guide_pos >= COAST_GUIDE_OFF) || (D.guide_pos <= -COAST_GUIDE_OFF))) { return fail_now(); }
    r = rpmt_pass_run();
    if (r < 0) { return fail_now(); }
    if (r > 0) {                     /* 회 끝. 다음 자리로 */
        rpm_release();
        if (r == 2) {                /* 비뚤어졌다. 충격 없이 푼 지령에서 보통 정지한 뒤 정한다 */
            tagact_stop();
            s_id.rstop = 1;
            s_id.t = 0;
            return 0;
        }
        leave(0);
    }
    return 0;
}

int ident_rpm_key(void)
{
    if (!s_rpm || (*s_r.state != 1)) { return 0; }
    (void)fail_now();
    return 1;
}

/* 연속 시험 0회 둘째 직선 끝. 좌우 펄스 비, 입력 L 로 휠 지름(바로 저장). BLD_GA 면 바로 선다.
 * -1 실패(잰 L 이 입력과 10% 넘게 다르다), 1 정지 시작, 0 그대로 회전 */
static int bld_leg2(float leg)
{
    uint32_t dl = run_pulses(0) - s_bld.l0, dr = run_pulses(1) - s_bld.r0;
    int16_t in = agv_build.track_in_mm;

    agv_build.track_mm = leg;
    if (dl != 0UL) { agv_build.lr_permille = (int16_t)(((float)dr / (float)dl - 1.0f) * 1000.0f); }
    if ((s_build & BLD_DIA) && (in > 0)) {
        float r = (float)in / leg;
        if ((r < 0.9f) || (r > 1.1f)) { return -1; }
        agv_build.dia_new = clamp_p(P_WHEEL_DIA_MM, (int32_t)((float)agv_build.cfg_dia * r + 0.5f));
        (void)param_set(P_WHEEL_DIA_MM, agv_build.dia_new);
        *s_r.track = (float)in;
    }
    if (!(s_build & BLD_GA)) { return 0; }
    s_bld.line = tune_line_at(0);
    s_bld.ga = 1;
    tagact_stop();
    enter(PH_STOP);
    return 1;
}

static void add_leg(float leg)
{
    if (!s_top && !sweep() && !coast() && !s_rpm) { agv_ident.total_m += leg / 1000.0f; }
    if (s_all) { agv_all.total_m += leg / 1000.0f; }
}

/* 선을 봤다(line = 그때 펄스). 회전 중심이 선 위에 오도록 GD 만큼 더 가서 돈다. 위치를 그 선으로 맞춘다 */
static int cross_at(uint32_t line)
{
    float leg = (float)(line - s_id.begin) / P.one_length_pulse + P.guide_dist_mm;
    float lp;

    add_leg(leg);
    if ((PASS == 0u) && (s_id.legs == 1)) {
        *s_r.track = leg;
        if (leg < TRACK_MIN_MM) { return fail_now(); }
        if (s_build) {
            int r = bld_leg2(leg);
            if (r < 0) { return fail_now(); }
            if (r > 0) { return 0; }
        }
        s_pos.len = *s_r.track;     /* 지름을 고쳤으면 입력 L */
        s_pos.ok = 1;
        s_len_seen = s_pos.len;
    }
    lp = (s_pos.dir > 0) ? s_pos.len : 0.0f;
    s_pos.at = lp - (float)s_pos.dir * P.guide_dist_mm;
    s_pos.ref = line;
    s_pos.turn_at = lp;
    s_id.legs++;
    D.tape_slow = (PASS == 0u);    /* 0회 회전은 회전 보정비를 잰다. 나머지는 회전 속도(3번)로 돈다 */
    s_id.turn_seen = 0;
    tagact_turn_align_cross(TURN_CW, 180u, 0u);
    if (!D.tape_align) { return fail_now(); }
    enter(PH_TURN);
    return 0;
}

static int cross_seen(void) { return cross_at(tune_line_at(0)); }

/* 바닥 rpm 시험 측정 중 교차선. 실패가 아니다: 그 단계를 끊고(cut 4) 고정을 풀어 실제 속도에서 잇고,
 * 회 끝에 선 찾기가 선을 본 것처럼 GD 더 가서 서고 돈다 */
static int rpm_line(void)
{
    if (ahead_mm() - P.guide_dist_mm > LINE_TOL_MM) { return fail_now(); }   /* 위치 모델이 틀렸다 */
    rpmt_pass_cut(RPMT_CUT_LINE);
    rpm_release();
    return cross_at(travel());
}

/* 통합(지정) 시험 끝. 51·39번을 되돌리고 그 자리에서 기다린다 */
static int all_end(void)
{
    agv_all.state = 2;
    opt_restore();
    if (s_build) { build_restore(2); }
    return 1;
}

/* ---- 고속 스윕 (튜닝 t3.6) ---- */

/* 0 에서 to rpm 까지 주행 램프 가속 거리 mm */
static float accel_mm(int16_t to)
{
    const float k = 1000.0f / 6000.0f / P.one_meter_rpm;
    int16_t v = 0, a = 0, n = 0;
    float sum = 0.0f;

    while ((v < to) && (n < 3000)) {
        (void)motor_scv_core(to, &v, P.acc, P.dcc, P.llow_speed, P.min_speed, P.min_speed, 1, &a);
        sum += (float)v;
        n++;
    }
    return sum * k;
}

/* plan() 이 정속·옆 계단 창을 WIN_MIN 씩 받는 최소 L. 가속 중 실제 속도가 늦으면 여유가 는다 */
static float mm_tick(int16_t v) { return (float)v / P.one_meter_rpm * (1000.0f / 6000.0f); }

/* 서 있다 출발해 v 로 가속하고 ticks 동안 정속으로 간 뒤 end mm 를 남기는 회에 필요한 앞 거리(회전 중심 → 앞 선) */
static float run_need(int16_t v, int16_t ticks, float end)
{
    return P.guide_dist_mm + accel_mm(v) + (float)ticks * mm_tick(v) / PLAN_SLACK + end;
}

float ident_hs_need_mm(int16_t mpm)
{
    int16_t v = param_rpm(mpm);
    return run_need(v, SETTLE_TICKS + 2 * WIN_MIN, DEC_MARGIN_MM + decel_mm(v));
}

void ident_start_hs(int16_t mpm)
{
    sweep_cand_t l[HS_REPS * HS_CANDS];
    uint16_t k;

    for (k = 0; k < HS_REPS * HS_CANDS; k++) { l[k] = s_hs_cand[k % HS_CANDS]; }
    all_begin(l, HS_REPS * HS_CANDS);
    memset(&agv_coast, 0, sizeof(agv_coast));
    memset(&agv_stopt, 0, sizeof(agv_stopt));
    s_temp = 1;                       /* 0회 뒤 바로 스윕 */
    s_hs = param_rpm(mpm);
    agv_all.hs_mpm = mpm;
    agv_all.need_mm = ident_hs_need_mm(mpm);
}

int ident_hs_mode(void) { return s_hs != 0; }

int16_t ident_hs_rms10(uint16_t c)
{
    float sum = 0.0f, n = 0.0f;
    uint16_t k, i;

    for (k = c; k < (uint16_t)agv_all.sweep_n; k += HS_CANDS) {
        for (i = 0; i < SWEEP_PASSES; i++) {
            const ident_level_t *lv = &agv_all.sw[k].lv[i];
            if (lv->v_mpm > 0.0f) { sum += lv->dev_rms_mm; n += 1.0f; }
        }
    }
    return (n > 0.0f) ? (int16_t)(sum / n * 10.0f + 0.5f) : -1;
}

uint16_t ident_hs_best(void)
{
    uint16_t c, best = HS_CANDS;
    int16_t v, m = 0x7FFF;

    for (c = 0; c < HS_CANDS; c++) {
        v = ident_hs_rms10(c);
        if ((v >= 0) && (v < m)) { m = v; best = c; }
    }
    return best;
}

/* ---- 연속 시험 (튜닝 t1·t2) ---- */

void ident_start_build(uint16_t mask, int16_t track_in_mm, int16_t cap_mpm)
{
    sweep_cand_t now;                 /* 편차 단계는 지금 설정 그대로 */

    now.opt = param_get(P_OPT_FLAGS);
    now.quirk = param_get(P_LEGACY_QUIRKS);
    now.src = SWEEP_SRC_NOW;
    all_begin(&now, (mask & BLD_STEER) ? 1u : 0u);
    memset(&agv_coast, 0, sizeof(agv_coast));
    memset(&agv_stopt, 0, sizeof(agv_stopt));
    memset(&agv_build, 0, sizeof(agv_build));
    memset(&s_bld, 0, sizeof(s_bld));
    s_bld.cid = CID_FIRST_MM;
    agv_build.state = 1;
    agv_build.mask = (int16_t)mask;
    agv_build.track_in_mm = track_in_mm;
    agv_build.cap_mpm = cap_mpm;
    agv_build.cfg_high = param_get(P_SPEED_HIGH);
    agv_build.cfg_dcc = param_get(P_DCC);
    agv_build.cfg_dia = param_get(P_WHEEL_DIA_MM);
    agv_build.cfg_wb = param_get(P_WHEEL_BASE_MM);
    agv_build.min_rec = agv_build.high_rec = agv_build.dcc_rec = agv_build.cid_rec = -1;
    agv_build.acc_max = agv_build.acc_rec = -1;
    memset(&s_acc, 0, sizeof(s_acc));
    s_acc.a0 = -1;
    agv_build.se_x10 = TUNE_NO_DATA;
    s_build = mask;
}

int ident_build_mode(void) { return s_build != 0u; }

/* 제작 정지 단계: LDS 가 켜져 있고 EDS 가 지금(추천) 1번보다 빠르면 min(EDS, 상한) 과 EDC(쓸 수 없으면 지금 13번)로
 * 달린다. 빠른 공차 프로필도 CD 안에 서야 한다. 단계 끝에 되돌린다(실패·끊김은 build_restore) */
static void stopt_profile(int on)
{
    int16_t eds = param_get(P_EMPTY_SPEED), edc = param_get(P_EMPTY_DCC), lo, up;

    if (!on) {
        if (s_bld.prof_high > 0) { (void)param_set_ram(P_SPEED_HIGH, s_bld.prof_high); }
        if (s_bld.prof_dcc > 0)  { (void)param_set_ram(P_DCC, s_bld.prof_dcc); }
        s_bld.prof_high = s_bld.prof_dcc = 0;
        return;
    }
    if ((agv_build.cap_mpm > 0) && (eds > agv_build.cap_mpm)) { eds = agv_build.cap_mpm; }
    if ((s_build & BLD_CHECK) || (param_get(P_LOAD_DETECT) == 0) || (eds <= param_get(P_SPEED_HIGH))) { return; }
    s_bld.prof_high = param_get(P_SPEED_HIGH);
    (void)param_set_ram(P_SPEED_HIGH, eds);
    param_limit(P_DCC, &lo, &up);
    if (edc >= lo) {
        s_bld.prof_dcc = param_get(P_DCC);
        (void)param_set_ram(P_DCC, edc);
    }
}

/* 단계가 끝났다. 추천값을 내고 바로 저장 몫을 저장한다. 1·13번은 뒤 단계를 위해 RAM 에 넣는다 */
static void stage_end(uint16_t st)
{
    build_result_t *b = &agv_build;
    uint16_t i, k;

    switch (st) {
    case 1:
        agv_all.track_mm = agv_ident.track_mm;
        agv_build.guide_k100 = tune_guide_k100();   /* 저장하면 뒤 측정은 고친 'h' 로 한다. 지금 값을 남긴다 */
        if (s_build & BLD_GUIDE) { (void)tune_guide_save(); }
        break;
    case 2:
        b->amp = agv_rpmt.amp;
        if (agv_rpmt.low_rpm > 0) {   /* 안정 최저 rpm × 1.2. 정지 바닥(350 rpm) 아래면 선 앞 정지가 실패하니 바닥 이상.
                                         4 m/min 미만은 파생값 계산이 6 으로 바꾸므로 4 이상 */
            float r = (float)agv_rpmt.low_rpm * 1.2f;
            int16_t v = (int16_t)ceilf(((r < (float)STOP_MIN_RPM) ? (float)STOP_MIN_RPM : r) / P.one_meter_rpm);
            b->min_rec = clamp_p(P_SPEED_MIN, (v < 4) ? 4 : v);
            if (s_build & BLD_MIN) { (void)param_set(P_SPEED_MIN, b->min_rec); }
        }
        break;
    case 3:
        top_finish();
        b->top_rpm = (int16_t)(((int32_t)agv_top.max_rpm[0] + agv_top.max_rpm[1]) / 2);
        if (b->top_rpm > 0) {
            int16_t v = (int16_t)(agv_top.max_mpm * 0.85f);
            b->high_rec = clamp_p(P_SPEED_HIGH, (v > b->cap_mpm) ? b->cap_mpm : v);
            (void)param_set_ram(P_SPEED_HIGH, b->high_rec);
        }
        break;
    case 4: {
        int32_t sum = 0;
        for (i = k = 0; i < BLD_COAST_PASSES; i++) {
            if ((agv_coast.p[i].dec_rpm_s > 0) && !(agv_coast.p[i].cut & COAST_CUT_BRAKE)) { sum += agv_coast.p[i].dec_rpm_s; k++; }
        }
        /* 유효한 회가 없으면 추천 없음(-1)·감속 측정 없음(0). 뒤 단계는 지금 DCC 로 간다 */
        if (k > 0u) {                 /* 타력으로 낼 수 있는 감속의 0.8. DCC 1 = 100 rpm/s */
            b->dec_rpm_s = (int16_t)(sum / (int32_t)k);
            b->dcc_rec = clamp_p(P_DCC, ((int32_t)b->dec_rpm_s * 8L) / 1000L);
            (void)param_set_ram(P_DCC, b->dcc_rec);
        }
        break;
    }
    case 5: {
        const stopt_pass_t *sp = &agv_stopt.p[PASS - 1u];
        stopt_profile(0);
        b->over_mm = (int16_t)sp->overshoot_mm;
        if (!(s_build & BLD_CHECK) && (sp->overshoot_mm <= (float)BLD_OVER_OK_MM)) { b->cid_rec = (int16_t)(sp->target_mm / 10.0f); }
        break;
    }
    default: {                        /* 편차: 정속 편차 RMS × 10 */
        const ident_level_t *lv = &agv_all.sw[0].lv[0];
        if (lv->v_mpm > 0.0f) { b->se_x10 = (int16_t)(lv->dev_rms_mm * 10.0f + 0.5f); }
        break;
    }
    }
}

/* mask 에서 st 다음 단계. 0 = 없음 */
static uint16_t bld_after(uint16_t st)
{
    do { st++; } while ((st <= BLD_STAGES) && !(s_build & (1u << (st - 1u))));
    return (st > BLD_STAGES) ? 0u : st;
}

/* mask 에서 다음 단계로 s_r 을 돌린다. 1 = 남은 단계 없음 */
static int build_next(void)
{
    uint16_t st = bld_after((uint16_t)agv_all.stage);

    if (st == 0u) { return 1; }
    s_id.retry = 0;
    s_top = 0;
    s_rpm = 0;
    agv_all.stage = (int16_t)st;
    switch (st) {
    case 2:
        rpm_point();
        agv_rpmt.track_mm = agv_all.track_mm;
        break;
    case 3:
        point_top();
        agv_top.track_mm = agv_all.track_mm;
        break;
    case 4:
        point(&agv_coast.state, &agv_coast.fail, &agv_coast.pass, &agv_coast.phase, &agv_all.track_mm);
        break;
    case 5:
        point(&agv_stopt.state, &agv_stopt.fail, &agv_stopt.pass, &agv_stopt.phase, &agv_all.track_mm);
        stopt_profile(1);
        break;
    default:
        agv_all.pass = agv_all.phase = 0;
        point(&agv_all.state, &agv_all.fail, &agv_all.pass, &agv_all.phase, &agv_all.track_mm);
        break;
    }
    if ((st >= 3u) && (s_acc.a0 < 0)) { acc_begin(); }   /* 최고 속도 단계(없으면 그 뒤)부터 ACC 찾기 */
    *s_r.state = 1;
    return 0;
}

/* 제작 최고 속도: 1회가 포화까지 가서 1번 추천이 이미 상한이면 2회는 추천을 바꾸지 못한다 */
static int top_capped(void)
{
    const top_pass_t *tp = &agv_top.p[0];
    return !(s_build & BLD_CHECK) && (agv_build.cap_mpm > 0) && tp->sat
        && (tp->top_mpm * 0.85f >= (float)agv_build.cap_mpm);
}

/* 정지 단계 끝 조건: 점검은 한 번, 제작은 지나침이 BLD_OVER_OK_MM 이하이거나 목표가 상한 */
static int cid_done(void)
{
    const stopt_pass_t *sp = &agv_stopt.p[PASS - 1u];
    return (s_build & BLD_CHECK) || (sp->overshoot_mm <= (float)BLD_OVER_OK_MM) || (sp->target_mm >= CID_MAX_MM);
}

/* 제작 다음 목표. 제동 없이 서는 거리 T = 목표 + 지나침 은 목표에 거의 상관없다. 지나침 ≤ BLD_OVER_OK_MM 인
 * 가장 작은 10 cm 단위 = 올림(T - OK). 적어도 지금 + 10 cm, 많아야 CID_MAX_MM */
static float cid_next(void)
{
    const stopt_pass_t *sp = &agv_stopt.p[PASS - 1u];
    float d = ceilf((sp->target_mm + sp->overshoot_mm - (float)BLD_OVER_OK_MM) / 100.0f) * 100.0f;
    if (d < sp->target_mm + 100.0f) { d = sp->target_mm + 100.0f; }
    return (d > CID_MAX_MM) ? CID_MAX_MM : d;
}

/* 회전(또는 그 자리 정지)이 끝났다. 단계가 끝났으면 다음 단계로, 마지막이면 그 자리에서 기다린다 */
static int build_turn(void)
{
    uint16_t st = (uint16_t)agv_all.stage;
    int done = 0;

    if (s_id.retry) { done = 0; }    /* 속도를 낮춰 같은 회를 다시 */
    else if (st == 1u) {
        done = (s_id.legs >= 2);
        if (done) {                   /* 둘째 회전은 고친 지름으로 돌았다. 목표 펄스에는 지금 보정비가 걸려 있다 */
            float r = tune_turn_scale() * (float)D.tape_turned / (float)D.tape_total_pulse;
            int16_t v = (int16_t)(r * 1000.0f + 0.5f);
            agv_build.turn_x1000 = (v < TUNE_TURN_LOW) ? TUNE_TURN_LOW : ((v > TUNE_TURN_UP) ? TUNE_TURN_UP : v);
            if (s_build & BLD_WB) { tune_turn_scale_set(agv_build.turn_x1000); }
        }
    }
    else if (st == 2u) { done = (PASS >= 4u); }
    else if (st == 3u) { done = (PASS >= TOP_PASSES) || top_capped(); }
    else if (st == 4u) { done = (PASS >= BLD_COAST_PASSES); }
    else if (st == 5u) {
        done = cid_done();
        if (!done) { s_bld.cid = cid_next(); }
    }
    else {
        agv_all.sw[agv_all.sweep_k].done++;
        done = (PASS >= BLD_DEV_PASSES);
    }
    if (done) {
        *s_r.state = 2;
        stage_end(st);
        if (build_next()) { return all_end(); }
    }
    run_restart_now();
    s_id.restart = 1;
    return 0;
}

/* BLD_GA: 선을 보고 선 뒤 미끄러진 거리를 잰다. 기구 단계만 하고 끝낸다 */
static int bld_ga_wait(int stopped)
{
    if (!stopped) { return (++s_id.t > STOP_LIMIT) ? fail_now() : 0; }
    s_bld.ga = 0;
    agv_build.ov_mm = (int16_t)((float)(travel() - s_bld.line) / P.one_length_pulse + 0.5f);
    agv_ident.state = 2;
    stage_end(1u);
    return all_end();
}

static int turn_done(void);

/* ---- 회 사이 이동 ----
 * 측정이 끝나면 다음 회에 필요한 앞 거리(need)를 셈한다. 지금 자리에서 서면 뒤(온 쪽 선까지)가 need 이상이면
 * 그 자리에서 서서 직선 테이프 위에서 180도 정렬 회전한다. 모자라면 LOS(2번)로 더 가서 뒤가 need 가 되는 자리에서
 * 서고 돈다. need 가 직선에 안 들어가거나(최고 속도 시험, 개루프 rpm 회 등) 위치를 모르면 앞 선까지 LOS 로 가고
 * 마지막 APPROACH_MM 만 최저 속도로 들어가 선 위에서 돈다. 다음 회가 방향을 가리지 않고(제작·점검의 모든 회,
 * 그 밖의 짝 회 첫 회) 앞이 need 이상이면 돌지 않고 서서 같은 방향으로 잇는다. 시험이 끝나면 돌지 않고 선다 */

/* 다음 회의 종류와 번호(np). free 1 = 방향을 가리지 않는다(제작·점검, 짝 회의 첫 회, 단계 첫 회, 다시 하는 회) */
static int16_t next_kind(uint16_t *np, int16_t *free)
{
    uint16_t p = PASS, st = (uint16_t)agv_all.stage;
    int done;

    *free = 1;
    *np = p;
    if (s_top) {                                     /* 최고 속도 시험은 선 가까이서 끝난다. 이어지면 그 선에서 돈다 */
        int last = (p >= TOP_PASSES) || (s_build && top_capped());
        return (last && (!s_build || (bld_after(3u) == 0u))) ? NX_END : NX_LINE;
    }
    if (s_id.retry) { return sweep() ? NX_SWEEP : (stopt() ? NX_STOPT : (coast() ? NX_COAST : NX_IDENT)); }
    *np = (uint16_t)(p + 1u);
    /* 제작·점검(t1·t2)은 앞이 넉넉하면 같은 방향으로 잇는다. 식별·지정·통합 시험의 같은 속도 왕복 둘째 회는
     * 반대 방향 그대로다: 옆 계단·편차·타력(조향 끔)은 정렬 회전이 테이프 가운데에 다시 세운 뒤 재야 한다 */
    *free = s_build || ((p & 1u) == 0u);
    if (s_build) {
        done = (st == 2u) ? (p >= 4u) : (st == 4u) ? (p >= BLD_COAST_PASSES) : (st == 5u) ? cid_done()
             : (p >= BLD_DEV_PASSES);
        if (!done) {
            if (st == 2u) { *np = 4u; *free = 1; return NX_RPM; }   /* 점검 1회 뒤 4회 */
            return (st == 4u) ? NX_COAST : (st == 5u) ? NX_STOPT : NX_SWEEP;
        }
        *free = 1;
        *np = 1u;
        st = bld_after(st);
        if (st == 2u) { *np = (s_build & BLD_CHECK) ? 1u : 4u; return NX_RPM; }
        return (st == 0u) ? NX_END : (st == 3u) ? NX_LINE : (st == 4u) ? NX_COAST : (st == 5u) ? NX_STOPT : NX_SWEEP;
    }
    if (sweep()) {
        *free = ((p % SWEEP_PASSES) == 0u);
        if (p < (uint16_t)(agv_all.sweep_n * SWEEP_PASSES)) { return NX_SWEEP; }
        *free = 1;
        *np = 1u;
        return s_hs ? NX_END : (s_temp ? NX_COAST : NX_LINE);
    }
    if (stopt()) { return (p < STOPT_PASSES) ? NX_STOPT : NX_END; }
    if (coast()) {
        if (p < COAST_PASSES) { return NX_COAST; }
        *free = 1;
        *np = 1u;
        return NX_STOPT;
    }
    if (s_rpm) {
        *free = 1;
        return (p < RPMT_PASSES) ? NX_RPM : NX_END;
    }
    if (p < IDENT_PASSES) { return NX_IDENT; }
    *free = 1;
    *np = 1u;
    return !s_all ? NX_END : ((agv_all.sweep_n > 0) ? NX_SWEEP : NX_LINE);
}

/* 다음 회가 서 있다 출발해 측정을 다 하는 데 필요한 앞 거리. 0 = 다음 회 없음, NEED_LINE = 선에서 출발 */
static float next_need(int16_t *free)
{
    uint16_t p;
    int16_t k = next_kind(&p, free), v;
    float c, d;

    switch (k) {
    case NX_END:
        return 0.0f;
    case NX_IDENT:                   /* plan() 이 정속·옆 계단 창을 WIN_TICKS 씩 다 받는 거리 */
        v = rpm_at((uint16_t)((p - 1u) / 2u));
        return run_need(v, (int16_t)(SETTLE_TICKS + 2 * WIN_TICKS + ((p == 1u) ? 2 * SPD_TICKS : 0)),
                        (((p & 1u) != 0u) ? DEC_MARGIN_MM + decel_mm(v) : MARGIN_MM + sl_mm())
                        + ((p == 1u) ? SPD_STEP_MPM * (1000.0f / 6000.0f) * (float)SPD_TICKS : 0.0f));
    case NX_SWEEP:
        v = rpm_at(SWEEP_LEVEL);
        return run_need(v, SETTLE_TICKS + 2 * WIN_TICKS, DEC_MARGIN_MM + decel_mm(v));
    case NX_COAST:                   /* 타력은 길어야 COAST_TICKS. 그동안 v 그대로 간다고 본다 */
        v = rpm_at(s_build ? (uint16_t)(IDENT_LEVELS - 1u) : (uint16_t)((p + 1u) / 2u));
        return run_need(v, SETTLE_TICKS + COAST_TICKS, DEC_MARGIN_MM);
    case NX_STOPT:
        v = rpm_at(s_build ? (uint16_t)(IDENT_LEVELS - 1u) : (uint16_t)((p - 1u) / 2u));
        c = coast_mm((float)v, 0.0f);
        d = (s_build && !(s_build & BLD_CHECK) && (agv_all.stage == 5) && !s_id.retry) ? cid_next() : stopt_target(p);
        return run_need(v, SETTLE_TICKS, DEC_MARGIN_MM + ((c > d) ? c : d));
    case NX_RPM:
        c = rpmt_need_mm(p);
        return (c > 0.0f) ? c : NEED_LINE;
    default:
        return NEED_LINE;
    }
}

/* 측정 끝(still 1 = 이미 서 있다). 다음 회 자리를 정하고 순서 8 로 간다 */
static void leave(int still)
{
    int16_t free;
    float need = next_need(&free);

    tune_cross_clear();
    s_id.mv_ref = travel();
    s_id.exp = s_pos.ok ? ahead_mm() : 0.0f;
    s_id.go_dec = decel_mm(D.vin_out);
    s_id.turn = 1;
    s_id.need = need;
    if ((need <= 0.0f) || (free && s_pos.ok && (ahead_mm() - s_id.go_dec >= need))) {
        s_id.turn = 0;               /* 끝, 또는 앞이 넉넉해 같은 방향으로 잇는다 */
        s_id.need = 0.0f;
        s_id.mv = MV_POS;
    }
    else if (!s_pos.ok || (need > s_pos.len - P.guide_dist_mm - MARGIN_MM)) { s_id.mv = MV_LINE; }
    else { s_id.mv = MV_POS; }
    enter(PH_SEEK);
    if (!still) { return; }
    if ((s_id.mv == MV_POS) && (behind_mm() >= s_id.need)) { s_id.mv = MV_HALT; }
    else {
        run_restart_now();
        s_id.restart = 1;
    }
}

/* 4 틱마다 지금 지령(가속 중이면 4 틱 뒤 지령)에서 최저 속도까지 감속 거리 */
static void go_dec_update(int16_t to)
{
    if ((s_id.t++ & (TOP_DEC_EVERY - 1)) == 0) {
        s_id.go_dec = decel_mm((int16_t)(D.vin_out + ((D.vin_out < to) ? TOP_DEC_EVERY * P.acc : 0)));
    }
}

/* MV_POS: 뒤가 need 가 될 때까지 LOS, 그 뒤 최저 속도로 내려 선다. 직선 가운데라 선이 보이면 모델이 틀렸다 */
static int move_run(void)
{
    go_dec_update(P.low_speed);
    if ((tune_lines() >= 1u) || D.guide_all_on) { return fail_now(); }
    if ((behind_mm() + s_id.go_dec < s_id.need) && (ahead_mm() - s_id.go_dec > P.guide_dist_mm + MARGIN_MM)) {
        set_speed(P.low_speed, 1);
        return 0;
    }
    s_id.need = 0.0f;                /* 여기서부터 내린다(다시 올리지 않는다) */
    set_speed(P.min_speed, 1);
    if (D.vin_out <= P.min_speed) {
        tagact_stop();
        s_id.mv = MV_HALT;
        s_id.t = 0;
    }
    return 0;
}

/* MV_LINE: 선 자리를 알면 LOS 로 가다 센서가 선 APPROACH_MM 앞에서 최저 속도가 되게 내린다. 모르면 최저 속도.
 * 위치 모델보다 LINE_TOL_MM 넘게 일찍 보이거나 그만큼 지나도 안 보이면 실패(STOP 과 같이 선다) */
static int seek_run(void)
{
    int16_t v = P.min_speed;

    if (s_id.exp > 0.0f) {
        go_dec_update(P.low_speed);
        if (s_id.exp - mm_since(s_id.mv_ref) - P.guide_dist_mm - APPROACH_MM > s_id.go_dec) { v = P.low_speed; }
        else { s_id.exp = 0.0f; }   /* 내리기 시작했다 */
    }
    set_speed(v, 1);
    if (tune_lines() >= 1u) {
        if (s_pos.ok && (PASS > 0u) && (ahead_mm() - P.guide_dist_mm > LINE_TOL_MM)) { return fail_now(); }
        return cross_seen();
    }
    if ((PASS == 0u) ? (mm_since(s_id.begin) > FIND_LIMIT_MM) : (ahead_mm() - P.guide_dist_mm < -LINE_TOL_MM)) {
        return fail_now();
    }
    return 0;
}

/* MV_HALT: 서면 그 자리에서 180도 정렬 회전. 직선 테이프를 다시 잡으면 끝난다(못 잡으면 258, 실패 회 × 10 + 9).
 * turn 0 이면 돌지 않고 다음 회(또는 끝)로 */
static int halt_run(int stopped)
{
    if (D.stop_sw || D.estop || (D.system_err != 0) || (D.stop_err != 0)) { return fail_now(); }
    if (!stopped) { return (++s_id.t > STOP_LIMIT) ? fail_now() : 0; }
    add_leg(mm_since(s_id.begin));
    if (!s_id.turn) {
        s_pos.at = pos_mm();
        s_pos.ref = travel();
        enter(PH_TURN);              /* 재출발이 다음 회를 연다 */
        return turn_done();
    }
    s_pos.turn_at = pos_mm();
    D.tape_slow = 0;
    s_id.turn_seen = 0;
    tagact_turn_align_stopped(TURN_CW, 180u, 0u);
    if (!D.tape_align) { return fail_now(); }
    enter(PH_TURN);
    return 0;
}

/* 회전(또는 돌지 않고 섬)이 끝났다. 6회(최고 속도 시험은 2회) 뒤에는 그 자리에서 기다린다.
 * 통합 시험은 단계 1·2 끝에서 다음 단계로 가고 단계 3 끝에서 기다린다.
 * 지정 시험은 0회 두 번째 회전 뒤 단계 2, 단계 2 끝에서 단계 3(타력), 단계 3 끝에서 단계 4(정지 정확도)로 가고
 * 단계 4 끝에서 기다린다 */
static int turn_done(void)
{
    D.tape_slow = 0;
    if ((D.system_err != 0) || (D.stop_err != 0)) { return fail_now(); }
    if (s_build) { return build_turn(); }
    if (sweep()) {
        if (!s_id.retry) {
            agv_all.sw[agv_all.sweep_k].done++;
            if (PASS >= (uint16_t)(agv_all.sweep_n * SWEEP_PASSES)) {
                if (s_hs)        { return all_end(); }
                else if (s_temp) { coast_next(); }
                else             { all_next(); }
            }
        }
    }
    else if (coast()) {
        if (!s_id.retry && (PASS >= (stopt() ? STOPT_PASSES : COAST_PASSES))) {
            *s_r.state = 2;
            if (stopt()) { return all_end(); }
            stopt_next();
        }
    }
    else if (s_temp) {
        if (s_id.legs >= 2) {      /* 0회 끝. L 을 쟀다 */
            agv_ident.state = 2;
            all_next();
            if (s_hs && (agv_all.track_mm < agv_all.need_mm)) { return fail_now(); }   /* HS_FAIL_SHORT */
            if (agv_all.sweep_n == 0) { coast_next(); }
        }
    }
    else if (s_rpm) {
        if (PASS >= RPMT_PASSES) {
            *s_r.state = 2;
            return 1;
        }
    }
    else if (PASS >= (s_top ? TOP_PASSES : IDENT_PASSES)) {
        if (s_top) { top_finish(); }
        else       { fit_ff(); }
        *s_r.state = 2;
        if (s_all && !s_top) { all_next(); }
        else {
            if (s_all) { (void)all_end(); }
            return 1;
        }
    }
    run_restart_now();
    s_id.restart = 1;
    return 0;
}

int ident_run(void)
{
    ident_level_t *lv = cur_lv();
    int stopped = !D.flag_run && !D.flag_ready && !D.flag_stopping;
    uint16_t ph = (uint16_t)*s_r.phase;
    uint16_t i;

    if (*s_r.state != 1) { return (*s_r.state == 2) ? 1 : -1; }
    if (s_all) { agv_all.elapsed_s = (float)(++s_id.ticks) * (TICK_MS / 1000.0f); }
    if ((PASS > 0u) && !s_top && !coast() && !s_rpm) {
        if (D.risk_stop && !s_id.risk_prev) { lv->risk++; }
    }
    s_id.risk_prev = D.risk_stop;

    /* 일부러 세운 뒤 재출발을 기다린다 */
    if (s_id.restart) {
        if (!D.flag_run) { return (D.run_restart || D.flag_ready) ? 0 : fail_now(); }   /* STOP·오류가 지웠다 */
        s_id.restart = 0;
        if (ph == PH_SEEK) { return 0; }                          /* 회 사이 이동을 잇는다 */
        s_id.begin = travel();
        if ((PASS == 0u) && (s_id.legs == 1)) {                   /* L 을 잰다. 알려진 L 이 있으면 LOS 로 간다 */
            float hint = (s_len_seen > 0.0f) ? s_len_seen : (float)s_hint;
            s_bld.l0 = run_pulses(0);
            s_bld.r0 = run_pulses(1);
            seek_begin();
            s_id.mv_ref = s_id.begin;
            s_id.exp = hint * (1.0f - HINT_TOL);
        }
        else if (s_top)                       { top_begin(); }
        else if (s_rpm)                       { rpm_begin(); }
        else if (stopt())                     { stopt_begin(); }
        else if (coast())                     { coast_begin(); }
        else                                  { pass_begin(); }
        return 0;
    }
    if (s_id.rstop) {                /* 바닥 rpm: 끊은 회 뒤 보통 정지 */
        if (!stopped) { return (++s_id.t > STOP_LIMIT) ? fail_now() : 0; }
        s_id.rstop = 0;
        leave(1);
        return 0;
    }
    if (ph == PH_TURN) {
        if (D.flag_turn) { s_id.turn_seen = 1; }
        else if (s_id.turn_seen && !D.tape_align && !D.flag_stopping && (D.turn_motion == 0)) {
            s_pos.at = s_pos.turn_at;
            s_pos.ref = travel();
            s_pos.dir = (int16_t)-s_pos.dir;
            return turn_done();
        }
        return 0;
    }
    if (s_bld.ga) { return bld_ga_wait(stopped); }               /* 선 위에서 선다 */
    if ((ph == PH_SEEK) && (s_id.mv == MV_HALT)) { return halt_run(stopped); }
    if (s_rpm && (ph < PH_SEEK)) { return rpm_run(stopped); }   /* 교차선은 rpm_run 이 본다 */
    if ((ph <= PH_STOP) && D.guide_all_on) { return fail_now(); }   /* 측정 중 교차선 */
    if (coast() && (ph <= PH_COAST)) { return coast_run(stopped); }
    if (ph == PH_STOP) {
        if (stopped) {
            lv->stop_mm = mm_since(s_id.start);
            lv->stop_ms = ms(s_id.t);
            lv->run_m = mm_since(s_id.begin) / 1000.0f;
            agv_ident.stop_floor_rpm = stop_floor_rpm();
            leave(1);
        }
        else if (++s_id.t > STOP_LIMIT) { return fail_now(); }
        return 0;
    }
    if (stopped || D.flag_stopping) { return fail_now(); }
    if (ph == PH_SEEK) {
        if ((PASS > 0u) && !coast() && !s_rpm && (D.step == DRIVE_CHANGE_SPEED)) {   /* 감속 중 실제 - 지령 */
            float lag = ((float)(D.rpm_avg[0] + D.rpm_avg[1]) * 0.5f - (float)D.vin_out) / P.one_meter_rpm;
            float *dl = s_top ? &agv_top.p[PASS - 1u].dec_lag_mpm : &lv->dec_lag_mpm;
            if (lag > *dl) { *dl = lag; }
        }
        return (s_id.mv == MV_POS) ? move_run() : seek_run();
    }
    if (s_top) { return top_run(); }
    if (mm_since(s_id.begin) > s_id.budget) {
        if (ph == PH_RAMP) { return slow_down(); }
        lv->cut |= 1;
        sl_stop();
        return 0;
    }

    switch (ph) {
    case PH_RAMP:
        set_speed(pass_rpm(), 1);
        if (s_build) { acc_ramp(); }
        if (D.step == DRIVE_CHANGE_SPEED) {
            float lag = ((float)D.vin_out - (float)(D.rpm_avg[0] + D.rpm_avg[1]) * 0.5f) / P.one_meter_rpm;
            if (lag > lv->ramp_lag_mpm) { lv->ramp_lag_mpm = lag; }
        }
        if ((D.step == DRIVE_CRUISE) && (D.vin_out == s_id.want)) {
            if (s_build) { acc_judge(); }
            enter(PH_SETTLE);
            if (!plan(lv)) { return slow_down(); }
        }
        else if (++s_id.t > RAMP_LIMIT) { return fail_now(); }
        break;
    case PH_SETTLE:
        if (++s_id.t >= SETTLE_TICKS) {
            hold_clear();
            enter(PH_HOLD);
        }
        break;
    case PH_HOLD:
        hold_sample();
        if (++s_id.t >= s_id.hold_n) {
            hold_finish(lv);
            if (spd_pass()) {
                float d = SPD_STEP_MPM * P.one_meter_rpm;
                for (i = 0; i < 2u; i++) { step_begin(&s_id.spd[i], (float)D.rpm_flt[i], d, SPD_TICKS); }
                set_speed((int16_t)(s_id.want + (int16_t)d), 0);
                enter(PH_SPD_UP);
            }
            else { lat_begin(lv); }
        }
        break;
    case PH_LAT:
        step_sample(&s_id.lat, lateral());
        if (++s_id.t >= s_id.lat_n) {
            lat_finish(lv);
            sl_stop();
        }
        break;
    case PH_SPD_UP:
    case PH_SPD_DOWN:
        for (i = 0; i < 2u; i++) { step_sample(&s_id.spd[i], (float)D.rpm[i]); }
        if (++s_id.t < SPD_TICKS) { break; }
        if (ph == PH_SPD_UP) {
            for (i = 0; i < 2u; i++) {
                agv_ident.spd_delay_ms[i] = ms(s_id.spd[i].t10);
                agv_ident.spd_t63_ms[i] = ms(s_id.spd[i].t63);
                agv_ident.spd_over_pct[i] = over_pct(&s_id.spd[i]);
                step_begin(&s_id.spd[i], (float)D.rpm_flt[i], -s_id.spd[i].step, SPD_TICKS);
            }
            set_speed(pass_rpm(), 0);
            enter(PH_SPD_DOWN);
        }
        else {
            for (i = 0; i < 2u; i++) { agv_ident.spd_down_t63_ms[i] = ms(s_id.spd[i].t63); }
            lat_begin(lv);
        }
        break;
    default:
        break;
    }
    return 0;
}
