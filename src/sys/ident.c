/* ident.c - 식별 시험. 교차선 X, Y 사이를 오가며 잰다. 순서 번호(FND, 실패 코드):
 *  1 가속, 2 안정 0.5 s, 3 정속 측정, 4·5 속도 계단 올림·내림(1회만), 6 옆 계단, 7 SL 정지,
 *  8 최저 속도로 선 찾기, 9 정렬 180도 회전 뒤 재출발
 * 0회는 8 → 9 → 8(L 측정) → 9. 1~6회는 회전 중심이 선 위에서 출발한다.
 *  짝수 회(2, 4, 6): 1 → 7 SL 정지 → 8 → 9. 센서가 먼 선 300 mm 앞에서 서도록 SL 정지 시작점을 둔다.
 *  홀수 회(1, 3, 5): 1 → 8. 서지 않고 감속하며 선을 찾는다. 선 100 mm 앞에서 최저 속도가 되게 감속을 건다.
 * 정속·옆 계단 창은 남은 거리로 정한다.
 * 옆 계단은 조향 지령(p_test)을 4칸 옮겨 폐루프 응답을 본다. 속도 계단은 램프 없이 지령을 바꾼다.
 * 실패 코드 = 회 × 10 + 순서. 08 은 0회에 선을 못 찾았거나 L < 2 m (track_mm 가 0 이 아니면 후자)
 * 최고 속도 시험(TOP)은 같은 0회·선 찾기·회전을 쓰고 1·2회만 다르다: 1 가속, 3 최고 속도 정속, 8, 9.
 * 결과는 agv_top 에 쌓는다 */
#include <math.h>
#include <string.h>
#include "agv/ident.h"
#include "agv/drive.h"
#include "agv/motor.h"
#include "agv/param.h"
#include "agv/run.h"
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
#define SEEK_OVER_MM     1000.0f  /* 1~6회 선 찾기는 L 보다 이만큼 더 가면 실패 */
#define TRACK_MIN_MM     2000.0f
#define MARGIN_MM        300.0f   /* 센서가 먼 선 앞에 남길 거리 */
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

enum { PH_RAMP = 1, PH_SETTLE, PH_HOLD, PH_SPD_UP, PH_SPD_DOWN, PH_LAT, PH_STOP, PH_SEEK, PH_TURN };

ident_result_t agv_ident;
top_result_t   agv_top;

/* 지금 시험의 진행 칸. 식별 시험이면 agv_ident, 최고 속도 시험이면 agv_top */
static struct { int16_t *state, *fail, *pass, *phase; float *track; } s_r = {
    &agv_ident.state, &agv_ident.fail, &agv_ident.pass, &agv_ident.phase, &agv_ident.track_mm
};
static int16_t s_top;

static struct {
    int16_t ri, rn, last[2], n_tq, n_lag;
    float   dec;
} s_tp;

typedef struct {
    float   y0, step, peak, end_sum;
    int16_t t, t10, t63, t90, t_out, window, end;
} step_t;

static struct {
    int16_t  t, want, risk_prev, hold_n, lat_n, restart, turn_seen, legs;
    int16_t  cap[IDENT_LEVELS], retry;   /* 직선이 짧으면 그 단계 속도를 낮춰 같은 회를 다시 한다 */
    uint32_t start, begin;
    float    budget;
    step_t   lat, spd[2];
    union { float y[WIN_TICKS]; int16_t ring[2][TOP_RING]; } u;   /* ring: 최고 속도 시험 0.3 s 원시 rpm */
    float    steer_ss, rpm_s[2], rpm_ss[2], tq_s[2];
} s_id;

#define PASS ((uint16_t)*s_r.pass)

static uint32_t travel(void) { return run_pulses(D.backward ? 1u : 0u); }
static float mm_since(uint32_t from) { return (float)(travel() - from) / P.one_length_pulse; }

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

/* 1, 2회 → 1/3, 3, 4회 → 2/3, 5, 6회 → 최고 속도 */
static int16_t level_rpm(void)
{
    int16_t v = (int16_t)(((int32_t)P.target_speed * (int32_t)((PASS - 1u) / 2u + 1u)) / IDENT_LEVELS);
    return (v < P.min_speed) ? P.min_speed : v;
}

static int16_t pass_rpm(void)
{
    int16_t v = level_rpm();
    int16_t c = s_id.cap[(PASS - 1u) / 2u];
    return (v > c) ? c : v;
}

static void enter(uint16_t phase)
{
    *s_r.phase = (int16_t)phase;
    s_id.t = 0;
}

static void seek_begin(void)
{
    tune_cross_clear();
    enter(PH_SEEK);
}

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
    D.p_test = 0;
    seek_begin();
}

void ident_start(void)
{
    s_top = 0;
    s_r.state = &agv_ident.state;
    s_r.fail = &agv_ident.fail;
    s_r.pass = &agv_ident.pass;
    s_r.phase = &agv_ident.phase;
    s_r.track = &agv_ident.track_mm;
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
    s_top = 1;
    s_r.state = &agv_top.state;
    s_r.fail = &agv_top.fail;
    s_r.pass = &agv_top.pass;
    s_r.phase = &agv_top.phase;
    s_r.track = &agv_top.track_mm;
    memset(&agv_top, 0, sizeof(agv_top));
    agv_top.cfg_acc = P.acc;
    agv_top.cfg_dcc = P.dcc;
    start_common();
}

int ident_top_mode(void) { return s_top; }

void ident_abort(void)
{
    D.p_test = 0;
    motor_spike_set(0);
    if (*s_r.state == 1) {
        *s_r.fail = (int16_t)(*s_r.pass * 10 + *s_r.phase);
        *s_r.state = 3;
    }
}

uint16_t ident_pass(void)  { return (uint16_t)*s_r.pass; }
uint16_t ident_phase(void) { return (uint16_t)*s_r.phase; }

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

static int sl_pass(void) { return (PASS & 1u) == 0u; }

/* 회전 중심이 선 위에서 출발한다. SL 정지나 감속은 늦어도 budget 에서 시작해야 센서가 먼 선 앞에 선다 */
static void pass_begin(void)
{
    float sl;

    if (s_id.retry) {
        s_id.retry = 0;
        memset(&agv_ident.lv[PASS - 1u], 0, sizeof(ident_level_t));
        agv_ident.lv[PASS - 1u].cut = 2;
    }
    else { agv_ident.pass++; }
    if (pass_rpm() < level_rpm()) { agv_ident.lv[PASS - 1u].cut |= 2; }   /* 낮춘 속도로 잰다 */
    if (sl_pass()) {
        sl = (float)(D.backward ? P.stop_len : P.fwd_stop_len) / P.one_length_pulse;
        s_id.budget = agv_ident.track_mm - P.guide_dist_mm - MARGIN_MM - sl;
    }
    else {
        s_id.budget = agv_ident.track_mm - P.guide_dist_mm - DEC_MARGIN_MM - decel_mm(pass_rpm());
    }
    D.p_test = 0;
    enter(PH_RAMP);
}

/* 남은 거리로 정속·옆 계단 창을 나눈다. 0 = 창이 너무 짧다 */
static int plan(ident_level_t *lv)
{
    const float k = 1000.0f / 6000.0f;                                       /* m/min → mm/틱 */
    float v = (float)s_id.want / P.one_meter_rpm * k;
    float extra = (PASS == 1u) ? SPD_STEP_MPM * k * (float)SPD_TICKS : 0.0f;  /* 속도 계단 동안 더 가는 거리 */
    float rem = (s_id.budget - mm_since(s_id.begin) - extra) / v * PLAN_SLACK
              - (float)(SETTLE_TICKS + ((PASS == 1u) ? 2 * SPD_TICKS : 0));
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
    step_begin(&s_id.lat, lv->dev_mean_mm / GUIDE_MM,
               (float)LAT_STEP_BITS - lv->dev_mean_mm / GUIDE_MM, s_id.lat_n);
    enter(PH_LAT);
}

/* 측정 끝. 짝수 회는 SL 정지, 홀수 회는 최저 속도로 감속하며 바로 선을 찾는다('h' 응답을 켠다) */
static void sl_stop(void)
{
    D.p_test = 0;
    if (!sl_pass()) {
        agv_ident.lv[PASS - 1u].run_m = mm_since(s_id.begin) / 1000.0f;
        set_speed(P.min_speed, 1);
        tune_cross_clear();
        enter(PH_SEEK);
        return;
    }
    agv_ident.stop_sl_mm = (float)(D.backward ? P.stop_len : P.fwd_stop_len) / P.one_length_pulse;
    s_id.start = travel();
    tagact_stop_length();
    enter(PH_STOP);
}

static int fail_now(void)
{
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
    s_id.cap[(PASS - 1u) / 2u] = (c < P.min_speed) ? P.min_speed : c;
    s_id.retry = 1;
    sl_stop();
    return 0;
}

/* ---- 최고 속도 시험 1·2회 ---- */

/* 회전 중심이 선 위에서 출발한다. 감속 거리를 뺀 남은 거리가 0 이 되면 감속한다 */
static void top_begin(void)
{
    agv_top.pass++;
    memset(&s_tp, 0, sizeof(s_tp));
    s_id.budget = agv_top.track_mm - P.guide_dist_mm - DEC_MARGIN_MM;
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
    tune_cross_clear();
    enter(PH_SEEK);
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

/* 선을 봤다. 회전 중심이 선 위에 오도록 GD 만큼 더 가서 돈다 */
static int cross_seen(void)
{
    float leg = (float)(tune_line_at(0) - s_id.begin) / P.one_length_pulse + P.guide_dist_mm;

    if (!s_top) { agv_ident.total_m += leg / 1000.0f; }
    if ((PASS == 0u) && (s_id.legs == 1)) {
        *s_r.track = leg;
        if (leg < TRACK_MIN_MM) { return fail_now(); }
    }
    s_id.legs++;
    D.tape_slow = 1;
    s_id.turn_seen = 0;
    tagact_turn_align_cross(TURN_CW, 180u, 0u);
    if (!D.tape_align) { return fail_now(); }
    enter(PH_TURN);
    return 0;
}

/* 회전이 끝났다. 6회(최고 속도 시험은 2회) 뒤에는 선 위에서 기다린다 */
static int turn_done(void)
{
    D.tape_slow = 0;
    if ((D.system_err != 0) || (D.stop_err != 0)) { return fail_now(); }
    if (PASS >= (s_top ? TOP_PASSES : IDENT_PASSES)) {
        if (s_top) { top_finish(); }
        else       { fit_ff(); }
        *s_r.state = 2;
        return 1;
    }
    run_restart_now();
    s_id.restart = 1;
    return 0;
}

int ident_run(void)
{
    ident_level_t *lv = &agv_ident.lv[(PASS > 0u) ? PASS - 1u : 0u];
    int stopped = !D.flag_run && !D.flag_ready && !D.flag_stopping;
    uint16_t ph = (uint16_t)*s_r.phase;
    uint16_t i;

    if (*s_r.state != 1) { return (*s_r.state == 2) ? 1 : -1; }
    if ((PASS > 0u) && !s_top) {
        if (D.risk_stop && !s_id.risk_prev) { lv->risk++; }
    }
    s_id.risk_prev = D.risk_stop;

    /* 일부러 세운 뒤 재출발을 기다린다 */
    if (s_id.restart) {
        if (!D.flag_run) { return (D.run_restart || D.flag_ready) ? 0 : fail_now(); }   /* STOP·오류가 지웠다 */
        s_id.restart = 0;
        if (ph == PH_STOP) { seek_begin(); return 0; }
        s_id.begin = travel();
        if ((PASS == 0u) && (s_id.legs == 1)) { seek_begin(); }   /* L 을 잰다 */
        else if (s_top)                       { top_begin(); }
        else                                  { pass_begin(); }
        return 0;
    }
    if (ph == PH_TURN) {
        if (D.flag_turn) { s_id.turn_seen = 1; }
        else if (s_id.turn_seen && !D.tape_align && !D.flag_stopping && (D.turn_motion == 0)) { return turn_done(); }
        return 0;
    }
    if ((ph <= PH_STOP) && D.guide_all_on) { return fail_now(); }   /* 측정 중 교차선 */
    if (ph == PH_STOP) {
        if (stopped) {
            lv->stop_mm = mm_since(s_id.start);
            lv->stop_ms = ms(s_id.t);
            lv->run_m = mm_since(s_id.begin) / 1000.0f;
            agv_ident.stop_floor_rpm = stop_floor_rpm();
            run_restart_now();
            s_id.restart = 1;
        }
        else if (++s_id.t > STOP_LIMIT) { return fail_now(); }
        return 0;
    }
    if (stopped || D.flag_stopping) { return fail_now(); }
    if (ph == PH_SEEK) {
        if ((PASS > 0u) && (D.step == DRIVE_CHANGE_SPEED)) {   /* 감속 중 실제 - 지령 */
            float lag = ((float)(D.rpm_avg[0] + D.rpm_avg[1]) * 0.5f - (float)D.vin_out) / P.one_meter_rpm;
            float *dl = s_top ? &agv_top.p[PASS - 1u].dec_lag_mpm : &lv->dec_lag_mpm;
            if (lag > *dl) { *dl = lag; }
        }
        set_speed(P.min_speed, 1);
        if (tune_lines() >= 1u) { return cross_seen(); }
        if (mm_since(s_id.begin) > ((PASS == 0u) ? FIND_LIMIT_MM : *s_r.track + SEEK_OVER_MM)) {
            return fail_now();
        }
        return 0;
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
        if (D.step == DRIVE_CHANGE_SPEED) {
            float lag = ((float)D.vin_out - (float)(D.rpm_avg[0] + D.rpm_avg[1]) * 0.5f) / P.one_meter_rpm;
            if (lag > lv->ramp_lag_mpm) { lv->ramp_lag_mpm = lag; }
        }
        if ((D.step == DRIVE_CRUISE) && (D.vin_out == s_id.want)) {
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
            if (PASS == 1u) {
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
