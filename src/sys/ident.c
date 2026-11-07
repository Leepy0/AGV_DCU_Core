/* ident.c - 식별 시험. 속도 세 단계(최고 속도의 1/3, 2/3, 1)마다
 *  가속 → 안정 1 s → 정속 3 s 측정 → 옆 계단 2.5 s → 복귀 2.5 s
 * 1단계에서는 속도 계단을 더 재고, 마지막 단계 뒤 SL 정지를 잰다.
 * 옆 계단은 조향 지령(p_test)을 4칸 옮겨 폐루프 응답을 본다. 속도 계단은 램프 없이 지령을 바꾼다 */
#include <math.h>
#include <string.h>
#include "agv/ident.h"
#include "agv/drive.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/stop.h"
#include "agv/tagact.h"

#define D agv_drive
#define P param_dv

#define TICK_MS          10.0f
#define SETTLE_TICKS     100
#define HOLD_TICKS       300
#define LAT_TICKS        250
#define SPD_TICKS        200
#define END_TICKS        50       /* 창 끝 평균 */
#define RAMP_LIMIT       2000     /* 가속이 20 s 안에 안 끝나면 실패 */
#define STOP_LIMIT       3000
#define LAT_STEP_BITS    4
#define SPD_STEP_MPM     3.0f
#define GUIDE_MM         6.89f
#define BAND             0.1f     /* 정착 판정 ±10% */

enum { PH_RAMP = 1, PH_SETTLE, PH_HOLD, PH_LAT, PH_LAT_BACK, PH_SPD_UP, PH_SPD_DOWN, PH_STOP };

ident_result_t agv_ident;

typedef struct {
    float   y0, step, peak, end_sum;
    int16_t t, t10, t63, t90, t_out, window;
} step_t;

static struct {
    uint16_t level, phase;
    int16_t  t, want, risk_prev;
    uint32_t start, begin;
    step_t   lat, spd[2];
    float    y[HOLD_TICKS];
    float    steer_ss, rpm_s[2], rpm_ss[2], tq_s[2];
} s_id;

static uint32_t travel(void) { return run_pulses(D.backward ? 1u : 0u); }

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
}

static void step_sample(step_t *s, float y)
{
    float r = (s->step != 0.0f) ? (y - s->y0) / s->step : 0.0f;

    if ((s->t10 < 0) && (r >= 0.1f))   { s->t10 = s->t; }
    if ((s->t63 < 0) && (r >= 0.632f)) { s->t63 = s->t; }
    if ((s->t90 < 0) && (r >= 0.9f))   { s->t90 = s->t; }
    if (r > s->peak) { s->peak = r; }
    if (fabsf(r - 1.0f) > BAND) { s->t_out = s->t; }
    if (s->t >= s->window - END_TICKS) { s->end_sum += r; }
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

static int16_t level_rpm(uint16_t k)
{
    int16_t v = (int16_t)(((int32_t)P.target_speed * (int32_t)(k + 1u)) / IDENT_LEVELS);
    return (v < P.min_speed) ? P.min_speed : v;
}

static void enter(uint16_t phase)
{
    s_id.phase = phase;
    s_id.t = 0;
}

void ident_start(void)
{
    memset(&agv_ident, 0, sizeof(agv_ident));
    agv_ident.state = 1;
    agv_ident.cfg_high = param_get(P_SPEED_HIGH);
    agv_ident.cfg_kp = param_get(P_STEER_KP);
    agv_ident.cfg_ki = param_get(P_STEER_KI);
    agv_ident.cfg_opt = param_get(P_OPT_FLAGS);
    agv_ident.cfg_ff = param_get(P_SPEED_FF);
    s_id.level = 0;
    s_id.risk_prev = D.risk_stop;
    s_id.begin = travel();
    D.p_test = 0;
    enter(PH_RAMP);
}

void ident_abort(void)
{
    D.p_test = 0;
    if (agv_ident.state == 1) {
        agv_ident.fail = (int16_t)((s_id.level + 1u) * 10u + s_id.phase);
        agv_ident.state = 3;
    }
}

uint16_t ident_level(void) { return (uint16_t)(s_id.level + 1u); }
uint16_t ident_phase(void) { return s_id.phase; }

static void hold_clear(void)
{
    uint16_t i;
    s_id.steer_ss = 0.0f;
    for (i = 0; i < 2u; i++) { s_id.rpm_s[i] = s_id.rpm_ss[i] = s_id.tq_s[i] = 0.0f; }
}

static void hold_sample(void)
{
    uint16_t i;
    s_id.y[s_id.t] = lateral();
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
    const float n = (float)HOLD_TICKS;
    float sum = 0.0f, ss = 0.0f, lo, hi, mean, rms, hys;
    int16_t i, side = 0, cross = 0, first = -1, last = -1;

    lo = hi = s_id.y[0];
    for (i = 0; i < HOLD_TICKS; i++) {
        float y = s_id.y[i];
        sum += y;
        if (y < lo) { lo = y; }
        if (y > hi) { hi = y; }
    }
    mean = sum / n;
    for (i = 0; i < HOLD_TICKS; i++) { ss += (s_id.y[i] - mean) * (s_id.y[i] - mean); }
    rms = sqrtf(ss / n);
    hys = (rms * 0.5f > 0.3f) ? rms * 0.5f : 0.3f;
    for (i = 0; i < HOLD_TICKS; i++) {
        float d = s_id.y[i] - mean;
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
    lv->lat_err_mm = (s->end_sum / (float)END_TICKS - 1.0f) * s->step * GUIDE_MM;
}

/* 정속 토크 = 마찰 + 기울기 × rpm. 기울기 × 1000 이 52번 단위다 */
static void fit_ff(void)
{
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f, n = (float)IDENT_LEVELS, den;
    uint16_t k;

    for (k = 0; k < IDENT_LEVELS; k++) {
        float x = (agv_ident.lv[k].rpm_l + agv_ident.lv[k].rpm_r) * 0.5f;
        float y = (agv_ident.lv[k].tq_l + agv_ident.lv[k].tq_r) * 0.5f;
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    den = n * sxx - sx * sx;
    if (den <= 0.0f) { return; }
    agv_ident.ff_suggest = ((n * sxy - sx * sy) / den) * 1000.0f;
    agv_ident.tq_offset = (sy - (agv_ident.ff_suggest / 1000.0f) * sx) / n;
}

static int fail_now(void)
{
    ident_abort();
    if (D.flag_run) { tagact_stop(); }
    return -1;
}

int ident_run(void)
{
    ident_level_t *lv = &agv_ident.lv[s_id.level];
    int stopped = !D.flag_run && !D.flag_ready && !D.flag_stopping;
    uint16_t i;

    if (agv_ident.state != 1) { return (agv_ident.state == 2) ? 1 : -1; }
    if (D.risk_stop && !s_id.risk_prev) { lv->risk++; }
    s_id.risk_prev = D.risk_stop;

    if (s_id.phase == PH_STOP) {
        if (stopped) {
            agv_ident.stop_mm = (float)(travel() - s_id.start) / P.one_length_pulse;
            agv_ident.stop_ms = ms(s_id.t);
            agv_ident.stop_floor_rpm = stop_floor_rpm();
            agv_ident.total_m = (float)(travel() - s_id.begin) / P.one_length_pulse / 1000.0f;
            agv_ident.state = 2;
            return 1;
        }
        if (++s_id.t > STOP_LIMIT) { return fail_now(); }
        return 0;
    }
    if (stopped || D.flag_stopping) { return fail_now(); }

    switch (s_id.phase) {
    case PH_RAMP:
        set_speed(level_rpm(s_id.level), 1);
        if (D.step == DRIVE_CHANGE_SPEED) {
            float lag = ((float)D.vin_out - (float)(D.rpm_avg[0] + D.rpm_avg[1]) * 0.5f) / P.one_meter_rpm;
            if (lag > lv->ramp_lag_mpm) { lv->ramp_lag_mpm = lag; }
        }
        if ((D.step == DRIVE_CRUISE) && (D.vin_out == s_id.want)) { enter(PH_SETTLE); }
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
        if (++s_id.t >= HOLD_TICKS) {
            hold_finish(lv);
            D.p_test = LAT_STEP_BITS;
            step_begin(&s_id.lat, lv->dev_mean_mm / GUIDE_MM,
                       (float)LAT_STEP_BITS - lv->dev_mean_mm / GUIDE_MM, LAT_TICKS);
            enter(PH_LAT);
        }
        break;
    case PH_LAT:
        step_sample(&s_id.lat, lateral());
        if (++s_id.t >= LAT_TICKS) {
            lat_finish(lv);
            D.p_test = 0;
            enter(PH_LAT_BACK);
        }
        break;
    case PH_LAT_BACK:
        if (++s_id.t < LAT_TICKS) { break; }
        if (s_id.level == 0u) {
            float d = SPD_STEP_MPM * P.one_meter_rpm;
            for (i = 0; i < 2u; i++) { step_begin(&s_id.spd[i], (float)D.rpm_flt[i], d, SPD_TICKS); }
            set_speed((int16_t)(s_id.want + (int16_t)d), 0);
            enter(PH_SPD_UP);
        }
        else if (s_id.level + 1u < IDENT_LEVELS) {
            s_id.level++;
            enter(PH_RAMP);
        }
        else {
            fit_ff();
            agv_ident.stop_sl_mm = (float)(D.backward ? P.stop_len : P.fwd_stop_len) / P.one_length_pulse;
            s_id.start = travel();
            tagact_stop_length();
            enter(PH_STOP);
        }
        break;
    case PH_SPD_UP:
    case PH_SPD_DOWN:
        for (i = 0; i < 2u; i++) { step_sample(&s_id.spd[i], (float)D.rpm[i]); }
        if (++s_id.t < SPD_TICKS) { break; }
        if (s_id.phase == PH_SPD_UP) {
            for (i = 0; i < 2u; i++) {
                agv_ident.spd_delay_ms[i] = ms(s_id.spd[i].t10);
                agv_ident.spd_t63_ms[i] = ms(s_id.spd[i].t63);
                agv_ident.spd_over_pct[i] = over_pct(&s_id.spd[i]);
                step_begin(&s_id.spd[i], (float)D.rpm_flt[i], -s_id.spd[i].step, SPD_TICKS);
            }
            set_speed(level_rpm(0), 0);
            enter(PH_SPD_DOWN);
        }
        else {
            for (i = 0; i < 2u; i++) { agv_ident.spd_down_t63_ms[i] = ms(s_id.spd[i].t63); }
            s_id.level++;
            enter(PH_RAMP);
        }
        break;
    default:
        break;
    }
    return 0;
}
