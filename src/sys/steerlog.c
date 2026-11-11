/* steerlog.c - 조향 기록기. 설계: include/agv/steerlog.h. 주행 상태를 읽기만 하고 바꾸지 않는다 */
#include "agv/steerlog.h"
#if AGV_DEV_TESTS
#include <stddef.h>
#include <string.h>
#include "agv/drive.h"
#include "agv/guide.h"
#include "agv/param.h"
#include "agv/run.h"

#define D agv_drive
#define P param_dv

#define JUMP_UNITS      6
#define STEER_LIMIT_MAX 1500      /* steer.c STEER_LIMIT_RPM 과 같다 */
#define SPAN_NORMAL     3         /* steer.c GUIDE_SPAN_NORMAL 과 같다 */

/* .ebss(RAML1)·RAMH0 에 자리가 모자라 L0 데이터 구역(cmd 의 RAML0D)에 둔다. 부팅 때 steerlog_reset 이 지운다 */
#ifdef __TMS320C2000__
#pragma DATA_SECTION(agv_steerlog, "steerlog_ram")
#endif
steerlog_t agv_steerlog;

#ifdef __TMS320C2000__
#pragma DATA_SECTION(s_sl, "steerlog_ram")
#endif
static struct {
    float    dist;                  /* 출발 뒤 mm */
    uint32_t last_p[2];
    uint32_t prev_led;              /* 직전 틱(응답)의 값 */
    int16_t  prev_pos, prev_trk, prev_on, prev_off, prev_risk, prev_at;
    int16_t  prev_v;                /* 직전 틱 v_cmd. 이번 틱 steer_run 이 상한에 쓴 값 */
    int16_t  active;
} s_sl;

static uint32_t led_now(void) { return D.guide_wide ? D.guide_led32 : (uint32_t)D.guide_led; }

static int16_t clip8(int16_t v) { return (v > 127) ? 127 : ((v < -128) ? -128 : v); }

static int16_t curve_now(void)       /* steer.c curve_detected 와 같은 식 */
{
    uint16_t v = D.guide_led;
    int16_t n = 0;
    if (guide_source_led()) { return D.guide_span >= guide_curve_span(); }
    while (v != 0u) { n = (int16_t)(n + (v & 1u)); v >>= 1; }
    return n > SPAN_NORMAL;
}

static void remember(void)
{
    s_sl.prev_led = led_now();
    s_sl.prev_pos = D.guide_pos;
    s_sl.prev_trk = D.guide_track;
    s_sl.prev_on = D.guide_all_on;
    s_sl.prev_off = D.guide_all_off;
    s_sl.prev_risk = D.risk_stop;
    s_sl.prev_v = D.v_cmd;
}

void steerlog_reset(void)
{
    memset(&agv_steerlog, 0, sizeof(agv_steerlog));
    memset(&s_sl, 0, sizeof(s_sl));
}

void steerlog_event(int16_t kind)
{
    steerlog_ev_t *e;
    uint32_t led = led_now();

    if (!s_sl.active) { return; }
    if (agv_steerlog.ev_n < 0xFFFFu) { agv_steerlog.ev_n++; }
    if (agv_steerlog.ev_n > STEERLOG_EV) { return; }
    e = &agv_steerlog.ev[agv_steerlog.ev_n - 1u];
    e->kind = kind;
    e->trk = (int16_t)((D.guide_track & 3) | ((s_sl.prev_trk & 3) << 4));
    e->dist_mm = (int32_t)s_sl.dist;
    e->prev_pos = s_sl.prev_pos;
    e->pos = D.guide_pos;
    e->prev_lo = (uint16_t)(s_sl.prev_led & 0xFFFFu);
    e->prev_hi = (uint16_t)(s_sl.prev_led >> 16);
    e->lo = (uint16_t)(led & 0xFFFFu);
    e->hi = (uint16_t)(led >> 16);
    e->vin_out = D.vin_out;
    e->p_pi_out = (int16_t)D.p_pi_out;
    e->tape_w = (int16_t)guide_tape_width();
    e->tick = (uint16_t)agv_steerlog.total;
}

static void start(void)
{
    steerlog_cfg_t *c = &agv_steerlog.cfg;
    uint16_t runs = agv_steerlog.runs;

    memset(&agv_steerlog, 0, offsetof(steerlog_t, s));   /* 표본은 n 으로 지운 셈이다 */
    agv_steerlog.runs = (uint16_t)(runs + 1u);
    c->straight_len = P.straight_len;
    c->auto_drive_mode = P.auto_drive_mode;
    c->auto_slow_mode = P.auto_slow_mode;
    c->tag_pulse2 = P.tag_pulse[2];
    c->tag_pulse10 = P.tag_pulse[10];
    c->los = P.low_speed;
    c->drs = P.target_speed;
    c->acc = P.acc;
    c->dcc = P.dcc;
    c->obs_use = D.obs_use;
    c->speed_high = param_get(P_SPEED_HIGH);
    c->tst = param_get(P_DEV_TEST);
    s_sl.dist = 0.0f;
    s_sl.prev_at = 0;
    s_sl.active = 1;
}

void steerlog_tick(void)
{
    uint32_t p0 = run_pulses(0), p1 = run_pulses(1);
    steerlog_smp_t *s;
    int16_t p, lim, at, step;
    int16_t on = (int16_t)(D.guide_all_on != 0), off = (int16_t)(D.guide_all_off != 0);

    if ((D.control != RUN_DRIVE) || (D.vin_out == 0)) { s_sl.active = 0; }
    else {
        if (!s_sl.active) { start(); }
        else if (P.one_length_pulse > 0.0f) {
            s_sl.dist += 0.5f * (float)((p0 - s_sl.last_p[0]) + (p1 - s_sl.last_p[1])) / P.one_length_pulse;
        }
        /* 사건. 직전 값은 지난 틱 끝의 것이다(응답은 틱마다 많아야 하나) */
        if (D.risk_stop && !s_sl.prev_risk) { steerlog_event(STEERLOG_EV_RISK); }
        lim = (int16_t)(((s_sl.prev_v >= 0) ? s_sl.prev_v : -s_sl.prev_v) * 2);
        if (lim > STEER_LIMIT_MAX) { lim = STEER_LIMIT_MAX; }
        at = (int16_t)((lim > 0) && ((D.p_pi_out >= (float)lim) || (D.p_pi_out <= (float)-lim)));
        if (at && !s_sl.prev_at) { steerlog_event(STEERLOG_EV_LIMIT); }
        s_sl.prev_at = at;
        if (!on && !off && !s_sl.prev_on && !s_sl.prev_off
            && ((D.guide_pos - s_sl.prev_pos >= JUMP_UNITS) || (s_sl.prev_pos - D.guide_pos >= JUMP_UNITS))) {
            steerlog_event(STEERLOG_EV_JUMP);
        }
        if (on && !s_sl.prev_on)   { steerlog_event(STEERLOG_EV_ALL_ON); }
        if (off && !s_sl.prev_off) { steerlog_event(STEERLOG_EV_LOST); }

        s = &agv_steerlog.s[agv_steerlog.w];
        p = (int16_t)D.p_pi_out;
        step = (int16_t)(D.step - DRIVE_CHANGE_SPEED);
        if ((step < 0) || (step > 5)) { step = 7; }
        s->pf = (uint16_t)(((uint16_t)clip8(D.guide_pos) & 0xFFu) | ((uint16_t)step << 8)
                           | ((uint16_t)(D.guide_track & 3) << 11) | ((uint16_t)on << 13) | ((uint16_t)off << 14)
                           | ((uint16_t)curve_now() << 15));
        s->p2 = (int16_t)(p * 2 + (D.risk_stop ? 1 : 0));
        s->io = (int16_t)(uint16_t)(((uint16_t)clip8(D.offset_kp) << 8) | ((uint16_t)clip8((int16_t)(steer_i_out() / 16.0f)) & 0xFFu));
        s->v_cmd = D.v_cmd;
        s->rpm[0] = D.rpm[0];
        s->rpm[1] = D.rpm[1];
        agv_steerlog.w = (uint16_t)((agv_steerlog.w + 1u < STEERLOG_N) ? (agv_steerlog.w + 1u) : 0u);
        if (agv_steerlog.n < STEERLOG_N) { agv_steerlog.n++; }
        agv_steerlog.total++;
    }
    s_sl.last_p[0] = p0;
    s_sl.last_p[1] = p1;
    remember();
}
#endif /* AGV_DEV_TESTS */
