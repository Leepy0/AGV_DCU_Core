/* rpmtest.c - 최대 rpm 시험. rpmtest.h 참고.
 * DAC 1000 → 2000 → 3000 → 4000 → 4095, 단계마다 3 s. 앞 1.28 s 는 20 ms 마다 rpm 을 남겨
 * 오름 시간을 구하고, 끝 1 s 평균을 그 단계 rpm 으로 쓴다 */
#include <string.h>
#include "agv/rpmtest.h"
#include "agv/drive.h"
#include "agv/motor.h"
#include "agv/param.h"

#define D agv_drive
#define P param_dv

#define LEVEL_TICKS    300      /* 3 s */
#define WIN_TICKS      100      /* 끝 1 s */
#define RISE_EVERY     2        /* 20 ms */
#define RISE_SAMPLES   64       /* 1.28 s */
#define SPIKE_RPM      8000     /* 이보다 크면 튄 값으로 버린다 */
#define TURN_MIN_RPM   100      /* 1단계 끝에 이보다 느린 바퀴는 안 도는 것 */
#define RISE_MIN_RPM   50       /* 앞 단계와 이만큼도 안 다르면 오름 시간 0 */

static const int16_t s_dac[RPMT_LEVELS] = { 1000, 2000, 3000, 4000, MOTOR_DAC_MAX };

rpmt_result_t agv_rpmt;

static struct {
    int16_t tick, abort;
    int16_t prev[2];
    int32_t sum[2];
    int16_t n[2], lo[2], hi[2];
    int16_t rise[2][RISE_SAMPLES];
} s_rt;

static rpmt_level_t *cur(void) { return &agv_rpmt.lv[agv_rpmt.level - 1]; }

static void level_begin(void)
{
    uint16_t i;

    cur()->dac = s_dac[agv_rpmt.level - 1];
    s_rt.tick = 0;
    for (i = 0; i < 2u; i++) {
        s_rt.sum[i] = 0;
        s_rt.n[i] = 0;
        s_rt.lo[i] = 32767;
        s_rt.hi[i] = 0;
    }
    motor_dac_force(cur()->dac, cur()->dac);
}

void rpmt_start(void)
{
    memset(&agv_rpmt, 0, sizeof(agv_rpmt));
    memset(&s_rt, 0, sizeof(s_rt));
    agv_rpmt.state = 1;
    agv_rpmt.level = 1;
    level_begin();
}

/* 앞 단계 rpm 에서 이번 rpm 까지 90% 오른 첫 표본 */
static int16_t rise_ms(uint16_t i, int16_t m)
{
    int16_t from = s_rt.prev[i];
    int16_t th = (int16_t)(from + (int16_t)(((int32_t)(m - from) * 9L) / 10L));
    uint16_t k;

    if ((m - from < RISE_MIN_RPM) && (from - m < RISE_MIN_RPM)) { return 0; }
    for (k = 0; k < RISE_SAMPLES; k++) {
        int16_t v = s_rt.rise[i][k];
        if ((m > from) ? (v >= th) : (v <= th)) { return (int16_t)((k + 1u) * RISE_EVERY * 10u); }
    }
    return -1;
}

static void level_end(void)
{
    rpmt_level_t *l = cur();
    uint16_t i;

    for (i = 0; i < 2u; i++) {
        int16_t m = (s_rt.n[i] > 0) ? (int16_t)(s_rt.sum[i] / s_rt.n[i]) : 0;
        l->rpm[i] = m;
        l->ripple[i] = (s_rt.n[i] > 0) ? (int16_t)(s_rt.hi[i] - s_rt.lo[i]) : 0;
        l->spikes[i] = (int16_t)(WIN_TICKS - s_rt.n[i]);
        l->t90_ms[i] = rise_ms(i, m);
        if (m > agv_rpmt.max_rpm[i]) { agv_rpmt.max_rpm[i] = m; }
        s_rt.prev[i] = m;
    }
    l->mpm = ((float)l->rpm[0] + (float)l->rpm[1]) * 0.5f / P.one_meter_rpm;
    if (l->mpm > agv_rpmt.max_mpm) { agv_rpmt.max_mpm = l->mpm; }
}

static int fail(int16_t reason)
{
    motor_dac_release();
    agv_rpmt.state = 3;
    agv_rpmt.fail = (int16_t)(reason * 10 + agv_rpmt.level);
    return -1;
}

static void sample(uint16_t i)
{
    int16_t v = motor_raw_rpm(i);
    int16_t k = (int16_t)(s_rt.tick / RISE_EVERY - 1);

    if (((s_rt.tick % RISE_EVERY) == 0) && (k < RISE_SAMPLES)) {
        int16_t last = (k > 0) ? s_rt.rise[i][k - 1] : s_rt.prev[i];
        s_rt.rise[i][k] = (v > SPIKE_RPM) ? last : v;
    }
    if ((s_rt.tick > (LEVEL_TICKS - WIN_TICKS)) && (v <= SPIKE_RPM)) {
        s_rt.sum[i] += v;
        s_rt.n[i]++;
        if (v < s_rt.lo[i]) { s_rt.lo[i] = v; }
        if (v > s_rt.hi[i]) { s_rt.hi[i] = v; }
    }
}

int rpmt_run(void)
{
    if (agv_rpmt.state != 1) { return (agv_rpmt.state == 2) ? 1 : -1; }
    if (s_rt.abort != 0)                { return fail(s_rt.abort); }
    if (D.stop_sw || D.flag_stopping)   { return fail(1); }
    if (D.estop || (D.system_err != 0)) { return fail(2); }

    s_rt.tick++;
    sample(0);
    sample(1);
    if (s_rt.tick < LEVEL_TICKS) { return 0; }

    level_end();
    if ((agv_rpmt.level == 1) && ((cur()->rpm[0] < TURN_MIN_RPM) || (cur()->rpm[1] < TURN_MIN_RPM))) {
        return fail(3);
    }
    if (agv_rpmt.level >= RPMT_LEVELS) {
        motor_dac_release();
        agv_rpmt.state = 2;
        return 1;
    }
    agv_rpmt.level++;
    level_begin();
    return 0;
}

void rpmt_abort(int16_t reason)
{
    if (agv_rpmt.state != 1) { return; }
    s_rt.abort = reason;
    motor_dac_release();
}

uint16_t rpmt_level(void) { return (uint16_t)agv_rpmt.level; }
