/* wtest.c - 바퀴 지연 측정(t3.9 WT). 방식은 wtest.h */
#include "agv/wtest.h"
#if AGV_DEV_TESTS
#include <string.h>
#include "agv/drive.h"
#include "agv/motor.h"
#include "agv/param.h"
#include "agv/steer.h"
#include "agv/tagact.h"

#define D agv_drive

#define WT_STEADY_TICKS 50      /* 정속·편차 안 0.5 s 이어지면 측정 */
#define WT_STEADY_DEV   2       /* 측정 시작 편차 ±단위 */
#define WT_BACK_DEV     3       /* 복귀 뒤 이 안이면 정지 */
#define WT_WAIT_TICKS   800     /* 8 s 안에 정속을 못 잡으면 없음 */
#define WT_BACK_TICKS   300     /* 3 s 안에 못 돌아와도 정지(결과는 남긴다) */
#define WT_F10          0.10536f   /* ln(1/0.9): 1차 지연이 10% 에 닿는 때 ÷ τ */
#define WT_F1063        0.89464f   /* 10% → 63.2% 시간 ÷ τ */

enum { WT_OFF = 0, WT_WAIT, WT_MEAS, WT_BACK, WT_STOP };

#ifndef AGV_HOST_BUILD
#pragma DATA_SECTION(agv_wt, "gscan_ram")   /* 보존 영역(test_keep) 밖. t3.8 결과 뒤 */
#pragma DATA_SECTION(s_wt, "gscan_ram")
#endif
wt_result_t agv_wt;
static struct { int16_t t, steady; float tq, w; } s_wt;   /* tq·w: 정속 구간 두 바퀴 토크 지령·지령 rpm 합 */

#define W agv_wt

static const int16_t s_seg[WT_STEPS + 1] = { 0, WT_T_TICKS, 3 * WT_T_TICKS, 4 * WT_T_TICKS };   /* 계단 시작 틱, 끝 */

void wtest_reset(void)
{
    W.state = 0;
    W.phase = WT_OFF;
    steer_test_rpm = 0;
}

static int16_t iabs(int16_t v) { return (v < 0) ? (int16_t)-v : v; }

static void take(int16_t k)
{
    if ((k < 0) || (k >= WT_N)) { return; }
    W.rpm[0][k] = D.rpm[0];
    W.rpm[1][k] = D.rpm[1];
    if (k + 1 > W.n) { W.n = (int16_t)(k + 1); }
}

void wtest_start(void)
{
    memset(&W, 0, sizeof(W));
    s_wt.t = s_wt.steady = 0;
    s_wt.tq = s_wt.w = 0.0f;
    W.state = 1;
    W.phase = WT_WAIT;
    W.wtl = W.wtr = W.wdl = W.wdr = W.ff = TUNE_NO_DATA;
    steer_test_rpm = 0;
    tagact_speed(1);                 /* LOS */
}

/* 측정을 끊고 정지. 결과 없음 */
static void abort_wt(int16_t why)
{
    steer_test_rpm = 0;
    W.fail = why;
    W.state = 3;
    W.dev_end = D.guide_pos;
    if (D.flag_run && !D.flag_stopping) { tagact_stop(); }
    W.phase = WT_STOP;
}

/* 계단 하나: 시작 실측 y0, 지령 c 로 t10·t63(ms). 0 = 창 안에 63% 못 감 */
static int step_times(const int16_t *y, int16_t s, float y0, float c, float *t10, float *t63)
{
    int16_t len = (int16_t)(s_seg[s + 1] - s_seg[s]), j;
    int16_t k0 = (int16_t)(WT_PRE + s_seg[s] + 1);   /* t = 0 표본 */
    float span = c - y0, prev = 0.0f;
    int got10 = 0;

    if ((span < (float)WT_A_RPM * 0.5f) && (span > -(float)WT_A_RPM * 0.5f)) { return 0; }
    for (j = 1; j <= len; j++) {
        float r = ((float)y[k0 + j] - y0) / span;
        if (!got10 && (r >= 0.1f)) {
            *t10 = ((float)(j - 1) + (0.1f - prev) / (r - prev)) * 10.0f;
            got10 = 1;
        }
        if (got10 && (r >= 0.632f)) {
            *t63 = ((float)(j - 1) + (0.632f - prev) / (r - prev)) * 10.0f;
            return 1;
        }
        prev = r;
    }
    return 0;
}

void wtest_calc(void)
{
    uint16_t w;
    int16_t s, k;

    for (w = 0; w < 2u; w++) {
        const int16_t *y = W.rpm[w];
        float st = 0.0f, sd = 0.0f, y0 = 0.0f;
        for (k = 1; k <= WT_PRE + 1; k++) { y0 += (float)y[k]; }
        y0 /= (float)(WT_PRE + 1);
        W.used[w] = 0;
        for (s = 0; s < WT_STEPS; s++) {
            float t10, t63, tau, dl;
            if (s > 0) { y0 = (float)y[WT_PRE + s_seg[s] + 1]; }
            W.tau[w][s] = W.dead[w][s] = TUNE_NO_DATA;
            if (!step_times(y, s, y0, (float)W.cmd[w][s], &t10, &t63)) { continue; }
            tau = (t63 - t10) / WT_F1063;
            dl = t10 - WT_F10 * tau;
            W.tau[w][s] = (int16_t)(tau + 0.5f);
            W.dead[w][s] = (int16_t)((dl < 0.0f) ? (dl - 0.5f) : (dl + 0.5f));
            st += tau;
            sd += dl;
            W.used[w]++;
        }
        if (W.used[w] > 0) {
            float n = (float)W.used[w];
            int16_t a = (int16_t)(st / n + 0.5f), b = (int16_t)((sd < 0.0f) ? (sd / n - 0.5f) : (sd / n + 0.5f));
            if (w == 0u) { W.wtl = a; W.wdl = b; } else { W.wtr = a; W.wdr = b; }
        }
    }
    if ((W.used[0] == 0) || (W.used[1] == 0)) { W.state = 3; W.fail = 3; }
    else                                      { W.state = 2; }
}

int wtest_run(void)
{
    int16_t dev = iabs(D.guide_pos);
    int driving = D.flag_run && !D.flag_stopping && !D.system_err && !D.stop_err && (D.control == RUN_DRIVE);

    s_wt.t++;
    switch (W.phase) {
    case WT_WAIT:
        if (!driving) { abort_wt(4); break; }
        if (s_wt.t > WT_WAIT_TICKS) { abort_wt(2); break; }
        if ((D.step == DRIVE_CRUISE) && (D.vin_out == D.speed_cmd) && !D.guide_all_off && !D.risk_stop
            && (D.branch_start == 0) && (dev <= WT_STEADY_DEV)) {
            s_wt.steady++;
            s_wt.tq += (float)(D.tq_cmd[0] + D.tq_cmd[1]);
            s_wt.w += (float)(D.w_cmd[0] + D.w_cmd[1]);
        }
        else { s_wt.steady = 0; s_wt.tq = s_wt.w = 0.0f; }
        if (s_wt.steady > WT_STEADY_TICKS - WT_PRE - 1) { take((int16_t)(s_wt.steady - (WT_STEADY_TICKS - WT_PRE))); }
        if (s_wt.steady >= WT_STEADY_TICKS) {
            W.phase = WT_MEAS;
            W.v_rpm = D.vin_out;
            /* FF 추천: 1000 rpm 당 토크(파라미터 FF 단위, param.c speed_ff = FF/1000). 9999 에서 자른다(표시 OVF) */
            if (s_wt.w > 0.0f) {
                float f = s_wt.tq / s_wt.w * 1000.0f + 0.5f;
                W.ff = (int16_t)((f > 9999.0f) ? 9999.0f : f);
            }
            s_wt.t = 0;
            steer_test_rpm = WT_A_RPM;
        }
        break;
    case WT_MEAS: {               /* s_wt.t = 1 이 계단 1 다음 틱 */
        int16_t j = s_wt.t, s;
        int16_t half = D.guide_wide ? 32 : 16;
        take((int16_t)(WT_PRE + j));
        for (s = 0; s < WT_STEPS; s++) {   /* 계단 s 지령은 그 계단 다음 틱의 w_cmd(앞 틱 motor_run 이 냈다) */
            if (j == s_seg[s] + 1) { W.cmd[0][s] = D.w_cmd[0]; W.cmd[1][s] = D.w_cmd[1]; }
        }
        if (dev > W.dev_max) { W.dev_max = dev; }
        if (!driving) { abort_wt(4); break; }
        if (D.guide_all_off || (dev * 10 > half * 6)) { abort_wt(1); break; }
        if (j >= s_seg[WT_STEPS] + 1) {    /* 마지막 표본(가이드 조향은 앞 틱부터) */
            wtest_calc();
            W.phase = WT_BACK;
            s_wt.t = 0;
            break;
        }
        if (j == s_seg[WT_STEPS]) { steer_test_rpm = 0; }
        else { steer_test_rpm = (int16_t)((j < s_seg[1]) || (j >= s_seg[2]) ? WT_A_RPM : -WT_A_RPM); }
        break;
    }
    case WT_BACK:
        if (!driving) { W.phase = WT_STOP; break; }
        if ((dev <= WT_BACK_DEV) || (s_wt.t > WT_BACK_TICKS)) {
            W.dev_end = D.guide_pos;
            tagact_stop();
            W.phase = WT_STOP;
        }
        break;
    case WT_STOP:
        if (!D.flag_ready && !D.flag_run && !D.flag_stopping) {
            W.phase = WT_OFF;
            /* 정상(WTOK)이고 범위 안이면 FF 를 저장한다. 선 뒤라 주행 중 토크가 뛰지 않는다.
             * 속도 루프 시험(TST, 고정 FF) 중에는 저장하지 않는다: TST 0 으로 돌아가면 예전 이득에 FF 가 켜진 채 남는다 */
            W.ff_old = param_get(P_SPEED_FF);
            if ((W.state == 2) && (W.ff >= 0) && (W.ff <= 999) && param_tst_old) { (void)param_set(P_SPEED_FF, W.ff); }
            return 0;
        }
        break;
    default:
        return 0;
    }
    return 1;
}
#endif
