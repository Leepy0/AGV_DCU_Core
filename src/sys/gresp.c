/* gresp.c - 가이드 센서 응답 시험(t3.7 GR)·스캔 주기 시험(t3.8 GS). 방식은 gresp.h */
#include "agv/gresp.h"
#if AGV_DEV_TESTS
#include <math.h>
#include <string.h>
#include "agv/drive.h"
#include "agv/frame.h"
#include "agv/guide.h"
#include "agv/hal.h"
#include "agv/ident.h"
#include "agv/tagact.h"

#define D agv_drive

#define GR_FIRST     5u      /* 첫 간격에서 이만큼 보내도 정상 응답이 하나도 없으면 없음 */
#define GR_WAIT_MS   20u     /* 바로 재요청 간격의 응답 기다림 */
#define GR_FRAME_MAX 16u
#define CYC_01MS     (HAL_CYCLES_PER_MS / 10UL)

static const uint16_t s_gap[] = { 100u, 80u, 60u, 50u, 40u, 30u, 20u, 0u };   /* 0.1 ms. 0 = 응답이 끝나면 바로 */
#define GR_STEPS ((uint16_t)(sizeof(s_gap) / sizeof(s_gap[0])))

static struct {
    uint16_t on, step, sent, done, retry, bad, waiting, len, ok, line;
    uint32_t t_send, t_step;
    uint32_t c_min, c_max;
    uint16_t buf[GR_FRAME_MAX];
} s_gr;

#define R agv_all.gr

static int16_t to_01ms(uint32_t c)
{
    uint32_t v = (c + CYC_01MS / 2UL) / CYC_01MS;
    return (int16_t)((v > 32000UL) ? 32000UL : v);
}

static void drain(void)
{
    uint16_t b;
    while (hal_sci_rx(HAL_SCI_B, &b)) { }
    (void)hal_sci_take_errors(HAL_SCI_B);
}

static void send(uint32_t now)
{
    uint16_t buf[4];

    buf[0] = FRAME_STX;
    buf[1] = 'H';
    buf[2] = frame_checksum(&buf[1], 1);
    buf[3] = FRAME_ETX;
    s_gr.t_send = now;
    s_gr.waiting = 1;
    s_gr.len = 0;
    s_gr.sent++;
    (void)hal_sci_send(HAL_SCI_B, buf, 4);
}

static void finish(int16_t state)
{
    s_gr.on = 0;
    R.state = state;
    if (state == 3) { R.grl = R.grs = R.grh = TUNE_NO_DATA; }
    drain();
}

/* 요청 하나의 결과. ok 0 = 오류(무응답 포함) */
static void outcome(int ok, uint32_t stamp, int line)
{
    s_gr.waiting = 0;
    s_gr.len = 0;
    s_gr.done++;
    if (!ok) {
        s_gr.bad++;
        R.err++;
        return;
    }
    s_gr.ok++;
    if (line) { s_gr.line++; }
    if (s_gr.step == 0u) {             /* 응답 시간은 평상시 주기(10 ms) 간격에서만 */
        uint32_t c = stamp - s_gr.t_send;
        if ((R.n == 0) || (c < s_gr.c_min)) { s_gr.c_min = c; }
        if ((R.n == 0) || (c > s_gr.c_max)) { s_gr.c_max = c; }
        R.n++;
        R.grl = to_01ms(s_gr.c_min);
        R.grh = to_01ms(s_gr.c_max);
    }
}

static void feed(uint16_t b)
{
    guide_rx_t rx;

    b &= 0xFFu;
    if (!s_gr.waiting) { return; }     /* 늦게 온 바이트는 버린다(그 요청은 이미 무응답으로 셌다) */
    if (b == FRAME_STX) { s_gr.len = 0; }
    else if (s_gr.len == 0u) { return; }
    if (s_gr.len >= GR_FRAME_MAX) {    /* ETX 없이 길다 */
        outcome(0, 0, 0);
        return;
    }
    s_gr.buf[s_gr.len++] = b;
    if (b != FRAME_ETX) { return; }
    guide_parse(s_gr.buf, s_gr.len, &rx);
    outcome(rx.kind == GUIDE_RX_LED, hal_sci_rx_stamp(HAL_SCI_B), guide_lateral(&rx) != 0UL);
}

static void step_begin(uint16_t step, uint32_t now)
{
    s_gr.step = step;
    s_gr.sent = s_gr.done = s_gr.bad = 0;
    s_gr.t_step = now;
    R.step = (int16_t)(step + 1u);
    send(now);
}

/* 간격 하나가 끝났다 */
static void step_end(uint32_t now)
{
    if (s_gr.step == 0u) {
        if ((s_gr.ok == 0u) || (s_gr.line == 0u)) { finish(3); return; }   /* 무응답·라인 없음 */
    }
    if (s_gr.bad != 0u) {
        if (!s_gr.retry) {             /* 잡음일 수 있다. 같은 간격을 한 번 더 */
            s_gr.retry = 1;
            step_begin(s_gr.step, now);
            return;
        }
        finish(2);                     /* 안정 주기는 앞 간격 그대로 */
        return;
    }
    s_gr.retry = 0;
    if (s_gap[s_gr.step] != 0u) { R.grs = (int16_t)s_gap[s_gr.step]; }
    else { R.grs = to_01ms((now - s_gr.t_step) / GR_N); }   /* 바로 재요청: 평균 요청 주기 */
    if (s_gr.step + 1u >= GR_STEPS) { finish(2); return; }
    step_begin((uint16_t)(s_gr.step + 1u), now);
}

void gresp_start(void)
{
    s_gr.on = 1;
    s_gr.retry = s_gr.ok = s_gr.line = 0;
    s_gr.c_min = s_gr.c_max = 0;
    R.state = 1;
    R.grl = R.grs = R.grh = TUNE_NO_DATA;
    R.n = R.err = 0;
    drain();
    step_begin(0, hal_cycles());
}

static void gs_poll(void);

int gresp_busy(void) { return (s_gr.on != 0u) || (agv_gs.phase == 1u); }   /* GR, GS 라인 확인 */

void gresp_poll(void)
{
    uint16_t b, n;
    uint32_t now;

    if (agv_gs.phase != 0u) { gs_poll(); return; }
    if (!s_gr.on) { return; }
    n = hal_sci_take_errors(HAL_SCI_B);
    if (n != 0u) {                     /* 수신 오류는 어느 요청 것인지 모른다. 세고 간격을 오류로 친다 */
        R.err = (int16_t)(R.err + n);
        s_gr.bad++;
    }
    while (s_gr.on && hal_sci_rx(HAL_SCI_B, &b)) { feed(b); }
    now = hal_cycles();
    if (s_gr.waiting) {                /* 다음 요청 때까지(바로 재요청은 GR_WAIT_MS) 안 끝나면 무응답 */
        uint32_t lim = (s_gap[s_gr.step] != 0u) ? (uint32_t)s_gap[s_gr.step] * CYC_01MS : GR_WAIT_MS * HAL_CYCLES_PER_MS;
        if (now - s_gr.t_send < lim) { return; }
        outcome(0, 0, 0);
    }
    if ((s_gr.step == 0u) && (s_gr.done >= GR_FIRST) && (s_gr.ok == 0u)) { finish(3); return; }
    if (s_gr.done >= GR_N) { step_end(now); return; }
    if (s_gap[s_gr.step] == 0u) { send(now); return; }
    if (now - s_gr.t_send >= (uint32_t)s_gap[s_gr.step] * CYC_01MS) { send(now); }
}

/* ---- 스캔 주기 시험(t3.8 GS). 방식은 gresp.h ---- */

#define GS_FIRST      5u       /* 라인 확인: 이만큼 보내도 정상 응답이 없으면 없음 */
#define GS_TURN_TICKS 6000u    /* 회전이 60 s 안에 안 끝나면 거기까지로 셈한다 */
#define GS_START_TICKS 100u    /* 1 s 안에 회전이 시작되지 않으면 없음 */
#define GS_P_MIN      200u     /* 후보 P 0.01 ms: 2.00 ~ 20.00 ms, 0.02 ms 간격 */
#define GS_P_MAX      2000u
#define GS_P_STEP     2u
#define GS_PAIRS_MIN  20u
#ifndef GS_R_ABS
#define GS_R_ABS      0.25f    /* R 하한. 이 아래는 잡음과 못 가른다(P 약 2.5 ms 이하). 모의(test_curve gs)에서 정했다 */
#endif
#ifndef GS_SCORE
#define GS_SCORE      0.70f    /* R ÷ 기대 R 하한. 회전 기하(칸을 지나는 주기)가 만드는 가짜 봉우리는 0.6 아래 */
#endif
#define GS_PI         3.14159265f
#define GS_UNIT_CYC   (HAL_CYCLES_PER_MS / 100UL)   /* 0.01 ms */
#define GS_CALC_BUDGET 256u     /* 메인 루프 한 번에 셀 쌍 수(실기 약 0.1 ms) */

#ifndef AGV_HOST_BUILD
#pragma DATA_SECTION(agv_gs, "gscan_ram")   /* 보존 영역(test_keep) 밖. KEEP_MARK 와 무관 */
#pragma DATA_SECTION(s_tu, "gscan_ram")
#endif
gscan_result_t agv_gs;
static uint32_t s_tu[GS_BUF];   /* 계산용: t[0] 기준 0.01 ms */

static const int16_t s_sin64[64] = {
    0, 3212, 6393, 9512, 12539, 15446, 18204, 20787, 23170, 25329, 27245, 28898, 30273, 31356, 32137, 32609,
    32767, 32609, 32137, 31356, 30273, 28898, 27245, 25329, 23170, 20787, 18204, 15446, 12539, 9512, 6393, 3212,
    0, -3212, -6393, -9512, -12539, -15446, -18204, -20787, -23170, -25329, -27245, -28898, -30273, -31356, -32137, -32609,
    -32767, -32609, -32137, -31356, -30273, -28898, -27245, -25329, -23170, -20787, -18204, -15446, -12539, -9512, -6393, -3212
};

enum { GS_OFF = 0, GS_LINE, GS_TURN, GS_CALC };

static struct {
    uint16_t waiting, len, have, base_ok, tries, ok, line, turn_seen, ticks;
    uint32_t t_send, base, t_first;
    guide_rx_t rx;                 /* 마지막 응답. guide_poll 이 가져간다 */
    uint16_t buf[GR_FRAME_MAX];
    /* 계산 */
    uint16_t p, i, j, scale, n;
    int32_t  cs, sn;
    float    best, best_pass, sig2;
} s_gs;

#define G agv_gs

void gresp_reset(void)
{
    G.state = 0;
    G.phase = GS_OFF;
    G.nt = 0;
}

static void gs_send(uint32_t now)
{
    uint16_t buf[4];

    buf[0] = FRAME_STX;
    buf[1] = 'H';
    buf[2] = frame_checksum(&buf[1], 1);
    buf[3] = FRAME_ETX;
    s_gs.t_send = now;
    s_gs.waiting = 1;
    s_gs.len = 0;
    if (G.phase == GS_TURN) {
        if (G.req == 0u) { s_gs.t_first = now; }
        G.span = now - s_gs.t_first;
        G.req++;
    }
    (void)hal_sci_send(HAL_SCI_B, buf, 4);
}

static void gs_none(void)
{
    G.state = 3;
    G.phase = GS_OFF;
    G.gsn = 0;
    G.gsl = G.gsp = TUNE_NO_DATA;
    drain();
}

/* 요청 하나의 끝. rx 0 = 무응답·길이 넘침 */
static void gs_outcome(const guide_rx_t *rx)
{
    int ok = (rx != 0) && (rx->kind == GUIDE_RX_LED);

    s_gs.waiting = 0;
    s_gs.len = 0;
    if (rx != 0) {                     /* 회전 제어에는 받은 그대로 넘긴다(체크섬 실패도 guide_poll 이 평상시처럼 본다) */
        s_gs.rx = *rx;
        s_gs.have = 1;
    }
    if (!ok) {
        if (G.phase == GS_TURN) { G.err++; }
        else { s_gs.tries++; }
        return;
    }
    if (G.phase == GS_LINE) {
        s_gs.ok++;
        if (guide_lateral(rx) != 0UL) { s_gs.line = 1; }
        return;
    }
    if (s_gs.base_ok && (guide_lateral(rx) != s_gs.base)) {
        if (G.nt < GS_BUF) { G.t[G.nt++] = s_gs.t_send; }
        if (G.gsn < 32767) { G.gsn++; }
    }
    s_gs.base = guide_lateral(rx);
    s_gs.base_ok = 1;
}

static void gs_feed(uint16_t b)
{
    guide_rx_t rx;

    b &= 0xFFu;
    if (!s_gs.waiting) { return; }
    if (b == FRAME_STX) { s_gs.len = 0; }
    else if (s_gs.len == 0u) { return; }
    if (s_gs.len >= GR_FRAME_MAX) {
        gs_outcome(0);
        return;
    }
    s_gs.buf[s_gs.len++] = b;
    if (b != FRAME_ETX) { return; }
    guide_parse(s_gs.buf, s_gs.len, &rx);
    gs_outcome(&rx);
}

void gscan_start(void)
{
    memset(&s_gs, 0, sizeof(s_gs));
    G.state = 1;
    G.phase = GS_LINE;
    G.gsn = 0;
    G.gsl = G.gsp = TUNE_NO_DATA;
    G.p100 = G.r1000 = G.pmax100 = G.rmax1000 = 0;
    G.pairs = G.req = G.err = G.nt = 0;
    G.span = 0;
    drain();
    gs_send(hal_cycles());
}

int gscan_fast(void) { return G.phase == GS_TURN; }

int gscan_take(guide_rx_t *rx)
{
    if (!s_gs.have) { return 0; }
    *rx = s_gs.rx;
    s_gs.have = 0;
    return 1;
}

/* 가장 짧은 연속 간격, 0.01 ms 시각표, 첫 후보 */
void gscan_calc_begin(void)
{
    uint16_t k;
    uint32_t lo = 0xFFFFFFFFUL;

    for (k = 0; k < G.nt; k++) {
        s_tu[k] = (G.t[k] - G.t[0]) / GS_UNIT_CYC;
        if ((k > 0) && (G.t[k] - G.t[k - 1] < lo)) { lo = G.t[k] - G.t[k - 1]; }
    }
    G.gsl = (G.nt >= 2u) ? to_01ms(lo) : TUNE_NO_DATA;
    G.gsp = TUNE_NO_DATA;
    G.p100 = G.r1000 = G.pmax100 = G.rmax1000 = 0;
    G.pairs = 0;
    {   /* 검출 시각 오차 = 스캔 뒤 다음 요청까지(0~요청 주기 T 고르게). 두 시각 차의 분산 T²/6 (0.01 ms²) */
        float tr = (G.req > 1u) ? ((float)G.span / (float)(G.req - 1u) / (float)GS_UNIT_CYC) : 160.0f;
        s_gs.sig2 = tr * tr / 6.0f;
    }
    s_gs.best_pass = -1.0f;
    s_gs.p = GS_P_MIN;
    s_gs.i = 0;
    s_gs.j = 1;
    s_gs.cs = s_gs.sn = 0;
    s_gs.n = 0;
    s_gs.best = -1.0f;
    s_gs.scale = (uint16_t)((64UL << 16) / GS_P_MIN);
    G.phase = GS_CALC;
}

/* 후보 하나가 끝났다. 스캔 주기 P 라면 R 기대값 = exp(-2π²σ²/P²). R ≥ GS_R_ABS 이고 점수 R ÷ 기대값 ≥ GS_SCORE 면 통과.
 * 통과한 후보 가운데 R 이 가장 큰 P 를 고른다. 약수 P/k 는 R 이 R(P)^k² 로 낮고 배수 kP 는 위상이 갈려 낮다.
 * 회전 기하의 가짜 봉우리(11~20 ms, R 0.6 아래)는 기대 R(0.9 넘음)에 못 미쳐 걸러진다 */
static void gs_candidate_end(void)
{
    float c = (float)s_gs.cs, s = (float)s_gs.sn, r2 = (c * c) + (s * s), p = (float)s_gs.p;
    float r = (s_gs.n > 0u) ? (sqrtf(r2) / ((float)s_gs.n * 32767.0f)) : 0.0f;
    float rexp = expf(-2.0f * GS_PI * GS_PI * s_gs.sig2 / (p * p));
    uint16_t pass = (uint16_t)((s_gs.n >= GS_PAIRS_MIN) && (r >= GS_R_ABS) && (r >= GS_SCORE * rexp));

    G.pairs = s_gs.n;
    if (r2 > s_gs.best) {               /* 지시 원안(R 최대) 기록 */
        s_gs.best = r2;
        G.pmax100 = (int16_t)s_gs.p;
        G.rmax1000 = (int16_t)(r * 1000.0f + 0.5f);
    }
    if (pass && (r > s_gs.best_pass)) {
        s_gs.best_pass = r;
        G.p100 = (int16_t)s_gs.p;
        G.r1000 = (int16_t)(r * 1000.0f + 0.5f);
    }
    s_gs.p = (uint16_t)(s_gs.p + GS_P_STEP);
    s_gs.scale = (uint16_t)((64UL << 16) / s_gs.p);
    s_gs.i = 0;
    s_gs.j = 1;
    s_gs.cs = s_gs.sn = 0;
    s_gs.n = 0;
}

static void gs_calc_end(void)
{
    G.gsp = (G.p100 > 0) ? (int16_t)((G.p100 + 5) / 10) : TUNE_NO_DATA;
    G.state = 2;
    G.phase = GS_OFF;
}

/* 쌍 budget 개씩. 0 = 끝 */
int gscan_calc_step(uint16_t budget)
{
    while (budget-- > 0u) {
        uint32_t d;
        uint16_t r;

        if ((G.nt < 2u) || (s_gs.p > GS_P_MAX)) {
            gs_calc_end();
            return 0;
        }
        d = (s_gs.j < G.nt) ? (s_tu[s_gs.j] - s_tu[s_gs.i]) : 0xFFFFFFFFUL;
        if (d > (uint32_t)GS_PAIR_MS * 100UL) {   /* 이 i 는 끝. 다음 i, 마지막이면 다음 후보 */
            s_gs.i++;
            s_gs.j = (uint16_t)(s_gs.i + 1u);
            if (s_gs.j >= G.nt) { gs_candidate_end(); }
            continue;
        }
        r = (uint16_t)((uint16_t)d % s_gs.p);
        {
            uint16_t k = (uint16_t)((((uint32_t)r * s_gs.scale) + 0x8000UL) >> 16) & 63u;
            s_gs.cs += s_sin64[(k + 16u) & 63u];
            s_gs.sn += s_sin64[k];
        }
        s_gs.n++;
        s_gs.j++;
    }
    return 1;
}

int gscan_run(void)
{
    if (G.phase == GS_LINE) {
        if ((s_gs.ok == 0u) && (s_gs.tries >= GS_FIRST)) { gs_none(); return 0; }   /* 무응답 */
        if ((s_gs.ok > 0u) && !s_gs.line)                { gs_none(); return 0; }   /* 라인 없음 */
        if (s_gs.line && !D.flag_ready && !D.flag_run && !D.flag_stopping && !D.flag_turn) {
            tagact_turn_align_stopped(TURN_CW, 360u, 0u);
            if ((D.turn_pulses <= 0) || !D.tape_align) { gs_none(); return 0; }
            G.phase = GS_TURN;
            s_gs.ticks = 0;
            s_gs.turn_seen = 0;
            s_gs.base_ok = 0;
            s_gs.have = 0;
        }
        return 1;
    }
    if (G.phase == GS_TURN) {
        s_gs.ticks++;
        if (D.flag_turn) { s_gs.turn_seen = 1; }
        if ((!s_gs.turn_seen && (s_gs.ticks > GS_START_TICKS))
            || (s_gs.turn_seen && !D.flag_turn && !D.tape_align && !D.flag_stopping && (D.turn_motion == 0))
            || (s_gs.ticks > GS_TURN_TICKS)) {
            drain();
            gscan_calc_begin();
        }
        return 1;
    }
    return G.phase == GS_CALC;
}

static void gs_poll(void)
{
    uint16_t b, n;
    uint32_t now;

    if (G.phase == GS_CALC) {
        (void)gscan_calc_step(GS_CALC_BUDGET);
        return;
    }
    n = hal_sci_take_errors(HAL_SCI_B);
    if ((n != 0u) && (G.phase == GS_TURN)) { G.err = (uint16_t)(G.err + n); }
    while (hal_sci_rx(HAL_SCI_B, &b)) { gs_feed(b); }
    now = hal_cycles();
    if (s_gs.waiting) {
        if (now - s_gs.t_send < (uint32_t)GS_WAIT_MS * HAL_CYCLES_PER_MS) { return; }
        gs_outcome(0);
    }
    gs_send(now);
}
#endif
