/* gresp.c - 가이드 센서 응답 시험(t3.7 GR). 방식은 gresp.h */
#include "agv/gresp.h"
#if AGV_DEV_TESTS
#include "agv/frame.h"
#include "agv/guide.h"
#include "agv/hal.h"
#include "agv/ident.h"

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

int gresp_busy(void) { return s_gr.on != 0u; }

void gresp_poll(void)
{
    uint16_t b, n;
    uint32_t now;

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
#endif
