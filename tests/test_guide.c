/* test_guide.c - 응답 해석과 LED 패턴 위치 추적. 표를 돌린다 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "agv/guide.h"
#include "agv/tagact.h"
#include "fakes.h"

/* 본문에 STX·BCC(소문자)·ETX 를 붙여 응답을 만든다 */
static uint16_t make(uint16_t *out, const char *body, int bcc_delta)
{
    uint16_t n = (uint16_t)strlen(body), sum = 0;
    out[0] = 0x02;
    for (uint16_t i = 0; i < n; i++) { out[i + 1] = (uint16_t)body[i]; sum = (uint16_t)(sum + (uint8_t)body[i]); }
    sum &= 0x0F;
    out[n + 1] = (uint16_t)(((sum < 10) ? ('0' + sum) : ('a' + sum - 10)) + bcc_delta);
    out[n + 2] = 0x03;
    return (uint16_t)(n + 3);
}

static void parse(const char *body, guide_rx_t *rx)
{
    uint16_t b[32];
    uint16_t n = make(b, body, 0);
    guide_parse(b, n, rx);
}

static guide_rx_t rx_narrow(uint16_t led)
{
    guide_rx_t r;
    memset(&r, 0, sizeof(r));
    r.kind = GUIDE_RX_LED;
    r.led_lo = led;
    return r;
}

static guide_rx_t rx_wide(uint32_t lo)
{
    guide_rx_t r;
    memset(&r, 0, sizeof(r));
    r.kind = GUIDE_RX_LED;
    r.wide = 1;
    r.led_lo = lo;
    return r;
}

typedef struct {
    const char *name;
    int16_t  wide;
    uint32_t lo;
    int16_t  prev_set;      /* 먼저 이 패턴을 추적시켜 이전 위치를 만든다 */
    uint32_t prev_lo;
    int16_t  branch, backward, invert;
    uint16_t min_w, max_w, max_step;
    int16_t  pos, status;
    uint16_t width, blobs;
} track_case_t;

#define BITS(a, b)  ((uint32_t)(((b) >= 31 ? 0xFFFFFFFFUL : ((1UL << ((b) + 1)) - 1UL)) & ~((1UL << (a)) - 1UL)))  /* 비트 a~b */

static const track_case_t s_cases[] = {
    /* 일반: 단일 덩어리. LED7~9 = 7+9-17 = -1 */
    { "n single",     0, BITS(6, 8), 0, 0,            0, 0, 0, 1, 32, 0,  -1, GUIDE_NORMAL, 3, 1 },
    { "n left edge",  0, BITS(0, 0), 0, 0,            0, 0, 0, 1, 32, 0, -15, GUIDE_NORMAL, 1, 1 },
    { "n right edge", 0, BITS(15, 15), 0, 0,            0, 0, 0, 1, 32, 0,  15, GUIDE_NORMAL, 1, 1 },
    /* 일반: 두 덩어리 추적. 이전 위치가 왼쪽이면 왼쪽을 잡는다 */
    { "n two track L", 0, BITS(1, 2) | BITS(12, 13), 1, BITS(2, 3),  0, 0, 0, 1, 32, 0, -12, GUIDE_NORMAL, 2, 2 },
    { "n two track R", 0, BITS(1, 2) | BITS(12, 13), 1, BITS(11, 12), 0, 0, 0, 1, 32, 0, 10, GUIDE_NORMAL, 2, 2 },
    /* 일반: 두 덩어리 분기 좌우. 이전 위치를 무시한다 */
    { "n branch L",   0, BITS(1, 2) | BITS(12, 13), 1, BITS(11, 12), BRANCH_LEFT, 0, 0, 1, 32, 0, -12, GUIDE_NORMAL, 2, 2 },
    { "n branch R",   0, BITS(1, 2) | BITS(12, 13), 1, BITS(2, 3),  BRANCH_RIGHT, 0, 0, 1, 32, 0, 10, GUIDE_NORMAL, 2, 2 },
    /* 후진은 진행 방향 좌우가 센서 좌우와 반대다 */
    { "n back L",     0, BITS(1, 2) | BITS(12, 13), 0, 0, BRANCH_LEFT, 1, 0, 1, 32, 0,  10, GUIDE_NORMAL, 2, 2 },
    { "n back R",     0, BITS(1, 2) | BITS(12, 13), 0, 0, BRANCH_RIGHT, 1, 0, 1, 32, 0, -12, GUIDE_NORMAL, 2, 2 },
    /* 1칸 노이즈: 필터 끔이면 덩어리이고 가까우면 잡힌다. 켜면 버린다 */
    { "n noise off",  0, BITS(6, 8) | BITS(8 + 3, 8 + 3), 1, BITS(11, 11), 0, 0, 0, 1, 32, 0, 7, GUIDE_NORMAL, 1, 2 },
    { "n noise on",   0, BITS(6, 8) | BITS(8 + 3, 8 + 3), 1, BITS(11, 11), 0, 0, 0, 2, 32, 0, -1, GUIDE_NORMAL, 3, 2 },
    /* 전부 켜짐·꺼짐: 위치 유지 */
    { "n all on",     0, 0xFFFFUL, 1, BITS(9, 10), 0, 0, 0, 1, 32, 0, 4, GUIDE_ALL_ON, 16, 1 },
    { "n all off",    0, 0, 1, BITS(9, 10), 0, 0, 0, 1, 32, 0, 4, GUIDE_LOST, 0, 0 },
    /* 일반은 상위 비트를 보지 않는다 */
    { "n ignore hi",  0, BITS(6, 8) | 0xFFFF0000UL, 0, 0,         0, 0, 0, 1, 32, 0, -1, GUIDE_NORMAL, 3, 1 },

    /* wide: 단일 덩어리. LED15~18 = 15+18-33 = 0 */
    { "w single",     1, BITS(14, 17), 0, 0,            0, 0, 0, 1, 32, 0,   0, GUIDE_NORMAL, 4, 1 },
    { "w left edge",  1, BITS(0, 0), 0, 0,            0, 0, 0, 1, 32, 0, -31, GUIDE_NORMAL, 1, 1 },
    { "w right edge", 1, BITS(31, 31), 0, 0,            0, 0, 0, 1, 32, 0,  31, GUIDE_NORMAL, 1, 1 },
    /* wide: 두 덩어리 추적·분기 */
    { "w two track",  1, BITS(2, 3) | BITS(26, 27), 1, BITS(25, 26), 0, 0, 0, 1, 32, 0, 22, GUIDE_NORMAL, 2, 2 },
    { "w branch L",   1, BITS(2, 3) | BITS(26, 27), 1, BITS(25, 26), BRANCH_LEFT, 0, 0, 1, 32, 0, -26, GUIDE_NORMAL, 2, 2 },
    { "w branch R",   1, BITS(2, 3) | BITS(26, 27), 1, BITS(2, 3),  BRANCH_RIGHT, 0, 0, 1, 32, 0, 22, GUIDE_NORMAL, 2, 2 },
    { "w back L",     1, BITS(2, 3) | BITS(26, 27), 0, 0, BRANCH_LEFT, 1, 0, 1, 32, 0, 22, GUIDE_NORMAL, 2, 2 },
    /* wide: 1칸 노이즈, 필터 */
    { "w noise off",  1, BITS(14, 17) | BITS(20, 20), 1, BITS(20, 20), 0, 0, 0, 1, 32, 0, 9, GUIDE_NORMAL, 1, 2 },
    { "w noise on",   1, BITS(14, 17) | BITS(20, 20), 1, BITS(20, 20), 0, 0, 0, 2, 32, 0, 0, GUIDE_NORMAL, 4, 2 },
    /* wide: 전부 켜짐·꺼짐 */
    { "w all on 32",  1, 0xFFFFFFFFUL, 1, BITS(18, 19), 0, 0, 0, 1, 32, 0, 6, GUIDE_ALL_ON, 32, 1 },
    { "w all off",    1, 0, 1, BITS(18, 19), 0, 0, 0, 1, 32, 0, 6, GUIDE_LOST, 0, 0 },
    /* 최대 폭 필터: 넓은 덩어리를 버려 LOST */
    { "w max width",  1, BITS(5, 20), 1, BITS(13, 14), 0, 0, 0, 1, 8, 0,  -4, GUIDE_LOST, 0, 1 },
    { "n max width",  0, BITS(2, 9), 1, BITS(3, 4),   0, 0, 0, 1, 7, 0, -8, GUIDE_LOST, 0, 1 },
    /* 변화량 필터: 멀리 튄 덩어리를 버리고, 끄면 따라간다 */
    { "n step on",    0, BITS(12, 13), 1, BITS(3, 4),   0, 0, 0, 1, 32, 4, -8, GUIDE_LOST, 0, 1 },
    { "n step off",   0, BITS(12, 13), 1, BITS(3, 4),   0, 0, 0, 1, 32, 0, 10, GUIDE_NORMAL, 2, 1 },
    { "n step near",  0, BITS(4, 5), 1, BITS(3, 4),   0, 0, 0, 1, 32, 4, -6, GUIDE_NORMAL, 2, 1 },
    /* 분기는 변화량 제한을 받지 않는다 */
    { "n step branch", 0, BITS(1, 2) | BITS(12, 13), 1, BITS(3, 4),  BRANCH_RIGHT, 0, 0, 1, 32, 4, 10, GUIDE_NORMAL, 2, 2 },
    /* invert: 위치 부호와 분기 끝이 함께 뒤집힌다. 이전 위치 추적은 그대로 */
    { "n invert",     0, BITS(1, 2), 0, 0,            0, 0, 1, 1, 32, 0,  12, GUIDE_NORMAL, 2, 1 },
    { "n inv branch L", 0, BITS(1, 2) | BITS(12, 13), 0, 0, BRANCH_LEFT, 0, 1, 1, 32, 0, -10, GUIDE_NORMAL, 2, 2 },
    { "n inv back L", 0, BITS(1, 2) | BITS(12, 13), 0, 0, BRANCH_LEFT, 1, 1, 1, 32, 0,  12, GUIDE_NORMAL, 2, 2 },
    { "w invert",     1, BITS(26, 27), 0, 0,            0, 0, 1, 1, 32, 0, -22, GUIDE_NORMAL, 2, 1 },
};

/* LED i(1부터)의 감지 구간 [i-0.5, i+0.5] 를 테이프가 0.35 넘게 덮으면 켜진다. 단위 = 칸(6.89 mm) */
static float cover(float c, float w, int i)
{
    float lo = c - w / 2.0f, hi = c + w / 2.0f;
    float a = (lo > (float)i - 0.5f) ? lo : (float)i - 0.5f;
    float b = (hi < (float)i + 0.5f) ? hi : (float)i + 0.5f;
    return (b > a) ? (b - a) : 0.0f;
}

/* 직진(1) 또는 곡선(0, 좌우 20% 차)을 GUIDE_STRAIGHT_TICKS 이어 준다 */
static void straight_set(int on)
{
    int k;
    for (k = 0; k < GUIDE_STRAIGHT_TICKS; k++) { (void)guide_straight_tick(1000, on ? 1000 : 800, 50); }
}

/* 직진 판정과 직진 중에만 하는 폭 학습 */
static void test_straight(void)
{
    guide_out_t o;
    guide_rx_t r;
    int k;

    /* 경계: 좌우 차가 평균의 4% 이하(1000/1040 은 3.9%, 1000/1042 는 4.1%)가 0.3 s 이어져야 직진 */
    straight_set(0);
    for (k = 1; k < GUIDE_STRAIGHT_TICKS; k++) { assert(guide_straight_tick(1000, 1040, 50) == 0); }
    assert(guide_straight_tick(1000, 1040, 50) == 1);
    assert(guide_straight_tick(1000, 1042, 50) == 0);
    for (k = 0; k < 3 * GUIDE_STRAIGHT_TICKS; k++) { assert(guide_straight_tick(1042, 1000, 50) == 0); }
    /* 한 틱이라도 벗어나면 처음부터 다시 센다 */
    for (k = 1; k < GUIDE_STRAIGHT_TICKS; k++) { (void)guide_straight_tick(1000, 1000, 50); }
    (void)guide_straight_tick(1000, 1100, 50);
    for (k = 1; k < GUIDE_STRAIGHT_TICKS; k++) { assert(guide_straight_tick(1000, 1000, 50) == 0); }
    /* 후진도 크기로 본다. 평균이 최저 속도 밑이면 직진 아님 */
    straight_set(0);
    for (k = 0; k < GUIDE_STRAIGHT_TICKS; k++) { (void)guide_straight_tick(-1000, -1040, 50); }
    assert(guide_straight_tick(-1000, -1040, 50) == 1);
    for (k = 0; k < 2 * GUIDE_STRAIGHT_TICKS; k++) { assert(guide_straight_tick(49, 49, 50) == 0); }
    for (k = 0; k < GUIDE_STRAIGHT_TICKS; k++) { (void)guide_straight_tick(50, 50, 50); }
    assert(guide_straight_tick(50, 50, 50) == 1);

    /* 곡선에서 1 s 넘게 넓어져도 배운 폭 그대로. 위치는 그대로 낸다 */
    guide_reset();
    straight_set(1);
    r = rx_narrow(BITS(7, 8));
    for (k = 0; k < 5; k++) { guide_track_drive(&r, 0, 0, &o); }
    straight_set(0);
    r = rx_narrow(BITS(6, 9));
    for (k = 0; k < 3 * GUIDE_WIDE_LEARN_SCANS; k++) { guide_track_drive(&r, 0, 0, &o); }
    assert(guide_tape_width() == 2 && o.width == 4 && o.pos == 0 && o.track == GUIDE_TRK_NORMAL);
    /* 직진에서 같은 넓어짐은 배운다 */
    straight_set(1);
    for (k = 0; k < GUIDE_WIDE_LEARN_SCANS + 40; k++) { guide_track_drive(&r, 0, 0, &o); }
    assert(guide_tape_width() == 4);
    /* 곡선 중 좁아짐도 배우지 않는다. 직진이 되면 배운다 */
    straight_set(0);
    r = rx_narrow(BITS(7, 8));
    for (k = 0; k < 50; k++) { guide_track_drive(&r, 0, 0, &o); }
    assert(guide_tape_width() == 4);
    straight_set(1);
    for (k = 0; k < 20; k++) { guide_track_drive(&r, 0, 0, &o); }
    assert(guide_tape_width() == 2);
    /* 넓어짐을 세던 중 곡선이 끼면 수를 버린다 */
    r = rx_narrow(BITS(6, 9));
    for (k = 0; k < GUIDE_WIDE_LEARN_SCANS - 10; k++) { guide_track_drive(&r, 0, 0, &o); }
    straight_set(0);
    guide_track_drive(&r, 0, 0, &o);
    straight_set(1);
    for (k = 0; k < GUIDE_WIDE_LEARN_SCANS - 10; k++) { guide_track_drive(&r, 0, 0, &o); }
    assert(guide_tape_width() == 2);
    /* 해제 뒤 다시 배우기도 곡선에서는 하지 않는다 */
    r = rx_narrow(BITS(7, 8) | BITS(11, 12));
    guide_track_drive(&r, 0, 0, &o);
    r = rx_narrow(BITS(7, 11));
    guide_track_drive(&r, 0, 0, &o);
    assert(o.track == GUIDE_TRK_EDGE);
    straight_set(0);
    guide_track_release();
    r = rx_narrow(BITS(3, 12));
    for (k = 0; k < 70; k++) { guide_track_drive(&r, 0, 0, &o); }
    assert(o.track == GUIDE_TRK_NORMAL && o.pos == 0 && guide_tape_width() == 2);
    straight_set(1);
    for (k = 0; k < 70; k++) { guide_track_drive(&r, 0, 0, &o); }
    assert(guide_tape_width() >= 9);
    /* 처음(배운 폭 없음)은 직진 판정 전이어도 첫 덩어리 폭으로 시작한다 */
    guide_reset();
    straight_set(0);
    r = rx_narrow(BITS(7, 8));
    guide_track_drive(&r, 0, 0, &o);
    assert(guide_tape_width() == 2);
    straight_set(1);
}

/* 다시 잡을 때 지속 확인. 주행 위치 경로(guide_track_drive)에서 잡았다가 잃은 뒤에만 건다 */
static void test_reacq(void)
{
    guide_rx_t line = rx_narrow(BITS(7, 8)), none = rx_narrow(0), dot = rx_narrow(BITS(11, 12)), far = rx_narrow(BITS(2, 3));
    guide_rx_t two = rx_narrow(BITS(2, 3) | BITS(11, 12)), all = rx_narrow(0xFFFF);
    guide_out_t o;
    int k;

    /* (4) 첫 획득은 바로 */
    guide_reset();
    guide_track_drive(&dot, 0, 0, &o);
    assert(o.status == GUIDE_NORMAL && o.pos == 8);
    guide_track_drive(&line, 0, 0, &o);
    assert(o.status == GUIDE_NORMAL && o.pos == 0);
    /* (1) 잃은 뒤 다른 자리의 한 응답짜리 점: LOST 그대로, 위치 불변 */
    guide_track_drive(&none, 0, 0, &o);
    assert(o.status == GUIDE_LOST && o.pos == 0);
    guide_track_drive(&dot, 0, 0, &o);
    assert(o.status == GUIDE_LOST && o.pos == 0);
    guide_track_drive(&none, 0, 0, &o);
    assert(o.status == GUIDE_LOST && o.pos == 0);
    /* (2) 같은 자리에 3응답 연속이면 받는다. 한 칸(2단위) 움직여도 같은 자리다 */
    guide_track_drive(&dot, 0, 0, &o);
    assert(o.status == GUIDE_LOST);
    dot = rx_narrow(BITS(12, 13));
    guide_track_drive(&dot, 0, 0, &o);
    assert(o.status == GUIDE_LOST && o.pos == 0);
    guide_track_drive(&dot, 0, 0, &o);
    assert(o.status == GUIDE_NORMAL && o.pos == 10 && o.track == GUIDE_TRK_NORMAL);
    /* (3) 점이 2응답 뒤 사라지고 진짜 선이 다른 자리에 3응답 */
    guide_track_drive(&line, 0, 0, &o);
    guide_track_drive(&none, 0, 0, &o);
    dot = rx_narrow(BITS(11, 12));
    guide_track_drive(&dot, 0, 0, &o);
    guide_track_drive(&dot, 0, 0, &o);
    assert(o.status == GUIDE_LOST && o.pos == 0);
    for (k = 0; k < 3; k++) {
        guide_track_drive(&far, 0, 0, &o);
        assert((o.status == GUIDE_NORMAL) == (k == 2));
    }
    assert(o.pos == -10);
    /* 후보(-10)가 이어지는 동안 믿던 위치에 더 가까운 점이 끼어도 후보를 바꾸지 않는다 */
    guide_track_drive(&none, 0, 0, &o);
    guide_track_drive(&far, 0, 0, &o);
    guide_track_drive(&two, 0, 0, &o);
    guide_track_drive(&two, 0, 0, &o);
    assert(o.status == GUIDE_NORMAL && o.pos == -10);
    /* 전부 켜짐(교차선) 뒤 복귀는 잃은 것이 아니다: 바로 */
    guide_track_drive(&all, 0, 0, &o);
    assert(o.status == GUIDE_ALL_ON);
    guide_track_drive(&far, 0, 0, &o);
    assert(o.status == GUIDE_NORMAL && o.pos == -10);
    /* (5) 정렬 회전·튜닝 경로(guide_track)와 분기는 바로 */
    guide_track_drive(&none, 0, 0, &o);
    guide_track(&dot, 0, 0, &o);
    assert(o.status == GUIDE_NORMAL && o.pos == 8);
    guide_track_drive(&none, 0, 0, &o);
    guide_track_drive(&two, BRANCH_LEFT, 0, &o);
    assert(o.status == GUIDE_NORMAL && o.pos == -10);
    /* (6) 센서 전환·주행 시작 뒤 첫 획득은 바로 */
    guide_track_drive(&none, 0, 0, &o);
    guide_on_sensor_switch();
    guide_track_drive(&dot, 0, 0, &o);
    assert(o.status == GUIDE_NORMAL && o.pos == 8);
    guide_track_drive(&none, 0, 0, &o);
    guide_on_run_start();
    guide_track_drive(&far, 0, 0, &o);
    assert(o.status == GUIDE_NORMAL && o.pos == -10);
    guide_reset();
    puts("  reacq ok");
}

/* 주행선은 가만히 있고 교차선이 센서를 왼쪽에서 오른쪽으로 지나간다. 반환: 참값과의 최대 오차(위치 단위) */
static int cross_sim(int leds, float tape_mm, float main_c, float step, int *naive_worst)
{
    float w = tape_mm / 6.89f, wx = w * 2.0f;      /* 30도: 가로로 차지하는 폭 = 폭 / sin30 */
    float truth = 2.0f * main_c - (float)(leds + 1);
    int worst = 0, k;
    guide_out_t o;
    guide_rx_t r;

    guide_reset();
    *naive_worst = 0;
    for (k = -40; ; k++) {
        float xc = (k < 0) ? -100.0f : (-wx + step * (float)k);
        uint32_t bits = 0;
        int i, lo = 0, hi = 0, e;
        if (xc > (float)leds + wx + 2.0f) { break; }
        for (i = 1; i <= leds; i++) {
            if (cover(main_c, w, i) + cover(xc, wx, i) > 0.35f) { bits |= 1UL << (i - 1); }
        }
        r = (leds == 32) ? rx_wide(bits) : rx_narrow((uint16_t)bits);
        guide_track_drive(&r, 0, 0, &o);
        assert(o.status == GUIDE_NORMAL);
        e = (int)((float)o.pos - truth);
        if (e < 0) { e = -e; }
        if (e > worst) { worst = e; }
        /* 대책 없는 값: 주행선을 포함한 덩어리의 가운데 */
        for (i = (int)(main_c + 0.5f); (i >= 1) && ((bits >> (i - 1)) & 1UL); i--) { lo = i; }
        for (i = (int)(main_c + 0.5f); (i <= leds) && ((bits >> (i - 1)) & 1UL); i++) { hi = i; }
        e = (int)((float)(lo + hi - (leds + 1)) - truth);
        if (e < 0) { e = -e; }
        if ((lo != 0) && (e > *naive_worst)) { *naive_worst = e; }
    }
    return worst;
}

static void test_cross(void)
{
    static const float tape[2] = { 10.0f, 25.0f };
    static const float step[3] = { 0.84f, 0.3f, 0.07f };    /* 60, 20, 5 m/min 쯤 */
    int w, t, s, n;
    for (w = 0; w < 2; w++) {
        for (t = 0; t < 2; t++) {
            for (s = 0; s < 3; s++) {
                int leds = w ? 32 : 16;
                float c[3];
                int j;
                c[0] = (float)leds / 2.0f + 0.5f; c[1] = c[0] + 0.3f; c[2] = c[0] - 3.0f;
                for (j = 0; j < 3; j++) {
                    int e = cross_sim(leds, tape[t], c[j], step[s], &n);
                    if ((s == 0) && (j == 0)) { printf("  30도 교차 %s %.0f mm: 최대 오차 %d (대책 없으면 %d)\n", w ? "wide" : "일반", tape[t], e, n); }
                    if (e > 3) { printf("  leds %d tape %.0f c %.1f step %.2f: %d\n", leds, tape[t], c[j], step[s], e); }
                    assert(e <= 3);
                }
            }
        }
    }
    /* 붙음 판정·폭 학습 */
    { guide_out_t o;
      guide_rx_t r;
      int k;
      guide_reset();
      r = rx_narrow(BITS(7, 8));
      for (k = 0; k < 5; k++) { guide_track_drive(&r, 0, 0, &o); }
      assert(guide_tape_width() == 2 && o.track == GUIDE_TRK_NORMAL && guide_curve_span() == 4);
      /* 둘째 덩어리를 본 적 없이 한쪽으로 넓어지면 커브다: 가운데, 폭 그대로. 폭은 배우지 않는다 */
      r = rx_narrow(BITS(7, 10));
      guide_track_drive(&r, 0, 0, &o);
      assert(o.track == GUIDE_TRK_NORMAL && o.pos == 2 && o.width == 4);
      r = rx_narrow(BITS(7, 11));
      guide_track_drive(&r, 0, 0, &o);
      assert(o.track == GUIDE_TRK_NORMAL && o.pos == 3 && o.width == 5 && guide_tape_width() == 2);
      /* 떨어진 둘째 덩어리가 먼저 보인 뒤 붙으면 가장자리 고정·유지 */
      r = rx_narrow(BITS(7, 8) | BITS(11, 12));
      guide_track_drive(&r, 0, 0, &o);
      assert(o.track == GUIDE_TRK_NORMAL && o.pos == 0 && o.blobs == 2);
      r = rx_narrow(BITS(7, 11));
      guide_track_drive(&r, 0, 0, &o);
      assert(o.track == GUIDE_TRK_EDGE && o.pos == 0 && o.width == 2);
      /* 정렬 회전·튜닝 직접 호출은 가장자리를 고치지 않고 폭도 배우지 않는다 */
      { guide_out_t t;
        guide_track(&r, 0, 0, &t);
        assert(t.track == GUIDE_TRK_NORMAL && t.pos == 3 && t.width == 5 && guide_tape_width() == 2);
        r = rx_narrow(BITS(7, 8));
        guide_track(&r, 0, 0, &t);
        r = rx_narrow(BITS(7, 11));
        guide_track_drive(&r, 0, 0, &o);
        assert(o.track == GUIDE_TRK_EDGE && o.pos == 0); }
      /* 붙어 있는 동안은 둘째 덩어리를 본 기억이 낡지 않는다 */
      for (k = 0; k < 2 * GUIDE_MERGE_SCANS; k++) { guide_track_drive(&r, 0, 0, &o); }
      assert(o.track == GUIDE_TRK_EDGE && o.pos == 0);
      r = rx_narrow(BITS(3, 12));
      guide_track_drive(&r, 0, 0, &o);
      assert(o.track == GUIDE_TRK_HOLD && o.pos == 0);
      /* 상한을 넘겨 풀어 주면 넓은 덩어리를 가운데로 받고, 폭은 지우지 않고 느리게 올린다 */
      guide_track_release();
      guide_track_drive(&r, 0, 0, &o);
      assert(o.track == GUIDE_TRK_NORMAL && o.pos == 0 && guide_tape_width() <= 3);
      for (k = 0; k < 10; k++) { guide_track_drive(&r, 0, 0, &o); }
      assert(o.track == GUIDE_TRK_NORMAL && guide_tape_width() >= 5 && guide_tape_width() <= 8);
      for (k = 0; k < 60; k++) { guide_track_drive(&r, 0, 0, &o); }
      assert(guide_tape_width() >= 9);
      /* 다시 좁아지면 빨리 따라 내려온다(좁아지는 학습) */
      r = rx_narrow(BITS(7, 8));
      for (k = 0; k < 20; k++) { guide_track_drive(&r, 0, 0, &o); }
      assert(guide_tape_width() == 2 && o.track == GUIDE_TRK_NORMAL && o.pos == 0);
      /* 둘째 덩어리를 본 지 GUIDE_MERGE_SCANS 가 지나면 넓어져도 커브로 본다 */
      r = rx_narrow(BITS(7, 8) | BITS(11, 12));
      guide_track_drive(&r, 0, 0, &o);
      r = rx_narrow(BITS(7, 8));
      for (k = 0; k < GUIDE_MERGE_SCANS; k++) { guide_track_drive(&r, 0, 0, &o); }
      r = rx_narrow(BITS(7, 11));
      guide_track_drive(&r, 0, 0, &o);
      assert(o.track == GUIDE_TRK_NORMAL && o.pos == 3);
      /* 첫 학습이 교차선 위(7칸)에서 시작해도 정상 테이프에서 내려온다. 붙은 채로 시작하면 가운데 */
      guide_reset();
      r = rx_narrow(BITS(4, 10));
      guide_track_drive(&r, 0, 0, &o);
      assert(o.track == GUIDE_TRK_NORMAL && o.pos == -1 && guide_tape_width() == 7);
      r = rx_narrow(BITS(7, 8));
      for (k = 0; k < 20; k++) { guide_track_drive(&r, 0, 0, &o); }
      assert(guide_tape_width() == 2);
      /* 한 칸만 켜지는 구간이 길어도 배운 폭은 2칸 아래로 안 간다 */
      r = rx_narrow(BITS(7, 7));
      for (k = 0; k < 50; k++) { guide_track_drive(&r, 0, 0, &o); }
      assert(guide_tape_width() == 2 && o.pos == -1);
      /* 둘째 덩어리 없이 넓어진 테이프(2칸 → 4칸): 1 s 는 폭 그대로(커브 판정 가능), 그 뒤 올려 배워 커브 판정이 꺼진다 */
      guide_reset();
      r = rx_narrow(BITS(7, 8));
      for (k = 0; k < 5; k++) { guide_track_drive(&r, 0, 0, &o); }
      r = rx_narrow(BITS(6, 9));
      for (k = 0; k < GUIDE_WIDE_LEARN_SCANS; k++) {
          guide_track_drive(&r, 0, 0, &o);
          assert(guide_tape_width() == 2 && o.width == 4 && o.width >= guide_curve_span() && o.track == GUIDE_TRK_NORMAL);
      }
      for (k = 0; k < 40; k++) { guide_track_drive(&r, 0, 0, &o); }
      assert(guide_tape_width() == 4 && o.width == 4 && o.width < guide_curve_span() && o.pos == 0);
      /* 다 배운 뒤 가끔 한 칸 넓어지는 것은 무시한다 */
      r = rx_narrow(BITS(6, 10));
      guide_track_drive(&r, 0, 0, &o);
      r = rx_narrow(BITS(6, 9));
      guide_track_drive(&r, 0, 0, &o);
      assert(guide_tape_width() == 4);
      /* 0.5 s 만 넓었다가 돌아오면(커브) 배우지 않는다. 세던 수도 버린다 */
      guide_reset();
      r = rx_narrow(BITS(7, 8));
      for (k = 0; k < 5; k++) { guide_track_drive(&r, 0, 0, &o); }
      { int j;
        for (j = 0; j < 3; j++) {
            r = rx_narrow(BITS(6, 9));
            for (k = 0; k < GUIDE_WIDE_LEARN_SCANS / 2; k++) { guide_track_drive(&r, 0, 0, &o); }
            r = rx_narrow(BITS(7, 8));
            guide_track_drive(&r, 0, 0, &o);
            assert(guide_tape_width() == 2 && guide_curve_span() == 4);
        } }
      /* 센서 끝에 반만 걸리면 안쪽 가장자리에서 폭만큼 잡는다 */
      guide_reset();
      r = rx_narrow(BITS(7, 10));
      for (k = 0; k < 5; k++) { guide_track_drive(&r, 0, 0, &o); }
      r = rx_narrow(BITS(0, 1));
      guide_track_drive(&r, 0, 0, &o);
      assert(o.pos == 0 + 2 - 17 - 1);     /* 칸 -1~2 로 본다 */
    }
}

#if AGV_DEV_TESTS
#include "agv/drive.h"
#include "agv/param.h"

/* 넓은 덩어리 유지 규칙(guide_hold_test). 응답 하나 = 10 ms, 달린 거리는 vin_out × one_rpm_meter / 6 */
static void hold_speed(float mm_per_scan)
{
    agv_drive.vin_out = 600;
    param_dv.one_rpm_meter = mm_per_scan / 100.0f;
}

typedef struct { uint32_t lo; int n; } hold_step_t;

static guide_rx_t hold_rx(int leds, uint32_t lo) { return (leds == 32) ? rx_wide(lo) : rx_narrow((uint16_t)lo); }

/* 2칸 테이프(tape)를 배운 뒤 seq 를 주고 마지막에 tape 로 돌아온다. 반환: 조향이 받는 위치(guide_poll 과 같게
 * 끈 것의 전부 켜짐은 0)와 처음 위치의 차 최대. 응답마다 10 mm(60 m/min) */
static int hold_seq(int leds, int hold, uint32_t tape, const hold_step_t *seq, int nseq)
{
    guide_out_t o;
    guide_rx_t r;
    int j, k, base, worst = 0;

    guide_reset();
    guide_hold_test = (int16_t)hold;
    hold_speed(10.0f);
    r = hold_rx(leds, tape);
    for (k = 0; k < 5; k++) { guide_track_drive(&r, 0, 0, &o); }
    assert(guide_tape_width() == 2 && o.track == GUIDE_TRK_NORMAL);
    base = o.pos;
    for (j = 0; j <= nseq; j++) {
        int n = (j < nseq) ? seq[j].n : 5;
        r = hold_rx(leds, (j < nseq) ? seq[j].lo : tape);
        for (k = 0; k < n; k++) {
            int p, d;
            guide_track_drive(&r, 0, 0, &o);
            assert(o.status != GUIDE_LOST);
            p = ((o.status == GUIDE_ALL_ON) && !hold) ? 0 : o.pos;
            d = (p > base) ? (p - base) : (base - p);
            if (d > worst) { worst = d; }
            /* 켠 것: 유지·옮기는 중(전부 켜짐 포함)은 NORMAL 이 아니다 → 위험 감속에 새로 들어가지 않는다 */
            if (hold && (d >= 10)) { assert(o.track != GUIDE_TRK_NORMAL); }
        }
    }
    assert(o.track == GUIDE_TRK_NORMAL && o.pos == base);
    guide_hold_test = 0;
    agv_drive.vin_out = 0;
    return worst;
}

static void test_hold(void)
{
    int w;
    for (w = 0; w < 2; w++) {
        int L = w ? 32 : 16, H = L / 2;
        uint32_t all = w ? 0xFFFFFFFFUL : 0xFFFFUL;
        uint32_t tape = BITS(H, H + 1);                        /* 칸 H+1·H+2: 위치 +2 */
        uint32_t lo_half = BITS(0, H - 1) | tape, hi_half = BITS(H, L - 1);
        uint32_t dead = all & ~BITS(4, 4);                     /* 5번 칸이 죽은 전부 켜짐 */
        int sl = w ? 6 : 3, sh = L - 1 - sl;                   /* 센서보다 짧은 교차 테이프: 가운데 L - 2×sl 칸 */
        const hold_step_t cross[] = { { lo_half, 2 }, { all, 3 }, { hi_half, 2 } };
        const hold_step_t deadc[] = { { lo_half, 2 }, { dead, 3 }, { hi_half, 2 } };
        const hold_step_t shortc[] = { { BITS(sl, H + 1), 2 }, { BITS(sl, sh), 3 }, { BITS(H, sh), 2 } };
        const char *nm = w ? "wide" : "일반";
        int off, on;

        off = hold_seq(L, 0, tape, cross, 3);
        on = hold_seq(L, 1, tape, cross, 3);
        printf("  유지 %s 직각 교차: 끈 것 %d, 켠 것 %d\n", nm, off, on);
        assert(on <= 1 && off == (w ? 16 : 8));
        off = hold_seq(L, 0, tape, deadc, 3);
        on = hold_seq(L, 1, tape, deadc, 3);
        printf("  유지 %s 죽은 칸 하나 전부 켜짐: 끈 것 %d, 켠 것 %d\n", nm, off, on);
        assert(on <= 1 && off >= 8);
        off = hold_seq(L, 0, tape, shortc, 3);
        on = hold_seq(L, 1, tape, shortc, 3);
        printf("  유지 %s 짧은 교차 테이프: 끈 것 %d, 켠 것 %d\n", nm, off, on);
        assert(on <= 1 && off >= 5);

        /* 25 mm 테이프로 바뀜(2칸 → 4칸, 한쪽으로): 100 mm 유지, 그 뒤 30 mm 당 1단위로 가운데까지, 다 오면 폭을 다시 배운다 */
        { guide_out_t o;
          guide_rx_t r;
          int k, j;
          for (j = 0; j < 2; j++) {
              guide_reset();
              guide_hold_test = (int16_t)j;
              hold_speed(10.0f);
              r = hold_rx(L, tape);
              for (k = 0; k < 5; k++) { guide_track_drive(&r, 0, 0, &o); }
              assert(o.pos == 2);
              r = hold_rx(L, BITS(H - 2, H + 1));               /* 칸 H-1~H+2: 가운데 0 */
              for (k = 1; k <= 200; k++) {
                  guide_track_drive(&r, 0, 0, &o);
                  if (!j) { assert(o.pos == 0 && o.track == GUIDE_TRK_NORMAL); continue; }   /* 끈 것: 바로 가운데(2단위 튐) */
                  /* 켠 것: k 응답 = 10k mm. 100 mm 까지 2, 130 mm 에 1, 160 mm 에 0 */
                  if (k <= 12)      { assert(o.pos == 2 && o.track == GUIDE_TRK_EDGE); }
                  else if (k <= 15) { assert(o.pos == 1 && o.track == GUIDE_TRK_EDGE); }
                  else              { assert(o.pos == 0); }
                  if (k >= 17)      { assert(o.track == GUIDE_TRK_NORMAL); }
              }
              /* 켠 것은 상한 풀림과 같은 느린 평균으로 배운다: 3칸에서 멈추지만 4칸이 배운 폭 + 1 안이라 평소 추적이다 */
              printf("  유지 %s 25 mm: %s 배운 폭 %u\n", nm, j ? "켠 것" : "끈 것", guide_tape_width());
              assert(guide_tape_width() == (j ? 3u : 4u) && o.track == GUIDE_TRK_NORMAL);
          }
          /* 커브(양쪽으로 한 칸씩)는 켠 것도 바로 가운데, NORMAL */
          guide_reset();
          guide_hold_test = 1;
          r = hold_rx(L, BITS(H + 2, H + 3));                  /* 위치 +6 */
          for (k = 0; k < 5; k++) { guide_track_drive(&r, 0, 0, &o); }
          r = hold_rx(L, BITS(H + 1, H + 4));
          guide_track_drive(&r, 0, 0, &o);
          assert(o.track == GUIDE_TRK_NORMAL && o.pos == 6 && o.width == 4);
          /* 두 가장자리가 다 움직여 넓어지면 둘째 덩어리를 본 적이 없어도 유지(끈 것은 커브로 보고 가운데) */
          r = hold_rx(L, BITS(H + 2, H + 3));
          guide_track_drive(&r, 0, 0, &o);
          r = hold_rx(L, BITS(H, H + 7));
          guide_track_drive(&r, 0, 0, &o);
          assert(o.track == GUIDE_TRK_HOLD && o.pos == 6);
          /* 원래 폭으로 돌아왔는데 차가 크다(+6 → +14): 같은 속도(30 mm 당 1단위)로 옮긴다. 그동안 EDGE */
          r = hold_rx(L, BITS(H + 6, H + 7));
          for (k = 1; k <= 24; k++) {
              guide_track_drive(&r, 0, 0, &o);
              assert(o.pos == 6 + k / 3);
              assert(o.track == ((k < 24) ? GUIDE_TRK_EDGE : GUIDE_TRK_NORMAL));
          }
          guide_track_drive(&r, 0, 0, &o);
          assert(o.pos == 14 && o.track == GUIDE_TRK_NORMAL);
          /* 유지 중 잃으면 유지를 버리고 다시 잡기(GUIDE_REACQ_N) 규칙대로 받는다 */
          r = hold_rx(L, BITS(H + 1, L - 1));
          guide_track_drive(&r, 0, 0, &o);
          assert(o.track != GUIDE_TRK_NORMAL && o.pos == 14);
          r = hold_rx(L, 0);
          guide_track_drive(&r, 0, 0, &o);
          r = hold_rx(L, tape);
          for (k = 1; k <= GUIDE_REACQ_N; k++) {
              guide_track_drive(&r, 0, 0, &o);
              assert((k < GUIDE_REACQ_N) ? (o.status == GUIDE_LOST) : (o.status == GUIDE_NORMAL && o.pos == 2 && o.track == GUIDE_TRK_NORMAL));
          }
          /* 전부 켜짐이 상한을 넘겨 이어지면 0(전부 켜짐의 가운데) 쪽으로 옮긴다 */
          r = hold_rx(L, all);
          for (k = 1; k <= 20; k++) {
              guide_track_drive(&r, 0, 0, &o);
              assert(o.status == GUIDE_ALL_ON && o.track != GUIDE_TRK_NORMAL && o.pos == ((k <= 12) ? 2 : (k <= 15) ? 1 : 0));
          }
          guide_hold_test = 0;
          agv_drive.vin_out = 0; }
    }
    /* 30도 교차선이 옆에서 다가와 붙었다 떨어짐: 켠 것도 참값 오차 3 이하. 거리 = 가로 이동 / tan30 */
    { static const float tape_mm[2] = { 10.0f, 25.0f };
      static const float step[3] = { 0.84f, 0.3f, 0.07f };
      int t, s, n, e;
      guide_hold_test = 1;
      for (w = 0; w < 2; w++) {
          for (t = 0; t < 2; t++) {
              for (s = 0; s < 3; s++) {
                  int leds = w ? 32 : 16;
                  float c0 = (float)leds / 2.0f + 0.5f;
                  hold_speed(step[s] * 6.89f / 0.577f);
                  e = cross_sim(leds, tape_mm[t], c0, step[s], &n);
                  if (s == 0) { printf("  유지 30도 교차 %s %.0f mm: 켠 것 최대 오차 %d\n", w ? "wide" : "일반", tape_mm[t], e); }
                  if (e > 3) { printf("  leds %d tape %.0f step %.2f: %d\n", leds, tape_mm[t], step[s], e); }
                  assert(e <= 3);
              }
          }
      }
      guide_hold_test = 0;
      agv_drive.vin_out = 0; }
    guide_reset();
    puts("  hold ok");
}
#endif

static void run_table(void)
{
    for (size_t k = 0; k < sizeof(s_cases) / sizeof(s_cases[0]); k++) {
        const track_case_t *c = &s_cases[k];
        guide_cfg_t cfg;
        guide_out_t o;
        guide_rx_t r;

        guide_reset();
        guide_cfg_default(&cfg);
        cfg.invert_fwd = c->invert;
        cfg.invert_back = c->invert;
        guide_configure(&cfg);
        if (c->prev_set) {
            r = c->wide ? rx_wide(c->prev_lo) : rx_narrow((uint16_t)c->prev_lo);
            guide_track_drive(&r, 0, 0, &o);
            assert(o.status == GUIDE_NORMAL);
        }
        cfg.min_width = c->min_w;
        cfg.max_width = c->max_w;
        cfg.max_step = c->max_step;
        guide_configure(&cfg);
        r = c->wide ? rx_wide(c->lo) : rx_narrow(0);
        if (!c->wide) { r.led_lo = c->lo; }
        guide_track_drive(&r, c->branch, c->backward, &o);
        if ((o.pos != c->pos) || (o.status != c->status) || (o.width != c->width)
            || (o.blobs != c->blobs)) {
            printf("%s: pos %d status %d width %u blobs %u\n", c->name, o.pos, o.status, o.width, o.blobs);
            assert(0);
        }
    }
}

int main(void)
{
    guide_rx_t r;
    guide_cfg_t cfg;
    guide_out_t o;
    guide_blob_t b[GUIDE_BLOB_MAX];
    uint16_t raw[32], n;

    fake_reset();
    fake_params();
    guide_reset();
    straight_set(1);    /* 아래 폭 학습 시험은 직진 중이다. 곡선은 test_straight */

    /* 'a' */
    parse("a+05", &r);
    assert(r.kind == GUIDE_RX_POSITION && r.pos == 5 && !r.wide && r.status == GUIDE_NORMAL);
    parse("a-15", &r);
    assert(r.pos == -15 && r.status == GUIDE_NORMAL);         /* -15 는 정상값 */
    parse("a+17", &r);
    assert(r.status == GUIDE_LOST && r.pos == 17);
    parse("a+16", &r);
    assert(r.status == GUIDE_ALL_ON && r.pos == 0);
    parse("a-16", &r);
    assert(r.status == GUIDE_ALL_ON && r.pos == 0);
    assert(r.vert == 0);
    /* wide 'a': 부호 십 일 V(세로 센서). 사양서 예시 */
    parse("a+050", &r);
    assert(r.kind == GUIDE_RX_POSITION && r.wide && r.pos == 5 && r.status == GUIDE_NORMAL && r.vert == 0);
    parse("a+051", &r);
    assert(r.kind == GUIDE_RX_POSITION && r.wide && r.pos == 5 && r.vert == 1);
    parse("a-120", &r);
    assert(r.kind == GUIDE_RX_POSITION && r.pos == -12 && r.status == GUIDE_NORMAL);
    parse("a+320", &r);
    assert(r.kind == GUIDE_RX_POSITION && r.status == GUIDE_ALL_ON && r.pos == 0);
    parse("a+210", &r);
    assert(r.wide && r.pos == 21 && r.status == GUIDE_NORMAL);
    parse("a+330", &r);
    assert(r.wide && r.status == GUIDE_LOST);
    parse("a-311", &r);
    assert(r.status == GUIDE_NORMAL && r.pos == -31 && r.vert == 1);
    parse("a+05x", &r);
    assert(r.kind == GUIDE_RX_BAD_FORMAT);                    /* V 는 '0'·'1' 만 */
    parse("a+0501", &r);
    assert(r.kind == GUIDE_RX_BAD_FORMAT);                    /* 7·8 바이트가 아니다 */
    parse("a*05", &r);
    assert(r.kind == GUIDE_RX_BAD_FORMAT);
    parse("a+0x", &r);
    assert(r.kind == GUIDE_RX_BAD_FORMAT);

    /* 'h': 글자 순서와 비트 순서가 반대다. [2] 가 비트 0~3 */
    parse("h1234", &r);
    assert(r.kind == GUIDE_RX_LED && !r.wide && r.led_lo == 0x4321UL);
    parse("habcd", &r);
    assert(r.led_lo == 0xDCBAUL);
    parse("hABCD", &r);
    assert(r.led_lo == 0xDCBAUL);
    parse("h12345678", &r);                                   /* wide 사양: 16진 8자리, 세로 없음 */
    assert(r.wide && r.led_lo == 0x87654321UL);
    n = make(raw, "hd945a376", 0);                            /* 사양서 예시: 체크섬 'f' */
    assert(n == 12 && raw[n - 2] == 'f');
    guide_parse(raw, n, &r);
    assert(r.kind == GUIDE_RX_LED && r.wide && r.led_lo == 0x673A549DUL);
    {   /* 켜진 LED 1,3,4,5,8,11,13,15,18,20,21,22,25,26,27,30,31 */
        static const uint16_t on[] = { 1, 3, 4, 5, 8, 11, 13, 15, 18, 20, 21, 22, 25, 26, 27, 30, 31 };
        uint32_t want = 0;
        for (n = 0; n < sizeof(on) / sizeof(on[0]); n++) { want |= 1UL << (on[n] - 1u); }
        assert(guide_lateral(&r) == want);
    }
    parse("h123456789", &r);                                  /* 9자리는 없다 */
    assert(r.kind == GUIDE_RX_BAD_FORMAT);
    parse("h12345", &r);
    assert(r.kind == GUIDE_RX_BAD_FORMAT);                    /* 길이가 둘 다 아니다 */
    parse("h12x4", &r);
    assert(r.kind == GUIDE_RX_BAD_FORMAT);
    parse("Sxx", &r);
    assert(r.kind == GUIDE_RX_SENSOR_ERR);
    parse("Zxx", &r);
    assert(r.kind == GUIDE_RX_NONE);

    /* 체크섬: A~F 는 소문자로 비교. 대문자·틀린 값은 버린다 */
    n = make(raw, "h1234", 1);
    guide_parse(raw, n, &r);
    assert(r.kind == GUIDE_RX_BAD_BCC);
    n = make(raw, "a+00", 0);                                 /* BCC 가 'c' 인 응답 */
    assert(raw[n - 2] == 'c');
    guide_parse(raw, n, &r);
    assert(r.kind == GUIDE_RX_POSITION && r.pos == 0);
    raw[n - 2] = 'C';
    guide_parse(raw, n, &r);
    assert(r.kind == GUIDE_RX_BAD_BCC);
    guide_parse(raw, 2, &r);
    assert(r.kind == GUIDE_RX_BAD_FORMAT);

    /* 덩어리 목록 */
    assert(guide_blobs(0, 16, b) == 0);
    assert(guide_blobs(0xFFFFUL, 16, b) == 1 && b[0].start == 1 && b[0].end == 16);
    assert(guide_blobs(0xFFFFFFFFUL, 32, b) == 1 && b[0].end == 32);
    assert(guide_blobs(0x80000001UL, 32, b) == 2 && b[0].end == 1 && b[1].start == 32 && b[1].end == 32);
    assert(guide_blobs(0x55555555UL, 32, b) == GUIDE_BLOB_MAX);
    assert(guide_blobs(0xFFFF0000UL, 16, b) == 0);

    run_table();
    test_cross();
#if AGV_DEV_TESTS
    test_hold();        /* 직진 중(straight_set) 폭 학습이 필요하다. test_straight 앞 */
#endif
    test_straight();

    /* LOST 뒤에는 변화량 제한 없이 다시 잡는다(지속 확인 뒤) */
    guide_reset();
    guide_cfg_default(&cfg);
    guide_configure(&cfg);
    r = rx_narrow(0x0018);
    guide_track_drive(&r, 0, 0, &o);
    cfg.max_step = 2;
    guide_configure(&cfg);
    r = rx_narrow(0x3000);
    guide_track_drive(&r, 0, 0, &o);
    assert(o.status == GUIDE_LOST && o.pos == -8);
    guide_track_drive(&r, 0, 0, &o);       /* 주행 경로는 같은 자리에 GUIDE_REACQ_N 응답을 본 뒤 받는다 */
    guide_track_drive(&r, 0, 0, &o);
    assert(o.status == GUIDE_LOST && o.pos == -8);
    guide_track_drive(&r, 0, 0, &o);
    assert(o.status == GUIDE_NORMAL && o.pos == 10);

    /* ALL_ON 뒤에는 위치가 유지되어 다음 덩어리를 그 기준으로 고른다 */
    guide_reset();
    r = rx_narrow(0xC000);
    guide_track_drive(&r, 0, 0, &o);
    r = rx_narrow(0xFFFF);
    guide_track_drive(&r, 0, 0, &o);
    assert(o.pos == 14 && o.status == GUIDE_ALL_ON);
    r = rx_narrow(0xC003);
    guide_track_drive(&r, 0, 0, &o);
    assert(o.pos == 14);

    /* 'a' 소스에서 온 'h': 위치를 덮지 않는다. 비트를 켜면 자리표 10 */
    guide_reset();
    r = rx_narrow(0x0FF0);
    assert(guide_h_position(&r, 5) == 5);
    r = rx_narrow(0x0000);
    assert(guide_h_position(&r, 5) == 17);
    r = rx_narrow(0xFFFF);
    assert(guide_h_position(&r, 5) == 16);
    r = rx_wide(0);
    assert(guide_h_position(&r, 5) == 33);
    r = rx_wide(0xFFFFFFFFUL);
    assert(guide_h_position(&r, 5) == 32);
    r = rx_narrow(0x0FF0);
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_H_POSITION);
    assert(guide_h_position(&r, 5) == 10);
    (void)param_set(P_LEGACY_QUIRKS, 0);

    /* invert 는 센서별이다 */
    guide_reset();
    guide_cfg_default(&cfg);
    cfg.invert_back = 1;
    guide_configure(&cfg);
    r = rx_narrow(0x0006);                                     /* 위치 -12 */
    guide_track_drive(&r, 0, 0, &o);
    assert(o.pos == -12);
    guide_track_drive(&r, 0, 1, &o);
    assert(o.pos == 12);
    guide_reset();

    /* 센서 전환: 앞 센서의 마지막 위치로 뒤 센서 덩어리를 고르지 않는다 */
    r = rx_narrow(0xC000);                                     /* 앞 센서 위치 14 */
    guide_track_drive(&r, 0, 0, &o);
    assert(o.pos == 14);
    r = rx_narrow(0xC180);                                     /* 뒤 센서: 14 와 중앙(0) 두 덩어리 */
    guide_on_sensor_switch();
    guide_track_drive(&r, 0, 1, &o);
    assert(o.pos == 0 && o.width == 2);
    /* 전환 뒤 변화량 제한도 첫 값에는 걸지 않는다 */
    guide_cfg_default(&cfg);
    cfg.max_step = 1;
    guide_configure(&cfg);
    r = rx_narrow(0x0003);
    guide_on_sensor_switch();
    guide_track_drive(&r, 0, 1, &o);
    assert(o.status == GUIDE_NORMAL && o.pos == -14);
    guide_reset();

    /* 바이트 파서: 전환 직후 잡음 뒤 정상 복구 */
    { static const uint16_t noise[] = { 0x00, 0xFF, 0x03, 0x41, 0x02, 0x61, 0x02, 0x02, 0x7F, 0x03, 0x06, 0x03, 0x80, 0x02 };
      uint16_t f[20];
      int got = 0;
      for (unsigned i = 0; i < sizeof(noise) / sizeof(noise[0]); i++) {
          if (guide_feed(noise[i], &r)) { assert(r.kind != GUIDE_RX_POSITION && r.kind != GUIDE_RX_LED); }
      }
      assert(guide_comm_error() == 0);
      /* 끝에 STX 가 걸린 채 끊겨 있어도 다음 STX 에서 다시 시작한다 */
      n = make(f, "a+05", 0);
      for (uint16_t i = 0; i < n; i++) { got += guide_feed(f[i], &r); }
      assert(got == 1 && r.kind == GUIDE_RX_POSITION && r.pos == 5);
      /* 패킷 도중 전환: 자른 앞부분이 다음 프레임에 섞이지 않는다 */
      for (uint16_t i = 0; i < 3; i++) { (void)guide_feed(f[i], &r); }
      guide_on_sensor_switch();
      got = 0;
      for (uint16_t i = 3; i < n; i++) { got += guide_feed(f[i], &r); }
      assert(got == 0);
      for (uint16_t i = 0; i < n; i++) { got += guide_feed(f[i], &r); }
      assert(got == 1 && r.pos == 5);
      /* 넘침 */
      guide_on_sensor_switch();
      (void)guide_feed(2, &r);
      for (int i = 0; i < 40; i++) { assert(guide_feed('1', &r) == 0); }
      got = 0;
      for (uint16_t i = 0; i < n; i++) { got += guide_feed(f[i], &r); }
      assert(got == 1 && r.kind == GUIDE_RX_POSITION); }

    /* 두절: 첫 유효 응답 전에는 판정하지 않는다 */
    { uint16_t f[20], bad[20], nb;
      n = make(f, "a+05", 0);
      nb = make(bad, "a+05", 1);
#define FEED_FRAME(p, c) do { for (uint16_t k_ = 0; k_ < (c); k_++) { (void)guide_feed((p)[k_], &r); } } while (0)

      guide_reset();
      for (int i = 0; i < 100; i++) { assert(guide_on_request() == 0); }
      for (int i = 0; i < 30; i++) { FEED_FRAME(bad, nb); }    /* 깨진 프레임도 판정 전이다 */
      assert(guide_comm_error() == 0);

      /* 유효 응답 뒤 무응답 30 ms: 요청 4번째에서 세 번 놓친 것이 확정된다 */
      FEED_FRAME(f, n);
      assert(guide_on_request() == 0);
      assert(guide_on_request() == 0);
      assert(guide_on_request() == 0);
      assert(guide_on_request() == ERR_GUIDE_COMM);

      /* 리셋은 오류만 지운다. 센서가 죽은 채면 감시가 다시 잡는다 */
      guide_comm_clear();
      assert(guide_comm_error() == 0);
      assert(guide_on_request() == 0);
      assert(guide_on_request() == 0);
      assert(guide_on_request() == 0);
      assert(guide_on_request() == ERR_GUIDE_COMM);

      /* 전환하면 오류와 판정이 함께 초기화되고, 전환 중 무응답은 두절이 아니다 */
      guide_on_sensor_switch();
      assert(guide_comm_error() == 0);
      for (int i = 0; i < 100; i++) { assert(guide_on_request() == 0); }
      FEED_FRAME(f, n);

      /* 요청이 없으면 시간이 가도 쌓이지 않고, 응답이 오면 다시 0 부터 센다 */
      assert(guide_on_request() == 0);
      FEED_FRAME(f, n);
      assert(guide_on_request() == 0);
      assert(guide_on_request() == 0);
      FEED_FRAME(f, n);
      assert(guide_on_request() == 0);
      assert(guide_on_request() == 0);
      assert(guide_on_request() == 0);
      assert(guide_comm_error() == 0);

      /* 주행 시작도 같은 대기 구간이다 */
      guide_on_run_start();
      for (int i = 0; i < 10; i++) { assert(guide_on_request() == 0); }
      FEED_FRAME(f, n);

      /* P_GUIDE_TIMEOUT 을 주면 그 횟수를 쓴다 */
      (void)param_set(P_GUIDE_TIMEOUT, 5);
      for (int i = 0; i < 5; i++) { assert(guide_on_request() == 0); }
      assert(guide_on_request() == ERR_GUIDE_COMM);
      (void)param_set(P_GUIDE_TIMEOUT, 0);

      /* 체크섬 연속 실패 10회. 깨진 프레임은 무응답으로 세지 않는다 */
      guide_on_sensor_switch();
      FEED_FRAME(f, n);
      for (int i = 0; i < 9; i++) {
          assert(guide_on_request() == 0);
          FEED_FRAME(bad, nb);
      }
      assert(guide_comm_error() == 0);
      FEED_FRAME(f, n);                                         /* 중간에 정상이 오면 다시 센다 */
      for (int i = 0; i < 9; i++) { FEED_FRAME(bad, nb); }
      assert(guide_comm_error() == 0);
      FEED_FRAME(bad, nb);
      assert(guide_comm_error() == ERR_GUIDE_COMM); }
    guide_reset();

    /* 소스 선택: 기본 'h' 전용, GSR 값은 무시, QUIRK_SRC_A 로 예전 'a' */
    assert(guide_source_led() && guide_want_h(0, 0) && guide_want_h(1, 3));
    assert(param_set(P_GUIDE_CURVE_DETECT, 0) == 0 && guide_source_led());
    assert(param_set(P_GUIDE_CURVE_DETECT, 3) == -1);     /* 범위표는 그대로 */
    assert(param_set(P_LEGACY_QUIRKS, QUIRK_SRC_A) == 0);
    assert(!guide_source_led());
    assert(!guide_want_h(0, 0) && guide_want_h(1, 0) && !guide_want_h(1, 3));
    assert(param_set(P_LEGACY_QUIRKS, 0) == 0);

    test_reacq();
    puts("test_guide ok");
    return 0;
}
