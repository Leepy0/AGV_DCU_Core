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
        guide_track(&r, 0, 0, &o);
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
    /* 상한을 넘겨 풀어 주면 넓어진 덩어리를 그대로 받고 폭을 다시 배운다 */
    { guide_out_t o;
      guide_rx_t r;
      int k;
      guide_reset();
      r = rx_narrow(BITS(7, 8));
      for (k = 0; k < 5; k++) { guide_track(&r, 0, 0, &o); }
      assert(guide_tape_width() == 2 && o.track == GUIDE_TRK_NORMAL && guide_curve_span() == 4);
      r = rx_narrow(BITS(7, 11));
      guide_track(&r, 0, 0, &o);
      assert(o.track == GUIDE_TRK_EDGE && o.pos == 0 && o.width == 2);
      r = rx_narrow(BITS(3, 12));
      guide_track(&r, 0, 0, &o);
      assert(o.track == GUIDE_TRK_HOLD && o.pos == 0);
      guide_track_release();
      guide_track(&r, 0, 0, &o);
      assert(o.track == GUIDE_TRK_NORMAL && o.pos == 0 && guide_tape_width() == 10 && guide_curve_span() == 12);
      /* 센서 끝에 반만 걸리면 안쪽 가장자리에서 폭만큼 잡는다 */
      guide_reset();
      r = rx_narrow(BITS(7, 10));
      for (k = 0; k < 5; k++) { guide_track(&r, 0, 0, &o); }
      r = rx_narrow(BITS(0, 1));
      guide_track(&r, 0, 0, &o);
      assert(o.pos == 0 + 2 - 17 - 1);     /* 칸 -1~2 로 본다 */
    }
}

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
            guide_track(&r, 0, 0, &o);
            assert(o.status == GUIDE_NORMAL);
        }
        cfg.min_width = c->min_w;
        cfg.max_width = c->max_w;
        cfg.max_step = c->max_step;
        guide_configure(&cfg);
        r = c->wide ? rx_wide(c->lo) : rx_narrow(0);
        if (!c->wide) { r.led_lo = c->lo; }
        guide_track(&r, c->branch, c->backward, &o);
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

    /* LOST 뒤에는 변화량 제한 없이 다시 잡는다 */
    guide_reset();
    guide_cfg_default(&cfg);
    guide_configure(&cfg);
    r = rx_narrow(0x0018);
    guide_track(&r, 0, 0, &o);
    cfg.max_step = 2;
    guide_configure(&cfg);
    r = rx_narrow(0x3000);
    guide_track(&r, 0, 0, &o);
    assert(o.status == GUIDE_LOST && o.pos == -8);
    guide_track(&r, 0, 0, &o);
    assert(o.status == GUIDE_NORMAL && o.pos == 10);

    /* ALL_ON 뒤에는 위치가 유지되어 다음 덩어리를 그 기준으로 고른다 */
    guide_reset();
    r = rx_narrow(0xC000);
    guide_track(&r, 0, 0, &o);
    r = rx_narrow(0xFFFF);
    guide_track(&r, 0, 0, &o);
    assert(o.pos == 14 && o.status == GUIDE_ALL_ON);
    r = rx_narrow(0xC003);
    guide_track(&r, 0, 0, &o);
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
    guide_track(&r, 0, 0, &o);
    assert(o.pos == -12);
    guide_track(&r, 0, 1, &o);
    assert(o.pos == 12);
    guide_reset();

    /* 센서 전환: 앞 센서의 마지막 위치로 뒤 센서 덩어리를 고르지 않는다 */
    r = rx_narrow(0xC000);                                     /* 앞 센서 위치 14 */
    guide_track(&r, 0, 0, &o);
    assert(o.pos == 14);
    r = rx_narrow(0xC180);                                     /* 뒤 센서: 14 와 중앙(0) 두 덩어리 */
    guide_on_sensor_switch();
    guide_track(&r, 0, 1, &o);
    assert(o.pos == 0 && o.width == 2);
    /* 전환 뒤 변화량 제한도 첫 값에는 걸지 않는다 */
    guide_cfg_default(&cfg);
    cfg.max_step = 1;
    guide_configure(&cfg);
    r = rx_narrow(0x0003);
    guide_on_sensor_switch();
    guide_track(&r, 0, 1, &o);
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

    /* 소스 선택 */
    assert(!guide_source_led());
    assert(!guide_want_h(0, 0) && guide_want_h(1, 0) && !guide_want_h(1, 3));
    assert(param_set(P_GUIDE_CURVE_DETECT, GUIDE_SRC_LED_ONLY) == 0);
    assert(guide_source_led() && guide_want_h(0, 0) && guide_want_h(1, 3));
    assert(param_set(P_GUIDE_CURVE_DETECT, 3) == -1);

    puts("test_guide ok");
    return 0;
}
