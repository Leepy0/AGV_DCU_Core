/* test_gain.c - t1 조향 이득 찾기 판정(gain_judge)과 후보 순서(gain_seq). 하드웨어 없이 gain.c 만 링크한다 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "agv/gain.h"
#include "agv/guide.h"

static uint32_t s_rng = 12345u;
static float rnd(void)      /* -1 ~ +1 */
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return (float)((s_rng >> 8) & 0xFFFFu) / 32768.0f - 1.0f;
}

static int16_t q8(float u) { return (int16_t)lroundf(u * (float)GAIN_SCALE); }

/* 위치(단위)를 만드는 방법 */
typedef float (*gen_t)(int t);
static int s_n = 400;
static float s_a, s_hz, s_bias, s_noise;

static float g_sine(int t)     { return s_bias + s_a * sinf(6.2831853f * s_hz * (float)t * 0.01f) + s_noise * rnd(); }
static float g_growing(int t)  { return s_bias + (1.5f + 3.0f * (float)t / (float)s_n) * sinf(6.2831853f * s_hz * (float)t * 0.01f) + s_noise * rnd(); }
static float g_jitter(int t)   { (void)t; return (rnd() >= 0.0f) ? 1.0f : -1.0f; }

static void judge(gen_t g, int n, gain_log_t *r)
{
    static int16_t buf[GAIN_WIN_MAX];
    int t;
    for (t = 0; t < n; t++) { buf[t] = q8(g(t)); }
    gain_judge(buf, (int16_t)n, r);
}

static void show(const char *name, const gain_log_t *r)
{
    printf("  %-34s RMS %4.1f mm  봉우리 %4.1f  부호 바뀜 %3d  판정 %d\n", name, r->rms10 / 10.0f, r->peak10 / 10.0f, r->flips, r->verdict);
}

static void test_judge(void)
{
    gain_log_t r;

    /* 잡음·치우침만 있는 직선 = 깨끗 */
    s_bias = 3.0f; s_noise = 0.7f; s_a = 0.0f; s_hz = 1.0f;
    judge(g_sine, 400, &r);
    show("잡음 ±0.7 + 치우침 3", &r);
    assert(r.verdict == GAIN_V_CLEAN && r.flips == 0 && r.rms10 < 60);

    /* 1.4 Hz 로 커지는 ±1.5~4.5 단위 사행 = 탈락 */
    s_bias = 0.0f; s_noise = 0.2f; s_hz = 1.4f;
    judge(g_growing, 400, &r);
    show("1.4 Hz 커지는 사행", &r);
    assert(r.verdict != GAIN_V_CLEAN && r.flips >= 8 && r.peak10 >= 20);

    /* 반주기 길이·봉우리 기준: 일정 ±2.4 단위 1.4 Hz 는 RMS 5.8 mm(통과)지만 사행이라 탈락 */
    s_a = 2.4f; s_noise = 0.0f;
    judge(g_sine, 400, &r);
    show("1.4 Hz 일정 ±2.4 (RMS 만으론 통과)", &r);
    assert(r.verdict == GAIN_V_WOBBLE && r.rms10 <= 60 && r.flips >= 9 && r.peak10 >= 20 && r.peak10 <= 26);

    /* 같은 진폭이 0.2 s 반주기 밖(3 Hz, 반주기 0.167 s)이거나 1 s 밖(0.4 Hz, 반주기 1.25 s)이면 사행이 아니다 */
    s_hz = 3.0f;
    judge(g_sine, 400, &r);
    show("3 Hz ±2.4 (반주기 0.17 s)", &r);
    assert(r.verdict == GAIN_V_CLEAN && r.flips >= 8);
    s_hz = 0.4f;
    judge(g_sine, 400, &r);
    show("0.4 Hz ±2.4 (반주기 1.25 s)", &r);
    assert(r.verdict == GAIN_V_CLEAN);
    s_hz = 0.7f;                                  /* 반주기 0.71 s 는 사행 */
    judge(g_sine, 400, &r);
    assert(r.verdict == GAIN_V_WOBBLE);

    /* 0.1 Hz 느린 흔들림: 사행이 아니다(크면 RMS 만 탈락) */
    s_hz = 0.1f; s_a = 2.0f;
    judge(g_sine, 400, &r);
    show("0.1 Hz ±2", &r);
    assert(r.verdict == GAIN_V_CLEAN && r.flips <= 1);
    s_a = 8.0f;
    judge(g_sine, 400, &r);
    show("0.1 Hz ±8 (크면 RMS 만 탈락)", &r);
    assert(r.verdict == GAIN_V_RMS && r.flips <= 1);

    /* ±1 단위 떨림: 사행이 아니다. 매 틱 ±1 로 뒤집히고 평균이 한쪽으로 조금 치우쳐도 반주기가 1 틱이라 안 센다 */
    judge(g_jitter, 400, &r);
    show("±1 단위 떨림(매 틱)", &r);
    assert(r.verdict == GAIN_V_CLEAN);
    s_bias = 0.0f; s_a = 1.0f; s_hz = 1.4f; s_noise = 0.0f;       /* 정확히 ±1 인 사인: 이력 ±1 을 넘지 않는다 */
    judge(g_sine, 400, &r);
    assert(r.verdict == GAIN_V_CLEAN && r.flips == 0);

    /* 창이 짧으면 판정하지 않는다 */
    s_a = 3.0f;
    judge(g_sine, GAIN_WIN_MIN - 1, &r);
    assert(r.verdict == GAIN_V_NONE && r.rms10 == 0 && r.flips == 0);
    judge(g_sine, GAIN_WIN_MIN, &r);
    assert(r.verdict != GAIN_V_NONE);

    /* RMS 기준 6.0 mm = 1.742 단위: 일정 잡음 폭으로 양쪽을 본다(사인이 아니라 사행 없음) */
    s_a = 0.0f; s_hz = 1.0f; s_bias = 0.0f;
    s_noise = 1.7f * 1.732f;     /* 균일 ±a 의 RMS = a/√3 → 1.7 단위 = 5.86 mm */
    judge(g_sine, 400, &r);
    show("잡음 RMS 5.9 mm", &r);
    assert(r.rms10 >= 55 && r.rms10 <= 62 && r.verdict == GAIN_V_CLEAN);
    s_noise = 1.85f * 1.732f;    /* 6.4 mm */
    judge(g_sine, 400, &r);
    show("잡음 RMS 6.4 mm", &r);
    assert(r.rms10 >= 62 && r.verdict == GAIN_V_RMS);

    /* 표본이 튀어도(선을 잃음 ±500 단위) 정수가 넘치지 않는다 */
    {
        int16_t buf[GAIN_WIN_MAX];
        int t;
        for (t = 0; t < 400; t++) { buf[t] = (int16_t)((t & 1) ? 4000 : -4000); }
        gain_judge(buf, 400, &r);
        assert(r.verdict == GAIN_V_BOTH || r.verdict == GAIN_V_RMS);
    }
}

static void test_seq(void)
{
    gain_seq_t q;
    static const int16_t kp[GAIN_CANDS] = { 60, 50, 40, 30, 25, 20 }, ki[GAIN_CANDS] = { 5, 4, 3, 3, 2, 2 };
    int k, j;

    /* K_I = max(1, 반올림(K_P × 5 / 60)) */
    gain_seq_begin(&q);
    for (k = 0; k < GAIN_CANDS; k++) {
        q.idx = (int16_t)k;
        assert(gain_kp(&q) == kp[k] && gain_ki(&q) == ki[k]);
    }
    /* 60·50·40 탈락, 30 깨끗 → 30·3 */
    gain_seq_begin(&q);
    for (k = 0; k < 3; k++) { assert(gain_kp(&q) == kp[k]); gain_seq_next(&q, 0); assert(!q.done); }
    assert(gain_kp(&q) == 30 && gain_ki(&q) == 3);
    gain_seq_next(&q, 1);
    assert(q.done && !q.ng && gain_kp(&q) == 30 && gain_ki(&q) == 3);
    /* 60 에서 깨끗 → 한 번에 끝 */
    gain_seq_begin(&q);
    gain_seq_next(&q, 1);
    assert(q.done && !q.ng && gain_kp(&q) == 60 && gain_ki(&q) == 5);
    /* 앞에서 멈추는 모든 자리 */
    for (j = 0; j < GAIN_CANDS; j++) {
        gain_seq_begin(&q);
        for (k = 0; k < j; k++) { gain_seq_next(&q, 0); }
        gain_seq_next(&q, 1);
        assert(q.done && !q.ng && gain_kp(&q) == kp[j] && gain_ki(&q) == ki[j]);
    }
    /* 전부 탈락 → 20·2, NG */
    gain_seq_begin(&q);
    for (k = 0; k < GAIN_CANDS; k++) { assert(!q.done); gain_seq_next(&q, 0); }
    assert(q.done && q.ng && gain_kp(&q) == 20 && gain_ki(&q) == 2);
    /* 마지막 후보가 깨끗하면 NG 가 아니다 */
    gain_seq_begin(&q);
    for (k = 0; k < GAIN_CANDS - 1; k++) { gain_seq_next(&q, 0); }
    gain_seq_next(&q, 1);
    assert(q.done && !q.ng && gain_kp(&q) == 20);
}

int main(void)
{
    setvbuf(stdout, 0, _IONBF, 0);
    test_judge();
    test_seq();
    printf("test_gain ok\n");
    return 0;
}
