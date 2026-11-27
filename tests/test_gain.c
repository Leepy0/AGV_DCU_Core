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

/* 위치(단위)를 만드는 방법. t = 표본 번호(20 ms) */
#define TS 0.02f
typedef float (*gen_t)(int t);
static int s_n = GAIN_N_MAX;
static float s_a, s_hz, s_bias, s_noise;

static float g_sine(int t)     { return s_bias + s_a * sinf(6.2831853f * s_hz * (float)t * TS) + s_noise * rnd(); }
static float g_growing(int t)  { return s_bias + (1.5f + 3.0f * (float)t / (float)s_n) * sinf(6.2831853f * s_hz * (float)t * TS) + s_noise * rnd(); }
static float g_jitter(int t)   { (void)t; return (rnd() >= 0.0f) ? 1.0f : -1.0f; }
/* 출발 직후 과도: 0.7 Hz ±4 단위가 시정수 2 s 로 가라앉는다(봉우리 1.75 단위를 넘는 온전한 반주기는 1개) */
static float g_decay(int t)    { float x = (float)t * TS; return 4.0f * expf(-x / 2.0f) * sinf(6.2831853f * 0.7f * x) + s_noise * rnd(); }
/* 불규칙 반주기 사행: 반주기마다 부호를 바꾼 반 사인 */
static float g_irregular(int t)
{
    static const float h[] = { 0.73f, 1.1f, 0.93f, 1.37f, 1.1f, 0.9f, 1.2f, 1.0f };
    float x = (float)t * TS, t0 = 0.0f;
    int k;
    for (k = 0; k < (int)(sizeof(h) / sizeof(h[0])); k++) {
        if (x < t0 + h[k]) { return ((k & 1) ? -s_a : s_a) * sinf(3.1415927f * (x - t0) / h[k]); }
        t0 += h[k];
    }
    return 0.0f;
}

static void judge(gen_t g, int n, gain_log_t *r)
{
    static int16_t buf[GAIN_N_MAX];
    int t;
    for (t = 0; t < n; t++) { buf[t] = q8(g(t)); }
    gain_judge(buf, (int16_t)n, r);
}

static void show(const char *name, const gain_log_t *r)
{
    printf("  %-34s RMS %4.1f mm  봉우리 %4.1f  부호 바뀜 %3d  판정 %d\n", name, r->rms10 / 10.0f, r->peak10 / 10.0f, r->flips, r->verdict);
}

/* 일정 사인 ±a 단위 f Hz 를 8 s(표본 400개)로 판정 */
static void sine(float a, float hz, gain_log_t *r)
{
    s_bias = 0.0f; s_noise = 0.0f; s_a = a; s_hz = hz;
    judge(g_sine, GAIN_N_MAX, r);
}

static void test_judge(void)
{
    gain_log_t r;

    /* 잡음·치우침만 있는 직선 = 깨끗 */
    s_bias = 3.0f; s_noise = 0.7f; s_a = 0.0f; s_hz = 1.0f;
    judge(g_sine, GAIN_N_MAX, &r);
    show("잡음 ±0.7 + 치우침 3", &r);
    assert(r.verdict == GAIN_V_CLEAN && r.flips == 0 && r.rms10 < 60);

    /* 1.4 Hz 로 커지는 ±1.5~4.5 단위 사행 = 탈락 */
    s_bias = 0.0f; s_noise = 0.2f; s_hz = 1.4f;
    judge(g_growing, GAIN_N_MAX, &r);
    show("1.4 Hz 커지는 사행", &r);
    assert(r.verdict != GAIN_V_CLEAN && r.flips >= 8 && r.peak10 >= 20);

    /* 느린 사행(실차 0.5 Hz, 반주기 0.7~1.4 s) = 탈락. RMS 는 6 mm 안팎이라 사행 판정이 잡아야 한다 */
    sine(2.0f, 0.5f, &r);
    show("0.5 Hz ±2 (반주기 1 s)", &r);
    assert((r.verdict == GAIN_V_WOBBLE) && (r.flips >= 6));
    sine(2.0f, 0.36f, &r);
    show("0.36 Hz ±2 (반주기 1.39 s)", &r);
    assert(r.verdict == GAIN_V_WOBBLE);
    sine(2.0f, 0.7f, &r);
    show("0.7 Hz ±2 (반주기 0.71 s)", &r);
    assert(r.verdict == GAIN_V_WOBBLE);
    sine(2.4f, 0.4f, &r);                         /* 예전(반주기 1 s 까지)에는 사행 아님이던 0.4 Hz */
    show("0.4 Hz ±2.4 (반주기 1.25 s)", &r);
    assert(r.verdict == GAIN_V_WOBBLE);
    s_a = 2.0f;
    judge(g_irregular, GAIN_N_MAX, &r);
    show("불규칙 반주기 0.73~1.37 s ±2", &r);
    assert(r.verdict == GAIN_V_WOBBLE && r.flips >= 6);

    /* 반주기 길이·봉우리 기준: 일정 ±2.4 단위 1.4 Hz 는 RMS 5.8 mm(통과)지만 사행이라 탈락 */
    sine(2.4f, 1.4f, &r);
    show("1.4 Hz 일정 ±2.4 (RMS 만으론 통과)", &r);
    assert(r.verdict == GAIN_V_WOBBLE && r.rms10 <= 60 && r.flips >= 18 && r.peak10 >= 20 && r.peak10 <= 26);

    /* 같은 진폭이 0.2 s 반주기 밖(3 Hz, 반주기 0.167 s)이면 사행이 아니다 */
    sine(2.4f, 3.0f, &r);
    show("3 Hz ±2.4 (반주기 0.17 s)", &r);
    assert(r.verdict == GAIN_V_CLEAN && r.flips >= 8);

    /* 0.1 Hz 느린 흔들림(반주기 5 s): 사행이 아니다(크면 RMS 만 탈락) */
    sine(2.0f, 0.1f, &r);
    show("0.1 Hz ±2", &r);
    assert(r.verdict == GAIN_V_CLEAN && r.flips <= 1);
    sine(8.0f, 0.1f, &r);
    show("0.1 Hz ±8 (크면 RMS 만 탈락)", &r);
    assert(r.verdict == GAIN_V_RMS && r.flips <= 2);    /* 8 s 창은 0.8 주기라 창 평균을 빼면 두 번 바뀔 수 있다 */

    /* 봉우리 기준 1.75 단위: ±1.6 단위 0.5 Hz 는 반주기는 맞아도 봉우리가 모자라 사행이 아니다 */
    sine(1.6f, 0.5f, &r);
    show("0.5 Hz ±1.6 (봉우리 모자람)", &r);
    assert(r.verdict == GAIN_V_CLEAN && r.flips >= 6);

    /* 출발 직후 두세 번 흔들리고 가라앉는 과도: 온전한 반주기 3개 이하라 사행이 아니다 */
    s_noise = 0.3f;
    judge(g_decay, GAIN_N_MAX, &r);
    show("출발 과도(0.7 Hz ±4, 시정수 2 s)", &r);
    assert(r.verdict == GAIN_V_CLEAN && r.flips >= 2 && r.flips <= 4);

    /* ±1 단위 떨림: 사행이 아니다. 매 표본 ±1 로 뒤집히고 평균이 한쪽으로 조금 치우쳐도 반주기가 1 표본이라 안 센다 */
    judge(g_jitter, GAIN_N_MAX, &r);
    show("±1 단위 떨림(매 표본)", &r);
    assert(r.verdict == GAIN_V_CLEAN);
    sine(1.0f, 0.5f, &r);                         /* 정확히 ±1 인 사인: 이력 ±1 을 넘지 않는다 */
    assert(r.verdict == GAIN_V_CLEAN && r.flips == 0);

    /* 창이 6 s(표본 300개)보다 짧으면 판정하지 않는다 */
    s_bias = 0.0f; s_noise = 0.0f; s_a = 3.0f; s_hz = 0.5f;
    judge(g_sine, GAIN_N_MIN - 1, &r);
    assert(r.verdict == GAIN_V_NONE && r.rms10 == 0 && r.flips == 0);
    judge(g_sine, GAIN_N_MIN, &r);
    assert(r.verdict != GAIN_V_NONE);

    /* RMS 기준 6.0 mm = 1.742 단위: 일정 잡음 폭으로 양쪽을 본다(사인이 아니라 사행 없음) */
    s_a = 0.0f; s_hz = 1.0f; s_bias = 0.0f;
    s_noise = 1.7f * 1.732f;     /* 균일 ±a 의 RMS = a/√3 → 1.7 단위 = 5.86 mm */
    judge(g_sine, GAIN_N_MAX, &r);
    show("잡음 RMS 5.9 mm", &r);
    assert(r.rms10 >= 55 && r.rms10 <= 62 && r.verdict == GAIN_V_CLEAN);
    s_noise = 1.85f * 1.732f;    /* 6.4 mm */
    judge(g_sine, GAIN_N_MAX, &r);
    show("잡음 RMS 6.4 mm", &r);
    assert(r.rms10 >= 62 && r.verdict == GAIN_V_RMS);

    /* 표본이 튀어도(선을 잃음 ±500 단위) 정수가 넘치지 않는다 */
    {
        int16_t buf[GAIN_N_MAX];
        int t;
        for (t = 0; t < GAIN_N_MAX; t++) { buf[t] = (int16_t)((t & 1) ? 4000 : -4000); }
        gain_judge(buf, GAIN_N_MAX, &r);
        assert(r.verdict == GAIN_V_BOTH || r.verdict == GAIN_V_RMS);
    }
}

/* 판정 순서 v(1 = 깨끗)를 차례로 알리고 끝 상태를 본다. 끝나기 전에 순서가 남으면 실패 */
static void seq_case(const char *v, int16_t kp, int16_t ki, int16_t ng)
{
    gain_seq_t q;
    gain_seq_begin(&q);
    for (; *v; v++) { assert(!q.done); gain_seq_next(&q, *v == '1'); }
    assert(q.done && (q.ng == ng) && (gain_kp(&q) == kp) && (gain_ki(&q) == ki));
}

static void test_seq(void)
{
    gain_seq_t q;
    static const int16_t kp[GAIN_CANDS] = { 60, 60, 60, 50, 40 }, ki[GAIN_CANDS] = { 5, 2, 1, 1, 1 };
    int k;

    /* 후보 쌍 표 */
    gain_seq_begin(&q);
    assert(q.first == -1);
    for (k = 0; k < GAIN_CANDS; k++) {
        q.idx = (int16_t)k;
        assert(gain_kp(&q) == kp[k] && gain_ki(&q) == ki[k]);
    }
    /* 깨끗하면 한 칸 낮은 후보를 한 번 더 달린다: 다음에 달리는 후보 확인 */
    gain_seq_begin(&q);
    gain_seq_next(&q, 1);
    assert(!q.done && q.first == 0 && gain_kp(&q) == 60 && gain_ki(&q) == 2);

    seq_case("11", 60, 2, 0);           /* 60·5·60·2 깨끗 → 60·2 */
    seq_case("10", 60, 5, 0);           /* 60·5 깨끗·60·2 탈락 → 60·5 */
    seq_case("0011", 50, 1, 0);         /* 60·5·60·2 탈락, 60·1·50·1 깨끗 → 50·1 */
    seq_case("0010", 60, 1, 0);         /* 60·1 깨끗·50·1 탈락 → 60·1 */
    seq_case("00011", 40, 1, 0);        /* 50·1·40·1 깨끗 → 40·1 */
    seq_case("00010", 50, 1, 0);        /* 50·1 깨끗·40·1 탈락 → 50·1 */
    seq_case("00001", 40, 1, 0);        /* 40·1 만 깨끗 → 40·1, NG 아님(더 낮은 후보 없음) */
    seq_case("00000", 40, 1, 1);        /* 전부 탈락 → 40·1, NG */

    /* 재시도까지 판정 못 함: 처음 깨끗한 값 전이면 실패(0), 한 칸 낮춘 후보면 탈락과 같이 처음 깨끗한 값으로 끝 */
    gain_seq_begin(&q);
    gain_seq_next(&q, 0);                                       /* 60·5 탈락, 60·2 를 달린다 */
    assert((gain_seq_miss(&q) == 0) && !q.done && (q.idx == 1));
    gain_seq_next(&q, 1);                                       /* 60·2 깨끗, 60·1 을 달린다 */
    assert((gain_seq_miss(&q) == 1) && q.done && !q.ng && (gain_kp(&q) == 60) && (gain_ki(&q) == 2));
}

int main(void)
{
    setvbuf(stdout, 0, _IONBF, 0);
    test_judge();
    test_seq();
    printf("test_gain ok\n");
    return 0;
}
