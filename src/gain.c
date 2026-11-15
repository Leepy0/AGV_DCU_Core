/* gain.c - 조향 이득 찾기 판정(gain.h). 정수 누적, 끝에서만 sqrtf 한 번 */
#include <math.h>
#include "agv/gain.h"
#include "agv/guide.h"

static const int16_t s_kp[GAIN_CANDS] = { 60, 50, 40, 30, 25, 20 };

void gain_judge(const int16_t *g, int16_t n, gain_log_t *r)
{
    int32_t sum = 0, ss = 0, mean, peak_sum = 0;
    int16_t i, side = 0, flips = 0, start = -1, peak = 0, halves = 0, run = 0, best = 0;
    float rms_mm;
    int bad_rms, wobble;

    r->rms10 = r->peak10 = r->flips = r->verdict = 0;
    if (n < GAIN_WIN_MIN) { return; }
    for (i = 0; i < n; i++) { sum += g[i]; }
    mean = (sum >= 0) ? (sum + n / 2) / n : -((-sum + n / 2) / n);
    for (i = 0; i < n; i++) {
        int32_t d = (int32_t)g[i] - mean, a;
        int16_t now;
        if (d > 1000) { d = 1000; } else if (d < -1000) { d = -1000; }   /* 선을 잃어 튄 값에도 ss 가 넘치지 않게(125 단위) */
        a = (d < 0) ? -d : d;
        now = (d > GAIN_HYS8) ? 1 : ((d < -GAIN_HYS8) ? -1 : side);
        ss += d * d;
        if (now != side) {
            if (side != 0) {                /* 부호 바뀜. 앞 부호 바뀜이 있으면 그 사이가 온전한 반주기 하나 */
                flips++;
                if (start >= 0) {
                    int16_t len = (int16_t)(i - start);
                    peak_sum += peak;
                    halves++;
                    if ((len >= GAIN_HALF_MIN) && (len <= GAIN_HALF_MAX) && (peak >= GAIN_PEAK8)) {
                        if (++run > best) { best = run; }
                    }
                    else { run = 0; }
                }
                start = i;
            }
            side = now;                     /* 처음 한쪽에 든 자리는 부호 바뀜이 아니다: 그 앞은 잘려 있어 반주기로 세지 않는다 */
            peak = 0;
        }
        if (a > peak) { peak = (int16_t)a; }
    }
    rms_mm = sqrtf((float)ss / (float)n) / (float)GAIN_SCALE * GUIDE_UNIT_MM;
    r->rms10 = (int16_t)(rms_mm * 10.0f + 0.5f);
    r->peak10 = (halves > 0) ? (int16_t)((peak_sum * 10L) / ((int32_t)halves * GAIN_SCALE)) : 0;
    r->flips = flips;
    bad_rms = rms_mm > GAIN_RMS_MM;
    wobble = best >= GAIN_RUN_MIN;
    r->verdict = (int16_t)((bad_rms && wobble) ? GAIN_V_BOTH : (bad_rms ? GAIN_V_RMS : (wobble ? GAIN_V_WOBBLE : GAIN_V_CLEAN)));
}

void gain_seq_begin(gain_seq_t *s) { s->idx = 0; s->done = 0; s->ng = 0; }
int16_t gain_kp(const gain_seq_t *s) { return s_kp[s->idx]; }
int16_t gain_ki(const gain_seq_t *s)
{
    int16_t k = (int16_t)((s_kp[s->idx] * 5 + 30) / 60);     /* 반올림(K_P × 5 / 60) */
    return (k < 1) ? 1 : k;
}

void gain_seq_next(gain_seq_t *s, int clean)
{
    if (clean) { s->done = 1; return; }
    if (s->idx + 1 < GAIN_CANDS) { s->idx++; return; }
    s->done = 1;                                              /* 마지막 후보(20·2)도 탈락 */
    s->ng = 1;
}
