/* curvelog.c - 곡선 구간 기록기. 설계: include/agv/curvelog.h
 *
 * 곡선 판정은 마지막 0.2 m 창의 차체 곡률이라 판정 지점이 실제 곡선보다 늦다(끝은 창 길이만큼).
 * 그래서 누적은 0.2 m 늦은 표본으로 한다: 시작은 창 곡률이 문턱을 넘은 지점, 끝은 창 곡률이 문턱 아래로
 * 내려간 지점에서 창 길이만큼 앞이다. 둘 다 곡선 안쪽이라 직선 꼬리가 반경을 키우지 않는다.
 */
#include <string.h>
#include "agv/curvelog.h"
#include "agv/drive.h"
#include "agv/guide.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/tune.h"

#define D agv_drive
#define P param_dv

float sqrtf(float x);
float sinf(float x);
float cosf(float x);

#define CURVE_K_ON_PER_M   (1.0f / 8.0f)  /* 곡선 문턱: 차체 곡률 |dθ/ds| 가 이보다 크면 곡선(반경 8 m) */
#define CURVE_WIN_MM       200.0f         /* 곡률 창: 마지막 0.2 m 평균 */
#define CURVE_HOLD_MM      200.0f         /* 문턱 위(시작)·아래(끝)가 이만큼 이어져야 바뀐다 */
#define CURVE_MIN_MM       300.0f         /* 이보다 짧은 구간은 버린다 */
#define CURVE_R_MAX_MM     20000.0f       /* 반경이 이보다 크면 0(직선에 가깝다) */
#define CURVE_POST_TICKS   100            /* 출구 넘침을 보는 시간 1 s */
#define CURVE_FIT_MIN      5.0f           /* 원 맞춤 최소 점 수 */
#define RING_STEP_MM       10.0f          /* 표본 간격 */
#define RING_N             32             /* 0.2 m 지연 + 여유. 간격이 10 mm 이상이라 밀린 표본은 21 개 이하 */

/* .ebss(RAML1)에 안 들어가 RAMH0 시험 결과 뒤에 둔다(cmd 의 curvelog_ram). 부팅 때 curvelog_reset 이 지운다 */
#ifdef __TMS320C2000__
#pragma DATA_SECTION(agv_curvelog, "curvelog_ram")
#endif
curvelog_t agv_curvelog;

typedef struct {
    float    s, phi, sx, sy;    /* 이동 거리 mm, 방향 rad, 센서 점 mm */
    int32_t  tick;
    int16_t  pos, ok, lost_n;   /* ok = 선 있음. lost_n = 그때까지 선 잃음 누적 */
} ring_t;

/* 한 구간의 누적. 원 맞춤은 곡선 좌표(m)에서 대수적 원 맞춤(Kasa) 합으로 한다 */
typedef struct {
    float   len, dphi, ticks;
    float   n, sx, sy, sxx, syy, sxy, sxxx, syyy, sxxy, sxyy;
    float   dev_n, dev_sum, dev_sum2, dev_max;
    int16_t lost;
} acc_t;

#ifdef __TMS320C2000__
#pragma DATA_SECTION(s_cl, "curvelog_ram")
#endif
static struct {
    float    x, y, phi, s;          /* 활성 동안 이어지는 오도메트리(mm, rad, 진행 방향 기준) */
    int32_t  tick;
    int16_t  lost_n, prev_lost;
    ring_t   ring[RING_N];
    uint16_t push_n, proc_n;        /* 넣은 표본 수, 누적에 쓴 표본 수 */
    ring_t   pp;                    /* 마지막으로 처리한 표본 */
    int16_t  back;                  /* 지난 틱 진행 방향. -1 = 모름 */
    int16_t  sens_above;            /* 센서 점 곡률이 문턱 위 */
    float    sens_cross_s;          /* 센서 점 곡률이 문턱을 넘은 이동 거리 */
    int16_t  st;                    /* 0 없음, 1 후보, 2 곡선 */
    float    hold_mm;               /* 후보: 문턱 위 거리. 곡선: 문턱 아래 거리 */
    float    start_s, ox, oy, ophi; /* 시작 지점과 곡선 좌표 원점 */
    int16_t  begun, lag;
    acc_t    a, snap;               /* snap = 곡선 중 창 곡률이 처음 문턱 아래로 내려간 때의 누적(끝 후보) */
    int16_t  post_on, post_sign;    /* 출구 넘침: 끝 표본부터 1 s */
    int32_t  post_t0;
    float    post_max;
    curve_rec_t *post_rec;          /* 0 = 아직 기록 전(끝 확정 대기) */
    uint32_t last_p[2];
    int16_t  prev_mode;
} s_cl;

static void state_clear(void)
{
    s_cl.push_n = s_cl.proc_n = 0;
    s_cl.back = -1;
    s_cl.st = 0;
    s_cl.sens_above = 0;
    s_cl.post_on = 0;
}

static void group_start(void)
{
    curve_group_t *g = &agv_curvelog.g[agv_curvelog.group - 1];
    memset(g, 0, sizeof(*g));
    g->lg = param_get(P_LEGACY_QUIRKS);
    g->opt = param_get(P_OPT_FLAGS);
    state_clear();
}

void curvelog_reset(void)
{
    memset(&agv_curvelog, 0, sizeof(agv_curvelog));
    memset(&s_cl, 0, sizeof(s_cl));
    agv_curvelog.group = 1;
    s_cl.last_p[0] = run_pulses(0);
    s_cl.last_p[1] = run_pulses(1);
    s_cl.prev_mode = D.ui_mode;
    group_start();
}

void curvelog_next_group(void)
{
    agv_curvelog.group = (int16_t)((agv_curvelog.group % CURVELOG_GROUPS) + 1);
    group_start();
}

static float absf(float v) { return (v < 0.0f) ? -v : v; }

static int16_t clip16(float v)
{
    if (v > 32767.0f) { return 32767; }
    if (v < -32767.0f) { return -32767; }
    return (int16_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

static void acc_add(acc_t *a, const ring_t *k, const ring_t *prev)
{
    a->len += k->s - prev->s;
    a->dphi += k->phi - prev->phi;
    a->ticks += (float)(k->tick - prev->tick);
    a->lost = (int16_t)(a->lost + (k->lost_n - prev->lost_n));
    if (k->ok) {
        float c = cosf(s_cl.ophi), s = sinf(s_cl.ophi);
        float dx = k->sx - s_cl.ox, dy = k->sy - s_cl.oy;
        float u = (dx * c + dy * s) * 0.001f, v = (-dx * s + dy * c) * 0.001f;
        float d = (float)k->pos;
        a->n += 1.0f;
        a->sx += u;  a->sy += v;
        a->sxx += u * u;  a->syy += v * v;  a->sxy += u * v;
        a->sxxx += u * u * u;  a->syyy += v * v * v;
        a->sxxy += u * u * v;  a->sxyy += u * v * v;
        a->dev_n += 1.0f;
        a->dev_sum += d;
        a->dev_sum2 += d * d;
        if (absf(d) > a->dev_max) { a->dev_max = absf(d); }
    }
}

/* 대수적 원 맞춤(Kasa). 평균을 뺀 모멘트로 푼다. 반경 mm, 0 = 측정 없음 */
static float kasa_radius_mm(const acc_t *a)
{
    float n = a->n, mx, my, suu, svv, suv, suuu, svvv, suvv, svuu, det, uc, vc, r2, b1, b2;
    if (n < CURVE_FIT_MIN) { return 0.0f; }
    mx = a->sx / n;
    my = a->sy / n;
    suu = a->sxx / n - mx * mx;
    svv = a->syy / n - my * my;
    suv = a->sxy / n - mx * my;
    suuu = a->sxxx / n - 3.0f * mx * a->sxx / n + 2.0f * mx * mx * mx;
    svvv = a->syyy / n - 3.0f * my * a->syy / n + 2.0f * my * my * my;
    suvv = a->sxyy / n - mx * a->syy / n - 2.0f * my * a->sxy / n + 2.0f * mx * my * my;
    svuu = a->sxxy / n - my * a->sxx / n - 2.0f * mx * a->sxy / n + 2.0f * my * mx * mx;
    det = suu * svv - suv * suv;
    if (det <= 1.0e-6f * (suu + svv) * (suu + svv)) { return 0.0f; }   /* 한 줄에 놓였다 */
    b1 = 0.5f * (suuu + suvv);
    b2 = 0.5f * (svvv + svuu);
    uc = (b1 * svv - b2 * suv) / det;
    vc = (b2 * suu - b1 * suv) / det;
    r2 = uc * uc + vc * vc + suu + svv;
    return (r2 > 0.0f) ? sqrtf(r2) * 1000.0f : 0.0f;
}

static void finish(const acc_t *a, float gd)
{
    curve_group_t *g = &agv_curvelog.g[agv_curvelog.group - 1];
    curve_rec_t *r;
    float rt, rb;

    if ((a->len < CURVE_MIN_MM) || (g->n >= CURVELOG_CURVES)) {   /* 짧은 구간은 버리고, 넘치면 앞 곡선만 남긴다 */
        if ((a->len >= CURVE_MIN_MM) && (g->n < 32767)) { g->n++; }
        s_cl.post_on = 0;
        return;
    }
    r = &g->c[g->n];
    g->n++;
    memset(r, 0, sizeof(*r));
    r->seq = g->n;
    r->len_mm = clip16(a->len);
    r->v_x10 = (a->ticks > 0.0f) ? clip16(a->len * 60.0f / a->ticks) : 0;   /* mm/10 ms → m/min × 10 */
    rt = kasa_radius_mm(a);
    if (rt > CURVE_R_MAX_MM) { rt = 0.0f; }
    rb = (absf(a->dphi) > 0.0f) ? a->len / absf(a->dphi) : 0.0f;
    if (rb > CURVE_R_MAX_MM) { rb = 0.0f; }
    r->r_tape_mm = clip16(rt);
    r->r_body_mm = clip16(rb);
    r->r_theory_mm = (rt > gd) ? clip16(sqrtf(rt * rt - gd * gd)) : 0;
    r->dev_max_x10 = clip16(a->dev_max * GUIDE_UNIT_MM * 10.0f);
    r->dev_rms_x10 = (a->dev_n > 0.0f) ? clip16(sqrtf(a->dev_sum2 / a->dev_n) * GUIDE_UNIT_MM * 10.0f) : 0;
    r->entry_lag_mm = s_cl.lag;
    r->lost = a->lost;
    r->dir = (a->dphi >= 0.0f) ? 1 : -1;
    if (s_cl.post_on) {
        s_cl.post_rec = r;
        r->exit_over_x10 = clip16(s_cl.post_max * GUIDE_UNIT_MM * 10.0f);
    }
}

/* 세 점 외접원 곡률(1/m). 점 mm */
static float curv3(float ax, float ay, float bx, float by, float cx, float cy)
{
    float abx = bx - ax, aby = by - ay, acx = cx - ax, acy = cy - ay, bcx = cx - bx, bcy = cy - by;
    float l = sqrtf((abx * abx + aby * aby) * (acx * acx + acy * acy) * (bcx * bcx + bcy * bcy));
    return (l > 0.0f) ? 2000.0f * (abx * acy - aby * acx) / l : 0.0f;
}

static const ring_t *sample(uint16_t n) { return &s_cl.ring[n % RING_N]; }

/* 창 곡률: 차체 kb, 센서 점 ks(세 점: 지금, 약 0.1 m 전, 약 0.2 m 전). 창이 덜 찼으면 0 */
static void window_curv(float sx, float sy, int16_t sok, float *kb, float *ks)
{
    const ring_t *mid_s = 0, *far_s = 0;
    uint16_t i, n = (s_cl.push_n < RING_N) ? s_cl.push_n : RING_N;
    *kb = *ks = 0.0f;
    for (i = 1; i <= n; i++) {
        const ring_t *k = sample((uint16_t)(s_cl.push_n - i));
        float back = s_cl.s - k->s;
        if ((mid_s == 0) && (back >= 0.5f * CURVE_WIN_MM)) { mid_s = k; }
        if (back >= CURVE_WIN_MM) { far_s = k; break; }
    }
    if (far_s == 0) { return; }
    *kb = (s_cl.phi - far_s->phi) * 1000.0f / (s_cl.s - far_s->s);
    if (sok && (mid_s != 0) && mid_s->ok && far_s->ok) { *ks = curv3(far_s->sx, far_s->sy, mid_s->sx, mid_s->sy, sx, sy); }
}

/* 0.2 m 늦은 표본 하나: 곡선 누적과 출구 넘침 */
static void process(const ring_t *k)
{
    if ((s_cl.st != 0) && (k->s >= s_cl.start_s)) {
        if (!s_cl.begun) { s_cl.begun = 1; }          /* 첫 표본은 기준만 된다 */
        else             { acc_add(&s_cl.a, k, &s_cl.pp); }
    }
    if (s_cl.post_on) {
        if ((k->tick - s_cl.post_t0) > CURVE_POST_TICKS) { s_cl.post_on = 0; }
        else {
            float v = k->ok ? -(float)s_cl.post_sign * (float)k->pos : 0.0f;
            if (v > s_cl.post_max) { s_cl.post_max = v; }
            if (s_cl.post_rec != 0) { s_cl.post_rec->exit_over_x10 = clip16(s_cl.post_max * GUIDE_UNIT_MM * 10.0f); }
        }
    }
    s_cl.pp = *k;
}

void curvelog_feed(const curvelog_in_t *in)
{
    float sgn = in->backward ? -1.0f : 1.0f;
    float ds, dphi, mid, sx = 0.0f, sy = 0.0f, kb, ks, lat;
    int16_t above;

    if (!in->active || (in->wb_mm <= 0.0f) || ((s_cl.back >= 0) && (s_cl.back != in->backward))) {
        state_clear();
        if (in->active) { s_cl.back = in->backward; }
        s_cl.prev_lost = in->lost;
        return;
    }
    s_cl.back = in->backward;
    /* 오도메트리: 진행 방향 기준. 후진이면 바퀴 이동이 음수라 회전 부호가 바뀐다 */
    ds = 0.5f * (in->dl_mm + in->dr_mm);
    dphi = sgn * (in->dr_mm - in->dl_mm) / in->wb_mm;
    mid = s_cl.phi + 0.5f * dphi;
    s_cl.x += ds * cosf(mid);
    s_cl.y += ds * sinf(mid);
    s_cl.phi += dphi;
    s_cl.s += ds;
    s_cl.tick++;
    if (in->lost && !s_cl.prev_lost) { s_cl.lost_n++; }
    s_cl.prev_lost = in->lost;
    if (!in->lost) {
        lat = (float)in->pos * GUIDE_UNIT_MM;     /* + = 진행 방향 왼쪽 */
        sx = s_cl.x + in->gd_mm * cosf(s_cl.phi) - lat * sinf(s_cl.phi);
        sy = s_cl.y + in->gd_mm * sinf(s_cl.phi) + lat * cosf(s_cl.phi);
    }
    if ((s_cl.push_n == 0) || (s_cl.s - sample((uint16_t)(s_cl.push_n - 1u))->s >= RING_STEP_MM)) {
        ring_t *r = &s_cl.ring[s_cl.push_n % RING_N];
        r->s = s_cl.s;  r->phi = s_cl.phi;  r->sx = sx;  r->sy = sy;
        r->tick = s_cl.tick;  r->pos = in->pos;  r->ok = (int16_t)!in->lost;  r->lost_n = s_cl.lost_n;
        if (s_cl.push_n == 0u) { s_cl.pp = *r; s_cl.proc_n = 1; }   /* 첫 표본은 기준 */
        s_cl.push_n++;
    }
    while ((s_cl.proc_n < s_cl.push_n) && (sample(s_cl.proc_n)->s <= s_cl.s - CURVE_WIN_MM)) {
        process(sample(s_cl.proc_n));
        s_cl.proc_n++;
    }
    window_curv(sx, sy, (int16_t)!in->lost, &kb, &ks);
    if (absf(ks) > CURVE_K_ON_PER_M) {
        if (!s_cl.sens_above) { s_cl.sens_cross_s = s_cl.s; }
        s_cl.sens_above = 1;
    }
    else { s_cl.sens_above = 0; }
    above = (int16_t)(absf(kb) > CURVE_K_ON_PER_M);

    switch (s_cl.st) {
    case 0:
        if (above) {
            memset(&s_cl.a, 0, sizeof(s_cl.a));
            s_cl.begun = 0;
            s_cl.start_s = s_cl.s;
            s_cl.ox = s_cl.x;  s_cl.oy = s_cl.y;  s_cl.ophi = s_cl.phi;
            s_cl.lag = s_cl.sens_above ? clip16(s_cl.s - s_cl.sens_cross_s) : 0;
            s_cl.hold_mm = 0.0f;
            s_cl.st = 1;
        }
        break;
    case 1:
        if (!above) { s_cl.st = 0; break; }
        s_cl.hold_mm += ds;
        if (s_cl.hold_mm >= CURVE_HOLD_MM) { s_cl.st = 2; s_cl.hold_mm = 0.0f; }
        break;
    default:
        if (!above) {
            if (s_cl.hold_mm <= 0.0f) {             /* 처음 내려간 때. 누적은 창 길이만큼 앞(곡선 끝)까지다 */
                s_cl.snap = s_cl.a;
                s_cl.post_on = 1;
                s_cl.post_t0 = s_cl.pp.tick;
                s_cl.post_max = 0.0f;
                s_cl.post_rec = 0;
                s_cl.post_sign = (s_cl.snap.dev_sum >= 0.0f) ? 1 : -1;
            }
            s_cl.hold_mm += ds;
            if (s_cl.hold_mm >= CURVE_HOLD_MM) {
                finish(&s_cl.snap, in->gd_mm);
                s_cl.st = 0;
            }
        }
        else {
            if (s_cl.hold_mm > 0.0f) { s_cl.post_on = 0; }   /* 다시 곡선: 끝이 아니었다 */
            s_cl.hold_mm = 0.0f;
        }
        break;
    }
}

void curvelog_tick(void)
{
    curvelog_in_t in;
    uint32_t p0 = run_pulses(0), p1 = run_pulses(1);
    float k = (P.one_length_pulse > 0.0f) ? (1.0f / P.one_length_pulse) : 0.0f;

    if ((s_cl.prev_mode == UI_PARAM) && (D.ui_mode == UI_AUTO)) { curvelog_next_group(); }
    s_cl.prev_mode = D.ui_mode;
    in.dl_mm = (float)(p0 - s_cl.last_p[0]) * k;
    in.dr_mm = (float)(p1 - s_cl.last_p[1]) * k;
    s_cl.last_p[0] = p0;
    s_cl.last_p[1] = p1;
    in.backward = (int16_t)(D.backward != 0);
    in.pos = D.guide_pos;
    in.lost = (int16_t)(D.guide_all_off != 0);
    in.active = (int16_t)((D.ui_mode == UI_AUTO) && !tune_active() && D.flag_run && (D.control == RUN_DRIVE)
                          && !D.flag_turn);
    in.wb_mm = (float)param_get(P_WHEEL_BASE_MM) * 10.0f * tune_turn_scale();   /* 제자리 회전과 같은 휠 간격 */
    in.gd_mm = P.guide_dist_mm;
    curvelog_feed(&in);
}
