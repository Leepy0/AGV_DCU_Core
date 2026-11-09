/* rpmtest.c - 바닥 rpm 시험의 회 안 측정. rpmtest.h 참고.
 * 선 찾기·회전·재출발·먼 선까지 이동·실패 처리는 ident.c 가 한다. 여기는 선에서 출발한 뒤 측정만 한다.
 * 개루프 회(1~3)는 매 틱 motor_dac_force 로 낸다. 풀기는 ident.c 가 한다.
 * 개루프 회는 먼저 최저 속도 폐루프로 APPR_MM 간다(접근). 엔코더에 방향이 없어 방향은 이 폐루프 구간이 정한다:
 * 고정은 그 폐루프가 RUN_DRIVE 로 달리는 중에만 걸고, motor.c 는 고정 중에도 같은 D.backward 방향을 낸다.
 * 정지 마찰로 한 바퀴만 먼저 풀려 차가 도는 일도 없앤다. 3회는 접근 뒤 DAC 0 으로 서고 나서 오름을 시작한다 */
#include <math.h>
#include "agv/rpmtest.h"
#include "agv/drive.h"
#include "agv/ident.h"
#include "agv/motor.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/stop.h"

#define D agv_drive
#define P param_dv

#define TICK_MS       10
#define UP_TICKS      150      /* 1회 단계 1.5 s */
#define DN_TICKS      100      /* 2회 단계 1 s */
#define BUF_TICKS     150
#define WIN_TICKS     50       /* 끝 0.5 s */
#define STEADY_RPM_S  200
#define BAND_MIN_RPM  20
#define MOVE_RPM      50       /* 앰프 추정: 이보다 느린 단계는 안 본다 */
#define BRK_DAC       600
#define BRK_TICKS     300      /* 3 s 램프 */
#define BRK_WAIT      30       /* 내린 뒤 마지막 펄스를 기다린다 */
#define LOW_TICKS     150
#define LOW_WIN       100      /* 끝 1 s */
#define STALL_IDLE    11       /* 펄스 없이 11 틱 = 100 ms 넘음 */
#define STILL_TICKS   20       /* 두 바퀴 200 ms 조용하면 섰다 */
#define STOPW_TICKS   300      /* 3 s 안에 안 서면 넘어간다 */
#define CRUISE_TICKS  30       /* 바닥 속도 정속 0.3 s 뒤 트리거 */
#define V0_TICKS      10
#define PEAK_TICKS    20       /* 정지 전 0.2 s */
#define GAP_MM        400.0f   /* 트리거 간격 */
#define CUT_MM        300.0f   /* 먼 선까지 감속 거리 + 이만큼은 남긴다 */
#define DEC_EVERY     4        /* 감속 거리는 4 틱마다 잰다. CPU 를 아낀다 */
#define RING          64u      /* 2의 거듭제곱 */
#define APPR_MM       200.0f   /* 개루프 회 앞 폐루프 접근 거리. 남은 거리 guard 는 회 시작부터 재므로 이 거리도 든다 */
#define APPR_LIMIT    1000     /* 10 s 안에 접근을 못 끝내면 실패 */
#define ASYM_LO_RPM   20       /* 개루프: 한 바퀴가 이보다 느리고 */
#define ASYM_HI_RPM   100      /* 다른 바퀴가 이보다 빠르게 */
#define ASYM_TICKS    30       /* 0.3 s 넘게 가면 한 바퀴 멈춤(차가 그 바퀴를 축으로 돈다) */
#define ASYM_GUIDE    5        /* 개루프: 가이드가 이만큼 벗어나거나 라인을 잃으면 끊는다(실패 문턱 10 보다 먼저) */

static const int16_t s_up[RPMT_UP]   = { 200, 400, 600, 800, 1200, 1600, 2000 };
static const int16_t s_dn[RPMT_DN]   = { 2000, 1200, 600, 0 };
static const int16_t s_low[RPMT_LOW] = { 300, 250, 200, 150, 100 };
static const float   s_ramp_mm[4]    = { 0.0f, 0.0f, 30.0f, 60.0f };   /* 방법 1~3 */

#ifdef __TMS320C2000__
#pragma DATA_SECTION(agv_rpmt, "test_keep_rpm")   /* 리셋에 남는다. 링커가 test_keep_add 뒤에 둔다 */
#endif
rpmt_result_t agv_rpmt;

static struct {
    int16_t  t, n, y0, dac, dw[2], stopping, cruise;
    int16_t  appr, at;       /* 접근: 1 폐루프로 가는 중, 2 DAC 0 으로 서기 기다림(3회). at = 접근 틱 */
    int16_t  aw, as;         /* 한 바퀴 멈춤: 멈춘 바퀴(-1 없음), 이어진 틱 */
    uint16_t rn;
    uint32_t pass0, step0, trig;
    float    dec;
    int32_t  sum[2], tq[2];
    float    ss[2];
    union { int16_t w[2][BUF_TICKS]; int16_t ring[RING]; } u;   /* 1·2회 단계 rpm, 5회 좌우 평균 고리 */
} s_rt;

#define STEP ((uint16_t)agv_rpmt.phase)

static uint32_t travel(void) { return run_pulses(D.backward ? 1u : 0u); }
static float    mm_from(uint32_t p) { return (float)(travel() - p) / P.one_length_pulse; }
static int16_t  avg2(int16_t a, int16_t b) { return (int16_t)(((int32_t)a + (int32_t)b) / 2); }
static int16_t  dac_clamp(int32_t v) { return (int16_t)((v < 0) ? 0 : ((v > MOTOR_DAC_MAX) ? MOTOR_DAC_MAX : v)); }

/* 0 계속, 1 먼 선까지 남은 거리 < 감속 거리 + 300 mm, 2 과속 */
static int16_t guard(void)
{
    float rem = agv_rpmt.track_mm - P.guide_dist_mm - mm_from(s_rt.pass0);

    if ((s_rt.n++ & (DEC_EVERY - 1)) == 0) { s_rt.dec = ident_decel_mm(avg2(D.rpm_avg[0], D.rpm_avg[1])); }
    if ((D.rpm_avg[0] > agv_rpmt.lim_rpm) || (D.rpm_avg[1] > agv_rpmt.lim_rpm)) { return 2; }
    return (rem < s_rt.dec + CUT_MM) ? 1 : 0;
}

/* 개루프 좌우 어긋남. 0 계속, RPMT_CUT_STALL_L·R 한 바퀴 멈춤 0.3 s, RPMT_CUT_GUIDE 가이드 5칸 이상이나 라인 잃음.
 * c = 지금 공통 DAC. 0(내림·대기)이면 바퀴 멈춤은 안 본다 */
static int16_t asym(int16_t c)
{
    int16_t w = -1;

    if (D.guide_all_off || (D.guide_pos >= ASYM_GUIDE) || (D.guide_pos <= -ASYM_GUIDE)) { return RPMT_CUT_GUIDE; }
    if (c > 0) {
        if ((D.rpm[0] < ASYM_LO_RPM) && (D.rpm[1] > ASYM_HI_RPM))      { w = 0; }
        else if ((D.rpm[1] < ASYM_LO_RPM) && (D.rpm[0] > ASYM_HI_RPM)) { w = 1; }
    }
    if (w != s_rt.aw) {
        s_rt.aw = w;
        s_rt.as = 0;
    }
    if ((w < 0) || (++s_rt.as <= ASYM_TICKS)) { return 0; }
    return (w == 0) ? RPMT_CUT_STALL_L : RPMT_CUT_STALL_R;
}

/* 회 끝 반환값. 어긋남으로 끊었으면 2(세운다) */
static int pass_end(int16_t cut) { return (cut >= RPMT_CUT_STALL_L) ? 2 : 1; }

/* 공통 DAC 에 가이드 편차 비례 좌우 차. 조향과 같은 부호: 진행 방향 기준 + 편차면 우를 높인다 */
static void dac_out(int16_t c)
{
    int32_t lat = D.backward ? -(int32_t)D.guide_pos : (int32_t)D.guide_pos;
    int32_t d = (int32_t)RPMT_STEER_K * lat;
    int32_t m = (int32_t)c / 10;

    if (d > m)  { d = m; }
    if (d < -m) { d = -m; }
    s_rt.dw[0] = dac_clamp((int32_t)c - d);
    s_rt.dw[1] = dac_clamp((int32_t)c + d);
    motor_dac_force(s_rt.dw[0], s_rt.dw[1]);
}

/* ---- 결과 요약: FND 쪽과 앰프 추정 ---- */

static void summary(void)
{
    int16_t rise = 0, ok = 0, a = -1, b = -1, k;
    const rpmt_brk_t *br = &agv_rpmt.brk;

    /* 1회: 일정 DAC 에서 정상 rpm 에 머물면 속도 지령, 계속 오르면 토크 지령 */
    for (k = 0; k < RPMT_UP; k++) {
        const rpmt_step_t *s = &agv_rpmt.up[k];
        int16_t v = avg2(s->rpm[0], s->rpm[1]);
        if ((s->ms == 0) || (s->cut & 1) || (v <= MOVE_RPM)) { continue; }
        if (s->slope >= STEADY_RPM_S) { rise++; }
        else if (s->steady && !s->cut) {
            ok++;
            if ((a < 0) && (s->dac >= 600)) { a = k; }
            b = k;
        }
    }
    agv_rpmt.amp = 0;
    if (rise >= 2) { agv_rpmt.amp = 'T'; }
    else if ((ok >= 3) && (a >= 0) && (b > a)) {
        float ra = (float)avg2(agv_rpmt.up[a].rpm[0], agv_rpmt.up[a].rpm[1]);
        float rb = (float)avg2(agv_rpmt.up[b].rpm[0], agv_rpmt.up[b].rpm[1]);
        float q = (rb / ra) / ((float)agv_rpmt.up[b].dac / (float)agv_rpmt.up[a].dac);   /* 비례면 1 */
        if ((q > 0.5f) && (q < 2.0f)) { agv_rpmt.amp = 'S'; }
    }

    /* 4회: 위에서부터 멈춤 없이 돈 마지막 목표 */
    agv_rpmt.low_rpm = 0;
    for (k = 0; k < RPMT_LOW; k++) {
        const rpmt_low_t *l = &agv_rpmt.low[k];
        int16_t e = (int16_t)(avg2(l->mean[0], l->mean[1]) - l->target);
        if ((l->target == 0) || l->cut || l->stall[0] || l->stall[1] || (l->upd[0] == 0) || (l->upd[1] == 0)
            || (((e < 0) ? -e : e) > l->target / 4)) { break; }
        agv_rpmt.low_rpm = l->target;
    }

    agv_rpmt.brk_dac = ((br->move_dac[0] < 0) || (br->move_dac[1] < 0)) ? -1
                     : ((br->move_dac[0] > br->move_dac[1]) ? br->move_dac[0] : br->move_dac[1]);
}

/* ---- 1·2회 계단 ---- */

static int16_t y_at(int16_t k) { return avg2(s_rt.u.w[0][k], s_rt.u.w[1][k]); }

static void step_begin(rpmt_step_t *s, int16_t dac)
{
    s->dac = dac;
    s_rt.dac = dac;
    s_rt.t = 0;
    s_rt.y0 = avg2(D.rpm[0], D.rpm[1]);
    s_rt.step0 = travel();
    dac_out(dac);
}

static void step_end(rpmt_step_t *s, int16_t cut)
{
    int16_t n = (s_rt.t > BUF_TICKS) ? BUF_TICKS : s_rt.t;
    int16_t k0 = (n > WIN_TICKS) ? (int16_t)(n - WIN_TICKS) : 0;
    int16_t m = (int16_t)(n - k0), k, last = -1;
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f, den, sl, fin, band;
    uint16_t i;

    s->cut = cut;
    s->ms = (int16_t)(s_rt.t * TICK_MS);
    s->dist_mm = (int16_t)mm_from(s_rt.step0);
    s->dw[0] = s_rt.dw[0];
    s->dw[1] = s_rt.dw[1];
    if (m <= 0) { return; }
    for (i = 0; i < 2u; i++) {
        int32_t sum = 0;
        int16_t lo = 32767, hi = -32767;
        for (k = k0; k < n; k++) {
            int16_t v = s_rt.u.w[i][k];
            sum += v;
            if (v < lo) { lo = v; }
            if (v > hi) { hi = v; }
        }
        s->rpm[i] = (int16_t)(sum / m);
        s->ripple[i] = (int16_t)(hi - lo);
    }
    for (k = k0; k < n; k++) {
        float x = (float)(k - k0), y = (float)y_at(k);
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    den = (float)m * sxx - sx * sx;
    sl = (den > 0.0f) ? ((float)m * sxy - sx * sy) / den * (1000.0f / TICK_MS) : 0.0f;   /* rpm/틱 → rpm/s */
    if (sl > 32767.0f)  { sl = 32767.0f; }
    if (sl < -32767.0f) { sl = -32767.0f; }
    s->slope = (int16_t)sl;
    s->steady = (int16_t)(fabsf(sl) < (float)STEADY_RPM_S);
    fin = ((float)s->rpm[0] + (float)s->rpm[1]) * 0.5f;
    band = fabsf(fin - (float)s_rt.y0) * 0.1f;
    if (band < (float)BAND_MIN_RPM) { band = (float)BAND_MIN_RPM; }
    for (k = 0; k < n; k++) {
        if (fabsf((float)y_at(k) - fin) > band) { last = k; }
    }
    s->settle_ms = (last >= n - 1) ? -1 : (int16_t)((last + 1) * TICK_MS);
}

static int stair(rpmt_step_t *tab, const int16_t *dac, uint16_t n, int16_t len)
{
    rpmt_step_t *s = &tab[STEP - 1u];
    int16_t g = guard(), c;

    if (g == 0) { g = asym(s_rt.dac); }

    dac_out(s_rt.dac);
    if (s_rt.t < BUF_TICKS) {
        s_rt.u.w[0][s_rt.t] = D.rpm[0];
        s_rt.u.w[1][s_rt.t] = D.rpm[1];
    }
    s_rt.t++;
    if ((g == 0) && (s_rt.t < len)) { return 0; }
    step_end(s, g);
    if ((agv_rpmt.pass == 2) && (STEP >= 2u)) {    /* 내린 뒤 20 ms 마다 */
        int16_t *cv = agv_rpmt.dn_curve[STEP - 2u];
        for (c = 0; c < RPMT_DN_CURVE; c++) {
            int16_t k = (int16_t)(2 * c + 1);
            cv[c] = (k < s_rt.t) ? y_at(k) : 0;
        }
    }
    if ((g != 0) || (STEP >= n)) {
        summary();
        return pass_end(g);
    }
    agv_rpmt.phase++;
    step_begin(&tab[STEP - 1u], dac[STEP - 1u]);
    return 0;
}

/* ---- 3회 출발·정지 문턱 ---- */

static int brk_run(void)
{
    rpmt_brk_t *b = &agv_rpmt.brk;
    int16_t g = guard(), c;
    uint16_t i;

    if (g == 0) { g = asym((STEP == 1u) ? 1 : 0); }

    for (i = 0; i < 2u; i++) {      /* dw 는 앞 틱에 낸 DAC */
        if (motor_fresh_pulse(i)) {
            if (STEP == 1u) {
                if (b->move_dac[i] < 0) { b->move_dac[i] = s_rt.dw[i]; b->move_rpm[i] = D.rpm[i]; }
            }
            else { b->stop_dac[i] = s_rt.dw[i]; b->stop_rpm[i] = D.rpm[i]; }
        }
        if (D.rpm[i] > b->peak_rpm[i]) { b->peak_rpm[i] = D.rpm[i]; }
    }
    s_rt.t++;
    if (g != 0) { b->cut = g; }
    else if (STEP == 1u) {
        c = (int16_t)(((int32_t)BRK_DAC * s_rt.t) / BRK_TICKS);
        if (s_rt.t >= BRK_TICKS) { agv_rpmt.phase = 2; s_rt.t = 0; }
        dac_out(c);
        return 0;
    }
    else if (s_rt.t < BRK_TICKS + BRK_WAIT) {
        c = (s_rt.t < BRK_TICKS) ? (int16_t)(BRK_DAC - ((int32_t)BRK_DAC * s_rt.t) / BRK_TICKS) : 0;
        dac_out(c);
        return 0;
    }
    b->dist_mm = (int16_t)mm_from(s_rt.pass0);
    summary();
    return pass_end(b->cut);
}

/* ---- 4회 저속 폐루프 ---- */

static void low_begin(uint16_t k)
{
    agv_rpmt.low[k].target = s_low[k];
    s_rt.t = 0;
    s_rt.step0 = travel();
    s_rt.sum[0] = s_rt.sum[1] = s_rt.tq[0] = s_rt.tq[1] = 0;
    s_rt.ss[0] = s_rt.ss[1] = 0.0f;
}

static void low_end(rpmt_low_t *l, int16_t cut)
{
    int16_t n = (int16_t)(s_rt.t - (LOW_TICKS - LOW_WIN));
    uint16_t i;

    l->cut = cut;
    l->dist_mm = (int16_t)mm_from(s_rt.step0);
    if (n <= 0) { return; }
    for (i = 0; i < 2u; i++) {
        float mean = (float)s_rt.sum[i] / (float)n;
        float v = s_rt.ss[i] / (float)n - mean * mean;
        l->mean[i] = (int16_t)mean;
        l->ripple[i] = (int16_t)((v > 0.0f) ? sqrtf(v) : 0.0f);
        l->tq[i] = (int16_t)(s_rt.tq[i] / n);
    }
}

static int low_run(void)
{
    rpmt_low_t *l = &agv_rpmt.low[STEP - 1u];
    int16_t g = guard();
    uint16_t i;

    ident_speed(l->target, 1);       /* 바닥 속도(350) 아래로 바로 건다 */
    if (s_rt.t == LOW_TICKS - LOW_WIN) {
        for (i = 0; i < 2u; i++) { l->lo[i] = 32767; l->hi[i] = -32767; }
    }
    if (s_rt.t >= LOW_TICKS - LOW_WIN) {
        for (i = 0; i < 2u; i++) {
            int16_t r = D.rpm[i];
            s_rt.sum[i] += r;
            s_rt.ss[i] += (float)r * (float)r;
            s_rt.tq[i] += D.tq_cmd[i];
            if (r < l->lo[i]) { l->lo[i] = r; }
            if (r > l->hi[i]) { l->hi[i] = r; }
            if (motor_fresh_pulse(i)) { l->upd[i]++; }
            if (motor_idle_ticks(i) == STALL_IDLE) { l->stall[i]++; }
        }
    }
    s_rt.t++;
    if ((g == 0) && (s_rt.t < LOW_TICKS)) { return 0; }
    low_end(l, g);
    if ((g != 0) || (STEP >= RPMT_LOW)) {
        summary();
        return 1;
    }
    agv_rpmt.phase++;
    low_begin(STEP - 1u);
    return 0;
}

/* ---- 5회 350 rpm 아래 정지 ---- */

static int16_t ring_at(uint16_t back) { return s_rt.u.ring[(uint16_t)(s_rt.rn - 1u - back) & (RING - 1u)]; }

/* 섰다(또는 3 s). 지령 0 으로 두고 다음 트리거를 기다린다 */
static int stop_next(void)
{
    ident_speed(0, 0);
    s_rt.stopping = 0;
    s_rt.cruise = 0;
    if (STEP >= RPMT_STOPS) {
        summary();
        return 1;
    }
    agv_rpmt.phase++;
    return 0;
}

static int stop_pass(void)
{
    rpmt_stop_t *sp = &agv_rpmt.stp[STEP - 1u];
    int16_t g = guard(), i0, i1, idle;
    uint16_t k;
    float d, r;

    s_rt.u.ring[s_rt.rn & (RING - 1u)] = avg2(D.rpm[0], D.rpm[1]);
    s_rt.rn++;
    if (!s_rt.stopping) {
        ident_speed(STOP_MIN_RPM, 1);
        if (g != 0) {               /* 직선이 모자라 이 정지부터 못 한다 */
            sp->cut = 2;
            summary();
            return 1;
        }
        s_rt.cruise = (int16_t)(((D.step == DRIVE_CRUISE) && (D.vin_out == STOP_MIN_RPM)) ? s_rt.cruise + 1 : 0);
        if ((s_rt.cruise < CRUISE_TICKS) || ((STEP > 1u) && (mm_from(s_rt.trig) < GAP_MM))) { return 0; }
        {
            int32_t sum = 0;
            for (k = 0; k < V0_TICKS; k++) { sum += ring_at(k); }
            sp->v0 = (int16_t)(sum / V0_TICKS);
        }
        sp->method = (int16_t)(((STEP - 1u) % 3u) + 1u);
        s_rt.trig = travel();
        s_rt.t = 0;
        s_rt.stopping = 1;
        if (sp->method == 1) { ident_speed(0, 0); }   /* 지금 동작: 바닥에서 바로 0 */
        return 0;
    }
    s_rt.t++;
    d = mm_from(s_rt.trig);
    r = s_ramp_mm[sp->method];
    if (sp->method > 1) {           /* 시험이 직접 램프를 낸다. stop.c 는 쓰지 않는다 */
        float rem = r - d;
        ident_speed((rem > 0.0f) ? (int16_t)((float)STOP_MIN_RPM * sqrtf(rem / r)) : 0, 0);
    }
    i0 = motor_idle_ticks(0);
    i1 = motor_idle_ticks(1);
    idle = (i0 < i1) ? i0 : i1;
    if (idle >= STILL_TICKS) {      /* 고리 [idle] = 마지막 펄스 틱 */
        float peak = 0.0f;
        for (k = 0; k < PEAK_TICKS; k++) {
            float dv = (float)(ring_at((uint16_t)idle + k + 1u) - ring_at((uint16_t)idle + k));
            if (dv > peak) { peak = dv; }
        }
        sp->peak_dec = peak * (1000.0f / TICK_MS);
        for (k = 0; k < RPMT_TRACE; k++) { sp->trace[RPMT_TRACE - 1u - k] = ring_at((uint16_t)idle + k); }
        sp->t_ms = (int16_t)((s_rt.t - idle) * TICK_MS);
        sp->dist_mm = d;
        sp->under_mm = ((sp->method > 1) && (d < r)) ? r - d : 0.0f;
        return stop_next();
    }
    if (s_rt.t >= STOPW_TICKS) {
        sp->t_ms = -1;
        sp->cut |= 1;
        sp->dist_mm = d;
        return stop_next();
    }
    return 0;
}

/* ---- 개루프 회 앞 접근 ---- */

static int16_t idle_min(void)
{
    int16_t i0 = motor_idle_ticks(0), i1 = motor_idle_ticks(1);
    return (i0 < i1) ? i0 : i1;
}

/* 접근 끝. 고정은 폐루프가 RUN_DRIVE 로 달리고 있을 때만 건다. 1 = 3회 서기 대기로, -1 = 실패 */
static int open_begin(void)
{
    if (!D.flag_run || (D.control != RUN_DRIVE)) { return -1; }
    s_rt.aw = -1;
    s_rt.as = 0;
    switch (agv_rpmt.pass) {
    case 1: step_begin(&agv_rpmt.up[0], s_up[0]); break;
    case 2: step_begin(&agv_rpmt.dn[0], s_dn[0]); break;
    default:
        s_rt.appr = 2;
        s_rt.at = 0;
        dac_out(0);
        return 1;
    }
    s_rt.appr = 0;
    return 0;
}

/* 0 진행, 1 회 끝(cut), -1 실패 */
static int approach(void)
{
    int16_t g = guard();

    if (++s_rt.at > APPR_LIMIT) { return -1; }
    if (s_rt.appr == 1) {
        ident_speed(P.min_speed, 1);
        if ((g == 0) && (mm_from(s_rt.pass0) < APPR_MM)) { return 0; }
        return (open_begin() < 0) ? -1 : 0;      /* guard 가 걸렸으면 첫 단계 첫 틱에 끊는다 */
    }
    dac_out(0);                      /* 3회: 섰다(두 바퀴 200 ms 조용) */
    if (idle_min() >= STILL_TICKS) {
        s_rt.appr = 0;
        s_rt.t = 0;
        return 0;
    }
    if ((g == 0) && (s_rt.at < STOPW_TICKS)) { return 0; }
    agv_rpmt.brk.cut = (g != 0) ? g : 1;
    summary();
    return 1;
}

/* ---- ident.c 가 부른다 ---- */

void rpmt_pass_begin(void)
{
    rpmt_brk_t *b = &agv_rpmt.brk;
    uint16_t i;

    agv_rpmt.pass++;
    /* 연속 시험은 4회만. 점검(t2.4)은 앰프 형식을 보려고 1회도 한다 */
    if (ident_build_mode() && ((agv_rpmt.pass > 1) || !(agv_build.mask & BLD_CHECK))) { agv_rpmt.pass = 4; }
    agv_rpmt.phase = 1;
    s_rt.n = 0;
    s_rt.t = 0;
    s_rt.rn = 0;
    s_rt.stopping = s_rt.cruise = 0;
    s_rt.pass0 = s_rt.trig = s_rt.step0 = travel();
    s_rt.dw[0] = s_rt.dw[1] = 0;
    s_rt.appr = (agv_rpmt.pass <= 3) ? 1 : 0;
    s_rt.at = 0;
    if (agv_rpmt.pass == 3) {
        for (i = 0; i < 2u; i++) { b->move_dac[i] = b->stop_dac[i] = -1; }
    }
    if (agv_rpmt.pass == 4) { low_begin(0); }
}

/* 측정 중 교차선. 지금 단계를 why 로 끊고 회를 끝낸다 */
void rpmt_pass_cut(int16_t why)
{
    switch (agv_rpmt.pass) {
    case 1:  step_end(&agv_rpmt.up[STEP - 1u], why); break;
    case 2:  step_end(&agv_rpmt.dn[STEP - 1u], why); break;
    case 3:  agv_rpmt.brk.cut = why; break;
    case 4:  low_end(&agv_rpmt.low[STEP - 1u], why); break;
    case 5:  agv_rpmt.stp[STEP - 1u].cut |= why; break;
    default: break;
    }
    s_rt.appr = 0;
    summary();
}

int rpmt_pass_run(void)
{
    if (s_rt.appr) { return approach(); }
    switch (agv_rpmt.pass) {
    case 1:  return stair(agv_rpmt.up, s_up, RPMT_UP, UP_TICKS);
    case 2:  return stair(agv_rpmt.dn, s_dn, RPMT_DN, DN_TICKS);
    case 3:  return brk_run();
    case 4:  return low_run();
    case 5:  return stop_pass();
    default: return -1;
    }
}
