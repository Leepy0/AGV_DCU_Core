/* test_curve.c - 곡선 폐루프 모의. hal/host 위에서 부팅부터 10 ms 스캔까지 실제 코드를 돌린다.
 *
 * 실제 코드: 'h' 응답 해석·추적(guide_feed, guide_track, guide_poll), 조향(steer_run), 주행 단계와 위험 감속
 *            (motor_run, motor_drive_step), S커브, rpm 피드백, 곡선 기록기(curvelog_tick).
 * 모형:      바퀴 rpm 은 바퀴 지령(w_cmd)을 1차 지연으로 따라간다(가감속 한계, 최고 rpm). 토크 지령은 쓰지 않는다.
 *            차동 구동 운동학(휠 간격 × 회전 보정비), 회전 중심에서 GD 앞 센서, 테이프를 0.35 칸 넘게 덮은 LED 가 켜진다.
 *            센서는 10 ms 마다 그때 자세로 답한다(센서 안 지연 없음). 미끄럼·바닥 기울기·하중 없음.
 * 경로:      직선 → 원호(반경·각) → 직선.
 *
 * 인자 없이 돌리면 회귀 검사만 한다. 인자 sweep 은 미지수(LOS, 속도 루프 지연)를 훑은 표를 낸다.
 */
#include <assert.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "agv/curvelog.h"
#include "agv/drive.h"
#include "agv/guide.h"
#include "agv/ioport.h"
#include "agv/motor.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/seq.h"
#include "agv/sys.h"
#include "agv/tagact.h"
#include "agv/tune.h"
#include "hal_host.h"

#define D agv_drive
#define P param_dv

#define RAW_START   0x04
#define PITCH_MM    6.89f
#define TAPE_MM     10.0f
#define WB_MM       600.0f
#define TURN_SCALE  1.023f      /* 회전 보정비 */
#define GD_MM       250.0f
#define RPM_MAX     3540.0f
#define ACC_LIM     6000.0f     /* 바퀴 가감속 한계 rpm/s */
#define LEAD_MM     6000.0f     /* 곡선 앞 직선. 가속 구간 + 정속 2 m */
#define TAIL_MM     2000.0f
#define PI_F        3.14159265f

void site_register(void) { param_default_type(0); }
void site_manual(void)   { }

typedef struct {
    int   leds;             /* 32 wide, 16 일반 */
    float r_mm, deg;        /* 원호. r_mm 0 = 직선만(mode 2 는 deg 가 옆으로 꺾인 단위) */
    int   dir;              /* 1 왼쪽, -1 오른쪽 */
    int   v_mpm, los, drs;  /* 진입 속도, LOS, DRS (m/min) */
    float tau;              /* 속도 루프 시정수 s */
    int   lg;               /* LG */
    int   mode;             /* 0 곡선, 1 옆 4단위 계단 지령, 2 편차 5단위에서 복귀,
                               3 테이프 100 mm 끊김 + 그 안 옆 8단위 자리에 길이 deg mm 인 점(바닥 반사) */
} case_t;

typedef struct {
    float rms, max;         /* 곡선 구간 편차 mm (참값) */
    float body_r, v_avg;    /* 차체 반경 mm, 곡선 평균 속도 m/min */
    float exit_max;         /* 출구 뒤 편차 최대 mm */
    int   exit_flip;        /* 출구 뒤 부호 바뀜 횟수(±1단위 넘은 것만) */
    float tail_pp;          /* 끝 1 m 의 편차 폭 mm. 지속 발진 지표 */
    int   lost, risk, err;  /* 선 잃음 틱, 위험 감속 틱, 정지 오류 */
    float over, steady;     /* 직선: 넘침(단위), 끝 1 m 평균 편차(단위) */
    int   jump;             /* 직선: 계단·끊김 뒤 가이드 위치 절대값 최대(단위) */
    curve_rec_t rec;        /* 곡선 기록기 첫 기록 */
    int   rec_n;
} res_t;

static struct { float x, y, th, rpm[2], acc[2], s; } s_v;
static const case_t *s_c;

/* 경로 점과 접선. s = 경로 길이 mm */
static void path(float s, float *px, float *py, float *tx, float *ty)
{
    float arc = (s_c->r_mm > 0.0f) ? s_c->r_mm * s_c->deg * PI_F / 180.0f : 0.0f;
    float d = (float)s_c->dir;
    if ((s <= LEAD_MM) || (arc <= 0.0f)) { *px = s; *py = 0.0f; *tx = 1.0f; *ty = 0.0f; return; }
    {
        float a = ((s < LEAD_MM + arc) ? (s - LEAD_MM) : arc) / s_c->r_mm;
        float rest = (s < LEAD_MM + arc) ? 0.0f : (s - LEAD_MM - arc);
        *tx = cosf(a);
        *ty = d * sinf(a);
        *px = LEAD_MM + s_c->r_mm * sinf(a) + rest * *tx;
        *py = d * s_c->r_mm * (1.0f - cosf(a)) + rest * *ty;
    }
}

/* 센서 줄과 테이프가 만나는 점. 반환 1 = 있음. u = 센서 가운데에서 왼쪽으로 mm, slant = 비스듬히 지나며 넓어지는 배율 */
static int cross(float *u, float *slant)
{
    float hx = cosf(s_v.th), hy = sinf(s_v.th);
    float sx = s_v.x + GD_MM * hx, sy = s_v.y + GD_MM * hy;
    float px, py, tx, ty, dot = 1.0f;
    int k;
    for (k = 0; k < 8; k++) {
        path(s_v.s, &px, &py, &tx, &ty);
        dot = tx * hx + ty * hy;
        if (dot < 0.2f) { return 0; }
        s_v.s -= ((px - sx) * hx + (py - sy) * hy) / dot;
    }
    path(s_v.s, &px, &py, &tx, &ty);
    *u = -(px - sx) * hy + (py - sy) * hx;
    *slant = 1.0f / dot;
    return 1;
}

static float cover(float c, float w, int i)
{
    float lo = c - w / 2.0f, hi = c + w / 2.0f;
    float a = (lo > (float)i - 0.5f) ? lo : (float)i - 0.5f;
    float b = (hi < (float)i + 0.5f) ? hi : (float)i + 0.5f;
    return (b > a) ? (b - a) : 0.0f;
}

static void send_h(uint32_t bits, int leds)
{
    static const char hex[] = "0123456789abcdef";
    host_sci_t *b = &host.sci[HAL_SCI_B];
    uint16_t f[12], sum = 0;
    int i, n = leds / 4;

    if (b->rx_pos >= b->rx_len) { b->rx_pos = b->rx_len = 0; }
    f[0] = 2;
    f[1] = 'h';
    for (i = 0; i < n; i++) { f[2 + i] = (uint16_t)hex[(bits >> (i * 4)) & 0xFu]; }
    for (i = 1; i < 2 + n; i++) { sum = (uint16_t)(sum + f[i]); }
    f[2 + n] = (uint16_t)hex[sum & 0xFu];
    f[3 + n] = 3;
    for (i = 0; i < 4 + n; i++) { b->rx[b->rx_len++] = f[i]; }
}

static void sensor_reply(float u, float slant, int seen)
{
    uint32_t bits = 0;
    int i, leds = s_c->leds;
    if (seen) {
        float c = (float)(leds + 1) / 2.0f + u / PITCH_MM;
        for (i = 1; i <= leds; i++) {
            if (cover(c, TAPE_MM * slant / PITCH_MM, i) > 0.35f) { bits |= 1UL << (i - 1); }
        }
    }
    send_h(bits, leds);
}

static void plant(void)
{
    float k = 1000.0f / 60.0f / P.one_meter_rpm;      /* rpm → mm/s */
    int i, n;
    for (i = 0; i < 2; i++) {
        float cmd = D.amp_run ? (float)D.w_cmd[i] : 0.0f;
        float prev = s_v.rpm[i], lim = ACC_LIM * 0.01f;
        s_v.rpm[i] += (cmd - s_v.rpm[i]) * (0.01f / s_c->tau);
        if (s_v.rpm[i] - prev > lim)  { s_v.rpm[i] = prev + lim; }
        if (prev - s_v.rpm[i] > lim)  { s_v.rpm[i] = prev - lim; }
        if (s_v.rpm[i] > RPM_MAX)     { s_v.rpm[i] = RPM_MAX; }
        if (s_v.rpm[i] > 5.0f) {
            s_v.acc[i] += s_v.rpm[i] / 500.0f;
            while (s_v.acc[i] >= 1.0f) {
                s_v.acc[i] -= 1.0f;
                sys_encoder((uint16_t)i, (uint32_t)(500000000.0f / s_v.rpm[i]));
            }
        }
    }
    for (n = 0; n < 5; n++) {
        float vl = s_v.rpm[0] * k, vr = s_v.rpm[1] * k, v = (vl + vr) * 0.5f;
        s_v.th += (vr - vl) / (WB_MM * TURN_SCALE) * 0.002f;
        s_v.x += v * cosf(s_v.th) * 0.002f;
        s_v.y += v * sinf(s_v.th) * 0.002f;
    }
}

static void scan(void)
{
    int i;
    for (i = 0; i < 5; i++) { sys_tick_2ms(); }
    sys_scan();
}

static void sim(const case_t *c, res_t *r)
{
    float arc = (c->r_mm > 0.0f) ? c->r_mm * c->deg * PI_F / 180.0f : 0.0f;
    float end = LEAD_MM + arc + TAIL_MM;
    float n = 0, sum2 = 0, th0 = 0, len = 0, px = 0, py = 0, vsum = 0;
    float tail_lo = 1e9f, tail_hi = -1e9f, st_sum = 0, st_n = 0, step_at = 3000.0f;
    int   t, in_prev = 0, side = 0, sped = 0, jog = 0;

    memset(r, 0, sizeof(*r));
    memset(&s_v, 0, sizeof(s_v));
    s_c = c;
    s_v.s = GD_MM;

    host_reset();
    memset(&D, 0, sizeof(D));
    sys_boot();
    host.adc = 300u << 4;
    for (t = 0; t < 60; t++) { scan(); }
    assert(sys_power_on());
    assert(param_set_ram(P_LEGACY_QUIRKS, (int16_t)c->lg) == 0);
    assert(param_set_ram(P_GEAR_RATIO, 60) == 0);              /* 94.5 rpm 당 1 m/min */
    assert(param_set_ram(P_WHEEL_BASE_MM, (int16_t)(WB_MM / 10.0f)) == 0);
    assert(param_set_ram(P_GUIDE_DIST_MM, (int16_t)GD_MM) == 0);
    assert(param_set_ram(P_SPEED_HIGH, (int16_t)c->drs) == 0);
    assert(param_set_ram(P_SPEED_LOW, (int16_t)c->los) == 0);
    tune_turn_scale_set((int16_t)(TURN_SCALE * 1000.0f + 0.5f));
    curvelog_reset();

    host.bus_in[BUS_IN_A] &= (uint16_t)~RAW_START;
    scan();
    host.bus_in[BUS_IN_A] |= RAW_START;
    scan();
    assert(D.flag_ready == 1);

    for (t = 0; t < 6000; t++) {
        float u = 0.0f, slant = 1.0f, dev;
        int seen = cross(&u, &slant);
        int in_curve = (arc > 0.0f) && (s_v.s > LEAD_MM) && (s_v.s <= LEAD_MM + arc);
        int after = (arc > 0.0f) ? (s_v.s > LEAD_MM + arc) : (s_v.s > step_at);

        if (s_v.s >= end) { break; }
        if (seen && (fabsf(u) > (float)c->leds * PITCH_MM / 2.0f + TAPE_MM)) { seen = 0; }
        if ((c->mode == 3) && (s_v.s > step_at) && (s_v.s <= step_at + 100.0f)) {
            seen = (s_v.s > step_at + 45.0f) && (s_v.s <= step_at + 45.0f + c->deg);
            u += 8.0f * GUIDE_UNIT_MM;
            slant = 1.0f;
        }
        sensor_reply(u, slant, seen);
        plant();
        scan();
        if (!sped && D.flag_run && (s_v.s > 500.0f)) { tagact_speed((uint16_t)c->v_mpm); sped = 1; }
        if ((c->mode == 1) && (s_v.s > step_at)) { D.p_test = 4; }
        if ((c->mode == 2) && (s_v.s > step_at) && !jog) { s_v.y -= c->deg * GUIDE_UNIT_MM; jog = 1; }   /* 테이프가 왼쪽으로 deg 단위 꺾였다 */

        dev = seen ? u : (float)c->leds * PITCH_MM / 2.0f;
        if (c->mode == 3) { dev = -s_v.y - GD_MM * sinf(s_v.th); }   /* 끊긴 자리에서도 주행선 기준 */
        if (D.guide_all_off && D.flag_run) { r->lost++; }
        if (D.risk_stop) { r->risk++; }
        if (in_curve) {
            float d = hypotf(s_v.x - px, s_v.y - py);
            if (!in_prev) { th0 = s_v.th; d = 0.0f; }
            len += d;
            n += 1.0f;
            sum2 += dev * dev;
            vsum += (s_v.rpm[0] + s_v.rpm[1]) * 0.5f / P.one_meter_rpm;
            if (fabsf(dev) > r->max) { r->max = fabsf(dev); }
            r->body_r = (fabsf(s_v.th - th0) > 1e-4f) ? len / fabsf(s_v.th - th0) : 0.0f;
        }
        if (after) {
            int a = (D.guide_pos >= 0) ? D.guide_pos : -D.guide_pos;
            if (a > r->jump) { r->jump = a; }
            float e = (c->mode == 1) ? dev - 4.0f * GUIDE_UNIT_MM : dev;
            int sg = (e > GUIDE_UNIT_MM) ? 1 : ((e < -GUIDE_UNIT_MM) ? -1 : 0);
            if (fabsf(e) > r->exit_max) { r->exit_max = fabsf(e); }
            if ((sg != 0) && (side != 0) && (sg != side)) { r->exit_flip++; }
            if (sg != 0) { side = sg; }
            if (s_v.s > end - 1000.0f) {
                if (e < tail_lo) { tail_lo = e; }
                if (e > tail_hi) { tail_hi = e; }
                st_sum += e;
                st_n += 1.0f;
            }
            if ((c->mode == 1) && (e > r->over * GUIDE_UNIT_MM)) { r->over = e / GUIDE_UNIT_MM; }
        }
        if ((c->mode == 2) && jog && (-dev > r->over * GUIDE_UNIT_MM)) { r->over = -dev / GUIDE_UNIT_MM; }   /* 반대쪽으로 넘어간 양 */
        in_prev = in_curve;
        px = s_v.x;
        py = s_v.y;
    }
    r->err = D.stop_err ? D.stop_err : D.system_err;
    if (s_v.s < end) { r->err = r->err ? r->err : -1; }        /* 끝까지 못 갔다 */
    r->rms = (n > 0) ? sqrtf(sum2 / n) : 0.0f;
    r->v_avg = (n > 0) ? vsum / n : 0.0f;
    r->tail_pp = (st_n > 0) ? tail_hi - tail_lo : 0.0f;
    r->steady = (st_n > 0) ? st_sum / st_n / GUIDE_UNIT_MM : 0.0f;
    r->rec_n = agv_curvelog.g[0].n;
    r->rec = agv_curvelog.g[0].c[0];
}

static void show(const char *name, const case_t *c, const res_t *r)
{
    printf("  %-10s R%4.0f v%2d LOS%2d tau%3.0f LG%3d L%2d | rms %5.1f max %5.1f 차체R %5.0f v %4.1f | 출구 max %5.1f 바뀜 %d 끝폭 %4.1f | 잃음 %d 위험 %d err %d"
           " | 기록 n%d rms %5.1f max %5.1f 차체R %d 테이프R %d len %d v %4.1f 넘침 %4.1f\n",
           name, c->r_mm, c->v_mpm, c->los, c->tau * 1000.0f, c->lg, c->leds, r->rms, r->max, r->body_r, r->v_avg,
           r->exit_max, r->exit_flip, r->tail_pp, r->lost, r->risk, r->err,
           r->rec_n, r->rec.dev_rms_x10 / 10.0f, r->rec.dev_max_x10 / 10.0f, r->rec.r_body_mm, r->rec.r_tape_mm, r->rec.len_mm,
           r->rec.v_x10 / 10.0f, r->rec.exit_over_x10 / 10.0f);
}

/* 실차 곡선(반경 0.55~0.96 m) 네 개. 각은 기록된 차체 방향 변화에서 잡았다 */
static const struct { const char *name; float r, deg; int dir; } s_curves[4] = {
    { "A", 650.0f, 45.0f, -1 }, { "B", 551.0f, 45.0f, 1 }, { "C", 909.0f, 90.0f, 1 }, { "D", 961.0f, 90.0f, -1 },
};

/* 미지수 훑기: LOS, 속도 루프 지연, 조향(LG 0 표준, 256 예전 조향, 368 예전 조향 전부) */
static void sweep(void)
{
    static const int   los[3] = { 25, 28, 31 };
    static const float tau[3] = { 0.05f, 0.1f, 0.15f };
    static const int   lg[3] = { 0, 256, 368 };
    int i, k, t, g;
    res_t r;
    for (g = 0; g < 3; g++) {
        for (k = 0; k < 3; k++) {
            for (t = 0; t < 3; t++) {
                for (i = 0; i < 4; i++) {
                    case_t c = { 32, s_curves[i].r, s_curves[i].deg, s_curves[i].dir, 31, los[k], 31, tau[t], lg[g], 0 };
                    sim(&c, &r);
                    show(s_curves[i].name, &c, &r);
                }
            }
        }
    }
}

static void on_abort(int sig) { (void)sig; _exit(3); }

int main(int argc, char **argv)
{
    int i, l, g;
    res_t r;

    signal(SIGABRT, on_abort);
    setvbuf(stdout, 0, _IONBF, 0);
    if ((argc > 1) && (strcmp(argv[1], "sweep") == 0)) { sweep(); return 0; }

    for (g = 0; g < 2; g++) {
        for (l = 32; l >= 16; l -= 16) {
            for (i = 0; i < 4; i++) {
                case_t c = { l, s_curves[i].r, s_curves[i].deg, s_curves[i].dir, 31, 31, 31, 0.1f, g ? QUIRK_STEER_STEP : 0, 0 };
                sim(&c, &r);
                show(s_curves[i].name, &c, &r);
                assert(r.risk > 0);                             /* 편차 10단위를 넘어 위험 감속에 들어간다 */
                if (g == 0) {
                    assert((r.lost == 0) && (r.err == 0));
                    assert(r.max < ((i < 2) ? 53.0f : 42.0f));  /* wide 센서 끝(107 mm)의 절반. 일반 센서도 같은 편차다 */
                    assert((r.exit_flip <= 1) && (r.tail_pp < 20.0f));
                }
                else if (l == 16) { assert(r.lost > 0); }       /* 예전 조향: 일반 센서는 선을 잃는다 */
                else if (i >= 2)  { assert((r.rms > 60.0f) && (r.max > 80.0f)); }   /* 실차 곡선에서 잰 큰 편차(RMS 75~80 mm) */
                else              { assert(r.max > 90.0f); }
            }
        }
    }
    for (i = 1; i <= 2; i++) {      /* 직선: 옆 4단위 계단 지령, 5단위 꺾임에서 복귀 */
        case_t c = { 32, 0.0f, 5.0f, 1, 31, 31, 31, 0.1f, 0, i };
        sim(&c, &r);
        printf("  직선 %d: 넘침 %.2f 단위, 끝 평균 %.2f, 바뀜 %d, 끝폭 %.1f mm\n", i, r.over, r.steady, r.exit_flip, r.tail_pp);
        assert((r.lost == 0) && (r.err == 0) && (r.risk == 0));
        assert(r.over < ((i == 1) ? 1.9f : 3.2f));              /* 고치기 전 2.03, 2.80 */
        assert((r.exit_flip <= 2) && (r.tail_pp < 5.0f) && (fabsf(r.steady) < 1.0f));
    }
    for (i = 10; i <= 40; i += 10) {     /* 끊긴 자리의 점. 31 m/min 은 응답당 5.2 mm: 10 mm 점은 2응답, 20 mm 는 3~4응답 */
        case_t c = { 32, 0.0f, (float)i, 1, 31, 31, 31, 0.1f, 0, 3 };
        sim(&c, &r);
        printf("  끊김 100 mm + 점 %d mm: 위치 최대 %d 단위, 편차 최대 %.1f mm, 잃음 %d 틱\n", i, r.jump, r.exit_max, r.lost);
        assert((r.err == 0) && (r.lost >= 15) && (r.exit_max < 20.0f));
        if (i == 10) { assert((r.jump <= 1) && (r.exit_max < 1.0f)); }   /* GUIDE_REACQ_N 응답보다 짧은 점은 받지 않는다 */
        if (i == 40) { assert(r.jump >= 8); }                            /* 긴 것은 받는다(지속 확인이 없으면 10 mm 도 이렇다) */
    }
    printf("test_curve ok\n");
    return 0;
}
