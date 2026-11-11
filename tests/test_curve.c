/* test_curve.c - 곡선 폐루프 모의. hal/host 위에서 부팅부터 10 ms 스캔까지 실제 코드를 돌린다.
 *
 * 실제 코드: 'h' 응답 해석·추적(guide_feed, guide_track, guide_poll), 조향(steer_run), 주행 단계와 위험 감속
 *            (motor_run, motor_drive_step), S커브, rpm 피드백, 곡선 기록기(curvelog_tick).
 * 모형:      바퀴 rpm 은 바퀴 지령(w_cmd)을 1차 지연으로 따라간다(가감속 한계, 최고 rpm). 토크 지령은 쓰지 않는다.
 *            엔코더는 모터 12펄스/회전 그대로 낸다(펄스 길이 = 바퀴 둘레/기어비/12). 조향 추정기는 이 정수 펄스로 적분한다.
 *            강건성(s_m): 실제 휠 간격·GD 배율, 왼쪽 펄스 손실(미끄럼), 오른쪽 바퀴 이동 배율(좌우 바퀴 지름 차).
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
#include <stdlib.h>
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
#include "agv/stop.h"
#include "hal_host.h"

#define D agv_drive
#define P param_dv

#define RAW_START   0x04
#define PITCH_MM    6.89f
#define TAPE_MM     10.0f
#define WB_MM       600.0f
#define TURN_SCALE  1.023f      /* 회전 보정비 */
#define GD_MM       s_gd
#define RPM_MAX     3540.0f
#define ACC_LIM     6000.0f     /* 바퀴 가감속 한계 rpm/s */
#define LEAD_MM     6000.0f     /* 곡선 앞 직선. 가속 구간 + 정속 2 m */
#define TAIL_MM     2000.0f
#define PI_F        3.14159265f
#define STEER_SLEW_TEST 50.5f   /* steer.c STEER_SLEW_RPM + 반올림 */

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
                               3 테이프 100 mm 끊김 + 그 안 옆 8단위 자리에 길이 deg mm 인 점(바닥 반사),
                               4 출발 때 회전 중심이 오른쪽으로 deg 단위(센서 위치 +deg) */
} case_t;

typedef struct {
    float rms, max;         /* 곡선 구간 편차 mm (참값) */
    float body_r, v_avg;    /* 차체 반경 mm, 곡선 평균 속도 m/min */
    float exit_max;         /* 출구 뒤 편차 최대 mm */
    int   exit_flip;        /* 출구 뒤 부호 바뀜 횟수(±1단위 넘은 것만) */
    float eover;            /* 출구 뒤 곡선 최대 편차와 반대쪽으로 넘어간 최대 mm */
    float settle;           /* 출구(직선은 계단) 뒤 ±2단위 안으로 들어와 머물기까지 거리 mm. 끝까지 못 들어오면 꼬리 길이 */
    float max_sg;           /* 곡선 최대 편차의 부호 */
    float tail_pp;          /* 끝 1 m 의 편차 폭 mm. 지속 발진 지표 */
    int   lost, risk, err;  /* 선 잃음 틱, 위험 감속 틱, 정지 오류 */
    float over, steady;     /* 직선: 넘침(단위), 끝 1 m 평균 편차(단위) */
    int   jump;             /* 직선: 계단·끊김 뒤 가이드 위치 절대값 최대(단위) */
    curve_rec_t rec;        /* 곡선 기록기 첫 기록 */
    int   rec_n;
    /* 계단 지표: 곡선 구간 조향 출력(D.p_pi_out)의 틱(10 ms)당 변화 절대값 rpm */
    float st_max, st_p95, st_n50;   /* 최대, 상위 5%, 50 rpm 넘는 변화 횟수(초당) */
    float st_chg;           /* 50 rpm 넘는 변화 중 그 틱이나 한 틱 앞에 위치 칸이 바뀐 비율 % */
    float st_per_unit;      /* 칸이 바뀐 틱의 출력 변화 / 위치 변화 평균 rpm/단위 */
    float st_wd_p95;        /* 바퀴 지령 차의 절반((w_cmd[0]-w_cmd[1])/2) 틱당 변화 상위 5% */
    float j_p95, j_n50;     /* 틱당 변화의 변화(|Δ(t)-Δ(t-1)|) 상위 5%, 50 rpm 넘는 횟수(초당). 꾸준한 기울기는 0, 계단은 크다 */
    float cerr;             /* 곡선 구간 회전 중심의 경로 오차 최대 mm(센서가 테이프 위면 안쪽으로 반경 - √(반경² - GD²)) */
    float tail_rms;         /* 끝 1 m 편차 RMS mm(직선 정속, 센서 양자화 포함) */
    float a_stmax, a_jn;    /* 직선: 계단·꺾임 0.3 m 뒤부터 출력 틱당 변화 최대 rpm, 변화의 변화 50 rpm 넘는 횟수/s */
    float pass_s, vmin;     /* 곡선(센서 투영 곡선 앞 0.5 m ~ 뒤 0.5 m) 통과 시간 s, 그 동안 최저 속도 m/min */
    float izero_s;          /* 안쪽 바퀴 지령 0 에 걸린 시간 s */
    float cmax;             /* 회전 중심 경로 오차 최대 mm(곡선 앞뒤 포함) */
} res_t;

/* 모형 어긋남: 실제 휠 간격·GD 배율, 왼쪽 펄스 손실 %, 오른쪽 바퀴 이동 배율 */
static struct { float wbx, gdx, slip, wr, pl; } s_m = { 1.0f, 1.0f, 0.0f, 1.0f, 1.0f };   /* pl: 실제 펄스 길이 배율(두 바퀴) */

static struct { float x, y, th, rpm[2], acc[2], s, drop; } s_v;
static float s_gd = 250.0f;     /* 센서 거리(모형과 GD 같음) */
static float s_cross_x, s_cross_w;   /* 교차 테이프 x mm·폭 mm. 폭 0 = 없음 */
static const case_t *s_c;
static float s_st[6000], s_wd[6000], s_j[6000];
static int   s_trace;       /* one ... 1: 틱마다 찍는다 */
static int   s_gmask = 7;   /* cmp 에서 돌릴 조향(b0 새, b1 37차, b2 예전) */

static int cmp_f(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

static float p95(float *v, int n)
{
    if (n <= 0) { return 0.0f; }
    qsort(v, (size_t)n, sizeof(float), cmp_f);
    return v[(n * 95) / 100];
}

/* 경로 점과 접선. s = 경로 길이 mm */
static void path(float s, float *px, float *py, float *tx, float *ty)
{
    float arc = (s_c->r_mm > 0.0f) ? s_c->r_mm * s_c->deg * PI_F / 180.0f : 0.0f;
    float x = LEAD_MM, y = 0.0f, h = 0.0f, rem = s - LEAD_MM;
    int k, n = (s_c->mode == 5) ? 2 : 1;      /* mode 5: 같은 반경으로 반대쪽 원호를 잇는 S 곡선 */
    if ((s <= LEAD_MM) || (arc <= 0.0f)) { *px = s; *py = 0.0f; *tx = 1.0f; *ty = 0.0f; return; }
    for (k = 0; k < n; k++) {
        float kap = (float)(k ? -s_c->dir : s_c->dir) / s_c->r_mm;
        float l = (rem < arc) ? rem : arc, a = l * kap;
        x += (sinf(h + a) - sinf(h)) / kap;
        y += (cosf(h) - cosf(h + a)) / kap;
        h += a;
        rem -= l;
        if (rem <= 0.0f) { break; }
    }
    *tx = cosf(h);
    *ty = sinf(h);
    *px = x + rem * *tx;
    *py = y + rem * *ty;
}

/* 30도 교차선(mode 6): 주행선(x 축)과 x = s_x30 에서 30도로 만나는 직선, 폭 s_x30w mm */
static float s_x30, s_x30w, s_xa = 30.0f;   /* s_xa: 교차선 각(도). 정렬 회전 모의는 90 */

/* 센서 줄과 테이프가 만나는 점. 반환 1 = 있음. u = 센서 가운데에서 왼쪽으로 mm, slant = 비스듬히 지나며 넓어지는 배율 */
static int cross(float *u, float *slant)
{
    float hx = cosf(s_v.th), hy = sinf(s_v.th);
    float sx = s_v.x + GD_MM * s_m.gdx * hx, sy = s_v.y + GD_MM * s_m.gdx * hy;
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

/* 회전 중심의 경로 옆 거리 mm(+ 왼쪽). 경로 위 가장 가까운 점은 센서 점 투영에서 GD 뒤부터 찾는다 */
static float center_err(void)
{
    float s = s_v.s - GD_MM, px, py, tx, ty;
    int k;
    for (k = 0; k < 8; k++) {
        path(s, &px, &py, &tx, &ty);
        s += (s_v.x - px) * tx + (s_v.y - py) * ty;
    }
    path(s, &px, &py, &tx, &ty);
    return -(s_v.x - px) * ty + (s_v.y - py) * tx;
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
    if ((s_cross_w > 0.0f) && (fabsf(s_v.x + GD_MM * cosf(s_v.th) - s_cross_x) < s_cross_w * 0.5f)) {
        send_h((leds == 32) ? 0xFFFFFFFFUL : 0xFFFFUL, leds);    /* 센서 줄이 교차 테이프 위: 전부 켜짐 */
        return;
    }
    if (seen) {
        float c = (float)(leds + 1) / 2.0f + u / PITCH_MM;
        for (i = 1; i <= leds; i++) {
            if (cover(c, TAPE_MM * slant / PITCH_MM, i) > 0.35f) { bits |= 1UL << (i - 1); }
        }
    }
    if (s_x30w > 0.0f) {        /* 교차선과 센서 줄이 만나는 자리 u2. 폭은 줄 방향으로 w / |sin| */
        float nx = -sinf(s_v.th), ny = cosf(s_v.th), dx = cosf(s_xa * PI_F / 180.0f), dy = sinf(s_xa * PI_F / 180.0f);
        float sx = s_v.x + GD_MM * cosf(s_v.th), sy = s_v.y + GD_MM * sinf(s_v.th);
        float cr = nx * dy - ny * dx;
        if (fabsf(cr) < 0.02f) {        /* 센서 줄이 선과 나란하다: 선 위면 전부 켜짐 */
            if (fabsf((s_x30 - sx) * dy - (0.0f - sy) * dx) < s_x30w * 0.5f) { bits = (leds == 32) ? 0xFFFFFFFFUL : 0xFFFFUL; }
        }
        else {
            float u2 = ((s_x30 - sx) * dy - (0.0f - sy) * dx) / cr;
            float c2 = (float)(leds + 1) / 2.0f + u2 / PITCH_MM;
            for (i = 1; i <= leds; i++) {
                if (cover(c2, s_x30w / fabsf(cr) / PITCH_MM, i) > 0.35f) { bits |= 1UL << (i - 1); }
            }
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
                if (i == 0) {           /* 미끄럼: 왼쪽 펄스를 slip % 버린다 */
                    s_v.drop += s_m.slip * 0.01f;
                    if (s_v.drop >= 1.0f) { s_v.drop -= 1.0f; continue; }
                }
                sys_encoder((uint16_t)i, (uint32_t)(500000000.0f / s_v.rpm[i]));
            }
        }
    }
    for (n = 0; n < 5; n++) {
        float vl = s_v.rpm[0] * k * s_m.pl, vr = s_v.rpm[1] * k * s_m.wr * s_m.pl, v;
        if (D.control == RUN_TURN) {    /* 제자리 회전: CW 는 왼쪽 앞으로·오른쪽 뒤로 */
            if (D.motion == MOTION_CW) { vr = -vr; } else { vl = -vl; }
        }
        v = (vl + vr) * 0.5f;
        s_v.th += (vr - vl) / (WB_MM * TURN_SCALE * s_m.wbx) * 0.002f;
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
    float arc = (c->r_mm > 0.0f) ? c->r_mm * c->deg * PI_F / 180.0f * ((c->mode == 5) ? 2.0f : 1.0f) : 0.0f;
    int   an = 0, aj = 0;
    float a_prev = 0.0f, a_d = 0.0f;
    float end = LEAD_MM + arc + TAIL_MM;
    float n = 0, sum2 = 0, th0 = 0, len = 0, px = 0, py = 0, vsum = 0;
    float tail_lo = 1e9f, tail_hi = -1e9f, st_sum = 0, st_n = 0, st_sum2 = 0, step_at = (c->mode == 4) ? 0.0f : 3000.0f;
    int   t, in_prev = 0, side = 0, sped = 0, jog = 0;
    int   ns = 0, n50 = 0, n50c = 0, chg_n = 0, pos_prev = 0, chg_prev = 0;
    float out_prev = 0.0f, wd_prev = 0.0f, per_sum = 0.0f, d_prev = 0.0f;
    int   nj50 = 0;

    memset(r, 0, sizeof(*r));
    memset(&s_v, 0, sizeof(s_v));
    s_c = c;
    s_v.s = GD_MM;
    if (c->mode == 4) { s_v.y = -c->deg * GUIDE_UNIT_MM; jog = 1; }

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

    r->vmin = 1e9f;
    for (t = 0; t < 40000; t++) {
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
        s_x30w = ((c->mode == 6) && (fabsf(s_v.x - 3000.0f) < 1500.0f)) ? c->deg : 0.0f;
        s_x30 = 3000.0f;
        sensor_reply(u, slant, seen);
        plant();
        scan();
        if (!sped && D.flag_run && (s_v.s > 500.0f)) { tagact_speed((uint16_t)c->v_mpm); sped = 1; }
        if ((c->mode == 1) && (s_v.s > step_at)) { D.p_test = 4; }
        if ((c->mode == 2) && (s_v.s > step_at) && !jog) { s_v.y -= c->deg * GUIDE_UNIT_MM; jog = 1; }   /* 테이프가 왼쪽으로 deg 단위 꺾였다 */

        dev = seen ? u : (float)c->leds * PITCH_MM / 2.0f;
        if (c->mode == 3) { dev = -s_v.y - GD_MM * sinf(s_v.th); }   /* 끊긴 자리에서도 주행선 기준 */
        if (D.guide_all_off && D.flag_run) { r->lost++; }
        if ((arc > 0.0f) && (s_v.s > LEAD_MM - 500.0f) && (s_v.s < LEAD_MM + arc + 500.0f)) {
            float vv = (s_v.rpm[0] + s_v.rpm[1]) * 0.5f / P.one_meter_rpm, ce = fabsf(center_err());
            r->pass_s += 0.01f;
            if (vv < r->vmin) { r->vmin = vv; }
            if (ce > r->cmax) { r->cmax = ce; }
        }
        if ((D.control == RUN_DRIVE) && (D.vin_out > 0) && ((D.w_cmd[0] == 0) || (D.w_cmd[1] == 0))) { r->izero_s += 0.01f; }
        if (D.risk_stop) { r->risk++; }
        {   /* 계단 지표. 위치 칸이 바뀐 틱과 출력 점프를 맞춰 본다 */
            int chg = (D.guide_pos != pos_prev);
            float wd = (float)(D.w_cmd[0] - D.w_cmd[1]) * 0.5f;
            if (in_curve && in_prev && (D.control == RUN_DRIVE) && (ns < 6000)) {
                float st = fabsf(D.p_pi_out - out_prev);
                float jk = fabsf((D.p_pi_out - out_prev) - d_prev);
                s_j[ns] = jk;
                if (jk > 50.0f) { nj50++; }
                s_wd[ns] = fabsf(wd - wd_prev);
                s_st[ns++] = st;
                if (st > r->st_max) { r->st_max = st; }
                if (st > 50.0f) {
                    n50++;
                    if (chg || chg_prev) { n50c++; }
                }
                if (chg) {
                    per_sum += st / (float)abs(D.guide_pos - pos_prev);
                    chg_n++;
                }
            }
            d_prev = (in_curve && in_prev) ? D.p_pi_out - out_prev : 0.0f;
            chg_prev = chg;
            pos_prev = D.guide_pos;
            out_prev = D.p_pi_out;
            wd_prev = wd;
        }
        if (in_curve) {
            float d = hypotf(s_v.x - px, s_v.y - py);
            if (!in_prev) { th0 = s_v.th; d = 0.0f; }
            len += d;
            n += 1.0f;
            sum2 += dev * dev;
            vsum += (s_v.rpm[0] + s_v.rpm[1]) * 0.5f / P.one_meter_rpm;
            if (fabsf(dev) > r->max) { r->max = fabsf(dev); r->max_sg = (dev > 0.0f) ? 1.0f : -1.0f; }
            if (fabsf(center_err()) > r->cerr) { r->cerr = fabsf(center_err()); }
            r->body_r = (fabsf(s_v.th - th0) > 1e-4f) ? len / fabsf(s_v.th - th0) : 0.0f;
        }
        if ((arc <= 0.0f) && (s_v.s > step_at + 300.0f) && (D.control == RUN_DRIVE)) {
            float dd = D.p_pi_out - a_prev;
            if (an > 0) {
                if (fabsf(dd) > r->a_stmax) { r->a_stmax = fabsf(dd); }
                if (fabsf(dd - a_d) > 50.0f) { aj++; }
            }
            an++;
            a_d = dd;
        }
        a_prev = D.p_pi_out;
        if (after) {
            int a = (D.guide_pos >= 0) ? D.guide_pos : -D.guide_pos;
            if (a > r->jump) { r->jump = a; }
            float e = (c->mode == 1) ? dev - 4.0f * GUIDE_UNIT_MM : dev;
            int sg = (e > GUIDE_UNIT_MM) ? 1 : ((e < -GUIDE_UNIT_MM) ? -1 : 0);
            if (fabsf(e) > r->exit_max) { r->exit_max = fabsf(e); }
            if (-r->max_sg * e > r->eover) { r->eover = -r->max_sg * e; }
            if (fabsf(e) > 2.0f * GUIDE_UNIT_MM) { r->settle = s_v.s - ((arc > 0.0f) ? LEAD_MM + arc : step_at); }
            if ((sg != 0) && (side != 0) && (sg != side)) { r->exit_flip++; }
            if (sg != 0) { side = sg; }
            if (s_v.s > end - 1000.0f) {
                if (e < tail_lo) { tail_lo = e; }
                if (e > tail_hi) { tail_hi = e; }
                st_sum += e;
                st_sum2 += e * e;
                st_n += 1.0f;
            }
            if ((c->mode == 1) && (e > r->over * GUIDE_UNIT_MM)) { r->over = e / GUIDE_UNIT_MM; }
        }
        if (((c->mode == 2) || (c->mode == 4)) && jog && (-dev > r->over * GUIDE_UNIT_MM)) { r->over = -dev / GUIDE_UNIT_MM; }   /* 반대쪽으로 넘어간 양 */
        if (s_trace && (s_v.s > LEAD_MM - 600.0f) && (s_v.s < LEAD_MM + arc + 1200.0f)) {
printf("T %6.0f %d dev %6.1f pos %3d out %6.0f v %5d rpm %5.0f %5.0f\n", s_v.s - LEAD_MM, in_curve, dev, D.guide_pos, D.p_pi_out, D.v_cmd, s_v.rpm[0], s_v.rpm[1]);
        }
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
    r->tail_rms = (st_n > 0) ? sqrtf(st_sum2 / st_n) : 0.0f;
    if (r->vmin > 1e8f) { r->vmin = 0.0f; }
    r->a_jn = (an > 0) ? (float)aj / ((float)an * 0.01f) : 0.0f;
    r->rec_n = agv_curvelog.g[0].n;
    r->rec = agv_curvelog.g[0].c[0];
    r->st_n50 = (ns > 0) ? (float)n50 / ((float)ns * 0.01f) : 0.0f;
    r->st_chg = (n50 > 0) ? 100.0f * (float)n50c / (float)n50 : 0.0f;
    r->st_per_unit = (chg_n > 0) ? per_sum / (float)chg_n : 0.0f;
    r->st_p95 = p95(s_st, ns);
    r->j_p95 = p95(s_j, ns);
    r->j_n50 = (ns > 0) ? (float)nj50 / ((float)ns * 0.01f) : 0.0f;
    r->st_wd_p95 = p95(s_wd, ns);
}

static void show(const char *name, const case_t *c, const res_t *r)
{
    printf("  %-10s R%4.0f v%2d LOS%2d tau%3.0f LG%3d L%2d | rms %5.1f max %5.1f 차체R %5.0f v %4.1f | 출구 max %5.1f 바뀜 %d 끝폭 %4.1f | 잃음 %d 위험 %d err %d"
           " | 기록 n%d rms %5.1f max %5.1f 차체R %d 테이프R %d len %d v %4.1f 넘침 %4.1f\n",
           name, c->r_mm, c->v_mpm, c->los, c->tau * 1000.0f, c->lg, c->leds, r->rms, r->max, r->body_r, r->v_avg,
           r->exit_max, r->exit_flip, r->tail_pp, r->lost, r->risk, r->err,
           r->rec_n, r->rec.dev_rms_x10 / 10.0f, r->rec.dev_max_x10 / 10.0f, r->rec.r_body_mm, r->rec.r_tape_mm, r->rec.len_mm,
           r->rec.v_x10 / 10.0f, r->rec.exit_over_x10 / 10.0f);
    printf("  %-10s 계단: 최대 %4.0f 상위5%% %4.0f 50넘음 %4.1f/s 칸변화대응 %3.0f%% 칸당 %4.0f rpm/단위 바퀴차 상위5%% %4.0f"
           " | 변화의 변화 상위5%% %4.0f 50넘음 %4.1f/s | 출구 넘침 %4.1f 정착 %4.0f mm\n",
           "", r->st_max, r->st_p95, r->st_n50, r->st_chg, r->st_per_unit, r->st_wd_p95, r->j_p95, r->j_n50, r->eover, r->settle);
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

/* 교차 테이프 찾기: 순항(20 m/min) 중 교차 테이프 0.6 m 앞(센서 기준, 찾기 한계 2 × SL = 1 m)에서 찾기를 건다(kind 0 정렬 회전, 1 테이프 정지).
 * 결과: 회전 중심의 정지 위치(교차 테이프 기준, + 지나침), 정지까지 중심 최대 위치, 찾기 시간, 찾은 때 속도, 회전 시작 방향 */
typedef struct { float stop_x, peak_x, t_seek, v_find, v_cmd, head_mrad, lat; int err, found; } tape_res_t;

static void tape_sim(int kind, float gd, int dcc, float tau, float w, tape_res_t *r)
{
    static const case_t c0 = { 32, 0.0f, 0.0f, 1, 25, 25, 30, 0.1f, 0, 0 };
    case_t c = c0;
    int t, armed = 0, found = 0, still = 0;
    float t0 = 0.0f;

    c.tau = tau;
    memset(r, 0, sizeof(*r));
    memset(&s_v, 0, sizeof(s_v));
    s_c = &c;
    s_gd = gd;
    s_cross_x = 4000.0f;
    s_cross_w = w;
    s_v.s = GD_MM;
    host_reset();
    memset(&D, 0, sizeof(D));
    memset(seq_request_flag, 0, sizeof(seq_request_flag));   /* 앞 판의 테이프 정지 시퀀스 요청이 남지 않게 */
    sys_boot();
    host.adc = 300u << 4;
    for (t = 0; t < 60; t++) { scan(); }
    assert(sys_power_on());
    assert(param_set_ram(P_SPEED_MIN, 4) == 0);               /* SPN 은 다시 계산할 때 들어간다(WW 가 부른다) */
    assert(param_set_ram(P_GEAR_RATIO, 60) == 0);            /* 곡선 모의와 같은 94.5 rpm 당 1 m/min. SPN 4 = 378 rpm(정지 바닥 350 위) */
    assert(param_set_ram(P_WHEEL_BASE_MM, (int16_t)(WB_MM / 10.0f)) == 0);
    assert(param_set_ram(P_GUIDE_DIST_MM, (int16_t)gd) == 0);
    assert(param_set_ram(P_SPEED_HIGH, 30) == 0);
    assert(param_set_ram(P_SPEED_LOW, 25) == 0);
    assert(param_set_ram(P_SPEED_MIN, 4) == 0);
    assert(param_set_ram(P_DCC, (int16_t)dcc) == 0);
    assert(param_set_ram(P_TURN_FORBACK, 10) == 0);
    tune_turn_scale_set((int16_t)(TURN_SCALE * 1000.0f + 0.5f));
    host.bus_in[BUS_IN_A] &= (uint16_t)~RAW_START;
    scan();
    host.bus_in[BUS_IN_A] |= RAW_START;
    scan();
    for (t = 0; t < 4000; t++) {
        float u = 0.0f, slant = 1.0f;
        int seen = cross(&u, &slant);
        sensor_reply(u, slant, seen);
        plant();
        scan();
        if ((t == 50) && D.flag_run) { tagact_speed(20); }
        if (!armed && (s_v.x + GD_MM >= s_cross_x - 600.0f)) {
            armed = 1;
            t0 = (float)t;
            if (kind == 0) { D.tape_turn_dir = TURN_CW; D.tape_turn_deg = 90; D.tape_turn_next = 0; }
            else           { D.tape_stop_id = 1; }
        }
        if (armed && !found && !D.system_err && ((kind == 0) ? (D.tape_align != 0) : (D.tape_stop_id == 0))) {
            found = 1;
            r->t_seek = ((float)t - t0) * 0.01f;
            r->v_find = (s_v.rpm[0] + s_v.rpm[1]) * 0.5f / P.one_meter_rpm;
            r->v_cmd = (float)D.speed_cmd / P.one_meter_rpm;
        }
        if (found) {
            if (s_v.x - s_cross_x > r->peak_x - 0.0f || r->peak_x == 0.0f) { if (s_v.x - s_cross_x > r->peak_x) r->peak_x = s_v.x - s_cross_x; }
            if (D.flag_turn || ((s_v.rpm[0] < 1.0f) && (s_v.rpm[1] < 1.0f) && (++still > 50))) { break; }
            r->stop_x = s_v.x - s_cross_x;
            r->head_mrad = s_v.th * 1000.0f;
            r->lat = s_v.y;
        }
        if (armed && !found && (D.system_err || D.stop_err)) { break; }
    }
    if (r->peak_x < r->stop_x) { r->peak_x = r->stop_x; }
    r->found = found;
    r->err = D.system_err ? D.system_err : D.stop_err;
    s_cross_w = 0.0f;
    s_gd = 250.0f;
}

/* 정렬 회전 모의: 교차 테이프(90도 선)를 찾아 회전 중심을 그 위에 세우고 CW 90도 정렬 회전.
 * 슬립 = 실제 회전 / 엔코더 회전의 역수(s_m.wbx). 결과: 각-속도 곡선 v² 직선 적합도(R²), 중앙 도달 지령 rpm, 최종 각 오차,
 * 보정 횟수, 회전 시간, 감속 중 재가속 횟수 */
typedef struct { float r2, v_center, err_deg, t_turn, a_fit, line_deg, ctr_mm; int fix, reacc, err, done; } turn_res_t;

static void turn_sim(float gd, int leds, float tau, float slip, turn_res_t *r)
{
    static const case_t c0 = { 32, 0.0f, 0.0f, 1, 20, 20, 30, 0.1f, 0, 0 };
    case_t c = c0;
    int t, armed = 0, phase = 0, t0 = 0;
    float sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0, n = 0, prev_v = 1e9f, vpeak_dec = 0.0f;

    c.tau = tau;
    c.leds = leds;
    memset(r, 0, sizeof(*r));
    memset(&s_v, 0, sizeof(s_v));
    s_c = &c;
    s_gd = gd;
    s_x30 = 4000.0f; s_xa = 90.0f; s_x30w = 20.0f;
    s_v.s = GD_MM;
    host_reset();
    memset(&D, 0, sizeof(D));
    memset(seq_request_flag, 0, sizeof(seq_request_flag));
    sys_boot();
    host.adc = 300u << 4;
    for (t = 0; t < 60; t++) { scan(); }
    assert(sys_power_on());
    assert(param_set_ram(P_SPEED_MIN, 4) == 0);
    assert(param_set_ram(P_GEAR_RATIO, 60) == 0);
    assert(param_set_ram(P_WHEEL_BASE_MM, (int16_t)(WB_MM / 10.0f)) == 0);
    assert(param_set_ram(P_GUIDE_DIST_MM, (int16_t)gd) == 0);
    assert(param_set_ram(P_SPEED_HIGH, 30) == 0);
    assert(param_set_ram(P_SPEED_LOW, 25) == 0);
    assert(param_set_ram(P_TURN_FORBACK, 10) == 0);
    assert(param_set_ram(P_STOP_LEN_MM, 50) == 0);
    tune_turn_scale_set((int16_t)(TURN_SCALE * 1000.0f + 0.5f));
    host.bus_in[BUS_IN_A] &= (uint16_t)~RAW_START;
    scan();
    host.bus_in[BUS_IN_A] |= RAW_START;
    scan();
    for (t = 0; t < 8000; t++) {
        float u = 0.0f, slant = 1.0f;
        int seen = cross(&u, &slant);
        s_m.wbx = (D.control == RUN_TURN) ? slip : 1.0f;
        sensor_reply(u, slant, seen);
        plant();
        scan();
        if ((t == 50) && D.flag_run) { tagact_speed(15); }
        if (!armed && (s_v.x + GD_MM >= s_x30 - 600.0f)) {
            armed = 1;
            D.tape_turn_dir = TURN_CW; D.tape_turn_deg = 90; D.tape_turn_next = 0;
        }
        if ((phase == 0) && D.flag_turn && D.tape_align) { phase = 1; t0 = t; }
        if (phase == 1) {
            float rem = (90.0f + s_v.th * 180.0f / PI_F);       /* 남은 실제 각(도), CW 라 th 가 -90 으로 간다 */
            float v = (float)D.vin_out;
            if ((D.tape_step <= 2) && (rem < 25.0f) && (rem > 0.3f) && (v > 0.0f)) {
                sx += rem; sy += v * v; sxx += rem * rem; sxy += rem * v * v; syy += v * v * v * v; n += 1.0f;
                if (v > prev_v + 1.0f) { r->reacc++; }
                prev_v = v;
            }
            if ((D.tape_step == TAPE_STEP_SETTLE) && (r->v_center == 0.0f)) { r->v_center = (vpeak_dec > 0.0f) ? vpeak_dec : 0.01f; }
            if (D.tape_step <= 2) { vpeak_dec = (float)D.vin_out; }     /* 중앙에서 서기 직전 지령 */
            if (!D.tape_align && !D.flag_turn) { phase = 2; r->t_turn = (float)(t - t0) * 0.01f; break; }
        }
        if (D.system_err || D.stop_err) { break; }
    }
    (void)vpeak_dec;
    if (n > 3.0f) {
        float mx = sx / n, my = sy / n, cxy = sxy / n - mx * my, cxx = sxx / n - mx * mx, cyy = syy / n - my * my;
        r->r2 = (cxx > 0.0f && cyy > 0.0f) ? (cxy * cxy) / (cxx * cyy) : 0.0f;
        r->a_fit = (cxx > 0.0f) ? cxy / cxx : 0.0f;       /* v² 기울기(rpm²/도) */
    }
    r->err_deg = 90.0f + s_v.th * 180.0f / PI_F;
    {   /* 센서 점의 선 기준 옆 거리 → 선 기준 각 오차, 회전 중심의 선 기준 거리 */
        float sxp = s_v.x + GD_MM * cosf(s_v.th);
        r->line_deg = atanf((sxp - s_x30) / GD_MM) * 180.0f / PI_F;
        r->ctr_mm = s_v.x - s_x30;
    }
    r->fix = D.tape_fix_cnt;
    r->err = D.system_err ? D.system_err : D.stop_err;
    r->done = (phase == 2);
    s_x30w = 0.0f; s_xa = 30.0f; s_gd = 250.0f; s_m.wbx = 1.0f;
}

static void turn_table(void)
{
    static const float sl[3] = { 0.8f, 1.0f, 1.25f };
    int g, l, t, k;
    for (g = 0; g < 2; g++) {
        for (l = 0; l < 2; l++) {
            for (t = 0; t < 2; t++) {
                for (k = 0; k < 3; k++) {
                    turn_res_t r;
                    turn_sim(g ? 500.0f : 250.0f, l ? 16 : 32, t ? 0.15f : 0.1f, sl[k], &r);
                    printf("TURN GD%d/L%d/t%d/s%.2f | done %d err %d r2 %.3f vcenter %.0f errdeg %.2f fix %d time %.2f reacc %d linedeg %.2f ctr %.1f\n",
                           g ? 500 : 250, l ? 16 : 32, t ? 150 : 100, sl[k], r.done, r.err, r.r2, r.v_center, r.err_deg, r.fix, r.t_turn, r.reacc, r.line_deg, r.ctr_mm);
                }
            }
        }
    }
}

static void tape_table(void)
{
    static const int   dcc[2] = { 20, 50 };
    static const float gd[2] = { 250.0f, 400.0f };
    static const float tau[2] = { 0.05f, 0.15f };
    int k, i, j, l, w;
    tape_res_t r;
    for (k = 0; k < 2; k++) {
        for (j = 0; j < 2; j++) {
            for (i = 0; i < 2; i++) {
                for (l = 0; l < 2; l++) {
                    for (w = 0; w < 2; w++) {
                        tape_sim(k, gd[j], dcc[i], tau[l], w ? 50.0f : 10.0f, &r);
                        printf("  %s GD %3.0f DCC %2d 지연 %3.0f 폭 %2d | 찾음 %d 찾은 속도 %5.1f(지령 %4.1f) m/min 찾기 %5.2f s | 정지 위치 %7.1f mm 최대 %7.1f 옆 %5.1f 방향 %5.1f mrad | err %d\n",
                               k ? "정지" : "회전", gd[j], dcc[i], tau[l] * 1000.0f, w ? 50 : 10, r.found, r.v_find, r.v_cmd, r.t_seek,
                               r.stop_x, r.peak_x, r.lat, r.head_mrad, r.err);
                    }
                }
            }
        }
    }
}

/* 조향 비교: 새 조향(LG 0), 예전(LG 256), 예전 전부(LG 368). 기준선(v0.34.0 조향)은 따로 만든 실행 파일로 같은 표를 낸다 */
static const int   s_lg3[3] = { 0, QUIRK_STEER_STEP, QUIRK_STEER_LEGACY };
static const char *s_lgn[3] = { "새", "예전", "예전전체" };

/* 곡선 6개: 실차 A~D, 완만한 1.5 m·4 m */
static const struct { const char *name; float r, deg; int dir; } s_cur6[6] = {
    { "A", 650.0f, 45.0f, -1 }, { "B", 551.0f, 45.0f, 1 }, { "C", 909.0f, 90.0f, 1 }, { "D", 961.0f, 90.0f, -1 },
    { "R1500", 1500.0f, 90.0f, 1 }, { "R4000", 4000.0f, 45.0f, -1 },
};

typedef struct { float max, rms, cerr, p95, smax, jn, eover, settle, tail; int lost, flip, err, n; } agg_t;

static void agg_add(agg_t *a, const res_t *r)
{
    if (r->max > a->max)       { a->max = r->max; }
    if (r->rms > a->rms)       { a->rms = r->rms; }
    if (r->cerr > a->cerr)     { a->cerr = r->cerr; }
    if (r->st_p95 > a->p95)    { a->p95 = r->st_p95; }
    if (r->st_max > a->smax)   { a->smax = r->st_max; }
    if (r->j_n50 > a->jn)      { a->jn = r->j_n50; }
    if (r->eover > a->eover)   { a->eover = r->eover; }
    if (r->settle > a->settle) { a->settle = r->settle; }
    if (r->tail_pp > a->tail)  { a->tail = r->tail_pp; }
    if (r->exit_flip > a->flip) { a->flip = r->exit_flip; }
    a->lost += r->lost;
    a->err += (r->err != 0);
    a->n++;
}

static void agg_show(const char *name, const agg_t *a)
{
    printf("  %-24s n%3d | 최대 %5.1f rms %5.1f 중심 %5.1f | 계단 상위5%% %4.0f 최대 %4.0f 변화의변화 %4.1f/s | 넘침 %4.1f 정착 %5.0f 바뀜 %d 끝폭 %4.1f | 잃음 %d err %d\n",
           name, a->n, a->max, a->rms, a->cerr, a->p95, a->smax, a->jn, a->eover, a->settle, a->flip, a->tail, a->lost, a->err);
}

static void line(const char *name, const case_t *c, const res_t *r)
{
    printf("    %-5s L%2d v%2d tau%3.0f %-4s | 최대 %5.1f rms %5.1f 중심 %5.1f | 상위5%% %4.0f 최대 %4.0f jn %4.1f | 넘침 %4.1f 정착 %5.0f 바뀜 %d 끝폭 %4.1f | 잃음 %d 위험 %d err %d\n",
           name, c->leds, c->v_mpm, c->tau * 1000.0f, s_lgn[(c->lg == 0) ? 0 : ((c->lg == QUIRK_STEER_STEP) ? 1 : 2)],
           r->max, r->rms, r->cerr, r->st_p95, r->st_max, r->j_n50, r->eover, r->settle, r->exit_flip, r->tail_pp, r->lost, r->risk, r->err);
}

/* 비교 표: 곡선 6 × 진입 31·25·15 × 지연 50·100·150 ms × wide·일반, 직선, 강건성 */
static void compare(int verbose)
{
    static const int   vv[3] = { 31, 25, 15 };
    static const float tt[3] = { 0.05f, 0.1f, 0.15f };
    agg_t a[3][2], st[3][2], rb[3];
    int g, l, v, t, i, k;
    res_t r;
    char nm[64];

    memset(a, 0, sizeof(a));
    memset(st, 0, sizeof(st));
    memset(rb, 0, sizeof(rb));
    printf("[곡선]\n");
    for (l = 0; l < 2; l++) {
        for (v = 0; v < 3; v++) {
            for (t = 0; t < 3; t++) {
                for (i = 0; i < 6; i++) {
                    for (g = 0; g < 3; g++) {
                        if (!(s_gmask & (1 << g))) { continue; }
                        case_t c = { l ? 16 : 32, s_cur6[i].r, s_cur6[i].deg, s_cur6[i].dir, vv[v], vv[v], 31, tt[t], s_lg3[g], 0 };
                        sim(&c, &r);
                        agg_add(&a[g][l], &r);
                        if (verbose || ((v == 0) && (t == 1))) { line(s_cur6[i].name, &c, &r); }
                    }
                }
            }
        }
    }
    printf("[직선] 1 옆 4단위 지령, 2 5단위 꺾임 복귀, 4 출발 편차 5단위\n");
    for (l = 0; l < 2; l++) {
        for (v = 0; v < 3; v++) {
            for (k = 1; k <= 4; k++) {
                if (k == 3) { continue; }
                for (g = 0; g < 3; g++) {
                    if (!(s_gmask & (1 << g))) { continue; }
                    case_t c = { l ? 16 : 32, 0.0f, (k == 1) ? 4.0f : 5.0f, 1, vv[v], vv[v], 31, 0.1f, s_lg3[g], k };
                    sim(&c, &r);
                    st[g][l].max = (r.over > st[g][l].max) ? r.over : st[g][l].max;
                    st[g][l].settle = (r.settle > st[g][l].settle) ? r.settle : st[g][l].settle;
                    st[g][l].tail = (r.tail_rms > st[g][l].tail) ? r.tail_rms : st[g][l].tail;
                    st[g][l].lost += r.lost;
                    st[g][l].err += (r.err != 0) || (r.risk != 0);
                    st[g][l].flip = (r.exit_flip > st[g][l].flip) ? r.exit_flip : st[g][l].flip;
                    printf("    직선%d L%2d v%2d %-4s | 넘침 %4.2f 단위 정착 %5.0f mm 끝 평균 %5.2f 단위 끝 RMS %4.2f mm 끝폭 %4.1f 바뀜 %d | 잃음 %d 위험 %d err %d\n",
                           k, c.leds, c.v_mpm, s_lgn[g], r.over, r.settle, r.steady, r.tail_rms, r.tail_pp, r.exit_flip, r.lost, r.risk, r.err);
                }
            }
        }
    }
    printf("[강건성] 곡선 A~D, wide·일반, 31 m/min, 지연 100·150 ms. rms 칸 = 직선 꺾임 뒤 정상 편차 mm\n");
    for (k = 0; k < 7; k++) {
        static const char *kn[7] = { "휠간격-5%", "휠간격+5%", "GD-10%", "GD+10%", "왼쪽 펄스 2% 손실", "오른쪽 바퀴 +1.5%", "오른쪽 바퀴 -1.5%" };
        agg_t one[3];
        memset(one, 0, sizeof(one));
        for (g = 0; g < 3; g++) {
            if (!(s_gmask & (1 << g))) { continue; }
            s_m.wbx = (k == 0) ? 0.95f : ((k == 1) ? 1.05f : 1.0f);
            s_m.gdx = (k == 2) ? 0.9f : ((k == 3) ? 1.1f : 1.0f);
            s_m.slip = (k == 4) ? 2.0f : 0.0f;
            s_m.wr = (k == 5) ? 1.015f : ((k == 6) ? 0.985f : 1.0f);
            for (l = 0; l < 2; l++) {
                for (t = 1; t < 3; t++) {
                    for (i = 0; i < 4; i++) {
                        case_t c = { l ? 16 : 32, s_cur6[i].r, s_cur6[i].deg, s_cur6[i].dir, 31, 31, 31, tt[t], s_lg3[g], 0 };
                        sim(&c, &r);
                        agg_add(&one[g], &r);
                        agg_add(&rb[g], &r);
                        if (verbose) { line(s_cur6[i].name, &c, &r); }
                    }
                }
                {   /* 직선 꺾임 복귀: 정상 편차·발진 */
                    case_t c = { l ? 16 : 32, 0.0f, 5.0f, 1, 31, 31, 31, 0.15f, s_lg3[g], 2 };
                    sim(&c, &r);
                    one[g].rms = 0.0f;
                    if (fabsf(r.steady) * GUIDE_UNIT_MM > one[g].rms) { one[g].rms = fabsf(r.steady) * GUIDE_UNIT_MM; }
                    if (r.tail_pp > one[g].tail) { one[g].tail = r.tail_pp; }
                    one[g].lost += r.lost;
                }
            }
            s_m.wbx = s_m.gdx = s_m.wr = 1.0f;
            s_m.slip = 0.0f;
            snprintf(nm, sizeof(nm), "%s %s", kn[k], s_lgn[g]);
            agg_show(nm, &one[g]);
        }
    }
    printf("[요약 곡선] 끝폭 = 출구 뒤 끝 1 m 편차 폭\n");
    for (l = 0; l < 2; l++) {
        for (g = 0; g < 3; g++) {
            snprintf(nm, sizeof(nm), "%s %s", l ? "일반" : "wide", s_lgn[g]);
            agg_show(nm, &a[g][l]);
        }
    }
    printf("[요약 직선] 넘침(단위)·정착(mm)·끝 RMS(mm) 최대\n");
    for (l = 0; l < 2; l++) {
        for (g = 0; g < 3; g++) {
            printf("  %-4s %-4s 넘침 %4.2f 정착 %5.0f 끝 RMS %4.2f 바뀜 %d 잃음 %d 오류·위험 %d\n",
                   l ? "일반" : "wide", s_lgn[g], st[g][l].max, st[g][l].settle, st[g][l].tail, st[g][l].flip, st[g][l].lost, st[g][l].err);
        }
    }
    printf("[요약 강건성]\n");
    for (g = 0; g < 3; g++) { agg_show(s_lgn[g], &rb[g]); }
}

/* 합격 표: 한 줄에 한 조건. 같은 표를 기준선(v0.34.0 조향) 실행 파일과 견준다. LG 0 만 돈다 */
static void acc_line(const char *grp, const char *name, const case_t *c, const res_t *r)
{
    printf("K %s/%s/L%d/v%d/t%d/m%d/d%.1f | max %.1f rms %.1f jn %.2f p95 %.0f smax %.0f eover %.1f settle %.0f tail %.1f lost %d flip %d err %d"
           " astmax %.0f ajn %.2f over %.2f trms %.2f risk %d vavg %.1f\n",
           grp, name, c->leds, c->v_mpm, (int)(c->tau * 1000.0f + 0.5f), c->mode, c->deg, r->max, r->rms, r->j_n50, r->st_p95, r->st_max,
           r->eover, r->settle, r->tail_pp, r->lost, r->exit_flip, r->err, r->a_stmax, r->a_jn, r->over, r->tail_rms, r->risk, r->v_avg);
}

static void acc_case(const char *grp, const char *name, case_t c)
{
    res_t r;
    sim(&c, &r);
    acc_line(grp, name, &c, &r);
}

static void acceptance(void)
{
    static const int   vv[3] = { 31, 25, 15 };
    static const float tt[3] = { 0.05f, 0.1f, 0.15f };
    static const char *kn[9] = { "WW-20", "WW+20", "GD-20", "GD+20", "PL-20", "PL+20", "R+1.5", "R-1.5", "SLIP2" };
    int l, v, t, i, k;
    for (l = 0; l < 2; l++) {
        int leds = l ? 16 : 32;
        for (v = 0; v < 3; v++) {
            for (t = 0; t < 3; t++) {
                for (i = 0; i < 6; i++) {
                    case_t c = { leds, s_cur6[i].r, s_cur6[i].deg, s_cur6[i].dir, vv[v], vv[v], 31, tt[t], 0, 0 };
                    acc_case("곡선", s_cur6[i].name, c);
                }
            }
            for (k = 1; k <= 5; k++) {      /* 직선: 1 옆 4단위 지령, 2 5단위 꺾임, 4 출발 5단위, 5 = 반 단위 꺾임(1단위 떨림) */
                case_t c = { leds, 0.0f, (k == 1) ? 4.0f : ((k == 5) ? 1.0f : 5.0f), 1, vv[v], vv[v], 31, 0.1f, 0, (k == 5) ? 2 : k };
                if (k == 3) { continue; }
                acc_case("직선", (k == 5) ? "떨림" : ((k == 1) ? "계단" : ((k == 2) ? "꺾임" : "출발")), c);
            }
        }
        for (k = 0; k < 9; k++) {
            s_m.wbx = (k == 0) ? 0.8f : ((k == 1) ? 1.2f : 1.0f);
            s_m.gdx = (k == 2) ? 0.8f : ((k == 3) ? 1.2f : 1.0f);
            s_m.pl = (k == 4) ? 0.8f : ((k == 5) ? 1.2f : 1.0f);
            s_m.wr = (k == 6) ? 1.015f : ((k == 7) ? 0.985f : 1.0f);
            s_m.slip = (k == 8) ? 2.0f : 0.0f;
            for (t = 1; t < 3; t++) {
                for (i = 0; i < 4; i++) {
                    char g[24];
                    case_t c = { leds, s_cur6[i].r, s_cur6[i].deg, s_cur6[i].dir, 31, 31, 31, tt[t], 0, 0 };
                    snprintf(g, sizeof(g), "강건 %s", kn[k]);
                    acc_case(g, s_cur6[i].name, c);
                }
            }
            {
                char g[24];
                case_t c = { leds, 0.0f, 5.0f, 1, 31, 31, 31, 0.15f, 0, 2 };
                snprintf(g, sizeof(g), "강건 %s", kn[k]);
                acc_case(g, "꺾임", c);
            }
            s_m.wbx = s_m.gdx = s_m.pl = s_m.wr = 1.0f;
            s_m.slip = 0.0f;
        }
        for (v = 0; v < 2; v++) {           /* 잡음: 끊김 + 점(10·25 mm), 30도 교차선(10·25 mm) */
            for (k = 0; k < 2; k++) {
                case_t c3 = { leds, 0.0f, k ? 25.0f : 10.0f, 1, v ? 15 : 31, v ? 15 : 31, 31, 0.1f, 0, 3 };
                case_t c6 = { leds, 0.0f, k ? 25.0f : 10.0f, 1, v ? 15 : 31, v ? 15 : 31, 31, 0.1f, 0, 6 };
                acc_case("잡음", "점", c3);
                acc_case("잡음", "교차30", c6);
            }
        }
        for (t = 1; t < 3; t++) {           /* 태그 없는 최대 곡률: 최고 속도(DRS·LOS 37 m/min)로 진입 */
            static const float rr[6] = { 500.0f, 800.0f, 1000.0f, 1500.0f, 800.0f, 500.0f };
            for (i = 0; i < 6; i++) {
                char nm[16];
                case_t c = { leds, rr[i], (i < 4) ? 90.0f : 60.0f, (i & 1) ? 1 : -1, 37, 37, 37, tt[t], 0, (i < 4) ? 0 : 5 };
                snprintf(nm, sizeof(nm), "%s%.0f", (i < 4) ? "R" : "S", rr[i]);
                acc_case("태그없음", nm, c);
            }
        }
    }
}

static void acc2_line(const char *grp, const char *name, const case_t *c, const res_t *r)
{
    printf("K %s/%s/GD%.0f/L%d/v%d/t%d/m%d/d%.0f | max %.1f cmax %.1f rms %.1f jn %.2f eover %.1f settle %.0f tail %.1f lost %d flip %d err %d"
           " astmax %.0f ajn %.2f over %.2f trms %.2f risk %d pass %.2f vmin %.1f izero %.2f vavg %.1f\n",
           grp, name, s_gd, c->leds, c->v_mpm, (int)(c->tau * 1000.0f + 0.5f), c->mode, c->deg, r->max, r->cmax, r->rms, r->j_n50,
           r->eover, r->settle, r->tail_pp, r->lost, r->exit_flip, r->err, r->a_stmax, r->a_jn, r->over, r->tail_rms, r->risk,
           r->pass_s, r->vmin, r->izero_s, r->v_avg);
}

static void acc2_case(const char *grp, const char *name, case_t c)
{
    res_t r;
    sim(&c, &r);
    acc2_line(grp, name, &c, &r);
}

/* 경로 기억 비교 표: GD 250·500 × wide·일반, 태그 없음(속도는 처음부터 그대로) */
static void acceptance2(void)
{
    static const float rr[7] = { 400.0f, 500.0f, 600.0f, 800.0f, 1000.0f, 1500.0f, 2000.0f };
    static const float aa[3] = { 45.0f, 90.0f, 180.0f };
    static const int   vv[4] = { 15, 25, 31, 37 };
    static const char *kn[9] = { "WW-20", "WW+20", "GD-20", "GD+20", "PL-20", "PL+20", "R+1.5", "R-1.5", "SLIP2" };
    int g, l, i, a, v, t, k;
    for (g = 0; g < 2; g++) {
        for (l = 0; l < 2; l++) {
            int leds = l ? 16 : 32;
            float gd = g ? 500.0f : 250.0f;
            char nm[24];
            for (i = 0; i < 9; i++) {
                for (a = 0; a < 3; a++) {
                    for (v = 0; v < 4; v++) {
                        for (t = 0; t < 2; t++) {
                            int s_shape = (i >= 7);
                            float r = s_shape ? ((i == 7) ? 600.0f : 1000.0f) : rr[i];
                            case_t c = { leds, r, aa[a], (a & 1) ? 1 : -1, vv[v], vv[v], 37, t ? 0.15f : 0.1f, 0, s_shape ? 5 : 0 };
                            s_gd = gd;
                            snprintf(nm, sizeof(nm), "%s%.0f", s_shape ? "S" : "R", r);
                            acc2_case("곡선", nm, c);
                        }
                    }
                }
            }
            for (v = 0; v < 4; v++) {
                for (k = 1; k <= 5; k++) {
                    case_t c = { leds, 0.0f, (k == 1) ? 4.0f : ((k == 5) ? 1.0f : 5.0f), 1, vv[v], vv[v], 37, 0.1f, 0, (k == 5) ? 2 : k };
                    if (k == 3) { continue; }
                    s_gd = gd;
                    acc2_case("직선", (k == 5) ? "떨림" : ((k == 1) ? "계단" : ((k == 2) ? "꺾임" : "출발")), c);
                }
            }
            for (k = 0; k < 9; k++) {
                s_m.wbx = (k == 0) ? 0.8f : ((k == 1) ? 1.2f : 1.0f);
                s_m.gdx = (k == 2) ? 0.8f : ((k == 3) ? 1.2f : 1.0f);
                s_m.pl = (k == 4) ? 0.8f : ((k == 5) ? 1.2f : 1.0f);
                s_m.wr = (k == 6) ? 1.015f : ((k == 7) ? 0.985f : 1.0f);
                s_m.slip = (k == 8) ? 2.0f : 0.0f;
                for (i = 0; i < 2; i++) {
                    char gr[24];
                    case_t c = { leds, i ? 1000.0f : 600.0f, 90.0f, i ? 1 : -1, 31, 31, 37, 0.15f, 0, 0 };
                    s_gd = gd;
                    snprintf(gr, sizeof(gr), "강건 %s", kn[k]);
                    acc2_case(gr, i ? "R1000" : "R600", c);
                }
                {
                    case_t c = { leds, 0.0f, 5.0f, 1, 31, 31, 37, 0.15f, 0, 2 };
                    char gr[24];
                    s_gd = gd;
                    snprintf(gr, sizeof(gr), "강건 %s", kn[k]);
                    acc2_case(gr, "꺾임", c);
                }
                s_m.wbx = s_m.gdx = s_m.pl = s_m.wr = 1.0f;
                s_m.slip = 0.0f;
            }
            for (v = 0; v < 2; v++) {
                for (k = 0; k < 2; k++) {
                    case_t c3 = { leds, 0.0f, k ? 25.0f : 10.0f, 1, v ? 15 : 31, v ? 15 : 31, 37, 0.1f, 0, 3 };
                    case_t c6 = { leds, 0.0f, k ? 25.0f : 10.0f, 1, v ? 15 : 31, v ? 15 : 31, 37, 0.1f, 0, 6 };
                    s_gd = gd;
                    acc2_case("잡음", "끊김점", c3);
                    acc2_case("잡음", "교차30", c6);
                }
            }
        }
    }
    s_gd = 250.0f;
}

static void on_abort(int sig) { (void)sig; _exit(3); }

int main(int argc, char **argv)
{
    int i, l, g;
    res_t r;

    signal(SIGABRT, on_abort);
    setvbuf(stdout, 0, _IONBF, 0);
    if ((argc > 1) && (strcmp(argv[1], "sweep") == 0)) { sweep(); return 0; }
    if ((argc > 1) && (strcmp(argv[1], "acc") == 0))   { acceptance(); return 0; }
    if ((argc > 1) && (strcmp(argv[1], "acc2") == 0))  { acceptance2(); return 0; }
    if ((argc > 1) && (strcmp(argv[1], "tape") == 0))  { tape_table(); return 0; }
    if ((argc > 1) && (strcmp(argv[1], "turn") == 0))  { turn_table(); return 0; }
    if ((argc > 1) && (strcmp(argv[1], "cmp") == 0))   { if (argc > 2) { s_gmask = atoi(argv[2]); } compare(argc > 3); return 0; }
    if ((argc > 6) && (strcmp(argv[1], "one") == 0)) {  /* one LG 곡선(0~5, 직선은 6 + 모드) v tau_ms leds */
        int i = atoi(argv[3]);
        case_t c = { atoi(argv[6]), 0.0f, 5.0f, 1, atoi(argv[4]), atoi(argv[4]), 31, (float)atoi(argv[5]) * 0.001f, atoi(argv[2]), 0 };
        if (i < 6) { c.r_mm = s_cur6[i].r; c.deg = s_cur6[i].deg; c.dir = s_cur6[i].dir; }
        else       { c.mode = i - 6; c.deg = (c.mode == 1) ? 4.0f : 5.0f; }
        s_trace = (argc > 7);
        sim(&c, &r);
        show((i < 6) ? s_cur6[i].name : "직선", &c, &r);
        printf("  중심 %.1f 넘침(직선) %.2f 정착 %.0f 끝 RMS %.2f 끝 평균 %.2f\n", r.cerr, r.over, r.settle, r.tail_rms, r.steady);
        return 0;
    }

    for (g = 0; g < 2; g++) {
        for (l = 32; l >= 16; l -= 16) {
            for (i = 0; i < 4; i++) {
                case_t c = { l, s_curves[i].r, s_curves[i].deg, s_curves[i].dir, 31, 31, 31, 0.1f, g ? QUIRK_STEER_STEP : 0, 0 };
                sim(&c, &r);
                show(s_curves[i].name, &c, &r);
                assert(r.risk > 0);                             /* 편차 10단위를 넘어 위험 감속에 들어간다 */
                if (g == 0) {
                    assert((r.lost == 0) && (r.err == 0));
                    /* v0.33.0 43.6·49.7·37.0·36.1 mm(wide 끝 107 mm 의 절반 아래). 계단 완화 뒤 wide 는 출력 변화를 늘 묶어
                     * +4~14%(v0.33.0 의 +15% 안), 일반 센서는 10단위 위를 묶지 않아 +2~6% */
                    assert(r.max < ((i < 2) ? ((l == 32) ? 57.5f : 53.0f) : 42.0f));
                    assert((r.exit_flip <= 1) && (r.tail_pp < 20.0f));
                    /* 계단 완화: v0.33.0 은 칸이 바뀔 때마다 150~260 rpm 이 뛰었다(상위 5% 167~193, 변화의 변화 50 넘음 11~15/s).
                     * 일반 센서는 위험 감속 편차(10단위) 위를 묶지 않아 그 구간 칸 변화만 남는다 */
                    assert((r.st_p95 <= STEER_SLEW_TEST) && (r.st_n50 < 6.0f) && (r.j_n50 < 9.0f));
                    if (l == 32) { assert((r.st_max <= STEER_SLEW_TEST) && (r.j_n50 < 4.0f)); }   /* 남는 것은 묶인 기울기가 뒤집히는 순간 */
                    /* 곡선 끝 적분 감쇠: v0.33.0 은 넘침 11~19 mm, 2 m 안에 ±2단위로 못 들어왔다 */
                    assert((r.eover < 6.0f) && (r.settle < 800.0f));
                }
                else if (l == 16) { assert(r.lost > 0); }       /* 예전 조향: 일반 센서는 선을 잃는다 */
                else if (i >= 2)  { assert((r.rms > 60.0f) && (r.max > 80.0f)); }   /* 실차 곡선에서 잰 큰 편차(RMS 75~80 mm) */
                else              { assert(r.max > 90.0f); }
            }
        }
    }
    for (i = 0; i < 16; i++) {      /* 속도 루프 지연 150 ms·두 LOS 에서도 선을 잃지 않고 발진하지 않는다. 일반 센서(±55 mm) B 가 가장 빠듯하다 */
        case_t c = { (i < 8) ? 16 : 32, s_curves[i & 3].r, s_curves[i & 3].deg, s_curves[i & 3].dir, 31, (i & 4) ? 25 : 31, 31, 0.15f, 0, 0 };
        sim(&c, &r);
        show(s_curves[i & 3].name, &c, &r);
        assert((r.lost == 0) && (r.err == 0) && (r.exit_flip <= 2) && (r.tail_pp < 20.0f) && (r.max < 70.0f));   /* wide B: 출구 한 번 더 넘어간다(끝폭 7 mm) */
    }
    {   /* 120도(D 반경)와 진입 25 m/min: 같은 상한 */
        case_t c = { 32, 961.0f, 120.0f, -1, 31, 31, 31, 0.1f, 0, 0 };
        sim(&c, &r);
        show("D120", &c, &r);
        assert((r.lost == 0) && (r.max < 42.0f) && (r.st_max <= STEER_SLEW_TEST) && (r.eover < 6.0f) && (r.settle < 800.0f));
        for (i = 0; i < 4; i++) {
            case_t d = { 32, s_curves[i].r, s_curves[i].deg, s_curves[i].dir, 25, 25, 31, 0.1f, 0, 0 };
            sim(&d, &r);
            show(s_curves[i].name, &d, &r);
            assert((r.lost == 0) && (r.max < 45.0f) && (r.st_max <= STEER_SLEW_TEST) && (r.eover < 6.0f));
        }
    }
    for (i = 1; i <= 2; i++) {      /* 직선: 옆 4단위 계단 지령, 5단위 꺾임에서 복귀 */
        case_t c = { 32, 0.0f, 5.0f, 1, 31, 31, 31, 0.1f, 0, i };
        sim(&c, &r);
        printf("  직선 %d: 넘침 %.2f 단위, 끝 평균 %.2f, 바뀜 %d, 끝폭 %.1f mm\n", i, r.over, r.steady, r.exit_flip, r.tail_pp);
        assert((r.lost == 0) && (r.err == 0) && (r.risk == 0));
        assert(r.over < ((i == 1) ? 1.6f : 2.9f));              /* 고치기 전 2.03, 2.80. v0.33.0 1.52, 2.81: 계단 완화 뒤도 같다 */
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
    {   /* 교차 테이프 찾기. 정렬 회전 앞은 계산 속도(SPN 4 대신 8.7~12 m/min)로 찾아 회전 중심이 테이프 ±12 mm 안에 선다
         * (SPN 4: -1~+6 mm, 찾기 6.6~8.0 s → 2.7~3.7 s). 테이프 정지는 SPN 그대로 */
        tape_res_t tr;
        for (i = 0; i < 4; i++) {
            tape_sim(0, (i & 1) ? 400.0f : 250.0f, (i & 2) ? 50 : 20, 0.15f, 10.0f, &tr);
            printf("  교차 테이프 회전 GD %d DCC %d: 찾기 %.1f m/min %.2f s, 정지 위치 %.1f mm\n", (i & 1) ? 400 : 250, (i & 2) ? 50 : 20,
                   tr.v_cmd, tr.t_seek, tr.stop_x);
            assert(tr.found && (tr.err == 0) && (tr.v_cmd > 8.0f) && (tr.v_cmd < 12.1f) && (fabsf(tr.stop_x) < 12.0f) && (tr.t_seek < 4.0f));
        }
        tape_sim(1, 250.0f, 50, 0.15f, 10.0f, &tr);
        assert(tr.found && (tr.err == 0) && (fabsf(tr.v_cmd - 4.0f) < 0.1f) && (tr.stop_x < -240.0f) && (tr.stop_x > -255.0f));
    }
    {   /* 정렬 회전 끝 등감속(v0.35.0): 가운데 도달 지령 ≤ 정지 바닥, 다시 가속 없음, 선 기준 각 오차 1도 안, 보정 많아야 1번.
         * v0.34.0(최저 회전 속도 기어가기)은 가운데 도달 615~784 rpm, 보정 24조건 합 26 */
        turn_res_t tr;
        for (i = 0; i < 4; i++) {
            turn_sim((i & 1) ? 500.0f : 250.0f, (i & 2) ? 16 : 32, 0.15f, (i & 1) ? 0.8f : 1.25f, &tr);
            printf("  정렬 회전 GD %d L%d: 가운데 %.0f rpm, 선 기준 각 %.2f도, 보정 %d, %.2f s\n", (i & 1) ? 500 : 250, (i & 2) ? 16 : 32,
                   tr.v_center, tr.line_deg, tr.fix, tr.t_turn);
            assert(tr.done && (tr.err == 0) && (tr.v_center <= (float)stop_min_rpm()) && (tr.reacc == 0) && (fabsf(tr.line_deg) < 1.0f) && (tr.fix <= 1));
        }
    }
    printf("test_curve ok\n");
    return 0;
}
