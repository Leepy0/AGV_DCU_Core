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
 * 인자 est [자세히 0~2 [스캔 1]] 은 추정 조향(steer_alg_test 1) 대 37차 표, one2 는 한 조건(trace 가능).
 * 스캔 1: 센서가 6.3 ms(실측)마다만 LED 를 새로 읽는다. 응답은 마지막 스캔 때 자세(2 ms 자세 기록에서 가장 가까운 것).
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
#include "agv/gresp.h"
#include "agv/ioport.h"
#include "agv/motor.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/seq.h"
#include "agv/steer.h"
#include "agv/steerlog.h"
#include "agv/sys.h"
#include "agv/tagact.h"
#include "agv/tune.h"
#include "agv/ui.h"
#include "agv/wtest.h"
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

/* 시퀀스 MOVE_MM 모의(move 표): 출력 A 켬 → MOVE_MM(mm) → 출력 B 켬. 거리는 시험이 arg0 를 바꾼다 */
#include "agv/seq_dsl.h"
static seq_step_t s_mv_steps[] = {
    OUT_ON( OUT_EXT_1 )
    MOVE_MM( 100 )
    OUT_ON( OUT_EXT_3 )
    END_STOP
};
static const seq_def_t s_mv_seqs[] = { { 0, SEQ_SITE_BASE, s_mv_steps, sizeof(s_mv_steps) / sizeof(s_mv_steps[0]) } };
static const seq_site_t s_mv_site = { s_mv_seqs, 1, 0, 0, 0, 0, 0, 0, 0 };
void site_register(void) { param_default_type(0); (void)seq_bind(&s_mv_site); }
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
    int   alg;              /* steer_alg_test(AGV_DEV_TESTS): 0 37차, 1 추정 조향 */
    int   sct;              /* speed_curve_test(AGV_DEV_TESTS): 1 반응형 곡선 속도 */
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
    /* 곡선 정상 구간(센서 점이 곡선 시작 0.8 m 뒤 ~ 곡선 끝)의 좌우 바퀴 지령 차 |w_cmd[0] - w_cmd[1]| */
    float wob;              /* 흔들림 % = (최대 - 최소) / (최대 + 최소) × 100. 구간이 없으면 -1 */
    float wflip;            /* 증감 방향 바뀜 횟수/s(EST_FLIP_HYS rpm 넘게 되돌아선 것만) */
    int   vflip;            /* 곡선 앞 0.5 m ~ 뒤 0.5 m 동안 속도 지령(v_cmd) 증감 방향 바뀜 횟수. 감속 뒤 재가속 1번이 정상 */
    int   risk_n;           /* 위험 감속 들어간 횟수 */
    float idrop;            /* 센서 점이 곡선 위인 동안 적분 빠짐 최대 rpm: 그때까지 최대 |적분| - 지금 |적분| (AGV_DEV_TESTS) */
    int   pin;              /* 곡선 진입(센서 점이 곡선 앞 절반) 가이드 |위치| 최대 단위 */
    uint32_t hash;          /* 틱마다 조향 출력(D.p_pi_out) 비트의 FNV 해시. 고치기 전·후 수열 비교용 */
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
static int   s_sct;         /* 1: 모든 판에 speed_curve_test 1 */
static int   s_tst, s_kp;   /* kp 표: TST 값, 고정 K_P(TST 0 일 때). 0 = 끔 */
static int   s_ki;          /* tst 표: K_I 파라미터(TST 뒤에 넣는다). 0 = 그대로 */

static uint32_t fnv(uint32_t h, float v)
{
    uint32_t b;
    int i;
    memcpy(&b, &v, sizeof(b));
    for (i = 0; i < 4; i++) { h = (h ^ ((b >> (i * 8)) & 0xFFu)) * 16777619u; }
    return h;
}
/* 출발 정렬 표(start): 출발 자세와 출발 뒤 지표. old 1 = 출발 정렬 단계 끔(D.start_align 0 유지) */
static struct {
    int   on, old, lost, al_n;
    float off_u, th_deg, s0;            /* 옆 단위(+ 회전 중심이 오른쪽), 방향 어긋남 도, 곡선 위 출발 경로 자리 mm(0 = 직선) */
    float run_mm, yaw_max, izero, dev_max, align_mm, align_s, th_prev;
    float end_mm, end_s, t1m, vmax;     /* 출발 정렬 단계가 끝난 거리·시간(start_align 이 HOLD 아래로), 1 m 통과 시간, 최고 속도 m/min */
    float iclip;                        /* 1 m 안 안쪽 바퀴 지령이 0 아래에서 잘린 시간 s(izero 는 0 에 닿은 것까지) */
    int   t0, vflip, vdir, vprev, err_n; /* 출발 틱, 1 m 안 속도 지령(v_cmd) 증감 방향 바뀜, 오류 뒤 틱 */
} s_sa;
static int   s_acc, s_dcc;  /* 0 아니면 ACC·DCC 파라미터 */
static int   s_scan63;      /* 1: 센서가 6.3 ms 마다만 LED 를 새로 읽는다(실측 스캔 주기). 응답은 마지막 스캔 때 자세 */
static float s_hx[4], s_hy[4], s_hth[4];   /* 스캔 모의: 2 ms 전·4·6 ms 전 자세(0 = 지금) */
#ifndef EST_FLIP_HYS
#define EST_FLIP_HYS 20.0f     /* 바퀴 지령 차 방향 바뀜 히스테리시스 rpm. 펄스 1개 차의 ψ 계단(지령 차 약 ±10 rpm)은 세지 않는다 */
#endif

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
        memmove(&s_hx[1], &s_hx[0], 3 * sizeof(float));
        memmove(&s_hy[1], &s_hy[0], 3 * sizeof(float));
        memmove(&s_hth[1], &s_hth[0], 3 * sizeof(float));
        s_hx[0] = s_v.x;
        s_hy[0] = s_v.y;
        s_hth[0] = s_v.th;
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
    int   wn = 0, wdir = 0, wflips = 0, vdir = 0, vprev = 0;
    float wmin = 0.0f, wmax = 0.0f, wext = 0.0f, wref = 0.0f;
    int   risk_prev = 0;
#if AGV_DEV_TESTS
    float i_peak = 0.0f;
#endif

    memset(r, 0, sizeof(*r));
    r->hash = 2166136261u;
    memset(&s_v, 0, sizeof(s_v));
    s_c = c;
    s_v.s = GD_MM;
    if (c->mode == 4) { s_v.y = -c->deg * GUIDE_UNIT_MM; jog = 1; }
    if (s_sa.on) {      /* 출발 자세: 경로 s0(곡선 위면 접선 방향) + 오른쪽 off_u 단위 + 방향 th_deg */
        float px, py, tx, ty;
        path(s_sa.s0, &px, &py, &tx, &ty);
        s_v.th = atan2f(ty, tx);
        s_v.x = px + sinf(s_v.th) * s_sa.off_u * GUIDE_UNIT_MM;
        s_v.y = py - cosf(s_v.th) * s_sa.off_u * GUIDE_UNIT_MM;
        s_v.th += s_sa.th_deg * PI_F / 180.0f;
        s_v.s = s_sa.s0 + GD_MM;
        s_sa.run_mm = s_sa.yaw_max = s_sa.izero = s_sa.dev_max = s_sa.vmax = s_sa.iclip = 0.0f;
        s_sa.align_mm = s_sa.align_s = s_sa.end_mm = s_sa.end_s = s_sa.t1m = -1.0f;
        s_sa.lost = s_sa.al_n = s_sa.t0 = s_sa.vflip = s_sa.vdir = s_sa.vprev = s_sa.err_n = 0;
        s_sa.th_prev = s_v.th;
    }

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
    if (s_acc) { assert(param_set_ram(P_ACC, (int16_t)s_acc) == 0); }
    if (s_dcc) { assert(param_set_ram(P_DCC, (int16_t)s_dcc) == 0); }
    tune_turn_scale_set((int16_t)(TURN_SCALE * 1000.0f + 0.5f));
    curvelog_reset();
#if AGV_DEV_TESTS
    steer_alg_test = (int16_t)c->alg;
    speed_curve_test = (int16_t)(c->sct | s_sct);
    if (s_tst) { assert(param_set_ram(P_DEV_TEST, (int16_t)s_tst) == 0); }
    else if (s_kp) { assert(param_set_ram(P_STEER_KP, (int16_t)s_kp) == 0); }
    if (s_ki) { assert(param_set_ram(P_STEER_KI, (int16_t)s_ki) == 0); }
#endif

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
        if (s_scan63) {     /* 응답은 마지막 스캔(6.3 ms 간격) 때 자세로 만든다. 2 ms 자세 기록에서 가장 가까운 것 */
            float age = fmodf((float)t * 10.0f, 6.3f), x = s_v.x, y = s_v.y, th = s_v.th, s0 = s_v.s, u2 = 0.0f, sl2 = 1.0f;
            int h = (int)(age / 2.0f + 0.5f), seen2;
            if ((h > 0) && (t > 1)) { s_v.x = s_hx[h - 1]; s_v.y = s_hy[h - 1]; s_v.th = s_hth[h - 1]; }
            seen2 = cross(&u2, &sl2);
            if (seen2 && (fabsf(u2) > (float)c->leds * PITCH_MM / 2.0f + TAPE_MM)) { seen2 = 0; }
            if ((c->mode == 3) && (s_v.s > step_at) && (s_v.s <= step_at + 100.0f)) {
                seen2 = (s_v.s > step_at + 45.0f) && (s_v.s <= step_at + 45.0f + c->deg);
                u2 += 8.0f * GUIDE_UNIT_MM;
                sl2 = 1.0f;
            }
            sensor_reply(u2, sl2, seen2);
            s_v.x = x; s_v.y = y; s_v.th = th; s_v.s = s0;
        }
        else { sensor_reply(u, slant, seen); }
        {   float x_was = s_v.x, y_was = s_v.y;
            plant();
            if (s_sa.on && s_sa.old) { D.start_align = 0; }
            scan();
            if (s_sa.on && s_sa.old) { D.start_align = 0; }
            if (s_sa.on && D.flag_run) {    /* 출발 뒤 1 m 지표, 정렬(편차 ±3단위 10틱, 선 보임)까지 거리·시간 */
                float yr = fabsf(s_v.th - s_sa.th_prev) * 100.0f * 180.0f / PI_F, d = fabsf(seen ? u : (float)c->leds * PITCH_MM / 2.0f);
                float vv = (s_v.rpm[0] + s_v.rpm[1]) * 0.5f / P.one_meter_rpm;
                if (s_sa.t0 == 0) { s_sa.t0 = t; }
                s_sa.th_prev = s_v.th;
                s_sa.run_mm += sqrtf((s_v.x - x_was) * (s_v.x - x_was) + (s_v.y - y_was) * (s_v.y - y_was));
                if (vv > s_sa.vmax) { s_sa.vmax = vv; }
                if (s_sa.run_mm <= 1000.0f) {
                    if (yr > s_sa.yaw_max) { s_sa.yaw_max = yr; }
                    if ((D.vin_out > 0) && ((D.w_cmd[0] == 0) || (D.w_cmd[1] == 0))) { s_sa.izero += 0.01f; }
                    if ((D.vin_out > 0) && ((D.add_speed_l + D.v_cmd < 0) || (D.add_speed_r + D.v_cmd < 0))) { s_sa.iclip += 0.01f; }
                    if (d > s_sa.dev_max) { s_sa.dev_max = d; }
                    if (D.guide_all_off) { s_sa.lost++; }
                    if ((D.v_cmd != s_sa.vprev) && (s_sa.vprev != 0)) {     /* 속도 지령 증감 방향 바뀜 */
                        int dir = (D.v_cmd > s_sa.vprev) ? 1 : -1;
                        if ((s_sa.vdir != 0) && (dir != s_sa.vdir)) { s_sa.vflip++; }
                        s_sa.vdir = dir;
                    }
                    s_sa.vprev = D.v_cmd;
                }
                else if (s_sa.t1m < 0.0f) { s_sa.t1m = (float)(t - s_sa.t0) * 0.01f; }
                if ((s_sa.end_mm < 0.0f) && (D.start_align < DRIVE_ALIGN_HOLD)) { s_sa.end_mm = s_sa.run_mm; s_sa.end_s = (float)(t - s_sa.t0) * 0.01f; }
                if (s_sa.align_mm < 0.0f) {
                    s_sa.al_n = ((D.guide_pos <= 3) && (D.guide_pos >= -3) && !D.guide_all_off) ? s_sa.al_n + 1 : 0;
                    if (s_sa.al_n > 10) { s_sa.align_mm = s_sa.run_mm; s_sa.align_s = (float)(t - s_sa.t0) * 0.01f; }
                }
                if (s_trace) {
                    printf("  t %4d pos %3d off %d al %2d step %d risk %d cmd %5d v %5d w %5d %5d out %6.0f run %6.0f\n", t - s_sa.t0, D.guide_pos,
                           D.guide_all_off, D.start_align, D.step, D.risk_stop, D.speed_cmd, D.v_cmd, D.w_cmd[0], D.w_cmd[1], D.p_pi_out, s_sa.run_mm);
                }
                if (s_sa.run_mm > 2500.0f) { break; }
            }
            if (s_sa.on && (D.stop_err || D.system_err) && ((++s_sa.err_n > 500) || ((D.vin_out == 0) && (s_v.rpm[0] + s_v.rpm[1] < 1.0f)))) { break; }
        }
        if (!sped && D.flag_run && (s_v.s > 500.0f)) { tagact_speed((uint16_t)c->v_mpm); sped = 1; }
        if ((c->mode == 1) && (s_v.s > step_at)) { D.p_test = 4; }
        if ((c->mode == 2) && (s_v.s > step_at) && !jog) { s_v.y -= c->deg * GUIDE_UNIT_MM; jog = 1; }   /* 테이프가 왼쪽으로 deg 단위 꺾였다 */

        dev = seen ? u : (float)c->leds * PITCH_MM / 2.0f;
        if (c->mode == 3) { dev = -s_v.y - GD_MM * sinf(s_v.th); }   /* 끊긴 자리에서도 주행선 기준 */
        if (D.guide_all_off && D.flag_run) { r->lost++; }
        if ((arc > 0.0f) && (s_v.s > LEAD_MM - 500.0f) && (s_v.s < LEAD_MM + arc + 500.0f)) {
            float vv = (s_v.rpm[0] + s_v.rpm[1]) * 0.5f / P.one_meter_rpm, ce = fabsf(center_err());
            r->pass_s += 0.01f;
            if ((D.v_cmd != vprev) && (vprev != 0)) {
                int dir = (D.v_cmd > vprev) ? 1 : -1;
                if ((vdir != 0) && (dir != vdir)) { r->vflip++; }
                vdir = dir;
            }
            vprev = D.v_cmd;
            if (vv < r->vmin) { r->vmin = vv; }
            if (ce > r->cmax) { r->cmax = ce; }
        }
        if ((D.control == RUN_DRIVE) && (D.vin_out > 0) && ((D.w_cmd[0] == 0) || (D.w_cmd[1] == 0))) { r->izero_s += 0.01f; }
        if (D.risk_stop) { r->risk++; }
        if (D.risk_stop && !risk_prev) { r->risk_n++; }
        risk_prev = D.risk_stop;
        r->hash = fnv(r->hash, D.p_pi_out);
#if AGV_DEV_TESTS
        if (in_curve) {
            float ia = fabsf(steer_i_out());
            if (ia > i_peak) { i_peak = ia; }
            if (i_peak - ia > r->idrop) { r->idrop = i_peak - ia; }
        }
#endif
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
        if (in_curve && (s_v.s > LEAD_MM + 800.0f) && (D.control == RUN_DRIVE)) {    /* 바퀴 지령 차 흔들림·방향 바뀜 */
            float d = fabsf((float)(D.w_cmd[0] - D.w_cmd[1]));
            if (wn == 0) { wmin = wmax = wref = d; }
            if (d < wmin) { wmin = d; }
            if (d > wmax) { wmax = d; }
            if (wdir == 0) {
                if (d >= wref + EST_FLIP_HYS)      { wdir = 1; wext = d; }
                else if (d <= wref - EST_FLIP_HYS) { wdir = -1; wext = d; }
            }
            else if (wdir > 0) {
                if (d > wext) { wext = d; }
                else if (d <= wext - EST_FLIP_HYS) { wdir = -1; wext = d; wflips++; }
            }
            else {
                if (d < wext) { wext = d; }
                else if (d >= wext + EST_FLIP_HYS) { wdir = 1; wext = d; wflips++; }
            }
            wn++;
        }
        if (in_curve) {
            float d = hypotf(s_v.x - px, s_v.y - py);
            if (!in_prev) { th0 = s_v.th; d = 0.0f; }
            if ((s_v.s <= LEAD_MM + arc * 0.5f) && (abs(D.guide_pos) > r->pin)) { r->pin = abs(D.guide_pos); }
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
        if (s_trace && !s_sa.on && (s_v.s > ((arc > 0.0f) ? LEAD_MM : step_at) - 600.0f) && (s_v.s < ((arc > 0.0f) ? LEAD_MM + arc : step_at) + 1200.0f)) {
printf("T %6.0f %d dev %6.1f pos %3d trk %d out %6.0f v %5d rpm %5.0f %5.0f risk %d", s_v.s - ((arc > 0.0f) ? LEAD_MM : step_at), in_curve, dev, D.guide_pos, D.guide_track, D.p_pi_out, D.v_cmd, s_v.rpm[0], s_v.rpm[1], D.risk_stop);
#if AGV_DEV_TESTS
            printf(" i %6.0f", steer_i_out());
#endif
            printf("\n");
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
    r->wob = (wn > 0) ? (((wmax + wmin) > 0.0f) ? 100.0f * (wmax - wmin) / (wmax + wmin) : 0.0f) : -1.0f;
    r->wflip = (wn > 0) ? (float)wflips / ((float)wn * 0.01f) : 0.0f;
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
typedef struct {
    float r2, v_center, err_deg, t_turn, a_fit, line_deg, ctr_mm, dec_max; int fix, reacc, err, done;
    /* 등감속 지표(v0.38.0). 각은 회전 끝(실제 90도)까지 남은 도 */
    float dec_deg, line_v, line_at, floor_deg, floor_s, floor_ang, stop_s;   /* 감속 시작 각, 라인 감지 때 지령 rpm·남은 각, 바닥 진입 각, 바닥 구간 s·도, 감속 시작 → 정지 s */
    float ratio, const_s, a_min, a_max;   /* 감속 시작 → 지령 2 × 바닥 지령 감속도(rpm/틱) 최소/최대 비, 감속 시작 → 바닥 진입 정속(감속 0) s */
    float tail_s, tail_ang, drop_min;     /* 꼬리(지령 2 × 바닥 → 정지) s·도, 비 구간 안 이웃 틱 감속도 비 최소 */
    int   drop_n;                         /* 비 구간 안 감속도가 앞 틱보다 20% 넘게 떨어진 횟수 */
} turn_res_t;
static int s_ttrace;            /* turn 궤적 찍기 */

static void turn_sim(float gd, int leds, float tau, float slip, int tns, int dcc, turn_res_t *r)
{
    static const case_t c0 = { 32, 0.0f, 0.0f, 1, 20, 20, 30, 0.1f, 0, 0 };
    case_t c = c0;
    int t, armed = 0, phase = 0, t0 = 0, td = -1, tf = -1, tw = -1, seen0 = 0, zero = 0, nd = 0;
    float aprev = 0.0f;
    float sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0, n = 0, prev_v = 1e9f, vpeak_dec = 0.0f, v_peak = 0.0f, vprev = 0.0f, th_prev = 0.0f;

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
    assert(param_set_ram(P_SPEED_TURN, (int16_t)tns) == 0);
    assert(param_set_ram(P_DCC, (int16_t)dcc) == 0);
    assert(param_set_ram(P_WHEEL_BASE_MM, (int16_t)(WB_MM / 10.0f)) == 0);   /* 다시 계산(회전 속도) */
    tune_turn_scale_set((int16_t)(TURN_SCALE * 1000.0f + 0.5f));
    host.bus_in[BUS_IN_A] &= (uint16_t)~RAW_START;
    scan();
    host.bus_in[BUS_IN_A] |= RAW_START;
    scan();
    for (t = 0; t < 8000; t++) {
        float r0 = s_v.rpm[0], r1 = s_v.rpm[1];
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
        if (phase == 1) {   /* 바퀴 감속 최댓값 rpm/s(모터축) */
            float d0 = (r0 - s_v.rpm[0]) * 100.0f, d1 = (r1 - s_v.rpm[1]) * 100.0f;
            if (d0 > r->dec_max) { r->dec_max = d0; }
            if (d1 > r->dec_max) { r->dec_max = d1; }
        }
        if (phase == 1) {
            float rem = (90.0f + s_v.th * 180.0f / PI_F);       /* 남은 실제 각(도), CW 라 th 가 -90 으로 간다 */
            float v = (float)D.vin_out;
            if ((D.tape_step <= 2) && (rem < 25.0f) && (rem > 0.3f) && (v > 0.0f)) {
                sx += rem; sy += v * v; sxx += rem * rem; sxy += rem * v * v; syy += v * v * v * v; n += 1.0f;
                if ((prev_v < 1e8f) && (v > prev_v + 1.0f) && (v_peak - prev_v > 20.0f)) { r->reacc++; }   /* 감속을 시작한 뒤 올림 */
                if (v > v_peak) { v_peak = v; }
                prev_v = v;
            }
            if ((D.tape_step == TAPE_STEP_SETTLE) && (r->v_center == 0.0f)) { r->v_center = (vpeak_dec > 0.0f) ? vpeak_dec : 0.01f; }
            if (D.tape_step <= 2) { vpeak_dec = (float)D.vin_out; }     /* 중앙에서 서기 직전 지령 */
            {   /* 등감속 지표: 감속 시작(지령이 처음 줄어든 틱)부터. 비는 지령 2 × 바닥까지(꼬리는 따로), 정속은 바닥 진입까지 */
                float v = (float)D.vin_out, flo = (float)stop_min_rpm();
                if ((td < 0) && (D.tape_step <= 2) && (v < vprev) && (vprev > flo)) { td = t; r->dec_deg = rem; r->drop_min = 1.0f; }
                if ((td >= 0) && (tf < 0) && (D.tape_step <= 2)) {
                    if (v <= flo) { tf = t; r->floor_deg = rem; }
                    else if (t > td) {
                        float a = vprev - v;
                        if (a <= 0.0f) { zero++; }
                        if ((tw < 0) && (v <= 2.0f * flo)) { tw = t; }
                        if ((tw < 0) && (a > 0.0f)) {
                            if ((nd == 0) || (a < r->a_min)) { r->a_min = a; }
                            if ((nd == 0) || (a > r->a_max)) { r->a_max = a; }
                            if ((nd > 0) && (aprev > 0.0f)) {
                                if (a < 0.8f * aprev) { r->drop_n++; }
                                if (a / aprev < r->drop_min) { r->drop_min = a / aprev; }
                            }
                            nd++;
                        }
                        aprev = a;
                    }
                }
                if ((tw >= 0) && (D.tape_step <= 2)) { r->tail_s += 0.01f; r->tail_ang += fabsf(s_v.th - th_prev) * 180.0f / PI_F; }
                if (!seen0 && (D.tape_step >= 1) && (D.tape_step <= 2) && !D.guide_all_off && (rem < 60.0f)) { seen0 = 1; r->line_v = v; r->line_at = rem; }
                if ((D.tape_step <= 2) && (td >= 0) && (v <= flo) && (v > 0.0f)) { r->floor_s += 0.01f; r->floor_ang += fabsf(s_v.th - th_prev) * 180.0f / PI_F; }
                if ((td >= 0) && (r->stop_s == 0.0f) && (D.tape_step == TAPE_STEP_SETTLE)) { r->stop_s = (float)(t - td) * 0.01f; }
                if (s_ttrace) { printf("    t %4d step %d vin %5.0f rpm %5d/%5d plant %5.0f rem %7.2f pos %3d\n", t - t0, D.tape_step, v, D.rpm[0], D.rpm[1], s_v.rpm[0], rem, D.guide_pos); }
                vprev = v;
                th_prev = s_v.th;
            }
            if (!D.tape_align && !D.flag_turn) { phase = 2; r->t_turn = (float)(t - t0) * 0.01f; break; }
        }
        if (D.system_err || D.stop_err) { break; }
    }
    (void)vpeak_dec;
    r->const_s = (float)zero * 0.01f;
    r->ratio = (r->a_max > 0.0f) ? r->a_min / r->a_max : 0.0f;
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

static void turn_line(int gd, int leds, int tau, float sl, int tns, int dcc, const turn_res_t *r)
{
    printf("TURN GD%d/L%d/t%d/s%.2f/T%d/D%d | done %d err %d vcenter %.0f fix %d time %.2f reacc %d linedeg %.2f decmax %.0f"
           " | dec %.1f line %.0f@%.1f floor %.1f fs %.2f fa %.2f stop %.2f ratio %.2f const %.2f a %.0f-%.0f tail %.2f ta %.2f drop %d dmin %.2f\n",
           gd, leds, tau, sl, tns, dcc, r->done, r->err, r->v_center, r->fix, r->t_turn, r->reacc, r->line_deg, r->dec_max,
           r->dec_deg, r->line_v, r->line_at, r->floor_deg, r->floor_s, r->floor_ang, r->stop_s, r->ratio, r->const_s, r->a_min, r->a_max,
           r->tail_s, r->tail_ang, r->drop_n, r->drop_min);
}

static void turn_table(int full)
{
    static const float sl[3] = { 0.8f, 1.0f, 1.25f };
    static const int tn[3] = { 7, 15, 25 }, dc[3] = { 20, 40, 50 };
    int g, l, t, k, a, b;
    for (g = 0; g < (full ? 1 : 2); g++) {
        for (l = 0; l < 2; l++) {
            for (t = 0; t < 2; t++) {
                for (k = 0; k < 3; k++) for (a = 0; a < (full ? 3 : 1); a++) for (b = 0; b < (full ? 3 : 1); b++) {
                    turn_res_t r;
                    int tns = full ? tn[a] : 13, dcc = full ? dc[b] : 40;
                    turn_sim(g ? 500.0f : 250.0f, l ? 16 : 32, t ? 0.15f : 0.1f, sl[k], tns, dcc, &r);
                    turn_line(g ? 500 : 250, l ? 16 : 32, t ? 150 : 100, sl[k], tns, dcc, &r);
                }
                if (full) {         /* v0.38.0: TNS 25 · DCC 60(5호기 t1 결과) */
                    for (k = 0; k < 3; k++) {
                        turn_res_t r;
                        turn_sim(250.0f, l ? 16 : 32, t ? 0.15f : 0.1f, sl[k], 25, 60, &r);
                        turn_line(250, l ? 16 : 32, t ? 150 : 100, sl[k], 25, 60, &r);
                    }
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


#if AGV_DEV_TESTS
/* ---- 추정 조향(steer_alg_test 1) 대 37차 조향 ----
 * 곡선 s_cur6 × 속도 12·25·31 m/min × 바퀴 지연 100·200·300 ms × 센서 32·16칸 × GD 250·500 × 조향 2종.
 * 칸: 편차 최대·RMS mm, 곡선 평균 속도, 위험 감속 틱, 선 잃음 틱, 바퀴 지령 차 흔들림 %·방향 바뀜/s, 출구 넘침 mm, 끝 1 m RMS mm.
 * 합격(GD 250, 조건마다 37차와 견줌): 선 잃음 0, 방향 바뀜 ≤ 37차/2, 편차 최대 ≤ 37차, 끝 RMS ≤ 37차 + 1 mm.
 * 결과는 s_es 에 남긴다 */
static float med_f(float *v, int n)
{
    if (n <= 0) { return 0.0f; }
    qsort(v, (size_t)n, sizeof(float), cmp_f);
    return ((n & 1) != 0) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

/* 한 칸(GD·센서) 요약. 기준별 불합격 수는 조건마다 37차와 견준 것. 끝 RMS 는 37차가 끝까지 간 조건만 본다 */
typedef struct {
    int   n, nw, lost[2], risk[2], f_lost, f_max, f_flip, f_tail, bad, worse;
    int   lost_new;         /* 37차는 끝까지 갔는데 추정만 선을 잃은 조건 */
    float max[2], flip[2], wob[2], eover[2], tail[2];
} est_sum_t;
static est_sum_t s_es[2][2];        /* [GD 250·500][wide·일반] */

static void est_table(int print, int scan)
{
    static const int   vv[3] = { 12, 25, 31 };
    static const float tt[3] = { 0.1f, 0.2f, 0.3f };
    static float mx[2][54], fl[2][54], eo[2][54], wb[2][54], tr[2][54];
    static const char *an[2] = { "37차", "추정" };
    int g, l, v, t, i, a;
    s_scan63 = scan;
    memset(s_es, 0, sizeof(s_es));
    for (g = 0; g < 2; g++) {
        for (l = 0; l < 2; l++) {
            est_sum_t *e = &s_es[g][l];
            for (v = 0; v < 3; v++) {
                for (t = 0; t < 3; t++) {
                    for (i = 0; i < 6; i++) {
                        res_t r[2];
                        int ok0, f_lost, f_max, f_flip, f_tail;
                        for (a = 0; a < 2; a++) {
                            case_t c = { l ? 16 : 32, s_cur6[i].r, s_cur6[i].deg, s_cur6[i].dir, vv[v], vv[v], 31, tt[t], 0, 0, a };
                            s_gd = g ? 500.0f : 250.0f;
                            sim(&c, &r[a]);
                            if (print > 1) {
                                printf("E GD%d L%d v%d t%d %-5s %s | max %5.1f rms %5.1f v %4.1f 위험 %4d 잃음 %3d 흔들림 %5.1f%% 바뀜 %4.1f/s 넘침 %5.1f 끝RMS %4.1f err %d\n",
                                       g ? 500 : 250, c.leds, c.v_mpm, (int)(c.tau * 1000.0f + 0.5f), s_cur6[i].name, an[a], r[a].max, r[a].rms,
                                       r[a].v_avg, r[a].risk, r[a].lost, r[a].wob, r[a].wflip, r[a].eover, r[a].tail_rms, r[a].err);
                            }
                            mx[a][e->n] = r[a].max;
                            eo[a][e->n] = r[a].eover;
                            tr[a][e->n] = r[a].tail_rms;
                            e->lost[a] += (r[a].lost > 0) || (r[a].err != 0);
                            e->risk[a] += (r[a].risk > 0);
                            if (r[a].wob >= 0.0f) { fl[a][e->nw] = r[a].wflip; wb[a][e->nw] = r[a].wob; }
                        }
                        if (r[0].wob >= 0.0f) { e->nw++; }
                        e->n++;
                        ok0 = (r[0].lost == 0) && (r[0].err == 0);
                        f_lost = (r[1].lost > 0) || (r[1].err != 0);
                        f_max = r[1].max > r[0].max;
                        f_flip = r[1].wflip > 0.5f * r[0].wflip;
                        f_tail = ok0 && (r[1].tail_rms > r[0].tail_rms + 1.0f);
                        e->f_lost += f_lost;
                        e->lost_new += f_lost && ok0;
                        e->f_max += f_max;
                        e->f_flip += f_flip;
                        e->f_tail += f_tail;
                        e->worse += f_max || (f_lost && ok0) || (r[1].wflip > r[0].wflip) || (r[1].eover > r[0].eover + 1.0f);
                        if (f_lost || f_max || f_flip || f_tail) {
                            e->bad++;
                            if (print > 0) {
                                printf("  불합격 GD%d L%d v%d t%d %s: 최대 %.1f/%.1f 바뀜 %.1f/%.1f 끝RMS %.1f/%.1f 잃음 %d/%d\n", g ? 500 : 250, l ? 16 : 32,
                                       vv[v], (int)(tt[t] * 1000.0f + 0.5f), s_cur6[i].name, r[1].max, r[0].max, r[1].wflip, r[0].wflip,
                                       r[1].tail_rms, r[0].tail_rms, r[1].lost, r[0].lost);
                            }
                        }
                    }
                }
            }
            for (a = 0; a < 2; a++) {
                e->max[a] = med_f(mx[a], e->n);
                e->flip[a] = med_f(fl[a], e->nw);
                e->wob[a] = med_f(wb[a], e->nw);
                e->eover[a] = med_f(eo[a], e->n);
                e->tail[a] = med_f(tr[a], e->n);
                printf("S%s GD%d L%d %-4s | n %d 중앙: 편차최대 %5.1f 바뀜/s %4.1f(정상구간 %d) 흔들림 %4.1f%% 넘침 %4.1f 끝RMS %4.1f | 잃음 조건 %d 위험 감속 조건 %d\n",
                       scan ? "6.3" : "", g ? 500 : 250, l ? 16 : 32, an[a], e->n, e->max[a], e->flip[a], e->nw, e->wob[a], e->eover[a], e->tail[a],
                       e->lost[a], e->risk[a]);
            }
            printf("S%s GD%d L%d 추정이 37차보다 나쁜 조건 %d/%d(최대·잃음·바뀜·넘침+1 중 하나) | 합격선 밖 %d: 잃음 %d 최대 %d 바뀜 %d 끝RMS %d\n",
                   scan ? "6.3" : "", g ? 500 : 250, l ? 16 : 32, e->worse, e->n, e->bad, e->f_lost, e->f_max, e->f_flip, e->f_tail);
        }
    }
    s_gd = 250.0f;
    s_scan63 = 0;
}

/* 반응형 곡선 속도(speed_curve_test) 표. 조향 2종 × 끔/켬, 태그 없음.
 * 급한 곡선: R 400·500·600 mm 90도 × 31·37 m/min 진입. 완만한 곡선: R 1000·1500·4000 mm × 25 m/min.
 * × 지연 100·200·300 ms × 센서 32·16 × GD 250·500. 칸: 선 잃음 조건, 편차 최대·바뀜/s·출구 넘침·통과 시간·최저 속도 중앙,
 * 위험 감속 조건, 속도 지령 방향 바뀜 최대. 결과는 s_sc_sum 에 남긴다([급·완만][GD][센서][조향][끔·켬]) */
typedef struct { int n, lost, risk, vflip; float max, flip, eover, pass, vmin; } sc_sum_t;
static sc_sum_t s_sc_sum[2][2][2][2][2];

static void sc_table(int print)
{
    static const float rr[2][3] = { { 400.0f, 500.0f, 600.0f }, { 1000.0f, 1500.0f, 4000.0f } };
    static const float aa[2][3] = { { 90.0f, 90.0f, 90.0f }, { 90.0f, 90.0f, 45.0f } };
    static const int   vv[2][2] = { { 31, 37 }, { 25, 25 } };
    static const float tt[3] = { 0.1f, 0.2f, 0.3f };
    static float mx[18], fl[18], eo[18], ps[18], vm[18];
    static const char *gn[2] = { "급", "완만" };
    int k, g, l, a, s, v, t, i, nv;
    for (k = 0; k < 2; k++) {
        nv = k ? 1 : 2;
        for (g = 0; g < 2; g++) {
            for (l = 0; l < 2; l++) {
                for (a = 0; a < 2; a++) {
                    for (s = 0; s < 2; s++) {
                        sc_sum_t *e = &s_sc_sum[k][g][l][a][s];
                        memset(e, 0, sizeof(*e));
                        for (v = 0; v < nv; v++) {
                            for (t = 0; t < 3; t++) {
                                for (i = 0; i < 3; i++) {
                                    res_t r;
                                    case_t c = { l ? 16 : 32, rr[k][i], aa[k][i], (i & 1) ? 1 : -1, vv[k][v], vv[k][v], 37, tt[t], 0, 0, a, s };
                                    s_gd = g ? 500.0f : 250.0f;
                                    sim(&c, &r);
                                    if (print) {
                                        printf("V %s GD%d L%d R%.0f v%d t%d %s %s | max %5.1f 잃음 %3d 바뀜 %4.1f 넘침 %5.1f 통과 %5.2f s 최저 %4.1f 위험 %4d 속도바뀜 %d err %d\n",
                                               gn[k], g ? 500 : 250, c.leds, c.r_mm, c.v_mpm, (int)(c.tau * 1000.0f + 0.5f), a ? "추정" : "37차",
                                               s ? "켬" : "끔", r.max, r.lost, r.wflip, r.eover, r.pass_s, r.vmin, r.risk, r.vflip, r.err);
                                    }
                                    mx[e->n] = r.max;
                                    fl[e->n] = r.wflip;
                                    eo[e->n] = r.eover;
                                    ps[e->n] = r.pass_s;
                                    vm[e->n] = r.vmin;
                                    e->lost += (r.lost > 0) || (r.err != 0);
                                    e->risk += (r.risk > 0);
                                    if (r.vflip > e->vflip) { e->vflip = r.vflip; }
                                    e->n++;
                                }
                            }
                        }
                        e->max = med_f(mx, e->n);
                        e->flip = med_f(fl, e->n);
                        e->eover = med_f(eo, e->n);
                        e->pass = med_f(ps, e->n);
                        e->vmin = med_f(vm, e->n);
                        printf("SV %-4s GD%d L%d %s %s | n %d 잃음 조건 %2d | 중앙: 최대 %5.1f 바뀜/s %4.1f 넘침 %5.1f 통과 %5.2f s 최저 %4.1f m/min | 위험 감속 조건 %2d 속도바뀜 최대 %d\n",
                               gn[k], g ? 500 : 250, l ? 16 : 32, a ? "추정" : "37차", s ? "켬" : "끔", e->n, e->lost, e->max, e->flip, e->eover,
                               e->pass, e->vmin, e->risk, e->vflip);
                    }
                }
            }
        }
    }
    s_gd = 250.0f;
}

/* 잡음(모드 3 끊김 + 바닥 반사 점, 모드 6 30도 교차선): 추정 조향이 37차보다 나쁘지 않다. 반환 나쁜 조건 수 */
static int est_noise(int print)
{
    int g, l, v, k, m, a, worse = 0;
    for (g = 0; g < 2; g++) {
        for (l = 0; l < 2; l++) {
            for (v = 0; v < 2; v++) {
                for (k = 0; k < 2; k++) {
                    for (m = 0; m < 2; m++) {
                        res_t r[2];
                        for (a = 0; a < 2; a++) {
                            case_t c = { l ? 16 : 32, 0.0f, k ? 25.0f : 10.0f, 1, v ? 15 : 31, v ? 15 : 31, 31, 0.1f, 0, m ? 6 : 3, a };
                            s_gd = g ? 500.0f : 250.0f;
                            sim(&c, &r[a]);
                        }
                        if (print) {
                            printf("N GD%d L%d v%d %s%2.0f | 37차 편차 %5.1f 위치 %2d 잃음 %3d err %d | 추정 편차 %5.1f 위치 %2d 잃음 %3d err %d\n",
                                   g ? 500 : 250, l ? 16 : 32, v ? 15 : 31, m ? "교차" : "점", k ? 25.0f : 10.0f, r[0].exit_max, r[0].jump, r[0].lost,
                                   r[0].err, r[1].exit_max, r[1].jump, r[1].lost, r[1].err);
                        }
                        worse += (r[1].exit_max > r[0].exit_max + 1.0f) || (r[1].lost > r[0].lost) || ((r[1].err != 0) && (r[0].err == 0));
                    }
                }
            }
        }
    }
    s_gd = 250.0f;
    return worse;
}

/* ---- 스캔 주기 시험(t3.8 GS) 추정 모의 ----
 * 360도 제자리 회전 기하: 센서 줄은 회전 중심에서 GD 앞, 테이프는 회전 중심을 지나는 직선. 각 θ 에서 센서 줄 위 테이프 자리
 * u = GD·tanθ (옆 속도 GD·ω/cos²θ), 폭 w/|cosθ|. wide 32칸(6.89 mm), 칸의 0.35 넘게 덮이면 켜짐.
 * 회전: 0.5 s 가속, 정속 ω = 2·TNS/WW, 0.5 s 감속. 센서는 위상 무작위·시계 오차 ±0.5% 로 P 마다 읽고 마지막 스캔을 답한다
 * (P 0 = 요청을 받을 때마다 새로 읽음). 요청은 1.6 ms ± 0.05 ms 마다, 요청 4바이트(0.35 ms) 뒤에 센서가 받는다 */
static uint32_t s_gseed;
static float gs_rand(void) { s_gseed = s_gseed * 1103515245UL + 12345UL; return (float)((s_gseed >> 8) & 0xFFFFu) / 65536.0f; }

static float gs_theta(float t, float w, float t_end)
{
    const float ta = 0.5f;
    if (t <= 0.0f) { return 0.0f; }
    if (t < ta) { return 0.5f * (w / ta) * t * t; }
    if (t < t_end - ta) { return 0.5f * w * ta + w * (t - ta); }
    if (t < t_end) { float r = t_end - t; return 2.0f * PI_F - 0.5f * (w / ta) * r * r; }
    return 2.0f * PI_F;
}

static uint32_t gs_bits(float th, float tape_mm)
{
    float c = cosf(th), u, wc, cc;
    uint32_t bits = 0;
    int i;
    if (fabsf(c) < 0.05f) { return 0; }
    u = 250.0f * sinf(th) / c;
    wc = tape_mm / fabsf(c) / PITCH_MM;
    cc = 16.5f + u / PITCH_MM;
    for (i = 1; i <= 32; i++) { if (cover(cc, wc, i) > 0.35f) { bits |= 1UL << (i - 1); } }
    return bits;
}

typedef struct { float p_true; int gsn, gsl, gsp, p100, r1000, pairs; } gs_res_t;

/* p_ms 0 = 요청마다 새로 읽음 */
static void gs_sim(float p_ms, int tns, float tape_mm, uint32_t seed, gs_res_t *r)
{
    float w = 2.0f * ((float)tns / 60.0f) * 1000.0f / (WB_MM * TURN_SCALE);    /* rad/s */
    float t_end = 2.0f * PI_F / w + 0.5f;
    float e = 0.0f, ph = 0.0f;
    double t = 0.0;
    uint32_t prev = 0xFFFFFFFFUL;

    s_gseed = seed;
    if (p_ms > 0.0f) {
        e = (gs_rand() * 2.0f - 1.0f) * 0.005f;
        ph = gs_rand() * p_ms * 0.001f;
    }
    r->p_true = p_ms * (1.0f + e);
    memset(&agv_gs, 0, sizeof(agv_gs));
    while (t < (double)t_end + 0.2) {
        float ta = (float)t + 0.000347f, ts = ta;
        uint32_t b;
        if (p_ms > 0.0f) {               /* 마지막 스캔 */
            float pp = r->p_true * 0.001f;
            float k = floorf((ta - ph) / pp);
            ts = ph + k * pp;
        }
        b = gs_bits(gs_theta(ts, w, t_end), tape_mm);
        if ((prev != 0xFFFFFFFFUL) && (b != prev)) {
            if (agv_gs.nt < GS_BUF) { agv_gs.t[agv_gs.nt++] = (uint32_t)(t * 1.0e8) + 12345u; }
            agv_gs.gsn++;
        }
        prev = b;
        t += 0.0016 + (double)((gs_rand() - 0.5f) * 0.0001f);
    }
    gscan_calc_begin();
    while (gscan_calc_step(1000)) { }
    r->gsn = agv_gs.gsn;
    r->gsl = agv_gs.gsl;
    r->gsp = agv_gs.gsp;
    r->p100 = agv_gs.p100;
    r->r1000 = agv_gs.r1000;
    r->pairs = agv_gs.pairs;
}

/* 표: 모델별 시드 seeds 개 × TNS 15·25 × 테이프 25·50 mm */
static int gs_table(int seeds, int print)
{
    static const float ps[9] = { 2.0f, 2.5f, 3.0f, 3.5f, 4.0f, 5.0f, 8.0f, 10.0f, 0.0f };
    int m, bad = 0;
    for (m = 0; m < 9; m++) {
        int n = 0, ok = 0, none = 0, wrong = 0, gsn_min = 99999, gsn_max = 0, gsl_min = 99999, rmin = 9999, rmax = 0, k;
        float gsn_sum = 0.0f, err_max = 0.0f;
        for (k = 0; k < seeds * 4; k++) {
            gs_res_t r;
            gs_sim(ps[m], (k & 1) ? 25 : 15, (k & 2) ? 50.0f : 25.0f, 1000u + 7919u * (uint32_t)k + 104729u * (uint32_t)m, &r);
            n++;
            gsn_sum += (float)r.gsn;
            if (r.gsn < gsn_min) { gsn_min = r.gsn; }
            if (r.gsn > gsn_max) { gsn_max = r.gsn; }
            if ((r.gsl != TUNE_NO_DATA) && (r.gsl < gsl_min)) { gsl_min = r.gsl; }
            if (r.r1000 < rmin) { rmin = r.r1000; }
            if (r.r1000 > rmax) { rmax = r.r1000; }
            if (r.gsp == TUNE_NO_DATA) { none++; }
            else {
                float er = fabsf((float)r.p100 * 0.01f - r.p_true);
                if (er > err_max) { err_max = er; }
                if ((ps[m] > 0.0f) && (er <= 0.1f)) { ok++; } else { wrong++; }
            }
            if (print > 1) { printf("    P %.1f k %d: GSN %d GSL %d GSP %d p100 %d R %d pairs %d true %.3f\n", ps[m], k, r.gsn, r.gsl, r.gsp, r.p100, r.r1000, r.pairs, r.p_true); }
        }
        if (print) {
            printf("GS P %s%4.1f | n %d GSN %d~%d (평균 %.0f) GSL 최소 %d | GSP 맞음 %d 없음 %d 틀림 %d, 맞은 오차 최대 %.3f ms | R×1000 %d~%d\n",
                   (ps[m] > 0.0f) ? "" : "요청마다 ", ps[m], n, gsn_min, gsn_max, gsn_sum / (float)n, gsl_min, ok, none, wrong, err_max, rmin, rmax);
        }
        /* 3.5 ms 이상은 모두 0.1 ms 안으로 찾고, 요청마다 읽는 센서는 모두 없음. 3.2 ms(요청 주기 1.6 × 2) 아래는
         * 요청 주기와의 맥놀이 P·T/(P − T) 와 가를 수 없어 하한 밖이다(3.0 ms 는 몇 % 가 3.43 ms) */
        if ((ps[m] >= 3.5f) && ((none > 0) || (wrong > 0))) { bad++; }
        if ((ps[m] == 0.0f) && (none != n)) { bad++; }
    }
    return bad;
}
#endif


#if AGV_DEV_TESTS
/* ---- t3.8 GS 전 과정 모의: 튜닝 메뉴 → START → 라인 확인 → 360도 정렬 회전(실제 회전 코드·차 모형) → 계산 → 결과 화면.
 * 메인 루프 10 us, 틱 2 ms, 차 모형 10 ms(그 사이 자세는 보간). 센서는 테이프(y = 0 직선) 기하로 스캔 주기 s_gf.p 마다 읽고
 * 요청을 받으면(요청 4바이트 뒤) 마지막 스캔을 0.2 ms 뒤 12바이트로 답한다(약 1.6 ms). p 0 = 요청마다 새로 읽음 */
static struct { float p, ph, tape; float hx[3], hy[3], hth[3]; uint32_t ht[3]; uint32_t busy; int line; } s_gf;

static void gf_pose(uint32_t at, float *x, float *y, float *th)
{
    int k = ((int32_t)(at - s_gf.ht[1]) >= 0) ? 1 : 0;      /* [ht0, ht1] 또는 [ht1, ht2] */
    float f = (float)(int32_t)(at - s_gf.ht[k]) / (float)(int32_t)(s_gf.ht[k + 1] - s_gf.ht[k]);
    if (f < 0.0f) { f = 0.0f; }
    if (f > 1.0f) { f = 1.0f; }
    *x = s_gf.hx[k] + (s_gf.hx[k + 1] - s_gf.hx[k]) * f;
    *y = s_gf.hy[k] + (s_gf.hy[k + 1] - s_gf.hy[k]) * f;
    *th = s_gf.hth[k] + (s_gf.hth[k + 1] - s_gf.hth[k]) * f;
}

static uint32_t gf_bits(uint32_t at)
{
    float x, y, th, c, sy, u, cc, wc;
    uint32_t bits = 0;
    int i;
    gf_pose(at, &x, &y, &th);
    if (!s_gf.line) { return 0; }
    c = cosf(th);
    if (fabsf(c) < 0.05f) { return 0; }
    sy = y + GD_MM * sinf(th);
    u = -sy / c;
    cc = 16.5f + u / PITCH_MM;
    wc = s_gf.tape / fabsf(c) / PITCH_MM;
    for (i = 1; i <= 32; i++) { if (cover(cc, wc, i) > 0.35f) { bits |= 1UL << (i - 1); } }
    return bits;
}

static void gf_send(uint16_t port, const uint16_t *buf, uint16_t n)
{
    static const char hex[] = "0123456789abcdef";
    host_sci_t *b = &host.sci[HAL_SCI_B];
    uint32_t arr = host.cycles + n * 8681u, ts = arr, bits, t;
    uint16_t f[12], sum = 0;
    int i, k;

    if ((port != HAL_SCI_B) || (n < 2) || (buf[1] != 'H')) { return; }
    if ((int32_t)(arr - s_gf.busy) < 0) { return; }
    if (s_gf.p > 0.0f) {             /* 마지막 스캔 */
        float pc = s_gf.p * 100000.0f;
        float kk = floorf(((float)arr - s_gf.ph) / pc);
        ts = (uint32_t)(s_gf.ph + kk * pc);
    }
    bits = gf_bits(ts);
    f[0] = 2;
    f[1] = 'h';
    for (i = 0; i < 8; i++) { f[2 + i] = (uint16_t)hex[(bits >> (i * 4)) & 0xFu]; }
    for (i = 1; i < 10; i++) { sum = (uint16_t)(sum + f[i]); }
    f[10] = (uint16_t)hex[sum & 0xFu];
    f[11] = 3;
    k = b->rx_len - b->rx_pos;
    memmove(b->rx, &b->rx[b->rx_pos], (size_t)k * sizeof(b->rx[0]));
    memmove(b->rx_at, &b->rx_at[b->rx_pos], (size_t)k * sizeof(b->rx_at[0]));
    b->rx_len = (uint16_t)k;
    b->rx_pos = 0;
    t = arr + 20000u;
    for (i = 0; i < 12; i++) {
        t += 8681u;
        b->rx[b->rx_len] = f[i];
        b->rx_at[b->rx_len++] = t;
    }
    s_gf.busy = t;
}

/* 10 ms 하나: 차 모형으로 다음 자세를 정하고 그 사이 메인 루프를 10 us 마다 돈다 */
static void gf_10ms(void)
{
    int i;
    s_gf.hx[0] = s_gf.hx[1]; s_gf.hy[0] = s_gf.hy[1]; s_gf.hth[0] = s_gf.hth[1]; s_gf.ht[0] = s_gf.ht[1];
    s_gf.hx[1] = s_v.x; s_gf.hy[1] = s_v.y; s_gf.hth[1] = s_v.th; s_gf.ht[1] = host.cycles;
    plant();
    s_gf.hx[2] = s_v.x; s_gf.hy[2] = s_v.y; s_gf.hth[2] = s_v.th; s_gf.ht[2] = host.cycles + 1000000u;
    for (i = 1; i <= 1000; i++) {
        host.cycles += 1000u;
        if ((i % 200) == 0) { sys_tick_2ms(); }
        sys_scan();
    }
}

static void gf_key(uint16_t k)
{
    int i;
    host.key = k;
    for (i = 0; i < 10; i++) { gf_10ms(); }
    host.key = 0;
    for (i = 0; i < 10; i++) { gf_10ms(); }
}

/* 결과: 0 끝까지 감, 시험 시간 ms, 끝 자세 각(도, 360 의 배수에서 뺀 값) */
static void gs_flow(float p_ms, float tape_mm, int tns, int line, uint32_t seed, int16_t *gsp, float *t_ms, float *end_deg)
{
    static const case_t c0 = { 32, 0.0f, 0.0f, 1, 20, 20, 30, 0.15f, 0, 0 };
    static case_t c;
    uint32_t t0;
    int t;

    c = c0;
    memset(&s_v, 0, sizeof(s_v));
    memset(&s_gf, 0, sizeof(s_gf));
    s_c = &c;
    s_gd = 250.0f;
    s_x30w = 0.0f;
    s_gseed = seed;
    s_gf.p = (p_ms > 0.0f) ? p_ms * (1.0f + (gs_rand() * 2.0f - 1.0f) * 0.005f) : 0.0f;
    s_gf.ph = gs_rand() * p_ms * 100000.0f;
    s_gf.tape = tape_mm;
    s_gf.line = line;
    host_reset();
    host.on_send = gf_send;
    memset(&D, 0, sizeof(D));
    memset(seq_request_flag, 0, sizeof(seq_request_flag));
    host.key = KEY_MODE + KEY_SET;      /* 튜닝 모드로 켠다 */
    sys_boot();
    host.adc = 300u << 4;
    for (t = 0; t < 60; t++) { gf_10ms(); }
    assert(sys_power_on() && tune_active());
    host.key = 0;
    assert(param_set_ram(P_GEAR_RATIO, 60) == 0);
    assert(param_set_ram(P_WHEEL_BASE_MM, (int16_t)(WB_MM / 10.0f)) == 0);
    assert(param_set_ram(P_GUIDE_DIST_MM, 250) == 0);
    assert(param_set_ram(P_TURN_FORBACK, 10) == 0);
    assert(param_set_ram(P_SPEED_TURN, (int16_t)tns) == 0);
    assert(param_set_ram(P_DCC, 40) == 0);
    assert(param_set_ram(P_WHEEL_BASE_MM, (int16_t)(WB_MM / 10.0f)) == 0);
    tune_turn_scale_set((int16_t)(TURN_SCALE * 1000.0f + 0.5f));
    for (t = 0; t < 120; t++) { gf_10ms(); }   /* EE 표시 1 s */
    gf_key(KEY_DOWN);                   /* t1 → t3 */
    gf_key(KEY_SET);                    /* t3.1 */
    gf_key(KEY_DOWN);                   /* t3.1 → t3.9 → t3.8 */
    gf_key(KEY_DOWN);
    assert(tune_menu() == 3080);
    t0 = host.cycles;
    host.bus_in[BUS_IN_A] &= (uint16_t)~RAW_START;
    gf_10ms();
    host.bus_in[BUS_IN_A] |= RAW_START;
    for (t = 0; (t < 3000) && (tune_state() != TUNE_GS_DONE); t++) {
        s_m.wbx = 1.0f;
        gf_10ms();
    }
    assert(tune_state() == TUNE_GS_DONE);
    *t_ms = (float)(host.cycles - t0) / 100000.0f;
    *gsp = agv_gs.gsp;
    {
        float d = fmodf(-s_v.th * 180.0f / PI_F, 360.0f);
        if (d > 180.0f) { d -= 360.0f; }
        *end_deg = d;
    }
    assert(!gscan_fast() && !gresp_busy());
    host.on_send = 0;
}

/* ---- 시퀀스 MOVE_MM 모의: 서 있다 시퀀스 요청 → A → mm 이동·정지 → B. 직선 테이프, 바퀴 지연 0.1 s.
 * obs_at 0 이 아니면 그 mm 를 지날 때 앞 장애물을 1 s 켰다 끈다. lose_at 0 이 아니면 그 mm 뒤 선이 없다 */
typedef struct { float run_mm, vmax_mpm, t_s; int b_early, done, stop_err, seq_on, paused; float x_obs; } mv_res_t;
static void wt_tick(int reply);

static void move_flow(int mm, int back, int obs_at, int lose_at, mv_res_t *r)
{
    static case_t c;
    case_t c0 = { 32, 0.0f, 0.0f, 1, 25, 25, 30, 0.1f, 0, 0 };
    float x0;
    int t, obs_t = 0, moving = 0;

    c = c0;
    memset(r, 0, sizeof(*r));
    memset(&s_v, 0, sizeof(s_v));
    s_c = &c;
    s_gd = 250.0f;
    s_x30w = 0.0f;
    s_cross_w = 0.0f;
    s_v.s = GD_MM;
    host_reset();
    memset(&D, 0, sizeof(D));
    memset(seq_request_flag, 0, sizeof(seq_request_flag));
    sys_boot();
    host.adc = 300u << 4;
    for (t = 0; t < 60; t++) { scan(); }
    assert(sys_power_on());
    assert(param_set_ram(P_GEAR_RATIO, 60) == 0);
    assert(param_set_ram(P_WHEEL_BASE_MM, (int16_t)(WB_MM / 10.0f)) == 0);
    assert(param_set_ram(P_GUIDE_DIST_MM, 250) == 0);
    assert(param_set_ram(P_SPEED_HIGH, 30) == 0);
    assert(param_set_ram(P_SPEED_LOW, 25) == 0);
    for (t = 0; t < 30; t++) { wt_tick(1); }
    if (back) { run_relay(1, 0); }
    s_mv_steps[1].arg0 = (uint16_t)mm;
    seq_request(SEQ_SITE_BASE);
    x0 = s_v.x;
    for (t = 0; (t < 6000) && !r->done; t++) {
        float u = 0.0f, slant = 1.0f, run = fabsf(s_v.x - x0);
        int seen = cross(&u, &slant);
        if (seen && (fabsf(u) > (float)s_c->leds * PITCH_MM / 2.0f + TAPE_MM)) { seen = 0; }
        if (lose_at && (run > (float)lose_at)) { seen = 0; }
        if (obs_at && (run > (float)obs_at) && (obs_t < 100)) { host.bus_in[BUS_IN_B] &= (uint16_t)~0x02u; obs_t++; r->paused |= (D.action_err == ERR_OBSTACLE); }
        else { host.bus_in[BUS_IN_B] |= 0x02u; }
        sensor_reply(u, slant, seen);
        plant();
        scan();
        if (D.move_on) { moving = 1; }
        if ((io_out_mask() & (1UL << OUT_EXT_3)) && ((s_v.rpm[0] > 20.0f) || (s_v.rpm[1] > 20.0f) || D.flag_run)   /* 모형 바퀴는 지수로 줄어 멈춤 판정 뒤에도 10 rpm 안팎(0.1 m/min) 남는다 */) { if (!r->b_early && getenv("MV_DBG")) { printf("    B 켜짐 rpm %.1f %.1f run %d\n", s_v.rpm[0], s_v.rpm[1], D.flag_run); } r->b_early = 1; }
        if (moving) {
            float v = (float)((D.vin_out >= 0) ? D.vin_out : -D.vin_out) / P.one_meter_rpm;
            if (v > r->vmax_mpm) { r->vmax_mpm = v; }
            r->t_s += 0.01f;
        }
        if ((io_out_mask() & (1UL << OUT_EXT_3)) || D.stop_err) { r->done = 1; }
    }
    if (getenv("MV_DBG")) {
        printf("    dbg move_on %d stopping %d run %d ready %d seq %d step %d phase %d reason %d ctrl %d err %d/%d/%d last %d over %.1f tgt %.1f\n", D.move_on, D.flag_stopping,
               D.flag_run, D.flag_ready, seq_is_active(), seq_active_step(), stop_phase(), stop_reason(), D.control, D.system_err, D.stop_err, D.action_err,
               stop_last.reason, stop_last.overshoot_mm, stop_last.target_mm);
    }
    for (t = 0; t < 30; t++) { wt_tick(1); }
    r->run_mm = fabsf(s_v.x - x0);
    r->stop_err = D.stop_err;
    r->seq_on = seq_is_active();
}

/* 모형(바퀴 1차 지연 0.1 s)에서 정지 제어가 목표를 1.8~5.3 mm 지나친다(속도에 비례, 바퀴 지연 몫). 회귀 기준은 7 mm */
#define MV_TOL 7.0f
static int move_table(void)
{
    static const int dist[6] = { 5, 20, 50, 100, 300, 1000 };
    int i, bad = 0;
    mv_res_t r;
    printf("  MOVE_MM 거리 | 정지 오차 mm | 최고 m/min | 시간 s\n");
    for (i = 0; i < 6; i++) {
        move_flow(dist[i], 0, 0, 0, &r);
        printf("  %8d | %6.1f | %5.1f | %4.2f%s\n", dist[i], r.run_mm - (float)dist[i], r.vmax_mpm, r.t_s,
               (r.b_early || !r.done || (fabsf(r.run_mm - (float)dist[i]) > MV_TOL)) ? "  <-- 벗어남" : "");
        if (r.b_early || !r.done || (fabsf(r.run_mm - (float)dist[i]) > MV_TOL)) { bad++; }
    }
    move_flow(100, 1, 0, 0, &r);
    printf("  후진 100: 오차 %.1f mm, %.2f s\n", r.run_mm - 100.0f, r.t_s);
    if (r.b_early || !r.done || (fabsf(r.run_mm - 100.0f) > MV_TOL)) { bad++; }
    move_flow(300, 0, 100, 0, &r);
    printf("  장애물(100 mm 에서 1 s) 300: 오차 %.1f mm, 멈춤 %d, %.2f s\n", r.run_mm - 300.0f, r.paused, r.t_s);
    if (r.b_early || !r.done || !r.paused || (fabsf(r.run_mm - 300.0f) > MV_TOL)) { bad++; }
    move_flow(1000, 0, 0, 200, &r);
    printf("  선 잃음(200 mm 뒤) 1000: 오류 %d, 시퀀스 %d, B 켜짐 %d, 간 거리 %.0f mm\n", r.stop_err, r.seq_on,
           (int)((io_out_mask() >> OUT_EXT_3) & 1u), r.run_mm);
    if ((r.stop_err == 0) || r.seq_on || ((io_out_mask() >> OUT_EXT_3) & 1u)) { bad++; }
    return bad;
}

/* ---- t3.9 WT 전 과정 모의: 튜닝 메뉴 → START → LOS 정속 → 짝 펄스 → 복귀 → 정지. 직선 테이프, 10 ms 마다 센서 응답·차 모형·스캔.
 * 차 모형 바퀴는 지령을 1차 지연 tau 로 따라간다(순수 지연 없음, 가감속 한계 ACC_LIM). 엔코더는 12펄스/회전 그대로 */
typedef struct { float dev_max, dev_end, run_mm, t_s; } wt_res_t;

static void wt_tick(int reply)
{
    float u = 0.0f, slant = 1.0f;
    int seen = cross(&u, &slant);
    if (seen && (fabsf(u) > (float)s_c->leds * PITCH_MM / 2.0f + TAPE_MM)) { seen = 0; }
    if (reply) { sensor_reply(u, slant, seen); }
    plant();
    scan();
}

static void wt_key(uint16_t k)
{
    int i;
    host.key = k;
    for (i = 0; i < 10; i++) { wt_tick(0); }
    host.key = 0;
    for (i = 0; i < 10; i++) { wt_tick(0); }
}

static void wt_flow(float tau, int los, int leds, float kick, wt_res_t *r)
{
    static case_t c;
    case_t c0 = { 32, 0.0f, 0.0f, 1, 20, 20, 30, 0.1f, 0, 0 };
    float x0;
    int t, ph = 0;

    c = c0;
    c.tau = tau;
    c.los = los;
    c.leds = leds;
    memset(r, 0, sizeof(*r));
    memset(&s_v, 0, sizeof(s_v));
    s_c = &c;
    s_gd = 250.0f;
    s_x30w = 0.0f;
    s_cross_w = 0.0f;
    s_v.s = GD_MM;
    host_reset();
    memset(&D, 0, sizeof(D));
    memset(seq_request_flag, 0, sizeof(seq_request_flag));
    host.key = KEY_MODE + KEY_SET;      /* 튜닝 모드로 켠다 */
    sys_boot();
    host.adc = 300u << 4;
    for (t = 0; t < 60; t++) { scan(); }
    assert(sys_power_on() && tune_active());
    host.key = 0;
    assert(param_set_ram(P_GEAR_RATIO, 60) == 0);
    assert(param_set_ram(P_WHEEL_BASE_MM, (int16_t)(WB_MM / 10.0f)) == 0);
    assert(param_set_ram(P_GUIDE_DIST_MM, 250) == 0);
    assert(param_set_ram(P_SPEED_HIGH, 30) == 0);
    assert(param_set_ram(P_SPEED_LOW, (int16_t)los) == 0);
    tune_turn_scale_set((int16_t)(TURN_SCALE * 1000.0f + 0.5f));
    for (t = 0; t < 120; t++) { scan(); }   /* EE 표시 1 s */
    wt_key(KEY_DOWN);                   /* t1 → t3 */
    wt_key(KEY_SET);                    /* t3.1 */
    wt_key(KEY_DOWN);                   /* t3.1 → t3.9 */
    assert(tune_menu() == 3090);
    host.bus_in[BUS_IN_A] &= (uint16_t)~RAW_START;
    wt_tick(0);
    host.bus_in[BUS_IN_A] |= RAW_START;
    x0 = s_v.x;
    for (t = 0; (t < 3000) && (tune_state() != TUNE_WT_DONE); t++) {
        float dev;
        if (agv_wt.phase == 2) { s_v.y -= kick; kick = 0.0f; }   /* 측정 시작에 옆으로 민다(실패 시험) */
        wt_tick(1);
        dev = fabsf(-s_v.y - GD_MM * sinf(s_v.th));   /* 센서 자리 옆 편차 mm(테이프 y = 0) */
        if ((agv_wt.phase == 2) && (dev > r->dev_max)) { r->dev_max = dev; }
        if ((ph == 3) && (agv_wt.phase == 4)) { r->dev_end = dev; }
        ph = agv_wt.phase;
    }
    assert(tune_state() == TUNE_WT_DONE);
    r->t_s = (float)t * 0.01f;
    r->run_mm = s_v.x - x0;
    assert(steer_test_rpm == 0);
}

/* 모형 tau 50·100·200·300 ms: 잰 τ 가 ±20%(또는 ±20 ms) 안, 순수 지연 -5~20 ms */
static int wt_table(int los)
{
    static const int taus[4] = { 50, 100, 200, 300 };
    int i, bad = 0;
    printf("  WT LOS %d m/min, A %d rpm, T %d ms\n", los, WT_A_RPM, WT_T_TICKS * 10);
    printf("  tau모형 |  TL  TR |  DL  DR | 계단(L R) | 옆편차 최대 끝 mm | 직선 mm | 시간 s\n");
    for (i = 0; i < 4; i++) {
        wt_res_t r;
        int k, ok;
        float tol;
        wt_flow((float)taus[i] * 0.001f, los, 32, 0.0f, &r);
        tol = (taus[i] * 0.2f > 20.0f) ? taus[i] * 0.2f : 20.0f;
        ok = (agv_wt.state == 2) && (D.system_err == 0) && (D.stop_err == 0);
        for (k = 0; k < 2; k++) {
            int16_t tm = k ? agv_wt.wtr : agv_wt.wtl, dl = k ? agv_wt.wdr : agv_wt.wdl;
            ok = ok && (fabsf((float)tm - (float)taus[i]) <= tol) && (dl >= -5) && (dl <= 20);
        }
        printf("  %6d  | %3d %3d | %3d %3d | %d %d | %5.1f %5.1f | %5.0f | %4.1f %s\n", taus[i], agv_wt.wtl, agv_wt.wtr,
               agv_wt.wdl, agv_wt.wdr, agv_wt.used[0], agv_wt.used[1], r.dev_max, r.dev_end, r.run_mm, r.t_s, ok ? "" : "<-- 벗어남");
        printf("          계단별 τ L %d %d %d R %d %d %d, 지연 L %d %d %d R %d %d %d\n",
               agv_wt.tau[0][0], agv_wt.tau[0][1], agv_wt.tau[0][2], agv_wt.tau[1][0], agv_wt.tau[1][1], agv_wt.tau[1][2],
               agv_wt.dead[0][0], agv_wt.dead[0][1], agv_wt.dead[0][2], agv_wt.dead[1][0], agv_wt.dead[1][1], agv_wt.dead[1][2]);
        bad += !ok;
    }
    return bad;
}
#endif

/* 출발 정렬 표(start): 실차 값 ACC 58, DCC 60, LOS 25, DRS 31, 휠 600, GD 250. 왼쪽 곡선(dir 1), 곡선 위는 접선 방향.
 * 출발 자리 × 센서 32·16 × 조향 37차·추정(AGV_DEV_TESTS) × 바퀴 지연 0.1·0.2·0.3 s. 예전(start_align 0 강제) 대 이 빌드 */
/* 곡선 위: 회전 중심이 테이프 위(센서는 바깥으로 R - √(R² - GD²) 벗어남, 손으로 놓은 자리), "추종"은 따라가다 선 자세
 * (센서가 테이프 위, 회전 중심은 안쪽 반경 √(R² - GD²)). 일반 센서는 R551 이하에서 중심이 테이프 위면 선이 안 보인다 */
#define SA_N    16
#define SA_NONE 15      /* 선 없는 곳(옆 40단위: wide 반폭 32단위 밖) */
static const struct { float r, deg, frac, off, th; int on; const char *name; } s_sa_pos[SA_N] = {
    { 0, 0, 0, 5, 0, 0, "옆5" }, { 0, 0, 0, 10, 0, 0, "옆10" }, { 0, 0, 0, 15, 0, 0, "옆15" }, { 0, 0, 0, 0, 5, 0, "방향5" }, { 0, 0, 0, 0, 10, 0, "방향10" },
    { 450, 180, 0.5f, 0, 0, 0, "R450/180중" }, { 500, 180, 0.5f, 0, 0, 0, "R500/180중" }, { 551, 180, 0.5f, 0, 0, 0, "R551/180중" },
    { 650, 180, 0.5f, 0, 0, 0, "R650/180중" }, { 909, 180, 0.5f, 0, 0, 0, "R909/180중" }, { 551, 45, 0.5f, 0, 0, 0, "R551/45중" },
    { 650, 45, 0.5f, 0, 0, 0, "R650/45중" }, { 551, 90, 1.0f / 3.0f, 0, 0, 0, "R551/90앞⅓" },
    { 450, 180, 0.5f, 0, 0, 1, "R450/180추종" }, { 551, 180, 0.5f, 0, 0, 1, "R551/180추종" }, { 0, 0, 0, 40, 0, 0, "선없음" } };
#define SA_R551(k) (((k) == 7) || ((k) == 10) || ((k) == 12) || ((k) == 14))
typedef struct { float yaw, izero, iclip, dev, align_mm, align_s, end_mm, end_s, t1m, vmax; int lost, err, risk, vflip; uint32_t hash; } sa_res_t;
typedef struct { int n, c1, c2, c4; float c3, yaw_worst, iz_worst; int c5, clip, r551_ok[2]; } sa_sum_t;

static void sa_run(int k, int leds, int alg, float tau, int old, sa_res_t *o)
{
    res_t r;
    case_t c = { leds, s_sa_pos[k].r, s_sa_pos[k].deg, 1, 25, 25, 31, tau, 0, 0, alg };
    memset(&s_sa, 0, sizeof(s_sa));
    s_sa.on = 1;
    s_sa.old = old;
    s_sa.off_u = s_sa_pos[k].on ? -(c.r_mm - sqrtf(c.r_mm * c.r_mm - GD_MM * GD_MM)) / GUIDE_UNIT_MM : s_sa_pos[k].off;
    s_sa.th_deg = s_sa_pos[k].th;
    s_sa.s0 = (c.r_mm > 0.0f) ? LEAD_MM + c.r_mm * c.deg * PI_F / 180.0f * s_sa_pos[k].frac : 0.0f;
    s_acc = 58;
    s_dcc = 60;
    sim(&c, &r);
    s_acc = s_dcc = 0;
    o->yaw = s_sa.yaw_max; o->izero = s_sa.izero; o->iclip = s_sa.iclip; o->dev = s_sa.dev_max; o->align_mm = s_sa.align_mm; o->align_s = s_sa.align_s;
    o->end_mm = s_sa.end_mm; o->end_s = s_sa.end_s; o->t1m = s_sa.t1m; o->vmax = s_sa.vmax; o->vflip = s_sa.vflip;
    o->lost = r.lost; o->risk = r.risk; o->err = D.stop_err ? D.stop_err : D.system_err; o->hash = r.hash;
    memset(&s_sa, 0, sizeof(s_sa));
}

static void sa_line(const char *who, int k, int leds, int alg, int tau_ms, const sa_res_t *o)
{
    printf("SA %-11s L%2d %s %3d %s | 잃음 %4d 오류 %3d | 회전율 %5.1f 안쪽0 %4.2f 잘림 %4.2f 편차 %5.1f | 확인 %5.0f %5.2f 끝 %5.0f %5.2f | 1m %5.2f 위험 %4d 바뀜 %d 최고 %4.1f\n",
           s_sa_pos[k].name, leds, alg ? "추정" : "37차", tau_ms, who, o->lost, o->err, o->yaw, o->izero, o->iclip, o->dev,
           o->align_mm, o->align_s, o->end_mm, o->end_s, o->t1m, o->risk, o->vflip, o->vmax);
}

/* 합격선(이 빌드 대 예전): c1 예전이 통과(잃음·오류 0)한 조건에서 선 잃음·오류, c2 회전율 최대·안쪽 바퀴 0 시간이 예전보다 나쁜 조건,
 * c3 옆 15·방향 10 회전율 최대 / 예전의 최댓값, c4 선 없는 곳에서 SPN 을 넘거나 오류로 서지 않은 조건, c5 1 m 안 속도 지령 방향 바뀜 최대,
 * clip 안쪽 바퀴 지령이 0 아래에서 잘린 시간이 예전보다 긴 조건. r551_bad: 예전이 통과한 R551 곡선 위 출발에서 이 빌드가 선을 잃거나
 * 오류(b0 wide, b1 일반), r551_ok: R551 곡선 위 출발에서 이 빌드가 통과한 조건 수(wide·일반) */
static void start_table(int print, sa_sum_t *s, int *r551_bad)
{
    static const int taus[3] = { 100, 200, 300 };
    int k, l, a, t;
    float spn = 0.0f;
    memset(s, 0, sizeof(*s));
    *r551_bad = 0;
    for (k = 0; k < SA_N; k++) {
        for (l = 32; l >= 16; l -= 16) {
            for (a = 0; a < (AGV_DEV_TESTS ? 2 : 1); a++) {
                for (t = 0; t < 3; t++) {
                    sa_res_t o, n;
                    int pass_o, pass_n;
                    sa_run(k, l, a, (float)taus[t] * 0.001f, 1, &o);
                    sa_run(k, l, a, (float)taus[t] * 0.001f, 0, &n);
                    spn = (float)P.min_speed / P.one_meter_rpm;
                    if (print) { sa_line("예전", k, l, a, taus[t], &o); sa_line("이것", k, l, a, taus[t], &n); }
                    pass_o = (o.lost == 0) && (o.err == 0);
                    pass_n = (n.lost == 0) && (n.err == 0);
                    s->n++;
                    if (pass_o && !pass_n) { s->c1++; }
                    if ((n.yaw > o.yaw) || (n.izero > o.izero + 0.001f)) { s->c2++; }
                    if (n.yaw - o.yaw > s->yaw_worst) { s->yaw_worst = n.yaw - o.yaw; }
                    if (n.izero - o.izero > s->iz_worst) { s->iz_worst = n.izero - o.izero; }
                    if (((k == 2) || (k == 4)) && (o.yaw > 0.0f) && (n.yaw / o.yaw > s->c3)) { s->c3 = n.yaw / o.yaw; }
                    if ((k == SA_NONE) && ((n.vmax > spn + 0.1f) || (n.err != ERR_GUIDE_RANGE) || (s_v.rpm[0] + s_v.rpm[1] >= 1.0f))) { s->c4++; }
                    if (n.vflip > s->c5) { s->c5 = n.vflip; }
                    if (n.iclip > o.iclip + 0.001f) { s->clip++; }
                    if (SA_R551(k) && pass_o && !pass_n) { *r551_bad |= (l == 32) ? 1 : 2; }
                    if (SA_R551(k) && pass_n) { s->r551_ok[(l == 32) ? 0 : 1]++; }
                }
            }
        }
    }
    printf("  출발 정렬 %d조건: 예전 통과·이것 실패 %d, 회전율·안쪽0 나빠짐 %d(최악 +%.1f 도/s, +%.2f s), 0 아래 잘림 나빠짐 %d, 옆15·방향10 회전율 비 최대 %.2f,"
           " 선없음 SPN·오류 정지 아님 %d, 1 m 속도 방향 바뀜 최대 %d, R551 실패 %d(통과 wide %d 일반 %d)\n",
           s->n, s->c1, s->c2, s->yaw_worst, s->iz_worst, s->clip, s->c3, s->c4, s->c5, *r551_bad, s->r551_ok[0], s->r551_ok[1]);
}

#if AGV_DEV_TESTS
/* 53차 TST 표(곡선 진입 보강, param.c s_tst). 급곡선: 90도 반경 0.9 m, LOS·진입 30 m/min(위험 감속 목표 21 m/min), wide 32칸,
 * GD 250, 휠 600, 전진·왼쪽, 바퀴 지연은 인자(20·100 ms). 지표: 진입(센서 점이 곡선 앞 절반) |위치| 최대 단위, 곡선 편차 최대 mm,
 * 위험 감속 틱(들어간 횟수), 곡선 평균 속도 m/min, 곡선 정상 구간 바퀴 지령 차 흔들림 %·방향 바뀜/s, 출구 넘침 mm,
 * 곡선 안 출력 변화의 변화 50 rpm 넘음/s(계단), 선 잃음 틱. 넓힘: 반경 0.55~2.0 m × 25·30 m/min 10조건(최대, 틱·잃음은 합).
 * 직선: 옆 5단위 꺾임·끊김 + 40 mm 점(31 m/min)과 출발 정렬 5자리(옆 5·10·15, 방향 5·10, LOS 25)에서 TST 1 과 조향 출력 수열이
 * 같은지(= E·F 가 안 걸렸다)와 넘침·회전율. 모의는 실차와 여러 번 어긋났다: 번호끼리 상대 비교로만 */
static const float s_tr[5] = { 550.0f, 700.0f, 1000.0f, 1500.0f, 2000.0f };

static void tst_line(const char *name, float rmm, int v, const res_t *r)
{
    printf("  %-5s R%4.0f v%2d | %3d %6.1f %5d(%d) %4.1f | %5.1f %4.1f | %5.1f %4.1f | %4d %d\n", name, rmm, v, r->pin, r->max, r->risk,
           r->risk_n, r->v_avg, r->wob, r->wflip, r->eover, r->j_n50, r->lost, r->err);
}

static void tst_table(int detail, float tau)
{
    static const int vv[2] = { 25, 30 };
    static const int rows[7] = { 0, 1, 2, 3, 4, 5, 6 };
    uint32_t h0 = 2166136261u;
    int g, i, j;
    res_t r;
    printf("  TST   R    v  | 진입단위 편차mm 위험틱(횟수) 평균v | 흔들림%% 바뀜/s | 출구넘침 계단/s | 잃음 오류  (바퀴 지연 %.0f ms)\n", tau * 1000.0f);
    for (g = 0; g < 7; g++) {
        case_t c = { 32, 900.0f, 90.0f, 1, 30, 30, 31, tau, 0, 0, 0 };
        char nm[8];
        s_tst = rows[g];
        sim(&c, &r);
        if (rows[g] == 0) { h0 = (h0 ^ r.hash) * 16777619u; }
        snprintf(nm, sizeof(nm), "%d", rows[g]);
        tst_line(nm, c.r_mm, c.v_mpm, &r);
    }
    printf("  넓힘(반경 0.55~2.0 m × 25·30 m/min)\n");
    for (g = 0; g < 7; g++) {
        res_t m;
        char nm[8];
        memset(&m, 0, sizeof(m));
        m.v_avg = 1e9f;
        for (i = 0; i < 5; i++) {
            for (j = 0; j < 2; j++) {
                case_t c = { 32, s_tr[i], 90.0f, 1, vv[j], vv[j], 31, tau, 0, 0, 0 };
                s_tst = rows[g];
                sim(&c, &r);
                snprintf(nm, sizeof(nm), "%d", rows[g]);
                if (detail) { tst_line(nm, s_tr[i], vv[j], &r); }
                if (rows[g] == 0) { h0 = (h0 ^ r.hash) * 16777619u; }
                if (r.pin > m.pin) { m.pin = r.pin; }
                if (r.max > m.max) { m.max = r.max; }
                if (r.eover > m.eover) { m.eover = r.eover; }
                if (r.wob > m.wob) { m.wob = r.wob; }
                if (r.wflip > m.wflip) { m.wflip = r.wflip; }
                if (r.j_n50 > m.j_n50) { m.j_n50 = r.j_n50; }
                if (r.v_avg < m.v_avg) { m.v_avg = r.v_avg; }
                m.risk += r.risk;
                m.risk_n += r.risk_n;
                m.lost += r.lost;
                m.err += (r.err != 0);
            }
        }
        snprintf(nm, sizeof(nm), "%d", rows[g]);
        tst_line(nm, 0.0f, 0, &m);              /* 평균 v 는 가장 느린 조건 */
    }
    printf("  TST 0 출력 수열 해시 %08lx\n", (unsigned long)h0);
    printf("  직선: TST 1 과 같으면 '=', 다르면 넘침 단위(꺾임)·편차 단위(점)·회전율 도/s(출발)\n");
    {
        static const int sk[5] = { 0, 1, 2, 3, 4 };
        res_t b1[2], rr;
        sa_res_t s1[5], so;
        for (g = 1; g < 7; g++) {
            printf("  %d |", rows[g]);
            for (i = 0; i < 2; i++) {       /* 옆 5단위 꺾임, 끊김 100 mm + 40 mm 점 */
                case_t c = { 32, 0.0f, i ? 40.0f : 5.0f, 1, 31, 31, 31, tau, 0, i ? 3 : 2, 0 };
                s_tst = rows[g];
                sim(&c, (g == 1) ? &b1[i] : &rr);
                if (g == 1) { printf(" %s %4.2f", i ? "점" : "꺾임", i ? (float)b1[i].jump : b1[i].over); }
                else if (rr.hash == b1[i].hash) { printf(" %s    =", i ? "점" : "꺾임"); }
                else { printf(" %s %4.2f", i ? "점" : "꺾임", i ? (float)rr.jump : rr.over); }
            }
            for (i = 0; i < 5; i++) {       /* 출발 정렬 */
                s_tst = rows[g];
                sa_run(sk[i], 32, 0, tau, 0, (g == 1) ? &s1[i] : &so);
                if (g == 1) { printf(" | %s %5.1f", s_sa_pos[sk[i]].name, s1[i].yaw); }
                else if (so.hash == s1[i].hash) { printf(" | %s     =", s_sa_pos[sk[i]].name); }
                else { printf(" | %s %5.1f%s", s_sa_pos[sk[i]].name, so.yaw, (so.lost || so.err) ? "!" : ""); }
            }
            printf("\n");
        }
    }
    s_tst = s_ki = 0;
}
#endif

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
    if ((argc > 1) && (strcmp(argv[1], "turn") == 0))  { turn_table(argc > 2); return 0; }
    if ((argc > 1) && (strcmp(argv[1], "start") == 0)) {   /* 출발 정렬 표: 예전(start_align 0 강제) 대 이 빌드, 조건마다 두 줄 */
        sa_sum_t s;
        int bad;
        start_table(1, &s, &bad);
        return (s.c1 || s.c4) ? 1 : 0;
    }
    if ((argc > 10) && (strcmp(argv[1], "start1") == 0)) {  /* start1 r deg frac off th leds alg tau_ms old [trace] */
        case_t c = { atoi(argv[7]), (float)atof(argv[2]), (float)atof(argv[3]), 1, 25, 25, 31, (float)atoi(argv[9]) * 0.001f, 0, 0, atoi(argv[8]) };
        memset(&s_sa, 0, sizeof(s_sa));
        s_sa.on = 1; s_sa.old = atoi(argv[10]); s_sa.off_u = (float)atof(argv[5]); s_sa.th_deg = (float)atof(argv[6]);
        s_sa.s0 = (c.r_mm > 0.0f) ? LEAD_MM + c.r_mm * c.deg * PI_F / 180.0f * (float)atof(argv[4]) : 0.0f;
        s_acc = 58; s_dcc = 60; s_trace = (argc > 11);
        sim(&c, &r);
        printf("잃음 %d 오류 %d 회전율 %.1f 안쪽0 %.2f 잘림 %.2f 편차 %.1f 확인 %.0f(%.2f) 끝 %.0f(%.2f) 1m %.2f 위험 %d 바뀜 %d 최고 %.1f\n",
               r.lost, D.stop_err ? D.stop_err : D.system_err, s_sa.yaw_max, s_sa.izero, s_sa.iclip, s_sa.dev_max, s_sa.align_mm, s_sa.align_s,
               s_sa.end_mm, s_sa.end_s, s_sa.t1m, r.risk, s_sa.vflip, s_sa.vmax);
        return 0;
    }
    if ((argc > 6) && (strcmp(argv[1], "turn1") == 0)) {   /* turn1 GD leds tau_ms TNS DCC [slip] [trace] */
        turn_res_t tr;
        float sl = (argc > 7) ? (float)atof(argv[7]) : 1.0f;
        s_ttrace = (argc > 8);
        turn_sim((float)atoi(argv[2]), atoi(argv[3]), (float)atoi(argv[4]) * 0.001f, sl, atoi(argv[5]), atoi(argv[6]), &tr);
        turn_line(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), sl, atoi(argv[5]), atoi(argv[6]), &tr);
        return 0;
    }
#if AGV_DEV_TESTS
    if ((argc > 1) && (strcmp(argv[1], "gs") == 0))    { return gs_table((argc > 2) ? atoi(argv[2]) : 10, (argc > 3) ? 2 : 1) ? 1 : 0; }
    if ((argc > 4) && (strcmp(argv[1], "gsflow") == 0)) {  /* gsflow P_ms tape_mm TNS [line 0] */
        int16_t gsp; float tm, ed;
        gs_flow((float)atof(argv[2]), (float)atof(argv[3]), atoi(argv[4]), (argc > 5) ? atoi(argv[5]) : 1, 77u, &gsp, &tm, &ed);
        printf("GSFLOW P %s tape %s TNS %s: state %d GSN %d GSL %d GSP %d p100 %d R %d pairs %d req %u err %u nt %u | %.0f ms, 끝 각 %.2f도, 오류 %d/%d\n",
               argv[2], argv[3], argv[4], agv_gs.state, agv_gs.gsn, agv_gs.gsl, gsp, agv_gs.p100, agv_gs.r1000, agv_gs.pairs,
               agv_gs.req, agv_gs.err, agv_gs.nt, tm, ed, D.system_err, D.stop_err);
        return 0;
    }
    if ((argc > 1) && (strcmp(argv[1], "est") == 0)) {  /* est [자세히 0~2 [스캔 6.3 ms 1]] */
        est_table((argc > 2) ? atoi(argv[2]) : 1, (argc > 3) ? atoi(argv[3]) : 0);
        printf("GD 250 합격선 밖 %d, 잡음 나쁜 조건 %d\n", s_es[0][0].bad + s_es[0][1].bad, est_noise(1));
        return 0;
    }
    if ((argc > 1) && (strcmp(argv[1], "sc") == 0)) { sc_table((argc > 2) ? atoi(argv[2]) : 0); return 0; }   /* sc [자세히 1] */
    if ((argc > 1) && (strcmp(argv[1], "estsc") == 0)) { s_sct = 1; est_table(0, 0); s_sct = 0; return 0; }    /* est 표를 모두 켜고 */
    if ((argc > 8) && (strcmp(argv[1], "one2") == 0)) {  /* one2 alg GD 곡선(0~5, 직선은 6 + 모드, 모드 3·6 은 25 mm) v tau_ms leds scan [trace 0/1 [speed_curve 0/1]] */
        int i = atoi(argv[4]);
        case_t c = { atoi(argv[7]), 0.0f, 25.0f, 1, atoi(argv[5]), atoi(argv[5]), 31, (float)atoi(argv[6]) * 0.001f, 0, i - 6, atoi(argv[2]) };
        if (i < 6) { c.r_mm = s_cur6[i].r; c.deg = s_cur6[i].deg; c.dir = s_cur6[i].dir; c.mode = 0; }
        s_gd = (float)atoi(argv[3]);
        s_scan63 = atoi(argv[8]);
        s_trace = (argc > 9) && atoi(argv[9]);
        s_sct = (argc > 10) ? atoi(argv[10]) : 0;
        sim(&c, &r);
        show((i < 6) ? s_cur6[i].name : "직선", &c, &r);
        printf("  흔들림 %.1f%% 바뀜 %.1f/s 넘침 %.1f 끝 RMS %.2f 중심 %.1f\n", r.wob, r.wflip, r.eover, r.tail_rms, r.cerr);
        return 0;
    }
    if ((argc > 1) && (strcmp(argv[1], "move") == 0)) { return move_table() ? 1 : 0; }   /* 시퀀스 MOVE_MM 표 */
    if ((argc > 1) && (strcmp(argv[1], "kp") == 0)) {   /* kp: TST 6(K_P 60 고정)·고정 K_P 45·50 대 평소, 곡선 C 25 m/min wide, 지연 100·200·300 ms */
        static const int kps[5] = { 0, -1, 50, 45, 35 };    /* 0 평소 경사, -1 TST 6 */
        int k, d;
        printf("  K_P    tau | 편차최대 mm 선잃음틱 위험틱 오류 | 흔들림%% 바뀜/s 끝RMS\n");
        for (k = 0; k < 5; k++) {
            for (d = 100; d <= 300; d += 100) {
                case_t c = { 32, 0.0f, 0.0f, 1, 25, 25, 31, (float)d * 0.001f, 0, 0, 0 };
                c.r_mm = s_cur6[2].r; c.deg = s_cur6[2].deg; c.dir = s_cur6[2].dir;
                s_tst = (kps[k] < 0) ? 6 : 0;
                s_kp = (kps[k] > 0) ? kps[k] : 0;
                sim(&c, &r);
                printf("  %4d %4d | %6.1f %5d %5d %d | %5.1f %4.1f %5.2f%s\n", kps[k], d, r.max, r.lost, r.risk, r.err,
                       r.wob, r.wflip, r.tail_rms, (kps[k] < 0) ? "  (TST 6 = K_P 60)" : ((kps[k] == 0) ? "  (평소 경사)" : ""));
            }
        }
        s_tst = s_kp = 0;
        return 0;
    }
    if ((argc > 1) && (strcmp(argv[1], "tst") == 0)) {    /* tst [자세히 0/1 [바퀴 지연 ms(20)]]: 53차 TST 표 */
        tst_table((argc > 2) && atoi(argv[2]), (argc > 3) ? (float)atoi(argv[3]) * 0.001f : 0.02f);
        return 0;
    }
    if ((argc > 5) && (strcmp(argv[1], "tst1") == 0)) {    /* tst1 TST K_I R_mm v [tau_ms(20) [trace]]: TST 한 조건(K_I 0 = TST 그대로) */
        case_t c = { 32, (float)atof(argv[4]), 90.0f, 1, atoi(argv[5]), atoi(argv[5]), 31, 0.02f, 0, 0, 0 };
        if (argc > 6) { c.tau = (float)atoi(argv[6]) * 0.001f; }
        s_tst = atoi(argv[2]);
        s_ki = atoi(argv[3]);
        s_trace = (argc > 7);
        sim(&c, &r);
        tst_line("one", c.r_mm, c.v_mpm, &r);
        show("tst1", &c, &r);
        s_tst = s_ki = 0;
        return 0;
    }
    if ((argc > 1) && (strcmp(argv[1], "wt") == 0)) {   /* wt [tau_ms [LOS [leds [kick_mm]]]] */
        wt_res_t wr;
        if (argc < 3) { return wt_table(20) ? 1 : 0; }
        wt_flow((float)atoi(argv[2]) * 0.001f, (argc > 3) ? atoi(argv[3]) : 20, (argc > 4) ? atoi(argv[4]) : 32, (argc > 5) ? (float)atof(argv[5]) : 0.0f, &wr);
        printf("WT tau %s: state %d fail %d TL %d TR %d DL %d DR %d 계단 %d %d n %d 옆 최대 %.1f 끝 %.1f mm 직선 %.0f mm %.1f s v %d\n",
               argv[2], agv_wt.state, agv_wt.fail, agv_wt.wtl, agv_wt.wtr, agv_wt.wdl, agv_wt.wdr, agv_wt.used[0], agv_wt.used[1],
               agv_wt.n, wr.dev_max, wr.dev_end, wr.run_mm, wr.t_s, agv_wt.v_rpm);
        return 0;
    }
#endif
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
            turn_sim((i & 1) ? 500.0f : 250.0f, (i & 2) ? 16 : 32, 0.15f, (i & 1) ? 0.8f : 1.25f, 13, 40, &tr);
            printf("  정렬 회전 GD %d L%d: 가운데 %.0f rpm, 선 기준 각 %.2f도, 보정 %d, %.2f s\n", (i & 1) ? 500 : 250, (i & 2) ? 16 : 32,
                   tr.v_center, tr.line_deg, tr.fix, tr.t_turn);
            /* v0.38.0: GD 500·슬립 0.8 은 라인 전 엔코더 끝점으로 감속해 보정 2번(지시 조건 GD 250 108 + 12 는 0) */
            assert(tr.done && (tr.err == 0) && (tr.v_center <= (float)stop_min_rpm()) && (tr.reacc == 0) && (fabsf(tr.line_deg) < 1.0f) && (tr.fix <= ((i & 1) ? 2 : 0)));
            assert((tr.const_s == 0.0f) && (tr.floor_s <= 1.0f) && (tr.drop_n <= 1));   /* v0.38.0: 감속 시작 뒤 정속 없음, 칸 바뀜에 감속이 안 꺼짐. 저크 2.4: 출발이 달라 끝 감속이 한 번 20% 넘게 준다(GD 250 L16) */
        }
    }
#if AGV_DEV_TESTS
    {   /* t3.8 GS: 추정(시드 3 × TNS 15·25 × 테이프 25·50 mm)과 전 과정(메뉴 → START → 360도 정렬 회전 → 계산) */
        int16_t gsp; float tm, ed;
        assert(gs_table(3, 1) == 0);
        gs_flow(5.0f, 25.0f, 15, 1, 77u, &gsp, &tm, &ed);
        printf("  GS 전 과정 P 5 ms: GSN %d GSL %d GSP %d, %.1f s, 끝 각 %.2f도\n", agv_gs.gsn, agv_gs.gsl, gsp, tm / 1000.0f, ed);
        assert((agv_gs.state == 2) && (abs(gsp - 50) <= 1) && (fabsf(ed) < 1.0f) && (D.system_err == 0) && (D.stop_err == 0));
        {   /* 결과 쪽 첫 칸 GN + 세 자리 */
            uint16_t d[5];
            ui_digits(d);
            assert((d[0] == 11u + ('G' - 'A')) && (d[1] == 11u + ('N' - 'A')) && (d[2] == (uint16_t)(agv_gs.gsn / 100))
                   && (d[3] == (uint16_t)((agv_gs.gsn / 10) % 10)) && (d[4] == (uint16_t)(agv_gs.gsn % 10)));
        }
        gs_flow(0.0f, 25.0f, 15, 1, 78u, &gsp, &tm, &ed);
        assert((agv_gs.state == 2) && (gsp == TUNE_NO_DATA) && (agv_gs.gsn > 50) && (fabsf(ed) < 1.0f));
        gs_flow(5.0f, 25.0f, 15, 0, 79u, &gsp, &tm, &ed);     /* 라인 없음: 돌지 않고 바로 없음 */
        assert((agv_gs.state == 3) && (gsp == TUNE_NO_DATA) && (tm < 200.0f) && (fabsf(ed) < 0.01f));
    }
    {   /* 추정 조향(steer_alg_test 1) 대 37차: 곡선 6 × 12·25·31 m/min × 지연 100·200·300 ms × 센서 32·16 × GD 250·500.
         * 지시 합격선(GD 250, 조건마다: 잃음 0, 바뀜 ≤ 37차/2, 최대 ≤ 37차, 끝 RMS ≤ 37차 + 1 mm)은 다 맞추지 못했다(v46-est):
         * 108조건 중 바뀜 26·끝 RMS 14·잃음 1(일반 B 31 m/min 300 ms, 37차도 잃음)·최대 1(같은 조건). 아래는 지금 수준을 지키는 회귀다 */
        int l;
        est_table(0, 0);
        for (l = 0; l < 2; l++) {
            const est_sum_t *e = &s_es[0][l];
            assert((e->lost_new == 0) && (e->lost[1] <= e->lost[0]));             /* 37차가 가는 곳에서 선을 잃지 않는다 */
            assert((e->max[1] < 0.7f * e->max[0]) && (e->f_max <= 1));          /* 편차 최대 중앙 19.7 대 31.3 mm */
            assert(e->flip[1] <= 0.5f * e->flip[0]);                            /* 바뀜/s 중앙 0.9 대 2.6~2.8 */
            assert(e->wob[1] <= 0.5f * e->wob[0]);                              /* 흔들림 중앙 13.5 대 29 % */
            assert(e->tail[1] <= e->tail[0] + 1.0f);                            /* 끝 1 m RMS 중앙 2.6 대 2.6 mm */
            assert((e->risk[1] < e->risk[0]) && (e->bad <= 16));                /* 위험 감속 조건 11 대 27, 합격선 밖 15·16 */
        }
        assert(est_noise(0) == 0);          /* 끊김 + 바닥 반사 점, 30도 교차선 10·25 mm: 37차보다 나쁘지 않다 */
    }
    {   /* 반응형 곡선 속도(speed_curve_test 1): 완만한 곡선(R 1·1.5·4 m, 25 m/min)은 추정 조향과 함께 통과 시간 +5% 안·선 잃음 0,
         * 속도 지령이 오르내리지 않는다(방향 바뀜 많아야 5, 감속 뒤 재가속 1번이 정상). 급한 곡선(R 0.4~0.6 m, 31·37 m/min)
         * 선 잃음 조건은 끈 때보다 늘지 않는다. 37차 조향 일반 센서는 곡선 안 편차가 커서 완만한 곡선 통과가 +3~18% 느려진다 */
        int g, l, a;
        sc_table(0);
        for (g = 0; g < 2; g++) {
            for (l = 0; l < 2; l++) {
                const sc_sum_t *off = &s_sc_sum[1][g][l][1][0], *on = &s_sc_sum[1][g][l][1][1];
                assert((on->pass <= 1.05f * off->pass) && (on->lost == 0));
                for (a = 0; a < 2; a++) { assert((s_sc_sum[1][g][l][a][1].vflip <= 5) && (s_sc_sum[0][g][l][a][1].vflip <= 5)); }
                assert(s_sc_sum[0][g][l][1][1].lost <= s_sc_sum[0][g][l][1][0].lost);
            }
        }
    }
    {   /* t3.9 WT: 모형 tau 50·100·200·300 ms 전 과정, 결과 쪽, 측정 중 편차 넘침 → 복귀·정지·없음 */
        wt_res_t wr;
        uint16_t d[5];
        assert(wt_table(20) == 0);
        ui_digits(d);                   /* 첫 쪽 WTOK + 쓴 계단 수 */
        assert((d[0] == 11u + ('W' - 'A')) && (d[1] == 11u + ('T' - 'A')) && (d[2] == 11u + ('O' - 'A')) && (d[3] == 11u + ('K' - 'A')) && (d[4] == 3u));
        wt_flow(0.1f, 20, 16, 40.0f, &wr);
        printf("  WT 측정 중 40 mm 밀림(일반 센서): state %d fail %d, 편차 최대 %.1f mm, %.1f s\n", agv_wt.state, agv_wt.fail, wr.dev_max, wr.t_s);
        assert((agv_wt.state == 3) && (agv_wt.fail == 1) && (agv_wt.wtl == TUNE_NO_DATA) && (D.system_err == 0) && (D.stop_err == 0));
    }
#endif
#if AGV_DEV_TESTS
    assert(move_table() == 0);          /* 시퀀스 MOVE_MM: 거리 표, 후진, 장애물 멈춤·이어 감, 선 잃음 오류·시퀀스 중단 */
#endif
    {   /* 출발 정렬 단계(start 표, ACC 58·DCC 60·LOS 25, 지연 0.1~0.3 s): 예전(정렬 단계 없음)이 통과한 곳에서 선을 잃지 않는다.
         * v0.40 의 조향 상한 × 0.5 는 wide R450·500·551 곡선 위 출발에서 선을 잃었다(9조건). 선 없는 곳은 SPN 으로 가다 오류로 선다.
         * R551 곡선 위: wide 는 회전 중심이 테이프 위, 일반 센서는 따라가다 선 자세(중심이 테이프 위면 선이 안 보인다).
         * 안쪽 바퀴는 0 아래로 잘리지 않는다(상한 |v| × 1). 0 에 닿는 시간·곡선 위 1 m 회전율은 예전보다 길거나 클 수 있다(보고서) */
        sa_sum_t s;
        int bad;
        start_table(0, &s, &bad);
        assert((s.c1 == 0) && (bad == 0) && (s.r551_ok[0] > 0) && (s.r551_ok[1] > 0) && (s.clip == 0));
        assert(!AGV_START_ALIGN || (s.c4 == 0));       /* 정렬 단계가 없으면 선 없는 곳도 LOS 로 간다 */
    }
    printf("test_curve ok\n");
    return 0;
}
