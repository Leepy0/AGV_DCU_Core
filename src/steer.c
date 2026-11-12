/* steer.c - 조향. 위치형 PI, 조건부 적분, 칼만 변화율 항, 속도별 게인 스케줄 */
#include "agv/steer.h"
#include "agv/drive.h"
#include "agv/guide.h"
#include "agv/motor.h"
#include "agv/param.h"
#include "agv/steerlog.h"
#include "agv/stop.h"
#include "agv/tune.h"

#define D agv_drive
#define P param_dv

#define GUIDE_KP_MIN                20.0f
#define GUIDE_KP_MAX                130.0f
#define GUIDE_KP_SPEED_GAIN         15.0f
#define GUIDE_KP_DEV_THRESHOLD      12     /* QUIRK_STEER_STEP 만: 이 편차를 넘으면 Kp 가 한 번에 최대 */
#define GUIDE_KP_DEV_THRESHOLD_WIDE 20
/* 편차 4단위부터 Kp 를 올려 12단위에서 최대(일반·wide 공통). 위험 감속(편차 10단위) 중에도 건다.
 * 폐루프 곡선 모의(tests/test_curve.c, 반경 0.55~0.96 m, 31 m/min): 편차 최대 92~108 mm(선 잃음 포함) → 37~50 mm.
 * 예전(위험 감속 중 속도 스케줄 Kp 만·적분 지움, 문턱에서 한 번에 최대)은 QUIRK_STEER_STEP 에 남긴다 */
#define GUIDE_KP_RAMP_LO            4
#define GUIDE_KP_RAMP_HI            12
#define GUIDE_KI_PERIOD_SCALE       0.2f
#define GUIDE_KI_MIN                0.1f
#define GUIDE_DEADBAND_SPEED_GAIN   2.0f
#define GUIDE_DEADBAND_MIN          1
#define GUIDE_LINEAR_ZONE           8
#define GUIDE_LINEAR_ZONE_WIDE      20
#define GUIDE_SCHED_BOOST_CAP       2.0f
#define GUIDE_KF_DT                 0.02f     /* 호출 주기와 같아야 한다 */
#define GUIDE_KF_Q_POS              0.02f
#define GUIDE_KF_Q_RATE             0.15f
#define GUIDE_KF_Q_BOOST            20.0f
#define GUIDE_KF_R                  4.0f
#define GUIDE_SPAN_NORMAL           3         /* 'a' 소스의 커브 판정. 레거시 고정값 */
#define GUIDE_KP_CURVE_BOOST        1.3f
#define GUIDE_KP_CURVE_MAX          (GUIDE_KP_MAX * 1.3f)
#define GUIDE_DEADBAND_CURVE_SCALE  0.3f
#define GUIDE_KD_RATE               0.08f
#define STEER_GUIDE_REF_MM          250.0f    /* 이득을 맞춘 센서 거리 */
#define STEER_WB_REF_MM             600.0f    /* 이득을 맞춘 휠 간격 */
#define STEER_LIMIT_RPM             1500
/* 곡선 계단 완화. 'h' 위치는 1단위 계단이라 Kp 가 오르는 편차 4~12단위에서 칸이 바뀔 때마다 출력이 150~260 rpm 씩 뛰었다
 * (기울기 Kp + a·dKp/da). 10 ms 당 출력 변화를 50 rpm 으로 묶는다. 센서 반폭의 약 60% 넘게 벗어나면 선을 놓치지 않게 묶지 않는다.
 * 곡선 모의(반경 0.55~0.96 m, 31 m/min): 틱당 변화 상위 5% 167~193 → 50 rpm, 편차 최대 wide +4~14%·일반 +2~6%, 직선 응답 같음.
 * 일반 센서를 위험 감속 편차 위에서도 묶으면 속도 루프 지연 150 ms·반경 0.55 m 에서 선을 잃는다 */
#define STEER_SLEW_RPM              50.0f
#define STEER_SLEW_DEV_MAX          10        /* 일반 센서(반폭 16단위). motor.c GUIDE_RISK_THRESHOLD 와 같다 */
#define STEER_SLEW_DEV_MAX_WIDE     20        /* wide 센서(반폭 32단위) */
/* 곡선 끝 적분 감쇠. 편차 6단위 이상(곡선)을 지나 4단위 아래로 들어오면 곡선 동안 쌓인 적분을 10 ms 마다 0.95 배로 뺀다.
 * 그대로 두면 출구에서 반대쪽으로 넘어간 뒤 적분이 빠질 때까지 2 m 넘게 4단위쯤 비켜 달린다(모의 D: 넘침 19 → 3 mm) */
#define STEER_I_EXIT_ARM            6
#define STEER_I_EXIT_DECAY          0.95f
#define STEER_I_EXIT_DONE           5.0f      /* 이보다 작아지면 감쇠를 끝낸다 rpm */

static struct {
    float i_out;
    int16_t prev_control, prev_branch, prev_backward, prev_p_cmd;
    int16_t previous_speed;
    int16_t i_exit;         /* 곡선 편차를 지났다: 4단위 아래로 들어오면 적분을 뺀다 */
} s_steer;

#if AGV_DEV_TESTS
int16_t steer_test_rpm;
int16_t steer_alg_test;
float steer_i_out(void) { return s_steer.i_out; }   /* 조향 기록기(steerlog.c)가 읽는다 */

/* 각도·곡률 추정 조향(시험, steer_alg_test 1). 상태는 진행 방향 기준이고 부호는 가이드 위치와 같다(+ = 테이프가 왼쪽):
 *   y 회전 중심 옆 편차 mm, ψ 차체 방향 - 테이프 방향 rad(+ = 오른쪽으로 돌아 있다), κ 테이프 곡률 1/m(+ = 오른쪽으로 굽는다).
 * 예측(작은 각): y += Δs·ψ, ψ += Δθ - Δs·κ. Δs·Δθ 는 좌우 누적 펄스(motor_pulse_seq)의 정수 차분이라 각 오차가 쌓이지 않는다.
 * 측정: 센서 점 편차 z = y + GD·ψ = 위치 × GUIDE_UNIT_MM. 선 잃음·전부 켜짐(교차선)·가장자리·유지 추적이면 예측만 한다(κ 유지).
 * 관측기: 거리 영역 오차 특성다항식 (s + q)(s² + p s + p²) = s³ + a2 s² + a1 s + a0 이면
 *   l3 = -a0, l2 = a1 - a0·GD, l1 = a2 - l2·GD. 고칠 때 이득 = l × 이번 틱 이동 거리.
 * 제어: 차체 곡률 c = κ - kψ·ψ - ky·(y - y_ref). 거리 영역 y'' + kψ y' + ky y = 0 (ky = ωn², kψ = 2ζωn).
 *   바퀴 지령 차의 절반 = c × 유효 휠 간격(WW × 회전 보정비) × 속도 지령 / 2. κ 가 곡선의 정상 바퀴 차를 맡아 적분이 없다.
 * 정규화: p = EST_OBS_M · 2·250 / (250 + GD) · g (≈ √(250/GD)), ωn = EST_WN_M · g, q = p · (EST_KA + (1 - EST_KA)·b),
 *   g = b + (1 - b) · min(1, EST_V_REF / 속도), b = 0~1(거른 혁신이 EST_BOOST_R0 단위 넘거나 위치가 EST_BOOST_Z0 단위 넘으면 1 쪽).
 *   GD(모의 tests/test_curve.c est, GD 500 선 잃음 조건 wide/일반, 각 54): 극을 GD 와 무관하게 두면 l1·Δs 가 31 m/min 한 틱에
 *   약 4 라 이산 관측기가 발산했다(steer-est 81a5031). 둘 다 1/GD(거리/GD 좌표에서 같은 폐루프)면 바퀴 지연은 그대로라
 *   GD 500 이 너무 느려 33/37 조건에서 선을 잃었다. 관측기만 ≈1/√GD·제어 ωn 거리 기준 고정이 1/15 로 가장 나았다(37차 0/3).
 *   휠 간격: 바퀴 지령을 곡률에서 운동학으로 바꾸므로 이득에 따로 넣지 않는다.
 *   속도·b: 'h' 1단위 계단과 바퀴 지연(0.1~0.3 s)이 맞물린 한계 진동은 고리 이득에 비례한다. 정상 곡선(혁신·위치 작음)에서는
 *   시간 대역을 EST_V_REF 속도 때로 묶고 κ 극을 낮춘다. 곡선 진입·출구(혁신·위치 큼)에서는 대역을 되돌린다.
 *   b 는 이득만 고르고 불감대 뒤 적분처럼 상태를 쌓지 않는다.
 * 곡선 끝: 혁신이 κ 를 0 쪽으로 밀고 거른 혁신이 1단위 넘으면 κ 몫을 EST_K_REL 배로 고친다 */
#ifndef EST_OBS_M
#define EST_OBS_M       0.016f    /* 관측기 극 p 1/mm, GD 250 기준(16/m) */
#endif
#ifndef EST_KA
#define EST_KA          0.1f      /* 정상 곡선에서 관측기 실수 극 q / p */
#endif
#ifndef EST_WN_M
#define EST_WN_M        0.004f    /* 제어 ωn 1/mm(4/m) */
#endif
#ifndef EST_ZETA
#define EST_ZETA        1.0f
#endif
#ifndef EST_V_REF
#define EST_V_REF       12.0f     /* m/min. 이보다 빠르면 정상 곡선 대역을 이 속도의 시간 대역으로 묶는다. 0 = 묶지 않음 */
#endif
#ifndef EST_BOOST_R0
#define EST_BOOST_R0    0.8f      /* 거른 혁신(단위)이 이것을 넘으면 1단위 동안 대역을 되돌린다. 0 = 늘 되돌림 */
#endif
#ifndef EST_BOOST_Z0
#define EST_BOOST_Z0    1.5f      /* 위치(단위)가 이것을 넘으면 2단위 동안 대역을 되돌린다. 0 = 안 봄 */
#endif
#ifndef EST_K_REL
#define EST_K_REL       3.0f
#endif
#ifndef EST_C_LPF
#define EST_C_LPF       0.5f      /* 곡률 지령 1차 거르기 비(틱당). 펄스 1개 차의 ψ 계단(31 m/min 에서 출력 약 10 rpm)을 줄인다 */
#endif
#define EST_RA_LPF      0.3f      /* 혁신 크기 거르기 비(틱당) */
#define EST_JUMP        2.5f      /* 혁신이 이 단위를 넘으면 테이프가 옆으로 뛴 것으로 본다 */
#define EST_JUMP_MM     30.0f     /* 뛴 위치가 이 거리 넘게 이어지면 y 로 받는다. 짧으면(바닥 반사·교차선 끝) 버린다 */
#ifndef EST_SETTLE_MM
#define EST_SETTLE_MM   40.0f     /* 모의 30도 교차선 25 mm: 0 이면 2.6~3.0 mm, 20 이면 일부 2.6 mm, 40 이면 0 */
#endif
#define EST_K_MAX       5.0f      /* κ 상한 1/m(반경 0.2 m) */

static struct {
    uint16_t pulse[2];      /* 지난 틱 누적 펄스 */
    float    y, psi, k;
    float    c;             /* 거른 곡률 지령 1/m */
    float    jump_mm;       /* 옆으로 뛴 위치가 이어진 거리 */
    float    ra;            /* 거른 혁신 크기(단위) */
    float    settle_mm;     /* 추적이 돌아온 뒤 측정을 쓰지 않을 남은 거리 */
    int16_t  ran;           /* 1 = 지난 틱에 추정 조향을 돌렸다. 아니면 새로 잡는다(주행 시작·분기 끝·전환) */
    int16_t  restart;
    int16_t  back;
} s_est;

/* RUN_DRIVE·분기 밖. 37차 조향이 낸 출력을 덮어쓴다 */
static void est_run(int16_t limit)
{
    float sg = D.backward ? -1.0f : 1.0f;
    float wb = (float)param_get(P_WHEEL_BASE_MM) * 10.0f * tune_turn_scale();
    float gd = (P.guide_dist_mm > 0.0f) ? P.guide_dist_mm : STEER_GUIDE_REF_MM;
    float z = (float)D.guide_pos * GUIDE_UNIT_MM;
    uint16_t pl = motor_pulse_seq(0), pr = motor_pulse_seq(1);
    int16_t dl = (int16_t)(uint16_t)(pl - s_est.pulse[0]), dr = (int16_t)(uint16_t)(pr - s_est.pulse[1]);
    int16_t v_abs = (D.v_cmd >= 0) ? D.v_cmd : (int16_t)(-D.v_cmd);
    int16_t restart = s_est.restart || (s_est.back != D.backward);
    float p = EST_OBS_M * (2.0f * STEER_GUIDE_REF_MM) / (STEER_GUIDE_REF_MM + gd), wn = EST_WN_M, c, out;
    float fb = (EST_BOOST_R0 > 0.0f) ? (s_est.ra - EST_BOOST_R0) : 1.0f;     /* 혁신이 크면(곡선 진입·출구) 대역을 되돌린다 */

    if (EST_BOOST_Z0 > 0.0f) {      /* 위치가 커도(출구에서 반대로 넘어감 등) 되돌린다 */
        float fz = ((float)((D.guide_pos >= 0) ? D.guide_pos : -D.guide_pos) - EST_BOOST_Z0) * 0.5f;
        if (fz > fb) { fb = fz; }
    }
    if (fb > 1.0f) { fb = 1.0f; } else if (fb < 0.0f) { fb = 0.0f; }
    if ((EST_V_REF > 0.0f) && ((float)v_abs > EST_V_REF * P.one_meter_rpm)) {
        float g = fb + ((1.0f - fb) * (EST_V_REF * P.one_meter_rpm) / (float)v_abs);
        p *= g;
        wn *= g;
    }
    s_est.pulse[0] = pl;
    s_est.pulse[1] = pr;
    if (restart) {
        s_est.y = z;
        s_est.psi = 0.0f;
        s_est.k = 0.0f;
        s_est.jump_mm = 0.0f;
        s_est.ra = 0.0f;
        s_est.settle_mm = 0.0f;
    }
    else {
        float ds = (float)(dl + dr) * 0.5f * P.one_pulse_length;
        s_est.y += ds * s_est.psi;
        s_est.psi += (sg * (float)(dl - dr) * P.one_pulse_length / wb) - (ds * s_est.k * 0.001f);
        /* 선 잃음·교차선 동안과 그 뒤 EST_SETTLE_MM 은 예측만 한다(추적이 막 돌아온 응답은 다른 선 가장자리가 섞인다).
         * 다시 잡은 위치도 같은 혁신 검사를 거친다(끊김 안 바닥 반사를 새 선으로 받지 않게) */
        if (D.guide_all_off || D.guide_all_on || (D.guide_track != GUIDE_TRK_NORMAL)) { s_est.settle_mm = EST_SETTLE_MM; }
        else if (s_est.settle_mm > 0.0f) { s_est.settle_mm -= ds; }
        else {
            float r = z - s_est.y - (gd * s_est.psi), w = 1.0f;
            s_est.ra += EST_RA_LPF * ((((r >= 0.0f) ? r : -r) * (1.0f / GUIDE_UNIT_MM)) - s_est.ra);
            if ((r > EST_JUMP * GUIDE_UNIT_MM) || (r < -EST_JUMP * GUIDE_UNIT_MM)) {
                s_est.jump_mm += ds;
                if (s_est.jump_mm >= EST_JUMP_MM) { s_est.y += r; s_est.jump_mm = 0.0f; }
                w = 0.0f;
            }
            else { s_est.jump_mm = 0.0f; }
            {
                float q = p * (EST_KA + ((1.0f - EST_KA) * fb)), a2 = q + p, a1 = (p * p) + (q * p), a0 = q * p * p;    /* (s + q)(s² + p s + p²) */
                float l1 = a1 - (a0 * gd), dk;
                r *= ds * w;
                s_est.y += (a2 - (l1 * gd)) * r;
                s_est.psi += l1 * r;
                dk = -a0 * 1000.0f * r;
                if (((dk > 0.0f) != (s_est.k > 0.0f)) && (s_est.ra > 1.0f)) { dk *= EST_K_REL; }    /* 곡선 끝: 빨리 푼다 */
                s_est.k += dk;
                if (s_est.k > EST_K_MAX)       { s_est.k = EST_K_MAX; }
                else if (s_est.k < -EST_K_MAX) { s_est.k = -EST_K_MAX; }
            }
        }
    }
    s_est.back = D.backward;
    s_est.ran = 1;
    c = s_est.k - (2000.0f * EST_ZETA * wn * s_est.psi) - (1000.0f * wn * wn * (s_est.y - (sg * (float)(D.p_cmd + D.p_test) * GUIDE_UNIT_MM)));
    s_est.c = restart ? c : (s_est.c + (EST_C_LPF * (c - s_est.c)));
    out = sg * s_est.c * wb * 0.0005f * (float)v_abs;
    if (out > (float)limit)       { out = (float)limit; }
    else if (out < (float)-limit) { out = (float)-limit; }
    D.p_pi_out = out;
    D.add_speed_l = (int16_t)((int16_t)out + D.offset_kp);
    D.add_speed_r = (int16_t)(-(int16_t)out - D.offset_kp);
}

/* 반응형 곡선 속도 상한(시험, speed_curve_test 1). motor.c 가 주행 목표를 min(지령, 상한)으로 한다. 단위 rpm.
 * 곡률 κ: 추정 조향이 돌면 κ 추정값, 아니면 실측 좌우 rpm 차로 본 차체 곡률(거른 값).
 * (a) 곡률: 필요한 바퀴 차의 절반 |κ|·wb·v/2 ≤ SC_BETA × 조향 한계 1500 rpm,
 *     그리고 바퀴 지연 τ 동안 곡률을 못 따라간 옆 어긋남 |κ|(vτ)²/2 ≤ SC_DELAY_E × 센서 반폭.
 * (b) 편차: |위치|/반폭이 SC_D0 를 넘으면 낮추기 시작해 SC_D1 에서 LLOW.
 * 내릴 때는 바로, 올릴 때는 틱당 (지령 - LLOW)/SC_RISE_TICKS 씩(출구에서 들썩이지 않게). 감·가속은 주행 S커브가 한다.
 * 교차선·끊김(전부 켜짐·꺼짐·가장자리·유지) 동안은 직전 상한을 그대로 둔다 */
#define SC_BETA         0.7f
#define SC_DELAY_E      0.25f
#define SC_TAU          0.2f      /* s. 실차 바퀴 지연 가정(44차 회전 끝 τ 와 같음) */
#define SC_D0           0.25f
#define SC_D1           0.6f
#ifndef SC_RISE_TICKS
#define SC_RISE_TICKS   300.0f    /* 상한 되올림: (지령 - LLOW) 전체를 3 s */
#endif
#ifndef SC_HYS
#define SC_HYS          0.2f      /* 모의 완만 곡선 37차 일반 센서: 0 이면 속도 지령 방향 바뀜 최대 39, 0.2·3 s 면 3 */
#endif
#define SC_K_LPF        0.2f
#ifndef SC_D_LPF
#define SC_D_LPF        0.1f      /* 편차 비 거르기(틱당). 37차 곡선 안 위치가 칸마다 오르내려 상한이 들썩이지 않게 */
#endif

int16_t speed_curve_test;
static struct { float k, d, lim; } s_sc;

int16_t steer_curve_vlim(void)
{
    float cmd = (float)D.speed_cmd, lo = (float)P.llow_speed, half = D.guide_wide ? 32.0f : 16.0f;
    float wb = (float)param_get(P_WHEEL_BASE_MM) * 10.0f * tune_turn_scale();
    float k, t, d, r;

    if (lo > cmd) { lo = cmd; }
    if ((s_sc.lim <= 0.0f) || (s_sc.lim > cmd)) { s_sc.lim = cmd; }
    if (D.guide_all_off || D.guide_all_on || (D.guide_track != GUIDE_TRK_NORMAL)) { return (int16_t)s_sc.lim; }
    r = (float)(D.rpm_avg[0] + D.rpm_avg[1]);
    if (r > 100.0f) { s_sc.k += SC_K_LPF * ((2000.0f * (float)(D.rpm_avg[0] - D.rpm_avg[1]) / (r * wb)) - s_sc.k); }
    k = ((steer_alg_test == 1) && s_est.ran) ? s_est.k : s_sc.k;
    if (k < 0.0f) { k = -k; }
    t = cmd;
    if (k > 0.05f) {
        float v1 = SC_BETA * 1500.0f * 2000.0f / (k * wb), x;
        if (v1 < t) { t = v1; }
        /* x = (v2 / t)², v2 = √(2e/κ)/τ 를 rpm 으로. sqrtf(라이브러리 117 워드) 대신 t·√x 를 뉴턴 2번(위에서 다가가 1~3% 크다) */
        x = (2000.0f * SC_DELAY_E * half * GUIDE_UNIT_MM / k) * (0.0036f / (SC_TAU * SC_TAU)) * P.one_meter_rpm * P.one_meter_rpm / (t * t);
        if (x < 1.0f) {
            float q = 0.5f * (1.0f + x);
            t *= 0.5f * (q + (x / q));
        }
    }
    s_sc.d += SC_D_LPF * (((float)((D.guide_pos >= 0) ? D.guide_pos : -D.guide_pos) / half) - s_sc.d);
    d = s_sc.d;
    if (d > SC_D0) {
        float f = (d - SC_D0) / (SC_D1 - SC_D0), v3;
        if (f > 1.0f) { f = 1.0f; }
        v3 = cmd - (f * (cmd - lo));
        if (v3 < t) { t = v3; }
    }
    if (t < lo) { t = lo; }
    /* 지령-LLOW 의 SC_HYS 안 변화는 무시한다(편차↔속도 되먹임으로 상한이 오르내려 가감속이 되풀이되지 않게) */
    if (t < s_sc.lim - (SC_HYS * (cmd - lo))) { s_sc.lim = t; }
    else if ((t > s_sc.lim + (SC_HYS * (cmd - lo))) || (t >= cmd)) {
        s_sc.lim += (cmd - lo) * (1.0f / SC_RISE_TICKS);
        if (s_sc.lim > t) { s_sc.lim = t; }
    }
    return (int16_t)s_sc.lim;
}
#endif

void steer_reset(void)
{
    s_steer.i_out = 0.0f;
    s_steer.prev_control = -1;
    s_steer.prev_branch = 0;
    s_steer.prev_backward = -1;
    s_steer.prev_p_cmd = 0;
    s_steer.previous_speed = 0;
    s_steer.i_exit = 0;
#if AGV_DEV_TESTS
    s_est.ran = 0;
    s_sc.lim = 0.0f;
    s_sc.k = 0.0f;
    s_sc.d = 0.0f;
#endif
}

static int16_t popcount16(uint16_t v)
{
    int16_t n = 0;
    while (v != 0) {
        n = (int16_t)(n + (v & 1u));
        v >>= 1;
    }
    return n;
}

/* 'a' 소스(QUIRK_SRC_A)는 레거시 동작 그대로다. 'h' 소스(기본)는 테이프 폭에 맞춘 임계를 쓴다 */
static int16_t curve_detected(void)
{
    if (guide_source_led()) { return D.guide_span >= guide_curve_span(); }
    return popcount16(D.guide_led) > GUIDE_SPAN_NORMAL;
}

static float sched_ratio(int16_t v_abs)
{
    float r;
    if (v_abs <= P.low_speed) {
        int16_t span = (P.low_speed > P.min_speed) ? (int16_t)(P.low_speed - P.min_speed) : 1;
        r = (float)(v_abs - P.min_speed) / (float)span;
        if (r < 0.0f) { r = 0.0f; }
    }
    else {
        int16_t span = (P.target_speed > P.low_speed) ? (int16_t)(P.target_speed - P.low_speed) : 1;
        r = 1.0f + ((float)(v_abs - P.low_speed) / (float)span);
        if (r > GUIDE_SCHED_BOOST_CAP) { r = GUIDE_SCHED_BOOST_CAP; }
    }
    return r;
}

/* 등속도 모델. 측정치는 센서 위치값 그대로다 */
static void kalman_step(int16_t curve_now, float dt)
{
    if (D.guide_kf_init == 0) {
        D.kf_pos = (float)D.guide_pos;
        D.kf_rate = 0.0f;
        D.kf_p00 = D.kf_p11 = 1.0f;
        D.kf_p01 = 0.0f;
        D.guide_kf_init = 1;
    }
    else {
        float q_scale = curve_now ? GUIDE_KF_Q_BOOST : 1.0f;
        float q00 = GUIDE_KF_Q_POS * q_scale;
        float q11 = GUIDE_KF_Q_RATE * q_scale;
        float pos_pred = D.kf_pos + (D.kf_rate * dt);
        float rate_pred = D.kf_rate;
        float p00_pred = D.kf_p00 + (2.0f * dt * D.kf_p01) + (dt * dt * D.kf_p11) + q00;
        float p01_pred = D.kf_p01 + (dt * D.kf_p11);
        float p11_pred = D.kf_p11 + q11;

        if (!D.guide_all_off && (D.guide_track != GUIDE_TRK_HOLD)) {
            float y = (float)D.guide_pos - pos_pred;
            /* 가장자리로 잡은 위치는 한 칸쯤 떨린다. 덜 믿는다 */
            float s = p00_pred + ((D.guide_track == GUIDE_TRK_EDGE) ? (GUIDE_KF_R * 4.0f) : GUIDE_KF_R);
            float k0 = p00_pred / s;
            float k1 = p01_pred / s;
            D.kf_pos = pos_pred + (k0 * y);
            D.kf_rate = rate_pred + (k1 * y);
            D.kf_p00 = (1.0f - k0) * p00_pred;
            D.kf_p01 = (1.0f - k0) * p01_pred;
            D.kf_p11 = p11_pred - (k1 * p01_pred);
        }
        else {          /* 전 채널 소등·유지 주행: 예측만 반영하고 불확실성은 키운다 */
            D.kf_pos = pos_pred;
            D.kf_rate = rate_pred;
            D.kf_p00 = p00_pred;
            D.kf_p01 = p01_pred;
            D.kf_p11 = p11_pred;
        }
    }
}

/* 좁은·넓은 가이드 공통. 임계값만 다르다. 반환은 조향 입력 위치값.
 * 기본은 불감대가 없다(불감대 안 아무 데나 머물고 지령 이동이 폭만큼 더 갔다). 불감대는 QUIRK_STEER_STEP 에서만 */
static float gains_and_deadband(float pos, int16_t v_abs, float r, int16_t curve_now, int16_t tuning_type,
                                int16_t dev_threshold, int16_t linear_zone)
{
    float sen = pos;
    int16_t band;

    if (param_get(P_STEER_KP) == 0) {
        D.p_kp = (v_abs > 0) ? (GUIDE_KP_MIN + (GUIDE_KP_SPEED_GAIN * r)) : GUIDE_KP_MIN;
        if (D.p_kp > GUIDE_KP_MAX) { D.p_kp = GUIDE_KP_MAX; }
        {
            int16_t a = (D.guide_pos >= 0) ? D.guide_pos : (int16_t)(-D.guide_pos);
            if (param_quirk(QUIRK_STEER_STEP)) {    /* 예전 조향: 위험 감속 중에는 강제 최대 게인을 걸지 않는다 */
                if (!D.risk_stop && (a > dev_threshold)) { D.p_kp = GUIDE_KP_MAX; }
            }
            else if (a > GUIDE_KP_RAMP_LO) {
                float k = (float)(a - GUIDE_KP_RAMP_LO) / (float)(GUIDE_KP_RAMP_HI - GUIDE_KP_RAMP_LO);
                if (k > 1.0f) { k = 1.0f; }
                if (D.p_kp < GUIDE_KP_MAX) { D.p_kp += (GUIDE_KP_MAX - D.p_kp) * k; }
            }
        }
        if (curve_now) {
            D.p_kp *= GUIDE_KP_CURVE_BOOST;
            if (D.p_kp > GUIDE_KP_CURVE_MAX) { D.p_kp = GUIDE_KP_CURVE_MAX; }
        }
    }
    if (param_get(P_STEER_KI) == 0) {
        D.p_ki = (v_abs > 0) ? ((1.0f - r) * GUIDE_KI_PERIOD_SCALE) : (1.0f * GUIDE_KI_PERIOD_SCALE);
        if (D.p_ki < GUIDE_KI_MIN) { D.p_ki = GUIDE_KI_MIN; }
    }
    if (param_quirk(QUIRK_STEER_STEP) && ((tuning_type == 1) || (tuning_type == 2))) {
        band = (int16_t)(GUIDE_DEADBAND_SPEED_GAIN * r);
        if (band < GUIDE_DEADBAND_MIN) { band = GUIDE_DEADBAND_MIN; }
        if (curve_now) { band = (int16_t)((float)band * GUIDE_DEADBAND_CURVE_SCALE); }
        if ((sen <= (float)band) && (sen >= (float)-band))                { sen = 0.0f; }
        else if ((sen > 0.0f) && (sen < (float)linear_zone))              { sen -= (float)band; }
        else if ((sen < 0.0f) && (sen > (float)-linear_zone))             { sen += (float)band; }
    }
    return sen;
}

void steer_run(int16_t tuning_type)
{
    int16_t v_abs = (D.vin_out >= 0) ? D.vin_out : (int16_t)(-D.vin_out);
    float   r = sched_ratio(v_abs);
    int16_t curve_now = curve_detected() ? 1 : 0;
    int16_t clear_integral;
    float   sen, pos;
    /* 10 ms 조향: 칼만 dt 와 적분 이득을 주기에 맞춘다 */
    int16_t fast = !param_quirk(QUIRK_STEER_20MS);
    float   dt = fast ? (GUIDE_KF_DT * 0.5f) : GUIDE_KF_DT;
    float   i_scale = fast ? 0.5f : 1.0f;
#if AGV_DEV_TESTS
    s_est.restart = !s_est.ran;     /* 지난 틱에 추정 조향을 안 돌렸으면 새로 잡는다 */
    s_est.ran = 0;
#endif

    /* 위험 감속·재가속 중에는 커브 반응성 부스트를 전부 걸지 않는다 */
    if (D.risk_stop) { curve_now = param_quirk(QUIRK_CURVE_NOW) ? 1 : 0; }

    kalman_step(curve_now, dt);

    /* 기존은 센서 원시값이다. 칼만 위치는 1단위(약 3.4 mm) 양자화와 한 틱 지연을 줄인다 */
    pos = param_opt(OPT_KF_POSITION) ? D.kf_pos : (float)D.guide_pos;
    sen = pos;
    if (D.guide_wide == 0) {
        sen = gains_and_deadband(pos, v_abs, r, curve_now, tuning_type, GUIDE_KP_DEV_THRESHOLD, GUIDE_LINEAR_ZONE);
    }
    else if (D.guide_wide == 1) {
        sen = gains_and_deadband(pos, v_abs, r, curve_now, tuning_type, GUIDE_KP_DEV_THRESHOLD_WIDE, GUIDE_LINEAR_ZONE_WIDE);
    }

    /* 조향 기준이나 운전 상태가 바뀌면 적분 기억을 지운다. 위험 감속 중 지우는 것은 예전 조향만 */
    clear_integral = (D.risk_stop && param_quirk(QUIRK_STEER_STEP)) || (D.obs_on && !stop_is_braking()) || (v_abs <= 0)
                  || D.guide_all_off || D.guide_all_on
                  || (s_steer.prev_control != D.control) || (s_steer.prev_branch != D.branch_start)
                  || (s_steer.prev_p_cmd != D.p_cmd) || (s_steer.prev_backward != D.backward);
    if (clear_integral) {
        s_steer.i_out = 0.0f;
        s_steer.i_exit = 0;
        if (D.control != RUN_DRIVE) { D.p_pi_out = 0.0f; }
    }

    /* 정지 진입 때의 조향은 유지하고 적분만 속도에 맞춰 줄인다 */
    if ((D.flag_stopping || stop_is_braking()) && (s_steer.previous_speed > 0) && (v_abs < s_steer.previous_speed)) {
        s_steer.i_out *= (float)v_abs / (float)s_steer.previous_speed;
    }
    s_steer.previous_speed = v_abs;
    s_steer.prev_control = D.control;
    s_steer.prev_branch = D.branch_start;
    s_steer.prev_backward = D.backward;
    s_steer.prev_p_cmd = D.p_cmd;

    if (D.control == RUN_DRIVE) {
        int16_t limit, dev = (D.guide_pos >= 0) ? D.guide_pos : (int16_t)(-D.guide_pos);
        int16_t smooth = !param_quirk(QUIRK_STEER_STEP);     /* 예전 조향은 계단 완화·적분 감쇠가 없다 */
        float p_out, i_candidate, rate_ff, unsat, out;
        /* 센서가 회전 중심에서 멀수록 같은 방향 오차에 위치가 크게 변한다. 기준 거리 250 mm 로 맞춘다 */
        float geom = (P.guide_dist_mm > 0.0f) ? (STEER_GUIDE_REF_MM / P.guide_dist_mm) : 1.0f;
        /* 같은 바퀴 차에 차체가 도는 속도는 휠 간격에 반비례한다. 간격이 넓으면 그만큼 크게 준다 */
        if (!param_quirk(QUIRK_NO_WB_GAIN)) { geom *= ((float)param_get(P_WHEEL_BASE_MM) * 10.0f) / STEER_WB_REF_MM; }

        D.p_err_1 = D.p_err;
        if (D.branch_start != 0) {        /* 분기 중에는 센서 대신 고정 편이 지령을 쓴다 */
            D.p_err = !D.backward ? D.p_cmd : (int16_t)(-D.p_cmd);
        }
        else {
            D.p_err = (int16_t)((float)(D.p_cmd + D.p_test) - (!D.backward ? sen : -sen));
        }

        limit = (int16_t)(((D.v_cmd >= 0) ? D.v_cmd : (int16_t)(-D.v_cmd)) * 2);
#if AGV_START_ALIGN   /* 출발 정렬 조향 상한(drive.h) */
        /* 출발 정렬: |v| × 1(평소의 절반). 안쪽 바퀴 지령이 0 아래로 잘리지 않고 바깥은 2배까지, 최소 회전 반경은 평소와 같은
         * 휠 간격/2 라 따라갈 수 있는 곡선은 그대로 따라간다(× 0.5 는 R 550 아래 곡선을 못 따라갔다). 끝난 뒤 DRIVE_ALIGN_RAMP 틱에 × 2 로 */
        if (D.start_align) {
            int16_t k = (D.start_align < DRIVE_ALIGN_RAMP) ? D.start_align : DRIVE_ALIGN_RAMP;
            limit = (int16_t)((int32_t)limit * (60 - k) / 60);
        }
#endif
        if (limit > STEER_LIMIT_RPM) { limit = STEER_LIMIT_RPM; }

        p_out = D.p_kp * (float)D.p_err * geom;
        rate_ff = GUIDE_KD_RATE * (!D.backward ? D.kf_rate : -D.kf_rate) * geom;
        /* 오차 = 지령 - 위치 이므로 감쇠는 -Kd × 위치 변화율이다. 예전 부호는 반대다 */
        if (!param_quirk(QUIRK_DAMP_SIGN_OLD)) { rate_ff = -rate_ff; }

        /* 다른 선과 붙은 동안의 오차를 적분에 쌓지 않는다 */
        if (!clear_integral && !D.flag_stopping && !stop_is_braking() && (D.guide_track == GUIDE_TRK_NORMAL)) {
            i_candidate = s_steer.i_out + (D.p_ki * (float)D.p_err * geom * i_scale);
            unsat = p_out + i_candidate + rate_ff;
            /* 이미 포화된 출력을 더 미는 방향일 때만 적분을 멈춘다 */
            if (!(((unsat > (float)limit) && (D.p_err > 0)) || ((unsat < (float)-limit) && (D.p_err < 0)))) {
                s_steer.i_out = i_candidate;
            }
        }
        if (s_steer.i_out > (float)limit)       { s_steer.i_out = (float)limit; }
        else if (s_steer.i_out < (float)-limit) { s_steer.i_out = (float)-limit; }
        if (smooth) {
            if (dev >= STEER_I_EXIT_ARM) { s_steer.i_exit = 1; }
            if (s_steer.i_exit && (dev < GUIDE_KP_RAMP_LO)) {
                s_steer.i_out *= fast ? STEER_I_EXIT_DECAY : (STEER_I_EXIT_DECAY * STEER_I_EXIT_DECAY);
                if ((s_steer.i_out < STEER_I_EXIT_DONE) && (s_steer.i_out > -STEER_I_EXIT_DONE)) { s_steer.i_exit = 0; }
            }
        }

        out = p_out + s_steer.i_out + rate_ff;
        /* 분기는 일부러 고정 편이를 거는 동작이라 묶지 않는다 */
        if (smooth && (D.branch_start == 0) && (dev < (D.guide_wide ? STEER_SLEW_DEV_MAX_WIDE : STEER_SLEW_DEV_MAX))) {
            float slew = fast ? STEER_SLEW_RPM : (STEER_SLEW_RPM * 2.0f);
            if (out > D.p_pi_out + slew)      { out = D.p_pi_out + slew; }
            else if (out < D.p_pi_out - slew) { out = D.p_pi_out - slew; }
        }
        if (out > (float)limit)       { out = (float)limit; }
        else if (out < (float)-limit) { out = (float)-limit; }
        D.p_pi_out = out;

        D.add_speed_l = (int16_t)((int16_t)out + D.offset_kp);
        D.add_speed_r = (int16_t)(-(int16_t)out - D.offset_kp);
#if AGV_DEV_TESTS
        /* 추정 조향 시험: 위 37차 계산은 그대로 돌고(적분·칼만 상태 유지) 출력만 덮어쓴다. 분기·예전 조향은 37차 쪽 */
        if ((steer_alg_test == 1) && (D.branch_start == 0) && !param_quirk(QUIRK_STEER_STEP)) { est_run(limit); }
        if (steer_test_rpm != 0) {   /* t3.9 바퀴 지연 측정: 바퀴 지령 차를 시험이 준다. 적분은 쌓지 않고 복귀 때 이 값에서 계단 완화 */
            s_steer.i_out = 0.0f;
            D.p_pi_out = (float)steer_test_rpm;
            D.add_speed_l = (int16_t)(steer_test_rpm + D.offset_kp);
            D.add_speed_r = (int16_t)(-steer_test_rpm - D.offset_kp);
        }
#endif
    }
}
