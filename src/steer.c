/* steer.c - 조향. 위치형 PI, 조건부 적분, 칼만 변화율 항, 속도별 게인 스케줄 */
#include "agv/steer.h"
#include "agv/drive.h"
#include "agv/guide.h"
#include "agv/param.h"
#include "agv/steerlog.h"
#include "agv/stop.h"

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
/* 곡선 끝 처리·속도 비례 적분(54차부터 기본: 실차 53차 TST 1 이 가장 나았다). 개발 이미지 TST 2(param_tst_old)는 예전 규칙이고
 * 속도 비례가 없다. 예전 규칙은 적분이 곡선을 붙든 채 편차가 4단위 아래로 내려오자 곡선 안에서 걸려 0.3 s 만에 -480 → -96 rpm 이
 * 빠졌고, 편차가 3 → 13단위로 다시 벌어졌다(실차 51차 덤프).
 *  A: 오차와 적분의 부호가 반대(선이 반대쪽으로 넘어감)일 때만 10 ms 마다 STEER_I_FLIP_DECAY 배. 무장 없음. |적분| < STEER_I_EXIT_DONE 은 그대로.
 *  B: |적분| ≥ STEER_I_FAST_MIN 이고 100 ms 동안 |위치| 가 STEER_I_FAST_UNITS 단위 이상 줄면(곡선이 끝나 선이 빠르게 돌아온다)
 *     그 틱도 STEER_I_FAST_DECAY 배. 곡선 안에서 적분이 편차를 줄이는 빠르기(실측 1단위 120~750 ms)에는 걸리지 않는다.
 *  C(속도 비례): 곡선에 필요한 조향은 속도에 비례한다. 속도 지령이 바뀌면 적분에 |v| 비(한 틱 0.9~1.1)를 곱한다.
 *     두 틱 다 STEER_I_V_MIN 이상일 때만. 정지 중 줄이기(아래 flag_stopping)가 먼저다 */
#define STEER_I_FLIP_DECAY          0.90f
#define STEER_I_FAST_MIN            100.0f
#define STEER_I_FAST_UNITS          2
#define STEER_I_FAST_DECAY          0.95f
#define STEER_I_V_MIN               200
static struct { int16_t pos[10], k, back; } s_ex;   /* B: 지난 10번 위치(부호 있음), back 100 ms 앞 */

static struct {
    float i_out;
    int16_t prev_control, prev_branch, prev_backward, prev_p_cmd;
    int16_t previous_speed;
    int16_t i_exit;         /* 곡선 편차를 지났다: 4단위 아래로 들어오면 적분을 뺀다 */
} s_steer;

/* B 위치 기록. 틱마다 한 번. fast 0(20 ms 조향)은 5번 앞이 100 ms 앞이다 */
static void hist_push(int16_t pos, int16_t fast)
{
    int16_t j = (int16_t)(s_ex.k - (fast ? 10 : 5));
    if (j < 0) { j = (int16_t)(j + 10); }
    s_ex.back = s_ex.pos[j];
    s_ex.pos[s_ex.k] = pos;
    if (++s_ex.k >= 10) { s_ex.k = 0; }
}

/* 곡선 끝 처리 A·B(위 STEER_I_FLIP_*). dev = |위치|. fast 0(20 ms 조향)은 같은 시간 상수: 두 번 곱하고 100 ms 는 5번 앞 */
static void exit_ab(int16_t dev, int16_t fast)
{
    int16_t back = (s_ex.back >= 0) ? s_ex.back : (int16_t)(-s_ex.back);
    float a = s_steer.i_out;
    if ((a < STEER_I_EXIT_DONE) && (a > -STEER_I_EXIT_DONE)) { return; }
    if (((float)D.p_err * a) < 0.0f) { s_steer.i_out *= fast ? STEER_I_FLIP_DECAY : (STEER_I_FLIP_DECAY * STEER_I_FLIP_DECAY); }
    if (((a >= STEER_I_FAST_MIN) || (a <= -STEER_I_FAST_MIN)) && ((back - dev) >= STEER_I_FAST_UNITS)) {
        s_steer.i_out *= fast ? STEER_I_FAST_DECAY : (STEER_I_FAST_DECAY * STEER_I_FAST_DECAY);
    }
}

#if AGV_DEV_TESTS
int16_t steer_test_rpm;
float steer_i_out(void) { return s_steer.i_out; }   /* 조향 기록기(steerlog.c)가 읽는다 */
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

    if (param_gain(P_STEER_KP) == 0) {     /* 자동 이득은 TST 2(예전)에서만. 기본은 0 = 권장값(param.c) */
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
    if (param_gain(P_STEER_KI) == 0) {
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

    /* 위험 감속·재가속 중에는 커브 반응성 부스트를 전부 걸지 않는다 */
    if (D.risk_stop) { curve_now = param_quirk(QUIRK_CURVE_NOW) ? 1 : 0; }

    kalman_step(curve_now, dt);

    pos = (float)D.guide_pos;      /* 센서 원시값. 칼만은 변화율 항만 낸다 */
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
    else if (!param_tst_old && (v_abs != s_steer.previous_speed) && (v_abs >= STEER_I_V_MIN) && (s_steer.previous_speed >= STEER_I_V_MIN)) {
        float k = (float)v_abs / (float)s_steer.previous_speed;     /* C: 위험 감속·재가속에 적분을 속도에 맞춰 옮긴다 */
        if (k > 1.1f) { k = 1.1f; } else if (k < 0.9f) { k = 0.9f; }
        s_steer.i_out *= k;
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
        if (smooth && !param_tst_old) { hist_push(D.guide_pos, fast); }

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
        if (smooth && !param_tst_old) { exit_ab(dev, fast); }
        else if (smooth) {      /* 예전 끝 감쇠(TST 2) */
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
        if (steer_test_rpm != 0) {   /* t3.9 바퀴 지연 측정: 바퀴 지령 차를 시험이 준다. 적분은 쌓지 않고 복귀 때 이 값에서 계단 완화 */
            s_steer.i_out = 0.0f;
            D.p_pi_out = (float)steer_test_rpm;
            D.add_speed_l = (int16_t)(steer_test_rpm + D.offset_kp);
            D.add_speed_r = (int16_t)(-steer_test_rpm - D.offset_kp);
        }
#endif
    }
}
