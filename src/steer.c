/* steer.c - 조향. 위치형 PI, 조건부 적분, 칼만 변화율 항, 속도별 게인 스케줄 */
#include "agv/steer.h"
#include "agv/drive.h"
#include "agv/guide.h"
#include "agv/param.h"
#include "agv/stop.h"

#define D agv_drive
#define P param_dv

#define GUIDE_KP_MIN                20.0f
#define GUIDE_KP_MAX                130.0f
#define GUIDE_KP_SPEED_GAIN         15.0f
#define GUIDE_KP_DEV_THRESHOLD      12
#define GUIDE_KP_DEV_THRESHOLD_WIDE 20
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
#define STEER_LIMIT_RPM             1500

static struct {
    float i_out;
    int16_t prev_control, prev_branch, prev_backward, prev_p_cmd;
    int16_t previous_speed;
} s_steer;

void steer_reset(void)
{
    s_steer.i_out = 0.0f;
    s_steer.prev_control = -1;
    s_steer.prev_branch = 0;
    s_steer.prev_backward = -1;
    s_steer.prev_p_cmd = 0;
    s_steer.previous_speed = 0;
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

/* 'a' 소스는 레거시 동작 그대로다. 'h' 소스만 테이프 폭에 맞춘 임계를 쓴다 */
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
static void kalman_step(int16_t curve_now)
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
        float pos_pred = D.kf_pos + (D.kf_rate * GUIDE_KF_DT);
        float rate_pred = D.kf_rate;
        float p00_pred = D.kf_p00 + (2.0f * GUIDE_KF_DT * D.kf_p01) + (GUIDE_KF_DT * GUIDE_KF_DT * D.kf_p11) + q00;
        float p01_pred = D.kf_p01 + (GUIDE_KF_DT * D.kf_p11);
        float p11_pred = D.kf_p11 + q11;

        if (!D.guide_all_off) {
            float y = (float)D.guide_pos - pos_pred;
            float s = p00_pred + GUIDE_KF_R;
            float k0 = p00_pred / s;
            float k1 = p01_pred / s;
            D.kf_pos = pos_pred + (k0 * y);
            D.kf_rate = rate_pred + (k1 * y);
            D.kf_p00 = (1.0f - k0) * p00_pred;
            D.kf_p01 = (1.0f - k0) * p01_pred;
            D.kf_p11 = p11_pred - (k1 * p01_pred);
        }
        else {          /* 전 채널 소등: 예측만 반영하고 불확실성은 키운다 */
            D.kf_pos = pos_pred;
            D.kf_rate = rate_pred;
            D.kf_p00 = p00_pred;
            D.kf_p01 = p01_pred;
            D.kf_p11 = p11_pred;
        }
    }
}

/* 좁은·넓은 가이드 공통. 임계값만 다르다. 반환은 불감대를 적용한 위치값 */
static float gains_and_deadband(int16_t v_abs, float r, int16_t curve_now, int16_t tuning_type,
                                int16_t dev_threshold, int16_t linear_zone)
{
    float sen = (float)D.guide_pos;
    int16_t band;

    if (param_get(P_STEER_KP) == 0) {
        D.p_kp = (v_abs > 0) ? (GUIDE_KP_MIN + (GUIDE_KP_SPEED_GAIN * r)) : GUIDE_KP_MIN;
        if (D.p_kp > GUIDE_KP_MAX) { D.p_kp = GUIDE_KP_MAX; }
        /* 위험 감속 중에는 강제 최대 게인을 걸지 않는다 */
        if (((D.guide_pos > dev_threshold) || (D.guide_pos < -dev_threshold)) && !D.risk_stop) {
            D.p_kp = GUIDE_KP_MAX;
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
    band = (int16_t)(GUIDE_DEADBAND_SPEED_GAIN * r);
    if (band < GUIDE_DEADBAND_MIN) { band = GUIDE_DEADBAND_MIN; }
    if (curve_now) { band = (int16_t)((float)band * GUIDE_DEADBAND_CURVE_SCALE); }
    if ((tuning_type == 1) || (tuning_type == 2)) {
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
    float   sen;

    /* 위험 감속·재가속 중에는 커브 반응성 부스트를 전부 걸지 않는다 */
    if (D.risk_stop) { curve_now = param_quirk(QUIRK_CURVE_NOW) ? 1 : 0; }

    kalman_step(curve_now);

    sen = (float)D.guide_pos;
    if (D.guide_wide == 0) {
        sen = gains_and_deadband(v_abs, r, curve_now, tuning_type, GUIDE_KP_DEV_THRESHOLD, GUIDE_LINEAR_ZONE);
    }
    else if (D.guide_wide == 1) {
        sen = gains_and_deadband(v_abs, r, curve_now, tuning_type, GUIDE_KP_DEV_THRESHOLD_WIDE, GUIDE_LINEAR_ZONE_WIDE);
    }

    /* 조향 기준이나 운전 상태가 바뀌면 적분 기억을 지운다 */
    clear_integral = D.risk_stop || (D.obs_on && !stop_is_braking()) || (v_abs <= 0)
                  || D.guide_all_off || D.guide_all_on
                  || (s_steer.prev_control != D.control) || (s_steer.prev_branch != D.branch_start)
                  || (s_steer.prev_p_cmd != D.p_cmd) || (s_steer.prev_backward != D.backward);
    if (clear_integral) {
        s_steer.i_out = 0.0f;
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
        int16_t limit;
        float p_out, i_candidate, rate_ff, unsat, out;

        D.p_err_1 = D.p_err;
        if (D.branch_start != 0) {        /* 분기 중에는 센서 대신 고정 편이 지령을 쓴다 */
            D.p_err = !D.backward ? D.p_cmd : (int16_t)(-D.p_cmd);
        }
        else {
            D.p_err = (int16_t)((float)D.p_cmd - (!D.backward ? sen : -sen));
        }

        limit = (int16_t)(((D.v_cmd >= 0) ? D.v_cmd : (int16_t)(-D.v_cmd)) * 2);
        if (limit > STEER_LIMIT_RPM) { limit = STEER_LIMIT_RPM; }

        p_out = D.p_kp * (float)D.p_err;
        rate_ff = GUIDE_KD_RATE * (!D.backward ? D.kf_rate : -D.kf_rate);

        if (!clear_integral && !D.flag_stopping && !stop_is_braking()) {
            i_candidate = s_steer.i_out + (D.p_ki * (float)D.p_err);
            unsat = p_out + i_candidate + rate_ff;
            /* 이미 포화된 출력을 더 미는 방향일 때만 적분을 멈춘다 */
            if (!(((unsat > (float)limit) && (D.p_err > 0)) || ((unsat < (float)-limit) && (D.p_err < 0)))) {
                s_steer.i_out = i_candidate;
            }
        }
        if (s_steer.i_out > (float)limit)       { s_steer.i_out = (float)limit; }
        else if (s_steer.i_out < (float)-limit) { s_steer.i_out = (float)-limit; }

        out = p_out + s_steer.i_out + rate_ff;
        if (out > (float)limit)       { out = (float)limit; }
        else if (out < (float)-limit) { out = (float)-limit; }
        D.p_pi_out = out;

        D.add_speed_l = (int16_t)((int16_t)out + D.offset_kp);
        D.add_speed_r = (int16_t)(-(int16_t)out - D.offset_kp);
    }
}
