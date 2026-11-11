/* motor.c - rpm 피드백, 속도 PI, S커브, 주행 단계 400~405 */
#include "agv/motor.h"
#include "agv/branch.h"
#include "agv/drive.h"
#include "agv/guide.h"
#include "agv/param.h"
#include "agv/steer.h"
#include "agv/stop.h"

float sqrtf(float x);
float atanf(float x);

#define D agv_drive
#define P param_dv

#define RPM_CONST            500000000L    /* SYSCLK 100 MHz, 12 PPR */
#define TCNT_SYSCLK_PER_TICK 1000000L
#define IDLE_TICKS_MAX       1000
#define SCV_JERK_DIV         8
/* 저속(LOS/2 아래)은 엔코더 펄스가 드물어(1회전 12펄스) 속도 제어가 계단처럼 거칠다. 가속 때 그 구간 체류를 줄인다 */
#define SCV_LOW_ACC_DIV      2
#define SCV_LOW_ACC_MIN      5
#define GUIDE_RISK_DCC       60
#define GUIDE_RISK_ACC       20
#define GUIDE_RISK_THRESHOLD 10
#define GUIDE_REALIGN_RANGE  3
#define GUIDE_REALIGN_TICKS  10
#define TAPE_SLIP_MIN_DEG    3.0f    /* 이만큼 돈 뒤에 슬립비를 잰다. 초반 잡음을 뺀다 */
/* 슬립 보정 범위 ±25%. 탐색 범위는 보정 전·후 목표를 함께 덮으므로 측정이 틀려도 라인이 범위 밖으로 나가지 않는다 */
#define TAPE_SLIP_LOW        0.8f
#define TAPE_SLIP_HIGH       1.25f
#define TAPE_SLIP_POS        12      /* 출발 라인이 이 안일 때만 슬립을 잰다. 센서 가장자리는 위치가 포화한다 */
#define TAPE_LOST_TICKS      5       /* 라인이 이 틱을 넘게 안 보이면 출발 라인을 벗어난 것 */
#define TAPE_SLOW_POS        12      /* 새 라인이 이 안에 들면 라인을 잡은 것(단계 2) */
#define TAPE_CENTER_POS      1       /* 이 안이면 회전을 끝낸다 */
#define TAPE_CREEP_DEG       2.0f    /* 최저 속도로 도는 마지막 각. 라인 가운데 맞추기 */
#define TAPE_PLAN_DCC        0.75f   /* 감속 계획 = 회전 감속(DCC/4, 많아야 10) × 이 값. S커브 지연 여유 */
#define TAPE_SETTLE_TICKS    20      /* 중앙에서 선 뒤 0.2 s 기다리고 바퀴가 서면 편차를 본다 */
#define TAPE_SETTLE_MAX      100     /* 바퀴가 안 서도 1 s 면 본다 */
#define TAPE_FIX_MAX         2       /* 보정 횟수 상한 */
#define TAPE_FIX_DEG         5.0f    /* 보정 한 번의 회전각 상한 */
#define TAPE_FIX_LOST_TICKS  10      /* 정착·보정 중 선 잃음은 이만큼 이어져야 인정한다. 일시정지 뒤 재개 첫 틱은 선 없음으로 시작한다 */
#define PI_F                 3.14159265f

agv_drive_t agv_drive;

static struct {
    volatile uint16_t pulse_seq[2];
    uint16_t observed_seq[2];
    int16_t  idle[2];
    int16_t  fresh[2];
    int16_t  pre_rpm[2];
    int16_t  hist[2][5];       /* w_rpm_flt_add1~5 */
    int16_t  steer_gate;       /* CntTimer50ms */
    int16_t  stall_cnt[2];     /* 지령이 바닥 속도 이상인 연속 틱 */
    int16_t  raw[2];           /* 스파이크 컷 전 rpm */
    int16_t  force[2], forcing;
    int16_t  spike;            /* 스파이크 컷. 0 = MOTOR_RPM_SPIKE */
} s_motor;

void motor_reset(void)
{
    uint16_t i, k;
    for (i = 0; i < 2; i++) {
        s_motor.pulse_seq[i] = 0;
        s_motor.observed_seq[i] = 0;
        s_motor.idle[i] = IDLE_TICKS_MAX;
        s_motor.fresh[i] = 0;
        s_motor.pre_rpm[i] = 0;
        for (k = 0; k < 5; k++) { s_motor.hist[i][k] = 0; }
    }
    s_motor.steer_gate = 0;
    s_motor.stall_cnt[0] = s_motor.stall_cnt[1] = 0;
    s_motor.raw[0] = s_motor.raw[1] = 0;
    s_motor.forcing = 0;
    s_motor.spike = 0;
}

static int16_t dac_clamp(int16_t v) { return (v < 0) ? 0 : ((v > MOTOR_DAC_MAX) ? MOTOR_DAC_MAX : v); }

void motor_dac_force(int16_t left, int16_t right)
{
    s_motor.force[0] = dac_clamp(left);
    s_motor.force[1] = dac_clamp(right);
    s_motor.forcing = 1;
}

void    motor_dac_release(void)             { s_motor.forcing = 0; }
int16_t motor_dac_forced(void)              { return s_motor.forcing; }
int16_t motor_raw_rpm(uint16_t wheel)       { return s_motor.raw[wheel]; }
void    motor_spike_set(int16_t rpm)        { s_motor.spike = (rpm > 0) ? rpm : 0; }
int16_t motor_spike(void)                   { return (s_motor.spike > 0) ? s_motor.spike : MOTOR_RPM_SPIKE; }

void    motor_encoder_pulse(uint16_t wheel) { s_motor.pulse_seq[wheel]++; }
int16_t motor_idle_ticks(uint16_t wheel)    { return s_motor.idle[wheel]; }
int16_t motor_fresh_pulse(uint16_t wheel)   { return s_motor.fresh[wheel]; }
uint16_t motor_pulse_seq(uint16_t wheel)    { return s_motor.pulse_seq[wheel]; }

/* ---- rpm 피드백 ---- */

void motor_feedback_update(void)
{
    uint16_t i;

    for (i = 0; i < 2; i++) {
        uint16_t sequence = s_motor.pulse_seq[i];
        int32_t period = D.period[i];
        int32_t cur = 0;

        s_motor.fresh[i] = (sequence != s_motor.observed_seq[i]);
        s_motor.observed_seq[i] = sequence;
        if (s_motor.fresh[i]) { s_motor.idle[i] = 0; }
        else if (s_motor.idle[i] < IDLE_TICKS_MAX) { s_motor.idle[i]++; }

        if ((period > 0) && (s_motor.idle[i] < STOP_QUIET_TICKS)) {
            cur = RPM_CONST / period;
            /* 저속에서 캡처가 늦으면 값이 계단으로 남는다. 경과 틱으로 상한을 낮춘다 */
            if (s_motor.idle[i] > 0) {
                int32_t bound = RPM_CONST / ((int32_t)s_motor.idle[i] * TCNT_SYSCLK_PER_TICK);
                if (cur > bound) { cur = bound; }
            }
        }

        s_motor.raw[i] = (int16_t)((cur > 32767L) ? 32767L : cur);
        if (cur > motor_spike()) { D.rpm[i] = s_motor.pre_rpm[i]; }
        else                       { s_motor.pre_rpm[i] = D.rpm[i] = (int16_t)cur; }

        D.rpm_avg[i] = (int16_t)((D.rpm[i] + s_motor.hist[i][0] + s_motor.hist[i][1] + s_motor.hist[i][2]
                                  + s_motor.hist[i][3] + s_motor.hist[i][4]) / 6);
        D.rpm_flt[i] = (int16_t)(((float)D.rpm_flt[i] * 0.9f) + ((float)D.rpm_avg[i] * 0.1f));

        s_motor.hist[i][4] = s_motor.hist[i][3];
        s_motor.hist[i][3] = s_motor.hist[i][2];
        s_motor.hist[i][2] = s_motor.hist[i][1];
        s_motor.hist[i][1] = s_motor.hist[i][0];
        s_motor.hist[i][0] = D.rpm[i];
    }
    D.rpm_fit_sum = (int16_t)(D.rpm_flt[0] + D.rpm_flt[1]);
}

/* 지령은 바닥 속도 이상인데 한쪽 바퀴만 펄스가 끊기면 엔코더·바퀴 이상이다.
 * 램프 구간은 지령 연속 틱으로, 정지 바닥 학습 구간은 제동 중 제외로 뺀다 */
void motor_stall_check(void)
{
    int16_t n = P.stall_ticks;
    uint16_t i;

    if ((n <= 0) || !D.amp_run || stop_is_braking() || ((D.control != RUN_DRIVE) && (D.control != RUN_TURN))) {
        s_motor.stall_cnt[0] = s_motor.stall_cnt[1] = 0;
        return;
    }
    for (i = 0; i < 2; i++) {
        if (D.w_cmd[i] >= stop_min_rpm()) { if (s_motor.stall_cnt[i] < IDLE_TICKS_MAX) { s_motor.stall_cnt[i]++; } }
        else                            { s_motor.stall_cnt[i] = 0; }
    }
    for (i = 0; i < 2; i++) {
        uint16_t other = 1u - i;
        if ((s_motor.stall_cnt[i] >= n) && (s_motor.idle[i] >= n) && (s_motor.idle[other] < n) && (D.system_err == 0)) {
            D.system_err = (i == 0) ? ERR_ENC_L_STALL : ERR_ENC_R_STALL;
        }
    }
}

/* ---- 속도 PI ---- */

static int16_t jog_wheel_cmd(uint16_t i, int16_t input)
{
    int16_t add_this  = (i == 0) ? D.add_speed_l : D.add_speed_r;
    int16_t add_other = (i == 0) ? D.add_speed_r : D.add_speed_l;
    int16_t only_this = (i == 0) ? D.jog_left_only : D.jog_right_only;
    uint16_t own_stop = (i == 0) ? 0xB : 0x7;     /* 이 키는 이 바퀴를 세우고 반대 바퀴만 돌린다 */
    uint16_t other_stop = (i == 0) ? 0x7 : 0xB;

    if ((add_this == 0) && ((uint16_t)D.jog_bits == own_stop)) {
        D.w_err[i] = 0;
        D.rpm_avg[i] = 0;
        return 0;
    }
    if ((add_other == 0) && ((uint16_t)D.jog_bits == other_stop)) {
        return add_this;
    }
    if ((only_this == 1) && (D.jog_stopping == 1)) {
        D.w_err[i] = 0;
        return 0;
    }
    return (int16_t)(add_this + input);
}

void motor_speed_pi(int16_t input)
{
    static const float ka = 1.0f / MOTOR_SPEED_KP;
    uint16_t i;

    D.v_cmd = input;
    for (i = 0; i < 2; i++) {
        switch (D.control) {
        case RUN_JOG:   D.w_cmd[i] = jog_wheel_cmd(i, input); break;
        case RUN_DRIVE: D.w_cmd[i] = (int16_t)(((i == 0) ? D.add_speed_l : D.add_speed_r) + input); break;
        case RUN_TURN:  D.w_cmd[i] = input; break;
        default:        break;
        }
        if (D.w_cmd[i] < 0) { D.w_cmd[i] = 0; }
        D.w_err_1[i] = D.w_err[i];
        D.w_err[i] = (int16_t)(D.w_cmd[i] - D.rpm_avg[i]);
        if (D.w_cmd[i] == 0) {
            D.w_err[i] = 0;
            D.w_pi_out[i] = D.iref_it[i] = D.iref_fb[i] = 0.0f;
        }
        /* 역계산 anti-windup */
        D.iref_it[i] += MOTOR_SPEED_KI * ((float)D.w_err[i] - ka * (D.iref_fb[i] - D.w_pi_out[i]));
        D.iref_fb[i] = MOTOR_SPEED_KP * (float)D.w_err[i] + D.iref_it[i];
        D.w_pi_out[i] = D.iref_fb[i];
        if (D.w_pi_out[i] > MOTOR_TORQUE_LIMIT) { D.w_pi_out[i] = MOTOR_TORQUE_LIMIT; }
        {   /* 피드포워드: 지령 rpm 에 비례한 토크를 더해 PI 가 메울 몫을 줄인다 */
            float tq = D.w_pi_out[i] + (P.speed_ff * (float)D.w_cmd[i]);
            if (tq > MOTOR_TORQUE_LIMIT) { tq = MOTOR_TORQUE_LIMIT; }
            D.tq_cmd[i] = (tq > 0.0f) ? (int16_t)tq : 0;
        }
    }
}

void motor_velocity(int16_t input)
{
    D.last_input = input;
    if (input == 0) {
        D.add_speed_l = D.add_speed_r = 0;
        D.offset_kp = 0;
        D.p_pi_out = 0.0f;
    }
    /* 평균 지령은 두고 두 바퀴 지령을 검증된 바닥 위에 둔다. 조향 권한은 바닥에서 0 이 된다 */
    if (stop_is_braking() && (input >= stop_floor_rpm())) {
        int16_t limit = (int16_t)(input - stop_floor_rpm());
        if (limit > STOP_COMMAND_MAX_RPM - input) { limit = (int16_t)(STOP_COMMAND_MAX_RPM - input); }
        D.add_speed_l = stop_clamp(D.add_speed_l, (int16_t)(-limit), limit);
        D.add_speed_r = stop_clamp(D.add_speed_r, (int16_t)(-limit), limit);
    }
    motor_speed_pi(input);
}

/* ---- S커브 ---- */

int16_t motor_scv_core(int16_t vin, int16_t *vout, int16_t acc, int16_t dcc, int16_t llow,
                       int16_t min_speed, int16_t stop_floor, int16_t use_dcc_floor, int16_t *accel)
{
    int16_t diff, a_lim, jerk, a_tgt, a_abs, n;
    int16_t v_abs = (*vout >= 0) ? *vout : (int16_t)(-*vout);
    int16_t low_acc = param_quirk(QUIRK_LOW_ACC) ? SCV_LOW_ACC_MIN : (int16_t)(acc / SCV_LOW_ACC_DIV);
    int16_t n_acc = ((*vout > llow) || (acc < SCV_LOW_ACC_MIN)) ? acc
                  : ((low_acc > SCV_LOW_ACC_MIN) ? low_acc : SCV_LOW_ACC_MIN);
    int16_t n_dcc = (!use_dcc_floor || (v_abs > min_speed * 2) || (dcc > 20)) ? dcc : 20;
    int32_t brake;

    if (*vout == 0) { *accel = 0; }
    diff = (int16_t)(vin - *vout);
    a_lim = ((*vout >= 0) == (diff >= 0)) ? n_acc : n_dcc;
    jerk = (int16_t)(a_lim / SCV_JERK_DIV);
    if (jerk < 1) { jerk = 1; }

    /* 지금 가속도를 0 으로 내려도 남은 차이 안에서 목표를 넘지 않는지 본다 */
    a_abs = (*accel >= 0) ? *accel : (int16_t)(-*accel);
    n = (int16_t)((a_abs + jerk - 1) / jerk);
    brake = ((int32_t)a_abs * (n + 1)) / 2;
    if (((diff >= 0) && ((int32_t)diff <= brake)) || ((diff < 0) && ((int32_t)(-diff) <= brake))) {
        a_tgt = 0;
    }
    else {
        a_tgt = (diff >= 0) ? a_lim : (int16_t)(-a_lim);
    }

    if (*accel < a_tgt) {
        *accel = (int16_t)(*accel + jerk);
        if (*accel > a_tgt) { *accel = a_tgt; }
    }
    else if (*accel > a_tgt) {
        *accel = (int16_t)(*accel - jerk);
        if (*accel < a_tgt) { *accel = a_tgt; }
    }
    *vout = (int16_t)(*vout + *accel);

    if (((diff >= 0) && (*vout >= vin)) || ((diff < 0) && (*vout <= vin))) {
        *vout = vin;
        *accel = 0;
    }
    v_abs = (*vout >= 0) ? *vout : (int16_t)(-*vout);
    if ((vin == 0) && (v_abs < stop_floor)) {
        *vout = 0;
        *accel = 0;
    }
    return *vout;
}

int16_t motor_scv_speed(int16_t target, int16_t acc, int16_t dcc)
{
    return motor_scv_core(target, &D.vin_out, acc, dcc, P.llow_speed, P.min_speed, P.min_speed, 1, &D.scv_accel);
}

/* ---- 주행 단계 ---- */

static void step_change_speed(void)
{
    D.cur_speed = motor_scv_speed(D.speed_cmd, P.acc, P.dcc);
    motor_velocity(D.cur_speed);
    if (D.cur_speed == D.speed_cmd) { D.step = DRIVE_CRUISE; }
}

void motor_drive_step(void)
{
    int16_t obs_sensor = 0;

    if (D.drive_pre != 0) { D.drive_pre(); }

    if ((D.obs_use == 1) && !D.flag_stopping && !D.cross_in_flag && !D.cross_in_on
        && !D.cross_out_flag && !D.cross_out_on) {
        if (((P.for_obs > 0) && !D.backward && (D.obs_f || D.obs_f_ext)) ||
            ((P.back_obs > 0) && (D.backward == 1) && (D.obs_b || D.obs_b_ext))) {
            if ((D.tape_stop_id == 0) && (D.tape_turn_dir == 0) && (D.turn_motion == 0) && (D.stop_pulses == 0)) {
                obs_sensor = 1;
            }
        }
    }

    if (obs_sensor || D.pause_stop || D.cross_in_on || D.cross_out_on || D.ext_sen_on) {
        D.obs_on = 1;
        D.step = DRIVE_OBS_STOP;
        if (D.pause_stop) {
            D.action_err = ERR_PAUSE;
        }
        else if (D.cross_in_on || D.cross_out_on) {
            D.action_err = D.cross_ack_error ? ERR_CROSS_NO_ACK : ERR_CROSS_WAIT;
        }
        else if (obs_sensor) {
            D.action_err = ERR_OBSTACLE;
        }
    }
    else {
        D.obs_on = 0;
        if ((D.action_err == ERR_OBSTACLE) || (D.action_err == ERR_CROSS_WAIT) || (D.action_err == ERR_CROSS_NO_ACK)) {
            D.action_err = 0;
        }
    }

    if ((D.action_err == ERR_OBSTACLE) && D.pause_stop) { D.action_err = ERR_CROSS_WAIT; }

    if (!D.obs_on && !D.flag_stopping
        && ((D.step == DRIVE_CRUISE) || (D.step == DRIVE_CHANGE_SPEED) || (D.step == DRIVE_RISK_RESUME))) {
        if ((D.vin_out >= P.llow_speed)
            && ((D.guide_pos >= GUIDE_RISK_THRESHOLD) || (D.guide_pos <= -GUIDE_RISK_THRESHOLD))) {
            D.risk_stop = 1;
            D.guide_realign_cnt = 0;
            D.step = DRIVE_RISK_STOP;
        }
    }

    branch_run();

    switch (D.step) {
    case DRIVE_CHANGE_SPEED:
        step_change_speed();
        break;
    case DRIVE_CRUISE:
        D.vin_out = D.speed_cmd;
        D.scv_accel = 0;
        motor_velocity(D.speed_cmd);
        break;
    case DRIVE_OBS_STOP:
        if (D.obs_on) {
            stop_begin_runtime(stop_blocking_reason());
            (void)stop_run();
            break;
        }
        step_change_speed();
        break;
    case DRIVE_COMM:
        D.cur_speed = motor_scv_speed(P.llow_speed, P.acc, P.dcc);
        motor_velocity(D.cur_speed);
        break;
    case DRIVE_RISK_STOP: {
        /* 지령이 더 낮으면 지령 그대로다. 위험 감속이 속도를 올리지 않고, 아래 풀림 조건(cur_speed <= speed_cmd)이 성립한다 */
        int16_t risk = (int16_t)((float)P.low_speed * 0.7f);
        D.cur_speed = motor_scv_speed((risk < D.speed_cmd) ? risk : D.speed_cmd, GUIDE_RISK_ACC, GUIDE_RISK_DCC);
        motor_velocity(D.cur_speed);
        if ((D.cur_speed <= D.speed_cmd) && (D.guide_pos <= GUIDE_REALIGN_RANGE) && (D.guide_pos >= -GUIDE_REALIGN_RANGE)) {
            D.guide_realign_cnt++;
            if (D.guide_realign_cnt >= GUIDE_REALIGN_TICKS) {
                D.guide_realign_cnt = 0;
                D.step = DRIVE_RISK_RESUME;
            }
        }
        else {
            D.guide_realign_cnt = 0;
        }
        break;
    }
    case DRIVE_RISK_RESUME:
        D.cur_speed = motor_scv_speed(D.speed_cmd, GUIDE_RISK_ACC, GUIDE_RISK_DCC);
        motor_velocity(D.cur_speed);
        if (D.cur_speed == D.speed_cmd) {
            D.risk_stop = 0;
            D.step = DRIVE_CRUISE;
        }
        break;
    default:
        break;
    }
}

/* ---- 테이프 회전 정렬 ---- */

/* 이 회전의 실제 1도당 두 바퀴 펄스. tagact_turn 이 넣은 turn_90(회전 보정비를 건 값), 없으면 휠 간격 값 */
static float tape_deg_pulse(void)
{
    return (float)((D.turn_90 > 0) ? D.turn_90 : P.turn_90) / 90.0f;
}

/* 센서 편차 → 회전각 펄스. 센서는 회전 중심에서 GD 앞이라 각 = atan(편차 / GD). 직선 근사는 큰 편차에서 각을 키운다 */
static int32_t guide_offset_to_pulse(int16_t guide)
{
    float offset_mm = (float)guide * guide_h_scale() * GUIDE_UNIT_MM;
    float angle_deg = atanf(offset_mm / P.guide_dist_mm) * (180.0f / PI_F);
    return (int32_t)(angle_deg * tape_deg_pulse());
}

static void tape_end(void)
{
    D.tape_step = 0;
    D.turn_pulses = 0;
    D.tape_align = 0;
    D.tape_fix_rev = 0;
}

static int32_t abs32(int32_t v) { return (v < 0) ? -v : v; }

/* 회전 끝 속도: 남은 각(가이드 위치로 고친 펄스)에 비례해 TAPE_CREEP_DEG 에서 최저 회전 속도, 중앙 쪽으로 정지 바닥까지 낮춘다.
 * 중앙에서는 tape_stop 이 0 으로 선다. 예전(QUIRK_FLOOR_350)은 최저 회전 속도 그대로 */
static int16_t tape_end_speed(float left)
{
    float v = (float)P.turn_min * left / (TAPE_CREEP_DEG * tape_deg_pulse());
    if (param_quirk(QUIRK_FLOOR_350) || (v >= (float)P.turn_min)) { return P.turn_min; }
    return (v > (float)stop_min_rpm()) ? (int16_t)v : stop_min_rpm();
}

/* 라인 중앙까지 left 펄스. 마지막 TAPE_CREEP_DEG 만 최저 속도로 가게 거리로 감속한다: v² = 최저² + 2·a·거리.
 * 새 라인을 잡았으면(단계 2) 그 위치로 tape_end_speed 가 더 낮춘다 */
static int16_t tape_plan(float left)
{
    float d = left - TAPE_CREEP_DEG * tape_deg_pulse();
    float k = ((float)STOP_ENCODER_PPR / 6000.0f) * 2.0f;     /* rpm·틱당 두 바퀴 펄스 */
    float low = (float)P.turn_min;
    float v;

    if (d <= 0.0f) { return P.turn_min; }
    v = sqrtf(low * low + 2.0f * TAPE_PLAN_DCC * (float)stop_turn_dcc() * d / k);
    return (v >= (float)P.turn_speed) ? P.turn_speed : (int16_t)v;
}

/* 새 라인이 보이면 위치로 남은 각을 알 수 있다. 중앙 예상을 고친다. 많아야 탐색 범위 끝 */
static void tape_refine(int32_t turned, int32_t pos, float hi)
{
    float end = (float)turned + (float)abs32(guide_offset_to_pulse((int16_t)pos));
    D.tape_end_cnt = (end > hi) ? hi : end;
}

/* 지령 0 으로 바로 서고 정착을 기다린다 */
static int16_t tape_stop(void)
{
    D.tape_step = TAPE_STEP_SETTLE;
    D.tape_wait = 0;
    D.tape_lost_cnt = 0;
    D.vin_out = 0;
    D.scv_accel = 0;
    return 0;
}

/* 중앙에서 선 뒤. 정착하면 편차를 보고, 중앙 밖이면 최저 속도로 편차를 줄이는 쪽으로 돌다가 중앙에서 다시 선다.
 * 방향은 새 라인이 다가올 때의 위치 부호(tape_side)와 견준다: 같으면 덜 왔다(같은 방향), 다르면 지나쳤다(반대 방향).
 * 센서 부호 실측과 무관하다. 보정은 tape_turned 에 넣지 않는다(회전 보정비 측정). 오류로 만들지 않고 tape_fix_end 에 사유만 남긴다 */
static int16_t tape_after(int16_t seen, int32_t pos)
{
    if (D.tape_step == TAPE_STEP_FIX) {
        if (!D.guide_all_off)                 { D.tape_lost_cnt = 0; }
        else if (++D.tape_lost_cnt >= TAPE_FIX_LOST_TICKS) { D.tape_fix_end = 3; return tape_stop(); }
        if (seen && (pos <= TAPE_CENTER_POS)) { return tape_stop(); }
        if ((float)D.tape_fix_turned > TAPE_FIX_DEG * tape_deg_pulse()) { D.tape_fix_end = 2; return tape_stop(); }
        return seen ? tape_end_speed((float)abs32(guide_offset_to_pulse((int16_t)pos))) : P.turn_min;
    }
    D.vin_out = 0;
    D.scv_accel = 0;
    D.tape_wait++;
    if ((D.tape_wait < TAPE_SETTLE_TICKS)
        || (((D.rpm[0] != 0) || (D.rpm[1] != 0)) && (D.tape_wait < TAPE_SETTLE_MAX))) {
        return 0;
    }
    if (!seen && (D.tape_fix_end < 0) && (++D.tape_lost_cnt < TAPE_FIX_LOST_TICKS)) { return 0; }
    if (D.tape_fix_cnt == 0) { D.tape_stop_pos = D.guide_pos; }
    D.tape_final_pos = D.guide_pos;
    if (D.tape_fix_end < 0) {
        if (!seen)                               { D.tape_fix_end = 3; }
        else if (pos <= TAPE_CENTER_POS)         { D.tape_fix_end = 0; }
        else if (D.tape_side == 0)               { D.tape_fix_end = 4; }
        else if (D.tape_fix_cnt >= TAPE_FIX_MAX) { D.tape_fix_end = 1; }
        else {
            D.tape_fix_cnt++;
            D.tape_fix_rev = (int16_t)(((D.guide_pos > 0) ? 1 : -1) != D.tape_side);
            D.tape_fix_turned = 0;
            D.tape_lost_cnt = 0;
            D.tape_step = TAPE_STEP_FIX;
            if (!param_quirk(QUIRK_TURN_START)) { D.vin_out = stop_min_rpm(); }   /* 바닥 아래 구간을 건너뛴다(회전 시작과 같다) */
            return tape_end_speed((float)abs32(guide_offset_to_pulse(D.guide_pos)));
        }
    }
    tape_end();
    return 0;
}

/* 정렬 회전 한 틱. 가이드는 회전 중 'h' 로만 받는다. 중앙까지는 위치 부호를 보지 않아 실측 전에도 쓴다.
 * 목표 = 휠 간격·회전 보정비로 잡은 펄스 ÷ 슬립비(출발 라인이 보이는 동안 최소제곱으로 계속 고침).
 * 탐색 범위 = 보정 전·후 목표 둘 다 ± tape_max. 라인을 보면 위치로 중앙 예상(tape_end_cnt)을 고치고 거기까지 거리로 감속한다 */
static int16_t tape_adjust(void)
{
    int16_t speed = D.tape_slow ? P.turn_min : P.turn_speed;
    int32_t turned = D.tape_turned;
    float   total = (float)D.tape_total_pulse;
    float   lo = ((D.tape_target_cnt < total) ? D.tape_target_cnt : total) - (float)D.tape_max;
    float   hi = ((D.tape_target_cnt > total) ? D.tape_target_cnt : total) + (float)D.tape_max;
    int32_t pos = abs32(D.guide_pos);
    int16_t seen = (int16_t)(!D.guide_all_off && !D.guide_all_on);
    int16_t in_win = (int16_t)((float)turned >= lo);

    switch (D.tape_step) {
    case 0:     /* 출발 라인에서 멀어지는 동안 슬립비를 잰다 */
        if (!D.guide_all_off) {
            D.tape_lost_cnt = 0;
            if (!D.tape_origin_ok) {
                D.tape_origin = D.guide_pos;
                D.tape_origin_ok = 1;
            }
            if (seen && (pos <= TAPE_SLIP_POS) && ((float)turned > tape_deg_pulse() * TAPE_SLIP_MIN_DEG)) {
                float m = (float)abs32(guide_offset_to_pulse((int16_t)(D.guide_pos - D.tape_origin)));
                float t = (float)turned;
                if (m > 0.0f) {      /* 원점을 지나는 직선 m = 비 × t 에 맞춘다. 많이 돈 표본일수록 무겁다 */
                    D.tape_sum_mt += m * t;
                    D.tape_sum_tt += t * t;
                    D.tape_slip_ratio = D.tape_sum_mt / D.tape_sum_tt;
                    if (D.tape_slip_ratio < TAPE_SLIP_LOW)  { D.tape_slip_ratio = TAPE_SLIP_LOW; }
                    if (D.tape_slip_ratio > TAPE_SLIP_HIGH) { D.tape_slip_ratio = TAPE_SLIP_HIGH; }
                    D.tape_target_cnt = D.tape_end_cnt = total / D.tape_slip_ratio;
                }
            }
        }
        else {
            D.tape_lost_cnt++;
        }
        /* 라인이 끝내 안 사라져도 목표각 앞에서는 찾기로 넘어간다 */
        if ((D.tape_lost_cnt > TAPE_LOST_TICKS) || in_win) {
            D.tape_step = 1;
        }
        break;
    case 1:     /* 탐색 범위에서만 새 라인을 찾는다 */
        if ((float)turned > hi) {
            D.system_err = ERR_GUIDE_SENSOR;
            tape_end();
        }
        else if (seen && in_win) {
            tape_refine(turned, pos, hi);
            if (pos > TAPE_CENTER_POS) { D.tape_side = (D.guide_pos > 0) ? 1 : -1; }
            if (pos <= TAPE_SLOW_POS) { D.tape_step = 2; }
        }
        break;
    case 2:     /* 라인을 잡았다. 중앙까지. 중앙이면 지령 0 으로 바로 선다(각도 우선) */
        if (seen) { tape_refine(turned, pos, hi); }
        if (seen && (pos > TAPE_CENTER_POS)) { D.tape_side = (D.guide_pos > 0) ? 1 : -1; }
        if (seen && (pos <= TAPE_CENTER_POS)) { return tape_stop(); }
        if ((float)turned > hi) { tape_end(); }
        break;
    case TAPE_STEP_SETTLE:
    case TAPE_STEP_FIX:
        return tape_after(seen, pos);
    default:
        break;
    }
    if (D.tape_align) {
        int16_t plan = tape_plan(D.tape_end_cnt - (float)turned);
        /* 감속을 시작하면 다시 올리지 않는다. 새 라인으로 끝점이 멀어져도 지금 속도로 가다가 마지막만 최저 속도 */
        if ((D.tape_plan_min > 0) && (plan > D.tape_plan_min)) { plan = D.tape_plan_min; }
        D.tape_plan_min = plan;
        if (plan < speed) { speed = plan; }
        if ((D.tape_step == 2) && seen) {      /* 라인 위 마지막 TAPE_CREEP_DEG: 가이드 위치로 남은 각에 비례 */
            float g = (float)abs32(guide_offset_to_pulse(D.guide_pos));
            if ((g < TAPE_CREEP_DEG * tape_deg_pulse()) && (tape_end_speed(g) < speed)) { speed = tape_end_speed(g); }
        }
    }
    return speed;
}

int16_t motor_turn_adjust(void)
{
    int16_t aligning = D.tape_align;
    int16_t speed = aligning ? tape_adjust() : P.turn_speed;

    if ((D.turn_obs_use == 1)
        && (((P.for_obs == 1) && !D.backward && D.obs_f) || ((P.back_obs == 1) && (D.backward == 1) && D.obs_b))) {
        return stop_alignment_speed(0);
    }
    return aligning ? stop_turn_speed(speed) : stop_alignment_speed(speed);
}

/* ---- 10 ms 본체 ---- */

static void run_reset_branch(void)
{
    uint16_t i, k;

    D.vin_out = 0;
    D.scv_accel = 0;
    if (D.system_err || D.estop) {
        if (stop_reason() != STOP_REASON_EMERGENCY) { stop_begin_runtime(STOP_REASON_EMERGENCY); }
        stop_invalidate_paused();
    }
    else if (D.stop_err != ERR_STOP_APPROACH) {
        stop_clear();
    }
    stop_clear_request();
    D.last_input = 0;
    D.flag_ready = D.flag_run = D.flag_jog_on = D.flag_turn = 0;
    D.step = DRIVE_CHANGE_SPEED;
    D.flag_stopping = 0;
    for (i = 0; i < 2; i++) {
        D.tq_cmd[i] = 0;
        D.w_pi_out[i] = 0.0f;
        D.w_err[i] = D.w_err_1[i] = 0;
        D.w_cmd[i] = 0;
        D.rpm[i] = 0;
        D.iref_it[i] = D.iref_fb[i] = 0.0f;
        for (k = 0; k < 5; k++) { s_motor.hist[i][k] = 0; }
    }
    D.add_speed_l = D.add_speed_r = 0;
    D.p_err = D.p_err_1 = 0;
    D.p_pi_out = 0.0f;
    D.speed_cmd = P.low_speed;
    D.drive_start_cnt = 0;
    D.restart_cnt = 100;
    if (!param_quirk(QUIRK_RISK_FLAG)) { D.risk_stop = 0; }
    if (D.stop_err != ERR_STOP_APPROACH) { D.stop_pulses = 0; }
    D.sound_on = 0;
}

void motor_run(void)
{
    if (!D.flag_stopping && (stop_phase() == STOP_PHASE_IDLE)) { D.immediate_stop = 0; }
    motor_feedback_update();
    motor_stall_check();

    if (s_motor.forcing) {      /* 바닥 rpm 시험: 주행·조향·정지 제어를 모두 건너뛴다 */
        uint16_t i;
        int16_t ok = (int16_t)(!D.system_err && !D.estop);
        D.amp_run = ok;
        D.motion = D.amp_dir = D.backward;   /* 폐루프 주행(RUN_DRIVE)이 낼 방향 그대로. 제어 상태로 바꾸지 않는다 */
        D.vin_out = 0;
        for (i = 0; i < 2; i++) {
            D.w_cmd[i] = 0;
            D.tq_cmd[i] = ok ? s_motor.force[i] : 0;
        }
        return;
    }

    s_motor.steer_gate++;
    if (!param_quirk(QUIRK_STEER_20MS) || (s_motor.steer_gate > 1)) {
        int16_t tuning = (D.vin_out > P.low_speed) ? 2 : 1;
        D.offset_kp = (int16_t)((float)D.w_cmd[0] * 0.0001f * (float)param_get(P_STEER_RPM_OFFSET));
        steer_run(tuning);
        s_motor.steer_gate = 0;
    }

    if (!D.system_err && !D.estop && (D.flag_ready || D.flag_run || D.flag_jog_on || D.flag_turn)) {
        D.amp_run = 1;
        D.sound_on = (param_get(P_SOUND) == 1) && (D.vin_out > 0);
        if (!stop_run()) {
            switch (D.control) {
            case RUN_JOG:
                motor_velocity(D.jog_rpm);
                break;
            case RUN_DRIVE:
                D.motion = D.backward;
                D.amp_dir = D.motion;
                motor_drive_step();
                break;
            case RUN_TURN:   /* 지나친 뒤 보정은 반대 방향(CCW 2 ↔ CW 3) */
                D.motion = (D.tape_align && D.tape_fix_rev) ? (int16_t)(D.turn_motion ^ 1) : D.turn_motion;
                D.amp_dir = D.motion;
                motor_velocity(motor_turn_adjust());
                break;
            case RUN_READY:
                break;
            default:
                D.tq_cmd[0] = D.tq_cmd[1] = 0;
                D.w_cmd[0] = D.w_cmd[1] = 0;
                break;
            }
        }
    }
    else {
        run_reset_branch();
    }
}
