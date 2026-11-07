/* motor.c - rpm 피드백, 속도 PI, S커브, 주행 단계 400~405 */
#include "agv/motor.h"
#include "agv/branch.h"
#include "agv/drive.h"
#include "agv/param.h"
#include "agv/steer.h"
#include "agv/stop.h"

#define D agv_drive
#define P param_dv

#define RPM_CONST            500000000L    /* SYSCLK 100 MHz, 12 PPR */
#define TCNT_SYSCLK_PER_TICK 1000000L
#define IDLE_TICKS_MAX       1000
#define SCV_JERK_DIV         8
#define GUIDE_RISK_DCC       60
#define GUIDE_RISK_ACC       20
#define GUIDE_RISK_THRESHOLD 10
#define GUIDE_REALIGN_RANGE  3
#define GUIDE_REALIGN_TICKS  10
#define TAPE_SLIP_MIN_DEG    3.0f    /* 이만큼 돈 뒤에 슬립비를 잰다. 초반 잡음을 뺀다 */
#define TAPE_LOST_TICKS      5       /* 라인이 이 틱을 넘게 안 보이면 출발 라인을 벗어난 것 */
#define TAPE_SLOW_POS        12      /* 새 라인이 이 안에 들면 최저 속도 */
#define TAPE_CENTER_POS      1       /* 이 안이면 회전을 끝낸다 */
#define GUIDE_BIT_PITCH_MM   6.89f
#define GUIDE_TO_CENTER_MM   250.0f
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
}

void    motor_encoder_pulse(uint16_t wheel) { s_motor.pulse_seq[wheel]++; }
int16_t motor_idle_ticks(uint16_t wheel)    { return s_motor.idle[wheel]; }
int16_t motor_fresh_pulse(uint16_t wheel)   { return s_motor.fresh[wheel]; }

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

        if (cur > MOTOR_RPM_SPIKE) { D.rpm[i] = s_motor.pre_rpm[i]; }
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
        D.tq_cmd[i] = (D.w_pi_out[i] > 0.0f) ? (int16_t)D.w_pi_out[i] : 0;
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
    int16_t n_acc = ((*vout > llow) || (acc < 5)) ? acc : 5;
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
    case DRIVE_RISK_STOP:
        D.cur_speed = motor_scv_speed((int16_t)((float)P.low_speed * 0.7f), GUIDE_RISK_ACC, GUIDE_RISK_DCC);
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

static int32_t guide_offset_to_pulse(int16_t guide)
{
    float offset_mm = (float)guide * GUIDE_BIT_PITCH_MM;
    float angle_deg = (offset_mm / GUIDE_TO_CENTER_MM) * (180.0f / PI_F);
    return (int32_t)(angle_deg * ((float)P.turn_90 / 90.0f));
}

static void tape_end(void)
{
    D.tape_step = 0;
    D.turn_pulses = 0;
    D.tape_align = 0;
}

static int32_t abs32(int32_t v) { return (v < 0) ? -v : v; }

/* 지금 속도에서 최저 속도까지 줄이는 데 드는 두 바퀴 합 펄스 */
static float tape_brake_pulses(void)
{
    float v = (float)D.vin_out;
    float low = (float)P.turn_min;

    if (v <= low) { return 0.0f; }
    return ((v * v - low * low) / (2.0f * (float)stop_turn_dcc())) * ((float)STOP_ENCODER_PPR / 6000.0f) * 2.0f;
}

/* 정렬 회전 한 틱. 가이드는 회전 중 'h' 로만 받는다. 위치 부호는 보지 않아 실측 전에도 쓴다 */
static int16_t tape_adjust(void)
{
    int16_t speed = P.turn_speed;
    int32_t turned = D.tape_turned;
    float   late = (float)turned - (D.tape_target_cnt + (float)D.tape_max);
    int32_t pos = abs32(D.guide_pos);
    int16_t seen = (int16_t)(!D.guide_all_off && !D.guide_all_on);
    float   brake = tape_brake_pulses();

    switch (D.tape_step) {
    case 0:     /* 출발 라인에서 멀어지는 동안 슬립비를 잰다 */
        if (!D.guide_all_off) {
            D.tape_lost_cnt = 0;
            if (turned > (int32_t)(((float)P.turn_90 / 90.0f) * TAPE_SLIP_MIN_DEG)) {
                int32_t measured = abs32(guide_offset_to_pulse(D.guide_pos));
                if (measured > 0) {
                    D.tape_slip_ratio = (float)measured / (float)turned;
                    if (D.tape_slip_ratio < 0.5f) { D.tape_slip_ratio = 0.5f; }
                    if (D.tape_slip_ratio > 1.5f) { D.tape_slip_ratio = 1.5f; }
                }
            }
        }
        else {
            D.tape_lost_cnt++;
        }
        /* 라인이 끝내 안 사라져도 목표각 앞에서는 찾기로 넘어간다 */
        if ((D.tape_lost_cnt > TAPE_LOST_TICKS)
            || ((float)turned >= ((float)D.tape_total_pulse - (float)D.tape_max))) {
            D.tape_target_cnt = (float)D.tape_total_pulse / D.tape_slip_ratio;
            D.tape_step = 1;
        }
        break;
    case 1:     /* 목표각 근방에서만 새 라인을 찾는다 */
        if (late > 0.0f) {
            D.system_err = ERR_GUIDE_SENSOR;
            tape_end();
        }
        else if (seen && (pos <= TAPE_SLOW_POS)
                 && ((float)turned >= (D.tape_target_cnt - (float)D.tape_max))) {
            speed = P.turn_min;
            D.tape_step = 2;
        }
        else if ((float)turned >= (D.tape_target_cnt - brake)) {
            speed = P.turn_min;      /* 감속이 느리니 라인을 보기 전에 줄인다 */
        }
        break;
    case 2:     /* 최저 속도로 중앙까지 */
        speed = P.turn_min;
        if ((seen && (pos <= TAPE_CENTER_POS)) || (late > 0.0f)) { tape_end(); }
        break;
    default:
        break;
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

    s_motor.steer_gate++;
    if (s_motor.steer_gate > 1) {
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
            case RUN_TURN:
                D.motion = D.turn_motion;
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
