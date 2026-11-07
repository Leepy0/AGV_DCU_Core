/* stop.c - MotorStop 상태기계. 상수와 동작은 원본 그대로이며 docs/07 의 비교 시험으로 확인한다 */
#include "agv/stop.h"
#include "agv/drive.h"
#include "agv/motor.h"
#include "agv/param.h"

#define D agv_drive
#define P param_dv

float sqrtf(float x);

static stop_ctrl_t s_ctrl;
static stop_ctrl_t s_paused;              /* 일시정지로 끊긴 위치 정지 */
static volatile int16_t s_request;
static int16_t s_paused_valid;
static int16_t s_paused_sign = 1;
static int16_t s_command_sign = 1;
static int16_t s_pause_alarm;

/* ---- 제어기 ---- */

int16_t stop_is_position(int16_t reason)
{
    return (reason >= STOP_REASON_TAG_POSITION) && (reason <= STOP_REASON_TURN_ANGLE);
}

int16_t stop_is_pause(int16_t reason)
{
    return (reason == STOP_REASON_OBSTACLE) || (reason == STOP_REASON_TRAFFIC) || (reason == STOP_REASON_PAUSE);
}

int16_t stop_clamp(int16_t value, int16_t lower, int16_t upper)
{
    if (value < lower) { return lower; }
    if (value > upper) { return upper; }
    return value;
}

static int16_t turn_dcc(int16_t dcc)
{
    return stop_clamp((int16_t)(dcc / STOP_TURN_DCC_DIV), 1, STOP_TURN_DCC_MAX);
}

void stop_ctrl_reset(stop_ctrl_t *c)
{
    c->reason = STOP_REASON_NONE;
    c->phase = STOP_PHASE_IDLE;
    c->command_rpm = 0;
    c->acceleration = 0;
    c->maximum_rpm = 0;
    c->floor_rpm = STOP_MIN_RPM;
    c->candidate_floor_rpm = STOP_MIN_RPM;
    c->accel_limit = 1;
    c->decel_limit = 1;
    c->elapsed_ticks = 0;
    c->floor_hold_ticks = 0;
    c->learn_ticks = 0;
    c->candidate_hold_ticks = 0;
    c->evidence_left = 0;
    c->evidence_right = 0;
    c->no_progress_ticks = 0;
    c->zero_command_ticks = 0;
    c->braking_start_rpm = 0;
    c->braking_start_pulses = 0;
    c->required_decel = 0.0f;
    c->braking_rate_limited = 0;
    c->previous_pulses = 0;
}

static void set_zero(stop_ctrl_t *c)
{
    c->command_rpm = 0;
    c->acceleration = 0;
    c->zero_command_ticks = 0;
    c->phase = STOP_PHASE_WAIT_QUIET;
}

/* 목표에서 0 이 되도록 제동을 계획한다. 바닥 속도는 결과 지령에만 적용한다 */
static void start_distance_brake(stop_ctrl_t *c, int32_t remaining, float scale)
{
    float blend, normalization;
    if ((remaining <= 0) || (scale <= 0.0f) || (c->command_rpm <= c->floor_rpm)) { return; }
    c->braking_start_rpm = c->command_rpm;
    c->braking_start_pulses = remaining;
    blend = (float)stop_clamp(STOP_ENTRY_BLEND_PCT, 0, 50) * 0.01f;
    normalization = 1.0f - blend * 0.5f;
    c->required_decel = (float)c->braking_start_rpm * (float)c->braking_start_rpm * scale /
                        (2.0f * (float)remaining * normalization);
    c->braking_rate_limited = (c->required_decel > (float)c->decel_limit) ? 1 : 0;
}

void stop_ctrl_begin(stop_ctrl_t *c, int16_t reason, int16_t entry_rpm, int16_t maximum_rpm,
                     int16_t acc, int16_t dcc, int32_t remaining, float pulse_scale)
{
    int16_t limit;
    stop_ctrl_reset(c);
    c->reason = reason;
    c->command_rpm = stop_clamp(entry_rpm, 0, STOP_COMMAND_MAX_RPM);
    c->maximum_rpm = stop_clamp(maximum_rpm, 0, STOP_COMMAND_MAX_RPM);
    c->accel_limit = stop_clamp(acc, 1, 60);
    c->previous_pulses = remaining;
    if (reason == STOP_REASON_EMERGENCY) {
        set_zero(c);
        return;
    }
    if (stop_is_position(reason)) {
        limit = (reason == STOP_REASON_TURN_ANGLE)
              ? turn_dcc(dcc)
              : stop_clamp(dcc, 1, STOP_RAPID_DCC_MAX);
        c->phase = (reason == STOP_REASON_TURN_ANGLE) ? STOP_PHASE_CRUISE : STOP_PHASE_DECELERATE;
        c->decel_limit = limit;
        if ((reason == STOP_REASON_TURN_ANGLE) && (c->accel_limit > limit)) { c->accel_limit = limit; }
        if (remaining <= 0) {
            set_zero(c);
        }
        else if ((maximum_rpm < c->floor_rpm) || (pulse_scale <= 0.0f)) {
            set_zero(c);
            c->phase = STOP_PHASE_FAILED;
        }
        else if (reason != STOP_REASON_TURN_ANGLE) {
            start_distance_brake(c, remaining, pulse_scale);
        }
    }
    else {
        c->decel_limit = stop_clamp((int16_t)(dcc / STOP_RAPID_DCC_DIV), 1, STOP_RAPID_DCC_MAX);
        c->phase = STOP_PHASE_DECELERATE;
        if (c->command_rpm <= c->floor_rpm) { set_zero(c); }
    }
}

static int16_t ramp(stop_ctrl_t *c, int16_t target)
{
    int16_t difference, limit, jerk, magnitude, ease_ticks, desired;
    int32_t easing;

    difference = (int16_t)(target - c->command_rpm);
    limit = (difference > 0) ? c->accel_limit : c->decel_limit;
    jerk = (int16_t)(limit / STOP_JERK_DIV);
    if (jerk < 1) { jerk = 1; }
    magnitude = (c->acceleration >= 0) ? c->acceleration : (int16_t)(-c->acceleration);
    ease_ticks = (int16_t)((magnitude + jerk - 1) / jerk);
    easing = (int32_t)magnitude * (ease_ticks + 1) / 2;
    if (((difference >= 0) && ((int32_t)difference <= easing)) ||
        ((difference < 0) && ((int32_t)(-difference) <= easing))) {
        desired = 0;
    }
    else {
        desired = (difference > 0) ? limit : (int16_t)(-limit);
    }
    if (c->acceleration < desired) {
        c->acceleration = (int16_t)(c->acceleration + jerk);
        if (c->acceleration > desired) { c->acceleration = desired; }
    }
    else if (c->acceleration > desired) {
        c->acceleration = (int16_t)(c->acceleration - jerk);
        if (c->acceleration < desired) { c->acceleration = desired; }
    }
    c->command_rpm = (int16_t)(c->command_rpm + c->acceleration);
    if (((difference >= 0) && (c->command_rpm >= target)) ||
        ((difference < 0) && (c->command_rpm <= target))) {
        c->command_rpm = target;
        c->acceleration = 0;
    }
    return c->command_rpm;
}

/* 바닥 위에서는 rpm 제곱이 남은 거리에 비례해 줄어든다. 진입 구간만 가속도를 섞는다 */
static int16_t distance_brake_speed(stop_ctrl_t *c, int32_t remaining)
{
    float ratio, progress, blend, normalization, fraction, nominal;
    int16_t target;

    if (c->braking_start_pulses <= 0) { return c->command_rpm; }
    ratio = (float)remaining / (float)c->braking_start_pulses;
    if (ratio > 1.0f) { ratio = 1.0f; }
    if (ratio < 0.0f) { ratio = 0.0f; }
    progress = 1.0f - ratio;
    blend = (float)stop_clamp(STOP_ENTRY_BLEND_PCT, 0, 50) * 0.01f;
    normalization = 1.0f - blend * 0.5f;
    if ((blend > 0.0f) && (progress < blend)) {
        fraction = progress * progress / (2.0f * blend * normalization);
    }
    else {
        fraction = (progress - blend * 0.5f) / normalization;
    }
    if (fraction > 1.0f) { fraction = 1.0f; }
    if (fraction < 0.0f) { fraction = 0.0f; }
    nominal = (float)c->braking_start_rpm * (float)c->braking_start_rpm * (1.0f - fraction);
    target = (int16_t)(sqrtf(nominal) + 0.5f);
    target = stop_clamp(target, c->floor_rpm, c->command_rpm);
    if (target < c->command_rpm - c->decel_limit) {
        target = (int16_t)(c->command_rpm - c->decel_limit);
        c->braking_rate_limited = 1;
    }
    c->acceleration = (int16_t)(target - c->command_rpm);
    c->command_rpm = target;
    if (target == c->floor_rpm) { c->acceleration = 0; }
    return c->command_rpm;
}

static float braking_pulses(const stop_ctrl_t *c, float scale)
{
    float speed = (float)c->command_rpm;
    float floor_rpm = (float)c->floor_rpm;
    float rpm_sum;
    int16_t jerk, transition;

    if (speed <= floor_rpm) { return 0.0f; }
    jerk = (int16_t)(c->decel_limit / STOP_JERK_DIV);
    if (jerk < 1) { jerk = 1; }
    transition = (int16_t)((c->decel_limit + c->acceleration + jerk - 1) / jerk);
    if (transition < 0) { transition = 0; }
    rpm_sum = (speed * speed - floor_rpm * floor_rpm) / (2.0f * (float)c->decel_limit) +
              speed * (float)(transition + STOP_JERK_DIV + STOP_FEEDBACK_DELAY);
    return rpm_sum * scale;
}

int16_t stop_ctrl_update(stop_ctrl_t *c, const stop_feedback_t *fb)
{
    int16_t both_idle, one_stalled, ceiling;

    both_idle = (fb->idle_left >= STOP_QUIET_TICKS) && (fb->idle_right >= STOP_QUIET_TICKS);
    if ((c->phase == STOP_PHASE_COMPLETE) || (c->phase == STOP_PHASE_FAILED) || (c->phase == STOP_PHASE_IDLE)) {
        return 0;
    }
    if (c->phase == STOP_PHASE_WAIT_QUIET) {
        if (c->zero_command_ticks < STOP_QUIET_TICKS) { c->zero_command_ticks++; }
        if (both_idle && (c->zero_command_ticks >= STOP_QUIET_TICKS)) { c->phase = STOP_PHASE_COMPLETE; }
        return 0;
    }
    if (c->elapsed_ticks < 30000) { c->elapsed_ticks++; }
    if (!stop_is_position(c->reason)) {
        (void)ramp(c, c->floor_rpm);
        if (c->command_rpm <= c->floor_rpm) { set_zero(c); }
        return c->command_rpm;
    }
    /* 목표 도달은 학습·가속보다 항상 먼저다 */
    if (fb->remaining_pulses <= 0) {
        set_zero(c);
        return 0;
    }
    if (fb->remaining_pulses < c->previous_pulses) { c->no_progress_ticks = 0; }
    else if (c->no_progress_ticks < STOP_NO_PROGRESS_TICKS) { c->no_progress_ticks++; }
    c->previous_pulses = fb->remaining_pulses;
    if (c->no_progress_ticks >= STOP_NO_PROGRESS_TICKS) {
        set_zero(c);
        c->phase = STOP_PHASE_FAILED;
        return 0;
    }
    if ((c->elapsed_ticks >= STOP_GRACE_TICKS) && both_idle && (fb->remaining_pulses <= fb->tolerance_pulses)) {
        set_zero(c);
        return 0;
    }
    if (c->phase == STOP_PHASE_CRUISE) {
        if ((float)fb->remaining_pulses > braking_pulses(c, fb->pulses_per_rpm_tick) +
            (float)c->maximum_rpm * fb->pulses_per_rpm_tick * 2.0f) {
            return ramp(c, c->maximum_rpm);
        }
        c->phase = STOP_PHASE_DECELERATE;
        start_distance_brake(c, fb->remaining_pulses, fb->pulses_per_rpm_tick);
    }
    one_stalled = (fb->idle_left >= STOP_STALL_TICKS) || (fb->idle_right >= STOP_STALL_TICKS);
    if ((c->phase == STOP_PHASE_APPROACH) && (c->command_rpm >= c->floor_rpm)) {
        if (c->floor_hold_ticks < STOP_GRACE_TICKS) { c->floor_hold_ticks++; }
    }
    else {
        c->floor_hold_ticks = 0;
    }
    if ((c->phase == STOP_PHASE_APPROACH) && one_stalled && (c->floor_hold_ticks >= STOP_GRACE_TICKS)) {
        c->phase = STOP_PHASE_LEARN_FLOOR;
        c->candidate_floor_rpm = c->floor_rpm;
        c->learn_ticks = 0;
        c->candidate_hold_ticks = STOP_LEARN_INTERVAL;
        c->evidence_left = c->evidence_right = 0;
    }
    if (c->phase == STOP_PHASE_LEARN_FLOOR) {
        if (c->learn_ticks < STOP_LEARN_TIMEOUT) { c->learn_ticks++; }
        if (c->learn_ticks >= STOP_LEARN_TIMEOUT) {
            set_zero(c);
            c->phase = STOP_PHASE_FAILED;
            return 0;
        }
        if (fb->fresh_left && (c->evidence_left < STOP_MOVE_CONFIRM))   { c->evidence_left++; }
        if (fb->fresh_right && (c->evidence_right < STOP_MOVE_CONFIRM)) { c->evidence_right++; }
        if ((c->evidence_left >= STOP_MOVE_CONFIRM) && (c->evidence_right >= STOP_MOVE_CONFIRM) &&
            !one_stalled && (c->command_rpm >= c->candidate_floor_rpm)) {
            c->floor_rpm = c->candidate_floor_rpm;
            c->phase = STOP_PHASE_APPROACH;
            c->floor_hold_ticks = 0;
            c->acceleration = 0;
            return c->command_rpm;
        }
        if (c->command_rpm >= c->candidate_floor_rpm) { c->candidate_hold_ticks++; }
        if (c->candidate_hold_ticks >= STOP_LEARN_INTERVAL) {
            ceiling = (c->maximum_rpm < STOP_LEARN_MAX_RPM) ? c->maximum_rpm : STOP_LEARN_MAX_RPM;
            if (ceiling < c->floor_rpm) { ceiling = c->floor_rpm; }
            c->candidate_floor_rpm = stop_clamp((int16_t)(c->candidate_floor_rpm + STOP_LEARN_STEP_RPM),
                                                c->floor_rpm, ceiling);
            c->candidate_hold_ticks = 0;
            c->evidence_left = c->evidence_right = 0;
        }
        return ramp(c, c->candidate_floor_rpm);
    }
    if (c->phase == STOP_PHASE_APPROACH) {
        return ramp(c, c->floor_rpm);
    }
    if (c->braking_start_pulses > 0) { (void)distance_brake_speed(c, fb->remaining_pulses); }
    else                             { (void)ramp(c, c->floor_rpm); }
    if (c->command_rpm == c->floor_rpm) { c->phase = STOP_PHASE_APPROACH; }
    return c->command_rpm;
}

/* ---- 런타임 ---- */

int16_t stop_is_braking(void)
{
    return (s_ctrl.phase != STOP_PHASE_IDLE) && (s_ctrl.phase != STOP_PHASE_CRUISE);
}

int16_t stop_phase(void)     { return s_ctrl.phase; }
int16_t stop_reason(void)    { return s_ctrl.reason; }
int16_t stop_floor_rpm(void) { return s_ctrl.floor_rpm; }
const stop_ctrl_t *stop_ctrl(void) { return &s_ctrl; }

void stop_clear(void)
{
    stop_ctrl_reset(&s_ctrl);
    s_paused_valid = 0;
}

void stop_clear_request(void)     { s_request = STOP_REASON_NONE; }
void stop_invalidate_paused(void) { s_paused_valid = 0; }

void stop_reset(void)
{
    stop_clear();
    s_request = STOP_REASON_NONE;
    s_paused_sign = 1;
    s_command_sign = 1;
    s_pause_alarm = 0;
}

void stop_request_tag(void)
{
    int16_t reason = ((D.turn_pulses > 0) && (D.turn_motion != 0) && (D.flag_turn == 0))
                   ? STOP_REASON_BEFORE_TURN : STOP_REASON_TAG_DOCKING;
    if ((uint16_t)s_request < STOP_REASON_COMMAND) { s_request = reason; }
}

/* 우선순위 20 > 11 > 10 > 12~14. 뒤늦은 일시정지가 명시적 정지를 약하게 만들지 못한다 */
void stop_request_rapid(int16_t reason)
{
    if (reason < STOP_REASON_COMMAND) { return; }
    if (s_request == STOP_REASON_EMERGENCY) { return; }
    if (reason == STOP_REASON_EMERGENCY) { s_request = reason; return; }
    if ((s_request == STOP_REASON_SWITCH) && (reason != STOP_REASON_SWITCH)) { return; }
    if ((s_request == STOP_REASON_COMMAND) && stop_is_pause(reason)) { return; }
    s_request = reason;
}

static void get_feedback(stop_feedback_t *fb, int16_t reason)
{
    float tolerance;
    fb->idle_left = motor_idle_ticks(0);
    fb->idle_right = motor_idle_ticks(1);
    fb->fresh_left = motor_fresh_pulse(0);
    fb->fresh_right = motor_fresh_pulse(1);
    fb->pulses_per_rpm_tick = (float)STOP_ENCODER_PPR / 6000.0f;
    if (reason == STOP_REASON_TURN_ANGLE) {
        fb->remaining_pulses = D.turn_pulses;
        fb->pulses_per_rpm_tick *= 2.0f;      /* 두 바퀴 엔코더가 모두 센다 */
        tolerance = P.one_length_pulse * (float)param_get(P_WHEEL_BASE_MM) * 10.0f *
                    (3.14159265f * STOP_ANGLE_TOL_DEG / 180.0f);
    }
    else {
        fb->remaining_pulses = D.stop_pulses;
        tolerance = P.one_length_pulse * STOP_DISTANCE_TOL_MM;
    }
    fb->tolerance_pulses = (tolerance > 0.0f) ? (int16_t)tolerance : 0;
    if (P.one_length_pulse <= 0.0f) { fb->pulses_per_rpm_tick = 0.0f; }
}

void stop_begin_runtime(int16_t reason)
{
    stop_feedback_t fb;
    int16_t entry = (D.last_input >= 0) ? D.last_input : (int16_t)(-D.last_input);
    int16_t maximum = (reason == STOP_REASON_TURN_ANGLE) ? P.turn_speed : D.speed_cmd;

    if ((reason != STOP_REASON_TURN_ANGLE) && (maximum < entry)) { maximum = entry; }
    /* 바닥 속도 아래는 바퀴가 안 돈다. 그 구간을 건너뛰어 회전 시작 지연을 줄인다 */
    if ((reason == STOP_REASON_TURN_ANGLE) && (entry < STOP_MIN_RPM) && !param_quirk(QUIRK_TURN_START)) {
        entry = STOP_MIN_RPM;
    }
    s_command_sign = (D.last_input < 0) ? -1 : 1;
    get_feedback(&fb, reason);
    stop_ctrl_begin(&s_ctrl, reason, entry, maximum, P.acc, P.dcc, fb.remaining_pulses, fb.pulses_per_rpm_tick);
    D.vin_out = D.last_input;
    D.scv_accel = 0;
}

int16_t stop_blocking_reason(void)
{
    int16_t obstacle_enabled;

    if ((D.control != RUN_DRIVE) && (D.control != RUN_TURN)) { return STOP_REASON_NONE; }
    if (D.cross_in_on || D.cross_out_on) { return STOP_REASON_TRAFFIC; }
    if (D.pause_stop) { return STOP_REASON_PAUSE; }
    if (D.ext_sen_on) { return STOP_REASON_OBSTACLE; }
    obstacle_enabled = (D.control == RUN_TURN) ? (D.turn_obs_use == 1) : (D.obs_use == 1);
    if (obstacle_enabled &&
        (((P.for_obs > 0) && !D.backward && (D.obs_f || D.obs_f_ext)) ||
         ((P.back_obs > 0) && (D.backward == 1) && (D.obs_b || D.obs_b_ext)))) {
        return STOP_REASON_OBSTACLE;
    }
    return STOP_REASON_NONE;
}

/* 위치 정지 완료 또는 일시정지가 아닌 정지 완료 처리.
 * 일시정지로 끝난 경우 남은 정지 거리는 재개 뒤 이어서 쓰므로 지우지 않는다 */
static void finish(void)
{
    int16_t reason = s_ctrl.reason;

    if (!stop_is_pause(reason)) { D.stop_pulses = 0; }
    if ((reason == STOP_REASON_TURN_ANGLE) || (!stop_is_position(reason) && !stop_is_pause(reason))) {
        D.turn_pulses = 0;
        D.turn_motion = 0;
        D.tape_align = 0;
    }
    D.flag_stopping = 1;
    D.flag_ready = D.flag_run = D.flag_jog_on = D.flag_turn = 0;
    D.amp_run = 0;
}

static void remember_paused(void)
{
    s_paused = s_ctrl;
    s_paused_sign = s_command_sign;
    s_paused_valid = 1;
}

static void resume_after_pause(void)
{
    if (s_paused_valid) {
        /* stop_pulses·turn_pulses 는 일시정지 내내 그대로이므로 끊긴 위치 정지를 남은 거리로 다시 시작한다.
         * 멈춘 뒤 다시 가는 것이라 진입 rpm 은 0 이고, 학습한 바닥 속도는 이어받는다 */
        stop_feedback_t fb;
        int16_t saved_floor = s_paused.floor_rpm;

        get_feedback(&fb, s_paused.reason);
        stop_ctrl_begin(&s_ctrl, s_paused.reason, 0, s_paused.maximum_rpm, P.acc, P.dcc,
                        fb.remaining_pulses, fb.pulses_per_rpm_tick);
        s_command_sign = s_paused_sign;
        D.vin_out = 0;
        D.scv_accel = 0;
        s_ctrl.floor_rpm = saved_floor;
        s_ctrl.candidate_floor_rpm = saved_floor;
        /* 기존은 0 에서 바닥 속도까지만 올려 남은 거리를 기어간다. 남은 거리가 길면 최대 속도로 가속했다가
         * 거리 제동으로 들어간다 (회전 각 제어와 같은 경로) */
        if (param_opt(OPT_RESUME_CRUISE) && stop_is_position(s_paused.reason)
            && (s_ctrl.phase == STOP_PHASE_DECELERATE) && (s_ctrl.braking_start_pulses <= 0)) {
            s_ctrl.phase = STOP_PHASE_CRUISE;
        }
        s_paused_valid = 0;
    }
    else if (D.flag_stopping) {
        /* 일시정지 중 영구 정지가 들어왔다. 다시 움직이지 않는다 */
        stop_begin_runtime(STOP_REASON_COMMAND);
    }
    else {
        stop_ctrl_reset(&s_ctrl);
        D.step = DRIVE_CHANGE_SPEED;
    }
    if (D.action_err == s_pause_alarm) { D.action_err = 0; }
    s_pause_alarm = 0;
    D.obs_on = 0;
}

int16_t stop_run(void)
{
    stop_feedback_t fb;
    int16_t request = s_request;
    int16_t blocking = stop_blocking_reason();

    s_request = STOP_REASON_NONE;
    if (D.stop_err != 0) { request = STOP_REASON_COMMAND; }
    if (D.stop_sw)       { request = STOP_REASON_SWITCH; }

    if (s_ctrl.phase == STOP_PHASE_FAILED) {
        D.vin_out = 0;
        motor_velocity(0);
        D.stop_err = ERR_STOP_APPROACH;
        D.wait_start = D.run_restart = 0;
        D.turn_motion = 0;
        D.turn_pulses = 0;
        if ((motor_idle_ticks(0) >= STOP_QUIET_TICKS) && (motor_idle_ticks(1) >= STOP_QUIET_TICKS)) {
            D.flag_ready = D.flag_run = D.flag_jog_on = D.flag_turn = 0;
            D.amp_run = 0;
        }
        return 1;
    }
    if ((request != STOP_REASON_NONE) &&
        !((s_ctrl.reason == STOP_REASON_TURN_ANGLE) && stop_is_position(request) && (D.turn_pulses <= 0))) {
        if (stop_is_pause(request) && stop_is_position(s_ctrl.reason)) { remember_paused(); }
        else if (!stop_is_pause(request)) { s_paused_valid = 0; }
        if ((request != s_ctrl.reason) || stop_is_position(request)) { stop_begin_runtime(request); }
    }
    if (s_ctrl.phase == STOP_PHASE_IDLE) {
        if (D.flag_stopping) {
            if (D.flag_turn && (D.motion >= MOTION_CCW)) {
                stop_begin_runtime(STOP_REASON_TURN_ANGLE);
            }
            else {
                stop_begin_runtime((D.stop_pulses > 0) ? STOP_REASON_TAG_POSITION : STOP_REASON_COMMAND);
            }
        }
        else if (D.flag_turn && (D.control == RUN_TURN) && !D.tape_align) {
            D.motion = D.turn_motion;
            stop_begin_runtime(STOP_REASON_TURN_ANGLE);
        }
    }
    /* 장애물·교차로는 태그 접근 마지막 구간도 끊는다. 위치 목표와 학습한 바닥을 기억해 두고,
     * 끊긴 목표를 도착으로 치지 않는다 */
    if ((blocking != STOP_REASON_NONE) &&
        ((s_ctrl.phase == STOP_PHASE_IDLE) || stop_is_position(s_ctrl.reason))) {
        if (stop_is_position(s_ctrl.reason)) { remember_paused(); }
        stop_begin_runtime(blocking);
    }
    if ((blocking == STOP_REASON_NONE) && stop_is_pause(s_ctrl.reason) && (s_ctrl.phase == STOP_PHASE_COMPLETE)) {
        resume_after_pause();
    }
    if (s_ctrl.phase == STOP_PHASE_IDLE) { return 0; }
    if (stop_is_pause(s_ctrl.reason)) {
        D.obs_on = 1;
        s_pause_alarm = (s_ctrl.reason == STOP_REASON_TRAFFIC)
                      ? (D.cross_ack_error ? ERR_CROSS_NO_ACK : ERR_CROSS_WAIT)
                      : ((s_ctrl.reason == STOP_REASON_PAUSE) ? ERR_PAUSE : ERR_OBSTACLE);
        D.action_err = s_pause_alarm;
    }
    if (D.flag_stopping) { D.stopping_process = 1; }
    if (D.control != RUN_JOG) { D.amp_dir = D.motion; }
    if (D.immediate_stop && stop_is_position(s_ctrl.reason)) {
        D.stop_pulses = 0;
        if ((s_ctrl.phase != STOP_PHASE_WAIT_QUIET) && (s_ctrl.phase != STOP_PHASE_COMPLETE)) { set_zero(&s_ctrl); }
    }
    get_feedback(&fb, s_ctrl.reason);
    D.cur_speed = (int16_t)(s_command_sign * stop_ctrl_update(&s_ctrl, &fb));
    D.vin_out = D.cur_speed;
    motor_velocity(D.cur_speed);
    if (s_ctrl.phase == STOP_PHASE_FAILED) {
        D.stop_err = ERR_STOP_APPROACH;
        D.wait_start = D.run_restart = 0;
    }
    else if (s_ctrl.phase == STOP_PHASE_COMPLETE) {
        /* 정지가 끝나면 위험 감속 표시도 끝이다. 남으면 다음 주행 내내 적분 0·커브 모드가 된다 */
        if (!param_quirk(QUIRK_RISK_FLAG)) { D.risk_stop = 0; }
        if (!stop_is_pause(s_ctrl.reason)) { finish(); }
    }
    return 1;
}

int16_t stop_alignment_speed(int16_t target_rpm)
{
    if (target_rpm <= 0) { return 0; }
    if (target_rpm < STOP_MIN_RPM) { target_rpm = STOP_MIN_RPM; }
    return motor_scv_speed(target_rpm, P.acc, P.dcc);
}

int16_t stop_turn_dcc(void) { return turn_dcc(P.dcc); }

int16_t stop_turn_speed(int16_t target_rpm)
{
    int16_t dcc = stop_turn_dcc();
    int16_t acc = (P.acc < dcc) ? P.acc : dcc;

    if (target_rpm <= 0) { return 0; }
    if (target_rpm < STOP_MIN_RPM) { target_rpm = STOP_MIN_RPM; }
    return motor_scv_core(target_rpm, &D.vin_out, acc, dcc, P.llow_speed, P.min_speed, P.min_speed, 0, &D.scv_accel);
}

void stop_now(void)
{
    if (s_ctrl.phase != STOP_PHASE_IDLE) { set_zero(&s_ctrl); }
    D.vin_out = 0;
    D.scv_accel = 0;
    motor_velocity(0);
}
