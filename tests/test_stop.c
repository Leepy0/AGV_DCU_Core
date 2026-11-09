/* test_stop.c - MotorStop 상태기계. 원본과 같은 궤적인지는 앱 저장소 비교 시험이 맡는다 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "agv/acs.h"
#include "agv/motor.h"
#include "agv/param.h"
#include "agv/steer.h"
#include "agv/stop.h"
#include "fakes.h"

#define D agv_drive
#define P param_dv

static long s_left_acc[2];
static long s_travel;          /* 주행 바퀴 누적 펄스 */
static int s_friction;         /* 1 = 제동 토크 없음: 지령보다 빨리 줄지 못하고 dv/dt = -(3000 + 0.1 v) rpm/s 로 준다 */
static float s_fv[2];

static void setup(int16_t rpm)
{
    fake_reset();
    fake_params();
    memset(&D, 0, sizeof(D));
    motor_reset();
    stop_reset();
    steer_reset();
    memset(s_left_acc, 0, sizeof(s_left_acc));
    s_travel = 0;
    s_friction = 0;
    s_fv[0] = s_fv[1] = (float)rpm;
    acs_reset();
    D.ui_mode = UI_AUTO;
    D.control = RUN_DRIVE;
    D.step = DRIVE_CRUISE;
    D.flag_run = 1;
    D.speed_cmd = rpm;
    D.vin_out = rpm;
    D.last_input = rpm;
}

/* 지령 rpm 대로 도는 바퀴. 일정 rpm 미만이면 안 돈다 */
static void plant(int16_t stiction)
{
    for (uint16_t i = 0; i < 2; i++) {
        int32_t rpm = D.w_cmd[i] > stiction ? D.w_cmd[i] : 0;
        if (s_friction) {
            float lo = s_fv[i] - (3000.0f + 0.1f * s_fv[i]) * 0.01f;
            s_fv[i] = ((float)rpm > lo) ? (float)rpm : ((lo > 0.0f) ? lo : 0.0f);
            rpm = (int32_t)s_fv[i];
        }
        s_left_acc[i] += rpm * 12;
        while (s_left_acc[i] >= 6000) {
            s_left_acc[i] -= 6000;
            motor_encoder_pulse(i);
            D.period[i] = 500000000L / (rpm ? rpm : 1);
            if ((i == 0) && !D.backward) { s_travel++; if (D.cross_pulses > 0) { D.cross_pulses--; } }
            if ((i == 0) && !D.backward && (D.stop_pulses > 0)) { D.stop_pulses--; }
            if ((i == 1) && D.backward && (D.stop_pulses > 0))  { D.stop_pulses--; }
            if (D.turn_pulses > 0) { D.turn_pulses--; }
        }
    }
}

static void tick(int16_t stiction)
{
    plant(stiction);
    D.action_err = 0;       /* 스케줄러가 매 10 ms 앞에서 지운다 */
    motor_run();
}

static int run_until_idle(int16_t stiction, int limit)
{
    int t = 0;
    while (D.flag_run && (t < limit)) { tick(stiction); t++; }
    return t;
}

/* 한 틱 지령 변화 한계. 바닥 속도 이하에서 0 으로 내리는 것은 설계다 (바퀴가 안 돈다) */
static int16_t s_prev;
static int s_jump, s_flip, s_zero;
static void watch_reset(void) { s_prev = D.last_input; s_jump = s_flip = s_zero = 0; }
static void watch(void)
{
    int16_t bound = (int16_t)(P.dcc + P.dcc / STOP_JERK_DIV + 1);
    int16_t step = (int16_t)(D.last_input - s_prev);
    if (step < 0) { step = (int16_t)(-step); }
    if ((step > bound) && !((D.last_input == 0) && (s_prev <= STOP_MIN_RPM + bound))) { s_jump++; }
    if (D.last_input < 0) { s_flip++; }
    if (D.last_input == 0) { s_zero++; }
    s_prev = D.last_input;
}
static void watch_tick(void) { tick(0); watch(); }

static float travel_mm(long from) { return (float)(s_travel - from) / P.one_length_pulse; }

/* 가속 중 정지·일시정지: 지령이 튀지 않고 부호가 안 바뀌고 끝까지 선다 */
static void test_ramp_continuity(void)
{
    int k, t;
    for (k = 0; k < 2; k++) {
        setup(400);
        D.speed_cmd = 3000;
        D.step = DRIVE_CHANGE_SPEED;
        watch_reset();
        for (t = 0; t < 60; t++) { watch_tick(); }
        assert(D.last_input > 600 && D.scv_accel > 0);       /* 가속 중이다 */
        if (k == 0) { stop_request_rapid(STOP_REASON_COMMAND); D.flag_stopping = 1; }
        else        { D.pause_stop = 1; }
        for (t = 0; (t < 3000) && D.flag_run && (stop_phase() != STOP_PHASE_COMPLETE); t++) { watch_tick(); }
        assert(s_jump == 0 && s_flip == 0);
        assert((k == 0) ? (D.flag_run == 0) : (stop_phase() == STOP_PHASE_COMPLETE));
        assert(D.last_input == 0);
    }
}

/* 교차로 대기 정지: 태그 뒤 CID cm 에서 선다 */
static long traffic_begin(int16_t rpm, int16_t cm)
{
    setup(rpm);
    (void)param_set(P_CROSS_IN_CM, cm);
    acs_cross_in(1);
    watch_reset();
    return s_travel;
}

static int run_until_hold(int limit)
{
    int t = 0;
    while (!((stop_reason() == STOP_REASON_TRAFFIC) && (stop_phase() == STOP_PHASE_COMPLETE)) && (t < limit)) {
        watch_tick();
        t++;
    }
    return t;
}

static void test_traffic(void)
{
    static const int16_t speeds[3] = { 1200, 2000, 3000 };
    int k, t;
    long from;

    for (k = 0; k < 3; k++) {
        int seen_drive = 0, seen_alarm = 0;
        from = traffic_begin(speeds[k], 60);
        assert(D.cross_in_flag && !D.cross_in_on);
        watch_reset();
        for (t = 0; (t < 3000) && (stop_reason() != STOP_REASON_TRAFFIC_POSITION); t++) {
            watch_tick();
            if ((stop_phase() == STOP_PHASE_IDLE) && (D.last_input == speeds[k])) { seen_drive++; }
        }
        assert(seen_drive > 0);                                   /* 제동점까지 주행 단계가 그대로 달린다 */
        assert(stop_phase() == STOP_PHASE_DECELERATE);
        for (t = 0; (t < 3000) && (stop_reason() == STOP_REASON_TRAFFIC_POSITION); t++) {
            watch_tick();
            if (D.action_err == ERR_CROSS_WAIT) { seen_alarm++; }
        }
        assert(seen_alarm > 0);
        assert(run_until_hold(3000) < 3000);
        assert(D.action_err == ERR_CROSS_WAIT && D.cross_in_on && !D.cross_in_flag);
        { float err = travel_mm(from) - 600.0f;
          printf("  traffic %d rpm: %.1f mm\n", speeds[k], travel_mm(from));
          assert(err > -3.0f && err < 3.0f);
          assert(stop_last.reason == STOP_REASON_TRAFFIC_POSITION);      /* 유지가 조용해질 때 쓴다 */
          assert(fabsf(stop_last.overshoot_mm - err) <= 0.5f / P.one_length_pulse + 0.01f);
          assert(stop_last.target_mm > 0.0f && stop_last.target_mm < 600.0f); }   /* 제동점에서 남은 거리 */
        assert(s_jump == 0 && s_flip == 0);
        for (t = 0; t < 200; t++) { tick(0); assert(D.last_input == 0); }   /* 허가까지 선다 */
        acs_cross_release(ACS_CROSS_IN);                          /* 선 뒤 허가: 오늘처럼 다시 출발 */
        tick(0);
        assert(stop_phase() == STOP_PHASE_IDLE && D.step == DRIVE_CHANGE_SPEED && D.action_err == 0);
        for (t = 0; t < 1000; t++) { tick(0); }
        assert(D.last_input == speeds[k] && D.flag_run == 1);
    }

    /* 서기 전에 허가: 0 으로 내리지 않고 지금 지령에서 다시 가속한다 */
    for (k = 0; k < 3; k++) {
        int16_t low;
        traffic_begin(speeds[k], 60);
        for (t = 0; (t < 3000) && (stop_reason() != STOP_REASON_TRAFFIC_POSITION); t++) { tick(0); }
        while (D.last_input > (speeds[k] * 2) / 3) { tick(0); }
        if (k == 2) { while (stop_phase() != STOP_PHASE_APPROACH) { tick(0); } }   /* 바닥 속도에서 허가 */
        watch_reset();
        low = D.last_input;
        acs_cross_release(ACS_CROSS_IN);
        for (t = 0; t < 1500; t++) { watch_tick(); if (D.last_input < low) { low = D.last_input; } }
        assert(s_zero == 0 && s_jump == 0 && s_flip == 0);
        assert(low > 0);
        assert(stop_phase() == STOP_PHASE_IDLE && D.last_input == speeds[k] && D.cross_in_on == 0);
    }

    /* 0 cm: 태그 읽은 자리에서 곧바로 급정지 */
    traffic_begin(2000, 0);
    assert(D.cross_in_on && !D.cross_in_flag);
    tick(0);
    assert(stop_reason() == STOP_REASON_TRAFFIC && stop_phase() == STOP_PHASE_DECELERATE);
    assert(run_until_hold(3000) < 3000);

    /* 명시적 정지가 이긴다. 선 뒤에는 허가까지 유지 */
    traffic_begin(2000, 60);
    for (t = 0; (t < 3000) && (stop_reason() != STOP_REASON_TRAFFIC_POSITION); t++) { tick(0); }
    tick(0);
    stop_request_rapid(STOP_REASON_COMMAND);
    D.flag_stopping = 1;
    tick(0);
    assert(stop_reason() == STOP_REASON_COMMAND);
    assert(run_until_idle(0, 3000) < 3000);
    assert(D.cross_in_on && !D.cross_in_flag);

    /* 남은 거리가 더 짧은 태그 정지가 이기고, 선 뒤 다시 출발해도 허가까지 안 움직인다 */
    from = traffic_begin(2000, 60);
    D.stop_pulses = 300;
    D.flag_stopping = 1;
    stop_request_tag();
    assert(run_until_idle(0, 3000) < 3000);
    assert(D.cross_in_on && !D.cross_in_flag);
    assert(travel_mm(from) < 0.5f + 300.0f / P.one_length_pulse);
    tick(0);                                                      /* 운전 끝: 제어기 초기화 */
    D.flag_stopping = 0;
    D.flag_run = 1;
    D.step = DRIVE_CHANGE_SPEED;
    for (t = 0; t < 300; t++) { tick(0); assert(D.last_input == 0); }
    assert(stop_reason() == STOP_REASON_TRAFFIC && D.action_err == ERR_CROSS_WAIT);
    acs_cross_release(ACS_CROSS_IN);
    for (t = 0; t < 300; t++) { tick(0); }
    assert(D.last_input > 0);

    /* 더 먼 태그 정지는 교차로 목표에서 끊기고, 허가 뒤 남은 거리를 이어 간다 */
    from = traffic_begin(2000, 60);
    D.stop_pulses = 1500;
    D.flag_stopping = 1;
    stop_request_tag();
    assert(run_until_hold(3000) < 3000);
    { float err = travel_mm(from) - 600.0f; assert(err > -3.0f && err < 3.0f); }
    assert(D.flag_run == 1 && D.stop_pulses > 0);
    acs_cross_release(ACS_CROSS_IN);
    tick(0);
    assert(stop_reason() == STOP_REASON_TAG_DOCKING);
    assert(run_until_idle(0, 5000) < 5000);
    assert(D.stop_pulses == 0);
    { long n = s_travel - from; assert(n >= 1498 && n <= 1502); }

    /* 제동 중 장애물: 교차로 목표를 지키고, 풀린 뒤 같은 자리에 선다 */
    from = traffic_begin(2000, 60);
    for (t = 0; (t < 3000) && (stop_reason() != STOP_REASON_TRAFFIC_POSITION); t++) { tick(0); }
    for (t = 0; t < 5; t++) { tick(0); }
    D.ext_sen_on = 1;
    tick(0);
    assert(stop_reason() == STOP_REASON_OBSTACLE);
    for (t = 0; (t < 3000) && (stop_phase() != STOP_PHASE_COMPLETE); t++) { tick(0); }
    assert(D.cross_in_flag && D.cross_pulses > 0);
    D.ext_sen_on = 0;
    { uint16_t n0 = stop_last.n;
      assert(run_until_hold(3000) < 3000);
      float err = travel_mm(from) - 600.0f; assert(err > -3.0f && err < 3.0f);
      assert(stop_last.n == n0 + 1 && stop_last.reason == STOP_REASON_TRAFFIC_POSITION);   /* 장애물 정지는 쓰지 않는다 */
      assert(fabsf(stop_last.overshoot_mm - err) <= 0.5f / P.one_length_pulse + 0.01f); }
}

/* 위치 정지 넘침 기록. 제동 토크가 없는 바퀴로 잰다 */
static void test_stop_last(void)
{
    long from;
    uint16_t n0;

    /* 거리가 넉넉한 태그 정지: 지령대로 서는 바퀴면 목표에 선다 */
    setup(2000);
    n0 = stop_last.n;
    from = s_travel;
    D.stop_pulses = 600;
    D.flag_stopping = 1;
    stop_request_tag();
    assert(run_until_idle(0, 3000) < 3000);
    printf("  stop_last 600 p: %.1f mm, %.0f rpm/s, %d\n", stop_last.overshoot_mm, stop_last.required_decel,
           stop_last.rate_limited);
    assert(stop_last.n == n0 + 1 && stop_last.reason == STOP_REASON_TAG_DOCKING && stop_last.entry_rpm == 2000);
    assert(stop_last.rate_limited == 0 && stop_last.required_decel > 0.0f);
    assert(stop_last.target_mm <= 600.0f / P.one_length_pulse + 0.01f && stop_last.target_mm > 590.0f / P.one_length_pulse);
    assert(fabsf(stop_last.overshoot_mm - (travel_mm(from) - 600.0f / P.one_length_pulse)) < 0.01f);   /* 목표 = 걸 때 자리 + 600 */
    assert(fabsf(stop_last.overshoot_mm) < STOP_DISTANCE_TOL_MM);
    assert(stop_last.stop_ticks > 0);

    /* 짧은 목표: dcc 로 묶이고, 지령이 0 이 돼도 마찰로만 서서 지나친다 */
    setup(3000);
    s_friction = 1;
    from = s_travel;
    D.stop_pulses = 20;
    D.flag_stopping = 1;
    stop_request_tag();
    assert(run_until_idle(0, 3000) < 3000);
    printf("  stop_last 20 p: %.1f mm, %.0f rpm/s, %d, max %.1f\n", stop_last.overshoot_mm, stop_last.required_decel,
           stop_last.rate_limited, stop_last.max_overshoot_mm);
    assert(stop_last.n == n0 + 2 && stop_last.rate_limited == 1);
    assert(stop_last.required_decel > (float)P.dcc * 100.0f);
    assert(stop_last.overshoot_mm > 100.0f / P.one_length_pulse);
    assert(fabsf(stop_last.overshoot_mm - (travel_mm(from) - 20.0f / P.one_length_pulse)) < 0.01f);
    assert(stop_last.max_overshoot_mm >= stop_last.overshoot_mm);

    /* 명령 정지가 끊으면 쓰지 않는다 */
    setup(2000);
    D.stop_pulses = 600;
    D.flag_stopping = 1;
    stop_request_tag();
    tick(0);
    stop_request_rapid(STOP_REASON_COMMAND);
    assert(run_until_idle(0, 3000) < 3000);
    assert(stop_last.n == n0 + 2);
}

/* OPT_PAUSE_RESUME_SMOOTH: 감속 중 원인이 풀리면 0 까지 가지 않는다 */
static void pause_begin(int16_t opt)
{
    setup(3000);
    (void)param_set(P_OPT_FLAGS, opt);
    watch_reset();
}

static void test_smooth_resume(void)
{
    int t;

    /* 장애물이 0.3 s 비면 감속 중에 지금 지령에서 다시 간다 */
    pause_begin(OPT_PAUSE_RESUME_SMOOTH);
    D.ext_sen_on = 1;
    for (t = 0; t < 5; t++) { watch_tick(); }
    D.ext_sen_on = 0;
    for (t = 0; t < 15; t++) { watch_tick(); }                       /* 0.3 s 보다 짧은 깜빡임 */
    assert(stop_reason() == STOP_REASON_OBSTACLE && stop_phase() == STOP_PHASE_DECELERATE);
    D.ext_sen_on = 1;
    for (t = 0; t < 5; t++) { watch_tick(); }
    D.ext_sen_on = 0;
    for (t = 0; t < STOP_OBSTACLE_CLEAR_TICKS - 1; t++) { watch_tick(); }
    assert(stop_reason() == STOP_REASON_OBSTACLE && stop_phase() == STOP_PHASE_DECELERATE);
    watch_tick();
    assert(stop_phase() == STOP_PHASE_IDLE && D.step == DRIVE_CHANGE_SPEED && D.action_err == 0);
    for (t = 0; t < 1500; t++) { watch_tick(); }
    assert(s_zero == 0 && s_jump == 0 && s_flip == 0 && D.last_input == 3000);

    /* 일시정지(PAUSE)는 풀리는 즉시 */
    pause_begin(OPT_PAUSE_RESUME_SMOOTH);
    D.pause_stop = 1;
    for (t = 0; t < 20; t++) { watch_tick(); }
    D.pause_stop = 0;
    watch_tick();
    assert(stop_phase() == STOP_PHASE_IDLE);
    for (t = 0; t < 1500; t++) { watch_tick(); }
    assert(s_zero == 0 && s_jump == 0 && D.last_input == 3000);

    /* 태그 위치 정지가 끊긴 경우: 같은 목표로 지금 지령에서 다시 제동한다 */
    pause_begin(OPT_PAUSE_RESUME_SMOOTH);
    D.stop_pulses = 3000;
    D.flag_stopping = 1;
    stop_request_tag();
    for (t = 0; t < 10; t++) { watch_tick(); }
    D.pause_stop = 1;
    for (t = 0; t < 10; t++) { watch_tick(); }
    D.pause_stop = 0;
    watch_tick();
    assert(stop_reason() == STOP_REASON_TAG_DOCKING && D.last_input > 1000);
    while (D.flag_run && (s_zero == 0)) { watch_tick(); }
    assert(D.stop_pulses == 0 && s_jump == 0 && s_flip == 0);
    { long n = s_travel; assert(n >= 2998 && n <= 3002); }

    /* 비트 꺼짐: 기존대로 0 까지 서고 나서 다시 출발 */
    pause_begin(0);
    D.ext_sen_on = 1;
    for (t = 0; t < 5; t++) { watch_tick(); }
    D.ext_sen_on = 0;
    for (t = 0; (t < 3000) && (stop_phase() != STOP_PHASE_IDLE); t++) { watch_tick(); }
    assert(s_zero > 0 && D.step == DRIVE_CHANGE_SPEED);
    (void)param_set(P_OPT_FLAGS, 0);
}

int main(void)
{
    stop_ctrl_t c;
    stop_feedback_t fb;

    /* 판정 함수 */
    assert(stop_is_position(STOP_REASON_TAG_POSITION) && stop_is_position(STOP_REASON_TURN_ANGLE));
    assert(!stop_is_position(STOP_REASON_COMMAND) && !stop_is_position(STOP_REASON_NONE));
    assert(stop_is_pause(STOP_REASON_OBSTACLE) && stop_is_pause(STOP_REASON_TRAFFIC) && stop_is_pause(STOP_REASON_PAUSE));
    assert(!stop_is_pause(STOP_REASON_SWITCH));

    /* Begin: 비상은 곧바로 0, 위치 0 거리는 도착, 위치 바닥 미만 최대속은 실패 */
    stop_ctrl_begin(&c, STOP_REASON_EMERGENCY, 2000, 2000, 10, 20, 0, 0.002f);
    assert(c.phase == STOP_PHASE_WAIT_QUIET && c.command_rpm == 0);
    stop_ctrl_begin(&c, STOP_REASON_TAG_DOCKING, 2000, 2000, 10, 20, 0, 0.002f);
    assert(c.phase == STOP_PHASE_WAIT_QUIET);
    stop_ctrl_begin(&c, STOP_REASON_TAG_DOCKING, 200, 200, 10, 20, 100, 0.002f);
    assert(c.phase == STOP_PHASE_FAILED);
    stop_ctrl_begin(&c, STOP_REASON_TAG_DOCKING, 2000, 2000, 10, 20, 100, 0.0f);
    assert(c.phase == STOP_PHASE_FAILED);
    /* 회전은 감속 제한 dcc/4, 상한 10, 가속도 그 아래 */
    stop_ctrl_begin(&c, STOP_REASON_TURN_ANGLE, 0, 1000, 60, 60, 500, 0.004f);
    assert(c.phase == STOP_PHASE_CRUISE && c.decel_limit == 10 && c.accel_limit == 10);
    /* 일반 정지: 바닥 이하면 바로 0 */
    stop_ctrl_begin(&c, STOP_REASON_COMMAND, 300, 1000, 10, 20, 0, 0.002f);
    assert(c.phase == STOP_PHASE_WAIT_QUIET);
    /* 제동 계획 값 */
    stop_ctrl_begin(&c, STOP_REASON_TAG_DOCKING, 2000, 2000, 10, 20, 1000, 0.002f);
    assert(c.braking_start_rpm == 2000 && c.braking_start_pulses == 1000);
    assert(c.required_decel == 2000.0f * 2000.0f * 0.002f / (2.0f * 1000.0f * (1.0f - 0.1f * 0.5f)));

    /* 위치 진행 없음 300 틱 → 실패 */
    stop_ctrl_begin(&c, STOP_REASON_TAG_DOCKING, 400, 2000, 10, 20, 1000, 0.002f);
    memset(&fb, 0, sizeof(fb));
    fb.remaining_pulses = 1000;
    fb.tolerance_pulses = 2;
    fb.pulses_per_rpm_tick = 0.002f;
    for (int i = 0; i < 299; i++) { assert(stop_ctrl_update(&c, &fb) >= 0); assert(c.phase != STOP_PHASE_FAILED); }
    (void)stop_ctrl_update(&c, &fb);
    assert(c.phase == STOP_PHASE_FAILED);

    /* 태그 정지: 거리를 다 쓰고 완료, 정지 거리 0, 플래그 해제 */
    setup(2000);
    D.stop_pulses = 600;
    D.flag_stopping = 1;
    stop_request_tag();
    tick(0);
    assert(stop_reason() == STOP_REASON_TAG_DOCKING && stop_phase() == STOP_PHASE_DECELERATE);
    assert(run_until_idle(0, 5000) < 5000);
    assert(D.stop_pulses == 0 && D.flag_run == 0 && D.amp_run == 0);

    /* 바닥 학습: 접근 구간에 들어서 바퀴가 350 으로 안 돌면 후보를 올려 찾는다.
     * 제동 곡선 위에서 멈추면 원본도 학습 없이 300 틱 뒤 실패한다 */
    setup(2000);
    D.stop_pulses = 400;
    D.flag_stopping = 1;
    stop_request_tag();
    { int learned = 0;
      for (int t = 0; t < 6000 && D.flag_run; t++) {
          tick((stop_phase() >= STOP_PHASE_APPROACH) ? 420 : 0);
          if (stop_phase() == STOP_PHASE_LEARN_FLOOR) { learned = 1; }
          if (stop_floor_rpm() > STOP_MIN_RPM) { learned = 2; }
          assert(stop_phase() != STOP_PHASE_FAILED);
      }
      assert(learned == 2);
      assert(stop_floor_rpm() >= 425 && stop_floor_rpm() <= 500);
      assert(D.stop_pulses == 0 && D.flag_run == 0); }

    /* 끝내 안 돌면 277. 재시도하지 않는다 */
    setup(2000);
    D.stop_pulses = 400;
    D.flag_stopping = 1;
    stop_request_tag();
    { int t = 0;
      while ((D.stop_err == 0) && (t < 6000)) { tick(5000); t++; }
      assert(D.stop_err == ERR_STOP_APPROACH);
      assert(stop_phase() == STOP_PHASE_FAILED);
      assert(D.stop_pulses > 0);                      /* 277 은 거리를 지우지 않는다 */
      D.wait_start = 1;
      tick(5000);
      assert(D.wait_start == 0); }

    /* 요청 우선순위 */
    setup(0);
    stop_request_rapid(STOP_REASON_COMMAND);
    stop_request_rapid(STOP_REASON_OBSTACLE);       /* 일시정지가 명시적 정지를 못 낮춘다 */
    D.vin_out = D.last_input = 2000;
    (void)stop_run();
    assert(stop_reason() == STOP_REASON_COMMAND);
    setup(2000);
    stop_request_rapid(STOP_REASON_SWITCH);
    stop_request_rapid(STOP_REASON_COMMAND);
    (void)stop_run();
    assert(stop_reason() == STOP_REASON_SWITCH);
    setup(2000);
    stop_request_rapid(STOP_REASON_EMERGENCY);
    stop_request_rapid(STOP_REASON_SWITCH);
    (void)stop_run();
    assert(stop_reason() == STOP_REASON_EMERGENCY);
    /* 태그는 명시적 정지를 덮지 못한다 */
    setup(2000);
    stop_request_rapid(STOP_REASON_COMMAND);
    stop_request_tag();
    (void)stop_run();
    assert(stop_reason() == STOP_REASON_COMMAND);
    /* STOP 스위치가 무엇보다 앞선다 */
    setup(2000);
    D.stop_sw = 1;
    (void)stop_run();
    assert(stop_reason() == STOP_REASON_SWITCH);

    /* 회전 전 태그 사유 */
    setup(2000);
    D.turn_pulses = 100;
    D.turn_motion = MOTION_CW;
    stop_request_tag();
    D.stop_pulses = 300;
    (void)stop_run();
    assert(stop_reason() == STOP_REASON_BEFORE_TURN);

    /* 일시정지 중 정지 거리 보존: 장애물로 끊겨도 남은 거리와 바닥을 이어 쓴다 */
    setup(2000);
    D.obs_use = 1;
    D.stop_pulses = 1200;
    D.flag_stopping = 1;
    stop_request_tag();
    for (int i = 0; i < 40; i++) { tick(0); }
    assert(stop_reason() == STOP_REASON_TAG_DOCKING);
    D.obs_f = 1;
    D.ext_sen_on = 1;
    tick(0);
    assert(stop_reason() == STOP_REASON_OBSTACLE);
    { int t = 0;
      while ((stop_phase() != STOP_PHASE_COMPLETE) && (t < 3000)) { tick(0); t++; }
      assert(stop_phase() == STOP_PHASE_COMPLETE); }
    { int16_t left = D.stop_pulses;
      assert(left > 0);
      for (int i = 0; i < 500; i++) {
          tick(0);
          assert(D.stop_pulses == left);          /* 정지 내내 그대로 */
          assert(D.flag_run == 1);                /* 완료 처리가 돌지 않는다 */
          assert(D.action_err == ERR_OBSTACLE);
          assert(D.obs_on == 1);
      }
      D.obs_f = 0;
      D.ext_sen_on = 0;
      tick(0);
      assert(stop_reason() == STOP_REASON_TAG_DOCKING);
      assert(D.action_err == 0 && D.obs_on == 0);
      assert(D.stop_pulses == left); }
    assert(run_until_idle(0, 5000) < 5000);
    assert(D.stop_pulses == 0);

    /* 일시정지 중 영구 정지가 오면 재개하지 않는다 */
    setup(2000);
    D.stop_pulses = 1200;
    D.flag_stopping = 1;
    stop_request_tag();
    for (int i = 0; i < 20; i++) { tick(0); }
    D.pause_stop = 1;
    tick(0);
    stop_request_rapid(STOP_REASON_COMMAND);
    tick(0);
    assert(stop_reason() == STOP_REASON_COMMAND);

    /* 위치 목표 없는 일시정지 뒤 재출발 */
    setup(1500);
    D.pause_stop = 1;
    tick(0);
    assert(stop_reason() == STOP_REASON_PAUSE);
    { int t = 0;
      while ((stop_phase() != STOP_PHASE_COMPLETE) && (t < 3000)) { tick(0); t++; }
      assert(stop_phase() == STOP_PHASE_COMPLETE && D.flag_run == 1); }
    D.pause_stop = 0;
    tick(0);
    assert(stop_phase() == STOP_PHASE_IDLE && D.step == DRIVE_CHANGE_SPEED);

    /* 선택 비트: 재개 때 남은 거리가 길면 바닥 속도 위로 가속한다 */
    { int16_t peak_off = 0, peak_on = 0, k;
      for (k = 0; k < 2; k++) {
          setup(2000);
          (void)param_set(P_OPT_FLAGS, k ? OPT_RESUME_CRUISE : 0);   /* setup 이 EEPROM 을 지운다 */
          D.stop_pulses = 6000;
          D.flag_stopping = 1;
          stop_request_tag();
          for (int i = 0; i < 20; i++) { tick(0); }
          D.pause_stop = 1;
          for (int t = 0; t < 3000 && stop_phase() != STOP_PHASE_COMPLETE; t++) { tick(0); }
          D.pause_stop = 0;
          tick(0);
          { int16_t peak = 0;
            for (int t = 0; t < 400; t++) { tick(0); if (D.vin_out > peak) { peak = D.vin_out; } }
            if (k) { peak_on = peak; } else { peak_off = peak; } }
      }
      assert(peak_off <= STOP_MIN_RPM);
      assert(peak_on > STOP_MIN_RPM + 100);
      (void)param_set(P_OPT_FLAGS, 0); }

    /* 위험 감속 표시는 일시정지 완료·정지 완료에서 내려간다 */
    setup(1500);
    D.risk_stop = 1;
    D.pause_stop = 1;
    for (int t = 0; t < 3000 && stop_phase() != STOP_PHASE_COMPLETE; t++) { tick(0); }
    assert(D.risk_stop == 0);
    setup(1500);
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_RISK_FLAG);
    D.risk_stop = 1;
    D.pause_stop = 1;
    for (int t = 0; t < 3000 && stop_phase() != STOP_PHASE_COMPLETE; t++) { tick(0); }
    assert(D.risk_stop == 1);
    (void)param_set(P_LEGACY_QUIRKS, 0);

    /* 회전 각 제어 */
    setup(0);
    D.control = RUN_TURN;
    D.flag_run = 0;
    D.flag_turn = 1;
    D.turn_motion = MOTION_CW;
    D.turn_pulses = 800;
    tick(0);
    assert(stop_reason() == STOP_REASON_TURN_ANGLE && stop_phase() == STOP_PHASE_CRUISE);
    assert(D.motion == MOTION_CW && D.amp_dir == MOTION_CW);
    { int t = 0;
      while (D.flag_turn && (t < 5000)) { tick(0); t++; }
      assert(!D.flag_turn && D.turn_pulses == 0 && D.turn_motion == 0); }

    /* RFID 즉시 정지 */
    setup(1500);
    D.stop_pulses = 1000;
    D.flag_stopping = 1;
    stop_request_tag();
    tick(0);
    D.immediate_stop = 1;
    tick(0);
    assert(D.stop_pulses == 0 && stop_phase() == STOP_PHASE_WAIT_QUIET && D.vin_out == 0);

    /* 제동 중 Add_speed 는 바닥 위 여유 안으로 묶인다 */
    setup(2000);
    D.stop_pulses = 600;
    D.flag_stopping = 1;
    stop_request_tag();
    tick(0);
    D.add_speed_l = 3000;
    D.add_speed_r = -3000;
    motor_velocity(1000);
    assert(D.add_speed_l == 1000 - stop_floor_rpm() && D.add_speed_r == -(1000 - stop_floor_rpm()));

    /* 정렬 속도: 바닥 아래 목표는 350 으로 */
    setup(0);
    assert(stop_alignment_speed(0) == 0);
    D.vin_out = 340;
    assert(stop_alignment_speed(100) > 340);

    test_ramp_continuity();
    test_traffic();
    test_smooth_resume();
    test_stop_last();
    puts("test_stop ok");
    return 0;
}
