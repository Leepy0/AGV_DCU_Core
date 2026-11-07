/* test_stop.c - MotorStop 상태기계. 원본과 같은 궤적인지는 앱 저장소 비교 시험이 맡는다 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "agv/motor.h"
#include "agv/steer.h"
#include "agv/stop.h"
#include "fakes.h"

#define D agv_drive
#define P param_dv

static long s_left_acc[2];

static void setup(int16_t rpm)
{
    fake_reset();
    fake_params();
    memset(&D, 0, sizeof(D));
    motor_reset();
    stop_reset();
    steer_reset();
    memset(s_left_acc, 0, sizeof(s_left_acc));
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
        s_left_acc[i] += rpm * 12;
        while (s_left_acc[i] >= 6000) {
            s_left_acc[i] -= 6000;
            motor_encoder_pulse(i);
            D.period[i] = 500000000L / (rpm ? rpm : 1);
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

    puts("test_stop ok");
    return 0;
}
