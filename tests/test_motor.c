/* test_motor.c - rpm 피드백, 속도 PI, S커브, 주행 단계 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "agv/branch.h"
#include "agv/motor.h"
#include "agv/steer.h"
#include "agv/stop.h"
#include "fakes.h"

#define D agv_drive
#define P param_dv

static void setup(void)
{
    fake_reset();
    fake_params();
    memset(&D, 0, sizeof(D));
    motor_reset();
    stop_reset();
    steer_reset();
    D.ui_mode = UI_AUTO;
    D.control = RUN_DRIVE;
    D.step = DRIVE_CHANGE_SPEED;
    D.flag_run = 1;
    D.speed_cmd = P.low_speed;
}

int main(void)
{
    int16_t v, a;

    /* 피드백: 주기 → rpm, 평균, IIR */
    setup();
    D.period[0] = 500000;
    D.period[1] = 250000;
    for (int i = 0; i < 6; i++) {
        motor_encoder_pulse(0);
        motor_encoder_pulse(1);
        motor_feedback_update();
        assert(motor_idle_ticks(0) == 0 && motor_fresh_pulse(1) == 1);
    }
    assert(D.rpm[0] == 1000 && D.rpm[1] == 2000);
    assert(D.rpm_avg[0] == 1000 && D.rpm_avg[1] == 2000);
    /* 펄스가 끊기면 경과 틱 상한으로 내려간다 */
    motor_feedback_update();
    assert(motor_idle_ticks(0) == 1 && motor_fresh_pulse(0) == 0);
    assert(D.rpm[0] == 500);            /* min(1000, 500/1) */
    motor_feedback_update();
    assert(D.rpm[0] == 250);
    for (int i = 0; i < 12; i++) { motor_feedback_update(); }
    assert(motor_idle_ticks(0) == 14 && D.rpm[0] == 35);
    motor_feedback_update();
    assert(D.rpm[0] == 0);              /* QUIET 15 틱 */
    /* 스파이크는 직전값 유지. 3500 은 통과 */
    motor_encoder_pulse(0);
    D.period[0] = 142857;               /* 3500 */
    motor_feedback_update();
    assert(D.rpm[0] == 3500);
    motor_encoder_pulse(0);
    D.period[0] = 142000;
    motor_feedback_update();
    assert(D.rpm[0] == 3500);
    /* 주기 0 이면 0 */
    motor_encoder_pulse(0);
    D.period[0] = 0;
    motor_feedback_update();
    assert(D.rpm[0] == 0);
    /* 유휴 카운터 상한 */
    for (int i = 0; i < 1100; i++) { motor_feedback_update(); }
    assert(motor_idle_ticks(0) == 1000);

    /* 속도 PI: 게인, 토크 상한 4000, 음수 없음 */
    setup();
    D.control = RUN_TURN;
    motor_speed_pi(100);
    assert(D.w_cmd[0] == 100 && D.w_err[0] == 100);
    assert(D.iref_it[0] == 0.2f * 100.0f);
    assert(D.tq_cmd[0] == (int16_t)(2.2f * 100.0f + D.iref_it[0]));
    for (int i = 0; i < 3000; i++) { motor_speed_pi(3000); }
    assert(D.tq_cmd[0] == 4000 && D.tq_cmd[1] == 4000 && D.w_pi_out[0] == 4000.0f);
    /* 포화 해소 뒤 적분이 묶여 있어 곧 내려온다 (anti-windup) */
    D.rpm_avg[0] = 3000;
    for (int i = 0; i < 200; i++) { motor_speed_pi(10); }
    assert(D.tq_cmd[0] == 0);
    /* 지령 0 이면 PI 초기화 */
    motor_speed_pi(0);
    assert(D.iref_it[0] == 0.0f && D.w_pi_out[0] == 0.0f && D.w_err[0] == 0);
    /* 주행은 Add_speed 합산, 음수는 0 */
    D.control = RUN_DRIVE;
    D.add_speed_l = 50;
    D.add_speed_r = -300;
    motor_speed_pi(200);
    assert(D.w_cmd[0] == 250 && D.w_cmd[1] == 0);
    /* 준비 상태는 w_cmd 를 건드리지 않는다 */
    D.control = RUN_READY;
    D.w_cmd[0] = 77;
    motor_speed_pi(200);
    assert(D.w_cmd[0] == 77);

    /* 조그 분기 */
    setup();
    D.control = RUN_JOG;
    D.jog_bits = 0xB;
    D.add_speed_l = 0;
    D.add_speed_r = 120;
    D.rpm_avg[0] = 55;
    motor_speed_pi(500);
    assert(D.w_cmd[0] == 0 && D.rpm_avg[0] == 0 && D.w_cmd[1] == 120);
    D.jog_bits = 0x7;
    D.add_speed_l = 90;
    D.add_speed_r = 0;
    motor_speed_pi(500);
    assert(D.w_cmd[0] == 90 && D.w_cmd[1] == 0);
    D.jog_bits = 0xE;
    D.add_speed_l = 10;
    D.add_speed_r = 20;
    motor_speed_pi(500);
    assert(D.w_cmd[0] == 510 && D.w_cmd[1] == 520);
    D.jog_left_only = 1;
    D.jog_stopping = 1;
    motor_speed_pi(500);
    assert(D.w_cmd[0] == 0 && D.w_cmd[1] == 520);

    /* S커브: 저속 가속 상한 5, 목표 도달, 오버슈트 없음, 가속도 연속 */
    setup();
    v = 0; a = 0;
    { int16_t prev_a = 0;
      for (int i = 0; i < 5000 && v != 2000; i++) {
          int16_t before = v;
          motor_scv_core(2000, &v, 60, 60, 500, 300, 300, 1, &a);
          assert(v <= 2000 && v >= before);
          if (v != 2000) { assert((a - prev_a <= 7) && (a - prev_a >= -7)); }     /* jerk = 60/8 */
          if (before <= 500) { assert(a <= 5); }
          prev_a = a;
      }
      assert(v == 2000 && a == 0); }
    /* 감속 저속 구간 상한 20, Vin 0 이면 바닥 아래에서 0 */
    { int16_t max_dec_low = 0;
      for (int i = 0; i < 5000 && v != 0; i++) {
          int16_t before = v;
          motor_scv_core(0, &v, 60, 60, 500, 300, 300, 1, &a);
          if (before <= 600 && (before - v) > max_dec_low && v != 0) { max_dec_low = (int16_t)(before - v); }
          assert(v >= 0);
      }
      assert(v == 0 && a == 0);
      assert(max_dec_low <= 60); }
    /* 바닥: |v| < stop_floor 이고 목표 0 이면 즉시 0 */
    v = 299; a = -3;
    motor_scv_core(0, &v, 10, 20, 500, 100, 300, 1, &a);
    assert(v == 0 && a == 0);
    /* 후진 값도 절댓값으로 비교 */
    v = -299; a = 0;
    motor_scv_core(0, &v, 10, 20, 500, 100, 300, 1, &a);
    assert(v == 0);
    /* 정지 상태에서 가속도 리셋 */
    v = 0; a = 40;
    motor_scv_core(100, &v, 8, 20, 500, 100, 0, 1, &a);
    assert(a == 1);

    /* 주행 단계: 400 → 401 정속 */
    setup();
    for (int i = 0; i < 5000 && D.step == DRIVE_CHANGE_SPEED; i++) { motor_drive_step(); }
    assert(D.step == DRIVE_CRUISE && D.vin_out == D.speed_cmd);
    motor_drive_step();
    assert(D.last_input == D.speed_cmd && D.scv_accel == 0);

    /* 위험 감속 404: 트리거, 목표 Low×0.7, 재정렬 10틱, 405, 플래그 해제 */
    D.guide_pos = 10;
    motor_drive_step();
    assert(D.step == DRIVE_RISK_STOP && D.risk_stop == 1);
    for (int i = 0; i < 2000; i++) { motor_drive_step(); }
    assert(D.vin_out == (int16_t)((float)P.low_speed * 0.7f));
    D.guide_pos = 3;
    for (int i = 0; i < 9; i++) { motor_drive_step(); assert(D.step == DRIVE_RISK_STOP); }
    D.guide_pos = 4;
    motor_drive_step();
    assert(D.guide_realign_cnt == 0);          /* 끊기면 다시 센다 */
    D.guide_pos = -3;
    for (int i = 0; i < 10; i++) { motor_drive_step(); }
    assert(D.step == DRIVE_RISK_RESUME);
    D.guide_pos = 0;
    for (int i = 0; i < 2000 && D.step == DRIVE_RISK_RESUME; i++) { motor_drive_step(); }
    assert(D.step == DRIVE_CRUISE && D.risk_stop == 0 && D.vin_out == D.speed_cmd);

    /* 트리거 조건: 저속이면 위험 감속 안 함 */
    setup();
    D.vin_out = (int16_t)(P.llow_speed - 1);
    D.guide_pos = 12;
    motor_drive_step();
    assert(D.step != DRIVE_RISK_STOP);
    /* 정지 중이면 안 함 */
    setup();
    D.vin_out = P.low_speed;
    D.flag_stopping = 1;
    D.guide_pos = 12;
    motor_drive_step();
    assert(D.step != DRIVE_RISK_STOP);

    /* 장애물: 402 진입과 281, 해제 뒤 재출발 */
    setup();
    D.step = DRIVE_CRUISE;
    D.obs_use = 1;
    D.vin_out = D.last_input = P.low_speed;
    D.obs_f = 1;
    motor_drive_step();
    assert(D.step == DRIVE_OBS_STOP && D.obs_on == 1 && D.action_err == ERR_OBSTACLE);
    assert(stop_reason() == STOP_REASON_OBSTACLE);
    /* 전진 중 후방 센서는 무시 */
    setup();
    D.obs_use = 1;
    D.obs_b = 1;
    motor_drive_step();
    assert(D.obs_on == 0);
    /* 교차로 경보 285, ACK 오류 286, 일시정지 290 */
    setup();
    D.cross_in_on = 1;
    motor_drive_step();
    assert(D.action_err == ERR_CROSS_WAIT);
    setup();
    D.cross_in_on = 1;
    D.cross_ack_error = 1;
    motor_drive_step();
    assert(D.action_err == ERR_CROSS_NO_ACK);
    setup();
    D.pause_stop = 1;
    motor_drive_step();
    assert(D.action_err == ERR_PAUSE);

    /* 정렬 회전 Step 2 */
    setup();
    D.control = RUN_TURN;
    D.tape_step = 2;
    D.tape_align = 1;
    D.turn_pulses = 2000;
    D.tape_turned = 1900;
    D.tape_target_cnt = 2000.0f;
    D.tape_max = 100;
    D.guide_pos = -3;
    (void)motor_turn_adjust();
    assert(D.tape_align == 1);                    /* -3 은 중앙이 아니다 */
    D.guide_pos = 1;
    (void)motor_turn_adjust();
    assert(D.tape_align == 0 && D.turn_pulses == 0);
    /* 일반 회전은 가이드를 보지 않는다 */
    setup();
    D.control = RUN_TURN;
    D.turn_pulses = 500;
    D.guide_all_off = 1;
    for (int i = 0; i < 20; i++) { (void)motor_turn_adjust(); }
    assert(D.turn_pulses == 500 && D.tape_step == 0);
    /* Step 2 동안 최저 속도 쪽으로 감속한다 */
    setup();
    D.tape_step = 2; D.tape_align = 1; D.turn_pulses = 2000; D.tape_turned = 1900;
    D.tape_target_cnt = 2000.0f; D.tape_max = 100;
    D.guide_pos = 5;
    D.vin_out = P.turn_speed;
    for (int i = 0; i < 2000; i++) { (void)motor_turn_adjust(); }
    assert(D.vin_out == ((P.turn_min > STOP_MIN_RPM) ? P.turn_min : STOP_MIN_RPM));

    /* motor_run: 플래그가 없으면 리셋 분기. 위험 표시 해제, 비트를 켜면 남는다 */
    setup();
    D.flag_run = 0;
    D.risk_stop = 1;
    D.stop_pulses = 100;
    D.tq_cmd[0] = 500;
    motor_run();
    assert(D.risk_stop == 0 && D.stop_pulses == 0 && D.tq_cmd[0] == 0 && D.speed_cmd == P.low_speed);
    assert(D.step == DRIVE_CHANGE_SPEED);
    setup();
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_RISK_FLAG);
    D.flag_run = 0;
    D.risk_stop = 1;
    motor_run();
    assert(D.risk_stop == 1);
    (void)param_set(P_LEGACY_QUIRKS, 0);

    /* 시스템 오류면 비상 정지로 들어간다 */
    setup();
    D.system_err = 200;
    motor_run();
    assert(stop_reason() == STOP_REASON_EMERGENCY && D.tq_cmd[0] == 0 && D.flag_run == 0);

    /* 277 은 정지 거리를 보존한다 */
    setup();
    D.flag_run = 0;
    D.stop_err = ERR_STOP_APPROACH;
    D.stop_pulses = 55;
    motor_run();
    assert(D.stop_pulses == 55);

    /* 분기: 준비 → 시작 거리 → 4-LED 창 세 번 → 중앙 복귀 지령 → 종료 */
    setup();
    (void)param_set(P_BRANCH_OFFSET, 54);
    D.branch_ready = 1;
    D.cnt_branch_l = P.tag_pulse[8];
    D.cnt_branch_r = 0;
    D.tag_l = 1;                                   /* 태그 센서 위에서는 거리를 다시 잰다 */
    branch_run();
    assert(D.branch_start == 0 && D.cnt_branch_l == 0);
    D.tag_l = 0;
    D.cnt_branch_l = (int16_t)(P.tag_pulse[8] + 1);
    D.guide_led = 0xFFFF;
    branch_run();
    assert(D.branch_start == 1 && D.branch_ready == 0 && D.p_cmd == -4 && D.guide_led == 0);
    { static const uint16_t zone[3] = { 0xF000, 0x0F00, 0x00F0 };
      for (int z = 0; z < 3; z++) {
          D.guide_led = (uint16_t)(zone[z] >> 0);
          for (int i = 0; i < 9; i++) { branch_run(); assert(D.branch_step == z); }
          branch_run();
          assert(D.branch_step == z + 1);
      } }
    assert(D.p_cmd == 1);
    branch_run();
    assert(D.branch_start == 0 && D.p_cmd == 0);
    /* 오른쪽 창, wide */
    setup();
    D.branch_start = 2;
    D.guide_wide = 1;
    D.guide_led32 = 0x00000F00UL;
    for (int i = 0; i < 10; i++) { branch_run(); }
    assert(D.branch_step == 1 && D.p_cmd == 4);
    /* 거리 초과와 잘못된 방향은 265 */
    setup();
    D.branch_start = 1;
    D.cnt_branch_r = (int16_t)(P.straight_len + 1);
    branch_run();
    assert(D.stop_err == ERR_BRANCH_TIMEOUT && D.flag_stopping == 1 && D.branch_start == 0);
    setup();
    D.branch_ready = 7;
    branch_run();
    assert(D.stop_err == ERR_BRANCH_TIMEOUT);

    /* 소리, 앰프, 방향 */
    setup();
    motor_run();
    assert(D.amp_run == 1 && D.amp_dir == MOTION_FWD);
    D.backward = 1;
    D.vin_out = 100;
    motor_run();
    assert(D.amp_dir == MOTION_BACK && D.sound_on == 1);

    puts("test_motor ok");
    return 0;
}
