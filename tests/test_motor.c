/* test_motor.c - rpm 피드백, 속도 PI, S커브, 주행 단계 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "agv/branch.h"
#include "agv/guide.h"
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
    { guide_cfg_t gc; guide_cfg_default(&gc); guide_configure(&gc); }   /* h 배율 1(부팅과 같게) */
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

    /* S커브 예전 저크(SCV_JERK_OLD = QUIRK_JERK_OLD, 레거시와 같은 출력): 저속 가속 상한 ACC/2, 목표 도달, 오버슈트 없음, 가속도 연속 */
    setup();
    v = 0; a = 0;
    { int16_t prev_a = 0;
      for (int i = 0; i < 5000 && v != 2000; i++) {
          int16_t before = v;
          motor_scv_core(2000, &v, 60, 60, 500, 300, 300, SCV_JERK_OLD, &a);
          assert(v <= 2000 && v >= before);
          if (v != 2000) { assert((a - prev_a <= 7) && (a - prev_a >= -7)); }     /* jerk = 60/8 */
          if (before <= 500) { assert(a <= 30); }
          prev_a = a;
      }
      assert(v == 2000 && a == 0); }
    /* 감속 저속 구간 상한 20, Vin 0 이면 바닥 아래에서 0 */
    { int16_t max_dec_low = 0;
      for (int i = 0; i < 5000 && v != 0; i++) {
          int16_t before = v;
          motor_scv_core(0, &v, 60, 60, 500, 300, 300, SCV_JERK_OLD, &a);
          if (before <= 600 && (before - v) > max_dec_low && v != 0) { max_dec_low = (int16_t)(before - v); }
          assert(v >= 0);
      }
      assert(v == 0 && a == 0);
      assert(max_dec_low <= 60); }
    /* 바닥 아래 0 으로 한 번에(예전 계단)는 v0.40.0 에서 뺐다: 예전 저크도 감속도를 풀며 0 에 닿는다 */
    v = 299; a = -3;
    motor_scv_core(0, &v, 10, 20, 500, 100, 300, SCV_JERK_OLD, &a);
    assert(v > 0 && v < 299);
    /* 정지 상태에서 가속도 리셋 */
    v = 0; a = 40;
    motor_scv_core(100, &v, 8, 20, 500, 100, 0, SCV_JERK_OLD, &a);
    assert(a == 1);
    /* 저속 구간 가속 한계 max(5, ACC/2): ACC 50·LOS 31 에서 0 → 400 rpm 이 예전(5) 의 1/3 이하, 첫 틱은 저크로 오른다 */
    { int16_t llow = (int16_t)(31.0f * P.one_meter_rpm / 2.0f);
      int t_new = 0, t_old = 0;
      assert(llow > 400);
      v = 0; a = 0;
      for (; t_new < 1000 && v < 400; t_new++) {
          int16_t prev_a = a;
          motor_scv_core(2000, &v, 50, 50, llow, 300, 300, SCV_JERK_OLD, &a);
          assert(a - prev_a <= 50 / 2 / 8 && a <= 25);   /* 저크 = (ACC/2)/8, 급발진 없음 */
          if (t_new == 0) { assert(a <= 3); }
      }
      v = 0; a = 0;
      for (; t_old < 1000 && v < 400; t_old++) { motor_scv_core(2000, &v, 5, 50, llow, 300, 300, SCV_JERK_OLD, &a); }
      printf("scv 0->400 rpm: ACC 50 %d ticks, low 5 %d ticks\n", t_new, t_old);
      assert(v >= 400 && t_new * 3 <= t_old);
      /* 작은 ACC(4)는 그대로 4 */
      v = 0; a = 0;
      for (int i = 0; i < 200; i++) { motor_scv_core(2000, &v, 4, 50, llow, 300, 300, SCV_JERK_OLD, &a); assert(a <= 4); }
      assert(a == 4);
      /* LG b10(QUIRK_LOW_ACC): 예전처럼 저속 구간 5 */
      assert(param_set(P_LEGACY_QUIRKS, QUIRK_LOW_ACC) == 0);
      v = 0; a = 0;
      for (int i = 0; i < 40; i++) { motor_scv_core(2000, &v, 50, 50, llow, 300, 300, SCV_JERK_OLD, &a); assert(a <= 5); }
      assert(param_set(P_LEGACY_QUIRKS, 0) == 0); }

    /* 새 저크(기본): 2.4 rpm/틱² 고정, DCC 는 상한. 31 m/min(2930 rpm) 은 25틱 만에 60, 6 m/min(567) 은 상한 전에 풀려 √(2.4 × 567) ≈ 37,
     * 목표 0 이면 바닥에서 뛰지 않고 감속도를 풀며 0 에 닿는다(stop_floor 300 이어도). QUIRK_JERK_OLD 면 예전과 같다 */
    for (int q = 0; q < 1; q++) {          /* v0.40.0: LG b11 의 예전 저크는 뺐다(q 1 은 돌지 않는다) */
        static const int16_t dv[3] = { 567, 2930, 189 };
        if (q) { assert(param_set(P_LEGACY_QUIRKS, QUIRK_JERK_OLD) == 0); }
        for (int c = 0; c < 3; c++) {
            int16_t peak = 0, prev_a = 0, jmax = 0, last = 0;
            int t = 0, t60 = -1;
            v = dv[c]; a = 0;
            for (; (t < 500) && (v != 0); t++) {
                int16_t before = v;
                motor_scv_core(0, &v, 60, 60, 500, 300, 300, 0, &a);
                assert(v >= 0 && v <= before);
                if (abs(a - prev_a) > jmax) { jmax = (int16_t)abs(a - prev_a); }
                if (-a > peak) { peak = (int16_t)(-a); }
                if ((t60 < 0) && (a == -60)) { t60 = t; }
                last = (int16_t)(before - v);
                prev_a = a;
            }
            printf("scv %s %4d -> 0 rpm: %d ticks, peak %d, jerk %d, end step %d\n", q ? "old" : "new", dv[c], t, peak, jmax, last);
            assert(v == 0 && a == 0);
            if (q) { assert(last > 100); continue; }       /* 예전: 바닥(300) 아래에서 0 으로 한 번에 */
            assert(jmax <= 3 && last <= 3);
            if (c == 0) { assert(peak >= 34 && peak <= 41); }
            if (c == 1) { assert(peak == 60 && t60 >= 23 && t60 <= 26); }
        }
        /* 가속도 같다: 0 → 2000, 저속 구간(llow 500 아래) 상한 ACC/2 = 30 */
        v = 0; a = 0;
        { int16_t prev_a = 0;
          for (int i = 0; (i < 500) && (v != 2000); i++) {
              int16_t before = v;
              motor_scv_core(2000, &v, 60, 60, 500, 300, 300, 0, &a);
              assert(v >= before && v <= 2000 && a - prev_a <= (q ? 7 : 3));
              if (before <= 500) { assert(a <= 30); }
              prev_a = a;
          } }
        assert(v == 2000 && a == 0);
    }
    assert(param_set(P_LEGACY_QUIRKS, 0) == 0);
    /* 주행 저크 = min(2.4, 예전 한계/8): 0 ↔ 2930 rpm(31 m/min), llow 500. 어느 한계에서도 최고 저크가 예전 이하 */
    { static const int16_t lims[7] = { 8, 20, 37, 58, 20, 40, 60 };
      for (int c = 0; c < 7; c++) {
          int16_t jm[2] = { 0, 0 }, dec = (int16_t)(c >= 4);
          for (int q = 0; q < 2; q++) {
              int16_t prev_a = 0;
              v = dec ? 2930 : 0; a = 0;
              for (int i = 0; (i < 3000) && (v != (dec ? 0 : 2930)); i++) {
                  motor_scv_core(dec ? 0 : 2930, &v, lims[c], lims[c], 500, 300, 0, (int16_t)(q ? SCV_JERK_OLD : 0), &a);
                  if ((v != (dec ? 0 : 2930)) && (abs(a - prev_a) > jm[q])) { jm[q] = (int16_t)abs(a - prev_a); }
                  prev_a = a;
              }
              assert(v == (dec ? 0 : 2930));
          }
          printf("scv jerk %s %2d: old %d new %d\n", dec ? "DCC" : "ACC", lims[c], jm[1], jm[0]);
          assert(jm[0] <= jm[1]);
      } }

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
    /* 위험 감속은 예전 저크(DCC 60/8 = 7 rpm/틱²). 새 저크면 4틱 뒤 -9 */
    assert(D.scv_accel == -7);
    for (int i = 0; i < 3; i++) { motor_drive_step(); }
    assert(D.scv_accel == -28);
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

    /* 지령이 Low×0.7 보다 낮으면(예: Low 31, 지령 20 m/min) 위험 감속이 속도를 올리지 않고, 편차가 돌아오면 풀린다 */
    setup();
    D.speed_cmd = (int16_t)((long)P.low_speed * 20 / 31);
    assert(D.speed_cmd < (int16_t)((float)P.low_speed * 0.7f) && D.speed_cmd >= P.llow_speed);
    for (int i = 0; i < 5000 && D.step != DRIVE_CRUISE; i++) { motor_drive_step(); }
    assert(D.step == DRIVE_CRUISE && D.vin_out == D.speed_cmd);
    D.guide_pos = 12;
    motor_drive_step();
    assert(D.step == DRIVE_RISK_STOP && D.risk_stop == 1);
    for (int i = 0; i < 2000; i++) { motor_drive_step(); assert(D.vin_out <= D.speed_cmd && D.step == DRIVE_RISK_STOP); }
    assert(D.vin_out == D.speed_cmd);
    D.guide_pos = 2;
    for (int i = 0; i < 2000 && D.step != DRIVE_CRUISE; i++) { motor_drive_step(); assert(D.vin_out <= D.speed_cmd); }
    assert(D.step == DRIVE_CRUISE && D.risk_stop == 0 && D.vin_out == D.speed_cmd);

    /* 위험 감속 중 선을 잃으면 위치 0 이라도 재정렬로 세지 않는다(선 없이 재가속하지 않는다) */
    setup();
    for (int i = 0; i < 5000 && D.step != DRIVE_CRUISE; i++) { motor_drive_step(); }
    D.guide_pos = 12;
    motor_drive_step();
    assert(D.step == DRIVE_RISK_STOP);
    D.guide_pos = 0;
    D.guide_all_off = 1;
    for (int i = 0; i < 100; i++) { motor_drive_step(); assert(D.step == DRIVE_RISK_STOP && D.guide_realign_cnt == 0); }
    D.guide_all_off = 0;
    for (int i = 0; i < 10; i++) { motor_drive_step(); }
    assert(D.step == DRIVE_RISK_RESUME);

#if AGV_START_ALIGN
    /* 출발 정렬 단계: 속도 상한 = |위치| ≤ 3 LLOW, ≥ 10·선 없음 SPN, 사이는 곧게(늘 때는 2단위 늦게). 지령은 그대로, 위험 감속에 안 들어간다 */
    {
        static const struct { int16_t pos, off, num; } st[6] = {
            { 12, 0, -1 }, { 6, 0, 3 }, { 3, 0, 0 }, { 5, 0, 0 }, { 7, 0, 2 }, { 0, 1, -1 } };   /* num: (편차 - 3) 칸, -1 = SPN */
        setup();
        D.start_align = DRIVE_ALIGN_HOLD;
        D.vin_out = D.last_input = P.llow_speed;
        D.guide_pos = 12;
        motor_drive_step();
        assert((D.step != DRIVE_RISK_STOP) && !D.risk_stop);
        for (int k = 0; k < 6; k++) {
            int16_t want = (st[k].num < 0) ? P.min_speed
                         : (int16_t)(P.llow_speed - (int32_t)(P.llow_speed - P.min_speed) * st[k].num / 7);
            D.guide_pos = st[k].pos;
            D.guide_all_off = st[k].off;
            for (int i = 0; i < 2000; i++) { motor_drive_step(); assert((D.step != DRIVE_RISK_STOP) && (D.vin_out <= D.speed_cmd)); }
            assert((D.vin_out == want) && (D.speed_cmd == P.low_speed));
        }
        D.guide_all_off = 0;
        D.guide_pos = 0;
        D.start_align = 0;      /* 단계가 끝나면 지령까지 */
        for (int i = 0; i < 2000; i++) { motor_drive_step(); }
        assert(D.vin_out == P.low_speed);
    }
#endif

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
    D.vin_out = P.turn_min;
    assert(motor_turn_adjust() == 0 && D.vin_out == 0);   /* 중앙에 든 틱에 지령 0 */
    assert(D.tape_step == TAPE_STEP_SETTLE && D.tape_align == 1 && D.turn_pulses == 2000);   /* 회전은 아직 안 끝났다 */
    /* 정지 뒤 보정. side = 새 라인이 다가온 쪽. 정착 뒤 편차 pos 를 보고 한 단계씩 */
    {
        static const struct { int16_t pos, rev; } fx[2] = { { 2, 0 }, { -2, 1 } };   /* 덜 옴 → 같은 방향, 지나침 → 반대 */
        for (int c = 0; c < 2; c++) {
            int32_t t0;
            setup();
            D.control = RUN_TURN;
            D.tape_step = 2;
            D.tape_align = 1;
            D.turn_90 = 900;                   /* 1도 10 펄스 */
            D.turn_pulses = 2000;
            D.tape_turned = t0 = 1900;
            D.tape_target_cnt = D.tape_end_cnt = 2000.0f;
            D.tape_max = 100;
            D.tape_fix_end = -1;
            D.guide_pos = 4;
            assert(motor_turn_adjust() > 0 && D.tape_side == 1);
            D.guide_pos = 0;
            assert(motor_turn_adjust() == 0);
            D.guide_pos = fx[c].pos;           /* 서는 동안 미끄러졌다 */
            for (int i = 0; i < 19; i++) { assert(motor_turn_adjust() == 0 && D.tape_step == TAPE_STEP_SETTLE); }
            D.rpm[0] = 30;                     /* 바퀴가 아직 돈다: 기다린다 */
            assert(motor_turn_adjust() == 0 && D.tape_step == TAPE_STEP_SETTLE);
            D.rpm[0] = 0;
            assert(motor_turn_adjust() > 0);
            assert(D.tape_step == TAPE_STEP_FIX && D.tape_fix_rev == fx[c].rev && D.tape_fix_cnt == 1);
            assert(D.tape_stop_pos == fx[c].pos && D.tape_align == 1 && D.turn_pulses == 2000);
            for (int i = 0; i < 30; i++) { assert(motor_turn_adjust() <= (P.turn_min > STOP_MIN_RPM ? P.turn_min : STOP_MIN_RPM)); }
            D.tape_fix_turned = 20;            /* 2도 */
            D.guide_pos = 0;                  /* 가운데 0 칸에서 선다(v0.35.0. 예전은 ±1단위) */
            assert(motor_turn_adjust() == 0 && D.vin_out == 0 && D.tape_step == TAPE_STEP_SETTLE);
            for (int i = 0; i < 19; i++) { (void)motor_turn_adjust(); }
            assert(D.tape_align == 1);
            assert(motor_turn_adjust() == 0);
            assert(D.tape_align == 0 && D.turn_pulses == 0 && D.tape_fix_rev == 0);
            assert(D.tape_fix_end == 0 && D.tape_fix_cnt == 1 && D.tape_final_pos == 0);
            assert(D.tape_turned == t0 && D.system_err == 0);
        }
    }
    /* 보정 상한: 두 번 보정해도 중앙 밖이면 오류 없이 끝(1). 보정 각 5도를 넘으면 서고 끝(2). 보정 중 선을 잃으면 서고 끝(3) */
    for (int c = 1; c <= 3; c++) {
        setup();
        D.control = RUN_TURN;
        D.tape_step = TAPE_STEP_SETTLE;
        D.tape_align = 1;
        D.turn_90 = 900;
        D.turn_pulses = 2000;
        D.tape_side = 1;
        D.tape_fix_end = -1;
        D.guide_pos = 3;
        for (int k = 0; (k < 10) && D.tape_align; k++) {
            for (int i = 0; (i < 40) && (D.tape_step == TAPE_STEP_SETTLE); i++) { (void)motor_turn_adjust(); }
            if (!D.tape_align) { break; }
            assert(D.tape_step == TAPE_STEP_FIX && motor_turn_adjust() > 0);
            if (c == 1) {                     /* 중앙에 갔다가 서는 동안 다시 3 */
                D.guide_pos = 0;
                assert(motor_turn_adjust() == 0);
                D.guide_pos = 3;
            }
            else if (c == 2) {
                D.tape_fix_turned = 50;
                assert(motor_turn_adjust() > 0);
                D.tape_fix_turned = 51;
                assert(motor_turn_adjust() == 0 && D.vin_out == 0);
            }
            else {      /* 일시정지 뒤 재개처럼 몇 틱 선이 없다가 돌아오면 보정을 잇는다. 10틱 이어지면 끝 */
                D.guide_all_off = 1;
                for (int i = 0; i < 9; i++) { assert(motor_turn_adjust() > 0 && D.tape_step == TAPE_STEP_FIX); }
                D.guide_all_off = 0;
                assert(motor_turn_adjust() > 0);
                D.guide_all_off = 1;
                for (int i = 0; i < 9; i++) { assert(motor_turn_adjust() > 0 && D.tape_step == TAPE_STEP_FIX); }
                assert(motor_turn_adjust() == 0 && D.vin_out == 0 && D.tape_align == 1 && D.tape_fix_end == 3);
            }
        }
        assert(D.tape_align == 0 && D.turn_pulses == 0 && D.system_err == 0);
        assert(D.tape_fix_end == c && D.tape_fix_cnt == ((c == 1) ? 2 : 1));
    }
    /* 다가온 쪽을 모르면 보정하지 않는다(4). 정지 뒤 선이 없으면 끝(3) */
    for (int c = 0; c < 2; c++) {
        setup();
        D.control = RUN_TURN;
        D.tape_step = TAPE_STEP_SETTLE;
        D.tape_align = 1;
        D.turn_pulses = 2000;
        D.tape_fix_end = -1;
        D.guide_pos = 3;
        D.tape_side = (int16_t)c;
        D.guide_all_off = (int16_t)c;
        for (int i = 0; i < 20; i++) { assert(motor_turn_adjust() == 0); }
        if (c) {    /* 정착 뒤 선 없음도 10틱 이어져야 끝낸다 */
            assert(D.tape_align == 1 && D.tape_fix_end == -1);
            for (int i = 0; i < 9; i++) { assert(motor_turn_adjust() == 0); }
        }
        assert(D.tape_align == 0 && D.tape_fix_cnt == 0 && D.tape_fix_end == (c ? 3 : 4) && D.system_err == 0);
    }
    /* 정렬 회전 감속은 일반 회전과 같다(v0.36.0 부터 DCC). 감속 시작 각이 잡음 각보다 크면(DCC 20, 일반 센서) 라인을 보기 전에 엔코더로 미리 줄인다 */
    setup();
    (void)param_set(P_DCC, 20);
    D.control = RUN_TURN;
    D.tape_step = 1;
    D.tape_align = 1;
    D.turn_pulses = 2000;
    D.tape_total_pulse = 2000;
    D.tape_target_cnt = D.tape_end_cnt = 2000.0f;
    D.tape_max = 100;
    D.guide_all_off = 1;
    D.tape_turned = 0;
    D.vin_out = P.turn_speed;
    (void)motor_turn_adjust();
    assert(D.vin_out == P.turn_speed);
    D.tape_turned = 1990;
    { int16_t v0 = D.vin_out;
      for (int i = 0; i < 40; i++) {
          int16_t v1;
          (void)motor_turn_adjust();
          v1 = D.vin_out;
          assert((v0 - v1) <= stop_turn_dcc());
          v0 = v1;
      }
      assert(D.vin_out < P.turn_speed); }
    assert(stop_turn_dcc() == 20);
    /* 거리 감속: 회전 속도로 오다가 중앙 앞 2도 근처에서 최저 속도가 된다. 최저 속도 구간은 몇 도뿐이다 */
    {
        float rate, acc = 0.0f, at_min = -1.0f;
        int16_t vmin = 32767;
        setup();
        (void)param_set(P_DCC, 40);
        D.control = RUN_TURN;
        D.tape_step = 1;
        D.tape_align = 1;
        D.turn_90 = 2435;                     /* 휠 간격 205 cm 쯤 */
        rate = (float)D.turn_90 / 90.0f;
        D.turn_pulses = D.tape_total_pulse = 4870;
        D.tape_target_cnt = D.tape_end_cnt = 4870.0f;
        D.tape_max = (int16_t)(rate * 20.0f);
        D.guide_all_off = 1;
        D.vin_out = P.turn_speed;
        D.tape_turned = 3000;
        while (D.tape_turned < 4870) {
            (void)motor_turn_adjust();
            acc += (float)D.vin_out * (12.0f / 6000.0f) * 2.0f;   /* 두 바퀴 펄스 */
            while (acc >= 1.0f) { acc -= 1.0f; D.tape_turned++; }
            if ((at_min < 0.0f) && (D.vin_out <= P.turn_min)) { at_min = (float)D.tape_turned; }
            if (D.vin_out < vmin) { vmin = D.vin_out; }
        }
        /* v0.38.0: 엔코더 끝점으로 바닥까지 한 번에 줄이고, 그때까지 라인이 없으면 v_잡음으로 찾는다(tape_plan_min -1) */
        printf("  정렬 회전 라인 못 봄: 최저 %d rpm, 끝 속도 %d rpm(회전 속도 %d)\n", vmin, D.vin_out, P.turn_speed);
        /* 새 저크(2.4)는 바닥까지 마지막 S커브가 느려 v_잡음으로 바뀌기 전 최저가 바닥 + 21(예전 + 8) */
        assert((vmin <= stop_min_rpm() + 30) && (D.tape_plan_min == -1) && (D.vin_out > stop_min_rpm()) && (D.vin_out < P.turn_speed));
    }
    /* 편차 → 각 환산: atan(편차 mm / GD). 1단위 = 6.89 / 2 mm. GD 250, 1도 10 펄스 */
    { static const struct { int16_t pos; float lo, hi; } cv[2] = { { 15, 115.0f, 118.0f }, { 31, 229.0f, 233.0f } };
      for (int c = 0; c < 2; c++) {
          setup();
          guide_reset();
          D.control = RUN_TURN;
          D.tape_step = 1;
          D.tape_align = 1;
          D.turn_90 = 900;
          D.turn_pulses = D.tape_total_pulse = 1800;
          D.tape_target_cnt = D.tape_end_cnt = 1800.0f;
          D.tape_max = 400;
          D.tape_turned = 1500;
          D.guide_pos = cv[c].pos;                  /* 일반 15단위 약 11.7도, wide 31단위 약 23.1도 */
          (void)motor_turn_adjust();
          printf("  편차 %d단위 → %.1f 도\n", cv[c].pos, (D.tape_end_cnt - 1500.0f) / 10.0f);
          assert(D.tape_end_cnt - 1500.0f >= cv[c].lo && D.tape_end_cnt - 1500.0f <= cv[c].hi);
      } }
    /* 재가속 없음: 감속 중 새 라인이 센서 끝(일반 15, wide 31단위)에 들어와 끝점이 멀어져도 속도 지령은 올라가지 않는다.
     * 실제 라인 중앙은 목표의 1.05 배. 위치는 atan 모형으로 준다 */
    for (int c = 0; c < 2; c++) {
        int16_t edge = c ? 31 : 15;
        float rate, acc = 0.0f, center;
        int16_t prev = 0, seen = 0, down = 0;
        setup();
        guide_reset();
        (void)param_set(P_DCC, 40);
        D.control = RUN_TURN;
        D.tape_step = 1;
        D.tape_align = 1;
        D.turn_90 = 2435;
        rate = (float)D.turn_90 / 90.0f;
        D.turn_pulses = D.tape_total_pulse = 4870;
        D.tape_target_cnt = D.tape_end_cnt = 4870.0f;
        D.tape_max = (int16_t)(rate * 20.0f);
        center = 4870.0f * 1.05f;
        D.guide_all_off = 1;
        D.vin_out = P.turn_speed;
        D.tape_turned = 3000;
        while (D.tape_align && (D.tape_turned < 6000)) {
            float deg = (center - (float)D.tape_turned) / rate;
            float u = 250.0f * tanf(deg * 3.14159265f / 180.0f) / (6.89f / 2.0f);
            int16_t on = (int16_t)(fabsf(u) <= (float)edge + 0.5f);
            int16_t v;
            D.guide_all_off = (int16_t)!on;
            if (on) { D.guide_pos = (int16_t)lroundf(u); }
            (void)motor_turn_adjust();
            v = D.vin_out;
            if (on && !seen) {
                printf("  %s 라인 진입: 남은 %.1f 도, 속도 %d → %d\n", c ? "wide" : "일반", deg, prev, v);
                assert(v <= prev);                   /* 들어오는 순간 올라가지 않는다 */
                seen = 1;
            }
            if (v < prev) { down = 1; }
            if (down && D.tape_align) { assert(v <= prev); }   /* 끝난 틱 제외. 감속을 시작한 뒤에는 한 번도 올라가지 않는다 */
            prev = v;
            acc += (float)D.vin_out * (12.0f / 6000.0f) * 2.0f;
            while (acc >= 1.0f) { acc -= 1.0f; D.tape_turned++; }
        }
        assert(seen && down && !D.tape_align && D.system_err == 0);
        assert(fabsf((float)D.tape_turned - center) < rate * 1.5f);   /* 중앙 판정(1단위 안, 약 1.1도 반올림) 안에서 끝 */
    }
    /* 슬립: 출발 라인과의 차로 재고 최소제곱으로 계속 고친다. 가장자리(12칸 밖)는 버린다. 범위 ±25% */
    setup();
    guide_reset();                            /* 'h' 배율 1 */
    D.control = RUN_TURN;
    D.tape_step = 0;
    D.tape_align = 1;
    D.turn_90 = 900;                          /* 1도 10 펄스 */
    D.turn_pulses = 2000;
    D.tape_total_pulse = 2000;
    D.tape_target_cnt = D.tape_end_cnt = 2000.0f;
    D.tape_max = 200;
    D.tape_slip_ratio = 1.0f;
    D.guide_pos = 5;
    (void)motor_turn_adjust();
    assert(D.tape_origin_ok == 1 && D.tape_origin == 5);
    D.tape_turned = 50;
    D.guide_pos = 12;                         /* 7단위 = 24.1 mm, GD 250 이면 5.5도 = 55 펄스 */
    (void)motor_turn_adjust();
    assert(D.tape_slip_ratio > 1.09f && D.tape_slip_ratio < 1.11f);
    assert(D.tape_target_cnt > 1810.0f && D.tape_target_cnt < 1830.0f && D.tape_end_cnt == D.tape_target_cnt);
    D.tape_turned = 120;
    D.guide_pos = 15;                         /* 가장자리: 버린다 */
    (void)motor_turn_adjust();
    assert(D.tape_slip_ratio > 1.09f && D.tape_slip_ratio < 1.11f);
    D.tape_turned = 31;                       /* 같은 55 펄스를 31 에서 보면 비가 커진다: (55·50 + 55·31) / (50² + 31²) = 1.29 */
    D.guide_pos = 12;
    (void)motor_turn_adjust();
    assert(D.tape_slip_ratio == 1.25f);
    assert(D.tape_target_cnt == 1600.0f);
    D.tape_turned = 300;
    D.tape_lost_cnt = 6;
    D.guide_all_off = 1;
    (void)motor_turn_adjust();
    assert(D.tape_step == 1);
    /* 탐색 범위는 보정 전(2000)·후(1600) 목표를 둘 다 덮는다: 1400 ~ 2200 */
    D.guide_all_off = 0;
    D.guide_pos = 3;
    D.tape_turned = 1390;
    (void)motor_turn_adjust();
    assert(D.tape_step == 1);                 /* 범위 밖 라인은 무시 */
    D.tape_turned = 2150;
    (void)motor_turn_adjust();
    assert(D.tape_step == 2 && D.system_err == 0);
    assert(D.tape_end_cnt > 2150.0f + 20.0f && D.tape_end_cnt < 2150.0f + 26.0f);   /* 3단위 = 10.3 mm = 2.4도 남음 */
    D.tape_step = 1;
    D.guide_all_off = 1;
    D.tape_turned = 2201;
    (void)motor_turn_adjust();
    assert(D.system_err == ERR_GUIDE_SENSOR && D.tape_align == 0);
    /* 일반 회전은 가이드를 보지 않는다 */
    setup();
    D.control = RUN_TURN;
    D.turn_pulses = 500;
    D.guide_all_off = 1;
    for (int i = 0; i < 20; i++) { (void)motor_turn_adjust(); }
    assert(D.turn_pulses == 500 && D.tape_step == 0);
    /* 단계 2: 라인 위치로 남은 각을 알고 그 끝에서 0 이 되는 등감속(바닥 아래는 바닥) */
    setup();
    D.control = RUN_TURN;
    D.tape_step = 2; D.tape_align = 1; D.turn_pulses = 2000; D.tape_turned = 1900;
    D.tape_total_pulse = 2000;
    D.tape_target_cnt = D.tape_end_cnt = 2000.0f; D.tape_max = 100;
    D.guide_pos = 4;                          /* 4단위 = 13.8 mm = 3.2도 남음. 9단위(7.1도)는 아직 감속 전(필요 감속도 < 0.85 × DCC) */
    D.vin_out = P.turn_speed;
    for (int i = 0; i < 3; i++) { (void)motor_turn_adjust(); }   /* v0.38.0: 지령을 직접 줄인다. 차가 안 움직이는 이 시험에서 오래 돌리면 바닥까지 간다 */
    assert(D.tape_align == 1 && D.vin_out > stop_min_rpm() && D.vin_out < P.turn_speed);   /* 3도 앞: 등감속 중 */
    D.guide_all_off = 1;                      /* 라인이 잠깐 안 보이면 마지막 예상을 쓴다 */
    D.tape_end_cnt = (float)D.tape_turned + (float)P.turn_90 / 90.0f;
    for (int i = 0; i < 2000; i++) { (void)motor_turn_adjust(); }
    assert((D.vin_out <= P.turn_min) && (D.vin_out >= stop_min_rpm()));   /* 1도 앞: 등감속, 다시 올리지 않는다 */
    /* 라인 위(단계 2)에서 2도 안: 가이드 위치로 남은 각에 비례해 정지 바닥까지 낮춘다. 예전(QUIRK_FLOOR_350)은 최저 회전 속도 */
    D.guide_all_off = 0;
    D.guide_pos = 2;                          /* 2단위 = 6.9 mm = 1.6도 */
    for (int i = 0; i < 300; i++) { (void)motor_turn_adjust(); }
    assert((D.tape_step == 2) && (D.vin_out < P.turn_min) && (D.vin_out >= stop_min_rpm()));
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_FLOOR_350);
    D.tape_plan_min = 0;                      /* 회전 중 LG 를 바꾼 시험이라 감속 기억을 지운다*/
    for (int i = 0; i < 300; i++) { (void)motor_turn_adjust(); }
    assert(D.vin_out == P.turn_min);
    (void)param_set(P_LEGACY_QUIRKS, 0);

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
    /* DAC 고정(바닥 rpm 시험)은 제어 상태와 상관없이 폐루프 주행과 같은 D.backward 방향이다 */
    D.control = RUN_READY;
    motor_dac_force(300, 300);
    motor_run();
    assert(D.amp_dir == MOTION_BACK && D.motion == MOTION_BACK && D.tq_cmd[0] == 300 && D.tq_cmd[1] == 300);
    D.backward = 0;
    D.control = RUN_DRIVE;
    motor_run();
    assert(D.amp_dir == MOTION_FWD);
    motor_dac_release();

    /* 조향 주기: 기본은 motor_run 마다(10 ms), QUIRK_STEER_20MS 는 두 번에 한 번 */
    setup();
    D.guide_pos = 3;
    motor_run();
    assert(D.guide_kf_init == 1);
    setup();
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_STEER_20MS);
    D.guide_pos = 3;
    motor_run();
    assert(D.guide_kf_init == 0);
    motor_run();
    assert(D.guide_kf_init == 1);
    (void)param_set(P_LEGACY_QUIRKS, 0);

    /* 속도 피드포워드: 오차 0 이어도 지령에 비례한 토크 */
    setup();
    D.control = RUN_TURN;
    D.rpm_avg[0] = D.rpm_avg[1] = 1000;
    motor_speed_pi(1000);
    assert(D.tq_cmd[0] == 0);
    (void)param_set(P_SPEED_FF, 100);
    motor_speed_pi(1000);
    assert(D.tq_cmd[0] == 100 && D.tq_cmd[1] == 100);
    (void)param_set(P_SPEED_FF, 0);

    /* 엔코더 고착: 지령 바닥 이상, 오른쪽만 펄스, 왼쪽은 N 틱 무펄스 */
    setup();
    D.amp_run = 1;
    D.w_cmd[0] = D.w_cmd[1] = 500;
    for (int i = 0; i < 20; i++) { motor_encoder_pulse(1); motor_feedback_update(); motor_stall_check(); }
    assert(D.system_err == 0);                     /* 꺼져 있다 */
    (void)param_set(P_ENC_STALL_TICKS, 5);
    for (int i = 0; i < 4; i++) { motor_encoder_pulse(1); motor_feedback_update(); motor_stall_check(); }
    assert(D.system_err == 0);
    motor_encoder_pulse(1); motor_feedback_update(); motor_stall_check();
    assert(D.system_err == ERR_ENC_L_STALL);
    setup();
    D.amp_run = 1;
    D.w_cmd[0] = D.w_cmd[1] = 500;
    for (int i = 0; i < 20; i++) { motor_feedback_update(); motor_stall_check(); }
    assert(D.system_err == 0);                     /* 둘 다 멈추면 고착이 아니다 (정지 상태) */
    (void)param_set(P_ENC_STALL_TICKS, 0);

    puts("test_motor ok");
    return 0;
}
