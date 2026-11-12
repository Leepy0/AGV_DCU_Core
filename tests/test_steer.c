/* test_steer.c - 조향 게인 스케줄, 불감대(예전 동작), 칼만, 한계, 결함 비트 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "agv/guide.h"
#include "agv/motor.h"
#include "agv/steer.h"
#include "agv/steerlog.h"
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
    { guide_cfg_t c; guide_cfg_default(&c); guide_configure(&c); guide_reset(); }   /* 'h' 커브 임계 기본 4 */
    stop_reset();
    steer_reset();
    D.control = RUN_DRIVE;
    D.vin_out = P.low_speed;
    D.v_cmd = P.low_speed;
}

static float near0(float x) { return (x < 0.0f) ? -x : x; }

static uint32_t fnv(uint32_t h, float v)
{
    uint32_t b;
    int i;
    memcpy(&b, &v, sizeof(b));
    for (i = 0; i < 4; i++) { h = (h ^ ((b >> (i * 8)) & 0xFFu)) * 16777619u; }
    return h;
}

int main(void)
{
    /* 평시 속도 r = 1: Kp 35, Ki 하한 0.1 */
    setup();
    D.guide_pos = 3;
    steer_run(1);
    assert(D.p_kp == 35.0f);
    assert(D.p_ki == 0.1f);

    /* 저속 r = 0: Kp 20 */
    setup();
    D.vin_out = P.min_speed;
    steer_run(1);
    assert(D.p_kp == 20.0f);
    assert(D.p_ki == 0.2f);

    /* 부스트 구간 상한 r = 2: Kp 50 */
    setup();
    D.vin_out = P.target_speed;
    steer_run(2);
    assert(D.p_kp == 50.0f);

    /* 속도 0 */
    setup();
    D.vin_out = 0;
    steer_run(1);
    assert(D.p_kp == 20.0f && D.p_ki == 0.2f);

    /* 큰 편차: 4단위부터 올려 12단위에서 130. 일반·wide 같다. 위험 감속 중에도 건다 */
    setup();
    D.guide_pos = 13;
    steer_run(1);
    assert(D.p_kp == 130.0f);
    setup();
    D.guide_pos = 4;                    /* 4단위까지는 속도별 값 그대로 */
    steer_run(1);
    assert(D.p_kp == 35.0f);
    setup();
    D.guide_pos = 8;                    /* 4 → 12 사이를 곧게 올린다: 35 + 95 × 4/8 */
    steer_run(1);
    assert(near0(D.p_kp - 82.5f) < 0.01f);
    setup();
    D.guide_wide = 1;
    D.guide_pos = -8;
    steer_run(1);
    assert(near0(D.p_kp - 82.5f) < 0.01f);
    setup();
    D.guide_wide = 1;
    D.guide_pos = 12;
    steer_run(1);
    assert(D.p_kp == 130.0f);
    /* 예전 계단은 QUIRK_STEER_STEP */
    setup();
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_STEER_STEP);
    D.guide_pos = 12;
    steer_run(1);
    assert(D.p_kp == 35.0f);
    D.guide_pos = 13;
    steer_run(1);
    assert(D.p_kp == 130.0f);
    (void)param_set(P_LEGACY_QUIRKS, 0);
    /* 편차를 한 칸씩 올릴 때 비례 출력이 한 번에 크게 뛰지 않는다. 최고 속도, 일반·wide */
    { int16_t wide;
      for (wide = 0; wide <= 1; wide++) {
          int16_t p, max = wide ? 31 : 15;
          float prev = 0.0f, worst = 0.0f;
          for (p = 0; p <= max; p++) {
              float out, d;
              setup();
              D.vin_out = D.v_cmd = P.target_speed;
              D.guide_wide = wide;
              D.guide_pos = p;
              steer_run(1);
              out = D.p_kp * (float)D.p_err;
              d = near0(out - prev);
              if (d > worst) { worst = d; }
              prev = out;
          }
          printf("  조향 계단 %s: 한 칸 최대 변화 %.0f\n", wide ? "wide" : "일반", worst);
          assert(worst < 400.0f);       /* 예전: 일반 4×50 → 8×50 → 13×130, 한 칸에 1000 넘게 뛴다 */
      } }
    D.guide_pos = 21;
    steer_run(1);
    assert(D.p_kp == 130.0f);
    setup();
    D.guide_pos = 13;
    D.risk_stop = 1;
    steer_run(1);
    assert(D.p_kp == 130.0f);
    /* 위험 감속 중에도 적분을 쌓는다. 출력 변화 묶음(틱당 50 rpm)이 따라잡은 뒤 본다 */
    D.guide_pos = 5;
    { int k; for (k = 0; k < 30; k++) { steer_run(1); } }
    { float a = D.p_pi_out - D.p_kp * (float)D.p_err;
      steer_run(1);
      assert(D.p_pi_out - D.p_kp * (float)D.p_err < a - 0.1f); }
    /* 예전 조향(QUIRK_STEER_STEP): 위험 감속 중에는 속도별 Kp 만 쓰고 적분을 지운다 */
    setup();
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_STEER_STEP);
    D.guide_pos = 13;
    D.risk_stop = 1;
    steer_run(1);
    steer_run(1);
    assert(D.p_kp == 35.0f);
    assert(near0(D.p_pi_out - 35.0f * (float)D.p_err) < 0.01f);
    (void)param_set(P_LEGACY_QUIRKS, 0);

    /* 커브('h' 소스): 덩어리 폭 4칸 이상이면 Kp ×1.3, 상한 169 */
    setup();
    D.guide_span = 4;
    steer_run(1);
    assert(D.p_kp == 35.0f * 1.3f);
    setup();
    D.guide_span = 3;
    steer_run(1);
    assert(D.p_kp == 35.0f);
    setup();
    D.guide_span = 16;
    D.guide_pos = 13;
    steer_run(1);
    assert(D.p_kp == 130.0f * 1.3f);

    /* 수동 게인이면 자동 계산을 건너뛴다 */
    setup();
    (void)param_set(P_STEER_KP, 40);
    D.guide_pos = 15;
    steer_run(1);
    assert(D.p_kp == 40.0f);
    (void)param_set(P_STEER_KP, 0);

    /* 기본은 불감대가 없다: 작은 편차도 그대로 오차가 된다 */
    setup();
    D.vin_out = P.min_speed;            /* 저속 r = 0 */
    D.guide_pos = 1;
    steer_run(1);
    assert(D.p_err == -1);
    setup();
    D.guide_pos = 2;                    /* r = 1 */
    steer_run(1);
    assert(D.p_err == -2);
    setup();
    D.guide_pos = 5;
    steer_run(1);
    assert(D.p_err == -5);
    setup();
    D.guide_pos = -5;
    steer_run(2);
    assert(D.p_err == 5);
    setup();
    D.guide_pos = 8;
    steer_run(1);
    assert(D.p_err == -8);
    /* 옆 이동 4칸 지령: 오차 0 인 위치는 정확히 4 하나다 */
    { int16_t p, zeros = 0;
      for (p = -10; p <= 10; p++) {
          setup();
          D.vin_out = D.v_cmd = P.target_speed;
          D.p_test = 4;
          D.guide_pos = p;
          steer_run(1);
          if (D.p_err == 0) { assert(p == 4); zeros++; }
      }
      assert(zeros == 1); }
    /* 예전 불감대는 QUIRK_STEER_STEP: 폭 2 (r=1), 안쪽 0, 선형 구간(8) 안에서만 폭을 뺀다 */
    setup();
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_STEER_STEP);
    D.guide_pos = 2;
    steer_run(1);
    assert(D.p_err == 0);
    setup();
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_STEER_STEP);
    D.guide_pos = 5;
    steer_run(1);
    assert(D.p_err == -3);
    setup();
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_STEER_STEP);
    D.guide_pos = -5;
    steer_run(2);
    assert(D.p_err == 3);
    setup();
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_STEER_STEP);
    D.guide_pos = 8;
    steer_run(1);
    assert(D.p_err == -8);
    setup();
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_STEER_STEP);
    D.guide_pos = 3;                    /* 커브면 폭 ×0.3 → 0 */
    D.guide_span = 4;
    steer_run(1);
    assert(D.p_err == -3);
    setup();
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_STEER_STEP);
    D.p_test = 4;                       /* 지령 이동이 불감대만큼 더 간다: 오차 0 이 6 */
    D.guide_pos = 6;
    steer_run(1);
    assert(D.p_err == 0);
    (void)param_set(P_LEGACY_QUIRKS, 0);
    /* tuning 0 은 불감대를 건너뛴다 */
    setup();
    D.guide_pos = 2;
    steer_run(0);
    assert(D.p_err == -2);
    /* wide 값이 0·1 이 아니면 게인·불감대 단계 전체를 건너뛴다 */
    setup();
    D.guide_wide = 2;
    D.p_kp = 7.0f;
    D.guide_pos = 2;
    steer_run(1);
    assert(D.p_kp == 7.0f && D.p_err == -2);

    /* 후진은 부호가 반대 */
    setup();
    D.backward = 1;
    D.guide_pos = 5;
    steer_run(1);
    assert(D.p_err == 5);

    /* 분기 중에는 센서를 무시한다 */
    setup();
    D.branch_start = 1;
    D.p_cmd = -4;
    D.guide_pos = 9;
    steer_run(1);
    assert(D.p_err == -4);
    D.backward = 1;
    steer_run(1);
    assert(D.p_err == 4);

    /* 조향 한계 2·|v|, 최대 1500 */
    setup();
    D.v_cmd = 100;
    D.guide_pos = 30;
    steer_run(1);
    assert(D.p_pi_out == -200.0f && D.add_speed_l == -200 && D.add_speed_r == 200);
    setup();
    D.v_cmd = 3000;
    D.vin_out = 3000;
    D.guide_pos = 30;
    steer_run(1);
    steer_run(1);
    assert(D.p_pi_out >= -1500.0f);
    D.v_cmd = 10000;
    D.guide_pos = 60;
    steer_run(1);
    assert(D.p_pi_out == -1500.0f);

    /* 오프셋: L 은 더하고 R 은 뺀다 */
    setup();
    D.offset_kp = 11;
    steer_run(1);
    assert(D.add_speed_l == 11 && D.add_speed_r == -11);

    /* 칼만: 첫 호출 초기화, 전 소등은 예측만 하고 불확실성을 키운다 */
    setup();
    D.guide_pos = 4;
    steer_run(1);
    assert(D.guide_kf_init == 1 && D.kf_pos == 4.0f && D.kf_rate == 0.0f && D.kf_p00 == 1.0f);
    D.guide_pos = 8;
    steer_run(1);
    assert(D.kf_rate > 0.0f && D.kf_pos > 4.0f && D.kf_pos < 8.0f);
    { float p = D.kf_p00, pos = D.kf_pos, rate = D.kf_rate;
      D.guide_all_off = 1;
      steer_run(1);
      assert(D.kf_rate == rate && D.kf_pos == pos + rate * 0.01f && D.kf_p00 > p); }   /* 기본 10 ms */
    /* 커브면 Q 가 20 배 */
    setup();
    steer_run(1);
    D.guide_pos = 4;
    D.guide_span = 8;
    { float p00 = D.kf_p00;
      float p11 = D.kf_p11;
      float p00_pred = p00 + 0.0001f * p11 + 0.02f * 20.0f;
      steer_run(1);
      assert(near0(D.kf_p00 - (1.0f - p00_pred / (p00_pred + 4.0f)) * p00_pred) < 1e-5f); }

    /* 적분: 조건이 맞으면 쌓이고 상태가 바뀌면 지운다 */
    setup();
    D.guide_pos = 5;
    steer_run(1);                                 /* prev_control 변화로 지움 */
    { float a, b;
      steer_run(1);
      a = D.p_pi_out;
      steer_run(1);
      b = D.p_pi_out;
      assert(b < a);                              /* 음의 적분 누적 */
      D.p_cmd = 1;
      steer_run(1);
      steer_run(1); }

    /* 결함 수정: 위험 감속 중 커브 부스트를 걸지 않는다. 비트를 켜면 구동작 */
    setup();
    D.risk_stop = 1;
    steer_run(1);
    assert(D.p_kp == 35.0f);
    setup();
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_CURVE_NOW);
    D.risk_stop = 1;
    steer_run(1);
    assert(D.p_kp == 35.0f * 1.3f);
    (void)param_set(P_LEGACY_QUIRKS, 0);

    /* 커브 판정: 'a' 소스(QUIRK_SRC_A)는 설정과 무관하게 레거시 판정. 'h' 소스(기본)만 설정값을 쓴다 */
    setup();
    guide_reset();
    (void)param_set(P_LEGACY_QUIRKS, QUIRK_SRC_A);
    { guide_cfg_t cfg;
      guide_cfg_default(&cfg);
      assert(cfg.curve_span == 4);
      cfg.curve_span = 2;
      guide_configure(&cfg);
      D.guide_span = 3;                         /* 'a' 소스에서는 쓰지 않는다 */
      D.guide_led = 0x0007;
      steer_run(1);
      assert(D.p_kp == 35.0f);
      D.guide_led = 0x000F;
      steer_run(1);
      assert(D.p_kp == 35.0f * 1.3f);
      (void)param_set(P_LEGACY_QUIRKS, 0);
      D.guide_led = 0;
      steer_run(1);
      assert(D.p_kp == 35.0f * 1.3f);           /* 폭 3 >= 임계 2 */
      D.guide_span = 1;
      D.guide_led = 0xFFFF;
      steer_run(1);
      assert(D.p_kp == 35.0f);                  /* LED 패턴이 아니라 덩어리 폭을 본다 */
      guide_reset();
      D.guide_span = 3;
      steer_run(1);
      assert(D.p_kp == 35.0f);                  /* 기본 임계 4 */
      D.guide_span = 4;
      steer_run(1);
      assert(D.p_kp == 35.0f * 1.3f);
      (void)param_set(P_GUIDE_CURVE_DETECT, 0); }   /* GSR 값은 무시: 위 결과와 같다 */

    /* 변화율 항 부호: 기본은 감쇠, QUIRK_DAMP_SIGN_OLD 는 예전 부호. 칼만 상태를 고정해 비교한다.
     * 편차 10단위: 일반 센서 출력 변화 묶음 밖이라 한 번에 나온다 */
    { float out_p, out_a, out_b;
      int k;
      for (k = 0; k < 3; k++) {
          setup();
          D.guide_pos = 10;
          D.guide_kf_init = 1;
          D.kf_pos = 10.0f;
          D.kf_rate = (k == 0) ? 0.0f : 50.0f;    /* y = 0 이라 변화율이 유지된다 */
          D.kf_p00 = D.kf_p11 = 0.0f; D.kf_p01 = 0.0f;
          (void)param_set(P_LEGACY_QUIRKS, (k == 1) ? QUIRK_DAMP_SIGN_OLD : 0);
          steer_run(0);
          if (k == 0) { out_p = D.p_pi_out; } else if (k == 1) { out_a = D.p_pi_out; } else { out_b = D.p_pi_out; }
      }
      assert(out_a != out_p);
      assert(near0((out_a - out_p) + (out_b - out_p)) < 1e-3f);
      assert(out_b < out_a);                      /* 기본: 양의 변화율에는 음의 감쇠 */
      (void)param_set(P_LEGACY_QUIRKS, 0); }

    /* 조향 주기: 기본 10 ms(칼만 dt 0.01), QUIRK_STEER_20MS 는 0.02 */
    { int k;
      for (k = 0; k < 2; k++) {
          float pos, rate;
          setup();
          (void)param_set(P_LEGACY_QUIRKS, k ? QUIRK_STEER_20MS : 0);
          D.guide_pos = 4;
          steer_run(1);
          D.guide_pos = 8;
          steer_run(1);
          pos = D.kf_pos;
          rate = D.kf_rate;
          D.guide_all_off = 1;
          steer_run(1);
          assert(D.kf_pos == pos + rate * (k ? 0.02f : 0.01f));
      }
      (void)param_set(P_LEGACY_QUIRKS, 0); }

    /* 선택 비트: 칼만 위치를 조향 입력으로 */
    setup();
    D.guide_kf_init = 1;
    D.kf_pos = 5.0f;
    D.kf_rate = 0.0f;
    D.guide_pos = 0;
    D.guide_all_off = 1;                           /* 예측만: kf_pos 가 5 로 남는다 */
    steer_run(0);
    assert(D.p_err == 0);
    (void)param_set(P_OPT_FLAGS, OPT_KF_POSITION);
    steer_run(0);
    assert(D.p_err == -5);
    (void)param_set(P_OPT_FLAGS, 0);

    /* 휠 간격 비례: 기본. 60 cm 면 그대로, 120 cm 면 비례 출력이 두 배. QUIRK_NO_WB_GAIN 은 비례하지 않는다.
     * 예전 OPT 값 19(b0·b1·b4)는 비어 있는 비트라 아무 일도 하지 않는다.
     * 편차 10단위(출력 변화 묶음 밖)에 지령을 9 옮겨 오차 1단위만 본다 */
    { float out60;
      setup();
      (void)param_set(P_WHEEL_BASE_MM, 60);
      D.guide_pos = 10;
      D.p_test = 9;
      steer_run(0);
      out60 = D.p_pi_out;
      setup();
      (void)param_set(P_WHEEL_BASE_MM, 120);
      D.guide_pos = 10;
      D.p_test = 9;
      steer_run(0);
      assert(near0(D.p_pi_out - 2.0f * out60) < 1e-2f);
      setup();
      (void)param_set(P_WHEEL_BASE_MM, 120);
      (void)param_set(P_OPT_FLAGS, 19);
      D.guide_pos = 10;
      D.p_test = 9;
      steer_run(0);
      assert(near0(D.p_pi_out - 2.0f * out60) < 1e-2f);
      (void)param_set(P_OPT_FLAGS, 0);
      setup();
      (void)param_set(P_WHEEL_BASE_MM, 120);
      (void)param_set(P_LEGACY_QUIRKS, QUIRK_NO_WB_GAIN);
      D.guide_pos = 10;
      D.p_test = 9;
      steer_run(0);
      assert(near0(D.p_pi_out - out60) < 1e-3f);
      (void)param_set(P_LEGACY_QUIRKS, 0);
      (void)param_set(P_WHEEL_BASE_MM, 60); }

    /* 식별 시험 옆 계단: p_test 만큼 지령이 옮겨 간다 */
    setup();
    D.guide_pos = 0;
    D.p_test = 4;
    steer_run(0);
    assert(D.p_err == 4);
    D.p_test = 0;

    /* 계단 완화: 10 ms 당 출력 변화 50 rpm. 일반 센서는 편차 10단위, wide 는 20단위부터 묶지 않는다.
     * 분기·예전 조향(QUIRK_STEER_STEP)은 묶지 않는다. 20 ms 조향은 한 번에 100 */
    { static const struct { int16_t wide, pos, branch, lg; float step; } cs[] = {
          { 0, 8, 0, 0, 50.0f }, { 0, -9, 0, 0, 50.0f }, { 1, 19, 0, 0, 50.0f }, { 0, 8, 0, QUIRK_STEER_20MS, 100.0f },
          { 0, 10, 0, 0, 0.0f }, { 1, 20, 0, 0, 0.0f }, { 0, 8, 1, 0, 0.0f }, { 0, 8, 0, QUIRK_STEER_STEP, 0.0f } };
      int k;
      for (k = 0; k < (int)(sizeof(cs) / sizeof(cs[0])); k++) {
          setup();
          (void)param_set(P_LEGACY_QUIRKS, cs[k].lg);
          D.vin_out = D.v_cmd = P.target_speed;
          D.guide_wide = cs[k].wide;
          steer_run(1);
          steer_run(1);
          assert(D.p_pi_out == 0.0f);
          D.guide_pos = cs[k].pos;
          D.branch_start = cs[k].branch;
          D.p_cmd = cs[k].branch ? 8 : 0;
          steer_run(1);
          if (cs[k].step > 0.0f) { assert(near0(D.p_pi_out) == cs[k].step); }
          else                   { assert(near0(D.p_pi_out) > 200.0f); }
      }
      (void)param_set(P_LEGACY_QUIRKS, 0); }

    /* 곡선 끝 적분 감쇠: 편차 6단위 이상을 지나 4단위 아래로 들어오면 10 ms 마다 0.95 배. 5단위까지만 간 적분은 그대로 */
    { int k, arm;
      float i0 = 0.0f, i1 = 0.0f;
      for (arm = 0; arm <= 1; arm++) {
          setup();
          D.vin_out = D.v_cmd = P.target_speed;
          D.guide_pos = arm ? 6 : 5;
          for (k = 0; k < 400; k++) { steer_run(1); }
          D.guide_pos = 3;
          for (k = 0; k < 60; k++) { steer_run(1); }        /* 출력 묶음이 따라잡게 */
          i0 = D.p_pi_out - D.p_kp * (float)D.p_err;
          steer_run(1);
          i1 = D.p_pi_out - D.p_kp * (float)D.p_err;
          printf("  적분 감쇠 %s: %.1f → %.1f\n", arm ? "6단위 뒤" : "5단위 뒤", i0, i1);
          if (arm) { assert((near0(i0) < 20.0f) && (near0(i1) < near0(i0) + 0.01f)); }
          else     { assert((i0 < -30.0f) && (near0(i1 - i0) < 1.0f)); }
      }
      setup();
      (void)param_set(P_LEGACY_QUIRKS, QUIRK_STEER_STEP);   /* 예전 조향은 그대로 */
      D.vin_out = D.v_cmd = P.target_speed;
      D.guide_pos = 6;
      for (k = 0; k < 400; k++) { steer_run(1); }
      D.guide_pos = 3;
      for (k = 0; k < 60; k++) { steer_run(1); }
      assert(D.p_pi_out - D.p_kp * (float)D.p_err < -30.0f);
      (void)param_set(P_LEGACY_QUIRKS, 0); }

    /* TST 0(곡선 끝 처리·속도 비례 적분 시험 끔)은 고치기 전(v0.42.0) 조향과 출력 수열이 한 비트까지 같다.
     * 정한 입력(편차 오르내림·넘어감·떨림, 곡선 폭, 위험 감속과 속도 바뀜, 다른 선, 후진, 정지)으로 자동 이득·K_P 60 K_I 5·20 ms 조향을
     * 돌려 출력·이득 비트의 FNV 해시를 v0.42.0 에서 잰 값과 맞춘다 */
    { static const struct { int16_t kp, ki, lg; } run[3] = { { 0, 0, 0 }, { 60, 5, 0 }, { 0, 0, QUIRK_STEER_20MS } };
      uint32_t h = 2166136261u;
      int k, t;
      for (k = 0; k < 3; k++) {
          int16_t v = 2000;
          setup();
          (void)param_set(P_LEGACY_QUIRKS, run[k].lg);
          (void)param_set(P_STEER_KP, run[k].kp);
          (void)param_set(P_STEER_KI, run[k].ki);
          for (t = 0; t < 3000; t++) {
              int16_t ph = (int16_t)(t % 600), a;
              D.guide_wide = (int16_t)((t / 1500) & 1);
              D.guide_pos = (int16_t)((ph < 150) ? (ph / 10) : ((ph < 300) ? (14 - (ph - 150) / 12)
                          : ((ph < 400) ? (2 - (ph - 300) / 16) : (((ph * 7) % 5) - 2))));
              a = (D.guide_pos >= 0) ? D.guide_pos : (int16_t)(-D.guide_pos);
              D.guide_span = (a > 8) ? 5 : 2;
              D.guide_track = ((ph >= 420) && (ph < 440)) ? GUIDE_TRK_EDGE : GUIDE_TRK_NORMAL;
              D.risk_stop = (int16_t)((a >= 10) || (D.risk_stop && (a > 3)));
              D.backward = (int16_t)((t >= 2400) && (t < 2700));
              D.flag_stopping = (int16_t)(t >= 2800);
              if (D.flag_stopping)  { v = (int16_t)((v > 10) ? (v - 10) : 0); }
              else if (D.risk_stop) { v = (int16_t)((v > 1420) ? (v - 20) : 1400); }
              else                  { v = (int16_t)((v < 1985) ? (v + 15) : 2000); }
              D.vin_out = D.v_cmd = v;
              steer_run(1);
              h = fnv(fnv(fnv(fnv(h, D.p_pi_out), D.p_kp), D.p_ki), (float)D.add_speed_l);
          }
      }
      printf("  TST 0 조향 수열 해시 %08lx\n", (unsigned long)h);
      assert(h == 0xbf0ff2d4u);                   /* v0.42.0(고치기 전) 값. 개발·현장 빌드 같다 */
      (void)param_set(P_LEGACY_QUIRKS, 0);
      (void)param_set(P_STEER_KP, 0);
      (void)param_set(P_STEER_KI, 0); }

#if AGV_DEV_TESTS
    /* A(steer_exit_test 1): 오차와 적분의 부호가 반대일 때만 10 ms 마다 0.9 배, 무장 없음. 6단위를 지난 뒤 3단위(예전 규칙이면
     * 감쇠)에서는 계속 쌓인다. 20 ms 조향은 같은 시간 상수(0.81). |적분| < 5 rpm 은 부호가 반대여도 쌓이는 만큼만 */
    { int k, slow;
      for (slow = 0; slow <= 1; slow++) {
          float i0, i1, i2;
          setup();
          (void)param_set(P_LEGACY_QUIRKS, slow ? QUIRK_STEER_20MS : 0);
          steer_exit_test = 1;
          D.vin_out = D.v_cmd = P.target_speed;
          D.guide_pos = 6;
          for (k = 0; k < 400; k++) { steer_run(1); }
          D.guide_pos = 3;
          steer_run(1);
          i0 = steer_i_out();
          steer_run(1);
          i1 = steer_i_out();                       /* 오차 -3: 한 틱에 d = i1 - i0 */
          D.guide_pos = -1;                         /* 선이 반대쪽으로 넘어갔다: 오차 +1, 적분은 음수 */
          steer_run(1);
          i2 = steer_i_out();
          printf("  끝 처리 A %s: %.2f → %.2f → %.2f\n", slow ? "20 ms" : "10 ms", i0, i1, i2);
          assert((i0 < -30.0f) && (i1 < i0));
          assert(near0(i2 - (i1 + (i0 - i1) / 3.0f) * (slow ? 0.81f : 0.9f)) < 1e-3f);
      }
      (void)param_set(P_LEGACY_QUIRKS, 0);
      setup();
      steer_exit_test = 1;
      D.vin_out = D.v_cmd = P.target_speed;
      D.guide_pos = 1;
      for (k = 0; k < 40; k++) { steer_run(1); }
      { float i0, i1, i2;
        D.guide_pos = -1;
        steer_run(1);
        i0 = steer_i_out();
        steer_run(1);
        i1 = steer_i_out();
        steer_run(1);
        i2 = steer_i_out();
        assert((i0 < 0.0f) && (i0 > -5.0f) && (i1 > i0) && (near0((i2 - i1) - (i1 - i0)) < 1e-4f)); } }

    /* B(steer_exit_test 2): |적분| ≥ 100 rpm 이고 100 ms 동안 |위치| 가 2단위 이상 줄면 그 틱도 0.95 배.
     * 곡선 안 빠르기(1단위 120 ms)·작은 적분(< 100)은 A 만 켠 것과 한 비트까지 같다. 위치는 내내 적분 쪽(A 는 안 걸린다) */
    { static const struct { int16_t ki, per; int fire; } cs[] = { { 5, 5, 1 }, { 5, 12, 0 }, { 0, 5, 0 } };
      int c, k, m;
      for (c = 0; c < 3; c++) {
          float ia[2];
          for (m = 1; m <= 2; m++) {
              setup();
              (void)param_set(P_STEER_KI, cs[c].ki);
              steer_exit_test = (int16_t)m;
              D.vin_out = D.v_cmd = P.target_speed;
              D.guide_pos = 10;
              for (k = 0; k < (cs[c].ki ? 300 : 100); k++) { steer_run(1); }
              assert(cs[c].ki ? (steer_i_out() < -100.0f) : (steer_i_out() > -100.0f));
              for (k = 0; k < 40; k++) {
                  D.guide_pos = (int16_t)(10 - k / cs[c].per);
                  steer_run(1);
              }
              ia[m - 1] = steer_i_out();
          }
          printf("  끝 처리 B K_I %d 1단위 %d0 ms: A %.1f, A+B %.1f\n", cs[c].ki, cs[c].per, ia[0], ia[1]);
          if (cs[c].fire) { assert(near0(ia[1]) < 0.9f * near0(ia[0])); }
          else            { assert(ia[1] == ia[0]); }
      }
      (void)param_set(P_STEER_KI, 0); }

    /* C(steer_iv_test 1): 속도 지령 크기가 바뀌면 적분에 |v| 비를 곱한다. 한 틱 0.9~1.1, 두 틱 다 200 rpm 이상일 때만.
     * 정지 중 감속은 예전 식(묶지 않음) 하나만 건다. 오차 0 인 틱이라 쌓이는 몫이 없다 */
    { static const struct { int16_t v0, v1, stop; float k; } cs[] = {
          { 2000, 1900, 0, 0.95f }, { 2000, 1000, 0, 0.9f }, { 1000, 1500, 0, 1.1f }, { 1000, 1050, 0, 1.05f },
          { 300, 199, 0, 1.0f }, { 199, 250, 0, 1.0f }, { 1500, 1500, 0, 1.0f }, { 2000, 1000, 1, 0.5f } };
      int c, k;
      for (c = 0; c < (int)(sizeof(cs) / sizeof(cs[0])); c++) {
          float a, b;
          setup();
          steer_iv_test = 1;
          D.vin_out = D.v_cmd = cs[c].v0;
          D.guide_pos = 5;                          /* 6 아래: 예전 끝 처리가 무장되지 않는다 */
          for (k = 0; k < 200; k++) { steer_run(1); }
          D.guide_pos = 0;
          steer_run(1);
          a = steer_i_out();
          D.flag_stopping = cs[c].stop;
          D.vin_out = D.v_cmd = cs[c].v1;
          steer_run(1);
          b = steer_i_out();
          assert((a < -10.0f) && (near0(b - a * cs[c].k) < 1e-4f * near0(a)));
      }
      printf("  속도 비례 적분 C ok\n"); }
#endif

    puts("test_steer ok");
    return 0;
}
