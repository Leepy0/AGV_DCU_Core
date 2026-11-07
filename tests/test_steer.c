/* test_steer.c - 조향 게인 스케줄, 불감대, 칼만, 한계, 결함 비트 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
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
    D.control = RUN_DRIVE;
    D.vin_out = P.low_speed;
    D.v_cmd = P.low_speed;
}

static float near0(float x) { return (x < 0.0f) ? -x : x; }

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

    /* 큰 편차 강제 130, wide 임계 20. 위험 감속 중에는 걸지 않는다 */
    setup();
    D.guide_pos = 13;
    steer_run(1);
    assert(D.p_kp == 130.0f);
    setup();
    D.guide_pos = 12;
    steer_run(1);
    assert(D.p_kp == 35.0f);
    setup();
    D.guide_wide = 1;
    D.guide_pos = 20;
    steer_run(1);
    assert(D.p_kp == 35.0f);
    D.guide_pos = 21;
    steer_run(1);
    assert(D.p_kp == 130.0f);
    setup();
    D.guide_pos = 13;
    D.risk_stop = 1;
    steer_run(1);
    assert(D.p_kp == 35.0f);

    /* 커브: 점등 4개 이상이면 Kp ×1.3, 상한 169 */
    setup();
    D.guide_led = 0x000F;
    steer_run(1);
    assert(D.p_kp == 35.0f * 1.3f);
    setup();
    D.guide_led = 0x0007;
    steer_run(1);
    assert(D.p_kp == 35.0f);
    setup();
    D.guide_led = 0xFFFF;
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

    /* 불감대: 폭 2 (r=1), 안쪽 0, 8 미만은 폭만큼 빼고 8 이상은 그대로 */
    setup();
    D.guide_pos = 2;
    steer_run(1);
    assert(D.p_err == 0);
    setup();
    D.guide_pos = 5;
    steer_run(1);
    assert(D.p_err == -3);
    setup();
    D.guide_pos = -5;
    steer_run(2);
    assert(D.p_err == 3);
    setup();
    D.guide_pos = 8;
    steer_run(1);
    assert(D.p_err == -8);
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
    assert(D.p_err == 3);

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
      assert(D.kf_rate == rate && D.kf_pos == pos + rate * 0.02f && D.kf_p00 > p); }
    /* 커브면 Q 가 20 배 */
    setup();
    steer_run(1);
    D.guide_pos = 4;
    D.guide_led = 0x00FF;
    { float p00 = D.kf_p00;
      float p11 = D.kf_p11;
      float p00_pred = p00 + 0.0004f * p11 + 0.02f * 20.0f;
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

    /* 커브 판정: 'a' 소스는 설정과 무관하게 레거시 판정. 'h' 소스만 설정값을 쓴다 */
    setup();
    guide_reset();
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
      (void)param_set(P_GUIDE_CURVE_DETECT, GUIDE_SRC_LED_ONLY);
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
      (void)param_set(P_GUIDE_CURVE_DETECT, 0); }

    /* 선택 비트: 변화율 항 부호. 칼만 상태를 고정해 두 경우의 출력을 비교한다 */
    { float out_p, out_a, out_b;
      int k;
      for (k = 0; k < 3; k++) {
          setup();
          D.guide_pos = 3;
          D.guide_kf_init = 1;
          D.kf_pos = 3.0f;
          D.kf_rate = (k == 0) ? 0.0f : 50.0f;    /* y = 0 이라 변화율이 유지된다 */
          D.kf_p00 = D.kf_p11 = 0.0f; D.kf_p01 = 0.0f;
          (void)param_set(P_OPT_FLAGS, (k == 2) ? OPT_STEER_D_SIGN : 0);
          steer_run(0);
          if (k == 0) { out_p = D.p_pi_out; } else if (k == 1) { out_a = D.p_pi_out; } else { out_b = D.p_pi_out; }
      }
      assert(out_a != out_p);
      assert(near0((out_a - out_p) + (out_b - out_p)) < 1e-3f);
      assert(out_b < out_a);                      /* 양의 변화율에는 음의 감쇠 */
      (void)param_set(P_OPT_FLAGS, 0); }

    /* 선택 비트: 10 ms 조향은 칼만 dt 0.01 */
    setup();
    (void)param_set(P_OPT_FLAGS, OPT_STEER_10MS);
    D.guide_pos = 4;
    steer_run(1);
    D.guide_pos = 8;
    steer_run(1);
    { float pos = D.kf_pos, rate = D.kf_rate;
      D.guide_all_off = 1;
      steer_run(1);
      assert(D.kf_pos == pos + rate * 0.01f); }
    (void)param_set(P_OPT_FLAGS, 0);

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

    puts("test_steer ok");
    return 0;
}
