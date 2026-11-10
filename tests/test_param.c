/* test_param.c - 표, 한계, 기본값, 파생값, 1회 초기화 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "fakes.h"

#define P param_dv

static uint16_t ee_sum(void)
{
    uint16_t s = 0;
    for (int i = 1; i <= 45; i++) { s = (uint16_t)(s + fake_ee[i]); }
    return s;
}

/* 현장 장비처럼: 옛 값이 남아 있고 표시가 없다 */
static void field_image(void)
{
    fake_reset();
    memset(fake_ee, 0xFF, sizeof(fake_ee));
    (void)param_boot(&fake_store, 1);
    fake_ee[31] = 7;
    fake_ee[38] = (uint16_t)(int16_t)-70;
    fake_ee[39] = 500;
    fake_ee[44] = 3;
    fake_ee[50] = 0xFFFF;
    fake_ee[0] = ee_sum();
    fake_ee_writes = 0;
}

int main(void)
{
    fake_reset();
    fake_params();

    assert(param_get(P_ACC) == 8 && param_get(P_DCC) == 40);
    /* 옛 턴 속도 30 은 15 로 줄이고 나머지 값을 지우지 않는다 */
    { int16_t id = param_get(P_WHEEL_BASE_MM);
      fake_ee[P_SPEED_TURN] = 30;
      fake_ee[0] = ee_sum();
      assert(param_boot(&fake_store, 2611) == 0);
      assert(param_get(P_SPEED_TURN) == 15 && fake_ee[P_SPEED_TURN] == 15 && fake_ee[0] == ee_sum());
      assert(param_get(P_WHEEL_BASE_MM) == id); }
    assert(param_set(P_SPEED_TURN, 13) == 0);
    /* 가이드센서 거리: 빈 EEPROM 이면 250, 체크섬 밖 115 에 저장, 범위 100~999 */
    assert(param_get(P_GUIDE_DIST_MM) == 250 && P.guide_dist_mm == 250.0f);
    assert(param_set(P_GUIDE_DIST_MM, 300) == 0 && fake_ee[PARAM_GUIDE_DIST_ADDR] == 300 && P.guide_dist_mm == 300.0f);
    assert(param_set(P_GUIDE_DIST_MM, 50) == -1);
    assert(param_boot(&fake_store, 2611) == 0 && param_get(P_GUIDE_DIST_MM) == 300);
    assert(param_set(P_GUIDE_DIST_MM, 250) == 0);
    /* 시험용 스위치: 기본 0, 체크섬 밖 116~118 */
    assert(param_get(P_OPT_FLAGS) == 0 && param_get(P_SPEED_FF) == 0 && param_get(P_ENC_STALL_TICKS) == 0);
    assert(param_set(P_OPT_FLAGS, 64) == -1 && param_set(P_OPT_FLAGS, 37) == 0 && param_opt(OPT_PAUSE_RESUME_SMOOTH));
    assert(param_set(P_OPT_FLAGS, 5) == 0 && fake_ee[116] == 5);
    assert(!param_opt(OPT_KF_POSITION) && param_opt(OPT_RESUME_CRUISE));
    assert(param_set(P_SPEED_FF, 120) == 0 && fake_ee[117] == 120 && P.speed_ff == 0.12f);
    assert(param_set(P_ENC_STALL_TICKS, 30) == 0 && fake_ee[118] == 30 && P.stall_ticks == 30);
    assert(param_boot(&fake_store, 2611) == 0 && param_get(P_OPT_FLAGS) == 5 && param_get(P_SPEED_FF) == 120);
    /* RAM 에만: 값은 바뀌고 EEPROM 은 그대로 */
    fake_ee_writes = 0;
    assert(param_set_ram(P_OPT_FLAGS, 64) == -1 && param_set_ram(0, 1) == -1);
    assert(param_set_ram(P_OPT_FLAGS, 2) == 0 && param_get(P_OPT_FLAGS) == 2 && !param_opt(OPT_RESUME_CRUISE));
    assert(param_set_ram(P_SPEED_TURN, 12) == 0 && param_get(P_SPEED_TURN) == 12);
    assert(fake_ee_writes == 0 && fake_ee[116] == 5 && fake_ee[P_SPEED_TURN] == 13);
    assert(param_boot(&fake_store, 2611) == 0 && param_get(P_OPT_FLAGS) == 5 && param_get(P_SPEED_TURN) == 13);
    assert(param_set(P_OPT_FLAGS, 0) == 0 && param_set(P_SPEED_FF, 0) == 0 && param_set(P_ENC_STALL_TICKS, 0) == 0);
    /* 공차 프로필: 기본 0, 77~81 저장, 0·적재 하한 아래는 적재 값. 부팅은 적재 */
    assert(param_get(P_EMPTY_SPEED) == 0 && param_get(P_LOAD_DETECT) == 0 && !param_profile_empty());
    { int16_t t = P.target_speed, d = P.dcc, a = P.acc;
      assert(param_set(P_EMPTY_SPEED, 45) == 0 && fake_ee[PARAM_EMPTY_ADDR0] == 45 && P.target_speed == t);
      assert(param_set(P_EMPTY_DCC, 30) == 0 && fake_ee[PARAM_EMPTY_ADDR0 + 3] == 30 && param_set(P_LOAD_DETECT, 2) == -1);
      param_profile(1);
      assert(P.target_speed == param_rpm(45) && P.dcc == 30 && P.acc == a);   /* EAC 0 = 적재 값 */
      assert(param_active(P_SPEED_HIGH) == 45 && param_active(P_SPEED_TURN) == param_get(P_SPEED_TURN));
      assert(param_set(P_EMPTY_SPEED, 5) == 0 && P.target_speed == t);           /* 1번 하한 7 아래 */
      param_profile(0);
      assert(P.target_speed == t && P.dcc == d && P.acc == a);
      assert(param_boot(&fake_store, 2611) == 0 && param_get(P_EMPTY_DCC) == 30 && !param_profile_empty());
      assert(param_set(P_EMPTY_SPEED, 0) == 0 && param_set(P_EMPTY_DCC, 0) == 0); }
    /* 빈 EEPROM → 한계 위반 → 기본값 1 세트 저장 */
    assert(param_get(P_SPEED_HIGH) == 30 && param_get(P_SPEED_LOW) == 25 && param_get(P_SPEED_TURN) == 13);
    assert(param_get(P_ACS_VERSION) == 11 && param_get(P_VERSION) == 2611);
    assert(fake_ee[1] == 30 && fake_ee[0] == ee_sum());
    assert(fake_ee[PARAM_MIGRATE_ADDR] == PARAM_MIGRATE_MARK);

    /* 파생값: 수식 그대로 */
    { float dia = 202.0f * 3.14159265f;
      float scale = (1000.0f / dia) * 40.0f;
      assert(P.one_meter_rpm == scale);
      assert(P.target_speed == (int16_t)(30.0f * scale));
      assert(P.low_speed == (int16_t)(25.0f * scale));
      assert(P.min_speed == (int16_t)(6.0f * scale));              /* set28 < 4 → 6 */
      assert(P.llow_min == (int16_t)(7.0f * scale));
      assert(P.llow_speed == P.low_speed / 2);
      assert(P.one_pulse_length == (dia / 40.0f) / 12.0f);
      assert(P.turn_90 == (int16_t)((60.0f * 10.0f * 3.14159265f) / P.one_pulse_length / 2.0f));
      assert(P.turn_180 == P.turn_90 * 2);
      assert(P.turn_16 == (P.turn_90 / 90) * 16 && P.turn_32 == P.turn_16);
      assert(P.stop_len == (int16_t)(35.0f * P.one_length_pulse * 10.0f));
      assert(P.tag_pulse[10] == (int16_t)(10.0f * (50.0f * P.one_length_pulse)));
      assert(P.straight_len == 3L * P.tag_pulse[10]);
      assert(P.for_obs == 1 && P.back_obs == 1 && P.turn_mode == 2 && P.forback_mode == 2); }

    /* 기본값 한 벌 */
    param_default_type(11);
    param_default();
    assert(param_get(P_SPEED_HIGH) == 30 && param_get(P_SPEED_LOW) == 25 && param_get(P_AGV_TYPE) == 11);
    assert(param_get(P_CROSS_IN_CM) == 60 && param_get(P_BAT_OFFSET_DV) == 0);
    assert(fake_ee[P_SPEED_HIGH] == 30 && fake_ee[P_AGV_TYPE] == 11 && fake_ee[0] == ee_sum());
    assert(param_get(P_GUIDE_CURVE_DETECT) == 0);   /* 새 파라미터 자리는 옛 기본값을 쓰지 않는다 */

    /* 한계: 일반, 하한 0 으로 낮춘 36, 자리별 */
    assert(param_set(P_SPEED_HIGH, 6) == -1 && param_set(P_SPEED_HIGH, 61) == -1);
    assert(param_get(P_SPEED_HIGH) == 30);
    assert(param_set(P_ACS_RETRY_LEGACY, 0) == 0);
    assert(param_get(P_ACS_RETRY_LEGACY) == 0);
    assert(param_set(P_OBS_USE, 23) == -1);          /* 자리 상한 22 */
    assert(param_set(P_OBS_USE, 12) == 0);
    assert(P.for_obs == 1 && P.back_obs == 2);
    assert(param_set(P_STEER_RPM_OFFSET, -999) == 0 && param_set(P_STEER_RPM_OFFSET, -1000) == -1);
    assert(param_set(0, 1) == -1 && param_set(P_COUNT, 1) == -1);

    /* 저장: 체크섬과 값 두 번. 런타임 번호는 저장하지 않는다 */
    fake_ee_writes = 0;
    assert(param_set(P_SPEED_TURN, 12) == 0);
    assert(fake_ee_writes == 2 && fake_ee[P_SPEED_TURN] == 12 && fake_ee[0] == ee_sum());
    assert(param_set(P_SPEED_TURN, 16) == -1);
    fake_ee_writes = 0;
    assert(param_set(P_BAT_DV, 240) == 0);
    assert(fake_ee_writes == 0 && param_get(P_BAT_DV) == 240);

    /* 번호별 재계산: 28 은 런타임에 최저속을 다시 계산하지 않는다, 3 은 하한 없이 쓴다 */
    { int16_t min_before = P.min_speed;
      assert(param_set(P_SPEED_MIN, 9) == 0);
      assert(P.min_speed == min_before && P.actuator_delay == 90); }
    assert(param_set(P_SPEED_TURN, 3) == 0 && P.turn_speed == param_rpm(3));
    assert(param_set(P_SPEED_LOW, 60) == 0 && P.low_speed == P.target_speed);
    assert(param_get(P_SPEED_LOW) == 30 && fake_ee[P_SPEED_LOW] == 30);   /* LOS 를 DRS 보다 크게 쓰면 DRS 로 잘린다 */

    /* DRS 를 저장할 때 LOS 가 DRS 를 넘지 않게 한다 */
    assert(param_set(P_SPEED_LOW, 25) == 0 && param_get(P_SPEED_LOW) == 25 && fake_ee[P_SPEED_LOW] == 25);
    assert(param_set(P_SPEED_HIGH, 20) == 0);
    assert(param_get(P_SPEED_LOW) == 20 && fake_ee[P_SPEED_HIGH] == 20 && fake_ee[P_SPEED_LOW] == 20);
    assert(fake_ee[0] == ee_sum() && P.low_speed == param_rpm(20));
    assert(param_set(P_SPEED_HIGH, 30) == 0 && param_get(P_SPEED_LOW) == 20 && fake_ee[P_SPEED_LOW] == 20);   /* 이상이면 LOS 불변 */
    assert(param_set(P_SPEED_HIGH, 20) == 0 && param_get(P_SPEED_LOW) == 20);                                  /* 같은 값도 불변 */
    assert(param_set(P_SPEED_LOW, 61) == -1 && param_get(P_SPEED_LOW) == 20);                                  /* 한계 위반은 그대로 거부 */
    /* RAM 만 바꾸는 임시 변경은 저장소의 LOS 를 건드리지 않는다(주행 속도는 파생값 자르기가 막는다) */
    assert(param_set(P_SPEED_HIGH, 30) == 0 && param_set(P_SPEED_LOW, 25) == 0);
    fake_ee_writes = 0;
    assert(param_set_ram(P_SPEED_HIGH, 10) == 0);
    assert(fake_ee_writes == 0 && fake_ee[P_SPEED_LOW] == 25 && fake_ee[P_SPEED_HIGH] == 30 && fake_ee[0] == ee_sum());
    assert(param_set_ram(P_SPEED_HIGH, 30) == 0);
    assert(param_set(P_STEER_KI, 5) == 0 && agv_drive.p_ki == 5.0f * 0.1f);
    assert(param_set(P_CROSS_MODE, 10) == 0);       /* 42번은 무시한다. 한계·저장만 남는다 */

    /* 체크섬 불일치는 읽은 값을 그대로 쓴다 */
    fake_ee[0] = (uint16_t)(fake_ee[0] + 1);
    fake_ee[1] = 44;
    assert(param_boot(&fake_store, 1) == 0);
    assert(param_get(P_SPEED_HIGH) == 44);

    /* 1회 초기화 */
    field_image();
    assert(param_boot(&fake_store, 1) == 0);
    assert(fake_ee[31] == 0 && fake_ee[38] == 0 && fake_ee[39] == 0 && fake_ee[44] == 0);
    assert(fake_ee[0] == ee_sum());
    assert(fake_ee[PARAM_MIGRATE_ADDR] == PARAM_MIGRATE_MARK);
    assert(param_get(P_LEGACY_QUIRKS) == 0 && param_get(P_SPEED_HIGH) == 30);
    /* 범위 밖 옛 값이 전체 기본값 초기화로 번지지 않는다 */
    assert(fake_ee[P_SPEED_LOW] == 25);

    /* 표시가 있으면 다시 쓰지 않는다: 시험용 비트가 유지된다 */
    assert(param_set(P_LEGACY_QUIRKS, 5) == 0);
    assert(param_boot(&fake_store, 1) == 0);
    assert(param_get(P_LEGACY_QUIRKS) == 5);
    assert(param_quirk(QUIRK_RISK_FLAG) == 1 && param_quirk(QUIRK_H_POSITION) == 0);
    /* 레거시 비트 b0~b10 까지 2047. 2048 은 넘는다 */
    assert(param_set(P_LEGACY_QUIRKS, 2048) == -1 && param_set(P_LEGACY_QUIRKS, 2047) == 0 && param_quirk(QUIRK_SRC_A) && param_quirk(QUIRK_LOW_ACC));
    assert(param_set(P_LEGACY_QUIRKS, QUIRK_STEER_LEGACY | 0x0F) == 0);
    assert(param_get(P_LEGACY_QUIRKS) == 383 && param_quirk(QUIRK_STEER_20MS) && param_quirk(QUIRK_NO_WB_GAIN));
    assert(param_set(P_LEGACY_QUIRKS, 5) == 0);

    /* 도중 전원 차단: 어느 쓰기에서 끊겨도 표시가 안 찍히고 다음 부팅에 끝난다 */
    for (int cut = 0; cut < 5; cut++) {
        field_image();
        fake_ee_cut = cut;
        (void)param_boot(&fake_store, 1);
        assert(fake_ee[PARAM_MIGRATE_ADDR] != PARAM_MIGRATE_MARK);
        fake_ee_cut = -1;
        fake_ee_writes = 0;
        (void)param_boot(&fake_store, 1);
        assert(fake_ee[PARAM_MIGRATE_ADDR] == PARAM_MIGRATE_MARK);
        assert(fake_ee[0] == ee_sum());
        assert(fake_ee[39] == 0 && param_get(P_LEGACY_QUIRKS) == 0);
    }

    /* 읽기 검증 실패: 표시 없음, 오류 반환, RAM 값은 0 */
    field_image();
    fake_ee_cut = 0;
    assert(param_boot(&fake_store, 1) == ERR_PARAM_CHECKSUM);
    assert(param_get(P_COMM_LINK) == 0 && param_get(P_LEGACY_QUIRKS) == 0);

    puts("test_param ok");
    return 0;
}
