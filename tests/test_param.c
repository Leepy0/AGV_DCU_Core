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
      assert(P.tag_total_pulse == P.tag_pulse[7]);
      assert(P.for_obs == 1 && P.back_obs == 1 && P.turn_mode == 2 && P.forback_mode == 2); }

    /* 기본값 세 벌 */
    param_default_types(11, 22, 33);
    param_default(2);
    assert(param_get(P_SPEED_HIGH) == 25 && param_get(P_GEAR_RATIO) == 60 && param_get(P_AGV_TYPE) == 22);
    assert(param_get(P_ACS_RETRY_SEC) == 1 && param_get(P_CROSS_MODE) == 1 && param_get(P_ERR_REPORT_MIN) == 10);
    assert(param_get(P_BAT_OFFSET_DV) == -9 && (int16_t)fake_ee[P_BAT_OFFSET_DV] == -9);
    assert(P.collision_type == 1);
    param_default(3);
    assert(param_get(P_WHEEL_BASE_MM) == 130 && param_get(P_AGV_ID) == 3 && param_get(P_SPEED_MIN) == 7);
    assert(param_get(P_STOP_LEN_MM) == 0 && param_get(P_AGV_TYPE) == 33);
    assert(P.min_speed == (int16_t)(7.0f * P.one_meter_rpm));
    param_default(1);
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
    assert(param_set(P_STEER_KI, 5) == 0 && agv_drive.p_ki == 5.0f * 0.1f);
    assert(param_set(P_CROSS_MODE, 10) == 0 && P.collision_type == 0);
    assert(param_set(P_CROSS_MODE, 1) == 0 && P.collision_type == 1);

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
