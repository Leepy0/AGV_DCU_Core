/*
 * param.h - 파라미터 표, 한계, 기본값, 파생값, EEPROM 저장
 *
 * 번호는 EEPROM 주소와 같고 바꾸지 않는다. 주소 0 은 1~45 의 합, 46~49 는 런타임 값이다.
 * 이름 대응: docs/02_네이밍_표준.md 4절
 */
#ifndef AGV_PARAM_H
#define AGV_PARAM_H

#include "agv/types.h"

enum {
    P_SPEED_HIGH = 1, P_SPEED_LOW, P_SPEED_TURN, P_AUTO_DRIVE_MODE, P_INTERFACE_MODE, P_SOUND,
    P_BUMPER_USE, P_OBS_USE, P_DISPLAY_MODE, P_BAT_OFFSET_DV, P_SPEED_JOG, P_ACC, P_DCC,
    P_AUTO_START_SEC, P_STATION_NO, P_BRANCH_MODE, P_TURN_FORBACK, P_TAG_DIGITS, P_STOP_LEN_MM,
    P_STEER_KP, P_STEER_KI, P_WHEEL_DIA_MM, P_GEAR_RATIO, P_WHEEL_BASE_MM, P_AGV_ID, P_AGV_TYPE,
    P_LAMP_EXT_USE, P_SPEED_MIN, P_SWITCH_START_NUM, P_TAG_TYPE,
    P_COMM_LINK,            /* 31: 0 지그비, 1 WiFi 브릿지 */
    P_STOP_LEN_FWD_MM, P_ACS_STANDBY, P_BRANCH_OFFSET, P_CROSS_IN_TIME,
    P_ACS_RETRY_LEGACY,     /* 36: 0 이면 ACS 통신 미사용 */
    P_ACS_RETRY_SEC,
    P_GUIDE_CURVE_DETECT,   /* 38: guide.h 의 GUIDE_SRC_*. 위치 소스 선택을 겸한다 */
    P_LEGACY_QUIRKS,        /* 39: 비트를 켜면 고치기 전 동작을 재현한다 */
    P_ERR_REPORT_MIN, P_CROSS_RETRY_100MS, P_CROSS_MODE, P_STEER_RPM_OFFSET,
    P_GUIDE_TIMEOUT,        /* 44 */
    P_ACS_VERSION,
    P_PARA_INIT, P_VERSION, P_BAT_DV, P_BAT_PERCENT,
    P_COUNT
};
#define P_SAVED_LAST 45   /* 이 번호까지 EEPROM 에 저장한다 */

/* P_LEGACY_QUIRKS 비트 (docs/07_주행_조향_성능.md 4절) */
#define QUIRK_RISK_FLAG    0x01u   /* b0 정지 완료 때 nGuide_Risk_Stop_Flag 를 내리지 않는다 */
#define QUIRK_H_POSITION   0x02u   /* b1 'h' 응답이 위치값을 자리표 10 으로 덮는다 */
#define QUIRK_CURVE_NOW    0x04u   /* b2 위험 감속 중 curve_now 를 켠다 */
#define QUIRK_TURN_STEP2   0x08u   /* b3 테이프 회전 Step 2 가 G < 0 으로 끝나고 최저속이 아니다 */

/* 주소 50 표시. 31·38·39·44 의 1회 초기화가 끝나면 쓴다 */
#define PARAM_MIGRATE_ADDR 50
#define PARAM_MIGRATE_MARK 0xA55Au

/* 저장 계층. 부팅 경로의 write 는 끝난 뒤 돌아와야 한다.
 * 운전 중에는 저장 큐에 넣는 구현으로 바꿔 끼워도 된다 */
typedef struct {
    uint16_t (*read)(uint16_t addr);
    void     (*write)(uint16_t addr, uint16_t value);
} param_store_t;

/* 파라미터에서 계산한 값. 속도는 모터 rpm */
typedef struct {
    float one_meter_rpm;     /* gOne_Meter_rpm_Scale: (m/min) → rpm */
    float one_rpm_meter;     /* gOne_rpm_Meter_Scale */
    float one_pulse_length;  /* gOne_Pulse_Length: mm/펄스 */
    float one_length_pulse;  /* gOne_Length_Pulse: 펄스/mm */
    int16_t target_speed, low_speed, llow_speed, min_speed, turn_speed, jog_speed;
    int16_t llow_min, turn_min;                     /* gLLow_Minimum_Speed, gTurn_Minimum_Speed */
    int16_t turn_16, turn_32, turn_90, turn_180;
    int16_t tag_pulse[12];                          /* const_Tag_Num_Pulse */
    int16_t tag_number, tag_total_pulse;            /* gTag_Number, Tag_Total_Pulse. Tag_nea 는 tag_pulse[n] */
    int32_t straight_len;                           /* gStraight_Langth */
    int16_t acc, dcc;                               /* gDelta_ACC, gDelta_DCC */
    int16_t stop_len, fwd_stop_len;                 /* gStop_length, gFoward_Stop_length */
    int16_t display_mode, turn_mode, forback_mode;
    int16_t auto_drive_mode, auto_slow_mode, agv_type_mode;
    int16_t actuator_delay, switch_start_num;
    int16_t for_bumper, back_bumper, for_obs, back_obs;
    int16_t acs_group, acs_standby;
    int16_t collision_type;                         /* gDA_Collision_Type */
} param_dv_t;

extern param_dv_t param_dv;

/* 부팅: 1회 초기화 → 읽기 3회 시도 → 파생값. 반환 0 정상, 그 외 ERR_PARAM_* */
int16_t  param_boot(const param_store_t *store, int16_t version);
int16_t  param_get(uint16_t idx);
/* 한계 검사 → 값 저장 → 파생값 갱신 → 저장 계층 쓰기. 반환 0 정상, -1 한계 벗어남 */
int16_t  param_set(uint16_t idx, int16_t value);
/* 기본값 적용 후 전부 저장. kind 1~3 = 키패드 77·88·99 */
void     param_default(uint16_t kind);
/* 기본값 세트별 P_AGV_TYPE. 사이트 값이라 앱이 준다. 0 이 초기값 */
void     param_default_types(int16_t kind1, int16_t kind2, int16_t kind3);
int16_t  param_quirk(uint16_t bit);
int16_t  param_rpm(int16_t m_per_min);                     /* GetSpeed_Pulse */
int16_t  param_digit(int16_t value, int16_t place);        /* Get_Decimal_Point_Value */

#endif /* AGV_PARAM_H */
