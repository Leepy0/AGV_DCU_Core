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
    P_AUTO_START_SEC, P_STATION_NO,
    P_BRANCH_MODE,          /* BRM: 미사용. 값이 남아 있어도 무시한다. 번호·EEPROM 자리는 유지.
                               레거시에서는 스테이션 태그의 정지/분기 방식이었다. 그런 사이트를 옮길 때 사이트 코드가 읽는다 */
    P_TURN_FORBACK,
    P_TAG_DIGITS,           /* TAG: 미사용. 값이 남아 있어도 무시한다. 번호·EEPROM 자리는 유지 */
    P_STOP_LEN_MM,
    P_STEER_KP, P_STEER_KI, P_WHEEL_DIA_MM, P_GEAR_RATIO, P_WHEEL_BASE_MM, P_AGV_ID, P_AGV_TYPE,
    P_LAMP_EXT_USE, P_SPEED_MIN, P_SWITCH_START_NUM, P_TAG_TYPE,
    P_COMM_LINK,            /* 31: 0 지그비, 1 WiFi 브릿지 */
    P_STOP_LEN_FWD_MM, P_ACS_STANDBY, P_BRANCH_OFFSET,
    P_CROSS_IN_CM,          /* 35: 교차로 대기 정지 위치, 'I' 태그 뒤 cm. 0 = 태그에서 바로 정지. 'O' 태그는 서지 않는다 */
    P_ACS_RETRY_LEGACY,     /* 36: 0 이면 ACS 통신 미사용 */
    P_ACS_RETRY_SEC,
    P_GUIDE_CURVE_DETECT,   /* 38 GSR: 미사용. 값이 남아 있어도 무시한다. 번호·EEPROM 자리는 유지. 위치 소스는 'h' 고정, 예전 'a' 는 QUIRK_SRC_A */
    P_LEGACY_QUIRKS,        /* 39: 비트를 켜면 고치기 전 동작을 재현한다 */
    P_ERR_REPORT_MIN, P_CROSS_RETRY_100MS, P_CROSS_MODE, P_STEER_RPM_OFFSET,
    P_GUIDE_TIMEOUT,        /* 44 */
    P_ACS_VERSION,
    P_PARA_INIT, P_VERSION, P_BAT_DV, P_BAT_PERCENT,
    P_GUIDE_DIST_MM,        /* 50: 회전 중심에서 가이드센서까지 mm. 체크섬 밖 EEPROM 115 에 따로 둔다 */
    P_OPT_FLAGS,            /* 51: 시험용 개선 스위치 비트. 0 = 전부 끔 */
    P_SPEED_FF,             /* 52: 속도 피드포워드, 1000 rpm 당 토크 지령. 0 = 끔 */
    P_ENC_STALL_TICKS,      /* 53: 엔코더 고착 판정 틱(10 ms). 0 = 끔 */
    /* 54~58: 공차 프로필. 체크섬 밖 EEPROM 77~81(KEEP 빈 칸). 공차 값이 적재 칸 하한보다 작으면(0 포함) 적재 값을 쓴다 */
    P_EMPTY_SPEED,          /* 54 EDS: 공차 최고 속도 m/min (적재 1번) */
    P_EMPTY_TURN,           /* 55 ETN: 공차 회전 속도 (적재 3번) */
    P_EMPTY_ACC,            /* 56 EAC: 공차 가속 (적재 12번) */
    P_EMPTY_DCC,            /* 57 EDC: 공차 감속 (적재 13번) */
    P_LOAD_DETECT,          /* 58 LDS: 적재 감지 사용. 0 = 늘 적재(예전과 같다) */
    P_COUNT
};
#define P_SAVED_LAST 45   /* 이 번호까지 EEPROM 에 저장한다 */

/* P_LEGACY_QUIRKS 비트 (docs/07_주행_조향_성능.md 4절) */
#define QUIRK_RISK_FLAG    0x01u   /* b0 정지 완료 때 nGuide_Risk_Stop_Flag 를 내리지 않는다 */
#define QUIRK_H_POSITION   0x02u   /* b1 'h' 응답이 위치값을 자리표 10 으로 덮는다 */
#define QUIRK_CURVE_NOW    0x04u   /* b2 위험 감속 중 curve_now 를 켠다 */
#define QUIRK_TURN_START   0x08u   /* b3 회전을 0 rpm 부터 시작한다 */
/* b4~b6: 기본이 된 조향 개선(예전 P_OPT_FLAGS b0·b1·b4)을 하나씩 끈다. 셋 다 켜면 예전 OPT 0 조향 */
#define QUIRK_STEER_20MS    0x10u  /* b4 조향을 20 ms 마다 돌린다 (기본 10 ms, 칼만 dt 0.01, 적분 이득 1/2) */
#define QUIRK_DAMP_SIGN_OLD 0x20u  /* b5 변화율 항을 예전 부호로 더한다 (기본은 감쇠) */
#define QUIRK_NO_WB_GAIN    0x40u  /* b6 조향 이득을 휠 간격에 비례시키지 않는다 (기본 60 cm 기준 비례) */
#define QUIRK_STEER_STEP     0x100u /* b8 예전 조향을 되살린다: 속도 연동 불감대(안쪽 0, 선형 구간 안에서만 폭을 뺀다), 편차 문턱에서 Kp 가 한 번에 최대가 된다 */
#define QUIRK_SRC_A         0x200u /* b9 위치 소스를 예전 'a' 로 되살린다(기본 'h' 전용). 조향 비트와 별개 */
#define QUIRK_LOW_ACC       0x400u /* b10 저속 구간(llow 아래) 가속 한계를 예전 5 로 둔다(기본 max(5, ACC/2)) */
#define QUIRK_FLOOR_350     0x800u /* b11 예전 저속: 정지 바닥 350 rpm, 멈춤 판단 고정 10틱, SPN 4 미만은 6, 정렬 회전 끝 최저 회전 속도, 예전 저크·정지 끝 계단 */
/* 예전 저크(가감속 한계/8, 어느 속도든 8틱에 한계)·정지 끝 계단(바닥에서 0 으로 한 번에). P_LEGACY_QUIRKS 한계가 4095(b0~b11)라
 * 빈 비트가 없어 b11 에 묶는다. 위험 감속(GUIDE_RISK_DCC)은 비트와 상관없이 예전 저크다 */
#define QUIRK_JERK_OLD      QUIRK_FLOOR_350
#define QUIRK_STEER_LEGACY  (QUIRK_STEER_20MS | QUIRK_DAMP_SIGN_OLD | QUIRK_NO_WB_GAIN | QUIRK_STEER_STEP)
#define QUIRK_PAUSE_FULL_STOP 0x80u /* b7 일시정지(장애물·PAUSE) 감속 중 원인이 풀려도 0 까지 서고 다시 출발한다(예전 동작) */

/* 주소 50 표시. 31·38·39·44 의 1회 초기화가 끝나면 쓴다 */
#define PARAM_MIGRATE_ADDR 50
#define PARAM_MIGRATE_MARK 0xA55Au
#define PARAM_GUIDE_DIST_ADDR    115
#define PARAM_GUIDE_DIST_DEFAULT 250
#define PARAM_GUIDE_DIST_LOW     100
#define PARAM_GUIDE_DIST_UP      999
#define PARAM_EMPTY_ADDR0        77    /* 54~58 번 → 77~81 */

/* P_OPT_FLAGS 비트. 기본 0 = 끔. 실차 A/B 뒤 기본값을 정한다.
 * b0·b1·b4 는 기본 동작이 되어 비었다(값이 남아 있어도 무시한다). 예전 동작은 QUIRK_STEER_LEGACY */
#define OPT_RESUME_CRUISE  0x04u   /* 일시정지 재개 때 남은 거리가 길면 최대 속도까지 가속한다 */
#define OPT_KF_POSITION    0x08u   /* 조향 입력을 칼만 추정 위치로 쓴다 (기존 원시값) */
#define OPT_PAUSE_RESUME_SMOOTH 0x20u /* 비었다(v0.26 기본 동작). 예전 동작은 QUIRK_PAUSE_FULL_STOP */

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
    float   guide_dist_mm;                          /* P_GUIDE_DIST_MM */
    float   speed_ff;                               /* P_SPEED_FF / 1000: rpm 당 토크 */
    int16_t stall_ticks;                            /* P_ENC_STALL_TICKS */
    int16_t turn_16, turn_32, turn_90, turn_180;
    int16_t tag_pulse[12];                          /* const_Tag_Num_Pulse */
    int32_t straight_len;                           /* gStraight_Langth */
    int16_t acc, dcc;                               /* gDelta_ACC, gDelta_DCC */
    int16_t stop_len, fwd_stop_len;                 /* gStop_length, gFoward_Stop_length */
    int16_t display_mode, turn_mode, forback_mode;
    int16_t auto_drive_mode, auto_slow_mode, agv_type_mode;
    int16_t actuator_delay;
    int16_t for_bumper, back_bumper, for_obs, back_obs;
    int16_t acs_group, acs_standby;
} param_dv_t;

extern param_dv_t param_dv;

/* 부팅: 1회 초기화 → 읽기 3회 시도 → 파생값. 반환 0 정상, 그 외 ERR_PARAM_* */
int16_t  param_boot(const param_store_t *store, int16_t version);
void     param_store_set(const param_store_t *store);       /* 부팅 뒤 저장 계층 교체 */
void     param_limit(uint16_t idx, int16_t *low, int16_t *up);
int16_t  param_get(uint16_t idx);
/* 한계 검사 → 값 저장 → 파생값 갱신 → 저장 계층 쓰기. 반환 0 정상, -1 한계 벗어남 */
int16_t  param_set(uint16_t idx, int16_t value);
int16_t  param_set_ram(uint16_t idx, int16_t value);   /* param_set 과 같고 EEPROM 에 쓰지 않는다. 시험용 */
/* 기본값 적용 후 전부 저장. 키패드 INT 77 */
void     param_default(void);
/* 기본값의 P_AGV_TYPE. 사이트 값이라 앱이 준다. 0 이 초기값 */
void     param_default_type(int16_t kind);
int16_t  param_quirk(uint16_t bit);
int16_t  param_opt(uint16_t bit);                          /* P_OPT_FLAGS 비트 */
int16_t  param_rpm(int16_t m_per_min);                     /* GetSpeed_Pulse */
int16_t  param_digit(int16_t value, int16_t place);        /* Get_Decimal_Point_Value */
/* 적재·공차 프로필. empty 1 이면 target_speed·turn_speed·acc·dcc(와 low·llow)를 공차 값으로 다시 계산한다.
 * 부팅은 적재(0). 바뀔 때만 부른다 */
void     param_profile(int16_t empty);
int16_t  param_profile_empty(void);
/* 1·3·12·13번의 지금 프로필 값. 공차이고 공차 값이 쓸 만하면 54~57번, 아니면 그 번호 값 */
int16_t  param_active(uint16_t idx);

#endif /* AGV_PARAM_H */
