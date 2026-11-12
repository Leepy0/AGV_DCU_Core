/* param.c - 파라미터 표, 한계, 기본값 3벌, 파생값, 1회 초기화 */
#include "agv/param.h"
#include "agv/drive.h"

#define D    agv_drive
#define PI_F 3.14159265f

param_dv_t param_dv;

static int16_t s_set[P_COUNT];
static const param_store_t *s_store;
static int16_t s_default_type;

typedef struct { int16_t low, up; } limit_t;

/* 인덱스 = 파라미터 번호. up 이 10000 을 넘으면 자리별 한계다.
 * 31·38·39·44 는 새 파라미터이고 36 하한은 0 이다 */
static const limit_t s_limit[P_PARA_INIT + 1] = {
    {0, 0},
    {7, 60}, {3, 60}, {3, 25}, {0, 3}, {0, 99}, {0, 1},
    {10000, 10055}, {10000, 10022}, {1, 10}, {-9, 9}, {7, 60}, {5, 60},
    {20, 60}, {0, 999}, {0, 99}, {0, 9}, {10000, 10022}, {5, 9},
    {0, 300}, {0, 99}, {0, 20}, {10, 500}, {1, 250}, {30, 300},
    {0, 99}, {0, 999}, {0, 1}, {0, 999}, {1, 99}, {0, 99},
    {0, 1}, {0, 900}, {10000, 10066}, {1, 99}, {0, 999}, {0, 99},
    {0, 989}, {0, 2}, {0, 4095}, {0, 99}, {0, 99}, {0, 10011},
    {-999, 999}, {0, 99}, {0, 99}, {0, 99}
};

/* 1~45 기본값(INT 77). 26(P_AGV_TYPE)은 param_default_type 으로 받는다. 38 은 새 파라미터라 0 */
static const int16_t s_defaults[P_SAVED_LAST] = {
    30, 25, 13, 0, 0, 1, 11, 11, 10, 0, 50, 8, 40, 0, 0, 0, 22, 5, 35, 0, 0, 202, 40, 60, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 54, 60, 5, 0, 0, 0, 5, 0, 0, 0, 0, 11
};

int16_t param_digit(int16_t value, int16_t place)
{
    int16_t tmp;
    if (place <= 0) { place = 1; }
    tmp = (int16_t)(value % (place * 10));
    return tmp ? (int16_t)(tmp / place) : 0;
}

int16_t param_get(uint16_t idx)        { return (idx < P_COUNT) ? s_set[idx] : 0; }

#if AGV_DEV_TESTS
/* TST(param.h): 0 기본(현장 이미지와 같다), 2 예전(53차 TST 0. 비교용). 1 은 비었다(limit_ok 가 받지 않는다) */
int16_t param_tst_old;

void param_tst_apply(void)
{
    D.p_kp = (float)param_gain(P_STEER_KP);      /* 0 이면 steer.c 가 경사로 다시 잡는다 */
    D.p_ki = (float)param_gain(P_STEER_KI) * 0.1f;
    param_tst_old = (int16_t)(s_set[P_DEV_TEST] == 2);
}
#endif

int16_t param_gain(uint16_t idx)
{
    int16_t v = param_get(idx);
#if AGV_DEV_TESTS
    if (s_set[P_DEV_TEST] == 2) { return v; }    /* 예전: 0 = 37차 자동 이득 */
#endif
    return v ? v : ((idx == P_STEER_KP) ? PARAM_KP_DEFAULT : PARAM_KI_DEFAULT);
}
int16_t param_opt(uint16_t bit)        { return (((uint16_t)s_set[P_OPT_FLAGS] & bit) != 0) ? 1 : 0; }
int16_t param_quirk(uint16_t bit)      { return (((uint16_t)s_set[P_LEGACY_QUIRKS] & bit) != 0) ? 1 : 0; }

int16_t param_rpm(int16_t m_per_min)
{
    return (int16_t)((float)m_per_min * param_dv.one_meter_rpm);
}

void param_default_type(int16_t kind) { s_default_type = kind; }

/* 체크섬 밖 EEPROM 에 따로 두는 파라미터. 현장 장비의 체크섬 범위를 건드리지 않는다 */
typedef struct { uint16_t idx, addr; int16_t low, up, def; } extra_t;
static const extra_t s_extra[] = {
    { P_GUIDE_DIST_MM,   PARAM_GUIDE_DIST_ADDR, PARAM_GUIDE_DIST_LOW, PARAM_GUIDE_DIST_UP, PARAM_GUIDE_DIST_DEFAULT },
    { P_OPT_FLAGS,       116, 0, 63,  0 },
    { P_SPEED_FF,        117, 0, 999, 0 },
    { P_ENC_STALL_TICKS, 118, 0, 99,  0 },
    /* 공차 프로필. 한계 위는 적재 칸(1·3·12·13번)과 같고 0 = 적재 값. 77~81 은 KEEP 의 빈 칸이다 */
    { P_EMPTY_SPEED, PARAM_EMPTY_ADDR0,      0, 60, 0 },
    { P_EMPTY_TURN,  PARAM_EMPTY_ADDR0 + 1u, 0, 25, 0 },
    { P_EMPTY_ACC,   PARAM_EMPTY_ADDR0 + 2u, 0, 60, 0 },
    { P_EMPTY_DCC,   PARAM_EMPTY_ADDR0 + 3u, 0, 60, 0 },
    { P_LOAD_DETECT, PARAM_EMPTY_ADDR0 + 4u, 0, 1,  0 }
#if AGV_DEV_TESTS
    , { P_DEV_TEST,  PARAM_EMPTY_ADDR0 + 5u, 0, 2,  0 }   /* 82: 레거시 KEEP 빈 칸(77~88). 53차까지의 3~8 은 부팅 때 0 */
#endif
};
static int16_t s_empty;
#define EXTRA_COUNT (sizeof(s_extra) / sizeof(s_extra[0]))

static const extra_t *extra_find(uint16_t i)
{
    uint16_t k;
    for (k = 0; k < EXTRA_COUNT; k++) { if (s_extra[k].idx == i) { return &s_extra[k]; } }
    return 0;
}

static int16_t limit_ok(uint16_t i, int16_t v)
{
    int16_t p;
    const extra_t *e = extra_find(i);
#if AGV_DEV_TESTS
    if ((i == P_DEV_TEST) && (v == 1)) { return 0; }     /* TST 1 은 비었다(54차). 53차의 1 도 부팅 때 0 */
#endif
    if (e != 0) { return (v >= e->low) && (v <= e->up); }
    if (i > P_PARA_INIT) { return 1; }
    if ((s_limit[i].low > 10000) || (s_limit[i].up > 10000)) {
        for (p = 1; p <= 1000; p = (int16_t)(p * 10)) {
            int16_t up = param_digit(s_limit[i].up, p);
            int16_t dig = param_digit(v, p);
            if ((up > 0) && ((dig < param_digit(s_limit[i].low, p)) || (dig > up))) { return 0; }
        }
        return 1;
    }
    return (v >= s_limit[i].low) && (v <= s_limit[i].up);
}

int16_t param_active(uint16_t idx)
{
    uint16_t e = (idx == P_SPEED_HIGH) ? P_EMPTY_SPEED : (idx == P_SPEED_TURN) ? P_EMPTY_TURN
               : (idx == P_ACC) ? P_EMPTY_ACC : (idx == P_DCC) ? P_EMPTY_DCC : 0u;
    if (s_empty && (e != 0u) && (s_set[e] >= s_limit[idx].low)) { return s_set[e]; }
    return param_get(idx);
}

int16_t param_profile_empty(void) { return s_empty; }

static int16_t sum_saved(void)
{
    uint16_t i;
    uint16_t sum = 0;
    for (i = 1; i <= P_SAVED_LAST; i++) { sum = (uint16_t)(sum + (uint16_t)s_set[i]); }
    return (int16_t)sum;
}

static void recalc_stop_len(void)
{
    param_dv.stop_len = (int16_t)((float)s_set[P_STOP_LEN_MM] * param_dv.one_length_pulse * 10.0f);
}

static void recalc_fwd_stop_len(void)
{
    param_dv.fwd_stop_len = (int16_t)((float)s_set[P_STOP_LEN_FWD_MM] * param_dv.one_length_pulse * 10.0f);
}

static void recalc_turn_pulses(void)
{
    param_dv.turn_90  = (int16_t)(((float)s_set[P_WHEEL_BASE_MM] * 10.0f * PI_F) / param_dv.one_pulse_length / 2.0f);
    param_dv.turn_180 = (int16_t)(param_dv.turn_90 * 2);
    param_dv.turn_16  = (int16_t)((param_dv.turn_90 / 90) << 4);
    param_dv.turn_32  = param_dv.turn_16;   /* 원본도 16 등분 값과 같다 */
}

static void recalc_llow(void)
{
    int16_t half = (int16_t)(param_dv.low_speed / 2);
    param_dv.llow_speed = (half > param_dv.llow_min) ? half : param_dv.llow_min;
}

static void recalc_low(void)
{
    int16_t low = param_rpm(s_set[P_SPEED_LOW]);
    param_dv.low_speed = (low < param_dv.target_speed) ? low : param_dv.target_speed;
}

/* init_AGV_Scale 대응 */
static void recalc_all(void)
{
    float dia = (float)s_set[P_WHEEL_DIA_MM] * PI_F;
    float gear = (float)s_set[P_GEAR_RATIO];
    int16_t k;

    param_dv.one_meter_rpm = (1000.0f / dia) * gear;
    param_dv.one_rpm_meter = 1.0f / param_dv.one_meter_rpm;
    param_dv.one_pulse_length = (dia / gear) / 12.0f;
    param_dv.one_length_pulse = 1.0f / param_dv.one_pulse_length;

    param_dv.llow_min = param_rpm(s_limit[P_SPEED_HIGH].low);
    param_dv.turn_min = param_dv.llow_min;
    param_dv.guide_dist_mm = (float)s_set[P_GUIDE_DIST_MM];
#if AGV_DEV_TESTS
    param_dv.speed_ff = (float)s_set[P_SPEED_FF] / 1000.0f;
#endif
    param_dv.stall_ticks = s_set[P_ENC_STALL_TICKS];
    param_dv.turn_speed = param_rpm(param_active(P_SPEED_TURN));
    if (param_dv.turn_speed < param_dv.turn_min) { param_dv.turn_speed = param_dv.turn_min; }
    recalc_turn_pulses();
    param_dv.jog_speed = param_rpm(s_set[P_SPEED_JOG]);
    param_dv.target_speed = param_rpm(param_active(P_SPEED_HIGH));
    recalc_low();
    /* SPN 은 3 m/min 이상(LOS 하한과 같다). 범위 표 하한(0)은 두고 파생값만 묶는다: 표를 올리면 저장된 0~2(기본 0)가 부팅 때
     * 한계 위반으로 모든 파라미터를 기본값으로 되돌린다 */
    param_dv.min_speed = param_rpm((s_set[P_SPEED_MIN] >= 3) ? s_set[P_SPEED_MIN] : 3);
    recalc_llow();

    param_dv.tag_pulse[0] = 0;
    for (k = 1; k <= 11; k++) {
        param_dv.tag_pulse[k] = (int16_t)((float)k * (50.0f * param_dv.one_length_pulse));
    }
    param_dv.straight_len = 3L * param_dv.tag_pulse[10];

    param_dv.display_mode = s_set[P_DISPLAY_MODE];
    param_dv.turn_mode = param_digit(s_set[P_TURN_FORBACK], 10);
    param_dv.forback_mode = param_digit(s_set[P_TURN_FORBACK], 1);
    param_dv.acc = param_active(P_ACC);
    param_dv.dcc = param_active(P_DCC);
    recalc_stop_len();
    recalc_fwd_stop_len();
    param_dv.auto_drive_mode = (int16_t)(s_set[P_AUTO_DRIVE_MODE] & 1);
    param_dv.auto_slow_mode = (int16_t)((s_set[P_AUTO_DRIVE_MODE] >> 1) & 1);
    param_dv.agv_type_mode = (int16_t)(10 * param_digit(s_set[P_AGV_TYPE], 100) + param_digit(s_set[P_AGV_TYPE], 10));
    param_dv.actuator_delay = (int16_t)(s_set[P_SPEED_MIN] * 10);
    param_dv.for_bumper = param_digit(s_set[P_BUMPER_USE], 10);
    param_dv.back_bumper = param_digit(s_set[P_BUMPER_USE], 1);
    param_dv.for_obs = param_digit(s_set[P_OBS_USE], 10);
    param_dv.back_obs = param_digit(s_set[P_OBS_USE], 1);
    param_dv.acs_group = param_digit(s_set[P_ACS_STANDBY], 10);
    param_dv.acs_standby = param_digit(s_set[P_ACS_STANDBY], 1);
}

void param_profile(int16_t empty)
{
    s_empty = (int16_t)(empty != 0);
    param_dv.target_speed = param_rpm(param_active(P_SPEED_HIGH));
    param_dv.turn_speed = param_rpm(param_active(P_SPEED_TURN));
    if (param_dv.turn_speed < param_dv.turn_min) { param_dv.turn_speed = param_dv.turn_min; }
    param_dv.acc = param_active(P_ACC);
    param_dv.dcc = param_active(P_DCC);
    recalc_low();
    recalc_llow();
}

/* init_para 대응. 번호마다 고치는 파생값이 다르다 */
static void recalc_one(uint16_t i)
{
    switch (i) {
    case P_SPEED_HIGH:
        param_dv.target_speed = param_rpm(param_active(P_SPEED_HIGH));
        recalc_llow();
        break;
    case P_SPEED_LOW:
        recalc_low();
        recalc_llow();
        break;
    case P_SPEED_TURN:
        param_dv.turn_speed = param_rpm(param_active(P_SPEED_TURN));
        recalc_turn_pulses();
        break;
    case P_AUTO_DRIVE_MODE:
        param_dv.auto_drive_mode = (int16_t)(s_set[P_AUTO_DRIVE_MODE] & 1);
        param_dv.auto_slow_mode = (int16_t)((s_set[P_AUTO_DRIVE_MODE] >> 1) & 1);
        break;
    case P_BUMPER_USE:
        param_dv.for_bumper = param_digit(s_set[P_BUMPER_USE], 10);
        param_dv.back_bumper = param_digit(s_set[P_BUMPER_USE], 1);
        break;
    case P_OBS_USE:
        param_dv.for_obs = param_digit(s_set[P_OBS_USE], 10);
        param_dv.back_obs = param_digit(s_set[P_OBS_USE], 1);
        break;
    case P_DISPLAY_MODE:  param_dv.display_mode = s_set[P_DISPLAY_MODE]; break;
    case P_SPEED_JOG:     param_dv.jog_speed = param_rpm(s_set[P_SPEED_JOG]); break;
    case P_ACC:           param_dv.acc = param_active(P_ACC); break;
    case P_DCC:           param_dv.dcc = param_active(P_DCC); break;
    case P_EMPTY_SPEED:
    case P_EMPTY_TURN:
    case P_EMPTY_ACC:
    case P_EMPTY_DCC:     if (s_empty) { param_profile(1); } break;
    case P_TURN_FORBACK:
        param_dv.turn_mode = param_digit(s_set[P_TURN_FORBACK], 10);
        param_dv.forback_mode = param_digit(s_set[P_TURN_FORBACK], 1);
        break;
    case P_STOP_LEN_MM:   recalc_stop_len(); break;
    case P_STEER_KP:      D.p_kp = (float)param_gain(P_STEER_KP); break;         /* 0 = 권장값(TST 2 는 자동 이득) */
    case P_STEER_KI:      D.p_ki = (float)param_gain(P_STEER_KI) * 0.1f; break;
    case P_WHEEL_DIA_MM:
    case P_GEAR_RATIO:
    case P_WHEEL_BASE_MM: recalc_all(); break;
    case P_GUIDE_DIST_MM: param_dv.guide_dist_mm = (float)s_set[P_GUIDE_DIST_MM]; break;
    case P_ENC_STALL_TICKS: param_dv.stall_ticks = s_set[P_ENC_STALL_TICKS]; break;
#if AGV_DEV_TESTS
    case P_SPEED_FF:      param_dv.speed_ff = (float)s_set[P_SPEED_FF] / 1000.0f; break;
    case P_DEV_TEST:      param_tst_apply(); break;
#endif
    case P_AGV_TYPE:
        param_dv.agv_type_mode = (int16_t)(10 * param_digit(s_set[P_AGV_TYPE], 100) + param_digit(s_set[P_AGV_TYPE], 10));
        break;
    case P_SPEED_MIN:     param_dv.actuator_delay = (int16_t)(s_set[P_SPEED_MIN] * 10); break;
    case P_STOP_LEN_FWD_MM:  recalc_fwd_stop_len(); break;
    case P_ACS_STANDBY:
        param_dv.acs_group = param_digit(s_set[P_ACS_STANDBY], 10);
        param_dv.acs_standby = param_digit(s_set[P_ACS_STANDBY], 1);
        break;
    default:
        break;
    }
}

/* 체크섬 먼저, 값은 그다음 (원본 순서) */
static void store_one(uint16_t i)
{
    s_set[0] = sum_saved();
    s_store->write(0, (uint16_t)s_set[0]);
    s_store->write(i, (uint16_t)s_set[i]);
}

static void store_all(void)
{
    uint16_t i;
    s_set[0] = sum_saved();
    s_store->write(0, (uint16_t)s_set[0]);
    for (i = 1; i <= P_SAVED_LAST; i++) { s_store->write(i, (uint16_t)s_set[i]); }
}

int16_t param_set_ram(uint16_t idx, int16_t value)
{
    if ((idx == 0) || (idx >= P_COUNT) || !limit_ok(idx, value)) { return -1; }
    s_set[idx] = value;
    recalc_one(idx);
    return 0;
}

/* 저장 규칙: LOS 는 DRS 를 넘지 않는다. LOS 를 크게 쓰면 DRS 로 자르고,
 * DRS 를 LOS 보다 낮게 쓰면 LOS 도 함께 낮춰 저장한다. param_set_ram 은 이 규칙 밖이다(임시 값) */
int16_t param_set(uint16_t idx, int16_t value)
{
    int16_t los_cut = 0;
    if ((idx == P_SPEED_LOW) && limit_ok(idx, value) && (value > s_set[P_SPEED_HIGH])) { value = s_set[P_SPEED_HIGH]; }
    if (param_set_ram(idx, value) != 0) { return -1; }
    if ((idx == P_SPEED_HIGH) && (s_set[P_SPEED_LOW] > value)) {
        s_set[P_SPEED_LOW] = value;
        recalc_one(P_SPEED_LOW);
        los_cut = 1;
    }
    if (idx <= P_SAVED_LAST) { store_one(idx); }
    if (los_cut) { store_one(P_SPEED_LOW); }
    {
        const extra_t *e = extra_find(idx);
        if (e != 0) { s_store->write(e->addr, (uint16_t)value); }
    }
    return 0;
}

static void after_load(void)
{
    recalc_all();
    D.p_kp = (float)param_gain(P_STEER_KP);
    D.p_ki = (float)param_gain(P_STEER_KI) * 0.1f;
#if AGV_DEV_TESTS
    param_tst_apply();
#endif
    /* 특정 차종군은 자동 주행 모드를 강제하고 저장한다 (AGV_Type_Mode == 60) */
    if (param_dv.agv_type_mode == 60) {
        (void)param_set(P_AUTO_DRIVE_MODE, 3);
    }
}

void param_default(void)
{
    uint16_t i;
    for (i = 1; i <= P_SAVED_LAST; i++) { s_set[i] = s_defaults[i - 1]; }
    s_set[P_AGV_TYPE] = s_default_type;
    store_all();
    after_load();
}

/* 현장 장비의 31·38·39·44 에는 옛 값이 남아 있다. 표시를 마지막에 쓰므로
 * 도중에 전원이 끊기면 다음 부팅에서 다시 수행한다. 체크섬 범위 밖인 주소 50 을 쓴다.
 * 반환 0 = 정상, 1 = 읽기 검증 실패(표시를 쓰지 않음) */
static int16_t migrate_new_params(void)
{
    static const uint16_t idx[4] = { P_COMM_LINK, P_GUIDE_CURVE_DETECT, P_LEGACY_QUIRKS, P_GUIDE_TIMEOUT };
    uint16_t i, k;
    uint16_t sum = 0;

    if (s_store->read(PARAM_MIGRATE_ADDR) == PARAM_MIGRATE_MARK) { return 0; }
    for (k = 0; k < 4; k++) { s_store->write(idx[k], 0); }
    for (i = 1; i <= P_SAVED_LAST; i++) { sum = (uint16_t)(sum + s_store->read(i)); }
    s_store->write(0, sum);
    for (k = 0; k < 4; k++) {
        if (s_store->read(idx[k]) != 0) { return 1; }
    }
    if (s_store->read(0) != sum) { return 1; }
    s_store->write(PARAM_MIGRATE_ADDR, PARAM_MIGRATE_MARK);
    return (s_store->read(PARAM_MIGRATE_ADDR) == PARAM_MIGRATE_MARK) ? 0 : 1;
}

/* 턴 속도 상한을 낮췄다. 옛 값이 한계 위반이 되면 파라미터 전체가 기본값으로 지워진다.
 * 그 값만 상한으로 줄여 다시 저장한다 */
static void clamp_turn_speed(void)
{
    uint16_t i, sum = 0;
    int16_t up = s_limit[P_SPEED_TURN].up;

    if ((int16_t)s_store->read(P_SPEED_TURN) <= up) { return; }
    s_store->write(P_SPEED_TURN, (uint16_t)up);
    for (i = 1; i <= P_SAVED_LAST; i++) { sum = (uint16_t)(sum + s_store->read(i)); }
    s_store->write(0, sum);
}

/* 반환 1 정상, 0 체크섬 불일치, -1 한계 위반 */
static int16_t read_all(void)
{
    uint16_t i;
    for (i = 0; i <= P_SAVED_LAST; i++) {
        s_set[i] = (int16_t)s_store->read(i);
        if ((i >= 1) && !limit_ok(i, s_set[i])) { return -1; }
    }
    return (s_set[0] == sum_saved()) ? 1 : 0;
}

void param_store_set(const param_store_t *store) { s_store = store; }

void param_limit(uint16_t idx, int16_t *low, int16_t *up)
{
    *low = (idx <= P_PARA_INIT) ? s_limit[idx].low : 0;
    *up = (idx <= P_PARA_INIT) ? s_limit[idx].up : 0;
    {
        const extra_t *e = extra_find(idx);
        if (e != 0) { *low = e->low; *up = e->up; }
    }
}

int16_t param_boot(const param_store_t *store, int16_t version)
{
    uint16_t attempt;
    int16_t result = 0;
    int16_t status = 0;

    s_store = store;
    s_empty = 0;
    s_set[P_PARA_INIT] = 0;
    s_set[P_VERSION] = version;
    s_set[P_BAT_DV] = 0;
    s_set[P_BAT_PERCENT] = 0;

    if (migrate_new_params() != 0) { status = ERR_PARAM_CHECKSUM; }
    clamp_turn_speed();
    {   /* 새 파라미터는 체크섬 밖에 둔다. 빈 EEPROM 이나 범위 밖이면 기본값 */
        uint16_t k;
        for (k = 0; k < EXTRA_COUNT; k++) {
            int16_t g = (int16_t)s_store->read(s_extra[k].addr);
            s_set[s_extra[k].idx] = limit_ok(s_extra[k].idx, g) ? g : s_extra[k].def;
        }
    }
    for (attempt = 0; attempt < 3; attempt++) {
        result = read_all();
        if (result == 1) { break; }
    }
    /* 한계 위반이면 기본값으로 초기화하고, 체크섬 불일치는 읽은 값을 그대로 쓴다 (원본 정책) */
    if (result < 0) {
        param_default();
        return status;
    }
    if (status != 0) {
        /* 검증에 실패한 칸은 믿지 않는다. 표시가 없으므로 다음 부팅에서 다시 시도한다 */
        s_set[P_COMM_LINK] = s_set[P_GUIDE_CURVE_DETECT] = 0;
        s_set[P_LEGACY_QUIRKS] = s_set[P_GUIDE_TIMEOUT] = 0;
    }
    after_load();
    return status;
}
