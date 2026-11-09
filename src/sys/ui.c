/* ui.c - 키, 파라미터 편집, 7세그, 탑램프·부저. 키 분기는 원본 라이브러리 디스어셈블을 따른다 */
#include "agv/ui.h"
#include "agv/acs.h"
#include "agv/comm.h"
#include "agv/drive.h"
#include "agv/hal.h"
#include "agv/io.h"
#include "agv/ioport.h"
#include "agv/link.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/seq.h"
#include "agv/stop.h"
#include "agv/tagact.h"
#include "agv/tune.h"
#include "agv/ident.h"
#include "agv/rpmtest.h"

#define D agv_drive
#define P param_dv

#define KEY_CONFIRM      5      /* 같은 값 6회 = 60 ms */
#define KEY_LONG         25
#define BLINK_TIMING     100    /* 2 ms 단위 */
#define BLINK_RPM        200
#define BACKUP_MSG_TICKS 500
#define PULSE_TICKS      50     /* 500 ms 마다 램프 처리 */
#define HISTORY_TICKS    500
#define GROUP_USE_LAST   15     /* 메뉴 순서표의 그룹 경계 */
#define GROUP_OTHER_FIRST 16
#define GROUP_OTHER_LAST 52
#define PARA_EDIT_LAST   46     /* 이 번호까지 편집한다 */
#define EDIT_OFF_FLAG    0
#define SCREEN_LIST      10     /* 이 값을 더하면 번호 목록 화면 */
#define SEG_BRAKE2       0x80u  /* ioport.h 참조 */
#define KEY_OUT_MASK     0x60u
/* 원본은 행 출력 직후에 읽어 한 박자 앞 행 값을 받았다(실차 키 배치로 확인).
 * 여기서는 다음 틱에 읽으므로 row[0](MODE 쪽 값 << 3)에 맞게 출력을 바꿔 둔다 */
#define KEY_OUT_ROW0     0x40u
#define KEY_OUT_ROW1     0x20u
#define DISP_DIGITS      5
#define TAG_SEG_OFFSET   54     /* 'T' 같은 글자를 세그먼트 색인으로 */
#define DIGIT_NONE       39
#define DIGIT_MINUS      37
#define DIGIT_UNDER      38

/* 세그먼트 표: 0~9, '.', A~Z 대용, -, _, 공백 */
static const uint16_t s_seg[] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x27, 0x7F, 0x67, 0x80,
    0x77, 0x7C, 0x58, 0x5E, 0x79, 0x71, 0x6F, 0x74, 0x04, 0x0E, 0x70, 0x38, 0x55, 0x54, 0x5C,
    0x73, 0x7B, 0x50, 0x6D, 0x78, 0x1C, 0x3E, 0x6A, 0x14, 0x6E, 0x49, 0x40, 0x08, 0x00, 0xFF
};
#define SEG_DOT  10
#define SEG_COUNT (sizeof(s_seg) / sizeof(s_seg[0]))

/* 메뉴 순서 → 파라미터 번호 */
static const uint16_t s_menu[] = {
    P_SPEED_HIGH, P_SPEED_LOW, P_SPEED_TURN, P_BUMPER_USE, P_OBS_USE, P_AUTO_START_SEC, P_TURN_FORBACK,
    P_STOP_LEN_MM, P_GUIDE_DIST_MM, P_WHEEL_DIA_MM, P_GEAR_RATIO, P_WHEEL_BASE_MM, P_AGV_ID, P_AGV_TYPE, P_SPEED_MIN,
    P_STOP_LEN_FWD_MM,
    P_AUTO_DRIVE_MODE, P_INTERFACE_MODE, P_SOUND, P_DISPLAY_MODE, P_BAT_OFFSET_DV, P_SPEED_JOG, P_ACC, P_DCC,
    P_STATION_NO, P_BRANCH_MODE, P_TAG_DIGITS, P_STEER_KP, P_STEER_KI, P_SWITCH_START_NUM, P_TAG_TYPE,
    P_COMM_LINK, P_ACS_STANDBY, P_BRANCH_OFFSET, P_CROSS_IN_CM, P_ACS_RETRY_LEGACY, P_ACS_RETRY_SEC,
    P_GUIDE_CURVE_DETECT, P_LEGACY_QUIRKS, P_ERR_REPORT_MIN, P_CROSS_RETRY_100MS, P_CROSS_MODE,
    P_STEER_RPM_OFFSET, P_GUIDE_TIMEOUT, P_OPT_FLAGS, P_SPEED_FF, P_ENC_STALL_TICKS, P_ACS_VERSION, P_LAMP_EXT_USE, P_PARA_INIT, P_VERSION, P_BAT_DV,
    P_BAT_PERCENT
};

/* 번호별 화면: 글자 3개 + 값 자릿수. 자릿수 0 은 ON/OFF 표시 */
typedef struct { char c[3]; uint16_t digits; } para_view_t;
static const para_view_t s_view[P_COUNT] = {
    {"   ", 2},
    {"DRS", 2}, {"LOS", 2}, {"TNS", 2}, {"ATD", 1}, {"ITM", 2}, {"SND", 0}, {"BPS", 2}, {"OBS", 2}, {"DSM", 2},
    {"BO-", 2}, {"JSP", 2}, {"ACC", 2}, {"DCC", 2}, {"AS ", 3}, {"ST ", 3}, {"BRM", 1}, {"TFB", 2}, {"TAG", 1},
    {"SL ", 3}, {"K_P", 2}, {"K_I", 2}, {"WS ", 3}, {"RG ", 3}, {"WW ", 3}, {"ID_", 2}, {"AT ", 3}, {"LO ", 3},
    {"ACD", 2}, {"SSN", 2}, {"TTY", 1}, {"MPU", 2}, {"FL ", 3}, {"ASS", 2}, {"BGO", 2}, {"CD ", 3}, {"WRT", 2},
    {"CT ", 3}, {"SO-", 2}, {"SU ", 3}, {"EST", 2}, {"RCI", 2}, {"LUS", 2}, {"C- ", 3}, {"CO-", 2}, {"AV-", 2},
    {"INT", 2}, {"V_ ", 3}, {"VT ", 3}, {"CR ", 3},
    {"GD ", 3}, {"OPT", 2}, {"FF ", 3}, {"ES ", 2}
};
static const char s_lamp_name[UI_LAMP_EXT_COUNT] = { 'G', 'R', 'Y' };

static struct {
    /* 키 */
    uint16_t scan, row[2], seg_b7;
    volatile uint16_t buffer;     /* Read_Key_buffer */
    uint16_t buffer_old, data, data_old, confirm;
    uint16_t long_cnt, long_press;
    /* 파라미터 화면 */
    int16_t  screen;              /* Flag_para_display_set_on */
    int16_t  edit;                /* Flag_para_edit_on */
    int16_t  place;               /* Flag_para_10Add */
    int16_t  menu;                /* DA_PARA_NUMBER */
    int16_t  para;                /* para_number */
    int16_t  value;               /* Para_value_temp */
    int16_t  lamp_ext[UI_LAMP_EXT_COUNT];
    volatile int16_t msg_save, msg_init;
    /* 표시 */
    volatile uint16_t digit[DISP_DIGITS], dot, flick;   /* dot·flick 는 자리별 비트 */
    uint16_t cursor;
    uint16_t blink_cnt, blink_on;
    volatile uint16_t msg_ticks;
    /* 램프 */
    uint16_t pulse_cnt, pulse_flag;
    /* 태그 이력 표시 */
    uint16_t hist_ticks, hist_n;
    /* IO 시험 출력. 0 = 켜짐 */
    uint16_t test_out[IO_TEST_OUT_LAST + 1u];
} s_ui;

void ui_reset(void)
{
    uint16_t i;
    uint16_t *p = (uint16_t *)&s_ui;
    for (i = 0; i < sizeof(s_ui) / sizeof(uint16_t); i++) { p[i] = 0; }
    s_ui.place = 1;
    s_ui.seg_b7 = SEG_BRAKE2;
    for (i = 0; i < DISP_DIGITS; i++) { s_ui.digit[i] = DIGIT_NONE; }
}

int16_t ui_para_number(void)             { return s_ui.para; }
int16_t ui_lamp_ext(uint16_t i)          { return s_ui.lamp_ext[i]; }
void    ui_lamp_ext_set(uint16_t i, int16_t v) { s_ui.lamp_ext[i] = v; }

uint16_t ui_param_label(uint16_t idx, char *c3)
{
    const para_view_t *v = &s_view[(idx < P_COUNT) ? idx : 0u];
    c3[0] = v->c[0];
    c3[1] = v->c[1];
    c3[2] = v->c[2];
    return v->digits;
}

/* show_param_value 와 같은 자리: 이름 3글자 위에 값 자릿수만큼 오른쪽부터. 음수 '-' 는 3자리 이상이면 둘째, 아니면 셋째 */
void ui_param_text(uint16_t idx, int16_t v, char *c)
{
    uint16_t n = ui_param_label(idx, c), i;
    int16_t m = (v < 0) ? (int16_t)-v : v;

    c[3] = c[4] = ' ';
    if (n == 0u) {
        c[3] = 'O';
        c[4] = (v == 1) ? 'N' : 'F';
        return;
    }
    for (i = 0; i < n; i++) {
        c[4u - i] = (char)('0' + m % 10);
        m = (int16_t)(m / 10);
    }
    if (v < 0) { c[(n >= 3u) ? 1u : 2u] = '-'; }
}

void ui_digits(uint16_t *out)
{
    uint16_t i;
    for (i = 0; i < DISP_DIGITS; i++) { out[i] = s_ui.digit[i]; }
}

/* ---- 2 ms ISR ---- */

/* 튜닝 중에는 비상정지·범퍼·앰프·장애물만 오류로 보인다 */
static int16_t safety_error(void)
{
    if ((D.system_err == ERR_ESTOP) || (D.system_err == ERR_BUMPER)
        || (D.system_err == ERR_AMP_L_OVERLOAD) || (D.system_err == ERR_AMP_R_OVERLOAD)
        || (D.system_err == ERR_ENC_L_STALL) || (D.system_err == ERR_ENC_R_STALL)) {
        return D.system_err;
    }
    return (D.action_err == ERR_OBSTACLE) ? D.action_err : 0;
}

static uint16_t any_error(void)
{
    if (tune_active()) { return (uint16_t)(safety_error() != 0); }
    return (D.system_err != 0) || (D.stop_err != 0) || (D.action_err != 0);
}

static void blink_step(void)
{
    uint16_t blink = (D.ui_mode == UI_PARAM) || (D.ui_mode == UI_IO_OUT) || (tune_flick() != 0u)
                  || ((D.ui_mode == UI_AUTO) && any_error())
                  || ((D.ui_mode == UI_JOG) && ((D.rpm_avg[0] > BLINK_RPM) || (D.rpm_avg[1] > BLINK_RPM)));

    if (!blink) {
        s_ui.blink_on = 1;
    }
    else if (s_ui.blink_cnt < BLINK_TIMING) {
        s_ui.blink_cnt++;
        s_ui.blink_on = 0;
    }
    else {
        s_ui.blink_on = 1;
        s_ui.blink_cnt = (s_ui.blink_cnt == 2u * BLINK_TIMING) ? 0u : (uint16_t)(s_ui.blink_cnt + 1u);
    }
    if (s_ui.msg_save || s_ui.msg_init) {
        if (++s_ui.msg_ticks > BACKUP_MSG_TICKS) {
            s_ui.msg_ticks = 0;
            s_ui.msg_save = 0;
            s_ui.msg_init = 0;
        }
    }
}

/* 앞 두 자리는 뒤집어 달린 표시기다 */
static uint16_t seg_rotate(uint16_t x)
{
    return (uint16_t)(((x & 0x07u) << 3) | ((x & 0x38u) >> 3) | (x & 0xC0u));
}

static uint16_t seg_code(uint16_t order, uint16_t value, uint16_t dot)
{
    uint16_t x = s_seg[(value < SEG_COUNT) ? value : DIGIT_NONE];
    if (dot && s_ui.blink_on) { x = (uint16_t)(x + s_seg[SEG_DOT]); }
    return (order < 2u) ? seg_rotate(x) : x;
}

uint16_t ui_key(void) { return s_ui.buffer; }

void ui_isr(uint16_t key_input)
{
    uint16_t order, value, dot, key_out;

    /* 키 행을 번갈아 읽는다 */
    s_ui.row[s_ui.scan] = key_input;
    s_ui.scan ^= 1u;
    s_ui.buffer = (uint16_t)((s_ui.row[0] << 3) + s_ui.row[1]);
    key_out = (s_ui.scan == 0u) ? KEY_OUT_ROW0 : KEY_OUT_ROW1;

    blink_step();
    s_ui.cursor = (uint16_t)((s_ui.cursor + 1u) % DISP_DIGITS);
    order = s_ui.cursor;
    value = s_ui.digit[order];
    if ((D.ui_mode == UI_PARAM) || (D.ui_mode == UI_IO_OUT) || (tune_flick() != 0u)) {
        if (!s_ui.blink_on && ((s_ui.flick >> order) & 1u)) { value = DIGIT_NONE; }
    }
    else if (!s_ui.blink_on) {
        value = DIGIT_NONE;
    }
    dot = (uint16_t)((s_ui.dot >> order) & 1u);
    if ((order == 4u) && comm_rx_flash()) { dot = 1; }
    hal_bus_write(BUS_SEG, (uint16_t)(s_ui.seg_b7 | key_out | (~(1u << order) & 0x1Fu)));
    hal_bus_write(BUS_DISP, seg_code(order, value, dot));
}

/* ---- 파라미터 편집 ---- */

static uint16_t letter(char c)
{
    if (c == '-') { return DIGIT_MINUS; }
    if (c == '_') { return DIGIT_UNDER; }
    if ((c >= 'A') && (c <= 'Z')) { return (uint16_t)(11 + (c - 'A')); }
    return DIGIT_NONE;
}

static int16_t para_value(int16_t para)
{
    if ((para >= UI_PARA_LAMP_FIRST) && (para <= UI_PARA_LAMP_LAST)) {
        return s_ui.lamp_ext[para - UI_PARA_LAMP_FIRST];
    }
    return param_get((uint16_t)para);
}

static uint16_t para_digits(int16_t para)
{
    if ((para >= UI_PARA_LAMP_FIRST) && (para <= UI_PARA_LAMP_LAST)) { return 2; }
    return s_view[para].digits;
}

static int16_t abs16(int16_t v) { return (v < 0) ? (int16_t)-v : v; }

static void para_limits(int16_t para, int16_t *low, int16_t *up)
{
    if ((para >= UI_PARA_LAMP_FIRST) && (para <= UI_PARA_LAMP_LAST)) {
        *low = 0;
        *up = 36;
    }
    else {
        param_limit((uint16_t)para, low, up);
    }
}

static int16_t place_value(void)
{
    switch (s_ui.place) {
    case 4:  return 1000;
    case 3:  return 100;
    case 2:  return 10;
    default: return 1;
    }
}

static void value_step(int16_t sign)
{
    int16_t low, up, p = place_value();

    para_limits(s_ui.para, &low, &up);
    if ((low > 10000) || (up > 10000)) {
        int16_t d = param_digit(s_ui.value, p);
        int16_t d_low = param_digit(low, p);
        int16_t d_up = param_digit(up, p);
        if ((d_up > 0) && ((sign > 0) ? (d_up > d) : (d_low < d))) {
            s_ui.value = (int16_t)(s_ui.value + sign * p);
        }
        return;
    }
    s_ui.value = (int16_t)(s_ui.value + sign * p);
    if (s_ui.value < low) { s_ui.value = low; }
    else if (s_ui.value > up) { s_ui.value = up; }
}

static void menu_load(void)
{
    s_ui.para = (int16_t)s_menu[s_ui.menu];
    s_ui.value = param_get((uint16_t)s_ui.para);
}

static void lamp_load(void) { s_ui.value = s_ui.lamp_ext[s_ui.para - UI_PARA_LAMP_FIRST]; }

/* 목록·그룹 화면에서 위아래 */
static void list_step(int16_t sign)
{
    int16_t group = (int16_t)(s_ui.screen - SCREEN_LIST);

    if (group == UI_GROUP_LAMP) {
        s_ui.para = (int16_t)(s_ui.para + sign);
        if (s_ui.para > UI_PARA_LAMP_LAST)  { s_ui.para = UI_PARA_LAMP_FIRST; }
        if (s_ui.para < UI_PARA_LAMP_FIRST) { s_ui.para = UI_PARA_LAMP_LAST; }
        lamp_load();
        return;
    }
    s_ui.menu = (int16_t)(s_ui.menu + sign);
    if (group == UI_GROUP_USE) {
        if (s_ui.menu > GROUP_USE_LAST) { s_ui.menu = 0; }
        if (s_ui.menu < 0)              { s_ui.menu = GROUP_USE_LAST; }
    }
    else {
        if (s_ui.menu > GROUP_OTHER_LAST)  { s_ui.menu = GROUP_OTHER_FIRST; }
        if (s_ui.menu < GROUP_OTHER_FIRST) { s_ui.menu = GROUP_OTHER_LAST; }
    }
    menu_load();
}

static void group_step(int16_t sign)
{
    s_ui.screen = (int16_t)(s_ui.screen + sign);
    if (s_ui.screen > UI_GROUP_LAMP) { s_ui.screen = UI_GROUP_USE; }
    if (s_ui.screen < UI_GROUP_USE)  { s_ui.screen = UI_GROUP_LAMP; }
    if ((s_ui.screen == UI_GROUP_LAMP) && (param_get(P_LAMP_EXT_USE) == 0)) {
        s_ui.screen = (sign > 0) ? UI_GROUP_USE : UI_GROUP_OTHER;
    }
}

static void up_down(int16_t sign)
{
    if ((D.ui_mode == UI_AUTO) && D.flag_run) {
        int16_t low, up, v = (int16_t)(P.display_mode + sign);
        param_limit(P_DISPLAY_MODE, &low, &up);
        if (v > up)  { v = low; }
        if (v < low) { v = up; }
        P.display_mode = v;
    }
    if (D.ui_mode != UI_PARAM) { return; }
    if (s_ui.screen > SCREEN_LIST) {
        if (s_ui.edit == 1) {
            if (para_digits(s_ui.para) != 0u) { value_step(sign); }
            else { s_ui.value = s_ui.value ? 0 : 1; }
        }
        else {
            list_step(sign);
        }
    }
    else if (s_ui.screen > 0) {
        group_step(sign);
    }
}

static void key_shift(void)
{
    if ((D.ui_mode == UI_PARAM) && (s_ui.screen > SCREEN_LIST) && (s_ui.edit == 1)) {
        s_ui.place++;
        if (s_ui.place > (int16_t)para_digits(s_ui.para)) { s_ui.place = 1; }
    }
}

static void key_mode(void)
{
    if (run_auto_started()) { return; }
    if (D.ui_mode == UI_AUTO) {
        if (!D.flag_run && !D.flag_jog_on) {
            D.ui_mode = UI_JOG;
            D.control = RUN_JOG;
            if (!D.system_err && !D.stop_err && !D.estop) { D.flag_jog_on = 1; }
        }
    }
    else if (D.ui_mode == UI_JOG) {
        if (!D.flag_run) {
            D.ui_mode = UI_PARAM;
            D.flag_stopping = 1;
            link_select();
            link_guide_init();
        }
    }
    else if ((D.ui_mode == UI_PARAM) && (s_ui.screen == 0)) {
        D.ui_mode = UI_AUTO;
        D.control = RUN_DRIVE;
        link_select();
        link_guide_init();
    }
}

static void key_reset(void)
{
    if (D.ui_mode != UI_PARAM) { return; }
    if (s_ui.screen > SCREEN_LIST) {
        if (s_ui.edit == 1) {
            s_ui.edit = 0;
            s_ui.value = para_value(s_ui.para);
        }
        else {
            s_ui.screen = (int16_t)(s_ui.screen - SCREEN_LIST);
        }
    }
    else if (s_ui.screen > 0) {
        s_ui.screen = 0;
    }
}

static void enter_group(void)
{
    if (s_ui.screen == UI_GROUP_LAMP) {
        s_ui.para = UI_PARA_LAMP_FIRST;
        lamp_load();
    }
    else {
        s_ui.menu = (s_ui.screen == UI_GROUP_USE) ? 0 : GROUP_OTHER_FIRST;
        menu_load();
    }
    s_ui.screen = (int16_t)(s_ui.screen + SCREEN_LIST);
}

/* 파라미터 46 에 77·88·99 를 넣고 확정하면 기본값 세트를 쓴다 */
static void init_defaults(uint16_t kind)
{
    param_default(kind);
    seq_pick_profile();
    s_ui.msg_init = 1;
    s_ui.msg_ticks = 0;
    s_ui.edit = EDIT_OFF_FLAG;
}

static void commit(void)
{
    if (s_ui.para == P_PARA_INIT) {
        if (s_ui.value == 77)      { init_defaults(1); }
        else if (s_ui.value == 88) { init_defaults(2); }
        else if (s_ui.value == 99) { init_defaults(3); }
        return;
    }
    s_ui.edit = EDIT_OFF_FLAG;
    s_ui.msg_save = 1;
    s_ui.msg_ticks = 0;
    if (s_ui.screen == (SCREEN_LIST + UI_GROUP_LAMP)) {
        s_ui.lamp_ext[s_ui.para - UI_PARA_LAMP_FIRST] = s_ui.value;
        return;
    }
    (void)param_set((uint16_t)s_ui.para, s_ui.value);
    if ((s_ui.para == P_TURN_FORBACK) && (P.forback_mode == 0)) { run_relay(0, 1); }
}

static void key_set(void)
{
    if (D.ui_mode != UI_PARAM) { return; }
    if (s_ui.screen == 0) {
        s_ui.screen = UI_GROUP_USE;
    }
    else if (s_ui.screen < SCREEN_LIST) {
        enter_group();
    }
    else if (s_ui.edit == 1) {
        commit();
    }
    else {
        s_ui.edit = (s_ui.para <= PARA_EDIT_LAST) || ((s_ui.para >= P_GUIDE_DIST_MM) && (s_ui.para < P_COUNT))
                 || (s_ui.para >= UI_PARA_LAMP_FIRST);
        if ((int16_t)para_digits(s_ui.para) < s_ui.place) { s_ui.place = 1; }
    }
}

/* ---- IO 시험 ---- */

static void test_out_step(int16_t sign)
{
    int16_t last = (int16_t)(io_test_ext_board() ? IO_TEST_OUT_LAST : IO_TEST_BOARD_LAST);

    s_ui.value = (int16_t)(s_ui.value + sign * place_value());
    if (s_ui.value < 1)         { s_ui.value = 1; }
    else if (s_ui.value > last) { s_ui.value = last; }
}

static void test_key(void)
{
    if (s_ui.data == KEY_MODE) {
        D.ui_mode = (D.ui_mode == UI_IO_IN) ? UI_IO_OUT : UI_IO_IN;
        return;
    }
    if (D.ui_mode != UI_IO_OUT) { return; }
    switch (s_ui.data) {
    case KEY_SHIFT: s_ui.place = (s_ui.place == 1) ? 2 : 1; break;
    case KEY_UP:    test_out_step(1); break;
    case KEY_DOWN:  test_out_step(-1); break;
    case KEY_SET:   s_ui.test_out[s_ui.value] = 0; break;
    case KEY_RESET: s_ui.test_out[s_ui.value] = 1; break;
    default: break;
    }
}

static void key_process(void)
{
    if (s_ui.long_cnt > KEY_LONG) {
        s_ui.long_press = 1;
        s_ui.long_cnt = 0;
    }
    if (s_ui.buffer == s_ui.buffer_old) {
        if (++s_ui.confirm > KEY_CONFIRM) {
            s_ui.confirm = 0;
            s_ui.data = s_ui.buffer_old;
        }
        if (s_ui.buffer != 0u) { s_ui.long_cnt++; }
    }
    else {
        s_ui.buffer_old = s_ui.buffer;
        s_ui.confirm = 0;
        s_ui.long_cnt = 0;
    }
    if (D.ui_mode >= UI_IO_IN) {
        if (s_ui.data != s_ui.data_old) {
            test_key();
            s_ui.data_old = s_ui.data;
        }
        return;
    }
    if (s_ui.data == s_ui.data_old) {
        /* 길게 누르면 위·아래·자리 이동만 반복한다 */
        if (!s_ui.long_press || !((s_ui.data == KEY_UP) || (s_ui.data == KEY_DOWN) || (s_ui.data == KEY_SHIFT))) {
            return;
        }
    }
    if (tune_active()) {
        int r = tune_key(s_ui.data);
        if (r != 0) {
            if (r == 2) { s_ui.msg_save = 1; }
            s_ui.data_old = s_ui.data;
            s_ui.long_press = 0;
            return;
        }
        if (s_ui.data == KEY_MODE) {        /* 튜닝 모드에서는 파라미터·조그 화면으로 가지 않는다 */
            s_ui.data_old = s_ui.data;
            return;
        }
    }
    switch (s_ui.data) {
    case KEY_MODE:  key_mode(); break;
    case KEY_SHIFT: key_shift(); break;
    case KEY_UP:    up_down(1); break;
    case KEY_DOWN:  up_down(-1); break;
    case KEY_RESET: key_reset(); break;
    case KEY_SET:   key_set(); break;
    default: break;
    }
    s_ui.data_old = s_ui.data;
    s_ui.long_press = 0;
}

/* ---- 표시 내용 ---- */

static void show(uint16_t a, uint16_t b, uint16_t c, uint16_t d, uint16_t e)
{
    s_ui.digit[0] = a;
    s_ui.digit[1] = b;
    s_ui.digit[2] = c;
    s_ui.digit[3] = d;
    s_ui.digit[4] = e;
}

static uint16_t digit_of(int16_t v, int16_t place) { return (uint16_t)param_digit(abs16(v), place); }

static void show_number_tail(int16_t v, uint16_t blank_zero)
{
    s_ui.digit[2] = digit_of(v, 100);
    s_ui.digit[3] = digit_of(v, 10);
    s_ui.digit[4] = digit_of(v, 1);
    if (blank_zero && (s_ui.digit[2] == 0u)) {
        s_ui.digit[2] = DIGIT_NONE;
        if (s_ui.digit[3] == 0u) { s_ui.digit[3] = DIGIT_NONE; }
    }
}

static void show_cross(void)
{
    int16_t in = D.cross_in_flag || D.cross_in_on;
    uint16_t which = in ? ACS_CROSS_IN : ACS_CROSS_OUT;
    uint16_t first;

    if (D.cross_ack_error)      { first = letter('N'); }
    else if (acs_cross_acked(which)) { first = letter('A'); }
    else { first = digit_of((int16_t)acs_cross_retry(which), 1); }
    s_ui.digit[0] = first;
    s_ui.digit[1] = letter(in ? 'I' : 'O');
    show_number_tail(acs_cross_num(which), 0);
}

static int16_t error_code(void)
{
    if (tune_active()) { return safety_error(); }
    if (D.system_err != 0) { return D.system_err; }
    if (D.stop_err != 0)   { return D.stop_err; }
    return D.action_err;
}

/* 남은 회전각. 정렬 회전은 펄스를 따로 세므로 목표까지 남은 양으로 본다 */
static int16_t turn_left_deg(void)
{
    int32_t left = D.tape_align ? ((int32_t)D.tape_target_cnt - D.tape_turned) : (int32_t)D.turn_pulses;

    if ((left <= 0) || (P.turn_90 <= 0)) { return 0; }
    return (int16_t)((left * 90L) / (int32_t)P.turn_90);
}

static void show_idle(void)
{
    if ((run_standby_num() == 0) && seq_is_active()) {
        int16_t step = (int16_t)seq_active_step();
        show(letter('R'), letter('U'), (step > 99) ? digit_of(step, 100) : letter('N'),
             digit_of(step, 10), digit_of(step, 1));
    }
    else if (acs_station_count() > 0u) {
        s_ui.digit[0] = letter('S');
        s_ui.digit[1] = 1;
        show_number_tail(acs_station_get(0), 1);
    }
    else if (run_standby_num() > 0) {
        show(letter('S'), letter('T'), letter('B'), digit_of(run_standby_num(), 10), digit_of(run_standby_num(), 1));
    }
    else if (D.flag_turn) {
        int16_t angle = turn_left_deg();
        show(letter('T'), letter('N'), digit_of(angle, 100), digit_of(angle, 10), digit_of(angle, 1));
    }
    else if ((run_wait_left() >= 0) && !D.flag_run) {
        s_ui.digit[0] = letter('A');
        s_ui.digit[1] = letter('T');
        show_number_tail(run_wait_left(), 1);
    }
    else {
        show(letter('A'), letter('U'), letter('T'), letter('O'), DIGIT_NONE);
    }
}

static int16_t run_value(void)
{
    switch (P.display_mode) {
    case DISP_SET_SPEED: return param_get(P_SPEED_HIGH);
    case DISP_CUR_SPEED:
        return (int16_t)((int16_t)((float)D.rpm_flt[0] * P.one_rpm_meter) * 100
                       + (int16_t)((float)D.rpm_flt[1] * P.one_rpm_meter));
    case DISP_TAG_COUNT: return comm_recent_num();
    case DISP_BAT_V:     s_ui.dot |= 1u << 3; return param_get(P_BAT_DV);
    case DISP_BAT_RATE:  s_ui.dot |= 1u << 3; return param_get(P_BAT_PERCENT);
    case DISP_STATION:   return (param_get(P_INTERFACE_MODE) == 0) ? param_get(P_STATION_NO) : acs_station_get(0);
    default:             return 0;
    }
}

static void show_run(void)
{
    static const char mark[] = " SCTLVRNK";   /* 항목별 첫 글자 */
    int16_t v = run_value();

    if (v > 0) {
        s_ui.digit[0] = digit_of(v, 10000);
        s_ui.digit[1] = digit_of(v, 1000);
        show_number_tail(v, 0);
    }
    if (D.branch_start != 0) {
        show(letter('B'), letter('R'), DIGIT_UNDER, letter((D.branch_start == BRANCH_LEFT) ? 'L' : 'R'),
             digit_of(D.branch_step, 1));
    }
    else if ((s_ui.digit[0] == 0u) || (s_ui.digit[0] == DIGIT_NONE)) {
        if (P.display_mode == DISP_TAG_INFO) {
            s_ui.digit[0] = (uint16_t)(comm_recent_index() - TAG_SEG_OFFSET);
            s_ui.digit[1] = DIGIT_NONE;
            show_number_tail(comm_recent_num(), 1);
        }
        else if ((P.display_mode >= DISP_SET_SPEED) && (P.display_mode <= DISP_STOCK)) {
            s_ui.digit[0] = letter(mark[P.display_mode]);
        }
    }
}

static void show_auto(void)
{
    int16_t len;

    if (D.cross_in_flag || D.cross_in_on || D.cross_out_flag || D.cross_out_on) {
        show_cross();
    }
    else if (any_error()) {
        s_ui.digit[0] = letter('E');
        s_ui.digit[1] = letter('R');
        show_number_tail(error_code(), 0);
    }
    else if (D.flag_stopping && (D.stop_pulses > 0)) {
        len = (int16_t)((float)D.stop_pulses / P.one_length_pulse);
        s_ui.digit[0] = letter('S');
        s_ui.digit[1] = letter('L');
        show_number_tail(len, 0);
        if (s_ui.digit[2] == 0u) { s_ui.digit[2] = DIGIT_NONE; }
        s_ui.dot |= 1u << 3;
    }
    else if (D.flag_run) {
        show_run();
    }
    else {
        show_idle();
    }
}

static void show_param_value(void)
{
    const para_view_t *v = &s_view[(s_ui.para < P_COUNT) ? s_ui.para : 0];
    uint16_t n = para_digits(s_ui.para);
    int16_t x = s_ui.value;

    if (s_ui.para >= UI_PARA_LAMP_FIRST) {
        s_ui.digit[0] = letter(s_lamp_name[s_ui.para - UI_PARA_LAMP_FIRST]);
    }
    else {
        s_ui.digit[0] = letter(v->c[0]);
        s_ui.digit[1] = letter(v->c[1]);
        s_ui.digit[2] = letter(v->c[2]);
    }
    if (n == 0u) {
        s_ui.digit[3] = letter('O');
        s_ui.digit[4] = letter((x == 1) ? 'N' : 'F');
        return;
    }
    if (n >= 4u) { s_ui.digit[1] = digit_of(x, 1000); }
    if (n >= 3u) { s_ui.digit[2] = digit_of(x, 100); }
    if (n >= 2u) { s_ui.digit[3] = digit_of(x, 10); }
    s_ui.digit[4] = digit_of(x, 1);
    if (x < 0) { s_ui.digit[(n >= 3u) ? 1u : 2u] = DIGIT_MINUS; }
    if (s_ui.edit == 1) {
        s_ui.flick = (uint16_t)(1u << (5 - s_ui.place));
    }
}

static void show_param(void)
{
    if ((s_ui.screen == 0) && !s_ui.edit) {
        show(letter('P'), letter('A'), letter('R'), letter('A'), DIGIT_NONE);
    }
    else if (s_ui.msg_save) {
        show(letter('S'), letter('A'), letter('V'), letter('E'), DIGIT_NONE);
    }
    else if (s_ui.msg_init) {
        show(letter('I'), letter('N'), letter('I'), letter('T'), DIGIT_NONE);
    }
    else if (s_ui.screen == UI_GROUP_USE) {
        show(letter('U'), letter('S'), letter('E'), DIGIT_NONE, DIGIT_NONE);
    }
    else if (s_ui.screen == UI_GROUP_OTHER) {
        show(letter('O'), letter('T'), letter('H'), DIGIT_NONE, DIGIT_NONE);
    }
    else if (s_ui.screen == UI_GROUP_LAMP) {
        show(letter('L'), letter('A'), letter('M'), letter('P'), DIGIT_NONE);
    }
    else if (s_ui.screen > SCREEN_LIST) {
        show_param_value();
    }
}

/* SET + RESET 를 누르는 동안 태그 이력을 1 s 마다 넘긴다 */
static void show_history(void)
{
    if (s_ui.buffer != (KEY_SET + KEY_RESET)) {
        s_ui.hist_ticks = 0;
        s_ui.hist_n = 0;
        return;
    }
    s_ui.hist_ticks = (uint16_t)(s_ui.hist_ticks + 5u);
    if (s_ui.hist_ticks > HISTORY_TICKS) {
        s_ui.hist_ticks = 0;
        s_ui.hist_n = (uint16_t)((s_ui.hist_n + 1u) % COMM_TAG_HISTORY);
    }
    s_ui.digit[0] = s_ui.hist_n;
    s_ui.digit[1] = (uint16_t)(comm_tag_index(s_ui.hist_n) - TAG_SEG_OFFSET);
    show_number_tail(comm_tag_num(s_ui.hist_n), 0);
    s_ui.flick = 0;
}

/* 메뉴·입력·연속 시험: tune_text 의 글자를 그대로 그린다 */
static void show_tune_text(void)
{
    char c[DISP_DIGITS];
    uint16_t i, dot = tune_text(c);

    if (s_ui.msg_save) {
        show(letter('S'), letter('A'), letter('V'), letter('E'), DIGIT_NONE);
        return;
    }
    for (i = 0; i < DISP_DIGITS; i++) {
        s_ui.digit[i] = ((c[i] >= '0') && (c[i] <= '9')) ? (uint16_t)(c[i] - '0') : letter(c[i]);
    }
    s_ui.dot = dot;
    s_ui.flick = tune_flick();
}

/* 이름 한 글자 + 네 자리. 9999 를 넘으면 자르지 않고 OVER */
static void show_wide(char name, int16_t v)
{
    if (v > 9999) { show(letter(name), letter('O'), letter('V'), letter('E'), letter('R')); }
    else { show(letter(name), digit_of(v, 1000), digit_of(v, 100), digit_of(v, 10), digit_of(v, 1)); }
}

static void show_tune(void)
{
    switch (tune_state()) {
    case TUNE_WAIT:
    case TUNE_BLD:
    case TUNE_BLD_DONE:
    case TUNE_SAVED:
        show_tune_text();
        break;
    case TUNE_IDENT:    /* 회, 순서 */
        show(letter('I'), letter('D'), DIGIT_NONE, ident_pass(), ident_phase());
        break;
    case TUNE_IDENT_DONE:
        if (agv_ident.state == 2) { show(letter('I'), letter('D'), letter('O'), letter('K'), DIGIT_NONE); }
        else { show(letter('I'), letter('D'), letter('F'), (uint16_t)(agv_ident.fail / 10), (uint16_t)(agv_ident.fail % 10)); }
        break;
    case TUNE_TOP:      /* 회, 순서 */
        show(letter('T'), letter('P'), DIGIT_NONE, ident_pass(), ident_phase());
        break;
    case TUNE_ALL:      /* 단계, 회 2자리, 순서 */
        show(letter('A'), ident_stage(), digit_of((int16_t)ident_pass(), 10), digit_of((int16_t)ident_pass(), 1), ident_phase());
        break;
    case TUNE_TEMP:     /* 단계, 회 2자리, 순서 */
        show(letter('T'), ident_stage(), digit_of((int16_t)ident_pass(), 10), digit_of((int16_t)ident_pass(), 1), ident_phase());
        break;
    case TUNE_TEMP_DONE: {   /* F + 단계 + 회 2자리 + 순서 */
        int16_t f = agv_all.fail;
        if (agv_all.state == 2) { show(letter('T'), letter('M'), letter('O'), letter('K'), DIGIT_NONE); }
        else { show(letter('F'), digit_of(f, 1000), digit_of(f, 100), digit_of(f, 10), digit_of(f, 1)); }
        break;
    }
    case TUNE_TOP_DONE:
    case TUNE_ALL_DONE:
        if ((tune_page() == 0u) && (tune_state() == TUNE_ALL_DONE)) {   /* F + 단계 + 회 2자리 + 순서 */
            int16_t f = agv_all.fail;
            if (agv_all.state == 2) { show(letter('A'), letter('L'), letter('O'), letter('K'), DIGIT_NONE); }
            else { show(letter('F'), digit_of(f, 1000), digit_of(f, 100), digit_of(f, 10), digit_of(f, 1)); }
        }
        else if (tune_page() == 0u) {
            if (agv_top.state == 2) { show(letter('T'), letter('P'), letter('O'), letter('K'), DIGIT_NONE); }
            else { show(letter('T'), letter('P'), letter('F'), (uint16_t)(agv_top.fail / 10), (uint16_t)(agv_top.fail % 10)); }
        }
        else { show_wide((tune_page() == 1u) ? 'L' : 'R', agv_top.max_rpm[tune_page() - 1u]); }   /* L·R + 최고 rpm 4자리 */
        break;
    case TUNE_HS:       /* 단계, 회 2자리, 순서 */
        show(letter('H'), ident_stage(), digit_of((int16_t)ident_pass(), 10), digit_of((int16_t)ident_pass(), 1), ident_phase());
        break;
    case TUNE_HS_DONE:
        if (tune_page() == 0u) {        /* HSOK + 가장 좋은 후보 번호, 또는 F + 단계 + 회 2자리 + 순서 */
            int16_t f = agv_all.fail;
            uint16_t best = ident_hs_best();
            if (agv_all.state == 2) {
                show(letter('H'), letter('S'), letter('O'), letter('K'), (best < HS_CANDS) ? best + 1u : DIGIT_MINUS);
            }
            else { show(letter('F'), digit_of(f, 1000), digit_of(f, 100), digit_of(f, 10), digit_of(f, 1)); }
        }
        else {          /* C + 후보 번호 + 평균 편차 RMS mm(소수 한 자리). 가장 좋은 후보는 첫 자리 점 */
            int16_t v = ident_hs_rms10(tune_page() - 1u);
            show(letter('C'), tune_page(), digit_of(v, 100), digit_of(v, 10), digit_of(v, 1));
            s_ui.dot = (uint16_t)(s_ui.dot | 0x08u | ((ident_hs_best() == tune_page() - 1u) ? 0x01u : 0u));
            if (v < 0) { s_ui.digit[2] = s_ui.digit[3] = s_ui.digit[4] = DIGIT_MINUS; }
            if (v > 999) { s_ui.digit[2] = letter('O'); s_ui.digit[3] = letter('V'); s_ui.digit[4] = letter('F'); }
        }
        break;
    case TUNE_RPM:      /* 회, 단계 */
        show(letter('R'), letter('P'), DIGIT_NONE, ident_pass(), ident_phase());
        break;
    case TUNE_RPM_DONE:
        if (tune_page() == 0u) {
            if (agv_rpmt.state == 2) { show(letter('R'), letter('P'), letter('O'), letter('K'), DIGIT_NONE); }
            else { show(letter('R'), letter('P'), letter('F'), (uint16_t)(agv_rpmt.fail / 10), (uint16_t)(agv_rpmt.fail % 10)); }
        }
        else if (tune_page() == 3u) {   /* A + 앰프 추정 S·T, 모르면 - */
            show(letter('A'), DIGIT_NONE, DIGIT_NONE, DIGIT_NONE, letter((agv_rpmt.amp != 0) ? (char)agv_rpmt.amp : '-'));
        }
        else {          /* L 저속 최저 rpm, B 출발 DAC. 4자리, 없으면 ---- */
            int16_t v = (tune_page() == 1u) ? agv_rpmt.low_rpm : agv_rpmt.brk_dac;
            show_wide((tune_page() == 1u) ? 'L' : 'B', v);
            if (v <= 0) { s_ui.digit[1] = s_ui.digit[2] = s_ui.digit[3] = s_ui.digit[4] = DIGIT_MINUS; }
        }
        break;
    default:
        break;
    }
}

static void display_update(void)
{
    show(DIGIT_NONE, DIGIT_NONE, DIGIT_NONE, DIGIT_NONE, DIGIT_NONE);
    s_ui.dot = any_error() ? 1u : 0u;
    s_ui.flick = 0;
    if (D.ui_mode == UI_AUTO)     {
        show_auto();
        if (tune_active() && !any_error() && !D.flag_turn) { show_tune(); }
    }
    else if (D.ui_mode == UI_JOG) {
        show(letter('J'), letter('O'), letter('G'),
             D.flag_jog_on ? letter('O') : DIGIT_NONE, D.flag_jog_on ? letter('N') : DIGIT_NONE);
    }
    else { show_param(); }
    show_history();
}

/* ---- IO 시험 표시 ---- */

void ui_iotest_wait(void) { show(letter('T'), letter('E'), letter('S'), letter('T'), DIGIT_NONE); }

void ui_iotest_enter(void)
{
    uint16_t i;

    for (i = 0; i <= IO_TEST_OUT_LAST; i++) { s_ui.test_out[i] = 1; }
    s_ui.value = 1;
    s_ui.place = 1;
    s_ui.data_old = s_ui.data;
    D.ui_mode = UI_IO_IN;
}

void ui_iotest_run(void)
{
    key_process();
    s_ui.dot = 0;
    s_ui.flick = 0;
    if (D.ui_mode == UI_IO_IN) {
        uint16_t n = io_test_input();
        show(letter('I'), letter('N'), DIGIT_NONE, digit_of((int16_t)n, 10), digit_of((int16_t)n, 1));
    }
    else {
        show(letter('O'), letter('U'), letter('T'), digit_of(s_ui.value, 10), digit_of(s_ui.value, 1));
        s_ui.flick = (uint16_t)(1u << (5 - s_ui.place));
        if (s_ui.test_out[s_ui.value] == 0u) { s_ui.dot = 1u << 4; }
    }
    s_ui.seg_b7 = s_ui.test_out[5] ? 0u : SEG_BRAKE2;   /* 번호 5 는 BRAKE_2. 레거시가 뒤집어 낸다 */
    io_test_write(s_ui.test_out);
    if (++s_ui.pulse_cnt >= PULSE_TICKS) {
        s_ui.pulse_cnt = 0;
        hal_led_toggle();
    }
}

/* ---- 램프·부저 ---- */

static void lamp_update(void)
{
    uint16_t r = 0, g = 0, y = 0, buzz;

    if (D.ui_mode == UI_AUTO) {
        if (D.action_err == ERR_BAT_LOW) { y = 1; }
        else if ((D.system_err != 0) || (D.stop_err != 0)
                 || ((D.action_err == ERR_OBSTACLE) && !D.cross_in_on && !D.cross_out_flag)
                 || (D.action_err == ERR_PAUSE) || (D.action_err == ERR_CROSS_NO_ACK)) { r = 1; }
        else { g = 1; }
    }
    else {
        y = 1;
    }
    if (tune_active()) {
        r = (uint16_t)(safety_error() != 0);
        g = (uint16_t)!r;
        y = 0;
    }
    io_out(OUT_LAMP_R, r);
    io_out(OUT_LAMP_G, g);
    io_out(OUT_LAMP_Y, y);
    buzz = (D.system_err != 0) || (D.stop_err != 0)
        || ((D.action_err != 0) && (D.action_err != ERR_CROSS_WAIT) && (D.action_err != ERR_CROSS_NO_ACK));
    if (tune_active()) { buzz = (uint16_t)(safety_error() != 0); }
    io_out(OUT_BUZZER, (uint16_t)(buzz && s_ui.pulse_flag));
}

void ui_run(void)
{
    key_process();
    display_update();
    if (++s_ui.pulse_cnt >= PULSE_TICKS) {
        s_ui.pulse_cnt = 0;
        s_ui.pulse_flag ^= 1u;
        lamp_update();
        hal_led_toggle();
    }
}
