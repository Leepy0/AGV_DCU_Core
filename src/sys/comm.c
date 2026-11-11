/* comm.c - 프레임 큐, 수신 처리, RFID 재시도 보상. 설계: docs/04_통신_계층.md 7절 */
#include "agv/comm.h"
#include "agv/acs.h"
#include "agv/drive.h"
#include "agv/frame.h"
#include "agv/hal.h"
#include "agv/log.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/seq.h"
#include "agv/stop.h"
#include "agv/tagact.h"

#define D agv_drive
#define P param_dv

#define RX_FLASH_TICKS  30u     /* 표시 한 바퀴(10 ms) 단위, 수신 점 300 ms */
#define NO_OVERLAP_NUM  (-1)
#define OVERLAP_NONE    '0'

typedef struct {
    frame_t  f;
    uint16_t ev;
    uint32_t left, right;       /* 프레임 완성 시점 거리 */
} rx_item_t;

/* DA_Rfid_Retry: 체크섬이 깨진 첫 'T' 시점을 기억해 뒤이은 재전송에서 이동 거리를 보상한다 */
typedef struct {
    uint16_t valid, age;
    uint16_t msgid;
    int16_t  num;
    int16_t  backward;
    uint32_t left, right;
} rfid_retry_t;

static rx_item_t s_q[COMM_QUEUE_LEN];
static volatile uint16_t s_head, s_tail;
static volatile uint16_t s_flash;
static rfid_retry_t s_rfid;
static uint16_t s_recent_index = 'T';
static int16_t  s_recent_num;
static uint16_t s_overlap_index = OVERLAP_NONE;
static int16_t  s_overlap_num = NO_OVERLAP_NUM;
static int16_t  s_hist_index[COMM_TAG_HISTORY], s_hist_num[COMM_TAG_HISTORY];

int16_t  comm_tag_index(uint16_t n)  { return (n < COMM_TAG_HISTORY) ? s_hist_index[n] : 0; }
int16_t  comm_tag_num(uint16_t n)    { return (n < COMM_TAG_HISTORY) ? s_hist_num[n] : 0; }
uint16_t comm_recent_index(void)     { return s_recent_index; }
int16_t  comm_recent_num(void)       { return s_recent_num; }

void comm_tag_restore(uint16_t n, int16_t index, int16_t num)
{
    if (n < COMM_TAG_HISTORY) {
        s_hist_index[n] = index;
        s_hist_num[n] = num;
    }
}

void comm_recent_clear(void)
{
    s_recent_index = 'T';
    s_recent_num = 0;
    s_overlap_index = OVERLAP_NONE;
    s_overlap_num = NO_OVERLAP_NUM;
}

void comm_reset(void)
{
    uint16_t i;
    s_head = s_tail = 0;
    s_flash = 0;
    s_rfid.valid = 0;
    comm_recent_clear();
    for (i = 0; i < COMM_TAG_HISTORY; i++) { comm_tag_restore(i, 0, 0); }
}

uint16_t comm_rx_flash(void)
{
    if (s_flash == 0u) { return 0; }
    s_flash--;
    return 1;
}

/* SCI-A RX ISR. 파라미터 모드에서는 PC 설정 도구를 위해 가이드센서로 중계한다 */
void comm_rx_byte(uint16_t byte)
{
    frame_ev_t ev;
    uint16_t next;
    uint16_t closed = frame_stat()->closed;

    if (D.ui_mode == UI_PARAM) {
        (void)hal_sci_send(HAL_SCI_B, &byte, 1);
        return;
    }
    if (D.ui_mode >= UI_IO_IN) { return; }
    ev = frame_feed(byte);
    if (frame_stat()->closed != closed) { s_flash = RX_FLASH_TICKS; }   /* 버린 프레임도 표시한다 (원본 동작) */
    if (ev == FRAME_EV_NONE) { return; }
    next = (uint16_t)((s_head + 1u) & (COMM_QUEUE_LEN - 1u));
    if (next == s_tail) { return; }          /* 넘친 프레임은 ACS 재전송에 맡긴다 */
    s_q[s_head].f = *frame_rx();
    s_q[s_head].ev = (uint16_t)ev;
    s_q[s_head].left = run_pulses(0);
    s_q[s_head].right = run_pulses(1);
    s_head = next;
}

static int16_t digits3(uint16_t a, uint16_t b, uint16_t c)
{
    return (int16_t)((a - '0') * 100 + (b - '0') * 10 + (c - '0'));
}

/* ---- RFID 재시도 ---- */

static int16_t rfid_moving(void)
{
    return (D.ui_mode == UI_AUTO) && (D.control == RUN_DRIVE) && D.flag_run && (D.vin_out != 0)
        && !D.flag_stopping && (D.stop_pulses == 0) && (D.turn_pulses <= 0) && !D.flag_turn
        && !D.system_err && !D.estop;
}

static int16_t is_msgid(uint16_t c) { return ((c >= '1') && (c <= '9')) || ((c >= 'A') && (c <= 'F')); }
static int16_t is_digit(uint16_t c) { return (c >= '0') && (c <= '9'); }

static int16_t rfid_match(uint16_t msgid, int16_t num)
{
    return ((s_rfid.msgid != 0u) && (s_rfid.msgid == msgid))
        || ((s_rfid.num >= 0) && (s_rfid.num == num));
}

static void rfid_bad(const rx_item_t *it)
{
    const uint16_t *b = it->f.b;
    uint16_t msgid;
    int16_t num;

    if (!rfid_moving()) { return; }
    msgid = is_msgid(b[3]) ? b[3] : 0u;
    num = (is_digit(b[2]) && is_digit(b[8]) && is_digit(b[9])) ? digits3(b[2], b[8], b[9]) : (int16_t)-1;
    if ((msgid == 0u) && (num < 0)) { return; }
    /* 같은 태그의 거듭된 실패에는 첫 시점을 지킨다 */
    if (s_rfid.valid && rfid_match(msgid, num) && (s_rfid.backward == D.backward)) { return; }
    s_rfid.valid = 1;
    s_rfid.age = 0;
    s_rfid.msgid = msgid;
    s_rfid.num = num;
    s_rfid.backward = D.backward;
    s_rfid.left = it->left;
    s_rfid.right = it->right;
}

static void rfid_tick(void)
{
    if (!D.flag_stopping) { D.immediate_stop = 0; }
    if (s_rfid.valid) {
        s_rfid.age++;
        if ((s_rfid.age > RFID_RETRY_TICKS) || !rfid_moving() || (D.backward != s_rfid.backward)) {
            s_rfid.valid = 0;
        }
    }
}

/* 재시도가 맞으면 첫 실패 뒤 이동 거리, 아니면 이번 프레임 수신 뒤 이동 거리 */
static int32_t tag_distance(const rx_item_t *it, int16_t num)
{
    const uint16_t *b = it->f.b;
    uint16_t wheel = D.backward ? 1u : 0u;
    uint32_t now = run_pulses(wheel);
    uint32_t from = wheel ? it->right : it->left;

    if (s_rfid.valid && is_msgid(b[3]) && rfid_moving() && rfid_match(b[3], num)
        && (s_rfid.backward == D.backward)) {
        from = wheel ? s_rfid.right : s_rfid.left;
    }
    s_rfid.valid = 0;
    return (int32_t)(now - from);
}

/* 교차로 대기 거리에서 수신 뒤 달린 거리를 뺀다. 이미 지나쳤으면 바로 정지 유지 */
static void cross_compensate(int32_t pulses)
{
    uint16_t irq;

    if ((pulses <= 0) || (!D.cross_in_flag && !D.cross_out_flag)) { return; }
    irq = hal_irq_save();
    if (pulses >= (int32_t)D.cross_pulses) {
        D.cross_pulses = 0;
        hal_irq_restore(irq);
        acs_cross_hold();
        return;
    }
    D.cross_pulses = (int16_t)(D.cross_pulses - (int16_t)pulses);
    hal_irq_restore(irq);
}

/* ---- 'E' ---- */

static void tag_history_push(void)
{
    uint16_t i;
    for (i = COMM_TAG_HISTORY - 1u; i > 0u; i--) {
        s_hist_index[i] = s_hist_index[i - 1u];
        s_hist_num[i] = s_hist_num[i - 1u];
    }
    s_hist_index[0] = (int16_t)s_recent_index;
    s_hist_num[0] = s_recent_num;
    s_overlap_index = s_recent_index;
    s_overlap_num = s_recent_num;
}

static void speed_command(const uint16_t *b, int16_t num)
{
    int16_t speed;

    if (b[2] == '1') {
        int16_t low, up;
        if (num == 0) { return; }                       /* 1S00 은 무시(예전에는 LOS 를 하한으로 저장했다) */
        param_limit(P_SPEED_LOW, &low, &up);
        if (num < low) { num = low; }
        if (num > param_get(P_SPEED_HIGH)) { num = param_get(P_SPEED_HIGH); }
        (void)param_set(P_SPEED_LOW, num);
        speed = P.low_speed;
    }
    else if (b[2] == '0') {
        speed = param_rpm(num);
        if (speed > P.target_speed)   { speed = P.target_speed; }
        else if (speed < P.min_speed) { speed = P.min_speed; }
        if (num == 0) { speed = param_rpm(param_get(P_SPEED_LOW)); }
        P.low_speed = speed;
        D.obs_use = 1;                                  /* 원본의 비교는 항상 참이다 */
    }
    else {
        return;
    }
    D.speed_cmd = speed;
    D.step = DRIVE_CHANGE_SPEED;
}

static void on_event(const rx_item_t *it)
{
    const uint16_t *b = it->f.b;
    uint16_t kind = b[6];
    int16_t num = (kind == 'T') ? digits3(b[2], b[8], b[9]) : digits3(b[7], b[8], b[9]);

    if ((D.control == RUN_TURN) || (D.turn_pulses > 0)) {
        s_rfid.valid = 0;                               /* 회전 중 태그는 ACK 없이 무시 */
        return;
    }
    (void)acs_send_tag_ack(b[3], b[1], kind, 'W');
    s_recent_index = kind;
    s_recent_num = num;
    if ((s_overlap_index == s_recent_index) && (s_overlap_num == s_recent_num)) {
        s_rfid.valid = 0;
        return;
    }
    tag_history_push();
    if (kind == 'T') {
        int32_t d = tag_distance(it, num);
        /* 진행 중인 정지의 남은 거리는 ISR 이 이미 줄이고 있다. 새 정지만 보정한다 */
        if (tagact_run((uint16_t)num)) { tagact_compensate(d); }
        return;
    }
    /* 'S' 의 [2] 는 PAN 이 아니라 부명령이다 */
    if (kind != 'S') { acs_set_pan(b[2]); }
    switch (kind) {
    case 'I':
        {   /* 대기 거리는 'T' 정지 거리와 같은 방식으로 수신 지연·RFID 재시도를 보정한다.
             * 다른 쪽 대기가 이미 건 거리는 ISR 이 줄이고 있으므로 새로 건 경우만 뺀다 */
            int32_t d = tag_distance(it, num);
            int16_t fresh = (int16_t)(!D.cross_in_flag && !D.cross_out_flag);
            if (D.cross_in_flag || D.cross_in_on) { break; }
            acs_cross_in(num);
            if (fresh) { cross_compensate(d); }
        }
        break;
    case 'O': acs_cross_out(num); break;      /* 보고 큐에만 넣는다. 서지 않는다 */
    case 'D': acs_report_slot(ACS_SLOT_DISPLAY, num); break;
    case 'B': run_standby(num, 1); break;
    case 'C': run_standby_clear(0); break;
    case 'S': speed_command(b, num); break;
    default: break;
    }
    s_rfid.valid = 0;
}

/* ---- 'W' ---- */

static void traffic_start(int16_t cross_id)
{
    if (acs_cross_num(ACS_CROSS_IN) > 0) {
        acs_cross_release(ACS_CROSS_IN);
        if (D.step == DRIVE_COMM) { D.step = DRIVE_CHANGE_SPEED; }
        if ((cross_id == 0) && (D.flag_stopping || !D.flag_run)) { run_wait_start(); }
    }
    else if ((cross_id == 0) && (D.vin_out == 0) && !seq_is_active() && (run_standby_num() == 0)) {
        run_wait_start();
    }
}

static void traffic_reset(void)
{
    run_seq_init();
    run_reset_request();
    run_wait_start();
    D.obs_use = 0;
}

static void on_traffic(const uint16_t *b)
{
    acs_set_pan(b[2]);
    if ((b[6] == 'S') && (b[7] == 'C')) { acs_station_reset(); }
    if (b[6] != 'C') { return; }
    (void)acs_send_ack(b[3], b[1], b[6], 'P');
    switch (b[9]) {
    case '0':
        if (D.cross_in_flag) {
            D.cross_in_on = 1;
            stop_request_rapid(STOP_REASON_TRAFFIC);
        }
        else {
            stop_request_rapid(STOP_REASON_COMMAND);
        }
        D.flag_stopping = 1;
        break;
    case '1':
        traffic_start(digits3(b[10], b[11], b[12]));
        break;
    case '2':
        traffic_reset();
        break;
    case '3':
        traffic_reset();
        tagact_forback(DIR_TOGGLE);
        break;
    default:
        break;
    }
}

/* ---- 'N', 'C' ---- */

static void on_next(const uint16_t *b)
{
    uint16_t i;

    acs_set_pan(b[2]);
    for (i = 0; i < 3u; i++) {
        (void)acs_station_push(digits3(b[6u + i * 3u], b[7u + i * 3u], b[8u + i * 3u]));
    }
    acs_call_ack(b[3]);
    acs_report_slot(ACS_SLOT_NEXT, acs_station_get(0));
    run_wait_start();
    run_standby_auto_off();
    acs_slot_clear(ACS_SLOT_STANDBY);
}

static void on_command(const uint16_t *b)
{
    if (b[6] == 'S') {
        (void)acs_station_push(digits3(b[7], b[8], b[9]));
        return;
    }
    if (b[6] != 'C') { return; }
    switch (b[9]) {
    case '1':
        run_restart_now();
        run_report_start();
        break;
    case '2':
        run_standby_auto_off();
        acs_station_reset();
        run_command_stop();
        break;
    case '3':
        run_command_stop();
        break;
    case '4':
        if (P.forback_mode > 0) {
            tagact_forback(DIR_TOGGLE);
            if (P.forback_mode == 2) {
                run_wait_start();
                run_report_start();
            }
        }
        run_reset_request();
        break;
    case '5':
        if (P.forback_mode > 0) { tagact_forback(DIR_TOGGLE); }
        break;
    default:
        break;
    }
}

static void dispatch(const rx_item_t *it)
{
    const uint16_t *b = it->f.b;

    switch (b[1]) {
    case 'E': on_event(it); break;
    case 'W': on_traffic(b); break;
    case 'A':
        acs_set_pan(b[2]);
        acs_on_ack(b[7], b[3]);
        break;
    case 'N': on_next(b); break;
    case 'C': on_command(b); break;
    default: break;                     /* 'D' 는 무시 */
    }
}

void comm_poll(void)
{
    uint16_t n;

    rfid_tick();
    n = hal_sci_take_errors(HAL_SCI_A);
    if (n != 0u) { log_event(LOG_SCI_ERROR, HAL_SCI_A, (int16_t)n); }
    while (s_tail != s_head) {
        const rx_item_t *it = &s_q[s_tail];
        if (it->ev == FRAME_EV_RFID_BAD) { rfid_bad(it); }
        else {
            dispatch(it);
            s_rfid.valid = 0;           /* 유효 프레임이 끼면 재시도 후보는 끝난다 */
        }
        s_tail = (uint16_t)((s_tail + 1u) & (COMM_QUEUE_LEN - 1u));
    }
}
