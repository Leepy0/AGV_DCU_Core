/* guide_poll.c - SCI-B 응답 처리, 요청 송신, 가이드 오류 판정. 10 ms */
#include "agv/tune.h"
#include "agv/guide.h"
#include "agv/drive.h"
#include "agv/frame.h"
#include "agv/hal.h"
#include "agv/log.h"
#include "agv/param.h"
#include "agv/run.h"

#define D agv_drive
#define P param_dv

#define COMM_ERR_LIMIT 10      /* 형식 오류·'S' 연속 횟수 */

enum { PREV_UNKNOWN = -1 };

static int16_t s_prev_back = PREV_UNKNOWN;
static int16_t s_ready_cnt;
static int16_t s_err_cnt;       /* gComm_Error_Cnt */
static float   s_trk_mm;        /* 가장자리 고정·유지로 달린 거리 mm */

void guide_poll_reset(void)
{
    s_prev_back = PREV_UNKNOWN;
    s_ready_cnt = 0;
    s_err_cnt = 0;
    guide_reset();
}

void guide_poll_start(void)
{
    s_ready_cnt = 0;
    s_err_cnt = 0;
    s_trk_mm = 0.0f;
    D.guide_track = GUIDE_TRK_NORMAL;
    D.guide_all_on = D.guide_all_off = 0;
    guide_on_run_start();
}

static void ok_clear(void)
{
    s_err_cnt = 0;
    run_rail_cut_clear();
    D.guide_all_on = D.guide_all_off = 0;
}

/* 값이 나쁘면 위치 0 으로 두고 횟수·거리로 확정한다. 전부 꺼짐은 거리로만 확정한다 */
static void range_error(int16_t code, int16_t all_off)
{
    if ((s_err_cnt >= COMM_ERR_LIMIT) || (run_rail_cut() > 2L * P.tag_pulse[6])) {
        D.stop_err = code;
        s_err_cnt = 0;
        run_rail_cut_clear();
        D.guide_all_on = D.guide_all_off = 0;
    }
    if (!all_off) { s_err_cnt++; }
    D.guide_pos = 0;
}

static void apply_status(int16_t status, int16_t pos)
{
    if (status == GUIDE_LOST) {
        D.guide_all_off = 1;
        range_error(ERR_GUIDE_RANGE, 1);
    }
    else if (status == GUIDE_ALL_ON) {
        D.guide_pos = 0;
        D.guide_all_on = 1;
    }
    else {
        D.guide_pos = pos;
        ok_clear();
    }
}

static void store_led(const guide_rx_t *rx)
{
    if (rx->wide) { D.guide_led32 = guide_lateral(rx); }
    else          { D.guide_led = (uint16_t)rx->led_lo; }
}

/* 1 = guide_track 을 불러 out 을 채웠다('h' 전용 소스). 같은 응답에 다시 부르면 추적 상태가 두 번 바뀐다 */
static int on_led(const guide_rx_t *rx, guide_out_t *out)
{
    store_led(rx);
    if (guide_source_led()) {   /* 위치를 'a' 단위로 바꾼다. 배율은 튜닝 모드가 잰 h_scale(기본 1) */
        float v;
        guide_track_drive(rx, D.branch_start, D.backward, out);
        D.guide_track = out->track;
        if (out->track == GUIDE_TRK_NORMAL) { s_trk_mm = 0.0f; }
        else {      /* vin_out 은 모터 rpm 이다. rpm × (m/min)/rpm × 10 ms = mm × 6 */
            s_trk_mm += (float)((D.vin_out >= 0) ? D.vin_out : -D.vin_out) * P.one_rpm_meter / 6.0f;
            if (s_trk_mm >= (float)((out->track == GUIDE_TRK_HOLD) ? GUIDE_HOLD_LIMIT_MM : GUIDE_EDGE_LIMIT_MM)) {
                s_trk_mm = 0.0f;
                guide_track_release();
            }
        }
        v = (float)out->pos * guide_h_scale();
        D.guide_span = out->width;
        apply_status(out->status, (int16_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f)));
        return 1;
    }
    /* 'a' 소스 중 분기·테이프 구간. 패턴이 전부 꺼짐·켜짐일 때만 위치 상태를 고친다 */
    {
        int16_t v = guide_h_position(rx, D.guide_pos);
        int16_t off = rx->wide ? 33 : 17;
        int16_t on = rx->wide ? 32 : 16;
        if (v == off)     { apply_status(GUIDE_LOST, 0); }
        else if (v == on) { apply_status(GUIDE_ALL_ON, 0); }
        else              { apply_status(GUIDE_NORMAL, v); }
    }
    return 0;
}

/* 정렬 회전은 라인이 사라지는 것이 정상이다. 오류로 치지 않고 위치와 상태만 고친다 */
static void on_turn_led(const guide_rx_t *rx)
{
    guide_out_t out;

    store_led(rx);
    guide_track(rx, 0, D.backward, &out);
    D.guide_all_off = (int16_t)(out.status == GUIDE_LOST);
    D.guide_all_on = (int16_t)(out.status == GUIDE_ALL_ON);
    if (out.status == GUIDE_NORMAL) { D.guide_pos = out.pos; }
}

static void on_rx(const guide_rx_t *rx)
{
    /* 'h' 전용 소스의 'a' 는 튜닝 배율 짝에만 쓴다. 조향 위치·오류 판정·추적 상태는 'h' 그대로 */
    int a_pair_only = (rx->kind == GUIDE_RX_POSITION) && guide_source_led();
    guide_out_t out;

    D.guide_wide = rx->wide;
    if ((rx->kind != GUIDE_RX_LED) && !a_pair_only) { D.guide_track = GUIDE_TRK_NORMAL; }
    /* 정렬 회전 중 'S' 같은 응답은 오류로 세지 않는다. 라인이 없으면 회전이 258 로 끝난다 */
    if (D.tape_align && D.flag_turn) {
        if (rx->kind == GUIDE_RX_LED) { on_turn_led(rx); }
        return;
    }
    switch (rx->kind) {
    case GUIDE_RX_POSITION:
        D.guide_vert = rx->vert;
        if (!a_pair_only) { apply_status(rx->status, rx->pos); }
        if (tune_active()) { tune_on_a(rx->status, rx->pos); }
        break;
    case GUIDE_RX_LED:
        if (on_led(rx, &out)) { if (tune_active()) { tune_on_h(rx, &out); } }
        else if (tune_active()) { tune_on_h(rx, 0); }
        break;
    case GUIDE_RX_SENSOR_ERR:
        range_error(ERR_GUIDE_SENSOR, 0);
        break;
    case GUIDE_RX_BAD_FORMAT:
        range_error(ERR_GUIDE_COMM, 0);
        break;
    default:
        return;                     /* 체크섬 실패는 guide 모듈이 센다 */
    }
    /* 첫 유효 응답이 주행 시작 신호다 */
    if (D.flag_ready) {
        D.control = RUN_DRIVE;
        D.flag_run = 1;
        D.flag_ready = 0;
    }
}

static void send_request(void)
{
    uint16_t buf[4];
    uint16_t tape = (uint16_t)((D.tape_stop_id != 0) || (D.tape_turn_dir != 0) || D.tape_align);
    uint16_t branch = (uint16_t)((D.branch_start != 0) && (D.branch_step != 3));
    int h;

    /* 테이프·분기는 'H' 우선. 튜닝 측정 중에는 위치 소스와 무관하게 'A'/'H' 를 번갈아 배율 짝을 만든다 */
    if (tape || branch)                      { h = 1; }
    else if (tune_active() && tune_measuring()) { h = tune_want_h(); }
    else                                     { h = guide_want_h(D.branch_start, D.branch_step); }
    buf[0] = FRAME_STX;
    buf[1] = h ? 'H' : 'A';
    buf[2] = frame_checksum(&buf[1], 1);
    buf[3] = FRAME_ETX;
    (void)hal_sci_send(HAL_SCI_B, buf, 4);
    if (guide_on_request() != 0) { D.stop_err = ERR_GUIDE_COMM; }
}

void guide_poll(void)
{
    uint16_t byte, n;
    guide_rx_t rx;

    /* 정렬 회전은 좌우 크기가 같아 보인다. 직진으로 치지 않는다 */
    (void)guide_straight_tick(D.rpm[0], D.rpm[1], D.flag_turn ? 0x7FFF : P.min_speed);
    if (D.backward != s_prev_back) {
        if (s_prev_back != PREV_UNKNOWN) { guide_on_sensor_switch(); }
        s_prev_back = D.backward;
    }
    n = hal_sci_take_errors(HAL_SCI_B);
    if (n != 0u) { log_event(LOG_SCI_ERROR, HAL_SCI_B, (int16_t)n); }

    while (hal_sci_rx(HAL_SCI_B, &byte)) {
        if (D.ui_mode == UI_PARAM)       { (void)link_send(&byte, 1); }   /* PC 설정 도구 중계 */
        else if (guide_feed(byte, &rx))  { on_rx(&rx); }
    }
    if (guide_comm_error() != 0) { D.stop_err = ERR_GUIDE_COMM; }

    if (D.flag_ready && (D.turn_pulses == 0)) {
        s_ready_cnt++;
        if (s_ready_cnt >= RUN_READY_TIMEOUT) { D.stop_err = ERR_GUIDE_RANGE; }
    }
    if (D.flag_ready || D.flag_run || (D.flag_turn && D.tape_align)) { send_request(); }
}
