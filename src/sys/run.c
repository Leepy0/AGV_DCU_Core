/* run.c - 런 조정: 시작·정지·비상, 오류 처리, 대기 시작, 시퀀스 디스패치, 주행 시작 처리 */
#include "agv/run.h"
#include "agv/acs.h"
#include "agv/comm.h"
#include "agv/drive.h"
#include "agv/guide.h"
#include "agv/hal.h"
#include "agv/io.h"
#include "agv/ioport.h"
#include "agv/link.h"
#include "agv/motor.h"
#include "agv/param.h"
#include "agv/seq.h"
#include "agv/steerlog.h"
#include "agv/stop.h"
#include "agv/storage.h"
#include "agv/tagact.h"
#include "agv/tune.h"
#include "agv/ui.h"

#define D agv_drive
#define P param_dv

#define BAT_OVER_DV        280
#define BAT_LOW_DV         230
#define BAT_LOW_TICKS      1000
#define BAT_ADC_INIT       3200u
#define BAT_SCALE          0.0814f
#define START_LOW_DONE     2        /* Inital_start 완료 값 */
#define START_ALIGN_RANGE  3
#define START_ALIGN_TICKS  10
#define START_HOLD_TICKS   100      /* START 를 1 s 누르면 고속 */
#define AUTO_SPEED_RANGE   3
#define AUTO_CURVE_RANGE   7
#define AUTO_SLOW_MIN_MPM  25
#define AUTO_SLOW_MAX_MPM  60
#define STOP_HOLD_TICKS    300      /* STOP 3 s 길게 누름 */
#define STATUS_TICKS       6000     /* 60 s 'P' 보고 */
#define STANDBY_TICKS      6000
#define ERR_ACTION_LIMIT   280      /* 이보다 작은 action_err 는 시퀀스를 끊는다 */
#define BUMPER_RUN_VIN     30
#define WAIT_SEC_TICKS     100
#define NO_ACTION_TYPE     2        /* 이 차종군은 Auto Start 를 유지한다 */
#define EMPTY_TICKS        500      /* 서서 적재 센서가 모두 꺼진 채 5 s 면 공차 */

static volatile uint32_t s_pulses[2];
static volatile int32_t  s_rail_cut, s_curve_cnt;
static uint16_t s_forback_in = IN_IF_5;
static uint16_t s_remote_pause;

static struct {
    int16_t idle;             /* gFlag_AGV_None_Action */
    int16_t auto_start;       /* Flag_Auto_Start */
    int16_t reset_req;        /* Flag_Reset_control */
    int16_t seq_go;           /* 이번 틱에 시퀀스를 평가한다 */
    int16_t dir_change;       /* Dir_Change_Flag */
    int16_t wait_sec, wait_ticks;
    int16_t start_prev, stop_prev, fb_prev;
    int16_t stop_hold;        /* Cnt_Stop_SW_1sec_Flag */
    int16_t bumper_run, bumper_cnt;
    int16_t standby, standby_auto, standby_cnt;
    int16_t status_cnt;
    int16_t bat_low_cnt;
    uint16_t bat_filtered;
    int16_t init_step, init_cnt;      /* Inital_start, Inital_cnt */
    int16_t drive_started;            /* Flag_Driving_start */
    int16_t was_stopping, prev_len;   /* 이번 틱 정지 요청 판단 */
    int16_t empty_cnt;                /* 공차 판정 */
} s_run;

static void drive_pre(void);

void run_reset(void)
{
    s_pulses[0] = s_pulses[1] = 0;
    s_rail_cut = s_curve_cnt = 0;
    s_run.idle = 0;
    s_run.auto_start = 0;
    s_run.reset_req = 0;
    s_run.seq_go = 0;
    s_run.dir_change = 0;
    s_run.wait_sec = s_run.wait_ticks = 0;
    s_run.start_prev = s_run.stop_prev = 0;
    s_run.fb_prev = 0;
    s_run.stop_hold = 0;
    s_run.bumper_run = s_run.bumper_cnt = 0;
    s_run.standby = s_run.standby_auto = s_run.standby_cnt = 0;
    s_run.status_cnt = 0;
    s_run.bat_low_cnt = 0;
    s_run.bat_filtered = BAT_ADC_INIT;
    s_run.init_step = s_run.init_cnt = 0;
    s_run.drive_started = 0;
    s_run.empty_cnt = 0;
    if (param_profile_empty()) { param_profile(0); }
    D.ui_mode = UI_AUTO;
    D.control = RUN_READY;
    D.step = DRIVE_CHANGE_SPEED;
    D.obs_use = 1;
    D.speed_cmd = P.low_speed;
    D.tape_slip_ratio = 1.0f;
    D.drive_pre = drive_pre;
}

uint32_t run_pulses(uint16_t wheel) { return s_pulses[wheel]; }
int      run_is_idle(void)          { return s_run.idle; }
int      run_auto_started(void)     { return s_run.auto_start; }
int32_t  run_rail_cut(void)         { return s_rail_cut; }
void     run_rail_cut_clear(void)   { s_rail_cut = 0; }
void     run_reset_request(void)    { s_run.reset_req = 1; }
void     run_wait_start(void)       { D.wait_start = 1; }
void     run_wait_cancel(void)      { D.wait_start = 0; }
void     run_forback_input(uint16_t id)
{
    /* 입력이 바뀐 스캔에 거짓 에지가 생겨 릴레이가 뒤집히지 않게 한다 */
    if (id != s_forback_in) {
        s_forback_in = id;
        s_run.fb_prev = 0;
    }
}
void     run_remote_pause(uint16_t on)  { s_remote_pause = on; }
void     run_restart_now(void)      { D.run_restart = 1; }
void     run_dir_change(int16_t c)  { s_run.dir_change = c; }
int16_t  run_standby_num(void)      { return s_run.standby; }
void     run_standby_auto_off(void) { s_run.standby = 0; s_run.standby_auto = 0; }

/* 0 이면 P_AUTO_START_SEC */
int16_t run_wait_left(void) { return D.wait_start ? s_run.wait_sec : -1; }

void run_request_restart(uint16_t sec)
{
    D.wait_start = 1;
    s_run.wait_sec = (int16_t)sec;
    s_run.wait_ticks = 0;
}

/* eCAP ISR. 정지 거리는 전진 좌, 후진 우 바퀴로 잰다 */
void run_encoder(uint16_t wheel, uint32_t period)
{
    motor_encoder_pulse(wheel);
    D.period[wheel] = (int32_t)period;
    s_pulses[wheel]++;
    s_rail_cut++;
    s_curve_cnt++;
    if (wheel == 1u) { D.straight_cnt++; }
    if ((D.branch_ready > 0) || (D.branch_start > 0)) {
        if (wheel == 0u) { D.cnt_branch_l++; }
        else             { D.cnt_branch_r++; }
    }
    if ((D.backward != 0) == (wheel == 1u)) {
        if (D.stop_pulses > 0) { D.stop_pulses--; }
        if (D.cross_pulses > 0) { D.cross_pulses--; }
        D.travel_pulses++;
    }
    if (D.flag_turn && (D.turn_pulses > 0)) {
        if (D.tape_align) {                       /* 끝은 가이드가 정한다. turn_pulses 는 회전 중 표시로 둔다 */
            if (D.tape_step >= TAPE_STEP_SETTLE) { D.tape_fix_turned++; }   /* 중앙에서 선 뒤는 회전 보정비 측정에 넣지 않는다 */
            else                                 { D.tape_turned++; }
        }
        else              { D.turn_pulses--; }
    }
}

/* ---- ACS 보고 ---- */

static void standby_forget(void)
{
    s_run.standby = 0;
    s_run.standby_auto = 0;
    s_run.status_cnt = 0;
    acs_slot_clear(ACS_SLOT_STANDBY);
}

void run_report_start(void)
{
    standby_forget();
    acs_report_slot(ACS_SLOT_START, 0);
}

void run_report_stop(void)
{
    standby_forget();
    acs_report_slot(ACS_SLOT_STOP, 0);
}

void run_standby(int16_t id, int16_t reset)
{
    D.wait_start = 0;
    if (reset) { acs_station_reset(); }
    s_run.standby = id;
    if ((id > 0) && (D.turn_pulses == 0) && (D.turn_motion == 0)) {
        stop_request_rapid(STOP_REASON_COMMAND);
        D.flag_stopping = 1;
        s_run.standby_auto = 1;
    }
    s_run.status_cnt = 0;
    acs_report_slot(ACS_SLOT_STANDBY, id);
}

void run_standby_clear(int16_t id)
{
    s_run.standby = 0;
    s_run.standby_auto = 0;
    s_run.status_cnt = 0;
    acs_report_slot(ACS_SLOT_STANDBY_CLEAR, id);
}

/* ---- 초기화 ---- */

void run_seq_init(void)
{
    seq_abort_all();
    comm_recent_clear();
    acs_cross_clear();
    D.tape_stop_id = 0;
    D.tape_turn_dir = 0;
    D.tape_turn_deg = 0;
    D.tape_turn_next = 0;
    D.stopping_process = 0;
}

static void tape_clear(void)
{
    D.tape_align = 0;
    D.tape_step = 0;
    D.tape_max = 0;
    D.tape_total_pulse = 0;
    D.tape_slip_ratio = 1.0f;
    D.tape_target_cnt = 0.0f;
    D.tape_end_cnt = 0.0f;
    D.tape_sum_mt = D.tape_sum_tt = 0.0f;
    D.tape_plan_min = 0;
    D.tape_lost_cnt = 0;
    D.tape_turned = 0;
}

/* Initial_Start */
static void start_run(void)
{
    D.flag_ready = 1;
#if AGV_START_ALIGN
    D.start_align = (int16_t)(DRIVE_ALIGN_HOLD + (tune_active() ? 1 : 0));   /* 튜닝은 속도를 스스로 정한다: 조향 상한만 */
#endif
    s_run.init_step = 0;
    s_run.init_cnt = 0;
    s_run.drive_started = 0;
    D.drive_start_cnt = 0;
    D.turn_pulses = 0;
    D.branch_return_cnt = 0;
    D.control = RUN_READY;
    D.wait_start = 0;
    D.run_restart = 0;
    D.turn_motion = 0;
    D.p_cmd = 0;
    D.p_test = 0;
    D.straight_cnt = 0;
    s_rail_cut = 0;
    s_curve_cnt = 0;
    D.pause_stop = 0;
    D.cross_in_on = D.cross_out_on = 0;
    s_run.standby_auto = 0;
    s_run.standby_cnt = 0;
    run_seq_init();
    s_run.wait_sec = 0;
    s_run.wait_ticks = 0;
    link_guide_init();
    guide_poll_start();
}

/* STOP 스위치. 누르고 있는 동안 매 틱 들어온다 */
static void stop_switch(void)
{
    stop_request_rapid(STOP_REASON_SWITCH);
    s_run.auto_start = 0;
    D.flag_stopping = 1;
    D.stop_pulses = 0;
    D.wait_start = 0;
    D.run_restart = 0;
    D.turn_motion = 0;
    D.turn_pulses = 0;
    s_run.reset_req = 1;
    D.drive_start_cnt = 0;
    s_run.bumper_run = 0;
    s_run.bumper_cnt = 0;
    D.pause_stop = 0;
    D.cross_in_on = D.cross_out_on = 0;
    run_seq_init();
    act_abort_all();
    D.ext_sen_on = 0;
    s_run.standby = 0;
    s_run.standby_auto = 0;
    s_run.standby_cnt = 0;
    tape_clear();
    P.dcc = param_active(P_DCC);
}

void run_command_stop(void)
{
    s_run.auto_start = 0;
    stop_request_rapid(STOP_REASON_COMMAND);
    D.flag_stopping = 1;
    D.stop_pulses = 0;
    D.wait_start = 0;
    D.run_restart = 0;
    D.turn_motion = 0;
    run_seq_init();
    act_abort_all();
    run_report_stop();
    s_run.reset_req = 1;
}

/* ---- 릴레이 ---- */

void run_relay(int16_t backward, int16_t save)
{
    if ((backward != 0) == (D.backward != 0)) { return; }
    io_out(OUT_AGV_BACK, (uint16_t)(backward != 0));
    D.backward = (int16_t)(backward != 0);
    if (save) { keep_set(KEEP_BACKWARD, D.backward); }
}

void run_boot_restore(void)
{
    uint16_t i;
    int16_t cnt = keep_get(KEEP_STATION_CNT);
    int16_t pan = keep_get(KEEP_PAN);

    /* 호기별 상태는 빈 EEPROM(0xFFFF)이면 쓰지 않는다 */
    if (keep_get(KEEP_BACKWARD) == 1) { run_relay(1, 0); }
    if ((cnt >= 0) && (cnt <= KEEP_STATION_SLOTS)) {
        for (i = 0; i < KEEP_STATION_SLOTS; i++) {
            acs_station_set(i, keep_get((uint16_t)(KEEP_STATION0 + i)), (uint16_t)cnt);
        }
    }
    if ((pan >= 1) && (pan <= 'F' - '0')) { acs_set_pan((uint16_t)(pan + '0')); }
    for (i = 0; i < COMM_TAG_HISTORY; i++) {
        comm_tag_restore(i, keep_get((uint16_t)(KEEP_TAG_INDEX0 + i)), keep_get((uint16_t)(KEEP_TAG_NUM0 + i)));
    }
    for (i = 0; i < UI_LAMP_EXT_COUNT; i++) {
        int16_t v = keep_get((uint16_t)(KEEP_LAMP_EXT0 + i));
        ui_lamp_ext_set(i, (v < 0) ? 0 : v);
    }
}

static void keep_sync(void)
{
    uint16_t i;

    for (i = 0; i < KEEP_STATION_SLOTS; i++) {
        keep_set((uint16_t)(KEEP_STATION0 + i), acs_station_get(i));
    }
    keep_set(KEEP_STATION_CNT, (int16_t)acs_station_count());
    keep_set(KEEP_PAN, (int16_t)(acs_pan() - '0'));
    for (i = 0; i < COMM_TAG_HISTORY; i++) {
        keep_set((uint16_t)(KEEP_TAG_INDEX0 + i), comm_tag_index(i));
        keep_set((uint16_t)(KEEP_TAG_NUM0 + i), comm_tag_num(i));
    }
    for (i = 0; i < UI_LAMP_EXT_COUNT; i++) {
        keep_set((uint16_t)(KEEP_LAMP_EXT0 + i), ui_lamp_ext(i));
    }
    keep_flush_if_changed();
}

/* ---- 배터리 ---- */

static int16_t charge_rate(int16_t dv)
{
    static const int16_t table[][2] = {
        {250, 999}, {249, 950}, {248, 900}, {247, 850}, {246, 800}, {245, 750}, {244, 700}, {243, 650},
        {242, 600}, {240, 550}, {239, 520}, {238, 500}, {237, 450}, {236, 400}, {235, 350}, {234, 300},
        {232, 250}, {230, 200}, {226, 100}
    };
    uint16_t i;
    for (i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
        if (dv >= table[i][0]) { return table[i][1]; }
    }
    return 0;
}

static void battery_check(void)
{
    uint16_t raw = (uint16_t)(hal_adc_battery() >> 4);
    int16_t dv;

    s_run.bat_filtered = (uint16_t)((float)s_run.bat_filtered * 0.6f + (float)raw * 0.4f);
    dv = (int16_t)((int16_t)((float)s_run.bat_filtered * BAT_SCALE) + param_digit(param_get(P_BAT_OFFSET_DV), 1));
    (void)param_set(P_BAT_DV, dv);
    (void)param_set(P_BAT_PERCENT, charge_rate(dv));
    if (dv > BAT_OVER_DV) { D.stop_err = ERR_BAT_OVER; }
    if (dv < BAT_LOW_DV) {
        if (++s_run.bat_low_cnt > BAT_LOW_TICKS) {
            D.action_err = ERR_BAT_LOW;
            s_run.bat_low_cnt = BAT_LOW_TICKS;
        }
    }
    else {
        s_run.bat_low_cnt = 0;
    }
}

/* ---- Auto_Run_Process ---- */

static void wait_tick(void)
{
    if (s_run.wait_sec <= 0) {
        D.wait_start = 0;
        D.run_restart = 1;
    }
    else if (s_run.wait_ticks >= WAIT_SEC_TICKS) {
        s_run.wait_sec--;
        s_run.wait_ticks = 0;
    }
    else {
        s_run.wait_ticks++;
    }
}

static int16_t seq_requested(void)
{
    uint16_t id;
    if (seq_is_active()) { return 1; }
    for (id = SEQ_NONE + 1; id < SEQ_MAX; id++) {
        if (seq_request_flag[id]) { return 1; }
    }
    return 0;
}

static void request_tag_if_new(int16_t was_stopping, int16_t prev_len)
{
    if (D.flag_stopping && (!was_stopping || (D.stop_pulses != prev_len))
        && !(D.flag_turn && (D.control == RUN_TURN))) {
        stop_request_tag();
    }
}

static void auto_run(void)
{
    int16_t was_stopping = D.flag_stopping;
    int16_t prev_len = D.stop_pulses;

    s_run.idle = 0;
    s_run.seq_go = 0;
    if (D.stop_err == ERR_STOP_APPROACH) { return; }      /* 실패한 접근은 리셋이 필요하다 */
    if ((D.ui_mode == UI_AUTO) && !D.flag_ready && !D.flag_run && !D.flag_stopping) {
        if (s_run.dir_change != 0) {
            run_relay((int16_t)(s_run.dir_change != 1), 1);
            s_run.dir_change = 0;
        }
        else if (D.turn_motion != 0) {
            s_run.seq_go = (int16_t)seq_is_active();
            if (D.turn_pulses > 0) {
                if (D.tape_align && !D.flag_turn) {
                    guide_poll_start();
                    D.guide_all_off = 1;     /* 첫 응답 전에는 라인이 있다고 보지 않는다 */
                    if (!param_quirk(QUIRK_TURN_START)) { D.vin_out = stop_min_rpm(); }
                }
                D.flag_turn = 1;
                D.control = RUN_TURN;
            }
            else {
                D.turn_motion = 0;
                D.flag_stopping = 1;
                D.turn_pulses = 0;
            }
        }
        else if (D.wait_start) {
            wait_tick();
        }
        else if (seq_requested()) {
            s_run.seq_go = 1;
        }
        else {
            if (P.agv_type_mode != NO_ACTION_TYPE) { s_run.auto_start = 0; }
            s_run.idle = 1;
        }
    }
    if (s_run.idle) { keep_sync(); }
    request_tag_if_new(was_stopping, prev_len);
}

/* ---- 입력 처리 ---- */

static void bumper_check(void)
{
    int16_t for_on = (int16_t)io_in(IN_BUMPER_1);
    int16_t back_on = (int16_t)io_in(IN_BUMPER_2);
    int16_t keep = param_get(P_BUMPER_USE);

    if ((D.vin_out > 0) && (((P.for_bumper >= 1) && for_on) || ((P.back_bumper >= 1) && back_on))) {
        D.estop = 1;
        D.system_err = ERR_BUMPER;
        if ((P.for_bumper >= 2) || (P.back_bumper >= 2)) {
            if (D.flag_run && !D.flag_stopping && (D.vin_out > BUMPER_RUN_VIN)) {
                s_run.bumper_run = 1;
                s_run.bumper_cnt = (int16_t)(keep * ((P.for_bumper >= 2) ? 10 : 100));
            }
        }
    }
    /* 범퍼가 풀리고 정지가 끝났으면 대기 뒤 스스로 재출발 */
    if ((D.system_err == ERR_BUMPER) && s_run.bumper_run && s_run.idle
        && ((P.for_bumper == 0) || !for_on) && ((P.back_bumper == 0) || !back_on)) {
        s_run.bumper_cnt--;
        if (s_run.bumper_cnt <= 0) {
            s_run.bumper_run = 0;
            D.wait_start = 1;
            D.estop = 0;
            D.obs_use = 1;
            D.system_err = 0;
        }
    }
}

static void stop_hold_check(int16_t stop_on)
{
    if (!stop_on || (D.ui_mode != UI_AUTO)) {
        s_run.stop_hold = 0;
        return;
    }
    if (++s_run.stop_hold >= STOP_HOLD_TICKS) {
        if (acs_station_get(0) > 0) { acs_report_slot(ACS_SLOT_LOADING, acs_station_get(0)); }
        s_run.stop_hold = 0;
        s_run.standby = 0;
        acs_station_reset();
    }
}

static void manual_forback(int16_t start_edge)
{
    int16_t fb = (int16_t)io_in(s_forback_in);
    /* 저활성 입력의 상승 에지 = 논리 하강. 놓는 순간에 동작한다 */
    int16_t fb_edge = (int16_t)(s_run.fb_prev && !fb);

    s_run.fb_prev = fb;
    if (s_run.auto_start || D.flag_run) { return; }
    if ((P.forback_mode > 0) && (((D.ui_mode == UI_PARAM) && start_edge) || fb_edge)) {
        run_relay((int16_t)!D.backward, 1);
    }
}

static void input_filter(void)
{
    int16_t start_on = (int16_t)io_in(IN_START_SW);
    int16_t stop_on = (int16_t)io_in(IN_STOP_SW);
    /* 원본은 원시 값의 상승 에지다. 스위치가 저활성이라 "놓는 순간"이다 */
    int16_t start_edge = (int16_t)(s_run.start_prev && !start_on);
    int16_t stop_edge = (int16_t)(s_run.stop_prev && !stop_on);
    int16_t report = (param_get(P_ACS_VERSION) == 2) || (param_get(P_ACS_VERSION) == 11);

    s_run.start_prev = start_on;
    s_run.stop_prev = stop_on;

    if (s_remote_pause) {
        if (io_in(IN_REMOTE_START)) { D.pause_stop = 0; }
        if (io_in(IN_REMOTE_STOP))  { D.pause_stop = 1; }
    }

    if (!stop_on && start_edge && report) { run_report_start(); }
    if ((!stop_on && start_edge) || D.run_restart) {
        if (s_run.idle) {
            s_run.auto_start = 1;
            start_run();
        }
    }
    D.stop_sw = stop_on;
    if (stop_on) { stop_switch(); }
    if (report && stop_edge) { run_report_stop(); }

    /* E-STOP·범퍼는 리셋보다 뒤에 있어 누르고 있는 동안 해제되지 않는다 */
    if (io_in(IN_ESTOP)) {
        D.estop = 1;
        D.system_err = ERR_ESTOP;
        run_seq_init();
        act_abort_all();
    }
    bumper_check();
    stop_hold_check(stop_on);
    manual_forback(start_edge);
}

/* ---- error_check 앞부분: motor_run 전에 끝낸다 ---- */

static void reset_and_alarm(void)
{
    D.amp_reset = 0;
    if (s_run.reset_req) {
        guide_comm_clear();
        if ((D.system_err == ERR_AMP_L_OVERLOAD) || (D.system_err == ERR_AMP_R_OVERLOAD)) { D.amp_reset = 1; }
        s_run.reset_req = 0;
        D.system_err = D.stop_err = D.action_err = 0;
        D.estop = 0;
    }
    if (io_in(IN_AMP_L_OVERLOAD)) { D.system_err = ERR_AMP_L_OVERLOAD; }
    if (io_in(IN_AMP_R_OVERLOAD)) { D.system_err = ERR_AMP_R_OVERLOAD; }
}

static void apply_errors(void)
{
    if (D.system_err != 0) {
        D.amp_run = 0;
        D.wait_start = 0;
        D.run_restart = 0;
    }
    if (D.stop_err != 0) {
        D.flag_stopping = 1;
        if (D.stop_err != ERR_STOP_APPROACH) { D.stop_pulses = 0; }
    }
    if (!D.cross_in_flag && !D.cross_in_on && !D.cross_out_flag && !D.cross_out_on
        && (D.step == DRIVE_COMM)) {
        D.step = DRIVE_CHANGE_SPEED;
    }
}

/* ---- 조그 (jog_process) ---- */

static int16_t square_acc(int16_t target)
{
    int16_t v = D.vin_out;
    int16_t acc = (v > P.llow_speed) ? P.acc : ((P.acc < 5) ? P.acc : 5);
    int16_t dcc = (v > 2 * P.min_speed) ? P.dcc : ((P.dcc < 60) ? P.dcc : 60);

    if ((target - v) >= 0) {
        v = (int16_t)(v + ((v >= 0) ? acc : dcc));
        if (target <= v) { v = target; }
    }
    else {
        v = (int16_t)(v - ((v >= 0) ? dcc : acc));
        if (target >= v) { v = target; }
    }
    if ((target == 0) && (P.min_speed > v)) { v = 0; }
    D.vin_out = v;
    return v;
}

static int16_t limit_add(int16_t value, int16_t delta, int16_t lower, int16_t upper)
{
    value = (int16_t)(value + delta);
    if (value > upper) { return upper; }
    if (value < lower) { return lower; }
    return value;
}

static int16_t s_key_l, s_key_r;

static void jog_curve(int16_t *add, int16_t *other, int16_t *key, int16_t dir)
{
    if (D.jog_left_only || D.jog_right_only) { return; }
    D.amp_dir = dir;
    *key = 1;
    *add = limit_add(*add, -5, (int16_t)-P.jog_speed, 0);
    *other = 0;
    D.jog_rpm = square_acc(P.jog_speed);
}

static void jog_step(void)
{
    uint16_t bits = io_jog_bits();

    D.jog_bits = (int16_t)bits;
    if ((bits == 0xFu) || (bits == 0u) || D.jog_stopping) {
        D.jog_stopping = 1;
        D.add_speed_l = D.add_speed_r = 0;
        s_key_l = s_key_r = 0;
        D.jog_rpm = square_acc(0);
        if (D.jog_rpm <= 10) {
            D.jog_left_only = D.jog_right_only = 0;
            D.jog_stopping = 0;
            D.w_pi_out[0] = D.w_pi_out[1] = 0.0f;
            D.w_err[0] = D.w_err[1] = 0;
        }
        return;
    }
    switch (bits) {
    case 0xEu:
        D.amp_dir = MOTION_FWD;
        D.jog_rpm = square_acc(P.jog_speed);
        break;
    case 0xDu:
        D.amp_dir = MOTION_BACK;
        D.jog_rpm = square_acc(P.jog_speed);
        break;
    case 0xBu:
        if (!s_key_l && !s_key_r) {
            D.amp_dir = MOTION_FWD;
            D.jog_left_only = 1;
            D.add_speed_l = 0;
            D.add_speed_r = limit_add(D.add_speed_r, 5, 0, 400);
            D.jog_rpm = square_acc(D.add_speed_r);
        }
        break;
    case 0x7u:
        if (!s_key_l && !s_key_r) {
            D.amp_dir = MOTION_FWD;
            D.jog_right_only = 1;
            D.add_speed_l = limit_add(D.add_speed_l, 5, 0, 400);
            D.add_speed_r = 0;
            D.jog_rpm = square_acc(D.add_speed_l);
        }
        break;
    case 0xAu:
        jog_curve(&D.add_speed_l, &D.add_speed_r, &s_key_l, MOTION_FWD);
        break;
    case 0x9u:
        jog_curve(&D.add_speed_l, &D.add_speed_r, &s_key_l, MOTION_BACK);
        break;
    case 0x6u:
        jog_curve(&D.add_speed_r, &D.add_speed_l, &s_key_r, MOTION_FWD);
        break;
    case 0x5u:
        jog_curve(&D.add_speed_r, &D.add_speed_l, &s_key_r, MOTION_BACK);
        break;
    case 0x3u:
        if (!s_key_l && !s_key_r) {
            D.amp_dir = D.jog_left_only ? MOTION_CCW : MOTION_CW;
            D.add_speed_l = limit_add(D.add_speed_l, 5, 0, 200);
            D.add_speed_r = limit_add(D.add_speed_r, 5, 0, 200);
            D.jog_rpm = square_acc(D.add_speed_r);
        }
        break;
    default:
        D.jog_rpm = 0;
        break;
    }
}

/* ---- 주행 시작 처리 (motor_drive_step 앞) ---- */

static int16_t guide_within(int16_t range)
{
    return (D.guide_pos <= range) && (D.guide_pos >= (int16_t)-range);
}

#if AGV_START_ALIGN
#define ALIGN_SEEN() (!D.guide_all_off)    /* 출발 정렬은 선이 보여야 끝난다 */
#else
#define ALIGN_SEEN() 1
#endif

static void start_low_speed(void)
{
    int16_t tag_clear = D.backward ? !D.tag_r : !D.tag_l;

#if AGV_START_ALIGN
    if ((D.start_align > 0) && (D.start_align <= DRIVE_ALIGN_RAMP)) { D.start_align--; }   /* 정렬 뒤 조향 상한 풀기 */
#endif
    if (s_run.init_step >= START_LOW_DONE) { return; }
    if (tag_clear) { s_run.init_step = 1; }
    if (s_run.init_step == 1) {
        if (guide_within(START_ALIGN_RANGE) && ALIGN_SEEN()) {
            if (++s_run.init_cnt > START_ALIGN_TICKS) {
#if AGV_START_ALIGN
                D.start_align = DRIVE_ALIGN_RAMP;
#endif
                D.speed_cmd = P.low_speed;
                D.step = DRIVE_CHANGE_SPEED;
                s_run.init_step = START_LOW_DONE;
            }
        }
        else {
            s_run.init_cnt = 0;
        }
    }
}

static void start_switch_high(void)
{
    if (s_run.drive_started || (s_run.init_step != START_LOW_DONE)) { return; }
    if (io_in(IN_START_SW)) { D.drive_start_cnt++; }
    else                    { D.drive_start_cnt = 0; }
    if (D.drive_start_cnt >= START_HOLD_TICKS) {
        D.speed_cmd = P.target_speed;
        D.step = DRIVE_CHANGE_SPEED;
        s_run.drive_started = 1;
    }
    if (D.tag_l || D.tag_r) { s_run.drive_started = 1; }
}

static void auto_speed_change(void)
{
    int16_t set_speed = param_get(P_SPEED_HIGH);

    if (D.flag_stopping) { return; }
    if (P.auto_drive_mode && (P.low_speed >= D.speed_cmd) && guide_within(AUTO_SPEED_RANGE)) {
        if (P.straight_len <= D.straight_cnt) {
#if AGV_DEV_TESTS
            if (D.speed_cmd != P.target_speed) { steerlog_event(STEERLOG_EV_DRS); }
#endif
            D.speed_cmd = P.target_speed;
            D.step = DRIVE_CHANGE_SPEED;
        }
    }
    else {
        D.straight_cnt = 0;
    }
    if ((set_speed >= AUTO_SLOW_MIN_MPM) && (set_speed <= AUTO_SLOW_MAX_MPM) && P.auto_slow_mode
        && (P.low_speed < D.speed_cmd)) {
        if (!guide_within(AUTO_CURVE_RANGE)) {
            if ((int32_t)P.tag_pulse[2] <= s_curve_cnt) {
#if AGV_DEV_TESTS
                steerlog_event(STEERLOG_EV_LOS);   /* 조건이 low_speed < speed_cmd 라 늘 낮춘다 */
#endif
                D.speed_cmd = P.low_speed;
                D.step = DRIVE_CHANGE_SPEED;
            }
        }
        else {
            s_curve_cnt = 0;
        }
    }
}

static void drive_pre(void)
{
    start_low_speed();
    if (tune_active()) { return; }      /* 튜닝·식별 시험은 속도를 스스로 정한다 */
    start_switch_high();
    if (D.obs_use) { auto_speed_change(); }
}

/* ---- 테이프 찾아 정지 (DA_Tape_Stop) ---- */

static uint16_t led_total(void)
{
    uint16_t i, n = 0;
    uint32_t v = D.guide_wide ? D.guide_led32 : (uint32_t)D.guide_led;
    for (i = 0; i < 16u; i++) { n = (uint16_t)(n + (uint16_t)((v >> i) & 1UL)); }
    return n;
}

/* 레거시 DA_Tape_Turn·DA_Tape_Stop 과 같다: LED1~16(wide 는 좌우 32 중 앞 16) 합이 6 이상인 응답 두 번.
 * 10 으로 올렸을 때 wide 센서가 교차 테이프를 못 보고 지나쳤다(5호기) */
#define TAPE_LED_MIN 6
#define TAPE_ID_TURN (-1)   /* 시퀀스 번호와 겹치지 않는 회전 탐색 표시 */

static struct { int16_t prev_id; uint32_t start_len; int16_t hits; } s_tape;

/* 정렬 회전 앞 교차 테이프 찾기 속도. 센서가 회전 중심보다 GD 앞이라 테이프를 본 뒤 GD 안에 서면 된다.
 * 인정(LED 6개 이상 2응답) 지연 t 동안 v·t 를 가고, 감속도 k·DCC(1 = 100 rpm/s)로 GD - 여유 - v·t 안에 선다:
 * v² = 2·k·a·(d0 - v·t) → v = √((k·a·t)² + 2·k·a·d0) - k·a·t. SPN 과 min(LOS, 12 m/min) 사이로 묶는다.
 * 모의(test_curve tape, DCC 20·50, GD 250·400, 바퀴 지연 50·150 ms): k 0.5 는 회전 중심 정지 위치가 SPN 4 보다
 * 최대 17 mm 더 나갔다. k 0.15 + 12 m/min 상한에서 +5 mm 안. 테이프 정지(바로 섬)는 속도만큼 더 가서 SPN 그대로 */
#ifndef SEEK_DCC_SHARE
#define SEEK_DCC_SHARE 0.15f    /* S커브 저크 풀림·앰프 응답·정지 제어 여유 */
#endif
#ifndef SEEK_MAX_MPM
#define SEEK_MAX_MPM   12
#endif
#define SEEK_MARGIN_MM 50.0f
float sqrtf(float x);
#define SEEK_ACK_S     0.02f
int16_t run_seek_rpm(void)
{
    float a = SEEK_DCC_SHARE * (float)P.dcc * 100.0f / P.one_meter_rpm * (1000.0f / 60.0f);   /* mm/s² */
    float d0 = P.guide_dist_mm - SEEK_MARGIN_MM, at = a * SEEK_ACK_S;
    float r = (d0 > 0.0f) ? (sqrtf((at * at) + (2.0f * a * d0)) - at) * (0.06f * P.one_meter_rpm) : 0.0f;   /* mm/s → rpm */
    if (r > (float)P.low_speed) { r = (float)P.low_speed; }
    if (r > (float)SEEK_MAX_MPM * P.one_meter_rpm) { r = (float)SEEK_MAX_MPM * P.one_meter_rpm; }
    return (r > (float)P.min_speed) ? (int16_t)r : P.min_speed;
}

/* 교차 테이프를 찾는다. 정렬 회전 앞은 찾기 속도(run_seek_rpm), 테이프 정지는 최저 속도(SPN). 1 찾음, -1 정지 거리 2배를 넘김, 0 계속 */
static int16_t tape_seek(int16_t id)
{
    if (id != s_tape.prev_id) {
        s_tape.prev_id = id;
        s_tape.start_len = s_pulses[0];
    }
    D.speed_cmd = (id == TAPE_ID_TURN) ? run_seek_rpm() : P.min_speed;
    D.step = DRIVE_CHANGE_SPEED;
    if (led_total() >= TAPE_LED_MIN) {
        if (++s_tape.hits > 1) {
            s_tape.hits = 0;
            return 1;
        }
    }
    else if ((int32_t)(s_pulses[0] - s_tape.start_len) > 2L * P.stop_len) {
        D.system_err = ERR_GUIDE_SENSOR;
        return -1;
    }
    return 0;
}

static void tape_stop(void)
{
    int16_t id = D.tape_stop_id;

    if (id != 0) {
        int16_t r = tape_seek(id);
        if (r == 1) {
            D.flag_stopping = 1;
            seq_request((uint16_t)id);
            D.stop_pulses = 0;
            stop_request_tag();
        }
        if (r != 0) { D.tape_stop_id = 0; }
    }
    else if (D.tape_turn_dir != 0) {
        int16_t r = tape_seek(TAPE_ID_TURN);
        if (r == 1) {
            tagact_turn_align_cross((turn_dir_t)D.tape_turn_dir, (uint16_t)D.tape_turn_deg, D.tape_turn_next);
        }
        if (r != 0) { D.tape_turn_dir = 0; }
    }
    else {
        s_tape.prev_id = 0;
        s_tape.hits = 0;
    }
}

/* ---- 적재·공차 ---- */

/* 켤 때와 모르면 적재. 센서 하나라도 켜지면 바로 적재, 서서 모두 꺼진 채 5 s 면 공차. 달리는 중에는 공차로 가지 않는다.
 * 달리는 중 적재로 바뀌면 지령을 적재 최고 속도로 낮춰 적재 DCC 로 줄인다 */
static void load_tick(void)
{
    uint32_t mask = seq_load_mask();
    int16_t site = seq_load_site();      /* 현장 판정이 있으면 그것, 없으면 센서 묶음, 둘 다 없으면 적재 */
    int16_t sensed = (site >= 0) ? (int16_t)(site != 0) : ((mask == 0UL) || ((io_in_mask() & mask) != 0UL));
    int16_t loaded = (param_get(P_LOAD_DETECT) == 0) || tune_active() || sensed;
    int16_t stopped = !D.flag_run || (D.vin_out == 0);

    if (loaded || !stopped) { s_run.empty_cnt = 0; }
    else if (s_run.empty_cnt < EMPTY_TICKS) { s_run.empty_cnt++; }
    if (loaded && param_profile_empty()) {
        param_profile(0);
        if (D.speed_cmd > P.target_speed) {
            D.speed_cmd = P.target_speed;
            if (D.step == DRIVE_CRUISE) { D.step = DRIVE_CHANGE_SPEED; }
        }
    }
    else if (!loaded && !param_profile_empty() && (s_run.empty_cnt >= EMPTY_TICKS)) {
        param_profile(1);
    }
}

/* ---- 공개 10 ms ---- */

void run_coordinate(void)
{
    D.tag_l = (int16_t)io_in(IN_TAG_L);
    D.tag_r = (int16_t)io_in(IN_TAG_R);
    D.obs_f = (int16_t)io_in(IN_OBS_F);
    D.obs_b = (int16_t)io_in(IN_OBS_B);

    reset_and_alarm();
    battery_check();
    tape_stop();
    auto_run();
    input_filter();
    apply_errors();
    if (D.flag_jog_on && (D.control == RUN_JOG)) { jog_step(); }
    load_tick();
    s_run.was_stopping = D.flag_stopping;
    s_run.prev_len = D.stop_pulses;
}

void run_seq(void)
{
    if (s_run.seq_go) { seq_run(); }
    act_run();
    request_tag_if_new(s_run.was_stopping, s_run.prev_len);
}

void run_after_motor(void)
{
    if ((D.system_err != 0) || (D.stop_err != 0) || ((D.action_err != 0) && (D.action_err < ERR_ACTION_LIMIT))) {
        run_seq_init();
    }
    if ((D.action_err == ERR_OBSTACLE) && (D.speed_cmd == P.target_speed)) {
        D.speed_cmd = P.low_speed;
        D.step = DRIVE_CHANGE_SPEED;
    }
    acs_error_tick();
    if (s_run.standby_auto) {
        if (++s_run.standby_cnt > STANDBY_TICKS) {
            acs_report_slot(ACS_SLOT_STANDBY, s_run.standby);
            s_run.standby_cnt = 0;
        }
    }
    else {
        s_run.standby_cnt = 0;
    }
    if (s_run.status_cnt <= STATUS_TICKS) { s_run.status_cnt++; }
    if ((s_run.status_cnt > STATUS_TICKS) && ((D.system_err != 0) || (D.stop_err != 0))) {
        acs_report_slot(ACS_SLOT_PROCESSING, 0);
        s_run.status_cnt = 0;
    }
}
