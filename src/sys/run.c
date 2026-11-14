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
    uint32_t align_from;              /* 출발 때 두 바퀴 펄스 합(출발 정렬 거리 끝) */
    int16_t drive_started;            /* Flag_Driving_start */
    int16_t was_stopping, prev_len;   /* 이번 틱 정지 요청 판단 */
    int16_t empty_cnt;                /* 공차 판정 */
} s_run;

static void drive_pre(void);
/* 교차 테이프 찾기(tape_seek)와 시퀀스 교차선까지 이동(move_tape_tick) 상태. 아래 "교차선" 절 */
static struct { int16_t prev_id; uint32_t len, last; } s_tape;   /* len: 찾는 동안 간 펄스 */
static struct { int16_t on, armed, ticks, kind; uint32_t start; } s_mt;
static struct { int16_t kind, run, mv, ticks, id, dir, deg, edge, miss, hold; uint16_t next, as; float lag; uint32_t front, back, target, start, halt; } s_gd;

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
    s_gd.kind = s_gd.run = 0;           /* travel_pulses 가 0 부터 다시 센다 */
}

uint32_t run_pulses(uint16_t wheel) { return s_pulses[wheel]; }
int      run_is_idle(void)          { return s_run.idle; }
int      run_auto_started(void)     { return s_run.auto_start; }
int32_t  run_rail_cut(void)         { return s_rail_cut; }
void     run_rail_cut_clear(void)   { s_rail_cut = 0; }
void     run_reset_request(void)    { s_run.reset_req = 1; }
void     run_wait_start(void)       { D.wait_start = 1; s_gd.as = 0; }   /* 다른 출처의 대기: GD 끊김 때 거두지 않는다 */
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
    if (reset & RUN_STANDBY_RESET) { acs_station_reset(); }
    s_run.standby = id;
    if ((id > 0) && (D.turn_pulses == 0) && (D.turn_motion == 0)) {
        if (!(reset & RUN_STANDBY_NOSTOP)) {
            stop_request_rapid(STOP_REASON_COMMAND);
            D.flag_stopping = 1;
        }
        s_run.standby_auto = 1;
    }
    if ((reset & RUN_STANDBY_AUTO) && (id > 0)) { s_run.standby_auto = 1; }   /* 번호 0 이하(호기 - n 이 0 이하 등)는 되풀이해 보내지 않는다 */
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

static int16_t seek_rpm(int16_t kind);
static int16_t tape_busy(void);

/* 교차선 요청(분류 3·4 정지와 정지 뒤 시퀀스, 교차선 회전)과 GD 구간을 지운다. 처리 완료·한계 초과 말고는 사람의 STOP·비상 정지·
 * 오류(범퍼 포함)·수동 전환·ACS 작업 취소(run_seq_init 을 부르는 곳)에서만 지운다. 다시 출발(start_run: START·ACS 출발·자동 출발)은
 * 지우지 않는다: 다른 정지로 서는 동안 보류한 요청을 다시 달리면 이어 찾는다. 레거시는 출발(Initial_Start)에서도 지웠다 */
void run_tape_req_clear(void)
{
    s_gd.kind = 0;          /* 맡은 요청을 놓는다. 서는 중이면 조향 0·선 없음 막기는 설 때까지 둔다(gd_tick) */
    s_gd.as = 0;            /* 끊긴 정지가 예약을 되돌리지 않게 */
    D.tape_autostart = 0;
    D.tape_stop_id = 0;
    D.tape_stop_kind = 0;
    D.tape_turn_dir = 0;
    D.tape_turn_deg = 0;
    D.tape_turn_next = 0;
}

static void seq_init_keep_tape(void)
{
    seq_abort_all();
    D.move_on = 0;
    s_mt.on = 0;
    comm_recent_clear();
    acs_cross_clear();
    D.stopping_process = 0;
}

void run_seq_init(void)
{
    seq_init_keep_tape();
    run_tape_req_clear();
    tagact_fs_clear();                  /* 센서 정지 요청도: ACS 작업 취소 뒤 다시 출발해 감지 자리에서 서지 않게 */
    if (D.obs_use == 2) { D.obs_use = 1; }   /* OBS_OFF_AUTO: 켜 줄 OBS_ON 에 이제 닿지 못한다. OBS_OFF(0) 는 레거시대로 둔다 */
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
    s_run.align_from = s_pulses[0] + s_pulses[1];
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
    if (D.move_on == 0) { seq_init_keep_tape(); }   /* MOVE_MM 출발은 시퀀스를 살려 둔다. 교차선 요청은 출발로 지우지 않는다 */
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
    run_tape_req_clear();               /* 앞에서 찾던 교차선·맡은 GD 목표는 반대 방향에 없다 */
    tagact_fs_clear();                  /* 센서 정지 요청도 */
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
                    D.vin_out = stop_min_rpm();
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
        else if (D.move_on == 1) {          /* 시퀀스 MOVE_MM: 시퀀스를 살려 둔 채 출발. 정지 거리는 주행 첫 틱(drive_pre)에 */
            s_run.seq_go = (int16_t)seq_is_active();
            start_run();
            /* 서 있는 지금부터 센다(엔코더 ISR 이 줄인다). 정지는 주행 첫 틱에 건다. 교차선까지 이동은 위치 정지가 없다(한계만) */
            D.stop_pulses = s_mt.on ? 0 : (int16_t)D.move_pulses;
            run_tape_req_clear();           /* 남은 교차선 요청이 이동보다 먼저 돌지 않게 */
            D.move_on = 2;
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
/* 선을 본 채 출발 뒤 DRIVE_ALIGN_MAX_MM 을 갔다. 곡선 위 출발은 편차가 ±3 안에 안 들어와 정렬 확인이 곡선 끝까지 안 된다.
 * 거리는 두 바퀴 평균: 한 바퀴(travel_pulses)는 왼쪽 곡선 안쪽 바퀴라 거의 서 있을 수 있다 */
#define ALIGN_FAR()  (!D.guide_all_off && ((float)(int32_t)(s_pulses[0] + s_pulses[1] - s_run.align_from) \
                                           >= (2.0f * (float)DRIVE_ALIGN_MAX_MM) * P.one_length_pulse))
#else
#define ALIGN_SEEN() 1
#define ALIGN_FAR()  0
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
        if (guide_within(START_ALIGN_RANGE) && ALIGN_SEEN()) { s_run.init_cnt++; }
        else                                                 { s_run.init_cnt = 0; }
        if ((s_run.init_cnt > START_ALIGN_TICKS) || ALIGN_FAR()) {
#if AGV_START_ALIGN
            D.start_align = DRIVE_ALIGN_RAMP;
#endif
            /* 지령(speed_cmd)은 덮지 않는다: 출발 때 이미 LOS 이고, 정렬 전에 읽은 속도 태그를 지우지 않는다 */
            D.step = DRIVE_CHANGE_SPEED;
            s_run.init_step = START_LOW_DONE;
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
    if ((D.move_on == 2) && s_mt.on) {  /* AGV_MOVE( 한계, TAPE… ) 첫 주행 틱: 찾는 속도로 간다. 서는 것은 move_tape_tick */
        D.speed_cmd = seek_rpm(s_mt.kind);
        s_mt.start = s_pulses[0];
        s_mt.armed = 0;
        s_mt.ticks = 0;
        D.move_on = 3;
    }
    else if (D.move_on == 2) {          /* MOVE_MM 첫 주행 틱: 지금 자리에서 move_pulses 앞에 선다(stop_run 이 STOP_REASON_MOVE 로 건다) */
        D.speed_cmd = D.move_speed;
        if (D.stop_pulses <= 0) { D.stop_pulses = 1; }   /* 출발 전에 다 왔으면 바로 선다 */
        D.flag_stopping = 1;
        D.move_on = 3;
    }
    if (tune_active() || tape_busy()) { return; }   /* 튜닝·식별 시험, 시퀀스 이동·교차선 찾기·GD 구간은 속도를 스스로 정한다 */
    start_switch_high();
    if (D.obs_use == 1) { auto_speed_change(); }
}

/* ---- 교차선 찾아 서기 (DA_Tape_Stop·DA_Tape_Turn) ---- */

#define TAPE_ID_TURN (-1)   /* 분류(3·4)와 겹치지 않는 회전 탐색 표시 */

/* 정렬 회전 앞 교차 테이프 찾기 속도. 센서가 회전 중심보다 GD 앞이라 테이프를 본 뒤 GD 안에 서면 된다.
 * 인정(새 응답 두 번) 지연 t 동안 v·t 를 가고, 감속도 k·DCC(1 = 100 rpm/s)로 GD - 여유 - v·t 안에 선다:
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

/* ---- 교차선: 인정(run_tape_rx), 찾아 서기(tape_stop), 교차선 뒤 GD 구간(gd_*) ----
 * 분류 3(AGV_STOP( TAPE )·SEQ_START( TAPE, x )·AGV_MOVE( 한계, TAPE ))은 SPN 으로 앞에서 찾아 본 자리에서 바로 선다.
 * 분류 4(… TAPE_GD)와 교차선 회전은 run_seek_rpm 으로 앞에서 찾은 교차선을 GD 구간 루틴으로 회전 중심 아래에 세운다.
 * 요청 전에 지난 교차선은 쓰지 않는다(태그를 읽을 때 센서는 교차선 앞이다. 틀린 선에서 서는 것이 가장 나쁘다) */

/* 인정: 새 가이드 응답마다 본다. 한쪽(바깥 끝 칸부터 주행 테이프 덩어리 바로 앞까지)이 다 켜졌고 그 칸 수가
 * TAPE_SIDE_MIN(일반 센서 TAPE_SIDE_MIN_N) 이상이며, 그 쪽이 마지막으로 2/3 넘게 비어 있던 때부터 TAPE_FILL_MM 안에
 * 찼으면(처음 찬 응답에서 정하고 찬 동안 그대로), 또는 전부 켜짐이면 교차선 위 응답이다. 주행 테이프가 켠 칸은 세지 않는다.
 * 비스듬한 분기·합류선은 그 쪽을 채우는 데 한참 걸리고(폭 / cos(각)만 켜져 아예 못 채우기도 한다), 교차선은 방향 오차 몇 도면
 * 한두 응답에 찬다. 칸 하나가 한 응답 꺼져도 기준이 새로 잡히지 않고, 옆에 칸 몇 개를 켠 선이 오래 있어도 교차선을 인정한다.
 * 빈 칸 하나를 허용하면(LED 불량·이음 틈) 모의에서 wide 가 50 mm 합류선을 잘못 인정해(6/912) 허용하지 않는다.
 * 찾는 중 교차선 위 응답이 이어서 두 번이면 인정한다(아닌 응답에 0, 응답마다 한 번).
 * 주행 테이프 덩어리는 교차선 위가 아닌 응답에서 추적기가 고른 덩어리(폭 ≤ 배운 폭 + 2)를 기억한다 */
#ifndef TAPE_SIDE_MIN
#define TAPE_SIDE_MIN    8
#endif
#ifndef TAPE_SIDE_MIN_N
#define TAPE_SIDE_MIN_N  4
#endif
#ifndef TAPE_FILL_MM
#define TAPE_FILL_MM     20.0f
#endif
static struct {
    int16_t  sat;            /* 마지막 응답이 교차선 위 */
    int16_t  hits;           /* 찾는 중 교차선 위 응답이 이어진 수(2 에서 멈춤) */
    int16_t  clear;          /* 교차선 위가 아닌 응답이 이어진 수(2 에서 멈춤). 시퀀스 이동 출발 때 선을 벗어남 */
    uint16_t lane;           /* 주행 테이프 덩어리: 시작 칸 | 끝 칸 << 8. 0 = 모름(가운데 두 칸) */
    int16_t  fill[2];        /* 왼쪽(칸 1)·오른쪽 쪽이 다 찼다: 0 아님, 1 빨리 참(교차선), 2 늦게 참 */
    uint32_t first;          /* 찾는 중 교차선 위 응답 첫째 때 travel_pulses(앞 가장자리) */
    uint32_t dark_at[2];     /* 그 쪽이 마지막으로 2/3 넘게 비어 있던 때 travel_pulses */
} s_cr;

/* 교차선 뒤 GD 구간(분류 4 정지·교차선 회전·AGV_MOVE( 한계, TAPE_GD )): 정지를 건 틱부터 설 때까지 조향 보정 0(D.steer_hold,
 * 두 바퀴 같은 지령, 적분 지움)과 "선 없음" 오류 막기. 목표 = 앞 가장자리 + GD + 반폭 - 응답 늦음(펄스, 절대 자리).
 * 반폭은 TAPE_HALF_MM 으로 걸고, 뒤 가장자리(교차선 위가 아닌 응답이 이어서 두 번, 첫 응답 자리)를 보면 (뒤 - 앞) / 2 로,
 * 앞 가장자리 뒤 TAPE_BACK_MM 을 가도 못 보면 TAPE_BACK_MM / 2(적어도 그만큼 넓다)로 고친다.
 * 인정한 교차선은 요청과 함께 맡는다(s_gd). 회전이 거부되거나(서 있어 속도 ≈ 0 등) GD 정지가 다른 정지(명령 정지 등)로 끝나면
 * 맡은 채 두고, 다시 달리면 그 목표로 남은 거리를 가서 서고 돈다. 정지 뒤 시퀀스는 위치 정지로 섰을 때 시작한다.
 * 목표를 TAPE_BACK_MM 넘게 지나면(선 뒤·다시 걸 때. 서는 중에는 제어 감속을 끝까지 한다) 회전·시퀀스를 걸지 않고 258.
 * 분류 3 은 기록만. 맡은 채 서 있는 동안(선 뒤 TAPE_SETTLE_TICKS 에 기준 자리) 바퀴가 TAPE_HALT_MM 넘게 돌면(밀림. 엔코더는
 * 방향을 모른다) 자리를 몰라 258. 맡은 채 다른 회전·시퀀스가 시작되면 맡은 것을 놓는다(그 뒤 자리는 교차선과 관계없다) */
#define TAPE_HALF_MM     10.0f
#define TAPE_BACK_MM     60.0f
#define TAPE_HALT_MM     20.0f   /* 맡은 채 서 있을 때 떨림·밀림 허용 */
#define TAPE_SETTLE_TICKS 50     /* 선 뒤 굴러감이 끝날 때까지(0.5 s) 밀림 기준을 잡지 않는다 */
#ifndef TAPE_LAG_MS
#define TAPE_LAG_MS      18.0f   /* 앞·뒤 가장자리 모두 늦게 본다: 응답 10 ms + 센서 스캔 나이 + 응답 사이 평균 5 ms. 모의로 맞춤 */
#endif
run_tape_last_t run_tape_last;
/* s_gd(위 선언): kind 맡은 요청(0 없음, 3 바로 정지, 4 GD 정지, TAPE_ID_TURN 교차선 회전. 위치 정지로 서거나 요청이 지워질 때까지.
 * 맡을 때 요청 칸을 옮겨 두고 지운다: id 정지 뒤 시퀀스, dir·deg·next 회전), run 1 = 정지를 건 뒤 서는 중(조향 0),
 * mv 1 = 시퀀스 이동, front·back 앞·뒤 가장자리, edge 0 분류 3, 1 뒤 가장자리 기다림, 2 봄, 3 못 봄, miss 뒤 가장자리의 아닌 응답 수,
 * lag 인정 때 응답 늦음(펄스), target 목표, start 인정한 자리, as 이 정지에 건 자동 출발(D.tape_autostart 값, 0 없음),
 * hold 맡은 채 선 뒤 틱(TAPE_SETTLE_TICKS 에서 멈춤), halt 그때 travel_pulses(밀림 기준) */

/* 교차선 요청을 처리하는 상태: 주행 중이고 다른 정지·회전이 진행 중이 아니다(flag_stopping·turn_motion). 아니면 요청을 두고(보류)
 * 속도 지령·인정을 건드리지 않는다. 장애물 일시정지는 정지 제어의 일시정지가 맡는다(flag_stopping 아님, 풀리면 이어 찾는다) */
static int16_t tape_req_ok(void)
{
    return D.flag_run && (D.control == RUN_DRIVE) && !D.flag_stopping && (D.turn_motion == 0);
}

static int16_t seeking(void)
{
    return (((D.tape_stop_kind != 0) || (D.tape_turn_dir != 0)) && tape_req_ok()) || (s_mt.on && s_mt.armed && !D.flag_stopping);
}

/* 교차선 일이 속도를 정하는 동안: 자동 속도 변경·START 고속이 덮지 않는다 */
static int16_t tape_busy(void)
{
    return (D.move_on != 0) || (D.tape_stop_kind != 0) || (D.tape_turn_dir != 0) || s_gd.kind || s_gd.run;
}

/* 쪽 i: 바깥 끝 비트 from 부터 안쪽(step)으로 n 칸(주행 테이프 바로 앞까지, 0~칸 수 max 로 묶는다) */
static int16_t side_lit(uint32_t v, int16_t i, int16_t from, int16_t step, int16_t n, int16_t min, int16_t max)
{
    int16_t k, dark = 0;
    n = (n < 0) ? 0 : ((n > max) ? max : n);
    for (k = 0; k < n; k++) { dark = (int16_t)(dark + !((v >> (from + k * step)) & 1UL)); }
    if ((n < min) || (dark > 0)) {
        if (3 * dark >= 2 * n) { s_cr.dark_at[i] = D.travel_pulses; }
        s_cr.fill[i] = 0;
        return 0;
    }
    if (s_cr.fill[i] == 0) {        /* 처음 찬 응답에서 정하고 찬 동안 그대로 */
        s_cr.fill[i] = ((float)(int32_t)(D.travel_pulses - s_cr.dark_at[i]) <= TAPE_FILL_MM * P.one_length_pulse) ? 1 : 2;
    }
    return s_cr.fill[i] == 1;
}

/* 맡은 교차선을 회전 중심 아래에 세울 자리(절대 펄스) */
static uint32_t gd_target(void)
{
    float half = (s_gd.edge >= 2) ? 0.5f * (float)(int32_t)(s_gd.back - s_gd.front) : TAPE_HALF_MM * P.one_length_pulse;
    return s_gd.front + (uint32_t)(int32_t)(P.guide_dist_mm * P.one_length_pulse + half - s_gd.lag + 0.5f);
}

/* 남은 정지 거리를 d 펄스만큼 고친다(ISR 이 줄이는 중이라 막고. 다 줄었으면 그대로) */
static void stop_add(int32_t d)
{
    uint16_t irq = hal_irq_save();
    if (D.stop_pulses > 0) {
        int32_t r = (int32_t)D.stop_pulses + d;
        D.stop_pulses = (int16_t)((r < 1L) ? 1L : ((r > 32767L) ? 32767L : r));
    }
    hal_irq_restore(irq);
}

/* 뒤 가장자리를 봤거나 못 봤다: 반폭으로 목표를 고친다. 서는 중이면 남은 정지 거리도 */
static void gd_retarget(void)
{
    uint32_t t = gd_target();
    if (s_gd.run) { stop_add((int32_t)(t - s_gd.target)); }
    s_gd.target = t;
}

/* guide_poll 이 'h' 응답마다 부른다. s·e = 추적기가 고른 덩어리(없으면 0) */
void run_tape_rx(uint16_t s, uint16_t e)
{
    int16_t n = D.guide_wide ? GUIDE_LED_WIDE : GUIDE_LED_NARROW, min = D.guide_wide ? TAPE_SIDE_MIN : TAPE_SIDE_MIN_N;
    int16_t ls = (int16_t)(s_cr.lane & 0xFFu), le = (int16_t)(s_cr.lane >> 8), w = (int16_t)guide_tape_width();
    uint32_t v = D.guide_wide ? D.guide_led32 : (uint32_t)D.guide_led;

    if (s_cr.lane == 0u) { ls = n / 2; le = (int16_t)(ls + 1); }
    s_cr.sat = side_lit(v, 0, 0, 1, (int16_t)(ls - 1), min, n);
    s_cr.sat |= side_lit(v, 1, (int16_t)(n - 1), -1, (int16_t)(n - le), min, n);    /* 두 쪽 다 본다(기준 때 기록) */
    s_cr.sat |= (v == (D.guide_wide ? 0xFFFFFFFFUL : 0xFFFFUL));
    if (!s_cr.sat && (s >= 1u) && (s <= e) && (e <= (uint16_t)n) && ((w == 0) || ((int16_t)(e - s) < w + 2))) {
        s_cr.lane = (uint16_t)(s | (e << 8));     /* 칸 1~n 인 덩어리만(추적기 시작 칸이 음수면 0xFFFF 로 온다) */
    }
    if (s_cr.sat && seeking()) {
        if (s_cr.hits == 0) { s_cr.first = D.travel_pulses; }
        if (s_cr.hits < 2) { s_cr.hits++; }
    }
    else { s_cr.hits = 0; }
    if (s_cr.sat) { s_cr.clear = 0; }
    else if (s_cr.clear < 2) { s_cr.clear++; }
    if (s_gd.edge == 1) {
        if (s_cr.sat) { s_gd.miss = 0; }
        else if (++s_gd.miss == 1) { s_gd.back = D.travel_pulses; }
        else {
            s_gd.edge = 2;
            gd_retarget();
        }
    }
}

/* 인정한 틱: 찾은 교차선(앞 가장자리 s_cr.first)을 맡는다. 태그 요청이면 요청 칸을 옮겨 두고 지운다 */
static void gd_claim(int16_t kind, int16_t mv)
{
    s_gd.kind = kind;
    s_gd.mv = mv;
    s_gd.ticks = 0;
    s_gd.start = D.travel_pulses;
    s_gd.front = s_cr.first;
    s_gd.edge = (kind == 3) ? 0 : 1;
    s_gd.miss = 0;
    s_gd.lag = (float)D.vin_out * P.one_rpm_meter * (TAPE_LAG_MS / 60.0f) * P.one_length_pulse;
    s_gd.target = gd_target();
    s_gd.as = 0;
    s_gd.hold = 0;
    if (!mv) {
        s_gd.id = D.tape_stop_id;
        s_gd.dir = D.tape_turn_dir;
        s_gd.deg = D.tape_turn_deg;
        s_gd.next = D.tape_turn_next;
        if (kind == TAPE_ID_TURN) { D.tape_turn_dir = 0; }
        else { D.tape_stop_kind = D.tape_stop_id = 0; }
        if ((kind == 3) && (s_gd.id != 0)) { seq_request((uint16_t)s_gd.id); }   /* 본 자리에서 바로 서니 지금(서면 auto_run 이 시작) */
    }
}

/* 맡은 교차선에 정지를 건다. 분류 3 은 본 자리에서 바로(정지 거리 0), 4·회전은 GD 구간(목표까지). 정지 거리를 돌려준다 */
static int16_t gd_start(int16_t kind, int16_t mv)
{
    int32_t p;

    s_gd.run = 1;
    s_gd.hold = 0;
    if (!mv && (kind != TAPE_ID_TURN) && (D.tape_autostart != 0u)) {   /* 분류 3·4 의 자동 출발은 정지를 실제로 걸 때 예약 */
        s_gd.as = D.tape_autostart;                                    /* 끊기면 gd_tick 이 거둔다 */
        tagact_autostart_arm((uint16_t)(D.tape_autostart - 1u));
        D.tape_autostart = 0;
    }
    if (kind == 3) { return 0; }
    D.steer_hold = 1;
    p = (int32_t)(s_gd.target - D.travel_pulses);
    return (p < 1L) ? 1 : ((p > 32767L) ? 32767 : (int16_t)p);
}

static void tape_found(int16_t kind, int16_t mv)
{
    int16_t p = gd_start(kind, mv);
    D.flag_stopping = 1;
    D.stop_pulses = p;
    stop_request_tag();
}

/* 기록(run_tape_last). 지나침: 4·회전은 목표 뒤, 3 은 정지를 건 뒤 간 거리 */
static void gd_record(void)
{
    run_tape_last.kind = s_gd.kind;
    run_tape_last.run_mm = (float)(int32_t)(D.travel_pulses - s_gd.start) * P.one_pulse_length;
    run_tape_last.width_mm = (s_gd.edge == 2) ? (float)(int32_t)(s_gd.back - s_gd.front) * P.one_pulse_length : -1.0f;
    run_tape_last.ticks = s_gd.ticks;
    run_tape_last.over_mm = (float)(int32_t)(D.travel_pulses - ((s_gd.kind == 3) ? s_gd.start : s_gd.target)) * P.one_pulse_length;
    run_tape_last.n++;
}

/* 4·회전이 목표를 TAPE_BACK_MM 넘게 지났다(서지 못함) */
static int16_t gd_overrun(void)
{
    return (s_gd.kind != 0) && (s_gd.kind != 3)
        && ((int32_t)(D.travel_pulses - s_gd.target) > (int32_t)(TAPE_BACK_MM * P.one_length_pulse));
}

/* 지나침 오류: 기록하고 회전·정지 뒤 시퀀스를 걸지 않고 258(자동 출발은 오류가 지운다) */
static void gd_overrun_fail(void)
{
    gd_record();
    D.system_err = ERR_GUIDE_SENSOR;
    if (s_gd.kind == TAPE_ID_TURN) {
        D.turn_pulses = 0;
        D.turn_motion = 0;
        D.turn_rpm = 0;
        D.tape_align = 0;
    }
    s_gd.kind = 0;
    s_gd.as = 0;
    D.tape_autostart = 0;
}

/* GD 구간 매 틱. 섰을 때: 위치 정지로 섰으면(분류 3 은 어떻게 섰든) 요청을 끝낸다(정지 뒤 시퀀스 시작, 기록). 다른 정지로 끊긴
 * 태그 요청은 맡은 채 두고(이 정지에 건 자동 출발은 거두어 되돌린다) 다시 달릴 때 이어 간다(tape_stop) */
static void gd_tick(void)
{
    if (s_gd.kind && !s_gd.run && (D.flag_turn || (D.turn_motion != 0) || seq_is_active())) {
        s_gd.kind = 0;                  /* 맡은 채 다른 회전·시퀀스: 놓는다(되돌려 둔 자동 출발도) */
        s_gd.as = 0;
        D.tape_autostart = 0;
        return;
    }
    if (!D.flag_run) {
        int16_t r = stop_reason();
        int16_t done = s_gd.run && ((s_gd.kind == 3) || (stop_is_position(r) && (r != STOP_REASON_TRAFFIC_POSITION)));
        D.steer_hold = 0;
        if (done && gd_overrun()) {
            s_gd.run = 0;
            gd_overrun_fail();
            return;
        }
        if (s_gd.run && !done && s_gd.as && s_gd.kind) {   /* 끊긴 정지의 자동 출발은 거둔다(교통 정지를 어기지 않게). 다시 걸 때 다시 */
            D.wait_start = 0;
            s_run.wait_sec = 0;                 /* 다음 다시 출발('W' 'C' '1' 등)이 남은 초를 기다리지 않게 */
            D.tape_autostart = s_gd.as;
        }
        if (s_gd.run) { s_gd.as = 0; }
        s_gd.run = 0;
        if (done && s_gd.kind) {
            gd_record();
            if (!s_gd.mv && (s_gd.kind == 4) && (s_gd.id != 0)) { seq_request((uint16_t)s_gd.id); }   /* GD 에 선 뒤에. auto_run 이 시작 */
        }
        if (done || s_gd.mv || !s_gd.kind) {
            s_gd.kind = 0;
            return;
        }
        if (s_gd.hold < TAPE_SETTLE_TICKS) {
            if (++s_gd.hold == TAPE_SETTLE_TICKS) { s_gd.halt = D.travel_pulses; }   /* 굴러감이 끝난 자리 */
        }
        else if ((float)(int32_t)(D.travel_pulses - s_gd.halt) > TAPE_HALT_MM * P.one_length_pulse) {
            s_gd.kind = 0;              /* 맡은 채 서 있는 동안 밀렸다: 교차선과 회전 중심 거리를 모른다 */
            D.system_err = ERR_GUIDE_SENSOR;
        }
        return;
    }
    s_gd.hold = 0;
    if (s_gd.ticks < 32767) { s_gd.ticks++; }
    if (s_gd.kind == 3) { return; }
    if (s_gd.run) { run_rail_cut_clear(); }
    if ((s_gd.edge == 1) && ((float)(int32_t)(D.travel_pulses - s_gd.front) > TAPE_BACK_MM * P.one_length_pulse)) {
        s_gd.back = D.travel_pulses;                /* 못 봤다: 교차선은 적어도 이만큼 넓다 */
        s_gd.edge = 3;
        gd_retarget();
    }
}

static int16_t seek_rpm(int16_t kind) { return (kind == 3) ? P.min_speed : run_seek_rpm(); }

/* 교차선을 앞에서 찾는다. 1 찾음, -1 한계(정지 거리 2배)를 넘김, 0 계속(보류 포함). 한계는 요청을 건 뒤 찾는 동안(보류 아닐 때) 간 거리 */
static int16_t tape_seek(int16_t rpm)
{
    uint32_t now = s_pulses[0];

    if (tape_req_ok()) { s_tape.len += now - s_tape.last; }
    s_tape.last = now;
    if (!tape_req_ok()) { return 0; }
    D.speed_cmd = rpm;
    D.step = DRIVE_CHANGE_SPEED;
    if (s_cr.hits >= 2) {
        s_cr.hits = 0;
        return 1;
    }
    if (!s_cr.sat && ((int32_t)s_tape.len > 2L * P.stop_len)) {
        D.system_err = ERR_GUIDE_SENSOR;
        return -1;
    }
    return 0;
}

/* 맡은 태그 요청에 정지를 건다(인정한 틱, 회전이 거부된 뒤, 다른 정지로 끊긴 뒤 다시 달릴 때). 1 걸었다,
 * 0 아직(회전이 거부됐다: 맡은 채 다음 틱에 다시). 목표를 TAPE_BACK_MM 넘게 지났으면 회전·자동 출발 없이 바로 제어 정지를 걸고
 * 선 뒤에 258(gd_tick) */
static int16_t gd_try(void)
{
    if (gd_overrun()) {
        s_gd.run = 1;
        D.flag_stopping = 1;
        D.stop_pulses = 0;
        stop_request_tag();
        return 1;
    }
    if (s_gd.kind != TAPE_ID_TURN) {
        tape_found(s_gd.kind, 0);
        return 1;
    }
    tagact_turn_align_cross((turn_dir_t)s_gd.dir, (uint16_t)s_gd.deg, s_gd.next);
    if (!D.tape_align) { return 0; }
    if (!D.flag_stopping) { s_gd.kind = 0; }        /* 서 있다 바로 도는 회전(시퀀스 중): 맡을 정지가 없다 */
    else { D.stop_pulses = gd_start(TAPE_ID_TURN, 0); }   /* tagact_turn 이 건 GD 정지를 목표로 고친다 */
    return 1;
}

/* 시퀀스 AGV_MOVE( 한계, TAPE… ): 서 있다 지금 방향으로 출발해 다음 교차선에서 선다. 출발 때 센서가 선 위면 한 번 벗어난 뒤부터 센다.
 * 한계 거리(D.move_pulses)를 넘기면 서고 오류(교차선 정지가 못 찾을 때와 같은 258). 시퀀스는 run_after_motor 가 끝낸다 */
#define MOVE_TAPE_ARM_TICKS 5    /* 출발 뒤 새 가이드 응답이 올 때까지 선 위인지 판단하지 않는다(10 ms × 5) */

void run_move_tape_start(int16_t kind) { s_mt.on = 1; s_mt.kind = kind; }
int  run_move_tape_on(void)            { return s_mt.on || s_gd.run || s_gd.kind; }   /* 'H' 를 받아야 한다(맡은 교차선의 뒤 가장자리 포함) */
int  run_tape_turn_held(void)          { return s_gd.kind == TAPE_ID_TURN; }

static void move_tape_tick(void)
{
    int16_t v = seek_rpm(s_mt.kind);

    if (D.speed_cmd != v) {
        D.speed_cmd = v;
        D.step = DRIVE_CHANGE_SPEED;
    }
    if (s_mt.ticks < MOVE_TAPE_ARM_TICKS) { s_mt.ticks++; }
    else if (!s_mt.armed) {
        if (s_cr.clear >= 2) {                     /* 교차선 밖(또는 출발한 선을 벗어남): 아닌 응답 둘 뒤부터 센다 */
            s_mt.armed = 1;
            s_cr.hits = 0;
        }
    }
    else if (s_cr.hits >= 2) {
        s_cr.hits = 0;
        s_mt.on = 0;
        D.move_on = MOVE_FOUND;                    /* 교차선 정지와 같은 태그 정지로 서면 finish 가 MOVE_DONE */
        gd_claim(s_mt.kind, 1);
        tape_found(s_mt.kind, 1);
        return;
    }
    if ((int32_t)(s_pulses[0] - s_mt.start) > D.move_pulses) {
        s_mt.on = 0;
        D.system_err = ERR_GUIDE_SENSOR;
    }
}

static void tape_stop(void)
{
    int16_t kind = D.tape_stop_kind;

    if (s_gd.run || s_gd.kind) { gd_tick(); }
    if ((D.move_on == 0) || (D.move_on >= MOVE_FOUND)) { s_mt.on = 0; }   /* 이동이 끝났다(어떤 원인이든) */
    if ((kind == 0) && (P.turn_mode <= 0)) { D.tape_turn_dir = 0; }   /* 회전 모드 0: 교차선 회전 요청은 받지 않는다 */
    if ((kind == 0) && (D.tape_turn_dir != 0)) { kind = TAPE_ID_TURN; }
    if (s_gd.kind && !s_gd.run) {                  /* 맡은 요청(회전 거부·다른 정지로 끊김): 달리면 다시 건다 */
        if (tape_req_ok() && (gd_try() < 0)) { run_tape_req_clear(); }
    }
    else if ((kind != 0) && !s_gd.run) {
        int16_t r;
        if (kind != s_tape.prev_id) {              /* 새 요청 */
            s_tape.prev_id = kind;
            s_tape.len = 0;
            s_tape.last = s_pulses[0];
        }
        r = tape_seek(seek_rpm(kind));
        if (r == 1) {
            gd_claim(kind, 0);
            r = gd_try();
        }
        if (r < 0) { run_tape_req_clear(); }
    }
    else if (s_mt.on && (D.move_on == 3)) {
        move_tape_tick();
    }
    else if (kind == 0) {
        s_tape.prev_id = 0;
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
    /* AGV_MOVE( 한계, TAPE_GD ): 이 틱 finish 가 도착(MOVE_DONE)으로 두었어도 목표를 지나쳤으면 시퀀스가 읽기 전에 실패 + 258 */
    if ((D.move_on == MOVE_DONE) && s_gd.mv && s_gd.run && gd_overrun()) {
        D.move_on = MOVE_FAIL;
        gd_overrun_fail();
    }
    if (s_run.seq_go) { seq_run(); }
    else              { seq_belt_tick(); }   /* 시퀀스가 살아 있으면(이동 중 등) 건 벨트 정지 조건은 매 틱 본다 */
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
