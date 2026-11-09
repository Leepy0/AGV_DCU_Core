/* tagact.c - 태그 규칙 표 실행과 개별 액션. 설계: docs/03_시퀀스_DSL.md 5절 */
#include "agv/tagact.h"
#include "agv/acs.h"
#include "agv/drive.h"
#include "agv/hal.h"
#include "agv/io.h"
#include "agv/log.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/seq.h"
#include "agv/stop.h"
#include "agv/tune.h"

#define D agv_drive
#define P param_dv

#define TURN_STOP_MIN_VIN 3     /* 이보다 빠르면 회전 전에 정지 거리를 건다 */
#define TAPE_WINDOW_DEG   20.0f /* 정렬 회전이 새 라인을 찾는 목표각 앞뒤 범위 */

static const tag_rule_t *s_rules;
static uint16_t s_nrules, s_profile, s_display_tag;

void tagact_bind(const tag_rule_t *rules, uint16_t n, uint16_t profile)
{
    s_rules = rules;
    s_nrules = n;
    s_profile = profile;
}

/* ---- 개별 액션 ---- */

void tagact_stop(void) { D.flag_stopping = 1; }

void tagact_stop_length(void)
{
    if (D.flag_stopping) { return; }     /* 진행 중인 정지의 거리를 바꾸지 않는다 */
    D.flag_stopping = 1;
    D.stop_pulses = D.backward ? P.stop_len : P.fwd_stop_len;
    stop_request_tag();
}

void tagact_speed(uint16_t level)
{
    switch (level) {
    case 0:  D.speed_cmd = P.target_speed; break;
    case 1:  D.speed_cmd = P.low_speed; break;
    case 2:  D.speed_cmd = P.llow_speed; break;
    case 3:  D.speed_cmd = P.min_speed; break;
    default: D.speed_cmd = param_rpm((int16_t)level); break;
    }
    D.step = DRIVE_CHANGE_SPEED;
}

void tagact_forback(dir_t d)
{
    if (P.forback_mode > 0) {
        run_dir_change((d == DIR_TOGGLE) ? (D.backward ? 1 : 2) : (int16_t)d);
    }
}

static int16_t s_turn_from_stop;   /* 1 = 멈춘 상태에서도 회전을 건다 (튜닝 모드) */
static int16_t s_turn_stop_len;    /* 0 = 정지 거리(SL). 교차선 기준이면 센서 거리(GD) */

void tagact_turn(turn_dir_t d, uint16_t deg, dir_t fb, uint16_t autostart, uint16_t obs)
{
    if ((P.turn_mode <= 0) || (D.turn_pulses != 0) || (D.stop_pulses != 0)
        || (!seq_is_active() && !s_turn_from_stop && (D.vin_out <= TURN_STOP_MIN_VIN))) {
        return;
    }
    if (D.vin_out >= TURN_STOP_MIN_VIN) {
        D.flag_stopping = 1;
        D.stop_pulses = (s_turn_stop_len > 0) ? s_turn_stop_len : P.stop_len;
    }
    /* 회전 뒤 방향 지정: 지금 방향과 다르면 반대로 돌아 같은 곳을 본다 */
    if ((int16_t)fb > DIR_TOGGLE) {
        if ((!D.backward && (fb == DIR_BACK_AFTER_TURN)) || (D.backward && (fb == DIR_FWD_AFTER_TURN))) {
            if (deg >= 180u) { return; }
            d = (d == TURN_CW) ? TURN_CCW : TURN_CW;
            deg = (uint16_t)(180u - deg);
        }
        fb = (dir_t)((int16_t)fb - DIR_TOGGLE);
    }
    tagact_forback(fb);
    D.turn_motion = (d == TURN_CW) ? MOTION_CW : MOTION_CCW;
    if (deg == 90u)       { D.turn_pulses = P.turn_90; }
    else if (deg == 180u) { D.turn_pulses = P.turn_180; }
    else                  { D.turn_pulses = (int16_t)((P.turn_90 / 90) * (int16_t)deg); }
    D.turn_obs_use = (int16_t)obs;
    if (autostart == 1u) { D.wait_start = (P.turn_mode == 2) ? 1 : 0; }
    if (D.flag_stopping) { stop_request_tag(); }
}

void tagact_turn_align(turn_dir_t d, uint16_t deg, uint16_t next)
{
    int16_t before = D.turn_pulses;
    uint16_t fb = next & 0xFu;

    tagact_turn(d, deg, (dir_t)fb, (uint16_t)((next >> 4) & 0xFu), (uint16_t)((next >> 8) & 0xFu));
    if ((before != 0) || (D.turn_pulses <= 0)) { return; }
    D.tape_total_pulse = D.turn_pulses;
    D.tape_target_cnt = (float)D.turn_pulses;
    D.tape_max = (int16_t)(((float)P.turn_90 / 90.0f) * TAPE_WINDOW_DEG);
    D.tape_slip_ratio = 1.0f;
    D.tape_lost_cnt = 0;
    D.tape_turned = 0;
    D.tape_origin_ok = 0;
    /* 방향이 바뀌면 센서 전원도 바뀌어 출발 라인을 볼 수 없다. 슬립 측정을 건너뛴다 */
    D.tape_step = (fb != 0u) ? 1 : 0;
    D.tape_align = 1;
}

/* 교차선을 앞 센서가 본 순간에 부른다. 센서 거리만큼 가야 회전 중심이 선 위에 온다 */
void tagact_turn_align_cross(turn_dir_t d, uint16_t deg, uint16_t next)
{
    s_turn_stop_len = (int16_t)(P.guide_dist_mm * P.one_length_pulse + 0.5f);
    tagact_turn_align(d, deg, next);
    s_turn_stop_len = 0;
}

void tagact_turn_align_stopped(turn_dir_t d, uint16_t deg, uint16_t next)
{
    s_turn_from_stop = 1;
    tagact_turn_align(d, deg, next);
    s_turn_from_stop = 0;
}

void tagact_branch(branch_dir_t d)
{
    if (D.speed_cmd > P.low_speed) {
        D.speed_cmd = P.low_speed;
        D.step = DRIVE_CHANGE_SPEED;
    }
    D.branch_start = (int16_t)d;
}

/* 수신과 처리 사이에 이미 달린 거리를 남은 정지 거리에서 뺀다. RFID 재시도도 같은 경로다 */
void tagact_compensate(int32_t pulses)
{
    uint16_t irq;

    if ((pulses <= 0) || !D.flag_stopping || (D.stop_pulses <= 0) || !D.flag_run || D.system_err || D.estop
        || (D.control != RUN_DRIVE) || (D.turn_pulses > 0) || D.flag_turn) {
        return;
    }
    irq = hal_irq_save();
    if (pulses >= (int32_t)D.stop_pulses) {
        D.stop_pulses = 0;
        D.immediate_stop = 1;
        hal_irq_restore(irq);
        stop_now();
        return;
    }
    D.stop_pulses = (int16_t)(D.stop_pulses - (int16_t)pulses);
    hal_irq_restore(irq);
}

/* ---- 표 ---- */

static uint16_t cond_ok(const tag_rule_t *r)
{
    return seq_xcond(r->cond, r->cond_a, r->cond_b, r->tag) && seq_xcond(r->cond2, r->cond2_a, r->cond2_b, r->tag)
        && seq_mask_true(io_in_mask(), r->in_on, r->in_off);
}

static const tag_rule_t *rule_find(uint16_t tag)
{
    uint16_t i;
    uint16_t profile = (s_profile != 0u) ? s_profile : seq_profile();

    for (i = 0; i < s_nrules; i++) {
        const tag_rule_t *r = &s_rules[i];
        if ((r->tag == tag) && ((r->profile == 0u) || (r->profile == profile)) && cond_ok(r)) { return r; }
    }
    return 0;
}

static uint16_t has_second_word(uint16_t code)
{
    uint16_t kind = (uint16_t)(TA_KIND(code) << 8);
    return (uint16_t)((kind == TA_TURN) || (kind == TA_TURN_ALIGN) || (kind == TA_TURN_TAPE)
                      || (kind == TA_ACS_NUM) || (kind == TA_STATE));
}

static void autostart(uint16_t sec)
{
    uint16_t wait = (sec != 0u) ? sec : (uint16_t)param_get(P_AUTO_START_SEC);
    D.flag_stopping = 1;
    if (wait != 0u) { run_request_restart(wait); }
}

static void act_exec(uint16_t code, uint16_t next)
{
    uint16_t arg = TA_ARG(code);

    switch ((uint16_t)(TA_KIND(code) << 8)) {
    case TA_STOP:           tagact_stop(); break;
    case TA_STOP_AUTOSTART: autostart(arg); break;
    case TA_SPEED:          tagact_speed(arg); break;
    case TA_FORBACK:        tagact_forback((dir_t)arg); break;
    case TA_TURN:
        tagact_turn((turn_dir_t)(arg & 3u), (uint16_t)((arg >> 2) * 90u), (dir_t)(next & 0xFu),
                    (uint16_t)((next >> 4) & 0xFu), (uint16_t)((next >> 8) & 0xFu));
        break;
    case TA_TURN_ALIGN:
        tagact_turn_align((turn_dir_t)(arg & 3u), (uint16_t)((arg >> 2) * 90u), next);
        break;
    case TA_TURN_TAPE:
        D.tape_turn_dir = (int16_t)(arg & 3u);
        D.tape_turn_deg = (int16_t)((arg >> 2) * 90u);
        D.tape_turn_next = next;
        break;
    case TA_BRANCH:         tagact_branch((branch_dir_t)arg); break;
    case TA_SEQ:            tagact_stop(); seq_request(arg); break;
    case TA_TAPE_STOP:      D.tape_stop_id = (int16_t)arg; break;
    case TA_ACS:            acs_report(arg, acs_station_get(0)); break;
    case TA_ACS_NUM:        acs_report(next, (int16_t)arg); break;
    case TA_OBS:            D.obs_use = (int16_t)arg; break;
    case TA_STATE:          seq_state_set(arg, next); break;
    case TA_STATION_RESET:  acs_station_reset(); break;
    case TA_ACT:
        if (act_start(arg) == 2) { log_event(LOG_ACT_NO_SLOT, (int16_t)arg, 0); }
        break;
    case TA_CALL:           (void)seq_call_once(arg); break;
    default: break;
    }
}

int tagact_run(uint16_t tag)
{
    int16_t was_stopping = D.flag_stopping;
    int16_t prev_len = D.stop_pulses;
    const tag_rule_t *r;
    uint16_t i, irq;
    int started;

    if (tune_active()) { return 0; }         /* 튜닝 주행은 태그를 쓰지 않는다 */
    r = rule_find(tag);
    if (r != 0) {
        for (i = 0; (i < 4u) && (r->act[i] != 0u); i++) {
            uint16_t second = has_second_word(r->act[i]);
            act_exec(r->act[i], (uint16_t)((second && (i < 3u)) ? r->act[i + 1u] : 0u));
            i = (uint16_t)(i + second);
        }
    }
    /* 이 태그로 시작된 정지만 거리를 건다. 레거시는 정지 중 태그마다 다시 걸어 정지 위치가 밀렸다.
     * shortcut: 방향별 정지 거리를 쓰는 타입을 옮길 때 사이트 선택으로 바꾼다 */
    started = (int)(D.flag_stopping && !was_stopping);
    if (started) { D.stop_pulses = P.stop_len; }
    s_display_tag = tag;
    irq = hal_irq_save();
    D.straight_cnt = 0;
    hal_irq_restore(irq);
    /* 이 태그로 새 정지가 시작될 때만 요청한다 */
    if (D.flag_stopping && (!was_stopping || (prev_len != D.stop_pulses))) { stop_request_tag(); }
    return started;
}

uint16_t tagact_display_tag(void) { return s_display_tag; }

uint16_t tagact_dry_run(uint16_t tag, uint16_t *acts, uint16_t cap)
{
    const tag_rule_t *r = rule_find(tag);
    uint16_t i, n = 0;

    if (r == 0) { return 0; }
    for (i = 0; (i < 4u) && (r->act[i] != 0u) && (n < cap); i++) { acts[n++] = r->act[i]; }
    return n;
}
