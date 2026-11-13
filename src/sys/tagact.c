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

float sqrtf(float x);

#define D agv_drive
#define P param_dv

#define TURN_STOP_MIN_VIN 3     /* 이보다 빠르면 회전 전에 정지 거리를 건다 */
#define TAPE_WINDOW_DEG   20.0f /* 정렬 회전이 새 라인을 찾는 목표각 앞뒤 범위 */

static const tag_rule_t *s_rules;
static const tag_packed_t *s_pk;   /* 압축 표. 0 = s_rules */
static tag_rule_t s_hit;           /* 압축 표에서 맞춰 보는 규칙을 펼친 칸. rule_find 가 다시 불리면 덮인다.
                                    * 돌려받은 포인터를 사용자 함수 호출 뒤에 쓰지 않는다 */
static uint16_t s_nrules, s_profile, s_display_tag;

void tagact_bind(const tag_rule_t *rules, uint16_t n, uint16_t profile)
{
    s_rules = rules;
    s_pk = 0;
    s_nrules = n;
    s_profile = profile;
}

void tagact_bind_packed(const tag_packed_t *t, uint16_t profile)
{
    s_rules = 0;
    s_pk = t;
    s_nrules = t->n;
    s_profile = profile;
}

/* 압축 규칙 i 를 s_hit 에 펼친다 */
static const tag_rule_t *pk_rule(uint16_t i)
{
    const tag_pack_t *p = &s_pk->rules[i];
    const tag_x_t *x1 = &s_pk->x[p->x & 0xFFu], *x2 = &s_pk->x[p->x >> 8];
    const tag_in_t *in = &s_pk->in[p->ai >> 8];
    const tag_act_t *a = &s_pk->act[p->ai & 0xFFu];
    uint16_t k;

    s_hit.tag = (uint16_t)(p->tp & 0x3FFu);
    s_hit.profile = (uint16_t)(p->tp >> 10);
    s_hit.cond = x1->cond;
    s_hit.cond_a = x1->a;
    s_hit.cond_b = x1->b;
    s_hit.cond2 = x2->cond;
    s_hit.cond2_a = x2->a;
    s_hit.cond2_b = x2->b;
    s_hit.in_on = in->on;
    s_hit.in_off = in->off;
    for (k = 0; k < 4u; k++) { s_hit.act[k] = a->act[k]; }
    return &s_hit;
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

/* 지금 자리에서 mm 만큼 더 가서 선다(태그 정지와 같은 정지). 이미 서는 중이면 거리를 바꾸지 않는다 */
void tagact_stop_after_mm(int32_t mm)
{
    float p;

    if (D.flag_stopping || (mm <= 0)) { return; }
    p = (float)mm * P.one_length_pulse;
    if (p > 32767.0f) { p = 32767.0f; }
    D.flag_stopping = 1;
    D.stop_pulses = (int16_t)p;
    stop_request_tag();
}

/* 시퀀스 MOVE_MM. 서 있을 때만 건다. run.c 가 출발시키고, 주행 첫 틱에 정지 거리를 걸어 STOP_REASON_MOVE(제동점까지 순항하는
 * 위치 정지)로 선다. 최고 속도 v(rpm): 0 → v → 0 이 거리 안에 들어가는 가장 빠른 값을 SPN~LLOW 로 묶는다.
 *   거리 d(rpm·틱) = 펄스 / (12/6000) − 끝 꼬리(STOP_TAIL_RPM_TICKS) 를
 *   d = A·v² + B·v 로 본다. A = (1/ACC + 1/DCC)/2(등가속·등감속), B = (ACC + DCC)/(2·2.4)(저크로 가감속도가 오르내리는 동안
 *   속도가 거의 그대로인 몫, 정지 램프 저크 2.4) + 15틱(순항 판정 여유 2틱 + 제동 여유 8 + 지연 5. stop.c braking_pulses·cruise_left).
 * v 가 커도 제동점은 정지 제어가 정한다(cruise_left). 이 값은 짧은 거리에서 가속만 하다 바로 서는 것을 막는 상한이다 */
void tagact_move_mm(int16_t mm)
{
    float pulses = (float)mm * P.one_length_pulse, d, a, b, v;
    int16_t acc = (P.acc > 0) ? P.acc : 1, dcc = (P.dcc > 0) ? P.dcc : 1;

    if ((mm < 1) || D.flag_run || D.flag_ready || D.flag_stopping || (D.move_on != 0)) { return; }
    if (pulses > 32767.0f) { pulses = 32767.0f; }
    d = pulses * (6000.0f / (float)STOP_ENCODER_PPR) - (float)STOP_TAIL_RPM_TICKS;
    a = 0.5f * (1.0f / (float)acc + 1.0f / (float)dcc);
    b = (float)(acc + dcc) * (float)STOP_JERK_DEN / (2.0f * (float)STOP_JERK_NUM) + 15.0f;
    v = (d > 0.0f) ? (sqrtf(b * b + 4.0f * a * d) - b) / (2.0f * a) : 0.0f;
    if (v > (float)P.llow_speed) { v = (float)P.llow_speed; }
    if (v < (float)P.min_speed)  { v = (float)P.min_speed; }
    D.move_pulses = (int16_t)(pulses + 0.5f);
    D.move_speed = (int16_t)v;
    D.move_on = 1;
}

/* 시퀀스 AGV_MOVE( 한계 mm, TAPE ). 서 있을 때만 건다. run.c 가 출발시키고 찾는 속도로 가다 다음 교차선에서 선다(run.c move_tape_tick).
 * 한계 거리는 D.move_pulses, 넘기면 오류 258 */
void tagact_move_tape(int16_t lim_mm)
{
    float pulses = (float)lim_mm * P.one_length_pulse;

    if ((lim_mm < 1) || D.flag_run || D.flag_ready || D.flag_stopping || (D.move_on != 0)) { return; }
    if (pulses > 32767.0f) { pulses = 32767.0f; }
    D.move_pulses = (int16_t)(pulses + 0.5f);
    D.move_on = 1;
    run_move_tape_start();
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
static int16_t s_turn_rpm;         /* 다음 tagact_turn 이 걸 회전 속도 rpm. 0 = 자동. 태그 둘째 칸에서 */

/* 태그 둘째 칸의 회전 속도 m/min → rpm. 0 = 자동. TNS 한계를 넘으면 한계, 회전 최저 속도 아래면 최저 속도 */
static int16_t turn_rpm(uint16_t next)
{
    int16_t v = (int16_t)(next >> TA_TURN_SPEED_SHIFT), low, up;

    if (v == 0) { return 0; }
    param_limit(P_SPEED_TURN, &low, &up);
    v = param_rpm((v > up) ? up : v);
    return (v < P.turn_min) ? P.turn_min : v;
}

/* 제자리 회전 90도 펄스. 튜닝 모드가 잰 회전 보정비가 있으면 건다(일반·정렬 회전 모두). 없으면 휠 간격 값 그대로 */
static int16_t turn_90(void)
{
    float s = tune_turn_scale();
    float q = (float)P.turn_90 * s + 0.5f;
    if (s == 1.0f) { return P.turn_90; }
    return (q > 16383.0f) ? 16383 : (int16_t)q;
}

/* 회전을 걸지 않는 상황: 회전 모드 0, 회전 중, 정지 거리 진행 중, 서 있음(시퀀스·튜닝 아니면) */
static int turn_refused(void)
{
    return (P.turn_mode <= 0) || (D.turn_pulses != 0) || (D.stop_pulses != 0)
           || (!seq_is_active() && !s_turn_from_stop && (D.vin_out <= TURN_STOP_MIN_VIN));
}

void tagact_turn(turn_dir_t d, uint16_t deg, dir_t fb, uint16_t autostart, uint16_t obs)
{
    if (turn_refused()) { return; }
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
    D.turn_rpm = s_turn_rpm;          /* 회전마다 새로 정한다. 시퀀스·정렬 호출은 0(자동) */
    tagact_forback(fb);
    D.turn_motion = (d == TURN_CW) ? MOTION_CW : MOTION_CCW;
    D.turn_90 = turn_90();
    if (deg == 90u)       { D.turn_pulses = D.turn_90; }
    else if (deg == 180u) { D.turn_pulses = (int16_t)(D.turn_90 * 2); }   /* 보정이 없으면 P.turn_180 과 같다 */
    else                  { D.turn_pulses = (int16_t)((D.turn_90 / 90) * (int16_t)deg); }
    D.turn_obs_use = (int16_t)obs;
    if (autostart == 1u) { D.wait_start = (P.turn_mode == 2) ? 1 : 0; }
    if (D.flag_stopping) { stop_request_tag(); }
}

void tagact_turn_next(turn_dir_t d, uint16_t deg, uint16_t next)
{
    s_turn_rpm = turn_rpm(next);
    tagact_turn(d, deg, (dir_t)(next & 0xFu), (uint16_t)((next >> 4) & 0xFu), (uint16_t)((next >> 8) & 0x7u));
    s_turn_rpm = 0;
}

void tagact_turn_align(turn_dir_t d, uint16_t deg, uint16_t next)
{
    int16_t before = D.turn_pulses;
    uint16_t fb = next & 0xFu;

    tagact_turn_next(d, deg, next);
    if ((before != 0) || (D.turn_pulses <= 0)) { return; }
    D.tape_total_pulse = D.turn_pulses;
    D.tape_target_cnt = D.tape_end_cnt = (float)D.turn_pulses;
    D.tape_sum_mt = D.tape_sum_tt = 0.0f;
    D.tape_plan_min = 0;
    D.tape_max = (int16_t)(((float)D.turn_90 / 90.0f) * TAPE_WINDOW_DEG);
    D.tape_slip_ratio = 1.0f;
    D.tape_lost_cnt = 0;
    D.tape_turned = 0;
    D.tape_origin_ok = 0;
    D.tape_side = 0;
    D.tape_fix_rev = 0;
    D.tape_fix_turned = 0;
    D.tape_stop_pos = D.tape_final_pos = 0;
    D.tape_fix_cnt = 0;
    D.tape_fix_end = -1;
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
        const tag_rule_t *r;
        if (s_pk != 0) {
            uint16_t tp = s_pk->rules[i].tp, pf = (uint16_t)(tp >> 10);
            if (((tp & 0x3FFu) != tag) || ((pf != 0u) && (pf != profile))) { continue; }
            r = pk_rule(i);
        }
        else { r = &s_rules[i]; }
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

/* 규칙의 정지 분류. 분류 2·3·4 가 있으면 그것(3·4 가 2 보다 앞, 같은 급이면 뒤의 것), 없으면 0.
 * *arg: 분류 2 는 10 mm 단위 거리, 분류 3·4 는 정지 뒤 시퀀스(그 칸이 0 이면 같은 규칙 TA_SEQ 의 시퀀스) */
static int16_t stop_class(const uint16_t *act, uint16_t *arg)
{
    int16_t cls = 0;
    uint16_t i, seq = 0;

    *arg = 0;
    for (i = 0; (i < 4u) && (act[i] != 0u); i = (uint16_t)(i + 1u + has_second_word(act[i]))) {
        uint16_t kind = (uint16_t)(TA_KIND(act[i]) << 8);
        int16_t c = (kind == TA_STOP_MM) ? 2 : (kind == TA_TAPE_STOP) ? TA_STOP_KIND_TAPE
                  : (kind == TA_TAPE_STOP_GD) ? TA_STOP_KIND_TAPE_GD : 0;
        if (kind == TA_SEQ) { seq = TA_ARG(act[i]); }
        if ((c != 0) && ((c >= 3) || (cls < 3))) {
            cls = c;
            *arg = TA_ARG(act[i]);
        }
    }
    if ((cls >= 3) && (*arg == 0u)) { *arg = seq; }
    return cls;
}

/* 분류 3·4 규칙에서는 태그 자리에서 서지 않는다(교차선에서 run.c 가 세운다) */
static void rule_stop(int16_t cls)
{
    if (cls < 3) { D.flag_stopping = 1; }
}

static void autostart(uint16_t sec, int16_t cls)
{
    uint16_t wait = (sec != 0u) ? sec : (uint16_t)param_get(P_AUTO_START_SEC);
    rule_stop(cls);
    if (wait != 0u) { run_request_restart(wait); }
}

/* 태그 보고. 대기('B')는 레거시 DA_ACS_Standby + 자동 재전송 켬 = ON: 정지, 번호 > 0 이면 자동 재전송 늘 켬,
 * TA_ACS_STANDBY_RESET 이면 먼저 스테이션 큐를 지운다. 분류 2·3·4 규칙이면 정지는 그 분류가 건다 */
static void tag_acs(uint16_t info, int16_t num, int16_t cls)
{
    if ((info & 0xFFu) == 'B') {
        run_standby(num, (int16_t)(RUN_STANDBY_AUTO | ((info & TA_ACS_STANDBY_RESET) ? RUN_STANDBY_RESET : 0)
                                   | ((cls != 0) ? RUN_STANDBY_NOSTOP : 0)));
    }
    else { acs_report(info & 0xFFu, num); }
}

#ifdef AGV_HOST_BUILD
void (*tagact_trace)(uint16_t code, uint16_t next);   /* 시험용: 실행하는 동작마다 알린다 */
#endif

static void act_exec(uint16_t code, uint16_t next, int16_t cls)
{
    uint16_t arg = TA_ARG(code);

#ifdef AGV_HOST_BUILD
    if (tagact_trace != 0) { tagact_trace(code, next); }
#endif

    switch ((uint16_t)(TA_KIND(code) << 8)) {
    case TA_STOP:
    case TA_STOP_MM:        rule_stop(cls); break;                /* 거리는 tagact_run 끝에서 */
    case TA_STOP_AUTOSTART: autostart(arg, cls); break;
    case TA_SPEED:          tagact_speed(arg); break;
    case TA_FORBACK:        tagact_forback((dir_t)arg); break;
    case TA_TURN:
        tagact_turn_next((turn_dir_t)(arg & 3u), (uint16_t)((arg >> 2) * 90u), next);
        break;
    case TA_TURN_ALIGN:
        tagact_turn_align((turn_dir_t)(arg & 3u), (uint16_t)((arg >> 2) * 90u), next);
        break;
    case TA_TURN_TAPE:      /* 일반 회전을 걸지 않는 상황과 정지 중에는 요청하지 않는다(찾기 속도가 정지와 부딪힌다) */
        if (!turn_refused() && !D.flag_stopping) {
            D.tape_turn_dir = (int16_t)(arg & 3u);
            D.tape_turn_deg = (int16_t)((arg >> 2) * 90u);
            D.tape_turn_next = next;
        }
        break;
    case TA_BRANCH:         tagact_branch((branch_dir_t)arg); break;
    case TA_SEQ:                                       /* 분류 3·4 규칙이면 교차선 정지 뒤 시퀀스(tagact_run 이 넣는다) */
        if (cls < 3) {
            rule_stop(cls);
            seq_request(arg);
        }
        break;
    case TA_ACS:            tag_acs(arg, acs_station_get(0), cls); break;
    case TA_ACS_NUM:
        tag_acs(next, (next & TA_ACS_STATION) ? acs_station_get(arg)
                      : (next & TA_ACS_AGV_MINUS) ? (int16_t)(param_get(P_AGV_ID) - (int16_t)arg) : (int16_t)arg, cls);
        break;
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
    uint16_t i, irq, carg = 0;
    int16_t cls = 0;
    int started;

    if (tune_active()) { return 0; }         /* 튜닝 주행은 태그를 쓰지 않는다 */
    r = rule_find(tag);
    if (r != 0) {
        uint16_t act[4];                      /* 사용자 함수가 tagact_run 을 다시 불러도 s_hit 가 덮이지 않게 복사 */
        for (i = 0; i < 4u; i++) { act[i] = r->act[i]; }
        cls = stop_class(act, &carg);         /* 분류 2·3·4 정지가 규칙 안 순서와 무관하게 이긴다 */
        for (i = 0; (i < 4u) && (act[i] != 0u); i++) {
            uint16_t second = has_second_word(act[i]);
            act_exec(act[i], (uint16_t)((second && (i < 3u)) ? act[i + 1u] : 0u), cls);
            i = (uint16_t)(i + second);
        }
    }
    if ((cls >= 3) && !was_stopping) {        /* 분류 3·4: 찾고 서는 것은 run.c(tape_stop). 진행 중인 정지는 깨지 않는다 */
        D.tape_stop_kind = cls;
        D.tape_stop_id = (int16_t)carg;
    }
    /* 이 태그로 시작된 정지만 거리를 건다. 레거시는 정지 중 태그마다 다시 걸어 정지 위치가 밀렸다.
     * 분류 2 는 SL 과 같은 단위(10 mm)·같은 정지에 거리만 다르다.
     * shortcut: 방향별 정지 거리를 쓰는 타입을 옮길 때 사이트 선택으로 바꾼다 */
    started = (int)(D.flag_stopping && !was_stopping);
    if (started) {
        D.stop_pulses = (cls == 2) ? (int16_t)((float)carg * P.one_length_pulse * 10.0f) : P.stop_len;
    }
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
