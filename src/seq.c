/* seq.c - 시퀀스·ACT 인터프리터. 표 형식: agv/seq.h, 설계: docs/03_시퀀스_DSL.md */
#include "agv/seq.h"
#include "agv/acs.h"
#include "agv/drive.h"
#include "agv/io.h"
#include "agv/log.h"
#include "agv/param.h"
#include "agv/tagact.h"
#include "agv/ui.h"

#define D agv_drive

#define SEQ_SCAN_STEPS   8     /* 한 스캔에 이어서 실행하는 스텝 수 */
#define LOW_BATTERY_DV   230   /* 저전압 판정과 같은 값 */

/* 한 실행 흐름. 시퀀스 1개 + ACT 슬롯 4개 */
typedef struct {
    seq_ctx_t c;
    const seq_step_t *tbl;
    uint16_t nsteps;
    uint16_t a, b;       /* 스텝 내부 카운터 */
    uint32_t owned;      /* ACT 가 켜 둔 출력 */
    uint16_t is_act;
    uint16_t active;
} seq_runner_t;

/* 평가 결과 */
enum { R_WAIT = 0, R_NEXT, R_JUMPED, R_ENDED };

int seq_request_flag[SEQ_MAX];

static const seq_site_t *s_site;
static uint16_t      s_nacts;      /* 금지 스텝이 있으면 0 으로 두어 ACT 를 쓰지 않는다 */
static uint16_t      s_profile;
static uint32_t      s_load_mask;
static seq_runner_t  s_seq;
static seq_runner_t  s_act[ACT_SLOTS];
static uint16_t      s_state[SEQ_STATE_COUNT];

uint16_t seq_profile(void)      { return s_profile; }
uint32_t seq_load_mask(void)    { return s_load_mask; }
int16_t  seq_load_site(void)    { return ((s_site != 0) && (s_site->load_fn != 0)) ? s_site->load_fn() : -1; }
int      seq_is_active(void)    { return s_seq.active != 0; }
uint16_t seq_active_id(void)    { return s_seq.active ? s_seq.c.id : (uint16_t)SEQ_NONE; }
uint16_t seq_active_step(void)  { return s_seq.c.step; }
uint16_t seq_state_get(uint16_t id)             { return (id < SEQ_STATE_COUNT) ? s_state[id] : 0; }
void     seq_state_set(uint16_t id, uint16_t v) { if (id < SEQ_STATE_COUNT) { s_state[id] = v; } }

static const seq_def_t *find_def(const seq_def_t *list, uint16_t n, uint16_t id)
{
    uint16_t i;
    for (i = 0; i < n; i++) {
        if ((list[i].id == id) && ((list[i].profile == 0) || (list[i].profile == s_profile))) {
            return &list[i];
        }
    }
    return 0;
}

static int16_t forbidden_in_act(uint16_t kind)
{
    return (kind == SK_SPEED) || (kind == SK_FORBACK) || (kind == SK_TURN) || (kind == SK_TURN_NOWAIT)
        || (kind == SK_ERROR_HOLD) || (kind == SK_END_RESTART) || (kind == SK_MOVE_MM) || (kind == SK_TURN_ALIGN)
        || (kind == SK_MOVE_TAPE);
}

void seq_pick_profile(void)
{
    uint16_t i;
    int16_t type = param_get(P_AGV_TYPE);
    int16_t id = param_get(P_AGV_ID);

    s_profile = 0;
    s_load_mask = 0;
    for (i = 0; (s_site != 0) && (i < s_site->nprofiles); i++) {
        const profile_row_t *row = &s_site->profiles[i];
        if (((row->agv_type == 0u) || (row->agv_type == (uint16_t)type)) && ((uint16_t)id >= row->id_from) && ((uint16_t)id <= row->id_to)) {
            s_profile = row->profile;
            s_load_mask = row->load_mask;
            break;
        }
    }
}

int seq_call_once(uint16_t idx)
{
    static seq_ctx_t ctx;
    if ((s_site == 0) || (idx >= s_site->ncalls)) { return 2; }
    return s_site->calls[idx](&ctx);
}

uint16_t seq_bind(const seq_site_t *site)
{
    uint16_t i, k;

    s_site = site;
    s_nacts = site->nacts;
    seq_pick_profile();
    for (i = 0; i < site->nacts; i++) {
        for (k = 0; k < site->acts[i].nsteps; k++) {
            if (forbidden_in_act(site->acts[i].steps[k].kind)) {
                s_nacts = 0;
                return site->acts[i].id;
            }
        }
    }
    return 0;
}

static void enter_step(seq_runner_t *r, uint16_t idx)
{
    r->c.step = idx;
    r->c.timer = 0;
    r->c.sub = 0;
    r->a = 0;
    r->b = 0;
}

static void runner_start(seq_runner_t *r, const seq_def_t *def, uint16_t is_act)
{
    uint16_t i;
    r->tbl = def->steps;
    r->nsteps = def->nsteps;
    r->c.id = def->id;
    for (i = 0; i < 4; i++) { r->c.scratch[i] = 0; }
    r->owned = 0;
    r->is_act = is_act;
    r->active = 1;
    enter_step(r, 0);
}

static void runner_end(seq_runner_t *r)
{
    r->active = 0;
    r->owned = 0;
    if (!r->is_act) { seq_request_flag[r->c.id] = 0; }
}

/* 시퀀스 오류는 action_err, ACT 오류는 주행을 세우는 stop_err 다 */
static void runner_fail(seq_runner_t *r, uint16_t code)
{
    if (r->is_act) { D.stop_err = (int16_t)code; }
    else           { D.action_err = (int16_t)code; }
    runner_end(r);
}

static void out_apply(seq_runner_t *r, uint32_t on, uint32_t off)
{
    io_out_masks(on, off);
    r->owned = (r->owned & ~off) | on;
}

uint16_t seq_mask_true(uint32_t v, uint32_t on, uint32_t off)
{
    uint32_t any = on & off;
    uint32_t all_on = on & ~any;
    uint32_t all_off = off & ~any;
    return ((v & all_on) == all_on) && ((v & all_off) == 0) && ((any == 0) || ((v & any) != 0));
}

uint16_t seq_xcond(uint16_t c, uint16_t a, uint16_t b, uint16_t tag)
{
    uint16_t kind = (uint16_t)(c & 0x7Fu);
    uint16_t slot = (uint16_t)(c >> 8);
    int16_t st = ((kind == XC_STATION) || (kind == XC_STATION_RUNNING) || (kind >= XC_STATION_MOD)) ? acs_station_get(slot) : 0;
    int16_t v;
    uint16_t ok;

    switch (kind) {
    case XC_NONE:            ok = 1; break;
    case XC_FORBACK_MODE:    ok = (uint16_t)(param_dv.forback_mode > 0); break;
    case XC_LOW_BATTERY:     ok = (uint16_t)(param_get(P_BAT_DV) < LOW_BATTERY_DV); break;
    case XC_FORWARD:         ok = (uint16_t)(D.backward == 0); break;
    case XC_BACKWARD:        ok = (uint16_t)(D.backward != 0); break;
    case XC_STATION_RUNNING:
    case XC_STATION:
        ok = (uint16_t)((st >= (int16_t)a) && (st <= (int16_t)b) && ((kind == XC_STATION) || !D.flag_stopping));
        break;
    case XC_CROSS:
        if (slot & 1u) { ok = (uint16_t)(D.cross_out_on || (!(slot & 2u) && D.cross_out_flag)); }
        else           { ok = (uint16_t)(D.cross_in_on || (!(slot & 2u) && D.cross_in_flag)); }
        break;
    case XC_ERR_EQ:          ok = (uint16_t)(D.action_err == (int16_t)a); break;
    case XC_PARAM_EQ:        ok = (uint16_t)(param_get(slot) == (int16_t)a); break;
    case XC_PARAM_GE:        ok = (uint16_t)(param_get(slot) >= (int16_t)a); break;
    case XC_PARAM_LE:        ok = (uint16_t)(param_get(slot) <= (int16_t)a); break;
    case XC_ID:
        v = param_get(P_AGV_ID);
        ok = (uint16_t)((v >= (int16_t)a) && (v <= (int16_t)b));
        break;
    case XC_KEY:             ok = (uint16_t)(ui_key() == a); break;
    case XC_STATION_MOD:     ok = (uint16_t)((a != 0u) && ((st % (int16_t)a) == (int16_t)b)); break;
    case XC_STATION_DIV:     ok = (uint16_t)((a != 0u) && ((st / (int16_t)a) == (int16_t)b)); break;
    case XC_TAG_MINUS_EQ_ST: ok = (uint16_t)((int16_t)(tag - a) == st); break;
    case XC_TAG_EQ_ST:       ok = (uint16_t)((int16_t)tag == st); break;
    default:                 return 0;
    }
    return (c & XC_NOT) ? (uint16_t)!ok : ok;
}

static uint16_t cond_true(const seq_step_t *s)
{
    return seq_mask_true(io_in_mask(), s->on, s->off);
}

/* WHEN·WAIT: 입력 조건 + 두 번째 "하나라도 ON" 묶음(out) + 확장 조건(xc, arg0, arg1) */
static uint16_t cond_full(const seq_step_t *s)
{
    uint32_t v = io_in_mask();
    return seq_mask_true(v, s->on, s->off) && ((s->out == 0) || ((v & s->out) != 0))
        && ((s->xc == 0) || seq_xcond(s->xc, s->arg0, s->arg1, 0));
}

static uint16_t cond_out_true(const seq_step_t *s)
{
    return seq_mask_true(io_out_mask(), s->on, s->off);
}

static void tick(seq_runner_t *r)
{
    if (r->c.timer != 0xFFFFu) { r->c.timer++; }
}

static uint16_t timed_out(const seq_runner_t *r, const seq_step_t *s)
{
    return (s->timeout != 0) && (r->c.timer > s->timeout);
}

/* 파라미터 arg0 x 단위 arg1 틱이 지났나. a = 단위 안 틱, b = 지난 단위 수라 16비트로 99분도 센다 */
static uint16_t param_time_over(seq_runner_t *r, const seq_step_t *s)
{
    int16_t v = param_get(s->arg0);

    r->a++;
    if (r->a >= s->arg1) {
        r->a = 0;
        if (r->b != 0xFFFFu) { r->b++; }
    }
    if (v < 0) { v = 0; }
    return (r->b > (uint16_t)v) || ((r->b == (uint16_t)v) && (r->a != 0));
}

/* 라벨은 분기할 때 표에서 찾는다. 없으면 표 오류로 끝낸다 */
static uint16_t jump(seq_runner_t *r, uint16_t label)
{
    uint16_t i;
    for (i = 0; i < r->nsteps; i++) {
        if ((r->tbl[i].kind == SK_LABEL) && (r->tbl[i].target == label)) {
            enter_step(r, i);
            return R_JUMPED;
        }
    }
    runner_fail(r, SEQ_ERR_TABLE);
    return R_ENDED;
}

/* 연속한 WHEN 묶음을 위에서 평가한다 */
static uint16_t eval_when(seq_runner_t *r)
{
    uint16_t i;
    tick(r);
    for (i = r->c.step; i < r->nsteps; i++) {
        const seq_step_t *s = &r->tbl[i];
        if (s->kind == SK_WHEN) {
            if (cond_full(s)) { return jump(r, s->target); }
        }
        else if (s->kind == SK_WHEN_STATE) {
            if (seq_state_get(s->arg0) == s->arg1) { return jump(r, s->target); }
        }
        else if (s->kind == SK_WHEN_TIMEOUT) {
            return (r->c.timer > s->timeout) ? jump(r, s->target) : (uint16_t)R_WAIT;
        }
        else {
            break;
        }
    }
    return R_WAIT;
}

/* WAIT·WAIT_STABLE. 조건이 먼저이고 타임아웃은 그다음이다 */
static uint16_t eval_wait_stable(seq_runner_t *r, const seq_step_t *s)
{
    uint16_t unmatched = (s->timeout & SEQ_TO_UNMATCHED) != 0;
    uint16_t limit = (uint16_t)(s->timeout & ~SEQ_TO_UNMATCHED);

    tick(r);
    if (cond_true(s)) {
        r->a++;
        if (r->a > s->arg0) { return R_NEXT; }
    }
    else {
        r->a = 0;
        r->b++;
    }
    if ((limit != 0) && ((unmatched ? r->b : r->c.timer) > limit)) { return jump(r, s->target); }
    return R_WAIT;
}

static uint16_t eval_pulse(seq_runner_t *r, const seq_step_t *s)
{
    if (r->b >= s->timeout) { return R_NEXT; }
    if (r->a == 0) { out_apply(r, s->out, 0); }
    if (r->a == s->arg0) { out_apply(r, 0, s->out); }
    r->a++;
    if (r->a >= (uint16_t)(s->arg0 + s->arg1)) {
        r->a = 0;
        r->b++;
        if (r->b >= s->timeout) {
            out_apply(r, 0, s->out);
            return R_NEXT;
        }
    }
    return R_WAIT;
}

static uint16_t eval_flicker(seq_runner_t *r, const seq_step_t *s)
{
    tick(r);
    if (cond_true(s)) {
        out_apply(r, 0, s->out);
        return R_NEXT;
    }
    if (timed_out(r, s)) {
        out_apply(r, 0, s->out);
        return jump(r, s->target);
    }
    if (r->a == 0) {
        r->b = (uint16_t)!r->b;
        if (r->b) { out_apply(r, s->out, 0); }
        else      { out_apply(r, 0, s->out); }
        r->a = s->arg0;
    }
    if (r->a > 0) { r->a--; }
    return R_WAIT;
}

/* 제한 시간은 응답 켜짐·꺼짐 대기 각각에 적용한다 */
static uint16_t eval_handshake(seq_runner_t *r, const seq_step_t *s)
{
    uint16_t in_on = (io_in_mask() & s->on) == s->on;

    tick(r);
    if (r->a == 0) {
        out_apply(r, s->out, 0);
        r->a = 1;
    }
    if (r->a == 1) {
        if (in_on) {
            out_apply(r, 0, s->out);
            r->a = 2;
            r->c.timer = 0;
            return R_WAIT;
        }
    }
    else if ((io_in_mask() & s->on) == 0) {
        return R_NEXT;
    }
    if (timed_out(r, s)) {
        out_apply(r, 0, s->out);
        return jump(r, s->target);
    }
    return R_WAIT;
}

static int16_t acs_number(const seq_step_t *s)
{
    if ((s->arg1 & 0xFF00u) == SEQ_NUM_STATION_K) { return acs_station_get(s->arg1 & 0xFFu); }
    switch (s->arg1) {
    case SEQ_NUM_STATION0:       return acs_station_get(0);
    case SEQ_NUM_STATION0_PLUS1: return (int16_t)(acs_station_get(0) + 1);
    case SEQ_NUM_AGV_ID:         return param_get(P_AGV_ID);
    default:                     return (int16_t)s->timeout;
    }
}

static uint16_t eval_turn(seq_runner_t *r, const seq_step_t *s)
{
    if (r->a == 0) {
        /* 셋째 칸은 태그 회전 둘째 칸과 같은 배치(장애물 b8~10, 속도 b11~15) */
        if (s->kind == SK_TURN_ALIGN) {
            int16_t busy = D.turn_pulses;
            tagact_turn_align_stopped((turn_dir_t)s->arg0, s->arg1, s->timeout);
            /* 정렬 회전이 거부되면(회전 모드 0, 회전·정지 진행 중 등) 다음 스텝으로 가지 않고 오류로 끝낸다 */
            if ((busy != 0) || !D.tape_align) {
                runner_fail(r, SEQ_ERR_TURN);
                return R_ENDED;
            }
        }
        else { tagact_turn_next((turn_dir_t)s->arg0, s->arg1, s->timeout); }
        r->a = 1;
        return R_WAIT;
    }
    if (s->kind == SK_TURN_NOWAIT) { return R_NEXT; }
    /* 오류가 선 틱에는 다음 스텝(출력)으로 가지 않는다. 이 틱 뒤 run_after_motor 가 시퀀스를 끝낸다 */
    if ((D.system_err != 0) || (D.stop_err != 0)) { return R_WAIT; }
    /* 정렬 회전은 정착·보정까지 끝나야 한다 */
    if ((s->kind == SK_TURN_ALIGN) && D.tape_align) { return R_WAIT; }
    /* 거부된 일반 회전은 다음 스캔에 바로 통과한다 */
    if ((D.turn_pulses > 0) || (D.turn_motion != 0) || D.flag_turn) { return R_WAIT; }
    /* 정렬이 실패로 끝났으면(tape_fix_end: 0 가운데에 섬. -1 선을 잡은 뒤 범위 초과, 1 보정 횟수 끝,
     * 2 보정 각 초과, 3 선 놓침, 4 다가온 쪽 모름) 다음 스텝으로 가지 않고 오류로 끝낸다 */
    if ((s->kind == SK_TURN_ALIGN) && (D.tape_fix_end != 0)) {
        runner_fail(r, ERR_GUIDE_SENSOR);
        return R_ENDED;
    }
    return R_NEXT;
}

static uint16_t eval_call(seq_runner_t *r, const seq_step_t *s)
{
    int ret;

    if (s->arg0 >= s_site->ncalls) {
        runner_fail(r, SEQ_ERR_TABLE);
        return R_ENDED;
    }
    tick(r);
    ret = s_site->calls[s->arg0](&r->c);
    if (ret == 1) { return R_NEXT; }
    if (ret == 2) {
        runner_fail(r, SEQ_ERR_CALL);
        return R_ENDED;
    }
    return R_WAIT;
}

static uint16_t eval_step(seq_runner_t *r, const seq_step_t *s)
{
    switch (s->kind) {
    case SK_END_RESTART:
        runner_end(r);
        run_request_restart(0);
        return R_ENDED;
    case SK_END_STOP:
        runner_end(r);
        return R_ENDED;
    case SK_END_ERROR:
        runner_fail(r, s->arg0);
        return R_ENDED;
    case SK_OUT:
        out_apply(r, s->on, s->off);
        return R_NEXT;
    case SK_WAIT:
        tick(r);
        if (cond_full(s)) { return R_NEXT; }
        return timed_out(r, s) ? jump(r, s->target) : (uint16_t)R_WAIT;
    case SK_WAIT_PARAM:
        tick(r);
        if (cond_full(s)) { return R_NEXT; }
        return (param_time_over(r, s) && (param_get(s->arg0) > 0)) ? jump(r, s->target) : (uint16_t)R_WAIT;
    case SK_DELAY_PARAM:
        return param_time_over(r, s) ? (uint16_t)R_NEXT : (uint16_t)R_WAIT;
    case SK_WAIT_STABLE:
        return eval_wait_stable(r, s);
    case SK_WAIT_OUT:
        return cond_out_true(s) ? (uint16_t)R_NEXT : (uint16_t)R_WAIT;
    case SK_DELAY:
        tick(r);
        return (r->c.timer > s->arg0) ? (uint16_t)R_NEXT : (uint16_t)R_WAIT;
    case SK_OUT_PULSE:
        return eval_pulse(r, s);
    case SK_OUT_FLICKER:
        return eval_flicker(r, s);
    case SK_HANDSHAKE:
        return eval_handshake(r, s);
    case SK_ACS:
        acs_report(s->arg0, acs_number(s));
        return R_NEXT;
    case SK_ACS_WAIT_ACK:
        tick(r);
        if (acs_pending() == 0) { return R_NEXT; }
        return timed_out(r, s) ? jump(r, s->target) : (uint16_t)R_WAIT;
    case SK_ACS_STATION:
        acs_station_set(s->arg0, (int16_t)s->arg1, s->timeout);
        return R_NEXT;
    case SK_STATION_RESET:
        acs_station_reset();
        return R_NEXT;
    case SK_SPEED:
        tagact_speed((s->arg0 >= SEQ_SPD_HIGH) ? (uint16_t)(s->arg0 - SEQ_SPD_HIGH) : s->arg0);
        return R_NEXT;
    case SK_FORBACK:
        tagact_forback((dir_t)s->arg0);
        return R_NEXT;
    case SK_TURN:
    case SK_TURN_NOWAIT:
    case SK_TURN_ALIGN:
        return eval_turn(r, s);
    case SK_MOVE_MM:            /* 회전처럼 시퀀스를 살려 둔 채 run.c 가 이동한다. 도착해 서야(MOVE_DONE) 다음 스텝.
                                 * 지금 못 걸면(회전·정지 중) 다음 스캔에 다시 건다. 범위 밖 거리는 시퀀스 오류 */
    case SK_MOVE_TAPE:          /* 교차선까지. 못 찾아 오류(258)로 서면 run_after_motor 가 끝낸다 */
        if (r->a == 0) {
            int16_t ok = (s->kind == SK_MOVE_MM) ? tagact_move_mm((int16_t)s->arg0) : tagact_move_tape((int16_t)s->arg0, (int16_t)s->arg1);
            if (ok < 0) {
                runner_fail(r, SEQ_ERR_TABLE);
                return R_ENDED;
            }
            r->a = (uint16_t)ok;
            return R_WAIT;
        }
        if (D.move_on == MOVE_DONE) {
            D.move_on = 0;
            return R_NEXT;
        }
        /* 도착 전에 다른 원인(명령 정지 등)으로 섰다: 통과하지 않고 실패한 접근(277)으로 세운다. run_after_motor 가 시퀀스를 끝낸다 */
        if (D.move_on == MOVE_FAIL) {
            D.move_on = 0;
            D.stop_err = ERR_STOP_APPROACH;
        }
        return R_WAIT;
    case SK_OBS:
        D.obs_use = (int16_t)s->arg0;
        return R_NEXT;
    case SK_SET_STATE:
        seq_state_set(s->arg0, s->arg1);
        return R_NEXT;
    case SK_WAIT_STATE:
        return (seq_state_get(s->arg0) == s->arg1) ? (uint16_t)R_NEXT : (uint16_t)R_WAIT;
    case SK_GOTO:
        return jump(r, s->target);
    case SK_WHEN:
    case SK_WHEN_STATE:
    case SK_WHEN_TIMEOUT:
        return eval_when(r);
    case SK_ERROR_HOLD:
        D.action_err = (int16_t)s->arg0;
        return cond_true(s) ? jump(r, s->target) : (uint16_t)R_WAIT;
    case SK_ACT_START:
        if (act_start(s->arg0) == 2) { log_event(LOG_ACT_NO_SLOT, (int16_t)s->arg0, 0); }
        return R_NEXT;
    case SK_WAIT_ACT:
        return act_is_busy(s->arg0) ? (uint16_t)R_WAIT : (uint16_t)R_NEXT;
    case SK_CALL:
        return eval_call(r, s);
    default:
        return R_NEXT;
    }
}

static void runner_scan(seq_runner_t *r)
{
    uint16_t budget = SEQ_SCAN_STEPS;

    while (r->active && (budget > 0)) {
        const seq_step_t *s;
        uint16_t result;

        if (r->c.step >= r->nsteps) {       /* 마지막 스텝을 지나면 끝 */
            runner_end(r);
            return;
        }
        s = &r->tbl[r->c.step];
        if (s->kind == SK_LABEL) {          /* 라벨은 실행 시간이 없다 */
            enter_step(r, (uint16_t)(r->c.step + 1));
            continue;
        }
        result = eval_step(r, s);
        if (result == R_WAIT) { return; }
        if (result == R_NEXT) { enter_step(r, (uint16_t)(r->c.step + 1)); }
        budget--;
    }
}

/* ---- 시퀀스 ---- */

void seq_request(uint16_t id)
{
    if ((id > SEQ_NONE) && (id < SEQ_MAX)) { seq_request_flag[id] = 1; }
}

void seq_cancel(uint16_t id)
{
    if (id >= SEQ_MAX) { return; }
    seq_request_flag[id] = 0;
    if (s_seq.active && (s_seq.c.id == id)) {
        s_seq.active = 0;
        io_seq_outputs_off();
    }
}

void seq_run(void)
{
    if (!s_seq.active && (s_site != 0)) {
        uint16_t i, id;
        for (i = 0; i < s_site->nseqs; i++) {
            const seq_def_t *def = &s_site->seqs[i];
            if (((def->profile == 0) || (def->profile == s_profile)) && seq_request_flag[def->id]) {
                runner_start(&s_seq, def, 0);
                break;
            }
        }
        if (!s_seq.active) {
            for (id = SEQ_NONE + 1; id < SEQ_MAX; id++) { seq_request_flag[id] = 0; }
        }
    }
    if (s_seq.active) { runner_scan(&s_seq); }
}

void seq_abort_all(void)
{
    uint16_t id;
    for (id = 0; id < SEQ_MAX; id++) { seq_request_flag[id] = 0; }
    s_seq.active = 0;
    /* 위험 감속 표시도 시퀀스 초기화와 함께 내린다 */
    D.risk_stop = 0;
    io_seq_outputs_off();
}

/* ---- ACT ---- */

int act_is_busy(uint16_t id)
{
    uint16_t i;
    for (i = 0; i < ACT_SLOTS; i++) {
        if (s_act[i].active && (s_act[i].c.id == id)) { return 1; }
    }
    return 0;
}

int act_start(uint16_t id)
{
    uint16_t i;
    const seq_def_t *def;

    if (act_is_busy(id)) { return 1; }
    if (s_site == 0) { return 2; }
    def = find_def(s_site->acts, s_nacts, id);
    if (def == 0) { return 2; }
    for (i = 0; i < ACT_SLOTS; i++) {
        if (!s_act[i].active) {
            runner_start(&s_act[i], def, 1);
            return 0;
        }
    }
    return 2;
}

void act_abort_all(void)
{
    uint16_t i;
    for (i = 0; i < ACT_SLOTS; i++) {
        if (s_act[i].active) {
            io_out_masks(0, s_act[i].owned);
            s_act[i].active = 0;
            s_act[i].owned = 0;
        }
    }
}

void act_run(void)
{
    uint16_t i;

    if ((D.system_err != 0) || (D.stop_err != 0)) {
        act_abort_all();
        return;
    }
    for (i = 0; i < ACT_SLOTS; i++) {
        if (s_act[i].active) { runner_scan(&s_act[i]); }
    }
}
