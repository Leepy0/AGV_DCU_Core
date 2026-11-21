/* seq.c - 시퀀스·ACT 인터프리터. 표 형식: agv/seq.h, 설계: docs/03_시퀀스_DSL.md */
#include "agv/seq.h"
#include "agv/acs.h"
#include "agv/drive.h"
#include "agv/io.h"
#include "agv/log.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/tagact.h"
#include "agv/ui.h"

#define D agv_drive

#define SEQ_SCAN_STEPS   8     /* 한 스캔에 이어서 실행하는 스텝 수 */
#define LOW_BATTERY_DV   230   /* 저전압 판정과 같은 값 */

/* 한 실행 흐름. 시퀀스 1개 + ACT 슬롯 4개 */
typedef struct {
    seq_ctx_t c;
    union {
        const seq_step_t *tbl;                  /* 펼친 표 */
        struct { uint16_t base, at; } pk;       /* 압축 표: 첫 스텝과 지금 스텝(c.step) 머리의 w 안 자리 */
    } t;
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
static uint16_t      s_profile;    /* 고른 행의 profile 칸 그대로(번호 | PF_F_*). RAM 을 늘리지 않으려고 한 칸에 둔다 */
#define PF_NUM       (s_profile & 0xFFu)
static uint32_t      s_load_mask;
static seq_runner_t  s_seq;
static seq_runner_t  s_act[ACT_SLOTS];
uint16_t             seq_state[SEQ_STATE_COUNT];
static uint16_t      s_belt[SEQ_BELTS];   /* 건 벨트 정지 조건: 시퀀스 표의 SK_BELT_UNTIL 스텝 번호(압축 표는 머리 자리) + 1, 0 빈 칸 */
static seq_step_t    s_buf;               /* 압축 표에서 푼 스텝. 한 번에 하나: 평가 중인 스텝, WHEN 묶음 훑기, 벨트 조건 */

uint16_t seq_profile(void)      { return PF_NUM; }
uint16_t seq_profile_flags(void) { return s_profile & 0xFF00u; }
uint32_t seq_load_mask(void)    { return s_load_mask; }
int16_t  seq_load_site(void)    { return ((s_site != 0) && (s_site->load_fn != 0)) ? s_site->load_fn() : -1; }
int      seq_is_active(void)    { return s_seq.active != 0; }
uint16_t seq_active_id(void)    { return s_seq.active ? s_seq.c.id : (uint16_t)SEQ_NONE; }
uint16_t seq_active_step(void)  { return s_seq.c.step; }
uint16_t seq_state_get(uint16_t id)             { return (id < SEQ_STATE_COUNT) ? seq_state[id] : 0; }
void     seq_state_set(uint16_t id, uint16_t v) { if (id < SEQ_STATE_COUNT) { seq_state[id] = v; } }

/* ---- 표 읽기: 펼친 표(seq_def_t·seq_step_t)와 압축 표(site->packed, agv/seq.h)는 여기서만 가른다 ---- */

#define PK (s_site->packed)

uint16_t seq_unpack(const uint16_t *p, seq_step_t *s)
{
    uint16_t m = p[0], k = 1;
#define TAKE(b)  ((m & (1u << (b))) ? p[k++] : 0u)
    s->kind = TAKE(0);
    s->target = TAKE(1);
    s->arg0 = TAKE(2);
    s->arg1 = TAKE(3);
    s->timeout = TAKE(4);
    s->xc = TAKE(5);
    s->on = TAKE(6);
    s->on |= (uint32_t)TAKE(7) << 16;
    s->off = TAKE(8);
    s->off |= (uint32_t)TAKE(9) << 16;
    s->out = TAKE(10);
    s->out |= (uint32_t)TAKE(11) << 16;
#undef TAKE
    return k;
}

/* 압축 스텝 p 의 칸 b 하나(그 칸이 0 이면 0). 풀지 않는다 */
static uint16_t pk_slot(const uint16_t *p, uint16_t b)
{
    uint16_t m = p[0], k = 1, i;
    if ((m & (1u << b)) == 0u) { return 0; }
    for (i = 0; i < b; i++) {
        if (m & (1u << i)) { k++; }
    }
    return p[k];
}

/* 스텝 i(압축 표는 머리 자리 at). 펼친 표는 표 자리를, 압축 표는 s_buf 에 풀어 돌려준다 */
static const seq_step_t *step_read(const seq_step_t *tbl, uint16_t i, uint16_t at)
{
    if (PK == 0) { return &tbl[i]; }
    (void)seq_unpack(&PK->w[at], &s_buf);
    return &s_buf;
}

/* 압축 표에서 at 다음 스텝의 머리 자리. 펼친 표는 0 */
static uint16_t step_next_at(uint16_t at)
{
    return (PK != 0) ? (uint16_t)(at + SEQ_PK_LEN(PK->w[at])) : 0u;
}

/* 스텝 i 의 종류와 target 만(라벨 훑기). s_buf 를 쓰지 않는다 */
static void step_head(const seq_step_t *tbl, uint16_t i, uint16_t at, uint16_t *kind, uint16_t *target)
{
    if (PK == 0) {
        *kind = tbl[i].kind;
        *target = tbl[i].target;
    }
    else {
        const uint16_t *p = &PK->w[at];
        *kind = (p[0] & 1u) ? p[1] : 0u;
        *target = (p[0] & 2u) ? p[1u + (p[0] & 1u)] : 0u;
    }
}

static uint16_t map_count(uint16_t is_act)
{
    if (is_act) { return s_nacts; }
    return (PK != 0) ? PK->nseqs : s_site->nseqs;
}

/* 대응표(is_act 면 ACT) i 번째 줄의 프로파일과 ID */
static void map_get(uint16_t is_act, uint16_t i, uint16_t *profile, uint16_t *id)
{
    if (PK != 0) {
        const seq_pmap_t *m = is_act ? &PK->acts[i] : &PK->seqs[i];
        *profile = (uint16_t)(m->pid & 0xFFu);
        *id = (uint16_t)(m->pid >> 8);
    }
    else {
        const seq_def_t *d = is_act ? &s_site->acts[i] : &s_site->seqs[i];
        *profile = d->profile;
        *id = d->id;
    }
}

/* ACT 대응표에서 id 의 줄. 없으면 0xFFFF */
static uint16_t find_act(uint16_t id)
{
    uint16_t i, pf, d;
    for (i = 0; i < s_nacts; i++) {
        map_get(1, i, &pf, &d);
        if ((d == id) && ((pf == 0) || (pf == PF_NUM))) { return i; }
    }
    return 0xFFFFu;
}

static int16_t forbidden_in_act(uint16_t kind)
{
    return (kind == SK_SPEED) || (kind == SK_FORBACK) || (kind == SK_TURN) || (kind == SK_TURN_NOWAIT)
        || (kind == SK_ERROR_HOLD) || (kind == SK_END_RESTART) || (kind == SK_MOVE_MM) || (kind == SK_TURN_ALIGN)
        || (kind == SK_MOVE_TAPE) || (kind == SK_BELT_UNTIL);
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
    uint16_t i, k, n;

    s_site = site;
    s_nacts = (site->packed != 0) ? site->packed->nacts : site->nacts;
    n = s_nacts;
    seq_pick_profile();
    for (i = 0; i < n; i++) {
        const seq_step_t *tbl = (PK != 0) ? 0 : site->acts[i].steps;
        uint16_t ns = (PK != 0) ? PK->acts[i].n : site->acts[i].nsteps;
        uint16_t at = (PK != 0) ? PK->acts[i].off : 0u;
        for (k = 0; k < ns; k++) {
            uint16_t kind, target, pf, id;
            step_head(tbl, k, at, &kind, &target);
            if (forbidden_in_act(kind)) {
                map_get(1, i, &pf, &id);
                s_nacts = 0;
                return id;
            }
            at = step_next_at(at);
        }
    }
    return 0;
}

/* at: 압축 표의 그 스텝 머리 자리(펼친 표는 쓰지 않는다) */
static void enter_step(seq_runner_t *r, uint16_t idx, uint16_t at)
{
    r->c.step = idx;
    if (PK != 0) { r->t.pk.at = at; }
    r->c.timer = 0;
    r->c.sub = 0;
    r->a = 0;
    r->b = 0;
}

/* 대응표(is_act 면 ACT) i 번째 줄로 시작한다 */
static void runner_start(seq_runner_t *r, uint16_t is_act, uint16_t i)
{
    uint16_t k, profile;
    if (PK != 0) {
        const seq_pmap_t *m = is_act ? &PK->acts[i] : &PK->seqs[i];
        r->t.pk.base = m->off;
        r->nsteps = m->n;
    }
    else {
        const seq_def_t *def = is_act ? &s_site->acts[i] : &s_site->seqs[i];
        r->t.tbl = def->steps;
        r->nsteps = def->nsteps;
    }
    map_get(is_act, i, &profile, &r->c.id);
    for (k = 0; k < 4; k++) { r->c.scratch[k] = 0; }
    r->owned = 0;
    r->is_act = is_act;
    r->active = 1;
    enter_step(r, 0, (PK != 0) ? r->t.pk.base : 0u);
}

/* 벨트 정지 조건을 모두 지운다(출력은 그대로) */
static void belt_clear(void)
{
    uint16_t i;
    for (i = 0; i < SEQ_BELTS; i++) { s_belt[i] = 0; }
}

static void runner_end(seq_runner_t *r)
{
    r->active = 0;
    r->owned = 0;
    if (!r->is_act) {
        seq_request_flag[r->c.id] = 0;
        belt_clear();
    }
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

/* ---- 벨트 정지 조건(SK_BELT_UNTIL). 칸은 시퀀스 표의 스텝을 가리킨다: 벨트 출력 = on | off, 조건 = out · arg0·arg1 ---- */

static const seq_step_t *belt_step(uint16_t i) { return step_read(s_seq.t.tbl, (uint16_t)(s_belt[i] - 1u), (uint16_t)(s_belt[i] - 1u)); }

/* 벨트 i 의 출력(on | off). s_buf 를 쓰지 않는다: 시퀀스 출력 스텝을 평가하는 중에 부른다 */
static uint32_t belt_outs(uint16_t i)
{
    if (PK != 0) {
        const uint16_t *p = &PK->w[s_belt[i] - 1u];
        return ((uint32_t)pk_slot(p, 6) | ((uint32_t)pk_slot(p, 7) << 16)) | ((uint32_t)pk_slot(p, 8) | ((uint32_t)pk_slot(p, 9) << 16));
    }
    return belt_step(i)->on | belt_step(i)->off;
}

/* 조건이 참인 벨트의 출력 둘을 끄고 칸을 비운다 */
static void belt_check(void)
{
    uint16_t i;
    for (i = 0; i < SEQ_BELTS; i++) {
        if (s_belt[i] != 0u) {
            const seq_step_t *s = belt_step(i);
            if (seq_mask_true(io_in_mask(), s->out, (uint32_t)s->arg0 | ((uint32_t)s->arg1 << 16))) {
                out_apply(&s_seq, 0, s->on | s->off);
                s_belt[i] = 0;
            }
        }
    }
}

/* 시퀀스의 출력 스텝이 건 벨트의 출력을 바꾸면 그 벨트 조건을 지운다(BELT_OFF·BELT_ON 을 다시 실행) */
static void belt_drop(uint32_t m)
{
    uint16_t i;
    for (i = 0; i < SEQ_BELTS; i++) {
        if ((s_belt[i] != 0u) && ((belt_outs(i) & m) != 0u)) { s_belt[i] = 0; }
    }
}

/* 켜고 빈 칸에 조건을 건다. 이미 참이면 이 틱에 끈다(끝 센서를 같은 틱에 보던 사용자 함수와 같다) */
static uint16_t belt_until(seq_runner_t *r, const seq_step_t *s)
{
    uint16_t i;
    belt_drop(s->on | s->off);
    out_apply(r, s->on, s->off);
    for (i = 0; (i < SEQ_BELTS) && (s_belt[i] != 0u); i++) {}
    if (i == SEQ_BELTS) {
        runner_fail(r, SEQ_ERR_TABLE);
        return R_ENDED;
    }
    s_belt[i] = (uint16_t)(((PK != 0) ? r->t.pk.at : r->c.step) + 1u);
    belt_check();
    return R_NEXT;
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
    int16_t st = ((kind == XC_STATION) || (kind == XC_STATION_RUNNING) || ((kind >= XC_STATION_MOD) && (kind <= XC_TAG_NUM))) ? acs_station_get(slot) : 0;
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
    case XC_PARAM_EQ:
        v = param_get(slot);
        ok = (uint16_t)(((b != 0u) ? param_digit(v, (int16_t)b) : v) == (int16_t)a);
        break;
    case XC_PARAM_GE:        ok = (uint16_t)(param_get(slot) >= (int16_t)a); break;
    case XC_PARAM_LE:        ok = (uint16_t)(param_get(slot) <= (int16_t)a); break;
    case XC_ID:
        v = param_get(P_AGV_ID);
        ok = (uint16_t)((v >= (int16_t)a) && (v <= (int16_t)b));
        break;
    case XC_KEY:             ok = (uint16_t)(ui_key() == a); break;
    case XC_STATION_MOD:     ok = (uint16_t)((a != 0u) && ((st % (int16_t)a) == (int16_t)b)); break;
    case XC_STATION_DIV:     ok = (uint16_t)((a != 0u) && ((st / (int16_t)a) == (int16_t)b)); break;
    case XC_TAG_NUM:         ok = (uint16_t)((int16_t)tag == (int16_t)(((b != 0u) ? (st % (int16_t)b) : st) + (int16_t)a)); break;
    case XC_STATE:
        v = (int16_t)seq_state_get(slot);
        ok = (uint16_t)(((uint16_t)v >= a) && ((uint16_t)v <= b));
        break;
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

/* 유지·누적 셈의 한도(SK_WAIT_STABLE·SK_WHEN_STABLE). 단위(arg1 상위 8비트)가 있으면 파라미터 arg0 x 단위를 부를 때마다 읽는다.
 * 음수는 0, 65534 로 묶는다(셈이 한도를 넘는 틱에 통과하므로 16비트 셈이 넘치지 않는다) */
static uint16_t hold_limit(const seq_step_t *s)
{
    uint16_t unit = SEQ_HOLD_UNIT(s->arg1);
    int16_t v;
    uint32_t t;

    if (unit == 0u) { return s->arg0; }
    v = param_get(s->arg0);
    t = (v > 0) ? (uint32_t)v * unit : 0UL;
    return (t > 65534UL) ? 65534u : (uint16_t)t;
}

/* 라벨은 분기할 때 표에서 찾는다. 없으면 표 오류로 끝낸다 */
static uint16_t jump(seq_runner_t *r, uint16_t label)
{
    uint16_t i, at = (PK != 0) ? r->t.pk.base : 0u;
    for (i = 0; i < r->nsteps; i++) {
        uint16_t kind, target;
        step_head(r->t.tbl, i, at, &kind, &target);
        if ((kind == SK_LABEL) && (target == label)) {
            enter_step(r, i, at);
            return R_JUMPED;
        }
        at = step_next_at(at);
    }
    runner_fail(r, SEQ_ERR_TABLE);
    return R_ENDED;
}

/* 연속한 WHEN 묶음을 위에서 평가한다. 앞 가지가 참이면 그 틱에 가고 뒤 가지(유지 셈)는 보지 않는다 */
static uint16_t eval_when(seq_runner_t *r)
{
    uint16_t i, h = 0, at = (PK != 0) ? r->t.pk.at : 0u;
    tick(r);
    for (i = r->c.step; i < r->nsteps; i++) {
        const seq_step_t *s = step_read(r->t.tbl, i, at);
        at = step_next_at(at);
        if (s->kind == SK_WHEN) {
            if (cond_full(s)) { return jump(r, s->target); }
        }
        else if (s->kind == SK_WHEN_OUT) {
            if (cond_out_true(s)) { return jump(r, s->target); }
        }
        else if (s->kind == SK_WHEN_STATE) {
            if (seq_state_get(s->arg0) == s->arg1) { return jump(r, s->target); }
        }
        else if (s->kind == SK_WHEN_STABLE) {
            /* 유지 가지: 셈은 묶음 안 첫째 a, 둘째 b(묶음에 들어올 때 0). 셋째는 표 오류 */
            uint16_t *cnt = (h == 0u) ? &r->a : &r->b;
            if (h++ >= 2u) {
                runner_fail(r, SEQ_ERR_TABLE);
                return R_ENDED;
            }
            if (cond_true(s)) {
                (*cnt)++;
                if (*cnt > hold_limit(s)) { return jump(r, s->target); }
            }
            else if (!(s->arg1 & SEQ_HOLD_ACCUM)) { *cnt = 0; }
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

/* WAIT·WAIT_STABLE·WAIT_ACCUM. 조건이 먼저이고 타임아웃은 그다음이다. 누적(SEQ_HOLD_ACCUM)은 거짓인 틱에 셈을 그대로 둔다 */
static uint16_t eval_wait_stable(seq_runner_t *r, const seq_step_t *s)
{
    uint16_t unmatched = (s->timeout & SEQ_TO_UNMATCHED) != 0;
    uint16_t limit = (uint16_t)(s->timeout & ~SEQ_TO_UNMATCHED);

    tick(r);
    if (cond_true(s)) {
        r->a++;
        if (r->a > hold_limit(s)) { return R_NEXT; }
    }
    else {
        if (!(s->arg1 & SEQ_HOLD_ACCUM)) { r->a = 0; }
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

/* 점멸 스텝의 주기·출력 칸. 조건에 상태 낱말이 없으면(xc == 0) arg0 = 주기, out = 출력 비트(예전 배치 그대로).
 * 상태 낱말이 있으면(xc != 0) arg0·arg1 이 상태 인자라 둘 다 out 한 칸에 넣는다: 주기 << 8 | (출력 번호 + 1, 0 = 출력 없음).
 * 그래서 점멸은 둘째 "하나라도 ON"(out)을 어느 쪽으로도 받지 못한다(매크로가 컴파일 오류로 막는다) */
static void flk_cells(const seq_step_t *s, uint16_t *period, uint32_t *outm)
{
    if (s->xc == 0u) {
        *period = s->arg0;
        *outm = s->out;
    }
    else {
        *period = (uint16_t)(s->out >> 8);
        *outm = ((s->out & 0xFFu) != 0u) ? (1UL << ((s->out & 0xFFu) - 1u)) : 0UL;
    }
}

/* 점멸 끝 조건: 입력 조건 + 확장 조건(상태 낱말). cond_full 과 달리 out 칸은 출력이라 둘째 묶음을 보지 않는다 */
static uint16_t flk_cond(const seq_step_t *s)
{
    return seq_mask_true(io_in_mask(), s->on, s->off) && ((s->xc == 0) || seq_xcond(s->xc, s->arg0, s->arg1, 0));
}

static uint16_t eval_flicker(seq_runner_t *r, const seq_step_t *s)
{
    uint16_t period;
    uint32_t outm;

    flk_cells(s, &period, &outm);
    tick(r);
    if (flk_cond(s)) {
        out_apply(r, 0, outm);
        return R_NEXT;
    }
    if (timed_out(r, s)) {
        out_apply(r, 0, outm);
        return jump(r, s->target);
    }
    if (r->a == 0) {
        r->b = (uint16_t)!r->b;
        if (r->b) { out_apply(r, outm, 0); }
        else      { out_apply(r, 0, outm); }
        r->a = period;
    }
    if (r->a > 0) { r->a--; }
    return R_WAIT;
}

/* 출력 켬, 응답 켜짐 대기, 응답이 켜지면 바로 다음 스텝(v0.58.0). 출력은 끄지 않는다: 끄는 것은 표가 정한다(OUT_OFF 등).
 * 제한 시간·라벨은 응답 켜짐 대기에 적용하고, 제한을 넘어도 출력은 그대로 둔다(라벨 쪽 스텝이 정한다).
 * 응답이 이미 켜져 있으면 켠 같은 틱에 끝난다 */
static uint16_t eval_handshake(seq_runner_t *r, const seq_step_t *s)
{
    tick(r);
    if (r->a == 0) {
        out_apply(r, s->out, 0);
        r->a = 1;
    }
    if ((io_in_mask() & s->on) == s->on) { return R_NEXT; }
    if (timed_out(r, s)) { return jump(r, s->target); }
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

/* 주행 스텝(회전·이동)의 실패: 지워지지 않는 오류(리셋까지 남는다)로 차를 세우고 표시·보고한다. 다음 스텝으로 가지 않고,
 * 이 틱 뒤 run_after_motor 가 시퀀스를 끝낸다(run_seq_init: 출력 정리 포함). runner_fail 의 action_err 는 틱마다 지워지고
 * 280 이상은 시퀀스 정리도 안 불러 출력이 켜진 채 조용히 끝났다 */
static uint16_t drive_fail(seq_runner_t *r, uint16_t code)
{
    if (code == ERR_GUIDE_SENSOR) { D.system_err = (int16_t)code; }   /* 가이드 센서 오류는 다른 경로(못 찾음 258)와 같은 칸 */
    else                          { D.stop_err = (int16_t)code; }
    r->a = 1;
    return R_WAIT;
}

static uint16_t eval_turn(seq_runner_t *r, const seq_step_t *s)
{
    if (r->a == 0) {
        /* 셋째 칸은 태그 회전 둘째 칸과 같은 배치(장애물 b8~10, 속도 b11~15) */
        if (s->kind == SK_TURN_ALIGN) {
            int16_t busy = D.turn_pulses;
            tagact_turn_align_stopped((turn_dir_t)s->arg0, s->arg1, s->timeout);
            /* 정렬 회전이 거부되면(회전 모드 0, 회전·정지 진행 중 등) 다음 스텝으로 가지 않고 오류로 끝낸다 */
            if ((busy != 0) || !D.tape_align) { return drive_fail(r, SEQ_ERR_TURN); }
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
    if ((s->kind == SK_TURN_ALIGN) && (D.tape_fix_end != 0)) { return drive_fail(r, ERR_GUIDE_SENSOR); }
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
        if (!r->is_act) { belt_drop(s->on | s->off); }
        out_apply(r, s->on, s->off);
        return R_NEXT;
    case SK_BELT_UNTIL:
        return belt_until(r, s);
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
        /* 대기 보고(STANDBY)는 태그와 같이 보고 + 자동 재전송 + 대기 번호 표시(run_standby). 정지는 걸지 않는다: 시퀀스는 이미 선 뒤에
         * 돌고(run_seq 가 정지·주행 중이 아닐 때만 부른다), ACT 는 주행을 멈추지 않는 동작이다. 정지를 걸면 시퀀스를 돌리는 스캔만 멈춘다 */
        if (s->arg0 == (uint16_t)'B') { run_standby(acs_number(s), RUN_STANDBY_AUTO | RUN_STANDBY_NOSTOP | RUN_STANDBY_SEQ); }
        else                          { acs_report(s->arg0, acs_number(s)); }
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
            if (ok < 0) { return drive_fail(r, SEQ_ERR_TABLE); }   /* 거리가 펄스 범위 밖 */
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
    case SK_WHEN_STABLE:
    case SK_WHEN_OUT:
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
        s = step_read(r->t.tbl, r->c.step, (PK != 0) ? r->t.pk.at : 0u);
        if (s->kind == SK_LABEL) {          /* 라벨은 실행 시간이 없다 */
            enter_step(r, (uint16_t)(r->c.step + 1), step_next_at((PK != 0) ? r->t.pk.at : 0u));
            continue;
        }
        result = eval_step(r, s);
        if (result == R_WAIT) { return; }
        /* 다음 스텝 머리는 평가 뒤에 구한다: 사용자 함수가 이 러너를 다시 시작했으면 새 표의 0 번이 지금 스텝이다 */
        if (result == R_NEXT) { enter_step(r, (uint16_t)(r->c.step + 1), step_next_at((PK != 0) ? r->t.pk.at : 0u)); }
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
        belt_clear();
        io_seq_outputs_off();
    }
}

void seq_belt_tick(void)
{
    if (s_seq.active) { belt_check(); }
}

void seq_run(void)
{
    if (!s_seq.active && (s_site != 0)) {
        uint16_t i, id, pf, n = map_count(0);
        for (i = 0; i < n; i++) {
            map_get(0, i, &pf, &id);
            if (((pf == 0) || (pf == PF_NUM)) && seq_request_flag[id]) {
                runner_start(&s_seq, 0, i);
                break;
            }
        }
        if (!s_seq.active) {
            for (id = SEQ_NONE + 1; id < SEQ_MAX; id++) { seq_request_flag[id] = 0; }
        }
    }
    if (s_seq.active) {
        belt_check();               /* 스텝보다 먼저: 이 틱에 선 벨트를 WAIT_BELTS 가 같은 틱에 본다 */
        runner_scan(&s_seq);
    }
}

void seq_abort_all(void)
{
    uint16_t id;
    for (id = 0; id < SEQ_MAX; id++) { seq_request_flag[id] = 0; }
    s_seq.active = 0;
    belt_clear();
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
    uint16_t i, row;

    if (act_is_busy(id)) { return 1; }
    if (s_site == 0) { return 2; }
    row = find_act(id);
    if (row == 0xFFFFu) { return 2; }
    for (i = 0; i < ACT_SLOTS; i++) {
        if (!s_act[i].active) {
            runner_start(&s_act[i], 1, row);
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
