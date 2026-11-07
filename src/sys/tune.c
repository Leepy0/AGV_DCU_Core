/* tune.c - 튜닝 모드. 전원을 켤 때 MODE + DOWN.
 * 주행로에 1 m 간격 교차선 A, B 를 그리고 A 앞에서 START.
 *  1. 최저 속도로 A, B 를 지난다. 두 선 사이 엔코더 거리로 휠 지름을 고쳐 바로 반영한다.
 *  2. B 에서 가이드센서 거리(GD)만큼 더 가서 멈추고 180도 정렬 회전. 회전 펄스로 휠 간격을 계산한다.
 *  3. 돌아오다 교차선(A)을 보면 바로 멈춘다. 선과 회전 중심 사이를 재서 넣으면
 *     감지 뒤 미끄러진 거리를 더해 GD 를 저장한다.
 * 직진 구간에서 'a' 와 'h' 를 번갈아 받아 가이드 부호·배율, 조향 편차, LED 점등 수, 좌우 펄스 비도 잰다.
 * 결과 키는 파라미터 편집과 같다: UP·DOWN 쪽 선택, SET 수정 시작, SHIFT 자리, UP·DOWN 값,
 * SET 확정·저장, RESET 취소. 수정 중이 아니면 RESET 은 다시 시험.
 * 대기(TUNE) 중 SHIFT 를 누르면 식별 시험(IDEN)으로 바뀐다. ident.c 참고 */
#include <math.h>
#include "agv/tune.h"
#include "agv/drive.h"
#include "agv/guide.h"
#include "agv/ident.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/stop.h"
#include "agv/storage.h"
#include "agv/tagact.h"
#include "agv/ui.h"

#define D agv_drive
#define P param_dv

#define LINE_GAP_MM       1000.0f  /* 교차선 A, B 간격 */
#define GAP_TOL           0.3f     /* 엔코더로 잰 간격이 ±30% 를 넘으면 선을 잘못 본 것 */
#define SEEK_LIMIT_MM     3000.0f  /* 이 거리 안에 선을 못 보면 실패 */
#define GAP_INPUT_LOW     (-200)
#define GAP_INPUT_UP      999
#define GUIDE_PAIRS_MIN   20       /* 'a'·'h' 짝이 이만큼 모여야 부호·배율을 믿는다 */
#define GUIDE_SCALE_MIN   0.5f
#define GUIDE_SCALE_MAX   2.0f
#define GUIDE_MARK        0x7E57

static struct {
    uint16_t state, turn_seen, page, toggle, ident;
    uint32_t start, l0, r0, dl, dr;
    int16_t  wheel_base, dia_before, failed;
    int16_t  editing, place, edit_value;
    /* 교차선 */
    uint16_t lines, led_prev;
    uint32_t line_at[2], stop_from;
    int16_t  overrun_mm;
    /* 측정 */
    int16_t  a_last, a_fresh;
    float    ah, hh, se;
    uint32_t pairs, se_n;
    int16_t  se_max, floor_max;
    uint16_t led_max;
} s_tune;

static void clear_measure(void)
{
    s_tune.dl = s_tune.dr = 0;
    s_tune.ah = s_tune.hh = s_tune.se = 0.0f;
    s_tune.pairs = s_tune.se_n = 0;
    s_tune.se_max = s_tune.floor_max = 0;
    s_tune.led_max = 0;
    s_tune.led_prev = 0;
    s_tune.lines = 0;
    s_tune.a_fresh = 0;
    s_tune.overrun_mm = 0;
    s_tune.page = 0;
    s_tune.editing = 0;
}

void tune_reset(void)
{
    s_tune.state = TUNE_OFF;
    s_tune.turn_seen = 0;
    s_tune.failed = 0;
    s_tune.ident = 0;
    clear_measure();
}

void     tune_arm(void)        { s_tune.state = TUNE_WAIT; clear_measure(); }
int      tune_active(void)     { return s_tune.state != TUNE_OFF; }
int      tune_failed(void)     { return s_tune.failed; }
int      tune_ident_mode(void) { return (int)s_tune.ident; }
int      tune_editing(void)    { return s_tune.editing; }
int16_t  tune_edit_place(void) { return s_tune.place; }
uint16_t tune_state(void)      { return s_tune.state; }
int16_t  tune_result(void)     { return s_tune.wheel_base; }
uint16_t tune_page(void)       { return s_tune.page; }
uint16_t tune_lines(void)      { return s_tune.lines; }

static uint32_t travel(void) { return run_pulses(D.backward ? 1u : 0u); }

static int measuring(void)
{
    return ((s_tune.state == TUNE_GO_1) || (s_tune.state == TUNE_GO_2)) && D.flag_run && !D.flag_stopping;
}

/* ---- 가이드 측정 (guide_poll 이 부른다) ---- */

int tune_want_h(void)
{
    if (!measuring()) { return 0; }
    s_tune.toggle ^= 1u;
    return (int)s_tune.toggle;
}

void tune_on_a(int16_t status, int16_t pos)
{
    int16_t m = (pos < 0) ? (int16_t)-pos : pos;

    if (!measuring() || (status != GUIDE_NORMAL)) { return; }
    s_tune.se += (float)pos * (float)pos;
    s_tune.se_n++;
    if (m > s_tune.se_max) { s_tune.se_max = m; }
    s_tune.a_last = pos;
    s_tune.a_fresh = 1;
}

void tune_on_h(const guide_rx_t *rx)
{
    guide_out_t out;
    uint32_t lat;
    uint16_t n = 0;

    if (!measuring()) { return; }
    lat = guide_lateral(rx);
    while (lat != 0UL) { n = (uint16_t)(n + (uint16_t)(lat & 1UL)); lat >>= 1; }
    /* 교차선은 앞쪽 에지에서 한 번 센다. 일반 라인 점등 수에는 넣지 않는다 */
    if (n >= GUIDE_CROSS_LED_MIN) {
        if ((s_tune.led_prev < GUIDE_CROSS_LED_MIN) && (s_tune.lines < 2u)) {
            s_tune.line_at[s_tune.lines] = travel();
            s_tune.lines++;
        }
    }
    else if (n > s_tune.led_max) {
        s_tune.led_max = n;
    }
    s_tune.led_prev = n;
    guide_track(rx, 0, D.backward, &out);
    if ((out.status == GUIDE_NORMAL) && s_tune.a_fresh) {
        s_tune.ah += (float)s_tune.a_last * (float)out.pos;
        s_tune.hh += (float)out.pos * (float)out.pos;
        s_tune.pairs++;
    }
    s_tune.a_fresh = 0;
}

/* 'a' = k × 'h'. 부호가 음수면 'h' 위치 부호가 반대다. 0 = 측정 부족 */
static float guide_ratio(void)
{
    float k;
    if ((s_tune.pairs < GUIDE_PAIRS_MIN) || (s_tune.hh <= 0.0f)) { return 0.0f; }
    k = s_tune.ah / s_tune.hh;
    if ((fabsf(k) < GUIDE_SCALE_MIN) || (fabsf(k) > GUIDE_SCALE_MAX)) { return 0.0f; }
    return k;
}

/* ---- 주행 순서 ---- */

static void go_slow(void)
{
    if (D.speed_cmd != P.min_speed) {
        D.speed_cmd = P.min_speed;
        D.step = DRIVE_CHANGE_SPEED;
    }
}

static void begin_leg(uint16_t next)
{
    s_tune.start = travel();
    s_tune.l0 = run_pulses(0);
    s_tune.r0 = run_pulses(1);
    s_tune.lines = 0;
    s_tune.led_prev = 0;
    s_tune.state = next;
}

static void end_leg(void)
{
    s_tune.dl += run_pulses(0) - s_tune.l0;
    s_tune.dr += run_pulses(1) - s_tune.r0;
}

static float leg_mm(void) { return (float)(travel() - s_tune.start) / P.one_length_pulse; }

static int16_t clamp_param(uint16_t idx, float value)
{
    int16_t low, up;
    int16_t v = (int16_t)(value + 0.5f);

    param_limit(idx, &low, &up);
    if (v < low) { v = low; }
    if (v > up)  { v = up; }
    return v;
}

static void fail(void)
{
    tagact_stop();
    D.tape_slow = 0;
    s_tune.failed = 1;
    s_tune.state = TUNE_WAIT;
}

/* A, B 사이 엔코더 거리로 휠 지름을 고친다. 실제 1 m 를 엔코더가 짧게 재면 바퀴가 큰 것이다 */
static int fix_diameter(void)
{
    float measured = (float)(s_tune.line_at[1] - s_tune.line_at[0]) / P.one_length_pulse;

    if ((measured < LINE_GAP_MM * (1.0f - GAP_TOL)) || (measured > LINE_GAP_MM * (1.0f + GAP_TOL))) { return 0; }
    (void)param_set(P_WHEEL_DIA_MM, clamp_param(P_WHEEL_DIA_MM,
                                                (float)param_get(P_WHEEL_DIA_MM) * LINE_GAP_MM / measured));
    return 1;
}

void tune_run(void)
{
    int stopped = !D.flag_run && !D.flag_ready;

    if ((stop_phase() != STOP_PHASE_IDLE) && (stop_floor_rpm() > s_tune.floor_max)) {
        s_tune.floor_max = stop_floor_rpm();
    }
    switch (s_tune.state) {
    case TUNE_WAIT:
        if (D.flag_run && s_tune.ident) {
            s_tune.failed = 0;
            ident_start();
            s_tune.state = TUNE_IDENT;
        }
        else if (D.flag_run) {
            clear_measure();
            s_tune.failed = 0;
            s_tune.dia_before = param_get(P_WHEEL_DIA_MM);
            begin_leg(TUNE_GO_1);
        }
        break;
    case TUNE_GO_1:          /* A, B 를 지나 B 위에서 회전 */
        if (stopped && !D.flag_stopping) { s_tune.state = TUNE_WAIT; break; }
        if (D.flag_stopping) { break; }
        go_slow();
        if (s_tune.lines >= 2u) {
            end_leg();
            if (!fix_diameter()) { fail(); break; }
            D.tape_slow = 1;
            s_tune.turn_seen = 0;
            tagact_turn_align_cross(TURN_CW, 180u, 0u);
            if (D.tape_align) { s_tune.state = TUNE_TURN; }
            else              { fail(); }
        }
        else if (leg_mm() > SEEK_LIMIT_MM) {
            fail();
        }
        break;
    case TUNE_IDENT:
        if (ident_run() != 0) { s_tune.state = TUNE_IDENT_DONE; }
        break;
    case TUNE_TURN:
        if (D.flag_turn) { s_tune.turn_seen = 1; }
        else if (s_tune.turn_seen && !D.tape_align && !D.flag_stopping && (D.turn_motion == 0)) {
            D.tape_slow = 0;
            if ((D.system_err != 0) || (D.stop_err != 0)) {
                s_tune.failed = 1;
                s_tune.state = TUNE_WAIT;
            }
            else {
                s_tune.wheel_base = clamp_param(P_WHEEL_BASE_MM, (float)param_get(P_WHEEL_BASE_MM)
                                                * (float)D.tape_turned / (float)D.tape_total_pulse);
                run_restart_now();
                s_tune.state = TUNE_RESTART;
            }
        }
        break;
    case TUNE_RESTART:
        if (D.flag_run) { begin_leg(TUNE_GO_2); }
        break;
    case TUNE_GO_2:          /* 돌아오다 교차선을 보면 바로 멈춘다 */
        if (stopped && !D.flag_stopping) { s_tune.state = TUNE_WAIT; break; }
        if (D.flag_stopping) { break; }
        go_slow();
        if (s_tune.lines >= 1u) {
            end_leg();
            s_tune.stop_from = s_tune.line_at[0];
            tagact_stop();
            s_tune.state = TUNE_STOPPING;
        }
        else if (leg_mm() > SEEK_LIMIT_MM) {
            fail();
        }
        break;
    case TUNE_STOPPING:
        if (stopped && !D.flag_stopping) {
            s_tune.overrun_mm = (int16_t)((float)(travel() - s_tune.stop_from) / P.one_length_pulse + 0.5f);
            s_tune.state = TUNE_DONE;
            s_tune.page = TUNE_PAGE_GAP;
        }
        break;
    default:
        break;
    }
}

/* ---- 결과 ---- */

/* 선과 회전 중심 사이 간격의 예상값. 지금 GD 에서 미끄러진 거리를 뺀 값이다 */
static int16_t expected_gap(void) { return (int16_t)(param_get(P_GUIDE_DIST_MM) - s_tune.overrun_mm); }

int16_t tune_value(uint16_t page)
{
    float k;

    if (s_tune.editing && (page == s_tune.page)) { return s_tune.edit_value; }
    switch (page) {
    case TUNE_PAGE_WHEEL_BASE: return s_tune.wheel_base;
    case TUNE_PAGE_DIAMETER:   return param_get(P_WHEEL_DIA_MM);
    case TUNE_PAGE_GAP:        return expected_gap();
    case TUNE_PAGE_OVERRUN:    return s_tune.overrun_mm;
    case TUNE_PAGE_GUIDE_DIST: return param_get(P_GUIDE_DIST_MM);
    case TUNE_PAGE_GUIDE:
        k = guide_ratio();
        return (k == 0.0f) ? TUNE_NO_DATA : (int16_t)(k * 100.0f + ((k < 0.0f) ? -0.5f : 0.5f));
    case TUNE_PAGE_STEER_RMS:
        return (s_tune.se_n == 0) ? TUNE_NO_DATA : (int16_t)(sqrtf(s_tune.se / (float)s_tune.se_n) * 10.0f + 0.5f);
    case TUNE_PAGE_STEER_MAX:  return s_tune.se_max;
    case TUNE_PAGE_FLOOR:      return s_tune.floor_max;
    case TUNE_PAGE_LED:        return (int16_t)s_tune.led_max;
    case TUNE_PAGE_LR:
        if (s_tune.dl == 0UL) { return TUNE_NO_DATA; }
        return (int16_t)(((float)s_tune.dr / (float)s_tune.dl - 1.0f) * 1000.0f);
    default: return TUNE_NO_DATA;
    }
}

/* 저장한 가이드 부호·배율을 부팅 때 되살린다 */
void tune_load(void)
{
    guide_cfg_t cfg;
    int16_t inv, scale;

    if (keep_get(KEEP_GUIDE_MARK) != GUIDE_MARK) { return; }
    inv = keep_get(KEEP_GUIDE_INV);
    scale = keep_get(KEEP_GUIDE_SCALE);
    guide_cfg_get(&cfg);
    cfg.invert_fwd = inv & 1;
    cfg.invert_back = (inv >> 1) & 1;
    if ((scale >= (int16_t)(GUIDE_SCALE_MIN * 1000.0f)) && (scale <= (int16_t)(GUIDE_SCALE_MAX * 1000.0f))) {
        cfg.h_scale = (float)scale / 1000.0f;
    }
    guide_configure(&cfg);
}

static int save_guide(void)
{
    guide_cfg_t cfg;
    float k = guide_ratio();
    int16_t inv;

    if (k == 0.0f) { return 0; }
    guide_cfg_get(&cfg);
    if (k < 0.0f) {
        if (D.backward) { cfg.invert_back ^= 1; }
        else            { cfg.invert_fwd ^= 1; }
    }
    cfg.h_scale = cfg.h_scale * fabsf(k);
    if (cfg.h_scale < GUIDE_SCALE_MIN) { cfg.h_scale = GUIDE_SCALE_MIN; }
    if (cfg.h_scale > GUIDE_SCALE_MAX) { cfg.h_scale = GUIDE_SCALE_MAX; }
    guide_configure(&cfg);
    inv = (int16_t)((cfg.invert_fwd & 1) | ((cfg.invert_back & 1) << 1));
    keep_set(KEEP_GUIDE_INV, inv);
    keep_set(KEEP_GUIDE_SCALE, (int16_t)(cfg.h_scale * 1000.0f + 0.5f));
    keep_set(KEEP_GUIDE_MARK, GUIDE_MARK);
    keep_flush_if_changed();
    return 1;
}

static void edit_limits(uint16_t page, int16_t *low, int16_t *up)
{
    switch (page) {
    case TUNE_PAGE_WHEEL_BASE: param_limit(P_WHEEL_BASE_MM, low, up); break;
    case TUNE_PAGE_DIAMETER:   param_limit(P_WHEEL_DIA_MM, low, up); break;
    case TUNE_PAGE_GUIDE_DIST: param_limit(P_GUIDE_DIST_MM, low, up); break;
    default:                   *low = GAP_INPUT_LOW; *up = GAP_INPUT_UP; break;
    }
}

/* SHIFT·UP·DOWN. 처리했으면 1 */
static int edit_key(uint16_t key, uint16_t page)
{
    int16_t low, up, step;

    if (key == KEY_SHIFT) {
        s_tune.place = (int16_t)((s_tune.place >= 3) ? 1 : s_tune.place + 1);
        return 1;
    }
    if ((key != KEY_UP) && (key != KEY_DOWN)) { return 0; }
    step = (s_tune.place == 3) ? 100 : ((s_tune.place == 2) ? 10 : 1);
    edit_limits(page, &low, &up);
    s_tune.edit_value = (int16_t)(s_tune.edit_value + ((key == KEY_UP) ? step : -step));
    if (s_tune.edit_value < low) { s_tune.edit_value = low; }
    if (s_tune.edit_value > up)  { s_tune.edit_value = up; }
    return 1;
}

static int editable(uint16_t page)
{
    return (page == TUNE_PAGE_WHEEL_BASE) || (page == TUNE_PAGE_DIAMETER)
        || (page == TUNE_PAGE_GAP) || (page == TUNE_PAGE_GUIDE_DIST);
}

static int commit(void)
{
    int16_t v = s_tune.edit_value;

    s_tune.editing = 0;
    switch (s_tune.page) {
    case TUNE_PAGE_WHEEL_BASE:
        s_tune.wheel_base = v;
        return (param_set(P_WHEEL_BASE_MM, v) == 0) ? 2 : 1;
    case TUNE_PAGE_DIAMETER:
        return (param_set(P_WHEEL_DIA_MM, v) == 0) ? 2 : 1;
    case TUNE_PAGE_GAP:          /* 잰 간격 + 감지 뒤 미끄러진 거리 = 센서와 회전 중심 사이 */
        return (param_set(P_GUIDE_DIST_MM, clamp_param(P_GUIDE_DIST_MM, (float)(v + s_tune.overrun_mm))) == 0) ? 2 : 1;
    case TUNE_PAGE_GUIDE_DIST:
        return (param_set(P_GUIDE_DIST_MM, v) == 0) ? 2 : 1;
    default:
        return 1;
    }
}

int tune_key(uint16_t key)
{
    if (D.flag_run || D.flag_stopping) { return 0; }
    if ((s_tune.state == TUNE_WAIT) && (key == KEY_SHIFT)) {
        s_tune.ident ^= 1u;
        s_tune.failed = 0;
        return 1;
    }
    if ((s_tune.state == TUNE_IDENT_DONE) && (key == KEY_RESET)) {
        s_tune.state = TUNE_WAIT;
        return 1;
    }
    if (s_tune.state != TUNE_DONE) { return 0; }
    if (s_tune.editing) {
        if (edit_key(key, s_tune.page)) { return 1; }
        if (key == KEY_RESET) { s_tune.editing = 0; return 1; }
        if (key == KEY_SET)   { return commit(); }
        return 0;
    }
    switch (key) {
    case KEY_UP:
        s_tune.page = (uint16_t)((s_tune.page + 1u) % TUNE_PAGE_COUNT);
        return 1;
    case KEY_DOWN:
        s_tune.page = (uint16_t)((s_tune.page + TUNE_PAGE_COUNT - 1u) % TUNE_PAGE_COUNT);
        return 1;
    case KEY_SET:
        if (editable(s_tune.page)) {
            s_tune.edit_value = tune_value(s_tune.page);   /* 수정 표시를 켜기 전에 읽는다 */
            s_tune.editing = 1;
            s_tune.place = 1;
            return 1;
        }
        if (s_tune.page == TUNE_PAGE_GUIDE) { return save_guide() ? 2 : 1; }
        return 1;
    case KEY_RESET:
        s_tune.state = TUNE_WAIT;
        return 1;
    default:
        return 0;
    }
}
