/* tune.c - 튜닝 모드. 전원을 켤 때 MODE + DOWN.
 * 출발점에서 1 m 지점에 정확히 멈춘다(1 m - 정지 거리에서 정지 시작). 그 자리에서 실측 오차를 넣고 SET 하면
 * 태그 없이 180도 정렬 회전, 다시 1 m 돌아와 출발점에 멈춘다. 직진 구간에서 'a' 와 'h' 를 번갈아 받아
 * 가이드 부호·배율, 조향 편차, LED 점등 수, 좌우 펄스 비를 재고, 회전에서 휠 간격을 계산한다.
 * 결과는 쪽(page)으로 본다. SHIFT 다음 쪽, UP·DOWN 값 입력, SET 저장, RESET 다시 시험 */
#include <math.h>
#include "agv/tune.h"
#include "agv/drive.h"
#include "agv/guide.h"
#include "agv/param.h"
#include "agv/run.h"
#include "agv/stop.h"
#include "agv/storage.h"
#include "agv/tagact.h"
#include "agv/ui.h"

#define D agv_drive
#define P param_dv

#define TUNE_DIST_MM      1000.0f
#define ERR_MM_LIMIT      200      /* 오차 입력 범위 ±mm */
#define GUIDE_PAIRS_MIN   20       /* 'a'·'h' 짝이 이만큼 모여야 부호·배율을 믿는다 */
#define GUIDE_SCALE_MIN   0.5f
#define GUIDE_SCALE_MAX   2.0f
#define GUIDE_MARK        0x7E57

static struct {
    uint16_t state, turn_seen, page, toggle;
    uint32_t start, l0, r0, dl, dr;
    int16_t  wheel_base, err_turn, err_back, failed;
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
    s_tune.a_fresh = 0;
    s_tune.page = 0;
}

void tune_reset(void)
{
    s_tune.state = TUNE_OFF;
    s_tune.turn_seen = 0;
    s_tune.err_turn = s_tune.err_back = 0;
    s_tune.failed = 0;
    clear_measure();
}

void     tune_arm(void)    { s_tune.state = TUNE_WAIT; clear_measure(); }
int      tune_active(void) { return s_tune.state != TUNE_OFF; }
int      tune_failed(void) { return s_tune.failed; }
uint16_t tune_state(void)  { return s_tune.state; }
int16_t  tune_result(void) { return s_tune.wheel_base; }
uint16_t tune_page(void)   { return s_tune.page; }

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
    if (n > s_tune.led_max) { s_tune.led_max = n; }
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

static uint32_t travel(void) { return run_pulses(D.backward ? 1u : 0u); }

/* 1 m 지점에 멈추도록 정지 거리만큼 앞에서 정지를 건다 */
static int reached(void)
{
    float brake_at = TUNE_DIST_MM * P.one_length_pulse - (float)P.stop_len;
    return (float)(travel() - s_tune.start) >= ((brake_at > 0.0f) ? brake_at : 0.0f);
}

static void stop_here(void)
{
    D.flag_stopping = 1;
    D.stop_pulses = P.stop_len;
    stop_request_tag();
}

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
    s_tune.state = next;
}

static void end_leg(void)
{
    s_tune.dl += run_pulses(0) - s_tune.l0;
    s_tune.dr += run_pulses(1) - s_tune.r0;
}

static int16_t clamp_param(uint16_t idx, float value)
{
    int16_t low, up;
    int16_t v = (int16_t)(value + 0.5f);

    param_limit(idx, &low, &up);
    if (v < low) { v = low; }
    if (v > up)  { v = up; }
    return v;
}

void tune_run(void)
{
    int stopped = !D.flag_run && !D.flag_ready;

    if ((stop_phase() != STOP_PHASE_IDLE) && (stop_floor_rpm() > s_tune.floor_max)) {
        s_tune.floor_max = stop_floor_rpm();
    }
    switch (s_tune.state) {
    case TUNE_WAIT:
        if (D.flag_run) {
            clear_measure();
            s_tune.failed = 0;
            begin_leg(TUNE_GO_1);
        }
        break;
    case TUNE_GO_1:
        if (stopped && !D.flag_stopping) { s_tune.state = TUNE_WAIT; break; }
        go_slow();
        if (reached() && !D.flag_stopping) {
            end_leg();
            stop_here();
            s_tune.turn_seen = 0;
            s_tune.state = TUNE_PAUSE;
        }
        break;
    case TUNE_PAUSE:          /* 1 m 지점. 실측 오차를 넣고 SET 으로 회전 */
        /* 정지가 끝난 뒤 다시 달리면(START) 시험을 접는다. turn_seen 을 정지 확인에 빌려 쓴다 */
        if (stopped && !D.flag_stopping) { s_tune.turn_seen = 1; }
        else if (s_tune.turn_seen && D.flag_run) { s_tune.state = TUNE_WAIT; }
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
    case TUNE_GO_2:
        if (stopped && !D.flag_stopping) { s_tune.state = TUNE_WAIT; break; }
        go_slow();
        if (reached() && !D.flag_stopping) {
            end_leg();
            stop_here();
            s_tune.state = TUNE_DONE;
            s_tune.page = 0;
        }
        break;
    default:
        break;
    }
}

/* ---- 결과 ---- */

int16_t tune_value(uint16_t page)
{
    float k;

    switch (page) {
    case TUNE_PAGE_WHEEL_BASE: return s_tune.wheel_base;
    case TUNE_PAGE_DISTANCE:   return s_tune.err_turn;
    case TUNE_PAGE_RETURN:     return s_tune.err_back;
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

static void err_step(int16_t *e, int16_t sign)
{
    *e = (int16_t)(*e + sign);
    if (*e < -ERR_MM_LIMIT) { *e = -ERR_MM_LIMIT; }
    if (*e > ERR_MM_LIMIT)  { *e = ERR_MM_LIMIT; }
}

int tune_key(uint16_t key)
{
    int16_t sign = (key == KEY_UP) ? 1 : -1;

    if (D.flag_run || D.flag_stopping) { return 0; }
    if (s_tune.state == TUNE_PAUSE) {
        switch (key) {
        case KEY_UP:
        case KEY_DOWN:
            err_step(&s_tune.err_turn, sign);
            return 1;
        case KEY_SET:
            D.tape_slow = 1;
            s_tune.turn_seen = 0;
            tagact_turn_align_stopped(TURN_CW, 180u, 0u);
            if (D.tape_align) { s_tune.state = TUNE_TURN; }
            else              { D.tape_slow = 0; s_tune.failed = 1; s_tune.state = TUNE_WAIT; }
            return 1;
        case KEY_RESET:
            s_tune.state = TUNE_WAIT;
            return 1;
        default:
            return 0;
        }
    }
    if (s_tune.state != TUNE_DONE) { return 0; }
    switch (key) {
    case KEY_SHIFT:
        s_tune.page = (uint16_t)((s_tune.page + 1u) % TUNE_PAGE_COUNT);
        return 1;
    case KEY_UP:
    case KEY_DOWN:
        if (s_tune.page == TUNE_PAGE_DISTANCE) { err_step(&s_tune.err_turn, sign); }
        if (s_tune.page == TUNE_PAGE_RETURN)   { err_step(&s_tune.err_back, sign); }
        return 1;
    case KEY_SET:
        switch (s_tune.page) {
        case TUNE_PAGE_WHEEL_BASE:
            return (param_set(P_WHEEL_BASE_MM, s_tune.wheel_base) == 0) ? 2 : 1;
        case TUNE_PAGE_DISTANCE:
            /* 엔코더로 1 m 간 자리의 실측 오차로 휠 지름을 비례해 고친다 */
            return (param_set(P_WHEEL_DIA_MM, clamp_param(P_WHEEL_DIA_MM, (float)param_get(P_WHEEL_DIA_MM)
                                                * (TUNE_DIST_MM + (float)s_tune.err_turn) / TUNE_DIST_MM)) == 0) ? 2 : 1;
        case TUNE_PAGE_GUIDE:
            return save_guide() ? 2 : 1;
        default:
            return 1;
        }
    case KEY_RESET:
        s_tune.state = TUNE_WAIT;
        return 1;
    default:
        return 0;
    }
}
