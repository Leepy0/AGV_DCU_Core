/* tune.c - 튜닝 모드 메뉴, 입력, 연속 시험(t1·t2) 결과 쪽과 기준값. 키와 FND 는 tune.h.
 * 주행은 ident.c(연속 시험 ident_start_build, 개발 시험)와 rpmtest.c 가 한다.
 * 여기는 선 찾기 중 'a' 와 'h' 를 번갈아 받아 교차선 수, 가이드 부호·배율, 직진 편차도 잰다.
 * 저장 구분:
 *  바로 저장    입력 확정(22·50번, L, 상한), 연속 시험 중 22·24·28번·'h' 부호·배율(ident.c), 점검 GA 확정(50번)
 *  채택 저장    결과 쪽 1·13·35번(점검은 28·1·13번). SET 채택, RESET 건너뛰기, SAVE 에서 쓴다
 *  기준값       t1 SAVE 때 EEPROM 110~114·119·125. t2 가 견준다
 *  안 건드림    20·21번(KP·KI) */
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

#define GAP_INPUT_LOW     (-200)
#define GAP_INPUT_UP      999
#define TRACK_UP          9999
#define CAP_DEFAULT       60
#define GUIDE_PAIRS_MIN   20       /* 'a'·'h' 짝이 이만큼 모여야 부호·배율을 믿는다 */
#define GUIDE_SCALE_MIN   0.5f
#define GUIDE_SCALE_MAX   2.0f
#define GUIDE_MARK        0x7E57
#define PAGE_MAX          20
#define ALT_TICKS         100      /* 채택 쪽: 1 s 지금 값, 1 s 새 값 */

const uint16_t tune_base_addr[TUNE_BASE_N] = { 111, 112, 113, 114, 119, 125 };

/* 입력 칸. t1 아래 1~4 번째, t2.x 아래 L, 점검 결과의 GA, t3.6 아래 속도 */
enum { F_DIA = 0, F_GD, F_CAP, F_TRACK, F_GA, F_HS };
static const struct { char name[2]; int16_t digits; } s_field[] = {
    { {'D', 'I'}, 3 }, { {'G', 'D'}, 3 }, { {'T', 'P'}, 2 }, { {'L', ' '}, 4 }, { {'G', 'A'}, 3 }, { {'S', 'P'}, 2 }
};
#define T1_INPUTS 4

#define T1_MASK (BLD_MECH | BLD_DIA | BLD_WB | BLD_GUIDE | BLD_RPM | BLD_MIN | BLD_TOP | BLD_COAST | BLD_STOPT | BLD_STEER)
static const struct { char tag[2]; uint16_t mask; } s_check[] = {
    { {'C', 'H'}, BLD_MECH | BLD_DIA | BLD_TOP | BLD_COAST | BLD_STOPT | BLD_STEER | BLD_CHECK },            /* t2.1 빠른 점검 */
    { {'W', 'H'}, BLD_MECH | BLD_DIA | BLD_WB | BLD_TOP | BLD_COAST | BLD_STOPT | BLD_STEER | BLD_CHECK },   /* t2.2 바퀴 교체 */
    { {'G', 'S'}, BLD_MECH | BLD_GUIDE | BLD_GA },                                                        /* t2.3 가이드 센서 교체 */
    { {'A', 'P'}, BLD_MECH | BLD_DIA | BLD_RPM | BLD_TOP | BLD_COAST | BLD_STOPT | BLD_STEER | BLD_CHECK }  /* t2.4 모터·앰프 교체 */
};
#define CHECK_N ((uint16_t)(sizeof(s_check) / sizeof(s_check[0])))

static void hs_start(void);

/* t3 개발 시험. 줄을 더하면 t3.n 이 는다. run 다음 상태가 끝 상태이고 ui.c 가 둘을 그린다 */
static const struct { char tag[2]; void (*start)(void); uint16_t run; } s_dev[] = {
    { {'I', 'D'}, ident_start,      TUNE_IDENT },
    { {'R', 'P'}, ident_start_rpm,  TUNE_RPM },
    { {'T', 'P'}, ident_start_top,  TUNE_TOP },
    { {'A', 'L'}, ident_start_all,  TUNE_ALL },
    { {'T', 'E'}, ident_start_temp, TUNE_TEMP },
    { {'H', 'S'}, hs_start,         TUNE_HS }
};
#define DEV_N ((uint16_t)(sizeof(s_dev) / sizeof(s_dev[0])))

/* 결과 쪽 */
enum { PK_SET = 0, PK_VAL, PK_GA, PK_SAVE };
enum { N_DI = 0, N_WW, N_G, N_LO, N_TP, N_DE, N_FL, N_ST, N_SE, N_LR, N_OV, N_BA, N_A };
static const char s_name[][2] = {
    {'D', 'I'}, {'W', 'W'}, {'G', ' '}, {'L', 'O'}, {'T', 'P'}, {'D', 'E'}, {'F', 'L'}, {'S', 'T'}, {'S', 'E'},
    {'L', 'R'}, {'O', 'V'}, {'B', 'A'}, {'A', ' '}
};
typedef struct { int16_t kind, id, v, old, flag; } page_t;   /* flag: PK_SET 채택, PK_VAL NG */

static struct {
    uint16_t state, cat, item, sub, page, n_page, toggle, alt;
    int16_t  editing, place, edit_value, track, cap;
    int16_t  hs, hs_up;           /* t3.6 속도(0 = 1번), 입력 상한 */
    page_t   pg[PAGE_MAX];
    /* 교차선 */
    uint16_t lines, led_prev;
    uint32_t line_at[2];
    /* 측정 */
    int16_t  a_last, a_fresh, floor_max;
    float    ah, hh;
    uint32_t pairs;
} s_tune;

static void clear_measure(void)
{
    s_tune.ah = s_tune.hh = 0.0f;
    s_tune.pairs = 0;
    s_tune.floor_max = 0;
    s_tune.led_prev = 0;
    s_tune.lines = 0;
    s_tune.a_fresh = 0;
    s_tune.page = 0;
    s_tune.editing = 0;
}

static void menu_top(void)
{
    s_tune.cat = 1;
    s_tune.item = s_tune.sub = 0;
}

void tune_reset(void)
{
    s_tune.state = TUNE_OFF;
    menu_top();
    clear_measure();
}

void tune_arm(void)
{
    s_tune.state = TUNE_WAIT;
    menu_top();
    clear_measure();
}

static int dev_pick(uint16_t n) { return (s_tune.cat == 3u) && (s_tune.item == n); }

int      tune_active(void)     { return s_tune.state != TUNE_OFF; }
int      tune_ident_mode(void) { return dev_pick(TUNE_DEV_IDENT); }
int      tune_rpm_mode(void)   { return dev_pick(TUNE_DEV_RPM); }
int      tune_top_mode(void)   { return dev_pick(TUNE_DEV_TOP); }
int      tune_all_mode(void)   { return dev_pick(TUNE_DEV_ALL); }
int      tune_temp_mode(void)  { return dev_pick(TUNE_DEV_TEMP); }
int      tune_hs_mode(void)    { return dev_pick(TUNE_DEV_HS); }
int16_t  tune_hs_mpm(void)     { return s_tune.hs; }

static void hs_start(void) { ident_start_hs(s_tune.hs ? s_tune.hs : param_get(P_SPEED_HIGH)); }

/* t3.6 입력 상한: 1번, 입력 L 이 있으면 그 L 에 측정 창이 들어가는 속도까지 */
static void hs_enter(void)
{
    int16_t lo, up;

    param_limit(P_SPEED_HIGH, &lo, &up);
    up = param_get(P_SPEED_HIGH);
    while ((s_tune.track > 0) && (up > lo) && (ident_hs_need_mm(up) > (float)s_tune.track)) { up--; }
    s_tune.hs_up = up;
    if ((s_tune.hs == 0) || (s_tune.hs > up)) { s_tune.hs = up; }
}
int      tune_editing(void)    { return s_tune.editing; }
uint16_t tune_state(void)      { return s_tune.state; }
uint16_t tune_page(void)       { return s_tune.page; }
uint16_t tune_menu(void)       { return (uint16_t)(s_tune.cat * 100u + s_tune.item * 10u + s_tune.sub); }
uint16_t tune_lines(void)      { return s_tune.lines; }
uint32_t tune_line_at(uint16_t i) { return s_tune.line_at[(i < 2u) ? i : 1u]; }
void     tune_cross_clear(void)  { s_tune.lines = 0; s_tune.led_prev = 0; }

/* t3 시험 진행 상태 */
static int dev_running(uint16_t st) { return (st >= TUNE_IDENT) && (((st - TUNE_IDENT) & 1u) == 0u); }

static int measuring(void)
{
    return ((s_tune.state == TUNE_BLD) || dev_running(s_tune.state)) && ident_want_h();   /* 선 찾기에서만 */
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
    if (!measuring() || (status != GUIDE_NORMAL)) { return; }
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
    /* 교차선은 앞쪽 에지에서 한 번 센다 */
    if ((n >= GUIDE_CROSS_LED_MIN) && (s_tune.led_prev < GUIDE_CROSS_LED_MIN) && (s_tune.lines < 2u)) {
        s_tune.line_at[s_tune.lines] = run_pulses(D.backward ? 1u : 0u);
        s_tune.lines++;
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

int16_t tune_guide_k100(void)
{
    float k = guide_ratio();
    return (k == 0.0f) ? TUNE_NO_DATA : (int16_t)(k * 100.0f + ((k < 0.0f) ? -0.5f : 0.5f));
}

int tune_guide_save(void)
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
    cfg.h_scale = fabsf(k);          /* k 는 배율을 걸기 전 'h' 위치(guide_track)와 견준 값이다 */
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

/* ---- EEPROM ---- */

static uint16_t ee(uint16_t addr) { return storage_queued.read(addr); }

/* 부팅: 가이드 부호·배율, 입력 L·상한. 빈 칸이나 한계 밖은 0·60 */
void tune_load(void)
{
    guide_cfg_t cfg;
    int16_t inv, scale, lo, up;

    s_tune.track = (int16_t)ee(TUNE_EE_TRACK);
    if ((s_tune.track < 0) || (s_tune.track > TRACK_UP)) { s_tune.track = 0; }
    param_limit(P_SPEED_HIGH, &lo, &up);
    s_tune.cap = (int16_t)ee(TUNE_EE_CAP);
    if ((s_tune.cap < lo) || (s_tune.cap > up)) { s_tune.cap = CAP_DEFAULT; }
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

int16_t tune_base(uint16_t i) { return (int16_t)ee(tune_base_addr[(i < TUNE_BASE_N) ? i : 0u]); }

int tune_base_valid(void)
{
    uint16_t i, sum = 0;
    for (i = 0; i < TUNE_BASE_N; i++) { sum = (uint16_t)(sum + ee(tune_base_addr[i])); }
    return ee(TUNE_EE_CHECK) == (uint16_t)(TUNE_BASE_MARK ^ sum);
}

/* 기준값은 t1 SAVE 때만 쓴다(쓰기 수명). 검사 칸이 맞지 않으면 기준값이 없는 것으로 본다 */
static void base_write(void)
{
    const build_result_t *b = &agv_build;
    int16_t v[TUNE_BASE_N];
    int16_t se = (b->se_x10 < 0) ? 0 : ((b->se_x10 > 255) ? 255 : b->se_x10);
    uint16_t i, sum = 0;

    v[TUNE_BASE_TOP] = b->top_rpm;
    v[TUNE_BASE_DEC] = b->dec_rpm_s;
    v[TUNE_BASE_FLOOR] = b->floor_rpm;
    v[TUNE_BASE_LR] = b->lr_permille;
    v[TUNE_BASE_BAT] = b->bat_dv;
    v[TUNE_BASE_AMP_SE] = (int16_t)(((uint16_t)b->amp << 8) | (uint16_t)se);
    for (i = 0; i < TUNE_BASE_N; i++) {
        storage_request(tune_base_addr[i], (uint16_t)v[i]);
        sum = (uint16_t)(sum + (uint16_t)v[i]);
    }
    storage_request(TUNE_EE_CHECK, (uint16_t)(TUNE_BASE_MARK ^ sum));
}

/* ---- 결과 쪽 ---- */

static void add(int16_t kind, int16_t id, int16_t v, int16_t old, int16_t flag)
{
    page_t *p;
    if (s_tune.n_page >= PAGE_MAX) { return; }
    p = &s_tune.pg[s_tune.n_page++];
    p->kind = kind;
    p->id = id;
    p->v = v;
    p->old = old;
    p->flag = flag;
}

static void add_set(uint16_t idx, int16_t v)
{
    if (v >= 0) { add(PK_SET, (int16_t)idx, v, param_get(idx), 0); }
}

/* 점검이고 기준값이 있으면 기준 대비 % 와 판정, 아니면 값 */
static void add_cmp(int16_t name, int16_t now, uint16_t bi, int16_t ng_pct)
{
    int16_t b = tune_base(bi);
    int32_t p;

    if (now <= 0) { add(PK_VAL, name, TUNE_NO_DATA, 0, 0); return; }
    if ((s_tune.cat != 2u) || !tune_base_valid() || (b <= 0)) { add(PK_VAL, name, now, 0, 0); return; }
    p = (((int32_t)now - b) * 100L) / b;
    if (p > 999L)  { p = 999L; }
    if (p < -999L) { p = -999L; }
    add(PK_VAL, name, (int16_t)p, 0, (ng_pct < 0) ? (p < ng_pct) : (p > ng_pct));
}

static void build_pages(void)
{
    const build_result_t *b = &agv_build;
    uint16_t m = (uint16_t)b->mask;
    int chk = (s_tune.cat == 2u);
    int16_t lr = (b->lr_permille < 0) ? (int16_t)-b->lr_permille : b->lr_permille;

    s_tune.n_page = 0;
    if (chk) { add_set(P_SPEED_MIN, b->min_rec); }
    add_set(P_SPEED_HIGH, b->high_rec);
    add_set(P_DCC, b->dcc_rec);
    add_set(P_CROSS_IN_CM, b->cid_rec);
    if (b->dia_new > 0)  { add(PK_VAL, N_DI, b->dia_new, 0, 0); }
    if (m & BLD_WB)      { add(PK_VAL, N_WW, b->wb_new, 0, 0); }
    if (m & BLD_GUIDE)   { add(PK_VAL, N_G, b->guide_k100, 0, 0); }
    if (m & BLD_MIN)     { add(PK_VAL, N_LO, b->min_rec, 0, 0); }
    if (m & BLD_TOP)     { add_cmp(N_TP, b->top_rpm, TUNE_BASE_TOP, TUNE_NG_TOP_PCT); }
    if (m & BLD_COAST)   { add_cmp(N_DE, b->dec_rpm_s, TUNE_BASE_DEC, TUNE_NG_DEC_PCT); }
    if (m & BLD_STOPT)   {
        add_cmp(N_FL, b->floor_rpm, TUNE_BASE_FLOOR, TUNE_NG_FLOOR_PCT);
        add(PK_VAL, N_ST, b->over_mm, 0, chk && (b->over_mm > BLD_OVER_OK_MM));
    }
    if (m & BLD_STEER)   {   /* 기준은 125 아래 바이트 */
        int16_t base = (int16_t)((uint16_t)tune_base(TUNE_BASE_AMP_SE) & 0xFFu);
        add(PK_VAL, N_SE, b->se_x10, 0, chk && tune_base_valid() && (base > 0) && (b->se_x10 != TUNE_NO_DATA)
                                        && ((int32_t)b->se_x10 * 100L > (int32_t)base * (100L + TUNE_NG_SE_PCT)));
    }
    add(PK_VAL, N_LR, b->lr_permille, 0, chk && (lr > TUNE_NG_LR));
    if (m & BLD_GA) {
        add(PK_VAL, N_OV, b->ov_mm, 0, 0);
        add(PK_GA, F_GA, 0, 0, 0);
    }
    add(PK_VAL, N_BA, b->bat_dv, 0, 0);
    if ((m & BLD_RPM) && chk) {      /* 앰프 형식은 점검(t2.4)만 잰다(1회). t1 기준값은 0(모름)이고 그때는 견주지 않는다 */
        int16_t base = (int16_t)((uint16_t)tune_base(TUNE_BASE_AMP_SE) >> 8);
        add(PK_VAL, N_A, b->amp, 0, tune_base_valid() && (base != 0) && (b->amp != base));
    }
    add(PK_SAVE, 0, 0, 0, 0);
}

static void bld_finish(int r)
{
    build_result_t *b = &agv_build;

    s_tune.state = TUNE_BLD_DONE;
    s_tune.page = 0;
    s_tune.editing = 0;
    if (r < 0) { return; }
    b->floor_rpm = s_tune.floor_max;
    b->bat_dv = param_get(P_BAT_DV);
    build_pages();
}

/* 채택한 값과 (t1) 기준값을 쓴다. 다시 켜야 끝난다 */
static int save_all(void)
{
    uint16_t i;

    for (i = 0; i < s_tune.n_page; i++) {
        const page_t *p = &s_tune.pg[i];
        if ((p->kind == PK_SET) && p->flag) { (void)param_set((uint16_t)p->id, p->v); }
    }
    if (s_tune.cat == 1u) { base_write(); }
    s_tune.state = TUNE_SAVED;
    return 2;
}

/* ---- 입력 ---- */

static int16_t field_get(int16_t f)
{
    switch (f) {
    case F_DIA:   return param_get(P_WHEEL_DIA_MM);
    case F_GD:    return param_get(P_GUIDE_DIST_MM);
    case F_CAP:   return s_tune.cap;
    case F_TRACK: return s_tune.track;
    case F_HS:    return s_tune.hs;
    default:      return (int16_t)(param_get(P_GUIDE_DIST_MM) - agv_build.ov_mm);   /* GA 예상값 */
    }
}

static void field_limits(int16_t f, int16_t *lo, int16_t *up)
{
    switch (f) {
    case F_DIA:   param_limit(P_WHEEL_DIA_MM, lo, up); break;
    case F_GD:    param_limit(P_GUIDE_DIST_MM, lo, up); break;
    case F_CAP:   param_limit(P_SPEED_HIGH, lo, up); break;
    case F_TRACK: *lo = 0; *up = TRACK_UP; break;
    case F_HS:    param_limit(P_SPEED_HIGH, lo, up); *up = s_tune.hs_up; break;
    default:      *lo = GAP_INPUT_LOW; *up = GAP_INPUT_UP; break;
    }
}

/* 확정 즉시 저장. 2 = 저장함 */
static int field_set(int16_t f, int16_t v)
{
    int16_t lo, up, g;

    switch (f) {
    case F_DIA: return (param_set(P_WHEEL_DIA_MM, v) == 0) ? 2 : 1;
    case F_GD:  return (param_set(P_GUIDE_DIST_MM, v) == 0) ? 2 : 1;
    case F_CAP:
        s_tune.cap = v;
        storage_request(TUNE_EE_CAP, (uint16_t)v);
        return 2;
    case F_TRACK:
        s_tune.track = v;
        storage_request(TUNE_EE_TRACK, (uint16_t)v);
        return 2;
    case F_HS:                 /* RAM 에만 */
        s_tune.hs = v;
        return 1;
    default:                   /* 잰 간격 + 감지 뒤 미끄러진 거리 = 센서와 회전 중심 사이 */
        param_limit(P_GUIDE_DIST_MM, &lo, &up);
        g = (int16_t)(v + agv_build.ov_mm);
        return (param_set(P_GUIDE_DIST_MM, (g < lo) ? lo : ((g > up) ? up : g)) == 0) ? 2 : 1;
    }
}

/* 지금 화면의 입력 칸. 없으면 -1 */
static int16_t cur_field(void)
{
    if (s_tune.state == TUNE_BLD_DONE) {
        return ((agv_build.state == 2) && (s_tune.pg[s_tune.page].kind == PK_GA)) ? F_GA : -1;
    }
    if (s_tune.state != TUNE_WAIT) { return -1; }
    if ((s_tune.cat == 1u) && (s_tune.item > 0u)) { return (int16_t)(s_tune.item - 1u); }
    if ((s_tune.cat == 2u) && (s_tune.sub > 0u))  { return F_TRACK; }
    if ((s_tune.cat == 3u) && (s_tune.sub > 0u))  { return F_HS; }
    return -1;
}

static int edit_key(uint16_t key)
{
    int16_t f = cur_field(), lo, up, step;

    switch (key) {
    case KEY_SHIFT:
        s_tune.place = (int16_t)((s_tune.place >= s_field[f].digits) ? 1 : s_tune.place + 1);
        return 1;
    case KEY_UP:
    case KEY_DOWN:
        step = (s_tune.place == 4) ? 1000 : ((s_tune.place == 3) ? 100 : ((s_tune.place == 2) ? 10 : 1));
        field_limits(f, &lo, &up);
        s_tune.edit_value = (int16_t)(s_tune.edit_value + ((key == KEY_UP) ? step : -step));
        if (s_tune.edit_value < lo) { s_tune.edit_value = lo; }
        if (s_tune.edit_value > up) { s_tune.edit_value = up; }
        return 1;
    case KEY_SET:
        s_tune.editing = 0;
        return field_set(f, s_tune.edit_value);
    case KEY_RESET:
        s_tune.editing = 0;
        return 1;
    default:
        return 1;              /* 고치는 중 MODE 는 듣지 않는다 */
    }
}

static int edit_begin(void)
{
    s_tune.edit_value = field_get(cur_field());
    s_tune.editing = 1;
    s_tune.place = 1;
    return 1;
}

/* ---- 키 ---- */

static uint16_t items(void)
{
    return (s_tune.cat == 1u) ? T1_INPUTS : ((s_tune.cat == 2u) ? CHECK_N : DEV_N);
}

static int menu_key(uint16_t key)
{
    switch (key) {
    case KEY_MODE:             /* 같은 단계 다음 칸 */
        if (s_tune.item == 0u)     { s_tune.cat = (uint16_t)(s_tune.cat % 3u + 1u); }
        else if (s_tune.sub == 0u) { s_tune.item = (uint16_t)(s_tune.item % items() + 1u); }
        return 1;
    case KEY_SET:
        if (s_tune.item == 0u) { s_tune.item = 1; return 1; }
        if (cur_field() >= 0) { return edit_begin(); }
        if ((s_tune.cat == 2u) && (s_check[s_tune.item - 1u].mask & BLD_DIA)) { s_tune.sub = 1; }
        if ((s_tune.cat == 3u) && (s_tune.item == TUNE_DEV_HS)) {
            s_tune.sub = 1;
            hs_enter();
        }
        return 1;
    case KEY_RESET:
        if (s_tune.sub > 0u)       { s_tune.sub = 0; }
        else if (s_tune.item > 0u) { s_tune.item = 0; }
        return 1;
    default:
        return 1;
    }
}

static void next_page(void)
{
    if (s_tune.page + 1u < s_tune.n_page) { s_tune.page++; }
}

static int result_key(uint16_t key)
{
    page_t *p = &s_tune.pg[s_tune.page];

    if (agv_build.state != 2) {            /* 실패: RESET 으로 메뉴 */
        if (key != KEY_RESET) { return 0; }
        s_tune.state = TUNE_WAIT;
        return 1;
    }
    switch (key) {
    case KEY_UP:   s_tune.page = (uint16_t)((s_tune.page + 1u) % s_tune.n_page); return 1;
    case KEY_DOWN: s_tune.page = (uint16_t)((s_tune.page + s_tune.n_page - 1u) % s_tune.n_page); return 1;
    case KEY_SET:
        if (p->kind == PK_SAVE) { return save_all(); }
        if (p->kind == PK_GA)   { return edit_begin(); }
        if (p->kind == PK_SET)  { p->flag = 1; }
        next_page();
        return 1;
    case KEY_RESET:
        if (p->kind == PK_SAVE) { s_tune.state = TUNE_WAIT; return 1; }   /* 저장 없이 메뉴로 */
        if (p->kind == PK_SET)  { p->flag = 0; }
        next_page();
        return 1;
    default:
        return 1;
    }
}

int tune_key(uint16_t key)
{
    uint16_t st = s_tune.state;

    if (key == 0u) { return 0; }
    if (st == TUNE_RPM) {               /* 시험 중 아무 키나 누르면 끊고 보통 정지 */
        (void)ident_rpm_key();
        return 1;
    }
    if (((st == TUNE_TEMP) || (st == TUNE_BLD)) && ident_coast_key()) { return 1; }   /* 타력·정지 중 아무 키 */
    if ((st == TUNE_BLD) && ident_rpm_key()) { return 1; }
    if (D.flag_run || D.flag_stopping) { return 0; }
    if (s_tune.editing) { return edit_key(key); }
    switch (st) {
    case TUNE_WAIT:     return menu_key(key);
    case TUNE_BLD_DONE: return result_key(key);
    case TUNE_SAVED:    return 1;      /* 전원을 다시 켠다 */
    case TUNE_RPM_DONE:
    case TUNE_TOP_DONE:
    case TUNE_ALL_DONE:
    case TUNE_HS_DONE: {
        uint16_t n = (st == TUNE_RPM_DONE) ? TUNE_RPMT_PAGES : ((st == TUNE_HS_DONE) ? TUNE_HS_PAGES : TUNE_RPM_PAGES);
        switch (key) {
        case KEY_UP:    s_tune.page = (uint16_t)((s_tune.page + 1u) % n); return 1;
        case KEY_DOWN:  s_tune.page = (uint16_t)((s_tune.page + n - 1u) % n); return 1;
        case KEY_RESET: s_tune.state = TUNE_WAIT; s_tune.page = 0; return 1;
        default:        return 0;
        }
    }
    case TUNE_IDENT_DONE:
    case TUNE_TEMP_DONE:
        if (key != KEY_RESET) { return 0; }
        s_tune.state = TUNE_WAIT;
        return 1;
    default:
        return 0;
    }
}

/* ---- 주행 ---- */

static void start(void)
{
    s_tune.editing = 0;
    s_tune.page = 0;
    if ((s_tune.cat == 1u) || ((s_tune.cat == 2u) && (s_tune.item > 0u))) {
        uint16_t m = (s_tune.cat == 1u) ? T1_MASK : s_check[s_tune.item - 1u].mask;
        clear_measure();
        ident_start_build(m, (m & BLD_DIA) ? s_tune.track : 0, s_tune.cap);
        s_tune.state = TUNE_BLD;
    }
    else if ((s_tune.cat == 3u) && (s_tune.item > 0u)) {
        s_dev[s_tune.item - 1u].start();
        s_tune.state = s_dev[s_tune.item - 1u].run;
    }
    else {
        tagact_stop();         /* 고른 시험이 없다 */
    }
}

void tune_run(void)
{
    uint16_t st = s_tune.state;

    if ((stop_phase() != STOP_PHASE_IDLE) && (stop_floor_rpm() > s_tune.floor_max)) {
        s_tune.floor_max = stop_floor_rpm();
    }
    if (++s_tune.alt >= 2u * ALT_TICKS) { s_tune.alt = 0; }
    if (st == TUNE_WAIT) {
        if (D.flag_run && !D.flag_stopping) { start(); }
    }
    else if (st == TUNE_BLD) {
        int r = ident_run();
        if (r != 0) { bld_finish(r); }
    }
    else if (dev_running(st)) {
        if (ident_run() != 0) { s_tune.state = (uint16_t)(st + 1u); }
    }
}

/* ---- 표시 ---- */

static void put(char *c, uint16_t at, int16_t v, uint16_t n)
{
    int16_t m = (v < 0) ? (int16_t)-v : v;
    while (n-- > 0u) {
        c[at + n] = (char)('0' + m % 10);
        m = (int16_t)(m / 10);
    }
}

/* 이름 두 글자 + 세 자리. 음수는 둘째 글자가 '-'. 1000 이상은 이름 한 글자 + 네 자리 */
static void show_value(char *c, const char *name, int16_t v)
{
    c[0] = name[0];
    c[1] = name[1];
    if (v == TUNE_NO_DATA) { c[2] = c[3] = c[4] = '-'; return; }
    if (v < -999) { v = -999; }
    if (v >= 1000) { put(c, 1, v, 4); return; }
    if (v < 0) { c[1] = '-'; }
    put(c, 2, v, 3);
}

static void show_field(char *c, int16_t f)
{
    int16_t v = s_tune.editing ? s_tune.edit_value : field_get(f);
    if (s_field[f].digits == 4) {
        c[0] = s_field[f].name[0];
        put(c, 1, v, 4);
    }
    else { show_value(c, s_field[f].name, v); }
}

/* 채택 쪽: 번호 두 자리 + 값 세 자리. 1 s 지금 값, 1 s 새 값(둘째 자리 점). 채택하면 첫 자리 점 */
static uint16_t show_page(char *c)
{
    const page_t *p = &s_tune.pg[s_tune.page];
    uint16_t dot = 0;

    switch (p->kind) {
    case PK_SET:
        put(c, 0, p->id, 2);
        if (p->id < 10) { c[0] = ' '; }
        if (s_tune.alt >= ALT_TICKS) { dot = 2u; put(c, 2, p->v, 3); }
        else                         { put(c, 2, p->old, 3); }
        if (p->flag) { dot |= 1u; }
        break;
    case PK_VAL:
        if (p->id == N_A) { c[0] = 'A'; c[4] = (p->v != 0) ? (char)p->v : '-'; }
        else              { show_value(c, s_name[p->id], p->v); }
        break;
    case PK_GA:
        show_field(c, F_GA);
        break;
    default:
        c[0] = 'S'; c[1] = 'A'; c[2] = 'V'; c[3] = 'E';
        break;
    }
    return dot;
}

uint16_t tune_text(char *c)
{
    uint16_t i, dot = 0;
    int16_t f;

    for (i = 0; i < 5u; i++) { c[i] = ' '; }
    switch (s_tune.state) {
    case TUNE_WAIT:
        f = cur_field();
        if (f >= 0) { show_field(c, f); break; }
        c[0] = 'T';
        c[1] = (char)('0' + s_tune.cat);
        if (s_tune.item > 0u) {        /* t2.1CH */
            const char *tag = (s_tune.cat == 2u) ? s_check[s_tune.item - 1u].tag : s_dev[s_tune.item - 1u].tag;
            dot = 2u;
            c[2] = (char)('0' + s_tune.item);
            c[3] = tag[0];
            c[4] = tag[1];
        }
        break;
    case TUNE_BLD:                     /* B·C + 단계 + 회 2자리 + 순서 */
        c[0] = (s_tune.cat == 1u) ? 'B' : 'C';
        put(c, 1, (int16_t)ident_stage(), 1);
        put(c, 2, (int16_t)ident_pass(), 2);
        put(c, 4, (int16_t)ident_phase(), 1);
        break;
    case TUNE_BLD_DONE:
        if (agv_build.state != 2) { c[0] = 'F'; put(c, 1, agv_build.fail, 4); }
        else                      { dot = show_page(c); }
        break;
    default:
        c[0] = 'R'; c[1] = 'B'; c[2] = 'O'; c[3] = 'T';
        break;
    }
    return dot;
}

uint16_t tune_flick(void)
{
    if (s_tune.editing) {          /* 고치는 자리 */
        return (uint16_t)(1u << (5 - s_tune.place));
    }
    if ((s_tune.state == TUNE_BLD_DONE) && (agv_build.state == 2)
        && (s_tune.pg[s_tune.page].kind == PK_VAL) && s_tune.pg[s_tune.page].flag) {
        return 0x03u;              /* NG: 이름이 깜박인다 */
    }
    return 0;
}
