/* acs.c - 슬롯 스케줄러, 교차로 상태기계, 스테이션 큐 */
#include "agv/acs.h"
#include "agv/drive.h"
#include "agv/frame.h"
#include "agv/log.h"
#include "agv/param.h"
#include "agv/seq.h"

#define D agv_drive

#define ACS_RETRY_REISSUE     10    /* 재전송 몇 회마다 MsgID 재발급 */
#define ACS_JITTER_MIN_TICKS  3
#define CROSS_INTERVAL_ZERO   50    /* 설정값이 0일 때만 쓴다. CT=1 은 10틱(0.1 s)이 의도다 */
#define CROSS_INTERVAL_MAX    500
#define CROSS_KEEPALIVE       500   /* ACK 뒤 간격. 더하지 않고 교체한다 */
#define CROSS_MSGID_REISSUE   30
#define CROSS_ACK_ERROR_TRY   20
#define CROSS_TRY_WRAP        100
#define ERR_REPORT_IDLE_TICKS 500   /* 오류 5초 뒤 'X' */
#define ERR_REPORT_TICKS_PER_MIN 6000UL

typedef struct {
    uint16_t info;          /* 0 = 비어 있음 */
    int16_t  num;
    int16_t  bat;
    uint16_t msgid;
    uint16_t try_cnt;
    uint16_t try_total;
    uint16_t born;          /* 생성 순번. 가장 오래된 슬롯을 고른다 */
    int32_t  timer;         /* 다음 송신까지 틱 */
} acs_slot_t;

typedef struct {
    int16_t  num;
    int16_t  bat;
    uint16_t msgid;
    uint16_t ack;
    uint16_t ack_error;
    uint16_t retry;
    uint16_t delay;         /* Flag 뒤 On 까지 지연 카운터 (Cnt_Flag_Cross_*) */
    int16_t  timer;
} acs_cross_t;

typedef struct { uint16_t slot, type, info; } acs_map_t;

/* 위에서 아래로 Info 를 찾으므로 'C' 는 호출 슬롯이 먼저다 */
static const acs_map_t s_map[] = {
    { ACS_SLOT_CALL, 'L', 'C' },
    { ACS_SLOT_CALL_ACK, 'A', 'N' },
    { ACS_SLOT_LOADING, 'E', 'L' }, { ACS_SLOT_UNLOADING, 'E', 'U' }, { ACS_SLOT_NEXT, 'E', 'N' },
    { ACS_SLOT_ERR, 'R', 'E' }, { ACS_SLOT_STANDBY, 'E', 'B' }, { ACS_SLOT_STANDBY_CLEAR, 'E', 'C' },
    { ACS_SLOT_START, 'R', 'R' }, { ACS_SLOT_ARRIVE, 'E', 'A' }, { ACS_SLOT_ERR_CLEAR, 'R', 'C' },
    { ACS_SLOT_STOP, 'R', 'S' }, { ACS_SLOT_DISPLAY, 'E', 'D' }, { ACS_SLOT_FINISH, 'E', 'F' },
    { ACS_SLOT_PROCESSING, 'E', 'P' }, { ACS_SLOT_IDLE_X, 'E', 'X' }
};
#define MAP_COUNT (sizeof(s_map) / sizeof(s_map[0]))

static acs_slot_t  s_slot[ACS_SLOT_COUNT];
static acs_cross_t s_cross[2];
static acs_stat_t  s_stat;
static uint16_t    s_born;
static uint16_t    s_pan = '1';
static uint32_t    s_jitter;
static int16_t     s_station[ACS_STATION_MAX];
static uint16_t    s_station_cnt;
static uint32_t    s_err_ticks;      /* DA_EST_cnt */
static uint16_t    s_err_reported;   /* AGV_Error_Status_Flag */

static uint16_t acs_enabled(void)   { return param_get(P_ACS_RETRY_LEGACY) != 0; }
uint16_t acs_pan(void)              { return s_pan; }
void     acs_set_pan(uint16_t pan)  { s_pan = pan; }
const acs_stat_t *acs_stat(void)    { return &s_stat; }
int16_t  acs_station_get(uint16_t idx)   { return (idx < ACS_STATION_MAX) ? s_station[idx] : 0; }
uint16_t acs_station_count(void)         { return s_station_cnt; }
int16_t  acs_cross_num(uint16_t which)   { return s_cross[which].num; }
uint16_t acs_cross_acked(uint16_t which) { return s_cross[which].ack; }
uint16_t acs_cross_retry(uint16_t which) { return s_cross[which].retry; }

void acs_reset(void)
{
    uint16_t i;
    for (i = 0; i < ACS_SLOT_COUNT; i++) { s_slot[i].info = 0; }
    for (i = 0; i < 2; i++) {
        s_cross[i].num = 0;
        s_cross[i].ack = 0;
        s_cross[i].ack_error = 0;
        s_cross[i].retry = 0;
        s_cross[i].delay = 0;
        s_cross[i].timer = 0;
    }
    s_stat.tx = s_stat.retry = s_stat.give_up = 0;
    s_born = 0;
    s_pan = '1';
    s_jitter = 0;
    s_err_ticks = 0;
    s_err_reported = 0;
    acs_station_reset();
}

/* ---- 지터 ---- */

static uint32_t avalanche32(uint32_t x)
{
    x ^= x >> 16;
    x *= 0x7FEB352DUL;
    x ^= x >> 15;
    x *= 0x846CA68BUL;
    x ^= x >> 16;
    return x;
}

static uint32_t jitter_next(void)
{
    uint32_t x = s_jitter;
    if (x == 0) {
        /* 동시 출발한 차들이 같은 수열을 타지 않도록 번호와 PAN 으로 시드를 만든다 */
        x = avalanche32(((uint32_t)(uint16_t)(s_pan - '0') << 8) | (uint32_t)((uint16_t)param_get(P_AGV_ID) & 0xFF));
        if (x == 0) { x = 1; }
    }
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_jitter = x;
    return x;
}

static int32_t jitter_span(int32_t base)
{
    int32_t width = base / 10;
    if (base <= 0) { return 0; }
    if (width < ACS_JITTER_MIN_TICKS) { width = ACS_JITTER_MIN_TICKS; }
    return (int32_t)(jitter_next() % (uint32_t)width);
}

/* ---- 슬롯 ---- */

static const acs_map_t *map_by_slot(uint16_t slot)
{
    uint16_t i;
    for (i = 0; i < MAP_COUNT; i++) {
        if (s_map[i].slot == slot) { return &s_map[i]; }
    }
    return 0;
}

static void slot_make(uint16_t slot, uint16_t info, int16_t num, uint16_t msgid)
{
    acs_slot_t *s = &s_slot[slot];
    s->info = info;
    s->num = num;
    s->bat = param_get(P_BAT_DV);
    s->msgid = msgid;
    s->try_cnt = 0;
    s->try_total = 0;
    s->timer = 0;
    s->born = s_born++;
}

void acs_report_slot(uint16_t slot, int16_t num)
{
    const acs_map_t *m = map_by_slot(slot);
    if ((m == 0) || !acs_enabled()) { return; }
    if ((m->info == 'D') && (num <= 0)) { return; }
    slot_make(slot, m->info, num, frame_msgid_next());
}

void acs_report(uint16_t info, int16_t num)
{
    uint16_t i;
    for (i = 0; i < MAP_COUNT; i++) {
        if ((s_map[i].info == info) && (s_map[i].slot != ACS_SLOT_CALL_ACK)) {
            acs_report_slot(s_map[i].slot, num);
            return;
        }
    }
}

void acs_call_ack(uint16_t rx_msgid)
{
    if (acs_enabled()) { slot_make(ACS_SLOT_CALL_ACK, 'N', 0, rx_msgid); }
}

void acs_slot_clear(uint16_t slot)
{
    s_slot[slot].info = 0;
    s_slot[slot].try_cnt = 0;
    s_slot[slot].try_total = 0;
}

int acs_send_ack(uint16_t rx_msgid, uint16_t c6, uint16_t c7, uint16_t c9)
{
    uint16_t buf[FRAME_LEN_LONG];
    if (!acs_enabled()) { return 0; }
    frame_build_ack(buf, s_pan, rx_msgid, c6, c7, c9);
    return link_send(buf, FRAME_LEN_LONG);
}

int acs_send_tag_ack(uint16_t rx_msgid, uint16_t c6, uint16_t c7, uint16_t c9)
{
    uint16_t buf[FRAME_LEN_LONG];
    frame_build_ack(buf, s_pan, rx_msgid, c6, c7, c9);
    return link_send(buf, FRAME_LEN_LONG);
}

uint16_t acs_pending(void)
{
    uint16_t i, n = 0;
    for (i = 0; i < ACS_SLOT_COUNT; i++) {
        if (s_slot[i].info != 0) { n++; }
    }
    return n;
}

static int16_t older(uint16_t a, uint16_t b)
{
    return (int16_t)(a - b) < 0;
}

void acs_on_ack(uint16_t info)
{
    uint16_t i;
    int16_t best = -1;

    for (i = 0; i < ACS_SLOT_COUNT; i++) {
        uint16_t match = (s_slot[i].info == info) || ((i == ACS_SLOT_CALL) && (info == 'L') && (s_slot[i].info != 0));
        if (match && ((best < 0) || older(s_slot[i].born, s_slot[best].born))) {
            best = (int16_t)i;
        }
    }
    if (best >= 0) { acs_slot_clear((uint16_t)best); }

    if (info == 'I') {
        s_cross[ACS_CROSS_IN].ack = 1;
        s_cross[ACS_CROSS_IN].ack_error = 0;
        s_cross[ACS_CROSS_IN].retry = 0;
        s_cross[ACS_CROSS_IN].timer = CROSS_KEEPALIVE;
    }
    else if (info == 'O') {
        s_cross[ACS_CROSS_OUT].num = 0;
        s_cross[ACS_CROSS_OUT].ack_error = 0;
        s_cross[ACS_CROSS_OUT].retry = 0;
        s_cross[ACS_CROSS_OUT].delay = 0;
        s_cross[ACS_CROSS_OUT].timer = CROSS_KEEPALIVE;
        D.cross_out_flag = 0;
        D.cross_out_on = 0;
    }
}

static int32_t retry_interval(void)
{
    int16_t sec = param_get(P_ACS_RETRY_SEC);
    return (sec > 0) ? (int32_t)sec * 100L : (int32_t)param_get(P_ACS_RETRY_LEGACY) * 10L;
}

static int slot_send(uint16_t slot)
{
    uint16_t buf[FRAME_LEN_LONG];
    acs_slot_t *s = &s_slot[slot];

    if (slot == ACS_SLOT_CALL_ACK) {
        frame_build_ack(buf, s_pan, s->msgid, 'N', '0', 'P');
    }
    else {
        frame_build(buf, map_by_slot(slot)->type, s_pan, s->msgid, s->info, s->num, s->bat);
    }
    return link_send(buf, FRAME_LEN_LONG);
}

/* 만료된 슬롯 중 가장 오래된 것 하나를 보낸다. 다른 슬롯을 기다리게 하지 않는다 */
static void slots_tick(uint16_t may_send)
{
    uint16_t i;
    int16_t cand = -1;

    for (i = 0; i < ACS_SLOT_COUNT; i++) {
        acs_slot_t *s = &s_slot[i];
        if (s->info == 0) { continue; }
        if (s->timer > 0) { s->timer--; }
        if ((s->timer <= 0) && ((cand < 0) || older(s->born, s_slot[cand].born))) {
            cand = (int16_t)i;
        }
    }
    if ((cand >= 0) && may_send && slot_send((uint16_t)cand)) {
        acs_slot_t *s = &s_slot[cand];
        int32_t iv;

        s_stat.tx++;
        if (s->try_cnt > 0) { s_stat.retry++; }
        if (cand == ACS_SLOT_CALL_ACK) {       /* 응답은 재전송하지 않는다 */
            acs_slot_clear(ACS_SLOT_CALL_ACK);
            return;
        }
        s->try_cnt++;
        s->try_total++;
        if ((s->try_cnt % ACS_RETRY_REISSUE) == 0) { s->msgid = frame_msgid_next(); }
        iv = retry_interval();
        s->timer = iv + jitter_span(iv);
        if (s->try_total >= ACS_MAX_TRY) {
            s_stat.give_up++;
            log_event(LOG_ACS_GIVE_UP, (int16_t)s->info, s->num);
            acs_slot_clear((uint16_t)cand);
        }
    }
}

/* ---- 교차로 ---- */

static void cross_clear(acs_cross_t *c)
{
    c->num = 0;
    c->ack = 0;
    c->timer = 0;
    c->retry = 0;
    c->ack_error = 0;
}

static void cross_begin(acs_cross_t *c, int16_t id)
{
    c->num = (int16_t)(id % 100);
    c->ack = 0;
    c->timer = 0;
    c->ack_error = 0;
    c->retry = 0;
    c->delay = 0;
    c->bat = param_get(P_BAT_DV);
    c->msgid = frame_msgid_next();
}

void acs_cross_release(uint16_t which)
{
    cross_clear(&s_cross[which]);
    s_cross[which].delay = 0;
    if (which == ACS_CROSS_IN) { D.cross_in_flag = D.cross_in_on = 0; }
    else                       { D.cross_out_flag = D.cross_out_on = 0; }
}

void acs_cross_clear(void)
{
    acs_cross_release(ACS_CROSS_IN);
    acs_cross_release(ACS_CROSS_OUT);
}

void acs_cross_in(int16_t id)
{
    if (!acs_enabled()) { return; }
    cross_begin(&s_cross[ACS_CROSS_IN], id);
    D.cross_in_flag = 1;
    if (seq_is_active()) { D.cross_in_on = 1; }
}

void acs_cross_out(int16_t id)
{
    if (!acs_enabled()) { return; }
    cross_begin(&s_cross[ACS_CROSS_OUT], id);
    D.cross_out_flag = 1;
}

static int32_t cross_interval(void)
{
    int16_t v = param_get(P_CROSS_RETRY_100MS);
    int32_t iv = (int32_t)((v > 0) ? v : param_get(P_ACS_RETRY_SEC)) * 10L;
    if (iv == 0) { iv = CROSS_INTERVAL_ZERO; }
    if (iv > CROSS_INTERVAL_MAX) { iv = CROSS_INTERVAL_MAX; }
    return iv;
}

static int cross_send(uint16_t which)
{
    uint16_t buf[FRAME_LEN_LONG];
    acs_cross_t *c = &s_cross[which];
    uint16_t next = (uint16_t)(c->retry + 1);

    if (((next % CROSS_MSGID_REISSUE) == 0) || c->ack) { c->msgid = frame_msgid_next(); }
    frame_build(buf, 'E', s_pan, c->msgid, (which == ACS_CROSS_IN) ? 'I' : 'O', c->num, c->bat);
    if (!link_send(buf, FRAME_LEN_LONG)) { return 0; }
    c->retry = next;
    if (c->retry >= CROSS_ACK_ERROR_TRY) { c->ack_error = 1; }
    if (c->retry >= CROSS_TRY_WRAP)      { c->retry = 0; }
    return 1;
}

/* Flag 뒤 P_CROSS_IN_TIME × 100 ms 안에 허가가 없으면 정지로 넘어간다 */
static void cross_delay(acs_cross_t *c, int16_t *flag, int16_t *on)
{
    if (*flag) {
        c->delay++;
        if (c->delay > (uint16_t)param_get(P_CROSS_IN_TIME) * 10u) {
            *on = 1;
            *flag = 0;
            c->delay = 0;
        }
    }
}

/* 반환: 이번 호출에 프레임을 보냈으면 1 */
static int cross_tick(uint16_t which, int16_t active, uint16_t may_send)
{
    acs_cross_t *c = &s_cross[which];
    int sent = 0;

    if (!active) {
        cross_clear(c);
        return 0;
    }
    /* 간격 변수는 방향마다 따로 둔다. 한쪽의 keep-alive 가 다른 쪽 간격을 덮지 않게 한다 */
    if (c->timer > 0) { c->timer--; }
    if ((c->timer <= 0) && may_send && cross_send(which)) {
        int32_t iv = c->ack ? (int32_t)CROSS_KEEPALIVE : cross_interval();
        c->timer = (int16_t)(iv + jitter_span(iv));
        sent = 1;
    }
    return sent;
}

/* ---- 스케줄러 ---- */

void acs_schedule(void)
{
    uint16_t may_send;
    int sent = 0;

    if (!acs_enabled()) { return; }
    /* 수동·파라미터 모드에서는 보내지 않고 슬롯도 소모하지 않는다 */
    may_send = (D.ui_mode == UI_AUTO);

    cross_delay(&s_cross[ACS_CROSS_IN], &D.cross_in_flag, &D.cross_in_on);
    cross_delay(&s_cross[ACS_CROSS_OUT], &D.cross_out_flag, &D.cross_out_on);
    if (param_dv.collision_type == 1) {
        sent  = cross_tick(ACS_CROSS_IN,  (int16_t)(D.cross_in_on || D.cross_in_flag), may_send);
        sent |= cross_tick(ACS_CROSS_OUT, (int16_t)(D.cross_out_on || D.cross_out_flag), (uint16_t)(may_send && !sent));
        D.cross_ack_error = (int16_t)(s_cross[ACS_CROSS_IN].ack_error || s_cross[ACS_CROSS_OUT].ack_error);
    }
    slots_tick((uint16_t)(may_send && !sent));
}

/* ---- 오류 보고 ---- */

void acs_error_tick(void)
{
    int16_t code;
    uint16_t any = (D.system_err != 0) || (D.stop_err != 0) || (D.action_err != 0);

    if (s_err_reported) {
        if (!any) {
            s_err_reported = 0;
            acs_report_slot(ACS_SLOT_ERR_CLEAR, 0);
        }
        return;
    }
    if (!any) {
        s_err_ticks = 0;      /* 해소되면 지운다. 지우지 않으면 다음 오류에서 'X' 가 일찍 나간다 */
        return;
    }
    code = (D.system_err != 0) ? D.system_err : ((D.stop_err != 0) ? D.stop_err : D.action_err);
    s_err_ticks++;
    if (s_err_ticks == ERR_REPORT_IDLE_TICKS) { acs_report_slot(ACS_SLOT_IDLE_X, code); }
    /* 16비트 곱으로 넘치면 보고가 영구히 안 나간다 */
    if (s_err_ticks > (uint32_t)param_get(P_ERR_REPORT_MIN) * ERR_REPORT_TICKS_PER_MIN) {
        acs_report_slot(ACS_SLOT_ERR, code);
        s_err_ticks = 0;
        s_err_reported = 1;
    }
}

/* ---- 스테이션 큐 ---- */

int acs_station_push(int16_t n)
{
    uint16_t i;
    if ((n == 0) || (s_station_cnt >= ACS_STATION_MAX)) { return 0; }
    for (i = 0; i < s_station_cnt; i++) {
        if (s_station[i] == n) { return 0; }
    }
    s_station[s_station_cnt++] = n;
    return 1;
}

void acs_station_reset(void)
{
    uint16_t i;
    for (i = 0; i < ACS_STATION_MAX; i++) { s_station[i] = 0; }
    s_station_cnt = 0;
}

void acs_station_set(uint16_t idx, int16_t value, uint16_t count)
{
    if (idx < ACS_STATION_MAX) { s_station[idx] = value; }
    if (count <= ACS_STATION_MAX) { s_station_cnt = count; }
}
