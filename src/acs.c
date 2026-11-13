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
#define CROSS_INTERVAL_MIN    10    /* 교차로 재전송 최소·기본 0.1 s. 41번 0 이면 이 값 */
#define CROSS_INTERVAL_MAX    500
#define CROSS_KEEPALIVE       500   /* ACK 뒤 간격. 더하지 않고 교체한다 */
#define CROSS_MSGID_REISSUE   30
#define CROSS_ACK_ERROR_TRY   20
#define CROSS_TRY_WRAP        100
#define ERR_REPORT_IDLE_TICKS 500   /* 오류 5초 뒤 'X' */
#define ERR_REPORT_TICKS_PER_MIN 6000UL
#define OUT_SAVE_TICKS        100   /* OUT 이 1 s 넘게 미확인이면 EEPROM 에 둔다 */
#define OUT_EE_MARK           0x4F00u   /* 머리 상위 'O', 하위 개수 */
#define OUT_EE_EMPTY          0xFFFFu

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
    uint16_t msgid_old;     /* 재발급 전 MsgID. 늦게 온 ACK 도 받는다 */
    uint16_t ack;
    uint16_t ack_error;
    uint16_t retry;
    int16_t  timer;
} acs_cross_t;

typedef struct { uint16_t slot, type, info; } acs_map_t;

typedef struct {
    int16_t  num;
    uint16_t age;           /* 넣은 뒤 틱. OUT_SAVE_TICKS 에서 멈춘다 */
    uint32_t at;            /* 태그 때 D.travel_pulses */
} acs_out_t;

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
static acs_out_t   s_out[ACS_OUT_MAX];
static uint16_t    s_out_cnt;
static const param_store_t *s_out_store;
static uint16_t    s_out_dirty;      /* 큐가 EEPROM 기록과 다르다 */
static uint16_t    s_out_written;    /* EEPROM 에 기록이 있다 */

static uint16_t acs_enabled(void)   { return param_get(P_ACS_RETRY_LEGACY) != 0; }
uint16_t acs_pan(void)              { return s_pan; }
void     acs_set_pan(uint16_t pan)  { s_pan = pan; }
const acs_stat_t *acs_stat(void)    { return &s_stat; }
int16_t  acs_station_get(uint16_t idx)   { return (idx < ACS_STATION_MAX) ? s_station[idx] : 0; }
uint16_t acs_station_count(void)         { return s_station_cnt; }
int16_t  acs_cross_num(uint16_t which)   { return s_cross[which].num; }
uint16_t acs_cross_acked(uint16_t which) { return s_cross[which].ack; }
uint16_t acs_cross_retry(uint16_t which) { return s_cross[which].retry; }
uint16_t acs_cross_out_count(void)       { return s_out_cnt; }

void acs_reset(void)
{
    uint16_t i;
    for (i = 0; i < ACS_SLOT_COUNT; i++) { s_slot[i].info = 0; }
    for (i = 0; i < 2; i++) {
        s_cross[i].num = 0;
        s_cross[i].ack = 0;
        s_cross[i].ack_error = 0;
        s_cross[i].retry = 0;
        s_cross[i].timer = 0;
    }
    s_stat.tx = s_stat.retry = s_stat.give_up = 0;
    s_born = 0;
    s_pan = '1';
    s_jitter = 0;
    s_err_ticks = 0;
    s_err_reported = 0;
    s_out_cnt = 0;
    s_out_store = 0;
    s_out_dirty = s_out_written = 0;
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

static void out_pop(void);

void acs_on_ack(uint16_t info, uint16_t msgid)
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
    else if ((info == 'O') && (s_out_cnt > 0)) {
        /* ACK 는 구역 번호 없이 MsgID 를 되돌린다. 재전송마다 오는 중복 ACK 가 다음 OUT 을 지우지 않게 맞춰 본다 */
        acs_cross_t *c = &s_cross[ACS_CROSS_OUT];
        if ((msgid == '0') || (msgid == c->msgid) || (msgid == c->msgid_old)) { out_pop(); }
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
    c->bat = param_get(P_BAT_DV);
    c->msgid = frame_msgid_next();
    c->msgid_old = c->msgid;
}

static int16_t s_in_keep;   /* 오류로 선 자리의 IN: 오류가 서 있는 동안 acs_cross_clear 가 지우지 않는다 */

void acs_cross_release(uint16_t which)
{
    if (which == ACS_CROSS_IN) {
        cross_clear(&s_cross[ACS_CROSS_IN]);
        D.cross_in_flag = D.cross_in_on = 0;
        s_in_keep = 0;
    }
    else {
        D.cross_out_flag = D.cross_out_on = 0;     /* 보고 큐는 ACK 까지 남는다. 2 m 를 넘었으면 stop.c 가 다시 건다 */
    }
}

void acs_cross_clear(void)
{
    if (!s_in_keep || ((D.stop_err == 0) && (D.system_err == 0))) { acs_cross_release(ACS_CROSS_IN); }
    acs_cross_release(ACS_CROSS_OUT);
}

/* 허가 없이 갈 수 있는 거리를 건다. 0 cm 는 태그에서 바로 선다.
 * shortcut: In·Out 이 함께 기다리면 먼저 건 (가까운) 목표 하나만 센다. 둘을 따로 세야 하면 카운터를 나눈다 */
static void cross_arm(int16_t *flag, int16_t *on)
{
    float n = (float)param_get(P_CROSS_IN_CM) * 10.0f * param_dv.one_length_pulse + 0.5f;

    if (n < 1.0f) {
        *on = 1;
        return;
    }
    if (!D.cross_in_flag && !D.cross_out_flag) { D.cross_pulses = (n > 32767.0f) ? (int16_t)32767 : (int16_t)n; }
    *flag = 1;
}

void acs_cross_in(int16_t id)
{
    if (!acs_enabled()) { return; }
    cross_begin(&s_cross[ACS_CROSS_IN], id);
    cross_arm(&D.cross_in_flag, &D.cross_in_on);
    if (seq_is_active()) { D.cross_in_on = 1; }
    /* 앞 구역 OUT 이 아직 미확인이면 같은 자리에서 OUT 확인도 기다린다 */
    if ((s_out_cnt > 0) && !D.cross_out_on) {
        D.cross_out_flag = D.cross_in_flag;
        D.cross_out_on = D.cross_in_on;
    }
}

/* 시퀀스 이동·교차선 찾기 중 IN 태그(서고 오류로 끝난다): IN 보고·허가 대기를 오류 동안 남긴다. 리셋 키로 오류를 풀고 START 하면
 * 교차로 대기 거리에서 허가를 기다린다. 사람의 STOP·ACS 취소처럼 오류가 없는 때의 정리는 지금처럼 지운다 */
void acs_cross_in_keep(int16_t id)
{
    acs_cross_in(id);
    s_in_keep = (int16_t)(s_cross[ACS_CROSS_IN].num != 0);
}

/* ---- OUT 큐 ---- */

/* 기록은 1 s 넘게 미확인일 때만, 지우기는 기록이 있을 때만 한다(EEPROM 수명).
 * 저장 큐는 낮은 주소부터 쓰므로 번호가 먼저, 머리가 나중에 써진다 */
static void out_save(void)
{
    uint16_t i;
    if (s_out_store == 0) { return; }
    for (i = 0; i < s_out_cnt; i++) { s_out_store->write((uint16_t)(ACS_OUT_EE_FIRST + i), (uint16_t)s_out[i].num); }
    s_out_store->write(ACS_OUT_EE_HEAD, (uint16_t)(OUT_EE_MARK | s_out_cnt));
    s_out_written = 1;
    s_out_dirty = 0;
}

static void out_push(int16_t num, uint16_t age)
{
    uint16_t i;
    for (i = 0; i < s_out_cnt; i++) {
        if (s_out[i].num == num) { return; }
    }
    if (s_out_cnt >= ACS_OUT_MAX) {
        /* 오래된 쪽을 남긴다. 먼저 들어간 구역일수록 오래 막고 있고, 보내는 순서도 지킨다 */
        log_event(LOG_CROSS_OUT_FULL, num, (int16_t)s_out_cnt);
        return;
    }
    s_out[s_out_cnt].num = num;
    s_out[s_out_cnt].age = age;
    s_out[s_out_cnt].at = D.travel_pulses;
    s_out_cnt++;
    s_out_dirty = 1;
    if (s_out_cnt == 1u) { cross_begin(&s_cross[ACS_CROSS_OUT], num); }
}

static void out_pop(void)
{
    uint16_t i;
    for (i = 1; i < s_out_cnt; i++) { s_out[i - 1u] = s_out[i]; }
    s_out_cnt--;
    s_out_dirty = 1;
    if (s_out_cnt > 0) {
        cross_begin(&s_cross[ACS_CROSS_OUT], s_out[0].num);     /* 다음 OUT 을 곧바로 보낸다 */
        return;
    }
    cross_clear(&s_cross[ACS_CROSS_OUT]);
    acs_cross_release(ACS_CROSS_OUT);
    if (s_out_written && (s_out_store != 0)) { s_out_store->write(ACS_OUT_EE_HEAD, OUT_EE_EMPTY); }
    s_out_written = s_out_dirty = 0;
}

static void out_tick(void)
{
    uint16_t i;
    for (i = 0; i < s_out_cnt; i++) {
        if (s_out[i].age < OUT_SAVE_TICKS) { s_out[i].age++; }
    }
    /* 맨 앞이 가장 오래됐다 */
    if (s_out_dirty && (s_out_cnt > 0) && (s_out[0].age >= OUT_SAVE_TICKS)) { out_save(); }
}

void acs_cross_out(int16_t id)
{
    if (!acs_enabled()) { return; }
    out_push((int16_t)(id % 100), 0);
}

int16_t acs_cross_out_due(void)
{
    float n = (float)CROSS_OUT_MAX_MM * param_dv.one_length_pulse;
    if (!acs_enabled() || (s_out_cnt == 0) || (n <= 0.0f)) { return 0; }
    return (float)(int32_t)(D.travel_pulses - s_out[0].at) >= n;
}

void acs_out_boot(const param_store_t *store)
{
    uint16_t i, n, head;

    s_out_store = store;
    if (store == 0) { return; }
    head = store->read(ACS_OUT_EE_HEAD);
    n = (uint16_t)(head & 0xFFu);
    if (((head & 0xFF00u) != OUT_EE_MARK) || (n == 0) || (n > ACS_OUT_MAX)) { return; }
    s_out_written = 1;
    for (i = 0; i < n; i++) {
        uint16_t v = store->read((uint16_t)(ACS_OUT_EE_FIRST + i));
        if (v < 100u) { out_push((int16_t)v, OUT_SAVE_TICKS); }
    }
    s_out_dirty = (uint16_t)(s_out_cnt != n);    /* 깨진 칸을 뺐으면 다시 쓴다 */
    if (s_out_cnt == 0) {
        store->write(ACS_OUT_EE_HEAD, OUT_EE_EMPTY);
        s_out_written = 0;
    }
}

void acs_cross_hold(void)
{
    if (D.cross_in_flag)  { D.cross_in_on = 1; D.cross_in_flag = 0; }
    if (D.cross_out_flag) { D.cross_out_on = 1; D.cross_out_flag = 0; }
}

static int32_t cross_interval(void)
{
    int16_t v = param_get(P_CROSS_RETRY_100MS);
    int32_t iv = (int32_t)v * 10L;      /* 0 = 기본 0.1 s */
    if (iv < CROSS_INTERVAL_MIN) { iv = CROSS_INTERVAL_MIN; }
    if (iv > CROSS_INTERVAL_MAX) { iv = CROSS_INTERVAL_MAX; }
    return iv;
}

static int cross_send(uint16_t which)
{
    uint16_t buf[FRAME_LEN_LONG];
    acs_cross_t *c = &s_cross[which];
    uint16_t next = (uint16_t)(c->retry + 1);

    if (((next % CROSS_MSGID_REISSUE) == 0) || c->ack) {
        c->msgid_old = c->msgid;
        c->msgid = frame_msgid_next();
    }
    frame_build(buf, 'E', s_pan, c->msgid, (which == ACS_CROSS_IN) ? 'I' : 'O', c->num, c->bat);
    if (!link_send(buf, FRAME_LEN_LONG)) { return 0; }
    c->retry = next;
    if (c->retry >= CROSS_ACK_ERROR_TRY) { c->ack_error = 1; }
    if (c->retry >= CROSS_TRY_WRAP)      { c->retry = 0; }
    return 1;
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

    /* 교차로 보고는 ACS 사용(36번 WRT)만 따른다. 42번(LUS)은 무시한다 */
    sent = cross_tick(ACS_CROSS_IN, (int16_t)(D.cross_in_on || D.cross_in_flag), may_send);
    /* OUT 은 모드·초기화와 무관하게 ACK 까지 보낸다. 놓치면 구역이 영영 막힌다 */
    sent |= cross_tick(ACS_CROSS_OUT, (int16_t)(s_out_cnt > 0), (uint16_t)!sent);
    D.cross_ack_error = (int16_t)(s_cross[ACS_CROSS_IN].ack_error || s_cross[ACS_CROSS_OUT].ack_error);
    out_tick();
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
